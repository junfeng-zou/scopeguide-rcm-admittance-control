classdef ZJFDobotCR5 < handle
    % CRRobot - 一个用于通过 TCP/IP 控制 CR 系列协作机器人的 MATLAB 类。
    %
    % 该类封装了与机器人控制器进行通信的细节，允许用户通过调用方法
    % 来发送指令，并通过读取属性来获取机器人的实时状态，无需图形界面。

    % --- 公共属性（用户可配置） ---
    properties
        IPAddress = '192.168.5.1';  % 机器人的 IP 地址
        DashboardPort = 29999;      % Dashboard 端口号
        MovePort = 30003;           % 运动控制端口号
        FeedbackPort = 30004;       % 实时反馈端口号
    end

    % --- 公共属性（只读状态） ---
    properties (SetAccess = private)
        IsConnected = false;            % 连接状态
        RobotMode = "Disconnected";     % 机器人模式
        CurrentSpeedRatio = 0;          % 当前速度比例
        JointAngles = zeros(1, 6);      % 实时关节角度 [J1, J2, J3, J4, J5, J6]
        CartesianPose = zeros(1, 6);    % 实时笛卡尔坐标 [X, Y, Z, Rx, Ry, Rz]
        DigitalInputs = zeros(1, 8);    % 数字输入状态
        DigitalOutputs = zeros(1, 8);   % 数字输出状态
        ActualJointSpeeds = zeros(1, 6);    % 实际关节速度 (QDActual)
        ActualJointCurrents = zeros(1, 6);  % 实际关节电流 (IActual)
        ActualTCPSpeed = zeros(1, 6);       % 实际末端速度 (TCPSpeedActual)
        CalculatedTCPForce = zeros(1, 6);   % TCP力值 (TCPForce)
        JointModes = zeros(1, 6);           % 关节控制模式 (JointModes)
        ActualQuaternion = [NaN, NaN, NaN, NaN]; % [qw,qx,qy,qz]
        ActiveUserIndex = uint8(0);          % V3 feedback User byte
        ActiveToolIndex = uint8(0);          % V3 feedback Tool byte
        RunQueuedCommand = false;            % V3 RunQueuedCmd flag
        PauseCommandFlag = false;            % V3 PauseCmdFlag
        FeedbackSequence = uint64(0);        % validated 30004 frame count
        LastFeedbackMonotonicSec = NaN;      % host time since Connect()
        InvalidFeedbackByteCount = uint64(0); % resynchronization drops
        LastDashboardResponse = '';     % Dashboard 端口的最后响应
        LastMoveResponse = '';          % Move 端口的最后响应
        TargetVelocity = zeros(1,6);
    end

    % --- 私有属性 ---
    properties (Access = private)
        DashboardClient                 % Dashboard TCP 客户端
        MoveClient                      % 运动控制 TCP 客户端
        FeedBackClient                  % 实时反馈 TCP 客户端
        StateDataArray                  % 存储反馈数据的原始字节数组
        FeedbackClockStart              % monotonic clock for this connection
        FeedbackByteBuffer = zeros(1, 0, 'uint8')
    end

    % --- 构造函数与析构函数 ---
    methods
        function self = ZJFDobotCR5(ip, dash_port, move_port, feedback_port)
            % 构造函数: 创建一个 CRRobot 对象
            % 可选参数: ip, dash_port, move_port, feedback_port
            if nargin > 0, self.IPAddress = ip; end
            if nargin > 1, self.DashboardPort = dash_port; end
            if nargin > 2, self.MovePort = move_port; end
            if nargin > 3, self.FeedbackPort = feedback_port; end
        end
        function delete(self)
            % 析构函数: 当对象被销毁时，自动断开连接
            if self.IsConnected
                fprintf('DobotCR5 object is being deleted. Disconnecting...\n');
                self.Disconnect();
            end
        end
    end

    % --- 公共方法：连接与控制 ---
    methods
        function Connect(self)
            % 连接到机器人
            if self.IsConnected
                disp('Already connected.');
                return;
            end
            try
                fprintf('Connecting to robot at %s...\n', self.IPAddress);
                % 设置一个合理的超时时间
                self.DashboardClient = tcpclient(self.IPAddress, self.DashboardPort, 'Timeout', 5);
                self.MoveClient = tcpclient(self.IPAddress, self.MovePort, 'Timeout', 5);
                self.FeedBackClient = tcpclient(self.IPAddress, self.FeedbackPort, 'Timeout', 5);
                self.FeedbackSequence = uint64(0);
                self.LastFeedbackMonotonicSec = NaN;
                self.InvalidFeedbackByteCount = uint64(0);
                self.FeedbackByteBuffer = zeros(1, 0, 'uint8');
                self.ActualQuaternion = [NaN, NaN, NaN, NaN];
                self.FeedbackClockStart = tic;

                % 配置实时数据反馈回调
                configureCallback(self.FeedBackClient, "byte", 1440, @self.FeedBackTcpCallback);

                self.IsConnected = true;
                disp('Successfully connected to the robot.');
            catch ME
                self.IsConnected = false;
                error('Failed to connect to the robot: %s', ME.message);
            end
        end
        function Disconnect(self)
            % 断开与机器人的连接
            if ~self.IsConnected
                disp('Already disconnected.');
                return;
            end
            try
                configureCallback(self.FeedBackClient, "off");
                clear self.DashboardClient;
                clear self.MoveClient;
                clear self.FeedBackClient;
                self.IsConnected = false;
                self.RobotMode = "Disconnected";
                self.FeedbackByteBuffer = zeros(1, 0, 'uint8');
                disp('Disconnected from the robot.');
            catch ME
                error('An error occurred during disconnection: %s', ME.message);
            end
        end
        function response = Enable(self,load)
            % 使能机器人
            cmd = sprintf("EnableRobot(%f)",load);
            response = self.sendDashboardCommand(cmd);
        end
        function response = Disable(self)
            % 下使能机器人
            response = self.sendDashboardCommand("DisableRobot()");
        end
        function response = Reset(self)
            % 复位机器人，清空指令队列
            response = self.sendDashboardCommand("ResetRobot()");
        end
        function response = ClearError(self)
            % 清除错误报警
            response = self.sendDashboardCommand("ClearError()");
        end
        function response = SetSpeedRatio(self, ratio)
            % 设置全局速度比例
            % ratio: 速度百分比 (1-100)
            cmd = sprintf("SpeedFactor(%d)", ratio);
            response = self.sendDashboardCommand(cmd);
        end
        function response = SetDO(self, index, status)
            % 设置数字输出端口状态（立即执行）
            % index: 端口号 (e.g., 1)
            % status: 状态 (1 for ON, 0 for OFF)
            cmd = sprintf("DOExecute(%d,%d)", index, status);
            response = self.sendDashboardCommand(cmd);
        end
        function errorInfo = GetErrorID(self)
            % 获取当前错误 ID
            errorInfo = self.sendDashboardCommand("GetErrorID()");
        end
        function snapshot = GetStateSnapshot(self)
            % Return one value-only snapshot for ScopeGuide consumers.
            % This adds feedback observability only. It does not alter the
            % original Dashboard or Move command transport behavior.
            if ~self.IsConnected || self.FeedbackSequence == 0 || ...
                    isempty(self.FeedbackClockStart) || ...
                    ~isfinite(self.LastFeedbackMonotonicSec)
                error('ZJFDobotCR5:NoFeedback', ...
                    'No validated 30004 feedback frame is available.');
            end
            snapshot = struct();
            snapshot.feedbackSequence = self.FeedbackSequence;
            snapshot.hostMonotonicSec = self.LastFeedbackMonotonicSec;
            snapshot.feedbackAgeSec = toc(self.FeedbackClockStart) - ...
                self.LastFeedbackMonotonicSec;
            snapshot.invalidFeedbackByteCount = ...
                self.InvalidFeedbackByteCount;
            snapshot.robotMode = self.RobotMode;
            snapshot.currentSpeedRatio = self.CurrentSpeedRatio;
            snapshot.jointAnglesDeg = self.JointAngles;
            snapshot.cartesianPose = self.CartesianPose;
            snapshot.actualJointSpeedsDegSec = self.ActualJointSpeeds;
            snapshot.actualTCPSpeed = self.ActualTCPSpeed;
            snapshot.actualQuaternionWxyz = self.ActualQuaternion;
            snapshot.activeUserIndex = self.ActiveUserIndex;
            snapshot.activeToolIndex = self.ActiveToolIndex;
            snapshot.runQueuedCommand = self.RunQueuedCommand;
            snapshot.pauseCommandFlag = self.PauseCommandFlag;
        end
    end

    % --- 公共方法：运动指令 ---
    methods
        function response = MovJ(self, pose)
            % 关节运动到指定的笛卡尔坐标点
            % pose: 1x6 数组 [X, Y, Z, Rx, Ry, Rz]
            cmd = sprintf("MovJ(%f,%f,%f,%f,%f,%f)", pose(1), pose(2), pose(3), pose(4), pose(5), pose(6));
            response = self.sendMoveCommand(cmd);
        end
        function response = MovL(self, pose)
            % 直线运动到指定的笛卡尔坐标点
            % pose: 1x6 数组 [X, Y, Z, Rx, Ry, Rz]
            cmd = sprintf("MovL(%f,%f,%f,%f,%f,%f)", pose(1), pose(2), pose(3), pose(4), pose(5), pose(6));
            response = self.sendMoveCommand(cmd);
        end
        function JointMovJ(self, joints)
            % 关节运动到指定的关节坐标点
            % joints: 1x6 数组 [J1, J2, J3, J4, J5, J6]
            cmd = sprintf("JointMovJ(%f,%f,%f,%f,%f,%f)", joints(1), joints(2), joints(3), joints(4), joints(5), joints(6));
            self.sendMoveCommand(cmd);
        end
        function response = MoveJog(self, axis)
            % 开始点动运动
            % axis: 字符串，如 'j1+', 'x-' 等
            cmd = sprintf("MoveJog(%s)", axis);
            response = self.sendMoveCommand(cmd);
        end
        function response = StopJog(self)
            % 停止点动运动
            response = self.sendMoveCommand("MoveJog()");
        end
        function response = StopMove(self)
            % 停止当前的运动脚本队列
            response = self.sendMoveCommand("StopScript()");
        end
        function response = ServoJ(self,joints,t,lookahead_time,gain)
            cmd = sprintf("ServoJ(%f,%f,%f,%f,%f,%f,t=%f,lookahead_time=%f,gain=%f)",joints(1), joints(2), joints(3), joints(4), joints(5), joints(6),t,lookahead_time,gain);
            % cmd = sprintf("ServoJ(%f,%f,%f,%f,%f,%f)",joints(1), joints(2), joints(3), joints(4), joints(5), joints(6));
            response = self.sendMoveCommand(cmd);
        end
    end

    % --- 私有方法 ---
    methods (Access = private)
        function response = sendDashboardCommand(self, command)
            % 发送指令到 Dashboard 端口并获取响应
            if ~self.IsConnected || isempty(self.DashboardClient)
                error('Not connected. Cannot send dashboard command.');
            end
            try
                write(self.DashboardClient, command, "char");
                pause(0.002); % 短暂等待响应
                if self.DashboardClient.NumBytesAvailable > 0
                    resBytes = read(self.DashboardClient, self.DashboardClient.NumBytesAvailable);
                    response = char(resBytes');
                    self.LastDashboardResponse = response;
                    % self.parseAndCheckError(response, command); % *** 新增：解析并检查错误 ***
                else
                    response = '';
                end
                fprintf('Sent(Dash): %s | Rcvd: %s\n', command, strtrim(response));
            catch ME
                error('Failed to send dashboard command "%s": %s', command, ME.message);
            end
        end

        function response = sendMoveCommand(self, command)
            % 发送指令到运动控制端口
            if ~self.IsConnected || isempty(self.MoveClient)
                error('Not connected. Cannot send move command.');
            end
            try
                write(self.MoveClient, command, "char");
                % pause(0.002); % 短暂等待响应
                if self.MoveClient.NumBytesAvailable > 0
                    resBytes = read(self.MoveClient, self.MoveClient.NumBytesAvailable);
                    response = char(resBytes');
                    self.LastMoveResponse = response;
                    % self.parseAndCheckError(response, command); % *** 新增：解析并检查错误 ***
                else
                    response = '';
                end
                 %fprintf('Sent(Move): %s | Rcvd: %s\n', command, strtrim(response));
            catch ME
                error('Failed to send move command "%s": %s', command, ME.message);
            end
        end

        % function sendMoveCommand(self, command)
        %     % 发送指令到运动控制端口
        %     write(self.MoveClient, command, "char");
        %     % pause(0.002); % 短暂等待响应
        % end

        function parseAndCheckError(~, response, command)
            % 解析响应字符串，如果 ErrorID 不为 0，则发出警告
            if isempty(response)
                return;
            end

            [tokens] = regexp(response, '^([-\d]+),', 'tokens');
            if isempty(tokens)
                return;
            end
            errorID = str2double(tokens{1}{1});

            if errorID ~= 0
                % --- 这是修正的核心部分 ---

                % 1. 先创建第一部分消息
                errorMsg = sprintf('Command ''%s'' failed. Received ErrorID: %d', command, errorID);

                % 2. 根据ErrorID获取第二部分描述
                errorDesc = ''; % 初始化为空
                switch errorID
                    case -1, errorDesc = 'Command received or executed failed.';
                    case -10000, errorDesc = 'Command does not exist.';
                    case -20000, errorDesc = 'Incorrect number of parameters.';
                    otherwise
                        if errorID > -40000 && errorID <= -30000
                            paramIndex = abs(errorID) + 30000;
                            errorDesc = sprintf('Parameter type error for parameter %d.', paramIndex);
                        elseif errorID > -50000 && errorID <= -40000
                            paramIndex = abs(errorID) + 40000;
                            errorDesc = sprintf('Parameter range error for parameter %d.', paramIndex);
                        else
                            errorDesc = 'Unknown error.';
                        end
                end

                % 3. 将两部分安全地组合成最终的警告信息字符串
                finalWarningMsg = sprintf('%s\nDescription: %s', errorMsg, errorDesc);

                % 4. 将这个已经拼接好的、安全的字符串传递给 warning 函数
                warning(finalWarningMsg);
            end
        end

        function res = TransBytes2Double(self, startIndex)
            % 将 8 个字节 (uint8) 转换为 double 类型
            bytes = self.StateDataArray(1, startIndex:startIndex+7);
            res = typecast(uint8(bytes), 'double');
        end

        function FeedBackTcpCallback(self, ~, ~)
            % 实时反馈数据的回调函数
            if self.FeedBackClient.NumBytesAvailable > 0
                incoming = read(self.FeedBackClient, ...
                    self.FeedBackClient.NumBytesAvailable, 'uint8');
            else
                return;
            end

            [frames, self.FeedbackByteBuffer, droppedByteCount] = ...
                decodeDobotFeedbackFrames(self.FeedbackByteBuffer, incoming);
            self.InvalidFeedbackByteCount = self.InvalidFeedbackByteCount + ...
                uint64(droppedByteCount);
            completeFrameCount = size(frames, 1);
            if completeFrameCount == 0
                return;
            end
            self.StateDataArray = frames(end, :);

            % 更新机器人模式
            % 实时反馈信息中的 RobotMode 是一个 uint64 [cite: 1297]
            % 此处使用字节位置 25 (MATLAB 1-based index) 来匹配原始代码逻辑
            % 这与 RobotMode() Dashboard 指令的返回值含义相对应 [cite: 532]
            robotModeVal = self.StateDataArray(1, 25);
            switch robotModeVal
                case 1, self.RobotMode = "INIT"; % 初始化状态
                case 2, self.RobotMode = "BRAKE_OPEN"; % 有关节抱闸松开
                case 3, self.RobotMode = "POWER_OFF"; % 本体未上电
                case 4, self.RobotMode = "DISABLED"; % 未使能
                case 5, self.RobotMode = "ENABLE"; % 使能且空闲
                case 6, self.RobotMode = "BACKDRIVE"; % 拖拽模式
                case 7, self.RobotMode = "RUNNING"; % 运行中
                case 8, self.RobotMode = "RECORDING"; % 轨迹录制模式
                case 9, self.RobotMode = "ERROR"; % 有未清除的报警
                case 10, self.RobotMode = "PAUSE"; % 暂停状态
                case 11, self.RobotMode = "JOG"; % 点动中
                otherwise, self.RobotMode = "UNKNOWN";
            end

            % 更新其他状态属性 (索引基于1-based MATLAB)
            self.CurrentSpeedRatio = self.TransBytes2Double(65); % 字节位置 64 [cite: 1297]

            self.JointAngles(1) = self.TransBytes2Double(433); % 实际关节位置，起始于字节 432 [cite: 1301]
            self.JointAngles(2) = self.TransBytes2Double(441);
            self.JointAngles(3) = self.TransBytes2Double(449);
            self.JointAngles(4) = self.TransBytes2Double(457);
            self.JointAngles(5) = self.TransBytes2Double(465);
            self.JointAngles(6) = self.TransBytes2Double(473);

            self.CartesianPose(1) = self.TransBytes2Double(625); % TCP 笛卡尔实际坐标值，起始于字节 624 [cite: 1301]
            self.CartesianPose(2) = self.TransBytes2Double(633);
            self.CartesianPose(3) = self.TransBytes2Double(641);
            self.CartesianPose(4) = self.TransBytes2Double(649);
            self.CartesianPose(5) = self.TransBytes2Double(657);
            self.CartesianPose(6) = self.TransBytes2Double(665);

            % Nova/CR V3 byte offsets 1012--1015 (zero based).
            self.ActiveUserIndex = uint8(self.StateDataArray(1, 1013));
            self.ActiveToolIndex = uint8(self.StateDataArray(1, 1014));
            self.RunQueuedCommand = ...
                self.StateDataArray(1, 1015) ~= uint8(0);
            self.PauseCommandFlag = ...
                self.StateDataArray(1, 1016) ~= uint8(0);

            % self.DigitalInputs = self.StateDataArray(1, 9:16); % 字节位置 8-15 [cite: 1297]
            % self.DigitalOutputs = self.StateDataArray(1, 17:24); % 字节位置 16-23 [cite: 1297]

            % QDActual (实际关节速度), 起始字节 480 -> MATLAB索引 481
            self.ActualJointSpeeds(1:6) = [ ...
                self.TransBytes2Double(481), self.TransBytes2Double(489), ...
                self.TransBytes2Double(497), self.TransBytes2Double(505), ...
                self.TransBytes2Double(513), self.TransBytes2Double(521)];

            % IActual (实际关节电流), 起始字节 528 -> MATLAB索引 529
            % self.ActualJointCurrents(1:6) = [ ...
            %     self.TransBytes2Double(529), self.TransBytes2Double(537), ...
            %     self.TransBytes2Double(545), self.TransBytes2Double(553), ...
            %     self.TransBytes2Double(561), self.TransBytes2Double(569)];

            % TCPSpeedActual (TCP笛卡尔实际速度值), 起始字节 672 -> MATLAB索引 673
            self.ActualTCPSpeed(1:6) = [ ...
                self.TransBytes2Double(673), self.TransBytes2Double(681), ...
                self.TransBytes2Double(689), self.TransBytes2Double(697), ...
                self.TransBytes2Double(705), self.TransBytes2Double(713)];

            % ActualQuaternion, protocol offset 1384, [qw,qx,qy,qz].
            self.ActualQuaternion(1:4) = [ ...
                self.TransBytes2Double(1385), self.TransBytes2Double(1393), ...
                self.TransBytes2Double(1401), self.TransBytes2Double(1409)];
            quaternionNorm = norm(self.ActualQuaternion);
            if isfinite(quaternionNorm) && quaternionNorm > eps
                self.ActualQuaternion = self.ActualQuaternion / ...
                    quaternionNorm;
            end
            self.FeedbackSequence = self.FeedbackSequence + ...
                uint64(completeFrameCount);
            if ~isempty(self.FeedbackClockStart)
                self.LastFeedbackMonotonicSec = toc(self.FeedbackClockStart);
            end

            % TCPForce (TCP力值), 起始字节 720 -> MATLAB索引 721
            % self.CalculatedTCPForce(1:6) = [ ...
            %     self.TransBytes2Double(721), self.TransBytes2Double(729), ...
            %     self.TransBytes2Double(737), self.TransBytes2Double(745), ...
            %     self.TransBytes2Double(753), self.TransBytes2Double(761)];

            % JointModes (关节控制模式), 起始字节 912 -> MATLAB索引 913
            % self.JointModes(1:6) = [ ...
            %     self.TransBytes2Double(913), self.TransBytes2Double(921), ...
            %     self.TransBytes2Double(929), self.TransBytes2Double(937), ...
            %     self.TransBytes2Double(945), self.TransBytes2Double(953)];
            %
            % self.TargetVelocity(1:6) = [ ...
            %     self.TransBytes2Double(241), self.TransBytes2Double(249), ...
            %     self.TransBytes2Double(257), self.TransBytes2Double(265), ...
            %     self.TransBytes2Double(273), self.TransBytes2Double(281)];

        end
    end
end
