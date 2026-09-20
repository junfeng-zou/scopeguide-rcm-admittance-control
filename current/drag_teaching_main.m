
function drag_teaching_main()
%% drag_teaching_main.m
% 基于六维力传感器的机械臂末端导纳控制拖动示教系统
% 适配: Dobot CR5 + OnRobot HEX-E/H QC 六维力传感器
%
% 算法链路:
%   传感器原始数据 → Rz180坐标变换 → 重力补偿 → 惯性力补偿
%   → 纯人手外力提取 → 死区滤波 → 一阶导纳模型 V=F/D
%   → 雅可比逆变换 q̇=J⁻¹V → ServoJ关节位置指令
%
% 坐标系定义 (参照 calibrate_end_effector_CoM.m):
%   零关节姿态下：
%   - 法兰 +Z 向左，+X 垂直纸面向里，+Y 向下，因此重力为法兰 +Y。
%   - 传感器 +Z 与法兰 +Z 同向，+X/+Y 与法兰相反，因此重力为传感器 -Y。
%   - F_flange = diag([-1,-1,1]) * F_sensor，力矩同样旋转。
%   - 所有补偿向量的“分量”统一用法兰坐标系表达；质心向量的原点
%     位于传感器测量中心，而不是法兰中心。
%
% 参考文献:
%   [1] OnRobot HEX-E/H QC Data Sheet v1.28
%   [2] Dobot CR5 User Manual

close all; clc;

%% ====================== 1. 用户可配置参数 ======================

% -------- 网络连接 --------
ROBOT_IP   = "192.168.50.105";        % Dobot CR5 控制柜 IP
SENSOR_IP  = "192.168.50.201";        % OnRobot HEX Modbus TCP IP (port 502)
DASH_PORT  = 29999;                    % Dashboard 端口
MOVE_PORT  = 30003;                    % 运动控制端口
FEED_PORT  = 30004;                    % 实时反馈端口

% -------- 末端工具参数 (来自标定结果 calibrate_end_effector_CoM.m) --------
M_TOOL  = 1.271095;                        % 最新联合标定的末端工具质量 (kg)
R_COM_FROM_SENSOR_ORIGIN_FLANGE = [0.006478; 0.060155; 0.047036];
% 上一行含义：原点=传感器测量中心，三个分量=法兰坐标系 [Xf,Yf,Zf] (m)
F_CALIBRATED_BIAS_FLANGE = [0.093464; -1.696729; 19.685233];
M_CALIBRATED_BIAS_FLANGE = [-0.551964; 0.057439; -0.141544];
% 上面两项只用于诊断启动零偏相对标定值的漂移，不参与控制量重复扣除。
GRAVITY = 9.81;                        % 重力加速度 (m/s²)

% -------- 坐标变换: 传感器系 → 法兰系 --------
% 实际安装：Xs=-Xf, Ys=-Yf, Zs=Zf，即传感器绕共同 Z 轴相对法兰转 180°。
% 用法：v_flange = R_SENSOR_TO_FLANGE * v_sensor。
R_SENSOR_TO_FLANGE = diag([-1, -1, 1]);

% 零关节姿态的已确认物理方向，用于运行时坐标自检。
ZERO_POSE_GRAVITY_FLANGE_UNIT = [0; 1; 0];
ZERO_POSE_GRAVITY_SENSOR_UNIT = [0; -1; 0];
ZERO_JOINT_TOL_DEG = 1.0;
ZERO_GRAVITY_DIRECTION_TOL = 0.05;

% -------- 导纳控制参数 (一阶模型: V_cmd = F_ext / D) --------
D_TRANS = [30; 30; 30];               % 平动阻尼
D_ROT   = [50;  50;  50];             % 转动阻尼系数 Nm/(rad/s)
ENABLE_ROTATION_CONTROL = false;       % 默认仅平移，避开当前腕部近奇异

% -------- 控制器反馈标定的局部数值雅可比 --------
USE_LOCAL_NUMERICAL_JACOBIAN = false;
NUMERICAL_JACOBIAN_FILE = fullfile(fileparts(mfilename('fullpath')), ...
    'cr5_numerical_jacobian_latest.mat');
NUMERICAL_JACOBIAN_START_TOL_DEG = 0.50;  % 启动姿态必须接近标定中心
NUMERICAL_JACOBIAN_MAX_DELTA_DEG = 2.00;  % 运行时局部模型有效范围
NUMERICAL_DLS_LAMBDA = 0.03;

% -------- 死区阈值 (低于此值视为噪声/零偏残差, 不响应) --------
F_DEADZONE_NORM = 0.3;                % 合力范数连续死区 (N)，保持施力方向
M_DEADZONE = [0.1; 0.1; 0.1];      % 力矩死区 (Nm)

% -------- 速度限制 --------
V_MAX_TRANS =0.40 ;                    % 最大平动速度 (m/s)
V_MAX_ROT   = 0.3;                     % 最大转动速度 (rad/s)
MAX_JOINT_VEL = 5;                  % 单关节最大速度 (rad/s)
MAX_JOINT_STEP_DEG = 5.0;              % 单次 ServoJ 最大关节增量 (deg)

% -------- 控制时序 --------
DT = 0.010;                            % Modbus TCP 轮询 50 Hz
DT_MIN = 0.010;                        % 实际积分周期下限 (s)
DT_MAX = 0.040;                        % 实际积分周期上限 (s)
SERVO_TIME_MIN = 0.015;                % ServoJ 插补时间下限 (s)
SERVO_TIME_MAX = 0.040;                % ServoJ 插补时间上限 (s)
SENSOR_TIMEOUT = 0.20;                 % 超过此时间无新力数据则停止 (s)

% -------- 启动预热与静止偏置估计 --------
ARM_SAMPLE_COUNT = 20;                 % 静止采样帧数，实测约 2 s
ARM_MAX_FORCE_STD = 0.40;              % 预热阶段单轴最大力标准差 (N)
ARM_MAX_MOMENT_STD = 0.03;             % 预热阶段单轴最大力矩标准差 (Nm)

% -------- 机器人反馈单位 --------
% Dobot TCPSpeedActual 的线速度单位是 mm/s，算法内部统一使用 m/s。
TCP_LINEAR_SPEED_SCALE = 1e-3;

% 惯性补偿依赖速度单位和反馈时序。先保持关闭，基础重力补偿稳定后再开启。
ENABLE_INERTIA_COMP = false;

% -------- 低通滤波系数 (0~1, 越小越平滑但延迟越大) --------
FILT_FORCE        = 0.35;              % 力信号滤波 (施力时)
FILT_FORCE_RELEASE = 0.70;             % 力信号滤波 (松手时, 快速释放)
FILT_VEL_ACTIVE   = 0.5;               % 施力时速度跟随系数
FILT_VEL_RELEASE  = 1;               % 松手时立即停止

% -------- 安全保护 --------
MAX_FORCE_STOP  = 60;                  % 外力超此值自动停止 (N)
MAX_MOMENT_STOP = 8;                   % 力矩超此值自动停止 (Nm)

% -------- 实时抖动诊断 --------
ENABLE_LIVE_DIAGNOSTICS = true;        % 关闭图窗后控制仍继续运行
DIAGNOSTIC_WINDOW_SEC = 30;             % 图中保留最近时间窗口 (s)
DIAGNOSTIC_UPDATE_HZ = 5;               % 图形刷新率，避免绘图干扰控制周期
CONSOLE_DIAGNOSTIC_INTERVAL = 10;        % 每多少个有效控制周期输出一次完整状态
REST_QDOT_THRESHOLD_DEG_S = 0.10;        % 无有效外力时的关节速度残留判断阈值
DIAGNOSTIC_LOG_FILE = fullfile(fileparts(mfilename('fullpath')), ...
    'drag_diagnostics_latest.mat');

validate_sensor_mount_rotation(R_SENSOR_TO_FLANGE, ...
    ZERO_POSE_GRAVITY_SENSOR_UNIT, ZERO_POSE_GRAVITY_FLANGE_UNIT);

localJacobian = struct([]);
if feature_enabled(USE_LOCAL_NUMERICAL_JACOBIAN)
    if feature_enabled(ENABLE_ROTATION_CONTROL)
        error('局部数值雅可比模式当前只允许平移控制，请保持 ENABLE_ROTATION_CONTROL=false。');
    end
    localJacobian = load_local_numerical_jacobian(NUMERICAL_JACOBIAN_FILE);
    fprintf(['[局部雅可比] 已加载 q0=[%.3f %.3f %.3f %.3f %.3f %.3f] deg, ' ...
        '位置RMSE=%.4f mm\n'], localJacobian.q0Deg, localJacobian.positionRmseMm);
end

%% ====================== 2. 初始化 ======================

fprintf('╔══════════════════════════════════════════╗\n');
fprintf('║   六维力传感器末端导纳拖动示教系统          ║\n');
fprintf('║   Dobot CR5 + OnRobot HEX                ║\n');
fprintf('╚══════════════════════════════════════════╝\n\n');

% 2.1 连接机器人
fprintf('[1/4] 连接机器人 (%s) ...\n', ROBOT_IP);
robot = ZJFDobotCR5(ROBOT_IP, DASH_PORT, MOVE_PORT, FEED_PORT);
robot.Connect();
if ~robot.IsConnected
    error('机器人连接失败。');
end
fprintf('      连接成功 | 当前模式: %s\n', robot.RobotMode);
robot_cleanup = onCleanup(@() safe_robot_shutdown(robot));

% 2.2 使能
fprintf('[2/4] 使能机器人 ...\n');
try
    robot.Enable(M_TOOL);
    pause(2.5);
    fprintf('      模式: %s | 关节角度: [%.1f %.1f %.1f %.1f %.1f %.1f]°\n', ...
        robot.RobotMode, robot.JointAngles);
    if feature_enabled(USE_LOCAL_NUMERICAL_JACOBIAN)
        startDeviationDeg = max(abs(robot.JointAngles(:)' - localJacobian.q0Deg));
        if startDeviationDeg > NUMERICAL_JACOBIAN_START_TOL_DEG
            error(['当前姿态距离数值雅可比标定中心 %.3f deg，超过启动限制 %.3f deg。' ...
                '请回到标定姿态或重新运行 calibrate_cr5_numerical_jacobian。'], ...
                startDeviationDeg, NUMERICAL_JACOBIAN_START_TOL_DEG);
        end
        fprintf('      局部雅可比启动检查通过 | max|q-q0|=%.3f deg\n', startDeviationDeg);
    end
catch ME
    if feature_enabled(USE_LOCAL_NUMERICAL_JACOBIAN)
        rethrow(ME);
    end
    warning(ME.identifier, '%s', ME.message);
end

% 2.3 设置速度
robot.SetSpeedRatio(30);

% 2.4 连接六维力传感器 (Modbus TCP, 50 Hz 轮询)
fprintf('[3/4] 连接六维力传感器 (Modbus TCP %s:502) ...\n', SENSOR_IP);
hex_modbus_read('init', SENSOR_IP);
fprintf('      Modbus TCP 连接成功\n');
sensor_cleanup = onCleanup(@() hex_modbus_read('close'));

% 2.5 初始化状态变量
fprintf('[4/4] 初始化控制变量 ...\n');
F_force_filt  = [];                % 首帧有效数据到达后直接初始化，避免启动瞬态
M_moment_filt = [];
V_cmd_filt    = zeros(6, 1);       % 速度指令低通滤波状态
V_cmd_filt_prev = zeros(6, 1);     % 上一帧速度 (变化率限制用)
v_linear_prev = zeros(3, 1);       % 上一帧 TCP 线速度 (用于加速度估算)
t_prev        = NaN;               % 上一帧有效数据时间戳
a_linear_filt = zeros(3, 1);       % 加速度低通滤波状态
FILT_ACCEL    = 0.3;               % 加速度滤波系数

last_sensor_time = NaN;
is_armed = false;
arm_count = 0;
arm_force_residual = zeros(3, ARM_SAMPLE_COUNT);
arm_moment_residual = zeros(3, ARM_SAMPLE_COUNT);
F_static_bias = zeros(3, 1);
M_static_bias = zeros(3, 1);
servo_diag_printed = false;
zero_pose_coordinate_checked = false;
last_control_time = NaN;
q_target_prev_rad = [];    % 上一轮 ServoJ 下发的目标 (弧度), 用于积分闭环
zero_idle_count = 0;         % 连续静止帧计数 (自动归零用)
ZERO_IDLE_FRAMES = 50;       % 连续静止 50 帧 (~0.5s) 后自动刷新偏置
ZERO_BIAS_ALPHA = 0.05;       % 偏置刷新平滑系数 (0=不动, 1=全换)
live_diag = init_drag_diagnostics(ENABLE_LIVE_DIAGNOSTICS, ...
    DIAGNOSTIC_WINDOW_SEC, DIAGNOSTIC_UPDATE_HZ, DT);

fprintf('      初始化完成。\n\n');
fprintf('┌──────────────────────────────────────────┐\n');
fprintf('│  拖动示教已启动。                         │\n');
fprintf('│  轻轻推拉机械臂末端即可拖动机器人运动。     │\n');
fprintf('│  按 Ctrl+C 停止拖动示教。                 │\n');
fprintf('└──────────────────────────────────────────┘\n\n');

%% ====================== 3. 主控制循环 ======================

timer_total = tic;
t_next_iter = 0;
iter = 0;

try
    while true
        % --- 等待下一个控制周期 ---
        t_now_s = toc(timer_total);
        if t_now_s < t_next_iter
            continue;
        end
        t_next_iter = t_now_s + DT;
        iter = iter + 1;

        % ====================================================
        % 步骤 3.1: 读取六维力传感器数据 (Modbus TCP 轮询)
        % ====================================================
        t0 = toc(timer_total);
        [F_sens, M_sens, ok_sensor] = hex_modbus_read('read');
        t_sensor = toc(timer_total) - t0;
        if ~ok_sensor
            % 没有新数据时不允许沿用旧外力继续运动。
            V_cmd_filt(:) = 0;
            t_check_s = toc(timer_total);
            if isnan(last_sensor_time) && t_check_s > SENSOR_TIMEOUT
                fprintf('\n!!! [传感器超时] 启动后 %.0f ms 内未收到首帧有效力数据 !!!\n', ...
                    t_check_s * 1000);
                break;
            elseif ~isnan(last_sensor_time) && (t_check_s - last_sensor_time) > SENSOR_TIMEOUT
                fprintf('\n!!! [传感器超时] %.0f ms 未收到有效力数据，停止控制 !!!\n', ...
                    (t_check_s - last_sensor_time) * 1000);
                break;
            end
            try
                robot.ServoJ(robot.JointAngles, DT);
            catch
            end
            continue;
        end
        sample_time_s = toc(timer_total);
        last_sensor_time = sample_time_s;

        % HTTP 长轮询决定了真实更新周期。积分和 ServoJ 必须使用实际周期，
        % 不能继续假设 32 ms，否则会出现小力不动、大力启停震荡。
        if isnan(last_control_time)
            control_dt = DT;
        else
            control_dt = min(DT_MAX, max(DT_MIN, sample_time_s - last_control_time));
        end
        servo_time = min(SERVO_TIME_MAX, max(SERVO_TIME_MIN, control_dt));
        last_control_time = sample_time_s;

        % ====================================================
        % 步骤 3.2: 坐标变换 — 传感器系 → 法兰系
        %   F_flange = Rz180 * F_sensor
        %   M_flange = Rz180 * M_sensor
        % ====================================================
        F_flange_raw = R_SENSOR_TO_FLANGE * F_sens;
        M_flange_raw = R_SENSOR_TO_FLANGE * M_sens;

        % 首帧直接作为滤波器初值，避免从零开始造成 75% 重力残差。
        if isempty(F_force_filt)
            F_force_filt = F_flange_raw;
            M_moment_filt = M_flange_raw;
        else
            % 自适应滤波: 力上升时慢(防抖), 下降时快(松手即停)
            if norm(F_flange_raw) < norm(F_force_filt)
                alpha_f = FILT_FORCE_RELEASE;
            else
                alpha_f = FILT_FORCE;
            end
            F_force_filt  = alpha_f * F_flange_raw + (1 - alpha_f) * F_force_filt;
            M_moment_filt = alpha_f * M_flange_raw + (1 - alpha_f) * M_moment_filt;
        end

        % ====================================================
        % 步骤 3.3: 读取机器人当前状态
        % ====================================================
        t1 = toc(timer_total);
        q_deg  = robot.JointAngles(:);     % 关节角度 (度), 6×1
        q_rad  = q_deg * pi / 180;         % 关节角度 (弧度)
        % ActualTCPSpeed 仅在惯性补偿开启时有意义，关闭时跳过以减少一次网络往返。
        if feature_enabled(ENABLE_INERTIA_COMP)
            tcp_spd = robot.ActualTCPSpeed;
            v_lin_now = tcp_spd(1:3)' * TCP_LINEAR_SPEED_SCALE;
        else
            v_lin_now = zeros(3, 1);
        end
        t_joints = toc(timer_total) - t1;

        % ====================================================
        % 步骤 3.4: 获取法兰姿态 — 直接用机器人控制器反馈的 RPY
        %   不用自算 FK (近似 DH 误差太大)，机器人自己最清楚姿态
        %  每 3 帧读一次姿态（拖动中姿态变化缓慢），减少网络往返。
        % ====================================================
        t2 = toc(timer_total);
        if iter == 1 || mod(iter, 3) == 0 || ~is_armed
            cart_pose = robot.CartesianPose;
            R_flange_in_base = dobot_rpy_deg_to_rotm(cart_pose(4:6));
            did_read_pose = true;
        else
            did_read_pose = false;
        end
        if feature_enabled(USE_LOCAL_NUMERICAL_JACOBIAN)
            T_all = [];
        else
            [~, T_all] = fkine_cr5(q_rad);  % 仅用于旧 DH Jacobian
        end
        t_pose_and_fk = toc(timer_total) - t2;

        % ====================================================
        % 步骤 3.5: 重力补偿
        %
        %   重力在基坐标系:   g_base = [0; 0; -GRAVITY]
        %   重力在法兰坐标系: g_flange = R^T * g_base
        %   注意：法兰 +Y 只对应六关节全零姿态。其他姿态禁止写死方向，
        %         必须使用控制器实时 RPY 计算 R 后再旋转重力。
        %   重力产生力:       F_g = m * g_flange
        %   重力产生力矩:     M_g = r_com × F_g
        %   (力矩绕传感器测量中心)
        % ====================================================
        g_flange  = R_flange_in_base' * [0; 0; -GRAVITY];
        F_gravity = M_TOOL * g_flange;
        M_gravity = cross(R_COM_FROM_SENSOR_ORIGIN_FLANGE, F_gravity);

        % 零关节姿态的强制坐标自检：物理重力必须是法兰 +Y、传感器 -Y。
        if ~zero_pose_coordinate_checked && max(abs(q_deg)) <= ZERO_JOINT_TOL_DEG
            g_flange_unit = g_flange / norm(g_flange);
            g_sensor_unit = R_SENSOR_TO_FLANGE' * g_flange_unit;
            if norm(g_flange_unit - ZERO_POSE_GRAVITY_FLANGE_UNIT) > ...
                    ZERO_GRAVITY_DIRECTION_TOL || ...
                    norm(g_sensor_unit - ZERO_POSE_GRAVITY_SENSOR_UNIT) > ...
                    ZERO_GRAVITY_DIRECTION_TOL
                error(['零姿态坐标自检失败：期望重力为法兰 +Y / 传感器 -Y，' ...
                    '实际法兰=[%.4f %.4f %.4f]，传感器=[%.4f %.4f %.4f]。' ...
                    '请检查 Dobot RPY 约定，禁止通过修改力符号绕过此错误。'], ...
                    g_flange_unit(1), g_flange_unit(2), g_flange_unit(3), ...
                    g_sensor_unit(1), g_sensor_unit(2), g_sensor_unit(3));
            end
            fprintf(['[坐标自检通过] q≈0: 重力法兰=(%.4f,%.4f,%.4f)，' ...
                '传感器=(%.4f,%.4f,%.4f)\n'], ...
                g_flange_unit(1), g_flange_unit(2), g_flange_unit(3), ...
                g_sensor_unit(1), g_sensor_unit(2), g_sensor_unit(3));
            zero_pose_coordinate_checked = true;
        end

        % ====================================================
        % 步骤 3.5.1: 启动预热和静止偏置估计
        %   仅估计“测量值 - 理论重力”的残差，不会重复扣除工具重力。
        %   预热完成前只保持当前位置，不发送运动速度。
        % ====================================================
        if ~is_armed
            arm_count = arm_count + 1;
            arm_force_residual(:, arm_count) = F_force_filt - F_gravity;
            arm_moment_residual(:, arm_count) = M_moment_filt - M_gravity;

            robot.ServoJ(q_deg', servo_time);
            if arm_count < ARM_SAMPLE_COUNT
                if mod(arm_count, 10) == 0
                    fprintf('[预热] 静止采样 %d/%d，请勿接触末端。\n', arm_count, ARM_SAMPLE_COUNT);
                end
                continue;
            end

            F_static_bias = mean(arm_force_residual, 2);
            M_static_bias = mean(arm_moment_residual, 2);
            F_noise_std = std(arm_force_residual, 0, 2);
            M_noise_std = std(arm_moment_residual, 0, 2);
            F_bias_drift = F_static_bias - F_CALIBRATED_BIAS_FLANGE;
            M_bias_drift = M_static_bias - M_CALIBRATED_BIAS_FLANGE;

            if any(F_noise_std > ARM_MAX_FORCE_STD) || any(M_noise_std > ARM_MAX_MOMENT_STD)
                error(['启动预热失败：静止数据波动过大。请检查是否有人接触末端、' ...
                    '软管是否拉扯或传感器是否漂移。F_std=[%.3f %.3f %.3f]N, ' ...
                    'M_std=[%.4f %.4f %.4f]Nm'], ...
                    F_noise_std(1), F_noise_std(2), F_noise_std(3), ...
                    M_noise_std(1), M_noise_std(2), M_noise_std(3));
            end

            fprintf('[预热完成] 静止偏置 F=(%.3f,%.3f,%.3f)N, M=(%.4f,%.4f,%.4f)Nm\n', ...
                F_static_bias(1), F_static_bias(2), F_static_bias(3), ...
                M_static_bias(1), M_static_bias(2), M_static_bias(3));
            fprintf('[预热噪声] F_std=(%.3f,%.3f,%.3f)N, M_std=(%.4f,%.4f,%.4f)Nm\n', ...
                F_noise_std(1), F_noise_std(2), F_noise_std(3), ...
                M_noise_std(1), M_noise_std(2), M_noise_std(3));
            fprintf(['[偏置漂移] 相对标定值 dF=(%.3f,%.3f,%.3f)N |dF|=%.3fN, ' ...
                'dM=(%.4f,%.4f,%.4f)Nm |dM|=%.4fNm\n'], ...
                F_bias_drift(1), F_bias_drift(2), F_bias_drift(3), norm(F_bias_drift), ...
                M_bias_drift(1), M_bias_drift(2), M_bias_drift(3), norm(M_bias_drift));
            fprintf('[补偿已激活] 静止时请确认 F_pre 各轴位于死区内、F_used=0、qdot_cmd≈0。\n');
            if norm(F_bias_drift) > 3.0 || norm(M_bias_drift) > 0.30
                warning(['当前启动偏置相对联合标定值漂移较大。请检查传感器清零状态、' ...
                    '预热、线缆拉扯；若多次启动仍一致，建议重新运行联合标定。']);
            end

            is_armed = true;
            v_linear_prev = v_lin_now;
            t_prev = sample_time_s;
            a_linear_filt(:) = 0;
            V_cmd_filt(:) = 0;
            q_target_prev_rad = q_rad;
            fprintf('导纳控制已解锁。\n\n');
            continue;
        end

        % ====================================================
        % 步骤 3.6: 惯性力补偿 (D'Alembert 原理)
        %
        %   末端工具加速度 a (法兰系)
        %   惯性力:   F_i = -m·a            (与加速度反向)
        %   惯性力矩: M_i = r_com × (-m·a)  (点质量假设, 绕传感器中心)
        %
        %   加速度从 TCP 线速度差分估算 + 低通滤波
        % ====================================================
        if feature_enabled(ENABLE_INERTIA_COMP) && ~isnan(t_prev) && ...
                sample_time_s > t_prev && (sample_time_s - t_prev) < SENSOR_TIMEOUT
            dt_actual = sample_time_s - t_prev;
            % TCP 加速度 (基坐标系 → 法兰坐标系)
            a_linear_base = (v_lin_now - v_linear_prev) / dt_actual;
            a_linear_flange = R_flange_in_base' * a_linear_base;

            % 低通滤波加速度 (避免差分噪声放大)
            a_linear_filt = FILT_ACCEL * a_linear_flange + (1 - FILT_ACCEL) * a_linear_filt;

            % D'Alembert 惯性力
            F_inertia = -M_TOOL * a_linear_filt;     % 在法兰坐标系下
            M_inertia = cross(R_COM_FROM_SENSOR_ORIGIN_FLANGE, F_inertia);
        else
            F_inertia = zeros(3, 1);
            M_inertia = zeros(3, 1);
            a_linear_filt(:) = 0;
        end

        % ====================================================
        % 步骤 3.7: 提取纯人手交互外力
        %   F_ext = F_measured - F_gravity - F_inertia
        % ====================================================
        F_ext = F_force_filt - F_gravity - F_static_bias - F_inertia;
        M_ext = M_moment_filt - M_gravity - M_static_bias - M_inertia;
        F_ext_pre_deadzone = F_ext;
        M_ext_pre_deadzone = M_ext;

        % ---- 安全急停检查 ----
        if norm(F_ext) > MAX_FORCE_STOP || norm(M_ext) > MAX_MOMENT_STOP
            fprintf('\n!!! [安全急停] 外力超限 |F|=%.1f N |M|=%.2f Nm !!!\n', ...
                norm(F_ext), norm(M_ext));
            break;
        end

        % ====================================================
        % 步骤 3.8: 死区滤波
        %   低于阈值的力/力矩视为传感器噪声或补偿残差
        % ====================================================
        F_ext = smooth_vector_deadzone(F_ext, F_DEADZONE_NORM);
        M_ext = smooth_deadzone(M_ext, M_DEADZONE);

        fprintf('[滤波] F_filt=(%6.2f,%6.2f,%6.2f)N M_filt=(%6.3f,%6.3f,%6.3f)Nm | [F_ext] 死区前:(%6.2f,%6.2f,%6.2f)N |F|=%.2f → 死区后:(%6.2f,%6.2f,%6.2f)N |F|=%.2f\n', ...
            F_force_filt(1), F_force_filt(2), F_force_filt(3), ...
            M_moment_filt(1), M_moment_filt(2), M_moment_filt(3), ...
            F_ext_pre_deadzone(1), F_ext_pre_deadzone(2), F_ext_pre_deadzone(3), norm(F_ext_pre_deadzone), ...
            F_ext(1), F_ext(2), F_ext(3), norm(F_ext));

        % 自动归零: 连续静止时渐进刷新偏置, 消除姿态相关补偿残差
        %  路径A: 力残差 < 死区×1.5 → 补偿准, 快速更新 (50帧)
        %  路径B: 力残差大但关节物理静止 → 补偿偏了, 慢速更新 (150帧, 大修正)
        is_force_quiet = all(abs(F_ext_pre_deadzone) < F_DEADZONE_NORM * 1.5);
        is_joint_still = max(abs(robot.ActualJointSpeeds)) < 0.1;
        is_idle = is_force_quiet || is_joint_still;
        if is_idle
            zero_idle_count = zero_idle_count + 1;
            need_frames = ZERO_IDLE_FRAMES;
            update_alpha = ZERO_BIAS_ALPHA;
            if ~is_force_quiet
                need_frames = ZERO_IDLE_FRAMES * 3;   % 路径B: 等久一点
                update_alpha = ZERO_BIAS_ALPHA * 3;   % 路径B: 步子大一点
            end
            if zero_idle_count >= need_frames
                residual_f = F_force_filt - F_gravity;
                F_static_bias = F_static_bias + update_alpha * (residual_f - F_static_bias);
                residual_m = M_moment_filt - M_gravity;
                M_static_bias = M_static_bias + update_alpha * (residual_m - M_static_bias);
                zero_idle_count = 0;
                tag = 'A(力安静)'; if ~is_force_quiet, tag = 'B(关节静止)'; end
                fprintf('[自动归零] F_bias=(%.2f,%.2f,%.2f)N | 路径=%s\n', ...
                    F_static_bias, tag);
            end
        else
            zero_idle_count = max(zero_idle_count - 1, 0);
        end

        % ====================================================
        % 步骤 3.9: 一阶导纳模型
        %   V_trans = F_ext ./ D_trans   (平动)
        %   V_rot   = M_ext ./ D_rot     (转动)
        % ====================================================
        V_trans_raw = F_ext ./ D_TRANS;      % 3×1 平动速度 (m/s)
        if feature_enabled(ENABLE_ROTATION_CONTROL)
            V_rot_raw = M_ext ./ D_ROT;
        else
            V_rot_raw = zeros(3, 1);
        end
        V_cmd_raw   = [V_trans_raw; V_rot_raw];  % 6×1

        % 施力时平滑跟随；松手时快速释放，避免 F_ext 已为零但速度仍残留。
        if any(abs(F_ext) > 0.01)
            alpha_vel = FILT_VEL_ACTIVE;
        else
            alpha_vel = FILT_VEL_RELEASE;
        end
        V_cmd_filt = alpha_vel * V_cmd_raw + (1 - alpha_vel) * V_cmd_filt;
        % 变化率限制: 削掉高频噪声尖峰, 正常推力变化不受影响
        dV_max = 0.15;  % 每帧速度变化上限 (m/s), ≈ 50mm/s/frame
        dV = V_cmd_filt - V_cmd_filt_prev;
        dV_norm = norm(dV(1:3));
        if dV_norm > dV_max
            dV(1:3) = dV(1:3) * (dV_max / dV_norm);
        end
        V_cmd_filt = V_cmd_filt_prev + dV;
        V_cmd_filt_prev = V_cmd_filt;
        V_cmd_filt(abs(V_cmd_filt) < 2e-4) = 0;

        % 速度指令限幅
        V_cmd_filt(1:3) = limit_vector_norm(V_cmd_filt(1:3), V_MAX_TRANS);
        V_cmd_filt(4:6) = limit_vector_norm(V_cmd_filt(4:6), V_MAX_ROT);

        % ====================================================
        % 步骤 3.10: 雅可比逆变换 → 关节速度
        %
        %   V_cmd 在法兰坐标系下 (因为 F_ext 在法兰系)
        %   J_geo 在基坐标系下: V_base = J_geo · q̇
        %   需做旋转变换: V_base = R6 · V_flange
        %   然后求解: q̇ = J_geo⁻¹ · R6 · V_flange
        %
        %   使用阻尼最小二乘 (DLS) 避免奇异
        % ====================================================
        t3 = toc(timer_total);
        if feature_enabled(USE_LOCAL_NUMERICAL_JACOBIAN)
            jacobian_deviation_deg = max(abs(q_deg' - localJacobian.q0Deg));
            if jacobian_deviation_deg > NUMERICAL_JACOBIAN_MAX_DELTA_DEG
                error(['已离开局部数值雅可比有效范围：max|q-q0|=%.3f deg > %.3f deg。' ...
                    '控制已停止，请在新姿态重新标定。'], ...
                    jacobian_deviation_deg, NUMERICAL_JACOBIAN_MAX_DELTA_DEG);
            end
            J_geo = localJacobian.JNumericBase;
            jacobian_source = 'numeric-local';
        else
            jacobian_deviation_deg = NaN;
            J_geo = jacob_cr5_geo(q_rad, T_all);
            jacobian_source = 'approx-DH';
        end

        if feature_enabled(ENABLE_ROTATION_CONTROL)
            % 完整 6D 模式会受到腕部奇异影响，仅在远离奇异位形时启用。
            J_active = J_geo;
            R6 = blkdiag(R_flange_in_base, R_flange_in_base);
            V_active_base = R6 * V_cmd_filt;
        else
            % 平移拖动只求解 Jv，避免 J5 接近 0 时 J4/J6 轴重合影响平移。
            J_active = J_geo(1:3, :);
            V_active_base = R_flange_in_base * V_cmd_filt(1:3);
        end

        singular_values = svd(J_active);
        sigma_min_active = min(singular_values);
        if feature_enabled(USE_LOCAL_NUMERICAL_JACOBIAN)
            lambda_dls = NUMERICAL_DLS_LAMBDA;
        else
            lambda_dls = 0.03;
        end
        if ~feature_enabled(USE_LOCAL_NUMERICAL_JACOBIAN) && sigma_min_active < 0.08
            lambda_dls = lambda_dls + 0.15 * (1 - sigma_min_active / 0.08)^2;
        end
        J_dls_inv = J_active' / ...
            (J_active * J_active' + lambda_dls^2 * eye(size(J_active, 1)));
        q_dot_cmd = J_dls_inv * V_active_base;

        % 关节速度限幅
        q_dot_cmd = saturate(q_dot_cmd, MAX_JOINT_VEL);

        % ====================================================
        % 步骤 3.11: 关节位置指令 (从上一目标积分, 避免反馈延迟)
        %   q_cmd = q_target_prev + q̇_des * Δt
        % ====================================================
        q_step_rad = q_dot_cmd * control_dt;
        q_step_rad = saturate(q_step_rad, MAX_JOINT_STEP_DEG * pi / 180);
        q_target_rad = q_target_prev_rad + q_step_rad;
        % 安全钳: 限制目标偏离当前位置不超过 3°, 防止正反馈漂移
        max_dev_rad = 3 * pi / 180;
        for jj = 1:6
            q_target_rad(jj) = max(q_rad(jj) - max_dev_rad, ...
                               min(q_rad(jj) + max_dev_rad, q_target_rad(jj)));
        end
        q_target_deg = q_target_rad * 180 / pi;

        % ====================================================
        % 步骤 3.12: 通过 ServoJ 发送关节位置指令
        % ====================================================
        t4 = toc(timer_total);
        robot.ServoJ(q_target_deg', servo_time);
        q_target_prev_rad = q_target_rad;
        t_servo = toc(timer_total) - t4;
        fprintf('[ServoJ] t=%.0fms | q_tgt=[%.3f %.3f %.3f %.3f %.3f %.3f]° | step_rad=[%.4f %.4f %.4f %.4f %.4f %.4f] | step_deg=[%.4f %.4f %.4f %.4f %.4f %.4f]°\n', ...
            servo_time*1000, q_target_deg, q_step_rad, q_step_rad' * 180/pi);
        t_loop = toc(timer_total) - t0;

        % 每个控制周期记录数据，图形只按较低频率刷新。
        q_speed_actual_deg_s = robot.ActualJointSpeeds(:);
        if numel(q_speed_actual_deg_s) ~= 6 || ...
                any(~isfinite(q_speed_actual_deg_s))
            q_speed_actual_deg_s = nan(6, 1);
        end
        force_model = F_gravity + F_static_bias + F_inertia;
        moment_model = M_gravity + M_static_bias + M_inertia;
        live_diag = update_drag_diagnostics(live_diag, sample_time_s, ...
            q_deg, q_target_deg, q_speed_actual_deg_s, q_dot_cmd * 180 / pi, ...
            F_force_filt, force_model, F_ext_pre_deadzone, F_ext, ...
            M_moment_filt, moment_model, M_ext_pre_deadzone, M_ext, ...
            V_cmd_filt(1:3) * 1000, ...
            V_cmd_raw(1:3) * 1000, ...
            [control_dt; t_sensor; t_servo; t_loop] * 1000, ...
            sigma_min_active, lambda_dls, F_sens, M_sens);

        % ---- 诊断: 首次运行时检查 ServoJ 响应 ----
        if ~servo_diag_printed
            fprintf('[诊断] 机器人模式: %s\n', robot.RobotMode);
            fprintf('[诊断] 当前关节: [%.2f %.2f %.2f %.2f %.2f %.2f]°\n', q_deg);
            fprintf('[诊断] 目标关节: [%.2f %.2f %.2f %.2f %.2f %.2f]°\n', q_target_deg);
            fprintf('[诊断] 关节增量: [%.4f %.4f %.4f %.4f %.4f %.4f]°\n', ...
                q_step_rad' * 180/pi);
            fprintf('[诊断] ServoJ 响应: [%s]\n', strtrim(robot.LastMoveResponse));
            servo_diag_printed = true;
        end

        % ---- 详细诊断 ----
        if mod(iter, CONSOLE_DIAGNOSTIC_INTERVAL) == 0
            q_dot_cmd_deg_s = q_dot_cmd * 180 / pi;
            force_is_active = any(abs(F_ext) > 1e-9);
            command_is_moving = max(abs(q_dot_cmd_deg_s)) > REST_QDOT_THRESHOLD_DEG_S;
            if force_is_active
                control_state = '检测到有效外力，机械臂跟随中';
            elseif command_is_moving
                control_state = '警告：无有效外力但仍有速度指令';
            else
                control_state = '静止正常：死区后外力和速度指令均为零';
            end

            fprintf('\n══════ [iter=%d | t=%.1fs] ══════\n', iter, sample_time_s);
            fprintf('  ┌── 时间分布 ─────────────────────────\n');
            fprintf('  │ modbus_read : %6.1f ms\n', t_sensor * 1000);
            fprintf('  │ JointAngles  : %6.1f ms\n', t_joints * 1000);
            if did_read_pose
                fprintf('  │ CartesianPose+FK: %6.1f ms (含读取)\n', t_pose_and_fk * 1000);
            else
                fprintf('  │ FK only       : %6.1f ms (复用姿态)\n', t_pose_and_fk * 1000);
            end
            fprintf('  │ J_geo + DLS  : %6.1f ms\n', (t4 - t3) * 1000);
            fprintf('  │ ServoJ       : %6.1f ms\n', t_servo * 1000);
            fprintf('  │ 本轮总计     : %6.1f ms (%.1f Hz)\n', ...
                t_loop * 1000, 1 / max(t_loop, 0.001));
            fprintf('  ├── 控制数据 ─────────────────────────\n');
            fprintf('  │ control_dt   : %6.1f ms  servo_time: %6.1f ms\n', ...
                control_dt * 1000, servo_time * 1000);
            fprintf('  │ sigma_min(J) : %.4f      lambda_dls: %.4f\n', ...
                sigma_min_active, lambda_dls);
            if feature_enabled(USE_LOCAL_NUMERICAL_JACOBIAN)
                fprintf('  │ J source     : %s  max|q-q0|=%.3f/%.3f deg\n', ...
                    jacobian_source, jacobian_deviation_deg, ...
                    NUMERICAL_JACOBIAN_MAX_DELTA_DEG);
            else
                fprintf('  │ J source     : %s\n', jacobian_source);
            end
            fprintf('  ├── 力数据 (法兰系) ──────────────────\n');
            fprintf('  │ F_raw  = (%6.2f, %6.2f, %6.2f) N\n', ...
                F_force_filt(1), F_force_filt(2), F_force_filt(3));
            fprintf('  │ F_grav = (%6.2f, %6.2f, %6.2f) N\n', ...
                F_gravity(1), F_gravity(2), F_gravity(3));
            fprintf('  │ F_bias = (%6.2f, %6.2f, %6.2f) N\n', ...
                F_static_bias(1), F_static_bias(2), F_static_bias(3));
            fprintf('  │ F_pre  = (%6.2f, %6.2f, %6.2f) N  |F|=%.2f  (补偿后/死区前)\n', ...
                F_ext_pre_deadzone(1), F_ext_pre_deadzone(2), ...
                F_ext_pre_deadzone(3), norm(F_ext_pre_deadzone));
            fprintf('  │ F_used = (%6.2f, %6.2f, %6.2f) N  |F|=%.2f  (导纳实际输入)\n', ...
                F_ext(1), F_ext(2), F_ext(3), norm(F_ext));
            fprintf('  │ M_pre  = (%6.3f, %6.3f, %6.3f) Nm |M|=%.3f\n', ...
                M_ext_pre_deadzone(1), M_ext_pre_deadzone(2), ...
                M_ext_pre_deadzone(3), norm(M_ext_pre_deadzone));
            fprintf('  │ M_used = (%6.3f, %6.3f, %6.3f) Nm |M|=%.3f\n', ...
                M_ext(1), M_ext(2), M_ext(3), norm(M_ext));
            fprintf('  ├── 速度指令 (法兰系) ────────────────\n');
            fprintf('  │ V_raw  = (%5.0f, %5.0f, %5.0f) mm/s | %5.0f %5.0f %5.0f deg/s\n', ...
                V_cmd_raw(1)*1000, V_cmd_raw(2)*1000, V_cmd_raw(3)*1000, ...
                V_cmd_raw(4)*180/pi, V_cmd_raw(5)*180/pi, V_cmd_raw(6)*180/pi);
            fprintf('  │ V_filt = (%5.0f, %5.0f, %5.0f) mm/s\n', ...
                V_cmd_filt(1)*1000, V_cmd_filt(2)*1000, V_cmd_filt(3)*1000);
            fprintf('  ├── 关节数据 ─────────────────────────\n');
            fprintf('  │ q_cur  = [%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f]°\n', q_deg);
            fprintf('  │ q_tgt  = [%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f]°\n', ...
                q_target_deg);
            fprintf('  │ qdot_cmd = [%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f] deg/s\n', ...
                q_dot_cmd_deg_s);
            fprintf('  │ qdot_act = [%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f] deg/s\n', ...
                q_speed_actual_deg_s);
            fprintf('  │ q_step = [%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f]°\n', ...
                q_step_rad' * 180/pi);
            fprintf('  │ 状态: %s\n', control_state);
            fprintf('  └──────────────────────────────────────\n');
        end
    end

catch ME
    fprintf('\n!!! 控制循环异常: %s !!!\n', ME.message);
    fprintf('堆栈: %s\n', getReport(ME));
end

%% ====================== 4. 安全退出 ======================

fprintf('\n正在安全退出...\n');
save_drag_diagnostics(live_diag, DIAGNOSTIC_LOG_FILE);
clear sensor_cleanup;
clear robot_cleanup;
fprintf('已安全退出。总运行时间: %.1f s | 控制周期数: %d\n', toc(timer_total), iter);
end


%% ========================================================================
%                          局 部 函 数
%% ========================================================================

% ==================== DH 参数与前向运动学 ====================

function dh = dh_params_cr5()
% DH_PARAMS_CR5  Dobot CR5 标准 DH 参数 (Denavit-Hartenberg)
% 返回 6×4 矩阵, 每行 [a_i, alpha_i, d_i, theta_offset_i]
%   a:     连杆长度 (m)
%   alpha: 连杆扭转角 (rad)
%   d:     连杆偏距 (m)
%   theta_offset: 关节角零位偏置 (rad)
%
% 注意: 本参数为近似值, 精确标定可提升雅可比矩阵精度。
%       实际力控精度对 DH 精度不敏感 (由导纳闭环保证)。
    dh = [
        0,       pi/2,   0.240,  0;       % J1: 基座旋转
        0.400,   0,      0.135,  pi/2;    % J2: 肩部 (大臂)
        0.330,   0,      0,      0;       % J3: 肘部
        0,       pi/2,   0,      pi/2;    % J4: 腕部俯仰
        0,      -pi/2,   0.120,  0;       % J5: 腕部偏转
        0,       0,      0.088,  0;       % J6: 法兰旋转
    ];
end

function [T_end, T_all] = fkine_cr5(q)
% FKINE_CR5  前向运动学 (标准 DH)
%   q:     6×1 关节角度 (弧度)
%   T_end: 4×4 末端法兰齐次变换矩阵 (基坐标系下)
%   T_all: 4×4×6 各连杆变换矩阵
    dh = dh_params_cr5();
    T_all = zeros(4, 4, 6);
    T = eye(4);

    for i = 1:6
        a_i     = dh(i, 1);
        alpha_i = dh(i, 2);
        d_i     = dh(i, 3);
        th_off  = dh(i, 4);
        theta_i = q(i) + th_off;

        c_th = cos(theta_i);  s_th = sin(theta_i);
        c_al = cos(alpha_i);  s_al = sin(alpha_i);

        T_i = [c_th,  -s_th*c_al,   s_th*s_al,  a_i*c_th;
               s_th,   c_th*c_al,  -c_th*s_al,  a_i*s_th;
               0,      s_al,        c_al,        d_i;
               0,      0,           0,           1];

        T = T * T_i;
        T_all(:, :, i) = T;
    end
    T_end = T;
end

% ==================== 几何雅可比矩阵 ====================

function J = jacob_cr5_geo(q, T_all)
% JACOB_CR5_GEO  几何雅可比矩阵 (基坐标系)
%   J = [Jv; Jw]  6×6
%   Jv: 线速度雅可比  Jw: 角速度雅可比
%   q:     6×1 关节角度 (弧度)
%   T_all: 4×4×6 各连杆变换 (可选, 传入可避免重复计算 FK)
    if nargin < 2
        [~, T_all] = fkine_cr5(q);
    end

    J = zeros(6, 6);
    p_tool = T_all(1:3, 4, 6);  % 法兰坐标系原点位置

    for i = 1:6
        if i == 1
            T_im1 = eye(4);
        else
            T_im1 = T_all(:, :, i - 1);
        end
        z_i = T_im1(1:3, 3);          % 关节轴方向 (基坐标系)
        p_i = T_im1(1:3, 4);          % 关节原点位置
        r   = p_tool - p_i;            % 关节 → 法兰向量

        J(1:3, i) = cross(z_i, r);   % 线速度 Jacobian 列
        J(4:6, i) = z_i;             % 角速度 Jacobian 列
    end
end

% ==================== 通用工具函数 ====================

function R = dobot_rpy_deg_to_rotm(rpy_deg)
% DOBOT_RPY_DEG_TO_ROTM  Dobot RPY 角 (度) → 旋转矩阵
%   rpy_deg: [Rx, Ry, Rz] 绕固定轴 X-Y-Z 的旋转角 (度)
%   R = Rz(Rz) * Ry(Ry) * Rx(Rx)
    rx = rpy_deg(1) * pi / 180;
    ry = rpy_deg(2) * pi / 180;
    rz = rpy_deg(3) * pi / 180;
    cx = cos(rx); sx = sin(rx);
    cy = cos(ry); sy = sin(ry);
    cz = cos(rz); sz = sin(rz);
    R = [cy*cz,  cz*sx*sy - cx*sz,  cx*cz*sy + sx*sz;
         cy*sz,  cx*cz + sx*sy*sz, -cz*sx + cx*sy*sz;
         -sy,    cy*sx,             cx*cy];
end

function y = smooth_deadzone(x, threshold)
% SMOOTH_DEADZONE  连续死区，越过阈值后从零开始增长，避免速度突跳。
    y = sign(x) .* max(abs(x) - threshold, 0);
end

function y = smooth_vector_deadzone(x, threshold)
% SMOOTH_VECTOR_DEADZONE 按合力范数设置连续死区，并保持原始施力方向。
    magnitude = norm(x);
    if magnitude <= threshold || magnitude <= eps
        y = zeros(size(x));
    else
        y = x * ((magnitude - threshold) / magnitude);
    end
end

function validate_sensor_mount_rotation(R_sensor_to_flange, ...
        gravity_sensor_zero, gravity_flange_zero)
% 验证安装旋转是合法右手旋转，并满足零姿态传感器 -Y -> 法兰 +Y。
    if norm(R_sensor_to_flange' * R_sensor_to_flange - eye(3), 'fro') > 1e-12 || ...
            abs(det(R_sensor_to_flange) - 1) > 1e-12
        error('R_SENSOR_TO_FLANGE 必须是右手正交旋转矩阵。');
    end

    mapped = R_sensor_to_flange * gravity_sensor_zero;
    if norm(mapped - gravity_flange_zero) > 1e-12
        error(['传感器安装矩阵错误：零姿态下必须将传感器 -Y 映射到法兰 +Y。' ...
            '当前映射结果为 [%.3f %.3f %.3f]。'], mapped(1), mapped(2), mapped(3));
    end
end

function calibration = load_local_numerical_jacobian(calibrationFile)
% LOAD_LOCAL_NUMERICAL_JACOBIAN 加载并验证控制器反馈标定的局部雅可比。
    if ~isfile(calibrationFile)
        error(['找不到局部数值雅可比文件: %s\n' ...
            '请先运行 calibrate_cr5_numerical_jacobian。'], calibrationFile);
    end

    data = load(calibrationFile, 'result');
    if ~isfield(data, 'result') || ...
            ~isfield(data.result, 'JNumericBase') || ...
            ~isfield(data.result, 'q0Deg')
        error('数值雅可比文件格式无效: %s', calibrationFile);
    end

    calibration = data.result;
    calibration.JNumericBase = double(calibration.JNumericBase);
    calibration.q0Deg = double(calibration.q0Deg(:)');
    if ~isequal(size(calibration.JNumericBase), [6, 6]) || ...
            numel(calibration.q0Deg) ~= 6 || ...
            any(~isfinite(calibration.JNumericBase), 'all') || ...
            any(~isfinite(calibration.q0Deg))
        error('数值雅可比矩阵或中心关节姿态无效: %s', calibrationFile);
    end

    if ~isfield(calibration, 'positionRmseMm')
        calibration.positionRmseMm = NaN;
    end
end

function tf = feature_enabled(value)
% FEATURE_ENABLED  保留运行时开关，避免静态分析把可选分支折叠掉。
    tf = logical(value);
end

function y = limit_vector_norm(x, max_norm)
% 对整个向量限幅，避免多轴同时饱和时合速度超过设定值。
    n = norm(x);
    if n > max_norm && n > 0
        y = x * (max_norm / n);
    else
        y = x;
    end
end

function y = saturate(x, lim)
% SATURATE  对称限幅 y = max(-lim, min(lim, x))
%   MATLAB 的 max/min 自动支持标量/向量 lim 的广播
    y = max(-lim, min(lim, x));
end

function diag = init_drag_diagnostics(enabled, windowSec, updateHz, nominalDt)
% INIT_DRAG_DIAGNOSTICS 创建低开销实时诊断面板和固定长度环形缓冲区。
    diag.enabled = logical(enabled);
    diag.plotActive = diag.enabled;
    diag.windowSec = windowSec;
    diag.updatePeriod = 1 / max(updateHz, 0.5);
    diag.lastDrawTime = -inf;
    diag.capacity = max(200, ceil(windowSec / max(nominalDt, 0.005)) + 10);
    diag.writeIndex = 0;
    diag.count = 0;

    diag.time = nan(1, diag.capacity);
    diag.qPosition = nan(6, diag.capacity);
    diag.qTargetPosition = nan(6, diag.capacity);
    diag.qVelocityActual = nan(6, diag.capacity);
    diag.qVelocityCommand = nan(6, diag.capacity);
    diag.forceMeasured = nan(3, diag.capacity);
    diag.forceModel = nan(3, diag.capacity);
    diag.forcePreDeadzone = nan(3, diag.capacity);
    diag.forcePostDeadzone = nan(3, diag.capacity);
    diag.momentMeasured = nan(3, diag.capacity);
    diag.momentModel = nan(3, diag.capacity);
    diag.momentPreDeadzone = nan(3, diag.capacity);
    diag.momentPostDeadzone = nan(3, diag.capacity);
    diag.forceSensorRaw = nan(3, diag.capacity);    % 传感器原始力 (传感器系)
    diag.momentSensorRaw = nan(3, diag.capacity);   % 传感器原始力矩 (传感器系)
    diag.forceNorms = nan(4, diag.capacity);
    diag.tcpVelocity = nan(3, diag.capacity);
    diag.velRaw = nan(3, diag.capacity);
    diag.timing = nan(4, diag.capacity);
    diag.condition = nan(4, diag.capacity);

    diag.figure = gobjects(0);
    diag.lines = struct();
    diag.title = gobjects(0);
    diag.axes = gobjects(0);
    if ~diag.enabled
        return;
    end

    try
        colors6 = lines(6);
        colors3 = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19];
        diag.figure = figure('Name', '拖动示教实时抖动诊断', ...
            'NumberTitle', 'off', 'Color', 'w', ...
            'Position', [40 40 1500 900]);
        layout = tiledlayout(diag.figure, 4, 2, ...
            'TileSpacing', 'compact', 'Padding', 'compact');
        diag.title = sgtitle(layout, '等待导纳控制解锁...', 'FontWeight', 'bold');

    ax = nexttile(layout);
    hold(ax, 'on'); grid(ax, 'on');
    diag.lines.qPosition = gobjects(6, 1);
    diag.lines.qTargetPosition = gobjects(6, 1);
    for j = 1:6
        diag.lines.qPosition(j) = plot(ax, nan, nan, ...
            'Color', colors6(j, :), 'LineWidth', 1.1, ...
            'DisplayName', sprintf('J%d', j));
        diag.lines.qTargetPosition(j) = plot(ax, nan, nan, '--', ...
            'Color', colors6(j, :), 'LineWidth', 0.9, ...
            'HandleVisibility', 'off');
    end
    title(ax, '关节位置：实线=反馈，虚线=目标'); ylabel(ax, 'deg');
    legend(ax, 'Location', 'eastoutside', 'NumColumns', 2);

    ax = nexttile(layout);
    hold(ax, 'on'); grid(ax, 'on');
    diag.lines.qVelocityActual = gobjects(6, 1);
    diag.lines.qVelocityCommand = gobjects(6, 1);
    for j = 1:6
        diag.lines.qVelocityActual(j) = plot(ax, nan, nan, ...
            'Color', colors6(j, :), 'LineWidth', 1.2, ...
            'DisplayName', sprintf('J%d actual', j));
        diag.lines.qVelocityCommand(j) = plot(ax, nan, nan, '--', ...
            'Color', colors6(j, :), 'LineWidth', 0.8, ...
            'HandleVisibility', 'off');
    end
    title(ax, '关节速度：实线=反馈，虚线=指令'); ylabel(ax, 'deg/s');
    legend(ax, 'Location', 'eastoutside', 'NumColumns', 2);

    ax = nexttile(layout);
    hold(ax, 'on'); grid(ax, 'on');
    diag.lines.forceMeasured = gobjects(3, 1);
    diag.lines.forcePre = gobjects(3, 1);
    diag.lines.forcePost = gobjects(3, 1);
    forceNames = {'Fx', 'Fy', 'Fz'};
    for j = 1:3
        diag.lines.forceMeasured(j) = plot(ax, nan, nan, ':', ...
            'Color', colors3(j, :), 'LineWidth', 0.8, ...
            'HandleVisibility', 'off');
        diag.lines.forcePre(j) = plot(ax, nan, nan, ...
            'Color', colors3(j, :), 'LineWidth', 1.2, ...
            'DisplayName', forceNames{j});
        diag.lines.forcePost(j) = plot(ax, nan, nan, '--', ...
            'Color', colors3(j, :), 'LineWidth', 0.8, ...
            'HandleVisibility', 'off');
    end
    yline(ax, 0, ':', 'Color', [0.45 0.45 0.45]);
    title(ax, '力(法兰): 点线=滤波, 实线=死前, 虚线=使用'); ylabel(ax, 'N');
    legend(ax, 'Location', 'eastoutside');

    ax = nexttile(layout);
    hold(ax, 'on'); grid(ax, 'on');
    diag.lines.momentMeasured = gobjects(3, 1);
    diag.lines.momentPre = gobjects(3, 1);
    diag.lines.momentPost = gobjects(3, 1);
    momentNames = {'Mx', 'My', 'Mz'};
    for j = 1:3
        diag.lines.momentMeasured(j) = plot(ax, nan, nan, ':', ...
            'Color', colors3(j, :), 'LineWidth', 0.8, ...
            'HandleVisibility', 'off');
        diag.lines.momentPre(j) = plot(ax, nan, nan, ...
            'Color', colors3(j, :), 'LineWidth', 1.2, ...
            'DisplayName', momentNames{j});
        diag.lines.momentPost(j) = plot(ax, nan, nan, '--', ...
            'Color', colors3(j, :), 'LineWidth', 0.8, ...
            'HandleVisibility', 'off');
    end
    yline(ax, 0, ':', 'Color', [0.45 0.45 0.45]);
    title(ax, '外力矩：实线=死区前，虚线=控制使用'); ylabel(ax, 'Nm');
    legend(ax, 'Location', 'eastoutside');

    ax = nexttile(layout);
    hold(ax, 'on'); grid(ax, 'on');
    normColors = [0.12 0.47 0.71; 1.00 0.50 0.05; 0.84 0.15 0.16; 0.17 0.63 0.17];
    normNames = {'|F measured|', '|F model|', '|F residual|', '|F used|'};
    diag.lines.forceNorms = gobjects(4, 1);
    for j = 1:4
        diag.lines.forceNorms(j) = plot(ax, nan, nan, ...
            'Color', normColors(j, :), 'LineWidth', 1.2, ...
            'DisplayName', normNames{j});
    end
    title(ax, '力补偿分解'); ylabel(ax, 'N');
    legend(ax, 'Location', 'eastoutside');

    ax = nexttile(layout);
    hold(ax, 'on'); grid(ax, 'on');
    diag.lines.tcpVelocity = gobjects(3, 1);
    velocityNames = {'Vx', 'Vy', 'Vz'};
    for j = 1:3
        diag.lines.tcpVelocity(j) = plot(ax, nan, nan, ...
            'Color', colors3(j, :), 'LineWidth', 1.2, ...
            'DisplayName', velocityNames{j});
    end
    yline(ax, 0, ':', 'Color', [0.45 0.45 0.45]);
    title(ax, '法兰系平移速度指令'); ylabel(ax, 'mm/s');
    legend(ax, 'Location', 'eastoutside');

    ax = nexttile(layout);
    hold(ax, 'on'); grid(ax, 'on');
    diag.lines.velRaw = gobjects(3, 1);
    diag.lines.velFilt = gobjects(3, 1);
    velNames = {'Vx', 'Vy', 'Vz'};
    for j = 1:3
        diag.lines.velRaw(j) = plot(ax, nan, nan, ':', ...
            'Color', colors3(j, :), 'LineWidth', 0.8, ...
            'HandleVisibility', 'off');
        diag.lines.velFilt(j) = plot(ax, nan, nan, ...
            'Color', colors3(j, :), 'LineWidth', 1.2, ...
            'DisplayName', velNames{j});
    end
    yline(ax, 0, ':', 'Color', [0.45 0.45 0.45]);
    title(ax, '速度指令(法兰系): 点线=Vraw, 实线=Vfilt'); ylabel(ax, 'mm/s'); xlabel(ax, 'time (s)');
    legend(ax, 'Location', 'eastoutside');

    ax = nexttile(layout);
    hold(ax, 'on'); grid(ax, 'on');
    yyaxis(ax, 'left');
    diag.lines.forceSensor = gobjects(3, 1);
    sensorForceNames = {'Fx_s', 'Fy_s', 'Fz_s'};
    for j = 1:3
        diag.lines.forceSensor(j) = plot(ax, nan, nan, ...
            'Color', colors3(j, :), 'LineWidth', 1.2, ...
            'DisplayName', sensorForceNames{j});
    end
    ylabel(ax, 'Force (N)');
    yyaxis(ax, 'right');
    diag.lines.momentSensor = gobjects(3, 1);
    sensorMomentNames = {'Mx_s', 'My_s', 'Mz_s'};
    for j = 1:3
        diag.lines.momentSensor(j) = plot(ax, nan, nan, '--', ...
            'Color', colors3(j, :), 'LineWidth', 1.0, ...
            'DisplayName', sensorMomentNames{j});
    end
    ylabel(ax, 'Moment (Nm)'); xlabel(ax, 'time (s)');
    title(ax, '传感器原始值(传感器系)');
    legend(ax, 'Location', 'eastoutside');

        diag.axes = findall(diag.figure, 'Type', 'axes');
    catch ME
        warning('drag_teaching:DiagnosticPlotDisabled', ...
            '实时诊断图创建失败，继续记录数据但不绘图: %s', ME.message);
        if isgraphics(diag.figure)
            close(diag.figure);
        end
        diag.figure = gobjects(0);
        diag.lines = struct();
        diag.title = gobjects(0);
        diag.axes = gobjects(0);
        diag.plotActive = false;
    end
end

function diag = update_drag_diagnostics(diag, timeSec, qPositionDeg, ...
        qTargetPositionDeg, qVelocityActualDegS, qVelocityCommandDegS, ...
        forceMeasured, forceModel, ...
        forcePreDeadzone, forcePostDeadzone, momentMeasured, momentModel, ...
        momentPreDeadzone, momentPostDeadzone, tcpVelocityMmS, velRawMmS, timingMs, ...
        sigmaMin, lambdaDls, forceSensorRaw, momentSensorRaw)
% UPDATE_DRAG_DIAGNOSTICS 记录每帧数据，并按较低频率刷新图形。
    if ~diag.enabled
        return;
    end

    diag.writeIndex = mod(diag.writeIndex, diag.capacity) + 1;
    k = diag.writeIndex;
    diag.count = min(diag.count + 1, diag.capacity);

    diag.time(k) = timeSec;
    diag.qPosition(:, k) = qPositionDeg(:);
    diag.qTargetPosition(:, k) = qTargetPositionDeg(:);
    diag.qVelocityActual(:, k) = qVelocityActualDegS(:);
    diag.qVelocityCommand(:, k) = qVelocityCommandDegS(:);
    diag.forceMeasured(:, k) = forceMeasured(:);
    diag.forceModel(:, k) = forceModel(:);
    diag.forcePreDeadzone(:, k) = forcePreDeadzone(:);
    diag.forcePostDeadzone(:, k) = forcePostDeadzone(:);
    diag.momentMeasured(:, k) = momentMeasured(:);
    diag.momentModel(:, k) = momentModel(:);
    diag.momentPreDeadzone(:, k) = momentPreDeadzone(:);
    diag.momentPostDeadzone(:, k) = momentPostDeadzone(:);
    diag.forceSensorRaw(:, k) = forceSensorRaw(:);
    diag.momentSensorRaw(:, k) = momentSensorRaw(:);
    diag.forceNorms(:, k) = [norm(forceMeasured); norm(forceModel); ...
        norm(forcePreDeadzone); norm(forcePostDeadzone)];
    diag.tcpVelocity(:, k) = tcpVelocityMmS(:);
    diag.velRaw(:, k) = velRawMmS(:);
    diag.timing(:, k) = timingMs(:);
    diag.condition(:, k) = [sigmaMin; lambdaDls; ...
        max(abs(qVelocityActualDegS)); max(abs(qVelocityCommandDegS))];

    if ~diag.plotActive
        return;
    end
    if ~isgraphics(diag.figure)
        diag.plotActive = false;
        return;
    end
    if timeSec - diag.lastDrawTime < diag.updatePeriod
        return;
    end
    diag.lastDrawTime = timeSec;

    indices = diagnostic_indices(diag);
    t = diag.time(indices);
    set_line_group(diag.lines.qPosition, t, diag.qPosition(:, indices));
    set_line_group(diag.lines.qTargetPosition, t, diag.qTargetPosition(:, indices));
    set_line_group(diag.lines.qVelocityActual, t, diag.qVelocityActual(:, indices));
    set_line_group(diag.lines.qVelocityCommand, t, diag.qVelocityCommand(:, indices));
    set_line_group(diag.lines.forceMeasured, t, diag.forceMeasured(:, indices));
    set_line_group(diag.lines.forcePre, t, diag.forcePreDeadzone(:, indices));
    set_line_group(diag.lines.forcePost, t, diag.forcePostDeadzone(:, indices));
    set_line_group(diag.lines.momentMeasured, t, diag.momentMeasured(:, indices));
    set_line_group(diag.lines.momentPre, t, diag.momentPreDeadzone(:, indices));
    set_line_group(diag.lines.momentPost, t, diag.momentPostDeadzone(:, indices));
    set_line_group(diag.lines.forceNorms, t, diag.forceNorms(:, indices));
    set_line_group(diag.lines.tcpVelocity, t, diag.tcpVelocity(:, indices));
    set_line_group(diag.lines.velRaw, t, diag.velRaw(:, indices));
    set_line_group(diag.lines.velFilt, t, diag.tcpVelocity(:, indices));
    set_line_group(diag.lines.forceSensor, t, diag.forceSensorRaw(:, indices));
    set_line_group(diag.lines.momentSensor, t, diag.momentSensorRaw(:, indices));

    xStart = max(0, timeSec - diag.windowSec);
    xEnd = max(diag.windowSec, timeSec);
    for iAxis = 1:numel(diag.axes)
        ax = diag.axes(iAxis);
        if isgraphics(ax)
            xlim(ax, [xStart, xEnd]);
        end
    end
    set(diag.title, 'String', sprintf( ...
        '实时抖动诊断 | |F residual|=%.2f N | max actual qdot=%.2f deg/s | %.1f Hz', ...
        norm(forcePreDeadzone), max(abs(qVelocityActualDegS)), ...
        1000 / max(timingMs(4), 0.1)));
    drawnow limitrate nocallbacks;
end

function indices = diagnostic_indices(diag)
% DIAGNOSTIC_INDICES 按时间顺序返回环形缓冲区索引。
    if diag.count < diag.capacity
        indices = 1:diag.count;
    else
        indices = [diag.writeIndex + 1:diag.capacity, 1:diag.writeIndex];
    end
end

function set_line_group(handles, xData, yData)
% SET_LINE_GROUP 批量更新同一坐标轴上的多条曲线。
    for j = 1:numel(handles)
        if isgraphics(handles(j))
            set(handles(j), 'XData', xData, 'YData', yData(j, :));
        end
    end
end

function save_drag_diagnostics(diag, outputFile)
% SAVE_DRAG_DIAGNOSTICS 保存按时间排序的诊断数据，供离线分析。
    if ~isstruct(diag) || diag.count < 1
        fprintf('[诊断] 没有可保存的实时诊断数据。\n');
        return;
    end

    indices = diagnostic_indices(diag);
    diagnostic = struct();
    diagnostic.time_s = diag.time(indices)';
    diagnostic.joint_position_deg = diag.qPosition(:, indices)';
    diagnostic.joint_target_position_deg = diag.qTargetPosition(:, indices)';
    diagnostic.joint_velocity_actual_deg_s = diag.qVelocityActual(:, indices)';
    diagnostic.joint_velocity_command_deg_s = diag.qVelocityCommand(:, indices)';
    diagnostic.force_measured_N = diag.forceMeasured(:, indices)';
    diagnostic.force_model_N = diag.forceModel(:, indices)';
    diagnostic.force_pre_deadzone_N = diag.forcePreDeadzone(:, indices)';
    diagnostic.force_used_N = diag.forcePostDeadzone(:, indices)';
    diagnostic.moment_measured_Nm = diag.momentMeasured(:, indices)';
    diagnostic.moment_model_Nm = diag.momentModel(:, indices)';
    diagnostic.moment_pre_deadzone_Nm = diag.momentPreDeadzone(:, indices)';
    diagnostic.moment_used_Nm = diag.momentPostDeadzone(:, indices)';
    diagnostic.force_norms_N = diag.forceNorms(:, indices)';
    diagnostic.tcp_velocity_command_mm_s = diag.tcpVelocity(:, indices)';
    diagnostic.timing_ms = diag.timing(:, indices)';
    diagnostic.condition = diag.condition(:, indices)';
    diagnostic.columns = struct( ...
        'force_norms_N', {{'measured', 'model', 'residual', 'used'}}, ...
        'timing_ms', {{'control_dt', 'modbus', 'servo', 'loop_total'}}, ...
        'condition', {{'sigma_min', 'lambda_dls', ...
            'max_actual_joint_speed_deg_s', 'max_command_joint_speed_deg_s'}});
    diagnostic.saved_at = datetime('now');

    try
        save(outputFile, 'diagnostic');
        fprintf('[诊断] 数据已保存到 %s，共 %d 帧。\n', outputFile, numel(indices));
    catch ME
        warning('drag_teaching:DiagnosticSaveFailed', ...
            '保存诊断数据失败: %s', ME.message);
    end
end

function safe_robot_shutdown(robot)
% 停止运动指令, 清空队列, 但保持使能(不掉电, 伺服保持位置)
    try
        robot.StopMove();                % 清空运动队列
    catch
    end
    try
        q_stop_deg = robot.JointAngles;
        robot.ServoJ(q_stop_deg, 0.3);    % 保持当前位置
        pause(0.3);
    catch
    end
    try
        robot.Disable();                  % 下使能
    catch
    end
    try
        robot.Disconnect();               % 断开连接
    catch
    end
end
