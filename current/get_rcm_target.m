%GET_RCM_TARGET 独立RCM点坐标采集程序
%
% 来源：dobot_HW_control2/get_rcm_target.m
% 已适配到当前ScopeGuide项目的robot目录和Dobot RPY变换函数。
%
% 使用方法：
%   1. 在控制柜/示教器上进入拖拽模式；
%   2. 运行本脚本；
%   3. 将内窥镜尖端拖到物理RCM点，并保持静止；
%   4. 回到MATLAB按Enter；
%   5. RCM点采集完成后，将机械臂拖到控制初始位置并再次按Enter；
%   6. 将输出的RCMTarget和InitialJointAngles复制到控制配置中。
%
% 安全说明：本脚本只建立反馈连接，不使能机器人，不发送运动指令。

clear;
clc;

%% 1. 用户配置

robotIP = '192.168.50.105';
dashboardPort = 29999;
movePort = 30003;
feedbackPort = 30004;
sampleCount = 50;
sampleInterval = 0.02;

% 必须与当前拖拽配置中的cfg.tool.TFlangeEndoscope完全一致。
% 变换后坐标系的原点必须位于手动放置到RCM点的内窥镜尖端。
toolTransform = [ ...
    1 0 0 0.003122985; ...
    0 1 0 0.118179746; ...
    0 0 1 0.428666535; ...
    0 0 0 1 ...
];

%% 2. 初始化程序路径与依赖

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'robot'));

if isempty(which('ZJFDobotCR5')) || ...
        isempty(which('scopeguide.geometry.dobotRpyToRotation'))
    error('缺少ZJFDobotCR5或ScopeGuide Dobot RPY变换函数。');
end

if norm(toolTransform(4, :) - [0, 0, 0, 1]) > 1e-10
    error('ToolTransform最后一行必须为[0 0 0 1]。');
end
if norm(toolTransform(1:3, 1:3)' * ...
        toolTransform(1:3, 1:3) - eye(3), 'fro') > 1e-6
    error('ToolTransform的旋转部分不是有效的正交矩阵。');
end

%% 3. 连接机器人反馈

fprintf('\n========== RCM点拖拽示教 ==========\n');
fprintf('机器人IP: %s\n', robotIP);
fprintf('本程序只读取反馈，不会使能或移动机器人。\n');
fprintf('当前报告TCP到内窥镜尖端的ToolTransform为：\n');
disp(toolTransform);

robot = ZJFDobotCR5(robotIP, dashboardPort, movePort, feedbackPort);
cleanupObj = onCleanup(@() disconnectRobot(robot));
robot.Connect();
waitForValidFeedback(robot, 5.0);

fprintf('当前机器人模式: %s\n', robot.RobotMode);
if robot.RobotMode ~= "BACKDRIVE"
    warning(['当前反馈模式不是BACKDRIVE。请通过控制柜或示教器进入拖拽模式，', ...
        '不要通过本脚本使能或移动机器人。']);
end

%% 4. 拖拽到RCM点后采样

fprintf('\n请将内窥镜尖端拖到物理RCM点，并保持机械臂完全静止。\n');
input('确认到位后按 Enter 开始采样...', 's');
pause(0.2);

tipSamples = zeros(sampleCount, 3);
tcpPoseSamples = zeros(sampleCount, 6);

for index = 1:sampleCount
    cartesianPose = double(robot.CartesianPose(:)');
    if numel(cartesianPose) ~= 6 || any(~isfinite(cartesianPose)) || ...
            norm(cartesianPose(1:3)) < eps
        error('第%d次采样未获得有效的TCP笛卡尔反馈。', index);
    end

    % 与当前ScopeGuide控制器保持相同的位姿解析：
    % XYZ单位为mm，Rx/Ry/Rz单位为degree，旋转约定为
    % R = Rz(rz) * Ry(ry) * Rx(rx)。
    baseToReportedTcp = eye(4);
    baseToReportedTcp(1:3, 1:3) = ...
        scopeguide.geometry.dobotRpyToRotation(cartesianPose(4:6));
    baseToReportedTcp(1:3, 4) = cartesianPose(1:3)' / 1000.0;
    baseToTip = baseToReportedTcp * toolTransform;

    tipSamples(index, :) = baseToTip(1:3, 4)';
    tcpPoseSamples(index, :) = cartesianPose;
    pause(sampleInterval);
end

%% 5. 输出RCMTarget与采样稳定性

rcmTarget = mean(tipSamples, 1);
sampleStdMm = std(tipSamples, 0, 1) * 1000.0;
sampleRangeMm = ...
    (max(tipSamples, [], 1) - min(tipSamples, [], 1)) * 1000.0;
meanTcpPose = mean(tcpPoseSamples, 1);

fprintf('\n========== RCM标定结果 ==========\n');
fprintf('平均TCP反馈（ToolVectorActual）[mm, deg]：\n');
fprintf('  [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', meanTcpPose);
fprintf('RCMTarget（机器人基坐标系，m）：\n');
fprintf('  [%.4f, %.4f, %.4f]\n', rcmTarget);
fprintf('RCMTarget（机器人基坐标系，mm）：\n');
fprintf('  [%.3f, %.3f, %.3f]\n', rcmTarget * 1000.0);
fprintf('采样标准差 [mm]：[%.4f, %.4f, %.4f]\n', sampleStdMm);
fprintf('采样峰峰值 [mm]：[%.4f, %.4f, %.4f]\n', sampleRangeMm);
fprintf('=================================\n\n');

if max(sampleStdMm) > 0.5
    warning(['采样期间尖端位置标准差超过0.5 mm。请确认机械臂已经静止，', ...
        '然后重新运行本脚本。']);
end

%% 6. 拖拽到控制初始位置后采集关节角

fprintf('\n请将机械臂拖到控制程序所需的初始位置，并保持完全静止。\n');
input('确认到位后按 Enter 开始采集初始关节角...', 's');
pause(0.2);

jointSamples = zeros(sampleCount, 6);
for index = 1:sampleCount
    jointAngles = double(robot.JointAngles(:)');
    if numel(jointAngles) ~= 6 || any(~isfinite(jointAngles))
        error('第%d次采样未获得有效的关节角反馈。', index);
    end

    jointSamples(index, :) = jointAngles;
    pause(sampleInterval);
end

initialJointAngles = mean(jointSamples, 1);
jointStdDeg = std(jointSamples, 0, 1);
jointRangeDeg = max(jointSamples, [], 1) - min(jointSamples, [], 1);

fprintf('\n========== 初始关节角采集结果 ==========\n');
fprintf('InitialJointAngles（deg）：\n');
fprintf('  [%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]\n', ...
    initialJointAngles);
fprintf(['采样标准差 [deg]：[%.4f, %.4f, %.4f, ' ...
    '%.4f, %.4f, %.4f]\n'], jointStdDeg);
fprintf(['采样峰峰值 [deg]：[%.4f, %.4f, %.4f, ' ...
    '%.4f, %.4f, %.4f]\n'], jointRangeDeg);
fprintf('========================================\n\n');

if max(jointStdDeg) > 0.05
    warning(['初始关节角采样期间标准差超过0.05 degree。', ...
        '请确认机械臂已经静止，然后重新采集。']);
end

%% 7. 输出可直接复制的控制参数

fprintf('复制到当前控制配置：\n');
fprintf("    'RCMTarget',      [%.4f, %.4f, %.4f], ...\n", ...
    rcmTarget);
fprintf("    'InitialJointAngles', " + ...
    "[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f], ...\n\n", ...
    initialJointAngles);

clear cleanupObj;

%% 本脚本使用的局部函数

function waitForValidFeedback(robot, timeoutSeconds)
    timer = tic;
    while toc(timer) < timeoutSeconds
        pose = double(robot.CartesianPose(:)');
        if numel(pose) == 6 && all(isfinite(pose)) && ...
                norm(pose(1:3)) > eps
            return;
        end
        pause(0.05);
    end
    error('连接成功，但在%.1f秒内没有收到有效机器人反馈。', ...
        timeoutSeconds);
end


function disconnectRobot(robot)
    if ~isempty(robot) && isvalid(robot) && robot.IsConnected
        try
            robot.Disconnect();
        catch ME
            warning('RCMCalibration:DisconnectFailed', ...
                '断开机器人连接失败：%s', ME.message);
        end
    end
end
