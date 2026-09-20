function [stationary, diagnostics] = ...
        isRobotStationaryForBaseline(robotState, options)
%ISROBOTSTATIONARYFORBASELINE Relaxed gate for residual-baseline learning.
% These thresholds reject deliberate robot motion without allowing isolated
% low-level feedback jitter to restart the startup-zero timer.  They are not
% robot motion-safety thresholds.

arguments
    robotState (1, 1) struct
    options.MaximumJointSpeedDegSec (1, 1) double ...
        {mustBeFinite, mustBePositive} = 2.0
    options.MaximumTcpTranslationSpeedMmSec (1, 1) double ...
        {mustBeFinite, mustBePositive} = 3.0
    options.MaximumTcpRotationSpeedDegSec (1, 1) double ...
        {mustBeFinite, mustBePositive} = 2.0
end

required = {'IsValid', 'JointVelocityRadSec', 'ActualTcpTwistBase'};
for index = 1:numel(required)
    if ~isfield(robotState, required{index})
        error('scopeguide:force:InvalidRobotStationaryState', ...
            'robotState.%s is required.', required{index});
    end
end

jointVelocity = double(robotState.JointVelocityRadSec(:));
tcpTwist = double(robotState.ActualTcpTwistBase(:));
validArrays = numel(jointVelocity) == 6 && numel(tcpTwist) == 6 && ...
    all(isfinite(jointVelocity)) && all(isfinite(tcpTwist));

diagnostics = struct();
diagnostics.RobotStateValid = logical(robotState.IsValid) && validArrays;
diagnostics.MaximumJointSpeedDegSec = NaN;
diagnostics.MaximumTcpTranslationSpeedMmSec = NaN;
diagnostics.MaximumTcpRotationSpeedDegSec = NaN;
diagnostics.JointSpeedPassed = false;
diagnostics.TcpTranslationSpeedPassed = false;
diagnostics.TcpRotationSpeedPassed = false;

if diagnostics.RobotStateValid
    diagnostics.MaximumJointSpeedDegSec = ...
        rad2deg(max(abs(jointVelocity)));
    diagnostics.MaximumTcpTranslationSpeedMmSec = ...
        1000 * max(abs(tcpTwist(1:3)));
    diagnostics.MaximumTcpRotationSpeedDegSec = ...
        rad2deg(max(abs(tcpTwist(4:6))));
    diagnostics.JointSpeedPassed = ...
        diagnostics.MaximumJointSpeedDegSec <= ...
        options.MaximumJointSpeedDegSec;
    diagnostics.TcpTranslationSpeedPassed = ...
        diagnostics.MaximumTcpTranslationSpeedMmSec <= ...
        options.MaximumTcpTranslationSpeedMmSec;
    diagnostics.TcpRotationSpeedPassed = ...
        diagnostics.MaximumTcpRotationSpeedDegSec <= ...
        options.MaximumTcpRotationSpeedDegSec;
end

stationary = diagnostics.RobotStateValid && ...
    diagnostics.JointSpeedPassed && ...
    diagnostics.TcpTranslationSpeedPassed && ...
    diagnostics.TcpRotationSpeedPassed;
end
