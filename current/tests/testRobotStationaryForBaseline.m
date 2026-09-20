function tests = testRobotStationaryForBaseline
tests = functiontests(localfunctions);
end

function setupOnce(~)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
end

function testRelaxedDefaultsAcceptLowLevelJointFeedbackJitter(testCase)
state = stationaryRobotState();
state.JointVelocityRadSec(2) = deg2rad(1.59);

[stationary, diagnostics] = ...
    scopeguide.force.isRobotStationaryForBaseline(state);

verifyTrue(testCase, stationary);
verifyEqual(testCase, diagnostics.MaximumJointSpeedDegSec, 1.59, ...
    'AbsTol', 1e-12);
end

function testDeliberateMotionStillFailsGate(testCase)
state = stationaryRobotState();
state.ActualTcpTwistBase(1) = 3.1e-3;
verifyFalse(testCase, ...
    scopeguide.force.isRobotStationaryForBaseline(state));

state = stationaryRobotState();
state.ActualTcpTwistBase(5) = deg2rad(2.1);
verifyFalse(testCase, ...
    scopeguide.force.isRobotStationaryForBaseline(state));

state = stationaryRobotState();
state.JointVelocityRadSec(4) = deg2rad(2.1);
verifyFalse(testCase, ...
    scopeguide.force.isRobotStationaryForBaseline(state));
end

function testInvalidRobotStateFailsClosed(testCase)
state = stationaryRobotState();
state.IsValid = false;
verifyFalse(testCase, ...
    scopeguide.force.isRobotStationaryForBaseline(state));
end

function state = stationaryRobotState()
state = scopeguide.types.robotState();
state.IsValid = true;
state.JointVelocityRadSec = zeros(6, 1);
state.ActualTcpTwistBase = zeros(6, 1);
end
