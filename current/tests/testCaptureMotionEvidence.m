function tests = testCaptureMotionEvidence
tests = functiontests(localfunctions);
end

function setupOnce(~)
% runtests('tests') temporarily executes from the tests directory; add the
% package root explicitly so this suite behaves like every other suite.
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
end

function testQdActualSpikeAloneDoesNotConfirmMotion(testCase)
state = struct();
[state, ~] = update(state, snapshot(1, 0.000, 0, 0, 0));
[state, evidence] = update(state, snapshot(2, 0.008, 0, 1.59, 0.03));
verifyFalse(testCase, evidence.ConfirmedMoving);
verifyEqual(testCase, state.ConsecutiveMovingFrames, 0);
verifyEqual(testCase, evidence.ReportedJointSpeedDegSec, 1.59, ...
    'AbsTol', 1e-12);
end

function testActualJointPositionChangeConfirmsMotion(testCase)
state = struct();
[state, ~] = update(state, snapshot(1, 0.000, 0, 0, 0));
[state, first] = update(state, snapshot(2, 0.008, 0.02, 0, 0));
[state, second] = update(state, snapshot(3, 0.016, 0.04, 0, 0));
verifyTrue(testCase, first.ConfirmedMoving);
verifyTrue(testCase, second.ConfirmedMoving);
verifyEqual(testCase, state.ConsecutiveMovingFrames, 2);
end

function testRepeatedSequenceDoesNotIncrementCounter(testCase)
state = struct();
[state, ~] = update(state, snapshot(1, 0.000, 0, 0, 0));
[state, first] = update(state, snapshot(2, 0.008, 0.02, 0, 0));
[state, repeated] = update(state, snapshot(2, 0.008, 0.02, 2, 2));
verifyTrue(testCase, first.ConfirmedMoving);
verifyFalse(testCase, repeated.IsNewFeedback);
verifyEqual(testCase, state.ConsecutiveMovingFrames, 1);
end

function [state, evidence] = update(state, value)
[state, evidence] = scopeguide.diagnostics.updateCaptureMotionEvidence( ...
    state, value);
end

function value = snapshot(sequence, timeSec, joint1Deg, qd1, tcpSpeed)
value = struct();
value.feedbackSequence = uint64(sequence);
value.hostMonotonicSec = timeSec;
value.jointAnglesDeg = [joint1Deg zeros(1, 5)];
value.cartesianPose = zeros(1, 6);
value.actualQuaternionWxyz = [1 0 0 0];
value.actualJointSpeedsDegSec = [qd1 zeros(1, 5)];
value.actualTCPSpeed = [tcpSpeed zeros(1, 5)];
end
