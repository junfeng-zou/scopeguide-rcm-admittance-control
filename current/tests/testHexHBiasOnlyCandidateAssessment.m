function tests = testHexHBiasOnlyCandidateAssessment
tests = functiontests(localfunctions);
end

function testConstantBiasPasses(testCase)
[wrench, quaternion, source] = syntheticData();
comparison = compareHexHResidualModels( ...
    wrench, quaternion, source, [1 2 3 5 6 8], [4 7 9]);
assessment = assessHexHBiasOnlyCandidate( ...
    comparison, zeros(1, 6));
verifyTrue(testCase, assessment.Passed);
verifyFalse(testCase, assessment.ActivationAuthorized);
verifyEqual(testCase, assessment.Status, ...
    "VALIDATED_CANDIDATE_NOT_ACTIVATED");
end

function testReturnDriftRejectsCandidate(testCase)
[wrench, quaternion, source] = syntheticData();
comparison = compareHexHResidualModels( ...
    wrench, quaternion, source, [1 2 3 5 6 8], [4 7 9]);
assessment = assessHexHBiasOnlyCandidate( ...
    comparison, [1 0 0 0 0 0]);
verifyFalse(testCase, assessment.Passed);
verifyTrue(testCase, ismember( ...
    "ReturnForceRepeatability", assessment.FailedGates));
end

function [wrench, quaternion, source] = syntheticData()
source = struct();
source.biasSensor = [0.2; -0.3; 0.4; 0.01; -0.02; 0.03];
source.massKg = 1.2;
source.comSensorM = [0.01; -0.03; 0.04];
source.gravityBaseMps2 = [0; 0; -9.80665];
source.gravityForceSign = 1;
source.rotationToolFromSensor = diag([-1 -1 1]);
source.quaternionConvention = 'base_from_tool';
source.valid = true;

rpy = [0 0 0; 20 0 0; 0 20 0; -20 0 0; 0 -20 0; ...
    15 15 0; -15 15 0; -15 -15 0; 15 -15 0];
quaternion = zeros(9, 4);
wrench = zeros(9, 6);
shift = [0.7 -0.2 0.5 0.02 0.01 -0.03];
for index = 1:9
    rotation = scopeguide.geometry.dobotRpyToRotation(rpy(index, :), pi/180);
    quaternion(index, :) = rotationToQuaternion(rotation);
    probe = source;
    probe.biasSensor = zeros(6, 1);
    gravity = compensateHexHWrench(zeros(6, 1), ...
        quaternion(index, :), probe).gravitySensor.';
    wrench(index, :) = source.biasSensor.' + gravity + shift;
end
end

function quaternion = rotationToQuaternion(rotation)
traceValue = trace(rotation);
scalar = sqrt(max(0, 1 + traceValue)) / 2;
quaternion = [scalar, ...
    (rotation(3, 2) - rotation(2, 3)) / (4 * scalar), ...
    (rotation(1, 3) - rotation(3, 1)) / (4 * scalar), ...
    (rotation(2, 1) - rotation(1, 2)) / (4 * scalar)];
quaternion = quaternion / norm(quaternion);
end
