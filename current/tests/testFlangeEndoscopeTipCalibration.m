function tests = testFlangeEndoscopeTipCalibration
tests = functiontests(localfunctions);
end

function setupOnce(~)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
end

function testRecoversExactTranslationAndFixedPoint(testCase)
[position, rotation, expectedTranslation, expectedFixedPoint] = ...
    syntheticFixedTipData();
knownToolRotation = rotationZ(deg2rad(12));

result = scopeguide.calibration.fitFixedTipTranslation( ...
    position, rotation, ...
    RotationFlangeToEndoscope=knownToolRotation);

verifyTrue(testCase, result.Valid);
verifyEqual(testCase, result.DesignRank, 6);
verifyEqual(testCase, result.TipTranslationFlangeM, ...
    expectedTranslation, 'AbsTol', 1e-11);
verifyEqual(testCase, result.FixedTipPointBaseM, ...
    expectedFixedPoint, 'AbsTol', 1e-11);
verifyEqual(testCase, result.TFlangeEndoscope(1:3, 1:3), ...
    knownToolRotation, 'AbsTol', 1e-12);
verifyEqual(testCase, result.TFlangeEndoscope(1:3, 4), ...
    expectedTranslation, 'AbsTol', 1e-11);
verifyLessThan(testCase, result.MaximumResidualM, 1e-11);
verifyFalse(testCase, result.RotationEstimated);
verifyTrue(testCase, result.DoesNotEstimateToolRotation);
verifyTrue(testCase, result.DoesNotSendHardwareCommands);
end

function testSmallMeasurementNoiseRemainsValid(testCase)
[position, rotation, expectedTranslation] = syntheticFixedTipData();
noiseM = 5e-5 * [ ...
     0.2 -0.3  0.1; -0.4  0.1  0.2; 0.1  0.2 -0.3; ...
     0.3 -0.1  0.4; -0.2  0.4 -0.1; 0.1 -0.2  0.3; ...
    -0.3  0.2  0.1;  0.4  0.1 -0.2; 0.2 -0.4  0.3; ...
    -0.1  0.3 -0.4];

result = scopeguide.calibration.fitFixedTipTranslation( ...
    position + noiseM, rotation);

verifyTrue(testCase, result.Valid);
verifyLessThan(testCase, norm( ...
    result.TipTranslationFlangeM - expectedTranslation), 1e-4);
verifyLessThan(testCase, result.RmsResidualM, 1e-4);
end

function testRepeatedOrientationIsRejected(testCase)
poseCount = 10;
rotation = repmat(eye(3), 1, 1, poseCount);
translation = [-0.00284; 0.113051; 0.355046];
fixedPoint = [-0.2906; 0.6310; 0.1388];
position = repmat((fixedPoint - translation).', poseCount, 1);

result = scopeguide.calibration.fitFixedTipTranslation( ...
    position, rotation);

verifyFalse(testCase, result.Valid);
verifyLessThan(testCase, result.DesignRank, 6);
verifyTrue(testCase, any(result.FailureReasons == ...
    "DESIGN_MATRIX_RANK_DEFICIENT"));
verifyTrue(testCase, any(result.FailureReasons == ...
    "ORIENTATION_SPAN_TOO_SMALL"));
end

function testLargeFixedPointResidualIsRejected(testCase)
[position, rotation] = syntheticFixedTipData();
position(4, :) = position(4, :) + [0.010, 0, 0];

result = scopeguide.calibration.fitFixedTipTranslation( ...
    position, rotation);

verifyFalse(testCase, result.Valid);
verifyTrue(testCase, any(result.FailureReasons == ...
    "MAXIMUM_FIXED_POINT_RESIDUAL_TOO_LARGE"));
end

function [position, rotation, translation, fixedPoint] = ...
        syntheticFixedTipData()
anglesDeg = [ ...
     0   0   0; ...
    20   0   0; ...
   -20   0   0; ...
     0  25   0; ...
     0 -25   0; ...
     0   0  30; ...
     0   0 -30; ...
    18  15   0; ...
   -18   0  20; ...
     0 -18 -20];
poseCount = size(anglesDeg, 1);
rotation = zeros(3, 3, poseCount);
for index = 1:poseCount
    angles = deg2rad(anglesDeg(index, :));
    rotation(:, :, index) = rotationZ(angles(3)) * ...
        rotationY(angles(2)) * rotationX(angles(1));
end
translation = [-0.00284; 0.113051; 0.355046];
fixedPoint = [-0.2906; 0.6310; 0.1388];
position = zeros(poseCount, 3);
for index = 1:poseCount
    position(index, :) = (fixedPoint - ...
        rotation(:, :, index) * translation).';
end
end

function rotation = rotationX(angle)
rotation = [1, 0, 0; 0, cos(angle), -sin(angle); ...
    0, sin(angle), cos(angle)];
end

function rotation = rotationY(angle)
rotation = [cos(angle), 0, sin(angle); 0, 1, 0; ...
    -sin(angle), 0, cos(angle)];
end

function rotation = rotationZ(angle)
rotation = [cos(angle), -sin(angle), 0; ...
    sin(angle), cos(angle), 0; 0, 0, 1];
end
