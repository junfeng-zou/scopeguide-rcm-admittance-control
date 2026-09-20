function tests = testForceResidualModelComparison
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'force_calibration'));
end

function testBiasOnlyRecoversConstantShift(testCase)
[source, wrench, quaternion, shift] = syntheticDataset();
comparison = compareHexHResidualModels( ...
    wrench, quaternion, source, [1 2 3 5 6 8], [4 7 9]);

verifyEqual(testCase, comparison.BiasCorrectionSensor, shift, ...
    'AbsTol', 1e-10);
verifyLessThan(testCase, ...
    comparison.Models.BiasOnly.Validation.ForceVectorRmseN, 1e-9);
verifyLessThan(testCase, ...
    comparison.Models.BiasOnly.Validation.MomentVectorRmseNm, 1e-9);
verifyGreaterThan(testCase, ...
    comparison.Models.Original.Validation.ForceVectorRmseN, 5);
end

function testSplitMustBeIndependent(testCase)
[source, wrench, quaternion] = syntheticDataset();
verifyError(testCase, @() compareHexHResidualModels( ...
    wrench, quaternion, source, 1:6, [6 7 8]), ...
    'scopeguide:residualModels:InvalidSplit');
end

function [source, wrench, quaternion, shift] = syntheticDataset()
source = struct();
source.biasSensor = [-0.5; 2.0; -4.0; 0.2; 0.1; -0.1];
source.massKg = 1.2;
source.comSensorM = [-0.004; -0.06; 0.05];
source.gravityBaseMps2 = [0; 0; -9.80665];
source.gravityForceSign = 1;
source.rotationToolFromSensor = diag([-1 -1 1]);
source.quaternionConvention = 'base_from_tool';
source.valid = true;

angles = [0 0 0; 20 0 0; 20 20 0; 0 20 0; ...
    -20 20 0; -20 0 0; -20 -20 0; 0 -20 0; 20 -20 0];
quaternion = zeros(9, 4);
for index = 1:9
    quaternion(index, :) = rpyToQuaternionWxyz(deg2rad(angles(index, :)));
end
shift = [0.6; 0.1; 11.0; 0.03; -0.10; 0.04];
shifted = source;
shifted.biasSensor = source.biasSensor + shift;
wrench = zeros(9, 6);
for index = 1:9
    zeroRaw = zeros(6, 1);
    components = compensateHexHWrench( ...
        zeroRaw, quaternion(index, :), shifted);
    wrench(index, :) = ...
        (shifted.biasSensor + components.gravitySensor).';
end
end

function quaternion = rpyToQuaternionWxyz(rpy)
rx = rpy(1); ry = rpy(2); rz = rpy(3);
cx = cos(rx/2); sx = sin(rx/2);
cy = cos(ry/2); sy = sin(ry/2);
cz = cos(rz/2); sz = sin(rz/2);
quaternion = [ ...
    cx*cy*cz + sx*sy*sz, ...
    sx*cy*cz - cx*sy*sz, ...
    cx*sy*cz + sx*cy*sz, ...
    cx*cy*sz - sx*sy*cz];
end
