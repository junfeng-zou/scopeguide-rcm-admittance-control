function [calibration, report] = fitHexHToolLoadCalibration( ...
        poseWrenchSensor, poseQuaternionWxyz, varargin)
%FITHEXHTOOLLOADCALIBRATION Fit HEX-H bias, rigid mass, and sensor-frame COM.
%
% Each input row is the contact-free static mean/median at one distinct pose.
% poseWrenchSensor columns are [Fx,Fy,Fz,Tx,Ty,Tz] in N and N*m.
% poseQuaternionWxyz is Dobot ActualQuaternion [qw,qx,qy,qz].
%
% The model is:
%   wrenchRaw = bias + [m*R_sensor_base*g; rCOM x Fgravity]
%
% This function never connects to or commands hardware.

parser = inputParser;
addParameter(parser, 'RToolFromSensor', diag([-1, -1, 1]), ...
    @(x) isnumeric(x) && isequal(size(x), [3, 3]));
addParameter(parser, 'GravityBaseMps2', [0, 0, -9.80665], ...
    @(x) isnumeric(x) && numel(x) == 3 && all(isfinite(x)));
addParameter(parser, 'GravityForceSign', 'auto', ...
    @(x) (ischar(x) || isstring(x)) || ...
    (isnumeric(x) && isscalar(x) && any(x == [-1, 1])));
addParameter(parser, 'QuaternionConvention', 'base_from_tool', ...
    @(x) ischar(x) || isstring(x));
addParameter(parser, 'MaximumConditionNumber', 1e6, ...
    @(x) isnumeric(x) && isscalar(x) && x > 1);
addParameter(parser, 'ComputeLeaveOneOut', true, ...
    @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
parse(parser, varargin{:});
opts = parser.Results;

wrenches = double(poseWrenchSensor);
quaternions = double(poseQuaternionWxyz);
if size(wrenches, 2) ~= 6 || size(quaternions, 2) ~= 4 || ...
        size(wrenches, 1) ~= size(quaternions, 1)
    error('calibration:InvalidInputShape', ...
        'Expected matching N-by-6 wrench and N-by-4 quaternion arrays.');
end
if size(wrenches, 1) < 6 || any(~isfinite(wrenches), 'all') || ...
        any(~isfinite(quaternions), 'all')
    error('calibration:InsufficientOrInvalidData', ...
        'At least six finite static poses are required.');
end

rotationToolFromSensor = validateRotation(opts.RToolFromSensor, ...
    'RToolFromSensor');
rotationSensorFromTool = rotationToolFromSensor.';
gravityBase = double(opts.GravityBaseMps2(:));
rotationSensorFromBase = rotationsFromQuaternions( ...
    quaternions, rotationSensorFromTool, opts.QuaternionConvention);

[calibration, report] = fitCore(wrenches, rotationSensorFromBase, ...
    gravityBase, opts.GravityForceSign, opts.MaximumConditionNumber);
calibration.rotationToolFromSensor = rotationToolFromSensor;
calibration.quaternionConvention = char(opts.QuaternionConvention);
calibration.wrenchOrder = {'Fx','Fy','Fz','Tx','Ty','Tz'};
calibration.wrenchUnits = {'N','N','N','N*m','N*m','N*m'};
calibration.comUnit = 'm';
calibration.valid = true;

poseCount = size(wrenches, 1);
report.leaveOneOutResidual = NaN(poseCount, 6);
if logical(opts.ComputeLeaveOneOut) && poseCount >= 7
    for holdout = 1:poseCount
        training = true(poseCount, 1);
        training(holdout) = false;
        [foldCalibration, ~] = fitCore( ...
            wrenches(training, :), rotationSensorFromBase(:, :, training), ...
            gravityBase, opts.GravityForceSign, opts.MaximumConditionNumber);
        prediction = predictOne(foldCalibration, ...
            rotationSensorFromBase(:, :, holdout), gravityBase);
        report.leaveOneOutResidual(holdout, :) = ...
            wrenches(holdout, :) - prediction.';
    end
    report.leaveOneOutForceRmseN = sqrt(mean( ...
        report.leaveOneOutResidual(:, 1:3).^2, 'all'));
    report.leaveOneOutTorqueRmseNm = sqrt(mean( ...
        report.leaveOneOutResidual(:, 4:6).^2, 'all'));
else
    report.leaveOneOutForceRmseN = NaN;
    report.leaveOneOutTorqueRmseNm = NaN;
end
end

function [calibration, report] = fitCore(wrenches, rotationsSensorFromBase, ...
        gravityBase, requestedSign, maximumConditionNumber)
poseCount = size(wrenches, 1);
gravityDirectionSensor = zeros(poseCount, 3);
for index = 1:poseCount
    gravityDirectionSensor(index, :) = ...
        (rotationsSensorFromBase(:, :, index) * gravityBase).';
end

if ischar(requestedSign) || isstring(requestedSign)
    if ~strcmpi(char(requestedSign), 'auto')
        error('calibration:InvalidGravitySign', ...
            'GravityForceSign must be auto, +1, or -1.');
    end
    trialSign = 1;
else
    trialSign = double(requestedSign);
end

[forceBias, signedMass, forceCondition] = fitForce( ...
    wrenches(:, 1:3), trialSign * gravityDirectionSensor, ...
    maximumConditionNumber);
if ischar(requestedSign) || isstring(requestedSign)
    gravitySign = sign(signedMass);
    if gravitySign == 0
        error('calibration:ZeroMass', ...
            'Estimated mass is zero; verify the static pose data.');
    end
    massKg = abs(signedMass);
else
    gravitySign = trialSign;
    massKg = signedMass;
    if massKg <= 0
        error('calibration:NegativeMass', ...
            ['Estimated mass is not positive. Reverse GravityForceSign ' ...
             'or verify the quaternion/frame convention.']);
    end
end

gravityForces = gravitySign * massKg * gravityDirectionSensor;
torqueDesign = zeros(3 * poseCount, 6);
torqueTarget = reshape(wrenches(:, 4:6).', [], 1);
for index = 1:poseCount
    rows = (3*index-2):(3*index);
    torqueDesign(rows, 1:3) = eye(3);
    torqueDesign(rows, 4:6) = -skew3(gravityForces(index, :));
end
torqueCondition = cond(torqueDesign);
if rank(torqueDesign) < 6 || ~isfinite(torqueCondition) || ...
        torqueCondition > maximumConditionNumber
    error('calibration:UnobservableTorqueModel', ...
        ['Static poses do not make torque bias and COM observable. ' ...
         'Torque design condition number: %.3g.'], torqueCondition);
end
torqueParameters = torqueDesign \ torqueTarget;

calibration = struct();
calibration.biasSensor = [forceBias(:); torqueParameters(1:3)].';
calibration.massKg = massKg;
calibration.comSensorM = torqueParameters(4:6).';
calibration.gravityBaseMps2 = gravityBase.';
calibration.gravityForceSign = gravitySign;

predicted = zeros(poseCount, 6);
gravityWrench = zeros(poseCount, 6);
for index = 1:poseCount
    gravityForce = gravityForces(index, :).';
    gravityTorque = cross(calibration.comSensorM.', gravityForce);
    gravityWrench(index, :) = [gravityForce; gravityTorque].';
    predicted(index, :) = calibration.biasSensor + gravityWrench(index, :);
end
residual = wrenches - predicted;
report = struct();
report.poseCount = poseCount;
report.forceConditionNumber = forceCondition;
report.torqueConditionNumber = torqueCondition;
report.maximumConditionNumber = max(forceCondition, torqueCondition);
report.predictedWrenchSensor = predicted;
report.gravityWrenchSensor = gravityWrench;
report.compensatedResidual = residual;
report.forceResidualRmsN = sqrt(mean(residual(:, 1:3).^2, 1));
report.torqueResidualRmsNm = sqrt(mean(residual(:, 4:6).^2, 1));
end

function [forceBias, massKg, conditionNumber] = fitForce( ...
        measuredForce, gravityDirectionSensor, maximumConditionNumber)
poseCount = size(measuredForce, 1);
design = zeros(3 * poseCount, 4);
for index = 1:poseCount
    rows = (3*index-2):(3*index);
    design(rows, 1:3) = eye(3);
    design(rows, 4) = gravityDirectionSensor(index, :).';
end
conditionNumber = cond(design);
if rank(design) < 4 || ~isfinite(conditionNumber) || ...
        conditionNumber > maximumConditionNumber
    error('calibration:UnobservableForceModel', ...
        ['Static poses do not sufficiently vary the gravity direction. ' ...
         'Force design condition number: %.3g.'], conditionNumber);
end
parameters = design \ reshape(measuredForce.', [], 1);
forceBias = parameters(1:3);
massKg = parameters(4);
end

function rotations = rotationsFromQuaternions(quaternions, ...
        rotationSensorFromTool, convention)
poseCount = size(quaternions, 1);
rotations = zeros(3, 3, poseCount);
for index = 1:poseCount
    quaternionRotation = quaternionWxyzToRotation(quaternions(index, :));
    if strcmpi(char(convention), 'base_from_tool')
        rotationToolFromBase = quaternionRotation.';
    elseif strcmpi(char(convention), 'tool_from_base')
        rotationToolFromBase = quaternionRotation;
    else
        error('calibration:InvalidQuaternionConvention', ...
            ['QuaternionConvention must be base_from_tool or ' ...
             'tool_from_base.']);
    end
    rotations(:, :, index) = rotationSensorFromTool * rotationToolFromBase;
end
end

function prediction = predictOne(calibration, rotationSensorFromBase, gravityBase)
force = calibration.gravityForceSign * calibration.massKg * ...
    rotationSensorFromBase * gravityBase;
torque = cross(calibration.comSensorM.', force);
prediction = calibration.biasSensor.' + [force; torque];
end

function matrix = validateRotation(value, name)
matrix = double(value);
if any(~isfinite(matrix), 'all') || ...
        norm(matrix * matrix.' - eye(3), 'fro') > 1e-7 || ...
        abs(det(matrix) - 1) > 1e-7
    error('calibration:InvalidRotation', ...
        '%s must be a finite right-handed rotation matrix.', name);
end
end

function matrix = skew3(vector)
value = double(vector(:));
matrix = [0, -value(3), value(2); ...
          value(3), 0, -value(1); ...
          -value(2), value(1), 0];
end
