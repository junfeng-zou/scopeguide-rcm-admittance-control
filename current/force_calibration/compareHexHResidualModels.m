function comparison = compareHexHResidualModels( ...
        poseWrenchSensor, poseQuaternionWxyz, sourceCalibration, ...
        trainingIndices, validationIndices, options)
%COMPAREHEXHRESIDUALMODELS Compare bias/load/joint correction hypotheses.
% This function is offline only and never communicates with hardware.

arguments
    poseWrenchSensor (:, 6) double
    poseQuaternionWxyz (:, 4) double
    sourceCalibration (1, 1) struct
    trainingIndices (1, :) double
    validationIndices (1, :) double
    options.MaximumConditionNumber (1, 1) double = 1e6
end

wrench = double(poseWrenchSensor);
quaternion = double(poseQuaternionWxyz);
poseCount = size(wrench, 1);
if size(quaternion, 1) ~= poseCount || poseCount < 9 || ...
        any(~isfinite(wrench), 'all') || any(~isfinite(quaternion), 'all')
    error('scopeguide:residualModels:InvalidData', ...
        'At least nine matching finite pose wrench/quaternion rows are required.');
end
training = validateIndices(trainingIndices, poseCount, 'trainingIndices');
validation = validateIndices(validationIndices, poseCount, 'validationIndices');
if numel(training) < 6 || numel(validation) < 3 || ...
        ~isempty(intersect(training, validation))
    error('scopeguide:residualModels:InvalidSplit', ...
        ['Use at least six training and three disjoint validation poses. ' ...
         'The split must be fixed before fitting.']);
end

source = normalizeCalibration(sourceCalibration);
sourceResidual = residualForCalibration(wrench, quaternion, source);

biasOnly = source;
biasCorrection = median(sourceResidual(training, :), 1).';
biasOnly.biasSensor = source.biasSensor + biasCorrection;

loadOnly = fitLoadWithFixedBias( ...
    wrench(training, :), quaternion(training, :), source, ...
    options.MaximumConditionNumber);

[joint, jointFitReport] = fitHexHToolLoadCalibration( ...
    wrench(training, :), quaternion(training, :), ...
    'RToolFromSensor', source.rotationToolFromSensor, ...
    'GravityBaseMps2', source.gravityBaseMps2, ...
    'GravityForceSign', source.gravityForceSign, ...
    'QuaternionConvention', source.quaternionConvention, ...
    'MaximumConditionNumber', options.MaximumConditionNumber, ...
    'ComputeLeaveOneOut', false);
joint = normalizeCalibration(joint);

comparison = struct();
comparison.PoseCount = poseCount;
comparison.TrainingIndices = training;
comparison.ValidationIndices = validation;
comparison.Models = struct();
comparison.Models.Original = buildModelReport( ...
    source, sourceResidual, training, validation);
comparison.Models.BiasOnly = buildModelReport( ...
    biasOnly, residualForCalibration(wrench, quaternion, biasOnly), ...
    training, validation);
comparison.Models.LoadOnly = buildModelReport( ...
    loadOnly, residualForCalibration(wrench, quaternion, loadOnly), ...
    training, validation);
comparison.Models.Joint = buildModelReport( ...
    joint, residualForCalibration(wrench, quaternion, joint), ...
    training, validation);
comparison.Models.Joint.FitConditionNumber = ...
    jointFitReport.maximumConditionNumber;
comparison.BiasCorrectionSensor = biasCorrection;
comparison.SourceCalibration = source;
end

function indices = validateIndices(value, poseCount, name)
indices = double(value(:).');
if isempty(indices) || any(~isfinite(indices)) || ...
        any(indices ~= round(indices)) || ...
        any(indices < 1 | indices > poseCount) || ...
        numel(unique(indices)) ~= numel(indices)
    error('scopeguide:residualModels:InvalidIndices', ...
        '%s must contain unique valid pose indices.', name);
end
end

function calibration = normalizeCalibration(input)
required = {'biasSensor','massKg','comSensorM','gravityBaseMps2', ...
    'gravityForceSign','rotationToolFromSensor','quaternionConvention'};
for index = 1:numel(required)
    if ~isfield(input, required{index})
        error('scopeguide:residualModels:InvalidCalibration', ...
            'Calibration is missing %s.', required{index});
    end
end
calibration = input;
calibration.biasSensor = finiteColumn(input.biasSensor, 6, 'biasSensor');
calibration.comSensorM = finiteColumn(input.comSensorM, 3, 'comSensorM');
calibration.gravityBaseMps2 = finiteColumn( ...
    input.gravityBaseMps2, 3, 'gravityBaseMps2');
calibration.massKg = double(input.massKg);
calibration.gravityForceSign = double(input.gravityForceSign);
calibration.rotationToolFromSensor = double(input.rotationToolFromSensor);
calibration.quaternionConvention = char(input.quaternionConvention);
calibration.valid = true;
if ~isfinite(calibration.massKg) || calibration.massKg <= 0 || ...
        ~ismember(calibration.gravityForceSign, [-1, 1])
    error('scopeguide:residualModels:InvalidCalibration', ...
        'Calibration mass and gravity sign are invalid.');
end
end

function value = finiteColumn(input, count, name)
value = double(input(:));
if numel(value) ~= count || any(~isfinite(value))
    error('scopeguide:residualModels:InvalidCalibration', ...
        '%s must contain %d finite values.', name, count);
end
end

function residual = residualForCalibration(wrench, quaternion, calibration)
residual = zeros(size(wrench));
for index = 1:size(wrench, 1)
    compensated = compensateHexHWrench( ...
        wrench(index, :).', quaternion(index, :), calibration);
    residual(index, :) = compensated.externalSensor.';
end
end

function calibration = fitLoadWithFixedBias( ...
        wrench, quaternion, source, maximumConditionNumber)
poseCount = size(wrench, 1);
gravityPerKg = zeros(poseCount, 3);
for index = 1:poseCount
    probe = source;
    probe.massKg = 1;
    probe.comSensorM = zeros(3, 1);
    compensated = compensateHexHWrench( ...
        source.biasSensor, quaternion(index, :), probe);
    gravityPerKg(index, :) = compensated.gravitySensor(1:3).';
end
forceDesign = reshape(gravityPerKg.', [], 1);
forceTarget = reshape((wrench(:, 1:3) - source.biasSensor(1:3).').', [], 1);
forceCondition = cond(forceDesign);
if rank(forceDesign) < 1 || ~isfinite(forceCondition) || ...
        forceCondition > maximumConditionNumber
    error('scopeguide:residualModels:UnobservableLoadForce', ...
        'The fixed-bias mass model is unobservable.');
end
mass = forceDesign \ forceTarget;
if ~isfinite(mass) || mass <= 0
    error('scopeguide:residualModels:InvalidLoadMass', ...
        'The fixed-bias load-only model produced non-positive mass.');
end

torqueDesign = zeros(3 * poseCount, 3);
torqueTarget = reshape( ...
    (wrench(:, 4:6) - source.biasSensor(4:6).').', [], 1);
for index = 1:poseCount
    force = mass * gravityPerKg(index, :).';
    torqueDesign((3*index-2):(3*index), :) = -skew3(force);
end
torqueCondition = cond(torqueDesign);
if rank(torqueDesign) < 3 || ~isfinite(torqueCondition) || ...
        torqueCondition > maximumConditionNumber
    error('scopeguide:residualModels:UnobservableLoadTorque', ...
        'The fixed-bias CoG model is unobservable (condition %.3g).', ...
        torqueCondition);
end
centerOfMass = torqueDesign \ torqueTarget;

calibration = source;
calibration.massKg = mass;
calibration.comSensorM = centerOfMass;
calibration.fitConditionNumber = max(forceCondition, torqueCondition);
end

function report = buildModelReport(calibration, residual, training, validation)
report = struct();
report.Calibration = calibration;
report.ResidualSensor = residual;
report.Training = residualMetrics(residual(training, :));
report.Validation = residualMetrics(residual(validation, :));
end

function metrics = residualMetrics(residual)
forceNorm = vecnorm(residual(:, 1:3), 2, 2);
momentNorm = vecnorm(residual(:, 4:6), 2, 2);
metrics = struct();
metrics.PoseCount = size(residual, 1);
metrics.MeanResidual = mean(residual, 1);
metrics.ForceVectorRmseN = sqrt(mean(forceNorm.^2));
metrics.MomentVectorRmseNm = sqrt(mean(momentNorm.^2));
metrics.MaximumForceNormN = max(forceNorm);
metrics.MaximumMomentNormNm = max(momentNorm);
metrics.ForceNormPerPoseN = forceNorm;
metrics.MomentNormPerPoseNm = momentNorm;
end

function matrix = skew3(vector)
value = double(vector(:));
matrix = [0, -value(3), value(2); ...
          value(3), 0, -value(1); ...
          -value(2), value(1), 0];
end
