function result = createExcludedPoseCalibration( ...
        sourceCalibrationMat, excludedPoseIndices, outputStem)
%CREATEEXCLUDEDPOSECALIBRATION Refit a saved calibration without bad poses.
%
% The source MAT must have been produced by
% calibrateHexHToolLoadWithDobot.m. The derived JSON and MAT are written
% beside the source file; the source files are never modified.

if nargin < 3
    outputStem = "";
end
sourceCalibrationMat = string(sourceCalibrationMat);
excludedPoseIndices = double(excludedPoseIndices(:).');
outputStem = string(outputStem);
if ~isscalar(sourceCalibrationMat) || ~isfile(sourceCalibrationMat)
    error('calibration:SourceCalibrationMissing', ...
        'Source calibration MAT does not exist: %s', sourceCalibrationMat);
end
if isempty(excludedPoseIndices) || ...
        any(~isfinite(excludedPoseIndices)) || ...
        any(excludedPoseIndices ~= round(excludedPoseIndices)) || ...
        numel(unique(excludedPoseIndices)) ~= numel(excludedPoseIndices)
    error('calibration:InvalidExcludedPoses', ...
        'excludedPoseIndices must contain unique integer pose indices.');
end
if ~isscalar(outputStem)
    error('calibration:InvalidOutputStem', ...
        'outputStem must be a scalar string.');
end

source = load(sourceCalibrationMat);
required = {'calibration','fitReport','poseSummary','rawData','cfg'};
for index = 1:numel(required)
    if ~isfield(source, required{index})
        error('calibration:InvalidSourceCalibration', ...
            'Source MAT is missing variable %s.', required{index});
    end
end

poseCount = height(source.poseSummary);
if any(excludedPoseIndices < 1 | excludedPoseIndices > poseCount)
    error('calibration:InvalidExcludedPoses', ...
        'Excluded pose indices must lie between 1 and %d.', poseCount);
end
retainedPoseIndices = setdiff(1:poseCount, excludedPoseIndices, 'stable');
if numel(retainedPoseIndices) < 7
    error('calibration:InsufficientRetainedPoses', ...
        'At least seven poses must remain after exclusion.');
end

summary = source.poseSummary;
poseWrench = [summary.Fx_N, summary.Fy_N, summary.Fz_N, ...
    summary.Tx_Nm, summary.Ty_Nm, summary.Tz_Nm];
poseQuaternion = [summary.qw, summary.qx, summary.qy, summary.qz];
[calibration, fitReport] = fitHexHToolLoadCalibration( ...
    poseWrench(retainedPoseIndices, :), ...
    poseQuaternion(retainedPoseIndices, :), ...
    'RToolFromSensor', source.cfg.RToolFromSensor, ...
    'GravityBaseMps2', source.cfg.GravityBaseMps2, ...
    'GravityForceSign', source.cfg.GravityForceSign, ...
    'QuaternionConvention', source.cfg.QuaternionConvention, ...
    'MaximumConditionNumber', source.cfg.MaximumConditionNumber, ...
    'ComputeLeaveOneOut', true);

copyFields = {'sensorOriginInToolM','robotIPAddress','sensorIPAddress', ...
    'zeroJointGravityReference'};
for index = 1:numel(copyFields)
    name = copyFields{index};
    if isfield(source.calibration, name)
        calibration.(name) = source.calibration.(name);
    end
end
calibration.createdUTC = char(datetime('now', 'TimeZone', 'UTC', ...
    'Format', 'yyyy-MM-dd''T''HH:mm:ss.SSSXXX'));

excludedPoseResidualFromRetainedFit = ...
    zeros(numel(excludedPoseIndices), 6);
calibrationForCompensation = calibration;
calibrationForCompensation.biasSensor = ...
    calibrationForCompensation.biasSensor(:);
calibrationForCompensation.comSensorM = ...
    calibrationForCompensation.comSensorM(:);
calibrationForCompensation.gravityBaseMps2 = ...
    calibrationForCompensation.gravityBaseMps2(:);
for index = 1:numel(excludedPoseIndices)
    pose = excludedPoseIndices(index);
    compensated = compensateHexHWrench( ...
        poseWrench(pose, :).', poseQuaternion(pose, :), ...
        calibrationForCompensation);
    excludedPoseResidualFromRetainedFit(index, :) = ...
        compensated.externalSensor.';
end
excludedPoseResidualFromNinePoseFit = ...
    excludedPoseResidualFromRetainedFit;

if strlength(outputStem) == 0
    labels = compose('pose%02d', excludedPoseIndices);
    outputStem = "calibration_without_" + join(labels, "_");
end
outputDirectory = string(fileparts(sourceCalibrationMat));
jsonFile = fullfile(outputDirectory, outputStem + ".json");
matFile = fullfile(outputDirectory, outputStem + ".mat");

sourceCalibrationMat = char(sourceCalibrationMat);
poseSummary = summary(retainedPoseIndices, :); %#ok<NASGU>
cfg = source.cfg; %#ok<NASGU>
rawData = source.rawData( ...
    ismember(source.rawData.poseIndex, retainedPoseIndices), :); %#ok<NASGU>
save(matFile, 'sourceCalibrationMat', 'retainedPoseIndices', ...
    'excludedPoseIndices', 'excludedPoseResidualFromRetainedFit', ...
    'excludedPoseResidualFromNinePoseFit', 'calibration', 'fitReport', ...
    'poseSummary', 'rawData', 'cfg');

payload = struct();
payload.sourceCalibrationMat = sourceCalibrationMat;
payload.retainedPoseIndices = retainedPoseIndices;
payload.excludedPoseIndices = excludedPoseIndices;
payload.excludedPoseResidualFromRetainedFit = ...
    excludedPoseResidualFromRetainedFit;
if numel(retainedPoseIndices) == 9
    payload.excludedPoseResidualFromNinePoseFit = ...
        excludedPoseResidualFromNinePoseFit;
end
payload.calibration = calibration;
payload.fitReport = fitReport;
writeJson(jsonFile, payload);

result = struct();
result.jsonFile = string(jsonFile);
result.matFile = string(matFile);
result.retainedPoseIndices = retainedPoseIndices;
result.excludedPoseIndices = excludedPoseIndices;
result.excludedPoseResidual = excludedPoseResidualFromRetainedFit;
result.calibration = calibration;
result.fitReport = fitReport;
end

function writeJson(path, payload)
try
    text = jsonencode(payload, 'PrettyPrint', true);
catch
    text = jsonencode(payload);
end
file = fopen(path, 'w');
if file < 0
    error('calibration:JsonWriteFailed', 'Could not open %s.', path);
end
guard = onCleanup(@() fclose(file)); %#ok<NASGU>
fwrite(file, text, 'char');
end
