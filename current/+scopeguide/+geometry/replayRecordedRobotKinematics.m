function validation = replayRecordedRobotKinematics(cfg, csvFile, options)
%REPLAYRECORDEDROBOTKINEMATICS Validate FK against recorded CR5 feedback.
% The source data is read-only and no hardware connection is created.

arguments
    cfg (1, 1) struct
    csvFile (1, 1) string
    options.RetainedPosesOnly (1, 1) logical = true
    options.StationaryOnly (1, 1) logical = true
    options.UniqueFeedbackOnly (1, 1) logical = true
end

validateRcmAdmittanceConfig(cfg);
if ~isfile(csvFile)
    error('scopeguide:kinematics:RecordedDataMissing', ...
        'Recorded robot data does not exist: %s', csvFile);
end
data = readtable(csvFile, 'VariableNamingRule', 'preserve');
sourceRowCount = height(data);
required = ["poseIndex", "robotFeedbackSequence", ...
    "robotFeedbackTimeSec", "stationary", ...
    "J1_deg", "J2_deg", "J3_deg", "J4_deg", "J5_deg", "J6_deg", ...
    "tcpPose1", "tcpPose2", "tcpPose3", ...
    "tcpPose4", "tcpPose5", "tcpPose6", "qw", "qx", "qy", "qz"];
missing = setdiff(required, string(data.Properties.VariableNames));
if ~isempty(missing)
    error('scopeguide:kinematics:RecordedColumnsMissing', ...
        'Recorded data is missing columns: %s.', strjoin(missing, ', '));
end

if options.RetainedPosesOnly
    [~, provenance] = loadHexHGravityCalibration( ...
        cfg.force.CalibrationFile);
    data = data(ismember(data.poseIndex, ...
        provenance.retainedPoseIndices), :);
end
retainedPoseRowCount = height(data);
if options.StationaryOnly
    data = data(logical(data.stationary), :);
end
inputRowCount = height(data);
if options.UniqueFeedbackOnly
    [~, uniqueIndices] = unique(data.robotFeedbackSequence, 'stable');
    data = data(uniqueIndices, :);
end
if isempty(data)
    error('scopeguide:kinematics:EmptyRecordedData', ...
        'No recorded robot samples remain after selection.');
end

jointDeg = [data.J1_deg, data.J2_deg, data.J3_deg, ...
    data.J4_deg, data.J5_deg, data.J6_deg];
controllerPose = [data.tcpPose1, data.tcpPose2, data.tcpPose3, ...
    data.tcpPose4, data.tcpPose5, data.tcpPose6];
quaternion = [data.qw, data.qx, data.qy, data.qz];
count = height(data);
flangePositionErrorM = nan(count, 1);
flangeOrientationErrorRad = nan(count, 1);
endoscopePositionErrorM = nan(count, 1);
endoscopeOrientationErrorRad = nan(count, 1);
rpyQuaternionMismatchRad = nan(count, 1);
modelFlangePositionM = nan(count, 3);
modelEndoscopePositionM = nan(count, 3);
controllerPositionM = controllerPose(:, 1:3) * ...
    cfg.robot.ControllerPoseTranslationScaleToM;

for index = 1:count
    q = jointDeg(index, :).'*cfg.robot.JointPositionScaleToRad;
    model = scopeguide.geometry.cr5ForwardKinematics(q, cfg);
    controllerRotation = ...
        scopeguide.geometry.rotationMatrixFromQuaternionWxyz( ...
        quaternion(index, :));
    rpyRotation = scopeguide.geometry.dobotRpyToRotation( ...
        controllerPose(index, 4:6), ...
        cfg.robot.ControllerPoseAngleScaleToRad);
    modelFlangePositionM(index, :) = model.TBaseFlange(1:3, 4).';
    modelEndoscopePositionM(index, :) = ...
        model.TBaseEndoscope(1:3, 4).';
    flangePositionErrorM(index) = norm( ...
        model.TBaseFlange(1:3, 4) - controllerPositionM(index, :).');
    flangeOrientationErrorRad(index) = ...
        scopeguide.geometry.rotationDistance( ...
        model.TBaseFlange(1:3, 1:3), controllerRotation);
    endoscopePositionErrorM(index) = norm( ...
        model.TBaseEndoscope(1:3, 4) - controllerPositionM(index, :).');
    endoscopeOrientationErrorRad(index) = ...
        scopeguide.geometry.rotationDistance( ...
        model.TBaseEndoscope(1:3, 1:3), controllerRotation);
    rpyQuaternionMismatchRad(index) = ...
        scopeguide.geometry.rotationDistance(rpyRotation, ...
        controllerRotation);
end

feedbackSequence = double(data.robotFeedbackSequence(:));
feedbackTimeSec = double(data.robotFeedbackTimeSec(:));
sequenceDelta = diff(feedbackSequence);
timeDelta = diff(feedbackTimeSec);
validPeriod = sequenceDelta > 0 & timeDelta >= 0;
perFramePeriodSec = timeDelta(validPeriod) ./ sequenceDelta(validPeriod);

flangePositionRmseM = rootMeanSquare(flangePositionErrorM);
endoscopePositionRmseM = rootMeanSquare(endoscopePositionErrorM);
if flangePositionRmseM < endoscopePositionRmseM
    inferredReference = "flange";
else
    inferredReference = "endoscope";
end

summary = struct();
summary.SourceFile = csvFile;
summary.SourceRowCount = sourceRowCount;
summary.RetainedPoseRowCount = retainedPoseRowCount;
summary.StationaryOnly = options.StationaryOnly;
summary.InputRowCount = inputRowCount;
summary.UniqueFeedbackCount = count;
summary.InputDuplicateRatio = 1 - count / inputRowCount;
summary.PoseIndices = unique(double(data.poseIndex(:))).';
summary.AllSamplesStationary = all(logical(data.stationary));
summary.InferredHistoricalControllerPoseReference = inferredReference;
summary.FlangePositionRmseM = flangePositionRmseM;
summary.FlangePositionMaximumM = max(flangePositionErrorM);
summary.FlangeOrientationRmseRad = ...
    rootMeanSquare(flangeOrientationErrorRad);
summary.FlangeOrientationMaximumRad = max(flangeOrientationErrorRad);
summary.EndoscopePositionRmseM = endoscopePositionRmseM;
summary.EndoscopePositionMaximumM = max(endoscopePositionErrorM);
summary.EndoscopeOrientationRmseRad = ...
    rootMeanSquare(endoscopeOrientationErrorRad);
summary.EndoscopeOrientationMaximumRad = ...
    max(endoscopeOrientationErrorRad);
summary.RpyQuaternionMismatchRmseRad = ...
    rootMeanSquare(rpyQuaternionMismatchRad);
summary.RpyQuaternionMismatchMaximumRad = ...
    max(rpyQuaternionMismatchRad);
summary.FeedbackPeriodP50Sec = percentile(perFramePeriodSec, 50);
summary.FeedbackPeriodP95Sec = percentile(perFramePeriodSec, 95);
summary.FeedbackPeriodP99Sec = percentile(perFramePeriodSec, 99);
summary.FeedbackRateFromP50Hz = 1 / summary.FeedbackPeriodP50Sec;
summary.FlangeErrorToSoftRcmRatio = ...
    summary.FlangePositionMaximumM / cfg.rcm.SoftRadiusM;
summary.FlangeErrorToHardRcmRatio = ...
    summary.FlangePositionMaximumM / cfg.rcm.HardRadiusM;
summary.NominalModelSupportsCurrentHardRcmCandidate = ...
    summary.FlangePositionMaximumM < cfg.rcm.HardRadiusM;
summary.CurrentLiveSemanticsStillRequireVerification = true;

validation = struct();
validation.PoseIndex = double(data.poseIndex(:));
validation.FeedbackSequence = uint64(data.robotFeedbackSequence(:));
validation.FeedbackTimeSec = feedbackTimeSec;
validation.JointPositionDeg = jointDeg;
validation.ControllerPoseRaw = controllerPose;
validation.ControllerPositionM = controllerPositionM;
validation.ModelFlangePositionM = modelFlangePositionM;
validation.ModelEndoscopePositionM = modelEndoscopePositionM;
validation.FlangePositionErrorM = flangePositionErrorM;
validation.FlangeOrientationErrorRad = flangeOrientationErrorRad;
validation.EndoscopePositionErrorM = endoscopePositionErrorM;
validation.EndoscopeOrientationErrorRad = endoscopeOrientationErrorRad;
validation.RpyQuaternionMismatchRad = rpyQuaternionMismatchRad;
validation.PerFramePeriodSec = perFramePeriodSec;
validation.Summary = summary;
end

function value = rootMeanSquare(values)
value = sqrt(mean(values.^2, 'omitnan'));
end

function value = percentile(values, percentage)
values = sort(double(values(isfinite(values))));
if isempty(values)
    value = NaN;
    return;
end
position = 1 + (numel(values) - 1) * percentage / 100;
lower = floor(position);
upper = ceil(position);
if lower == upper
    value = values(lower);
else
    fraction = position - lower;
    value = values(lower) * (1 - fraction) + values(upper) * fraction;
end
end
