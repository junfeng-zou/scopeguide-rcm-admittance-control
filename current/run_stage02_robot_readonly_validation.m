function [validation, outputDirectory] = ...
        run_stage02_robot_readonly_validation(options)
%RUN_STAGE02_ROBOT_READONLY_VALIDATION Observe CR5 feedback without motion.
% This entry point opens the robot network connections and reads 30004
% feedback. It never calls Enable, ServoJ, MovJ, MovL or any other command.
% Manually place the disabled robot at each validation pose, then run this
% function once per pose with a distinct PoseLabel.

arguments
    options.Config = struct([])
    options.PoseLabel (1, 1) string = "pose_01"
    options.DurationSec (1, 1) double {mustBeFinite, mustBePositive} = 10
    options.SampleRateHz (1, 1) double {mustBeFinite, mustBePositive} = 100
    options.WriteResults (1, 1) logical = true
end

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'robot'));
if isempty(options.Config)
    cfg = defaultRcmAdmittanceConfig();
    cfg.runtime.Mode = "live_dry_run";
else
    cfg = options.Config;
end
validateRcmAdmittanceConfig(cfg);
if cfg.runtime.Mode ~= "live_dry_run" || ...
        cfg.robot.EnableMotion || ~cfg.robot.DryRun
    error('scopeguide:stage02:ReadOnlyConfigurationRequired', ...
        ['Stage 2 live validation requires runtime.Mode=live_dry_run, ' ...
        'EnableMotion=false and DryRun=true.']);
end
if strlength(strtrim(options.PoseLabel)) == 0
    error('scopeguide:stage02:EmptyPoseLabel', ...
        'PoseLabel must be a nonempty scalar string.');
end

adapter = scopeguide.io.RobotAdapter(cfg);
cleanup = onCleanup(@() adapter.disconnect());
adapter.connectReadOnly();
validation = collectReadOnlySamples(adapter, cfg, options);
validation.Summary.AdapterCommandAttemptCount = ...
    double(adapter.CommandAttemptCount);
validation.Summary.AdapterCommandSentCount = ...
    double(adapter.CommandSentCount);
validation.Summary.PhysicalMotionCommandSent = false;
validation.Summary.ServoTimingMeasured = false;
validation.Summary.ModelAutomaticallyMarkedVerified = false;

if options.WriteResults
    outputDirectory = writeResults(projectRoot, cfg, validation);
else
    outputDirectory = "";
end
clear cleanup;
end

function validation = collectReadOnlySamples(adapter, cfg, options)
capacity = max(2, ceil(options.DurationSec * options.SampleRateHz) + 2);
readTimeSec = nan(capacity, 1);
feedbackSequence = zeros(capacity, 1, 'uint64');
feedbackTimeSec = nan(capacity, 1);
feedbackAgeSec = nan(capacity, 1);
invalidFeedbackByteCount = zeros(capacity, 1, 'uint64');
isValid = false(capacity, 1);
statusCode = strings(capacity, 1);
robotMode = strings(capacity, 1);
jointPositionRad = nan(capacity, 6);
controllerPositionM = nan(capacity, 3);
controllerQuaternionWxyz = nan(capacity, 4);
rpyQuaternionMismatchRad = nan(capacity, 1);
flangePositionErrorM = nan(capacity, 1);
flangeOrientationErrorRad = nan(capacity, 1);
endoscopePositionErrorM = nan(capacity, 1);
endoscopeOrientationErrorRad = nan(capacity, 1);

periodSec = 1 / options.SampleRateHz;
clock = tic;
nextReadSec = 0;
count = 0;
while toc(clock) < options.DurationSec
    waitUntil(clock, nextReadSec);
    count = count + 1;
    if count > capacity
        error('scopeguide:stage02:SampleCapacityExceeded', ...
            'Internal sample capacity was exceeded.');
    end
    state = adapter.readState();
    readTimeSec(count) = toc(clock);
    feedbackSequence(count) = state.FeedbackSequence;
    feedbackTimeSec(count) = state.HostMonotonicSec;
    feedbackAgeSec(count) = state.SampleAgeSec;
    invalidFeedbackByteCount(count) = state.InvalidFeedbackByteCount;
    isValid(count) = state.IsValid;
    statusCode(count) = state.StatusCode;
    robotMode(count) = state.RobotMode;
    jointPositionRad(count, :) = state.JointPositionRad.';
    controllerPositionM(count, :) = state.ControllerPosePositionM.';
    controllerQuaternionWxyz(count, :) = ...
        state.QuaternionBaseControllerWxyz.';
    rpyQuaternionMismatchRad(count) = ...
        state.ControllerRpyQuaternionMismatchRad;

    model = scopeguide.geometry.cr5ForwardKinematics( ...
        state.JointPositionRad, cfg);
    measuredRotation = state.TBaseControllerPose(1:3, 1:3);
    flangePositionErrorM(count) = norm( ...
        model.TBaseFlange(1:3, 4) - state.ControllerPosePositionM);
    flangeOrientationErrorRad(count) = ...
        scopeguide.geometry.rotationDistance( ...
        model.TBaseFlange(1:3, 1:3), measuredRotation);
    endoscopePositionErrorM(count) = norm( ...
        model.TBaseEndoscope(1:3, 4) - state.ControllerPosePositionM);
    endoscopeOrientationErrorRad(count) = ...
        scopeguide.geometry.rotationDistance( ...
        model.TBaseEndoscope(1:3, 1:3), measuredRotation);
    nextReadSec = nextReadSec + periodSec;
end

validation = struct();
validation.ReadTimeSec = readTimeSec(1:count);
validation.FeedbackSequence = feedbackSequence(1:count);
validation.FeedbackTimeSec = feedbackTimeSec(1:count);
validation.FeedbackAgeSec = feedbackAgeSec(1:count);
validation.InvalidFeedbackByteCount = ...
    invalidFeedbackByteCount(1:count);
validation.IsValid = isValid(1:count);
validation.StatusCode = statusCode(1:count);
validation.RobotMode = robotMode(1:count);
validation.JointPositionRad = jointPositionRad(1:count, :);
validation.ControllerPositionM = controllerPositionM(1:count, :);
validation.ControllerQuaternionWxyz = ...
    controllerQuaternionWxyz(1:count, :);
validation.RpyQuaternionMismatchRad = ...
    rpyQuaternionMismatchRad(1:count);
validation.FlangePositionErrorM = flangePositionErrorM(1:count);
validation.FlangeOrientationErrorRad = ...
    flangeOrientationErrorRad(1:count);
validation.EndoscopePositionErrorM = ...
    endoscopePositionErrorM(1:count);
validation.EndoscopeOrientationErrorRad = ...
    endoscopeOrientationErrorRad(1:count);

sequenceDelta = diff(double(feedbackSequence(1:count)));
feedbackTimeDelta = diff(feedbackTimeSec(1:count));
newFrame = sequenceDelta > 0 & feedbackTimeDelta >= 0;
perFramePeriodSec = feedbackTimeDelta(newFrame) ./ sequenceDelta(newFrame);
readPeriodSec = diff(readTimeSec(1:count));
flangeRmseM = rootMeanSquare(flangePositionErrorM(1:count));
endoscopeRmseM = rootMeanSquare(endoscopePositionErrorM(1:count));
if flangeRmseM < endoscopeRmseM
    inferredReference = "flange";
else
    inferredReference = "endoscope";
end

summary = struct();
summary.Stage = 2;
summary.Status = "current_robot_readonly_validation_complete";
summary.PoseLabel = options.PoseLabel;
summary.DurationRequestedSec = options.DurationSec;
summary.SampleRateRequestedHz = options.SampleRateHz;
summary.ReadSampleCount = count;
summary.UniqueFeedbackCount = numel(unique(feedbackSequence(1:count)));
summary.DuplicateReadRatio = 1 - summary.UniqueFeedbackCount / count;
sequenceSpan = double(max(feedbackSequence(1:count)) - ...
    min(feedbackSequence(1:count)) + uint64(1));
summary.FeedbackSequenceSpan = sequenceSpan;
summary.UnobservedFeedbackFrameRatio = max(0, ...
    1 - summary.UniqueFeedbackCount / sequenceSpan);
summary.FeedbackSequenceRegressionCount = sum(sequenceDelta < 0);
summary.ValidReadRatio = mean(isValid(1:count));
summary.StaleReadRatio = mean(statusCode(1:count) == "STALE_FEEDBACK");
summary.InvalidFeedbackByteCountStart = ...
    double(invalidFeedbackByteCount(1));
summary.InvalidFeedbackByteCountEnd = ...
    double(invalidFeedbackByteCount(count));
summary.InvalidFeedbackByteCountIncrease = ...
    summary.InvalidFeedbackByteCountEnd - ...
    summary.InvalidFeedbackByteCountStart;
summary.ObservedRobotModes = unique(robotMode(1:count)).';
summary.ReadPeriodP50Sec = percentile(readPeriodSec, 50);
summary.ReadPeriodP95Sec = percentile(readPeriodSec, 95);
summary.ReadPeriodP99Sec = percentile(readPeriodSec, 99);
summary.FeedbackPeriodP50Sec = percentile(perFramePeriodSec, 50);
summary.FeedbackPeriodP95Sec = percentile(perFramePeriodSec, 95);
summary.FeedbackPeriodP99Sec = percentile(perFramePeriodSec, 99);
summary.FeedbackAgeP50Sec = percentile(feedbackAgeSec(1:count), 50);
summary.FeedbackAgeP95Sec = percentile(feedbackAgeSec(1:count), 95);
summary.FeedbackAgeP99Sec = percentile(feedbackAgeSec(1:count), 99);
summary.FeedbackCallbackAbsoluteJitterP99Sec = percentile( ...
    abs(perFramePeriodSec - median(perFramePeriodSec, 'omitnan')), 99);
summary.TimingDataSufficient = numel(perFramePeriodSec) >= 100;
summary.InferredCurrentControllerPoseReference = inferredReference;
summary.FlangePositionRmseM = flangeRmseM;
summary.FlangePositionMaximumM = max(flangePositionErrorM(1:count));
summary.FlangeOrientationRmseRad = ...
    rootMeanSquare(flangeOrientationErrorRad(1:count));
summary.FlangeOrientationMaximumRad = ...
    max(flangeOrientationErrorRad(1:count));
summary.EndoscopePositionRmseM = endoscopeRmseM;
summary.EndoscopePositionMaximumM = ...
    max(endoscopePositionErrorM(1:count));
summary.EndoscopeOrientationRmseRad = ...
    rootMeanSquare(endoscopeOrientationErrorRad(1:count));
summary.EndoscopeOrientationMaximumRad = ...
    max(endoscopeOrientationErrorRad(1:count));
summary.RpyQuaternionMismatchRmseRad = ...
    rootMeanSquare(rpyQuaternionMismatchRad(1:count));
summary.RpyQuaternionMismatchMaximumRad = ...
    max(rpyQuaternionMismatchRad(1:count));
summary.FlangeErrorToSoftRcmRatio = ...
    summary.FlangePositionMaximumM / cfg.rcm.SoftRadiusM;
summary.FlangeErrorToHardRcmRatio = ...
    summary.FlangePositionMaximumM / cfg.rcm.HardRadiusM;
summary.NominalModelSupportsCurrentHardRcmCandidate = ...
    summary.FlangePositionMaximumM < cfg.rcm.HardRadiusM;
summary.SuggestedFeedbackStaleSec = max( ...
    3 * summary.FeedbackPeriodP99Sec, summary.FeedbackAgeP99Sec + ...
    summary.FeedbackPeriodP99Sec);
validation.PerFramePeriodSec = perFramePeriodSec;
validation.Summary = summary;
end

function outputDirectory = writeResults(projectRoot, cfg, validation)
timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
safeLabel = regexprep(validation.Summary.PoseLabel, '[^A-Za-z0-9_-]', '_');
outputDirectory = fullfile(projectRoot, 'results', ...
    "stage02_live_readonly_" + safeLabel + "_" + timestamp);
[created, message] = mkdir(outputDirectory);
if ~created && ~isfolder(outputDirectory)
    error('scopeguide:stage02:CannotCreateResultDirectory', ...
        'Cannot create %s: %s', outputDirectory, message);
end
writeJson(fullfile(outputDirectory, 'summary.json'), validation.Summary);
writeJson(fullfile(outputDirectory, 'config_snapshot.json'), cfg);
save(fullfile(outputDirectory, 'signals.mat'), 'validation', '-v7.3');

signals = table(validation.ReadTimeSec, ...
    double(validation.FeedbackSequence), validation.FeedbackTimeSec, ...
    validation.FeedbackAgeSec, double(validation.InvalidFeedbackByteCount), ...
    validation.IsValid, validation.StatusCode, validation.RobotMode, ...
    validation.JointPositionRad(:, 1), validation.JointPositionRad(:, 2), ...
    validation.JointPositionRad(:, 3), validation.JointPositionRad(:, 4), ...
    validation.JointPositionRad(:, 5), validation.JointPositionRad(:, 6), ...
    validation.ControllerPositionM(:, 1), ...
    validation.ControllerPositionM(:, 2), ...
    validation.ControllerPositionM(:, 3), ...
    validation.FlangePositionErrorM, ...
    validation.FlangeOrientationErrorRad, ...
    validation.EndoscopePositionErrorM, ...
    validation.EndoscopeOrientationErrorRad, ...
    validation.RpyQuaternionMismatchRad, ...
    'VariableNames', {'ReadTimeSec', 'FeedbackSequence', ...
    'FeedbackTimeSec', 'FeedbackAgeSec', 'InvalidFeedbackByteCount', ...
    'IsValid', 'StatusCode', 'RobotMode', 'J1Rad', 'J2Rad', 'J3Rad', ...
    'J4Rad', 'J5Rad', 'J6Rad', 'ControllerXM', 'ControllerYM', ...
    'ControllerZM', 'FlangePositionErrorM', ...
    'FlangeOrientationErrorRad', 'EndoscopePositionErrorM', ...
    'EndoscopeOrientationErrorRad', 'RpyQuaternionMismatchRad'});
writetable(signals, fullfile(outputDirectory, 'signals.csv'));
end

function waitUntil(clock, targetSec)
remainingSec = targetSec - toc(clock);
while remainingSec > 0
    pause(min(remainingSec, 0.001));
    remainingSec = targetSec - toc(clock);
end
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
upperIndex = ceil(position);
if lower == upperIndex
    value = values(lower);
else
    fraction = position - lower;
    value = values(lower) * (1 - fraction) + ...
        values(upperIndex) * fraction;
end
end

function writeJson(filePath, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end
[fileId, message] = fopen(filePath, 'w');
if fileId < 0
    error('scopeguide:stage02:CannotWriteJson', ...
        'Cannot open %s: %s', filePath, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end
