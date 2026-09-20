function replay = replayCalibrationDataset(cfg, csvFile, options)
%REPLAYCALIBRATIONDATASET Replay recorded pose-aware calibration samples.
% This function is offline-only and never constructs a hardware client.

arguments
    cfg (1, 1) struct
    csvFile (1, 1) string
    options.MaximumSamples (1, 1) double = inf
    options.RetainedPosesOnly (1, 1) logical = true
    options.AssumeSettledStaticPose (1, 1) logical = false
end

validateRcmAdmittanceConfig(cfg);
if ~isfile(csvFile)
    error('scopeguide:force:ReplayFileMissing', ...
        'Replay CSV does not exist: %s', csvFile);
end
if ~(options.MaximumSamples == inf || ...
        isfinite(options.MaximumSamples) && ...
        options.MaximumSamples >= 1 && ...
        options.MaximumSamples == fix(options.MaximumSamples))
    error('scopeguide:force:InvalidMaximumSamples', ...
        'MaximumSamples must be a positive integer or Inf.');
end

data = readtable(csvFile, 'VariableNamingRule', 'preserve');
required = ["poseIndex", "sensorSequence", "sensorMonotonicSec", ...
    "sensorReadDurationSec", "stationary", ...
    "Fx_N", "Fy_N", "Fz_N", "Tx_Nm", "Ty_Nm", "Tz_Nm", ...
    "qw", "qx", "qy", "qz"];
missing = setdiff(required, string(data.Properties.VariableNames));
if ~isempty(missing)
    error('scopeguide:force:ReplayColumnsMissing', ...
        'Replay CSV is missing columns: %s.', strjoin(missing, ', '));
end

pipeline = scopeguide.force.ForceProcessingPipeline(cfg);
retainedPoses = pipeline.CalibrationProvenance.retainedPoseIndices;
if options.RetainedPosesOnly && ~isempty(retainedPoses)
    data = data(ismember(data.poseIndex, retainedPoses), :);
end
if isfinite(options.MaximumSamples)
    data = data(1:min(height(data), options.MaximumSamples), :);
end
if isempty(data)
    error('scopeguide:force:EmptyReplay', ...
        'No samples remain after applying replay selection.');
end

count = height(data);
raw = nan(6, count);
external = nan(6, count);
fast = nan(6, count);
slow = nan(6, count);
deadzone = nan(6, count);
controlForce = nan(3, count);
controlWrench = nan(6, count);
baseline = nan(6, count);
valid = false(count, 1);
motionInputValid = false(count, 1);
safetyStop = false(count, 1);
safetyWarning = false(count, 1);
safetyStopReasons = strings(count, 1);
safetyWarningReasons = strings(count, 1);
neutralCheckPassed = false(count, 1);
statusCode = strings(count, 1);
sampleAgeSec = nan(count, 1);
dtSec = nan(count, 1);

previousPose = NaN;
for index = 1:count
    pose = double(data.poseIndex(index));
    if index > 1 && pose ~= previousPose
        pipeline.startNewSegment();
    end
    previousPose = pose;

    sample = scopeguide.types.forceSample();
    sample.RawWrenchSensor = [data.Fx_N(index); data.Fy_N(index); ...
        data.Fz_N(index); data.Tx_Nm(index); data.Ty_Nm(index); ...
        data.Tz_Nm(index)];
    sample.Sequence = uint64(data.sensorSequence(index));
    sample.HostMonotonicSec = double(data.sensorMonotonicSec(index));
    sample.ReadDurationSec = double(data.sensorReadDurationSec(index));
    sample.SampleAgeSec = 0;
    sample.DeviceStatus = 0;
    sample.IsValid = true;
    sample.StatusCode = "RECORDED";

    context = scopeguide.types.forceProcessingContext();
    context.HandleEnabled = false;
    settledStatic = logical(data.stationary(index)) || ...
        options.AssumeSettledStaticPose;
    context.RobotStationary = settledStatic;
    context.NoContactConfirmed = settledStatic;
    context.AllowBaselineUpdate = settledStatic;
    quaternion = [data.qw(index); data.qx(index); ...
        data.qy(index); data.qz(index)];

    processed = pipeline.step(sample, quaternion, ...
        sample.HostMonotonicSec, context);
    raw(:, index) = processed.RawWrenchSensor;
    external(:, index) = processed.ExternalWrenchToolAtSensorOrigin;
    fast(:, index) = processed.FastWrenchToolAtSensorOrigin;
    slow(:, index) = processed.SlowWrenchToolAtSensorOrigin;
    deadzone(:, index) = processed.DeadzoneWrenchToolAtSensorOrigin;
    controlForce(:, index) = processed.ControlForceTool;
    controlWrench(:, index) = ...
        processed.ControlWrenchToolAtSensorOrigin;
    baseline(:, index) = processed.BaselineWrenchTool;
    valid(index) = processed.Quality.Valid;
    motionInputValid(index) = processed.MotionInputValid;
    safetyStop(index) = processed.Safety.StopRequested;
    safetyWarning(index) = processed.Safety.WarningActive;
    safetyStopReasons(index) = joinReasons(processed.Safety.StopReasons);
    safetyWarningReasons(index) = ...
        joinReasons(processed.Safety.WarningReasons);
    neutralCheckPassed(index) = processed.NeutralCheckPassed;
    statusCode(index) = processed.Quality.StatusCode;
    sampleAgeSec(index) = processed.SampleAgeSec;
    dtSec(index) = processed.DtSec;
end

forceNorm = columnNorm(external(1:3, :));
slowForceNorm = columnNorm(slow(1:3, :));
controlForceNorm = columnNorm(controlForce);
momentNorm = columnNorm(external(4:6, :));

summary = struct();
summary.SourceFile = csvFile;
summary.CalibrationFile = string(cfg.force.CalibrationFile);
summary.SampleCount = count;
summary.PoseIndices = unique(double(data.poseIndex(:))).';
summary.ValidCount = nnz(valid);
summary.InvalidCount = nnz(~valid);
summary.MotionInputValidCount = nnz(motionInputValid);
summary.SafetyStopCount = nnz(safetyStop);
summary.SafetyWarningCount = nnz(safetyWarning);
summary.NeutralCheckPassCount = nnz(neutralCheckPassed);
summary.MaximumExternalForceNormN = max(forceNorm, [], 'omitnan');
summary.MaximumSlowForceNormN = max(slowForceNorm, [], 'omitnan');
summary.MaximumControlForceNormN = max(controlForceNorm, [], 'omitnan');
summary.MaximumExternalMomentNormNm = max(momentNorm, [], 'omitnan');
summary.ForceDeadzoneN = cfg.force.ForceDeadzoneN;
summary.MomentDeadzoneNm = cfg.force.MomentDeadzoneNm;
summary.MomentReference = "sensor_origin_rotated_to_tool_axes";
summary.MomentEligibleForControl = true;
summary.AssumeSettledStaticPose = options.AssumeSettledStaticPose;
summary.PipelineCounters = pipeline.Counters;

replay = struct();
replay.SourceFile = csvFile;
replay.PoseIndex = double(data.poseIndex(:));
replay.RecordedSequence = uint64(data.sensorSequence(:));
replay.TimestampSec = double(data.sensorMonotonicSec(:));
replay.DtSec = dtSec;
replay.SampleAgeSec = sampleAgeSec;
replay.RawWrenchSensor = raw;
replay.ExternalWrenchToolAtSensorOrigin = external;
replay.FastWrenchToolAtSensorOrigin = fast;
replay.SlowWrenchToolAtSensorOrigin = slow;
replay.DeadzoneWrenchToolAtSensorOrigin = deadzone;
replay.ControlForceTool = controlForce;
replay.ControlWrenchToolAtSensorOrigin = controlWrench;
replay.BaselineWrenchTool = baseline;
replay.QualityValid = valid;
replay.MotionInputValid = motionInputValid;
replay.SafetyStop = safetyStop;
replay.SafetyWarning = safetyWarning;
replay.SafetyStopReasons = safetyStopReasons;
replay.SafetyWarningReasons = safetyWarningReasons;
replay.NeutralCheckPassed = neutralCheckPassed;
replay.StatusCode = statusCode;
replay.Summary = summary;
end

function values = columnNorm(matrix)
values = sqrt(sum(matrix.^2, 1));
end

function joined = joinReasons(reasons)
if isempty(reasons)
    joined = "";
else
    joined = strjoin(string(reasons(:)), '|');
end
end
