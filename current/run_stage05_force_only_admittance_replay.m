function [replayResult, outputDirectory] = ...
        run_stage05_force_only_admittance_replay(options)
%RUN_STAGE05_FORCE_ONLY_ADMITTANCE_REPLAY Offline static-force replay.
% This entry point never constructs a robot or force-sensor client and
% never sends a command.  Until a measured pRcmBase is supplied it uses an
% explicitly labelled synthetic RCM/endoscope geometry for algorithm tests.

arguments
    options.Config = struct([])
    options.SourceFile = ""
    options.MaximumSamples (1, 1) double = inf
    options.WriteResults (1, 1) logical = true
end

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'force_sensor'));
addpath(fullfile(projectRoot, 'force_calibration'));
if isempty(options.Config)
    cfg = defaultRcmAdmittanceConfig();
else
    cfg = scopeguide.runtime.upgradeRcmAdmittanceConfig(options.Config);
end
cfg.force.AutomaticBaselineEnabled = true;
validateRcmAdmittanceConfig(cfg);

sourceFile = string(options.SourceFile);
if strlength(sourceFile) == 0
    sourceFile = fullfile(projectRoot, 'force_calibration', 'data', ...
        'hex_h_tool_calibration_20260812_154348', 'raw_samples.csv');
end
forceReplay = scopeguide.force.replayCalibrationDataset(cfg, ...
    sourceFile, MaximumSamples=options.MaximumSamples, ...
    RetainedPosesOnly=true, AssumeSettledStaticPose=true);

[TBaseEndoscope, pRcmBaseM, geometrySource] = ...
    selectOfflineGeometry(cfg);
count = numel(forceReplay.PoseIndex);
generalizedEffort = zeros(3, count);
generalizedVelocity = zeros(3, count);
relativeCoordinate = zeros(3, count);
tipTwist = zeros(6, count);
usedForAdmittance = false(count, 1);
resetApplied = false(count, 1);
admittance = scopeguide.control.ForceOnlyRcmAdmittance(cfg);
enabled = enabledStatus();
previousPose = NaN;
for index = 1:count
    poseChanged = index == 1 || ...
        forceReplay.PoseIndex(index) ~= previousPose;
    previousPose = forceReplay.PoseIndex(index);
    eligible = forceReplay.QualityValid(index) && ...
        forceReplay.MotionInputValid(index) && ...
        ~forceReplay.SafetyStop(index);
    if poseChanged || ~eligible
        admittance.reset();
        resetApplied(index) = true;
        continue;
    end
    dtSec = forceReplay.DtSec(index);
    if ~isfinite(dtSec) || dtSec <= 0
        dtSec = 1 / cfg.sensor.SampleRateHz;
    end
    output = admittance.step( ...
        forceReplay.DeadzoneWrenchToolAtSensorOrigin(:, index), ...
        TBaseEndoscope, pRcmBaseM, dtSec, enabled);
    if output.Valid
        generalizedEffort(:, index) = output.GeneralizedEffort;
        generalizedVelocity(:, index) = output.GeneralizedVelocity;
        relativeCoordinate(:, index) = output.RelativeCoordinate;
        tipTwist(:, index) = output.DesiredTipTwistBase;
        usedForAdmittance(index) = true;
    else
        resetApplied(index) = true;
    end
end

tail = trailingStaticMetrics(forceReplay.PoseIndex, ...
    forceReplay.TimestampSec, generalizedVelocity, ...
    usedForAdmittance, cfg);
summary = struct();
summary.Stage = 5;
summary.SourceFile = sourceFile;
summary.CalibrationFile = string(cfg.force.CalibrationFile);
summary.GeometrySource = geometrySource;
summary.StaticPoseAssumption = ...
    "recorded calibration captures are settled and no-contact";
summary.PRcmBaseM = pRcmBaseM;
summary.SampleCount = count;
summary.EligibleAdmittanceSampleCount = nnz(usedForAdmittance);
summary.ResetSampleCount = nnz(resetApplied);
summary.MaximumPivotSpeedRadSec = ...
    max(vecnorm(generalizedVelocity(1:2, :), 2, 1));
summary.MaximumInsertionSpeedMSec = ...
    max(abs(generalizedVelocity(3, :)));
summary.MaximumRelativePivotRad = ...
    max(vecnorm(relativeCoordinate(1:2, :), 2, 1));
summary.MaximumRelativeInsertionM = ...
    max(abs(relativeCoordinate(3, :)));
summary.TrailingStaticMetrics = tail;
summary.NoSustainedDrift = tail.Passed;
summary.RollRateAlwaysZero = true;
summary.MomentUsedForControl = true;
summary.OfflineOnly = true;
summary.HardwareConnectionsCreated = false;
summary.MotionCommandsSent = false;
summary.Parameters = admittance.Parameters;

replayResult = struct();
replayResult.Summary = summary;
replayResult.ForceReplay = forceReplay;
replayResult.GeneralizedEffort = generalizedEffort;
replayResult.GeneralizedVelocity = generalizedVelocity;
replayResult.RelativeCoordinate = relativeCoordinate;
replayResult.DesiredTipTwistBase = tipTwist;
replayResult.UsedForAdmittance = usedForAdmittance;
replayResult.ResetApplied = resetApplied;
replayResult.TBaseEndoscope = TBaseEndoscope;
replayResult.PRcmBaseM = pRcmBaseM;

if options.WriteResults
    timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
    outputDirectory = fullfile(projectRoot, 'results', ...
        "stage05_force_only_replay_" + timestamp);
    mkdir(outputDirectory);
    save(fullfile(outputDirectory, 'replay_result.mat'), ...
        'replayResult', 'cfg');
    writeJson(fullfile(outputDirectory, 'summary.json'), summary);
    signals = table(forceReplay.TimestampSec, forceReplay.PoseIndex, ...
        usedForAdmittance, generalizedEffort(1, :).', ...
        generalizedEffort(2, :).', generalizedEffort(3, :).', ...
        generalizedVelocity(1, :).', generalizedVelocity(2, :).', ...
        generalizedVelocity(3, :).', relativeCoordinate(1, :).', ...
        relativeCoordinate(2, :).', relativeCoordinate(3, :).', ...
        'VariableNames', {'TimestampSec', 'PoseIndex', 'Eligible', ...
        'GPivot1Nm', 'GPivot2Nm', 'GInsertionN', ...
        'UPivot1RadSec', 'UPivot2RadSec', 'UInsertionMSec', ...
        'XPivot1Rad', 'XPivot2Rad', 'XInsertionM'});
    writetable(signals, fullfile(outputDirectory, 'signals.csv'));
else
    outputDirectory = "";
end

fprintf('Stage 5 force-only replay: %s\n', ...
    passText(summary.NoSustainedDrift));
fprintf('Eligible samples: %d / %d; geometry=%s.\n', ...
    summary.EligibleAdmittanceSampleCount, count, geometrySource);
fprintf('Hardware connections: 0; motion commands: 0.\n');
if strlength(outputDirectory) > 0
    fprintf('Results: %s\n', outputDirectory);
end
end

function [transform, point, source] = selectOfflineGeometry(cfg)
if all(isfinite(cfg.rcm.PointBaseM))
    point = double(cfg.rcm.PointBaseM(:));
    source = "configured_manual_rcm_offline_only";
else
    point = [0; 0; 0];
    source = "synthetic_rcm_for_offline_algorithm_test";
end
transform = eye(4);
transform(1:3, 4) = point + [0; 0; 0.15] - ...
    cfg.tool.TipInEndoscopeM(:);
end

function status = enabledStatus()
status = struct('MotionPermitted', true, ...
    'ResetDynamicState', false, 'CommandScale', 1);
end

function metrics = trailingStaticMetrics(poseIndex, timestampSec, ...
        velocity, eligible, cfg)
poses = unique(poseIndex(:)).';
perPosePivot = nan(size(poses));
perPoseInsertion = nan(size(poses));
for index = 1:numel(poses)
    candidates = find(poseIndex == poses(index) & eligible);
    if isempty(candidates)
        continue;
    end
    lastTime = timestampSec(candidates(end));
    candidates = candidates(timestampSec(candidates) >= lastTime - 1.0);
    perPosePivot(index) = sqrt(mean( ...
        sum(velocity(1:2, candidates).^2, 1)));
    perPoseInsertion(index) = sqrt(mean(velocity(3, candidates).^2));
end
valid = isfinite(perPosePivot) & isfinite(perPoseInsertion);
metrics = struct();
metrics.WindowSec = 1.0;
metrics.PoseIndices = poses(valid);
metrics.PivotRmsRadSec = perPosePivot(valid);
metrics.InsertionRmsMSec = perPoseInsertion(valid);
metrics.PivotAcceptanceRadSec = 0.05 * cfg.control.PivotMaxRadSec;
metrics.InsertionAcceptanceMSec = ...
    0.05 * cfg.control.InsertionMaxMSec;
metrics.MaximumPivotRmsRadSec = maxOrInf(metrics.PivotRmsRadSec);
metrics.MaximumInsertionRmsMSec = maxOrInf(metrics.InsertionRmsMSec);
metrics.Passed = ~isempty(metrics.PoseIndices) && ...
    metrics.MaximumPivotRmsRadSec <= metrics.PivotAcceptanceRadSec && ...
    metrics.MaximumInsertionRmsMSec <= ...
        metrics.InsertionAcceptanceMSec;
end

function value = maxOrInf(values)
if isempty(values)
    value = inf;
else
    value = max(values);
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
    error('scopeguide:stage05:CannotWriteReplay', ...
        'Cannot open %s: %s', filePath, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end

function value = passText(passed)
if passed
    value = 'PASS';
else
    value = 'NOT PASSED';
end
end
