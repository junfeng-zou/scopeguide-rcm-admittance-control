function [report, outputDirectory] = ...
        run_force_deadzone_identification(options)
%RUN_FORCE_DEADZONE_IDENTIFICATION Identify force/moment deadzone candidates.
% The operator moves the robot with the teach pendant. MATLAB only reads
% robot feedback and the HEX-H; it never sends a motion, enable, reset,
% zero, or unzero command.

arguments
    options.Config = struct([])
    options.SensorConfig = struct([])
    options.CalibrationFile (1, 1) string = ""
    options.PoseCount (1, 1) double = 8
    options.CaptureReturnPose (1, 1) logical = true
    options.DurationPerPoseSec (1, 1) double = 60
    options.FilterSettleSec (1, 1) double = 2
    options.SampleRateHz (1, 1) double = 200
    options.StatusRateHz (1, 1) double = 20
    options.StableHoldSec (1, 1) double = 1
    options.StationaryTimeoutSec (1, 1) double = 60
    options.MaximumStationaryJointSpeedDegSec (1, 1) double = 0.15
    options.MaximumStationaryTcpTranslationSpeedMSec (1, 1) double = 2e-4
    options.MaximumStationaryTcpRotationSpeedRadSec (1, 1) double = deg2rad(0.20)
    options.RequireRobotEnabled (1, 1) logical = true
    options.RequireTypedConfirmation (1, 1) logical = true
    options.Percentile (1, 1) double = 99.5
    options.ForceMarginN (1, 1) double = 0.15
    options.MomentMarginNm (1, 1) double = 0.015
    options.MinimumEligibleRatio (1, 1) double = 0.90
end

validateOptions(options);
projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'force_sensor'));
addpath(fullfile(projectRoot, 'force_calibration'));
addpath(fullfile(projectRoot, 'robot'));

if isempty(options.Config)
    cfg = defaultRcmAdmittanceConfig();
else
    cfg = options.Config;
end
cfg.runtime.Mode = "live_dry_run";
cfg.robot.EnableMotion = false;
cfg.robot.DryRun = true;
cfg.sensor.SampleRateHz = options.SampleRateHz;
cfg.force.AutomaticBaselineEnabled = false;
if strlength(options.CalibrationFile) > 0
    cfg.force.CalibrationFile = options.CalibrationFile;
end
validateRcmAdmittanceConfig(cfg);
if ~isfile(cfg.force.CalibrationFile)
    error('scopeguide:force:DeadzoneCalibrationMissing', ...
        'Calibration file does not exist: %s', cfg.force.CalibrationFile);
end

if isempty(options.SensorConfig)
    sensorCfg = onrobot.defaultConfig();
else
    sensorCfg = options.SensorConfig;
end
sensorCfg.SampleRateHz = options.SampleRateHz;

outputDirectory = createOutputDirectory(projectRoot);
paths = outputPaths(outputDirectory);
writeJson(paths.OptionsJson, jsonSafeOptions(options));
writeJson(paths.ConfigJson, cfg);
writeJson(paths.SensorConfigJson, sensorCfg);

robot = scopeguide.io.RobotAdapter(cfg);
forceSource = scopeguide.io.ForceSensorAdapter( ...
    cfg, sensorCfg, [], StatusRateHz=options.StatusRateHz);
cleanup = onCleanup(@() disconnectHardware(forceSource, robot));
allData = table();
report = initialReport(cfg, sensorCfg, options, paths);

try
    printSafetyBanner(cfg, options);
    requireConfirmation(options, 'IDENTIFY DEADZONE', ...
        ['Type "IDENTIFY DEADZONE" only after the robot is thermally ' ...
         'stable, the endoscope is unloaded, cables are slack, and the ' ...
         'complete swept volume is clear: ']);

    fprintf('Connecting to robot feedback at %s ...\n', cfg.robot.IPAddress);
    robot.connectReadOnly();
    fprintf('Connecting to HEX-H at %s:%d ...\n', ...
        sensorCfg.IPAddress, sensorCfg.Port);
    forceSource.connect();
    if forceSource.DeviceStatus ~= 0
        error('scopeguide:force:DeadzoneSensorStatus', ...
            'HEX-H status is %d; aborting.', forceSource.DeviceStatus);
    end

    captureCount = options.PoseCount + double(options.CaptureReturnPose);
    sessionClock = tic;
    for captureIndex = 1:captureCount
        isReturn = options.CaptureReturnPose && ...
            captureIndex == captureCount;
        if isReturn
            poseLabel = "pose01_return";
            fprintf(['\n=== Return capture %d / %d ===\n' ...
                'Return as closely as practical to the first pose.\n'], ...
                captureIndex, captureCount);
        else
            poseLabel = "pose" + compose('%02d', captureIndex);
            fprintf('\n=== Static pose %d / %d ===\n', ...
                captureIndex, options.PoseCount);
            if captureIndex == 1
                fprintf(['Choose a representative reference pose and ' ...
                    'remember it for the final return check.\n']);
            else
                fprintf(['Use a distinct, collision-free orientation ' ...
                    'inside the intended operating workspace.\n']);
            end
        end
        fprintf(['Move only with the teach pendant. Keep the endoscope ' ...
            'unloaded and all cables slack.\n']);
        expected = sprintf('SAMPLE %d', captureIndex);
        requireConfirmation(options, expected, sprintf( ...
            'Type "%s" after the robot and cable have settled: ', expected));
        waitUntilStationary(robot, options);

        forceSource.Pipeline.startNewSegment();
        poseData = capturePose(forceSource, robot, captureIndex, ...
            poseLabel, sessionClock, options);
        allData = [allData; poseData]; %#ok<AGROW>
        checkpoint(allData, report, cfg, sensorCfg, options, paths);
        eligible = poseData.AnalysisEligible;
        fprintf(['Captured %d samples; eligible %d (%.2f%%); ' ...
            'actual %.1f Hz.\n'], height(poseData), nnz(eligible), ...
            100 * mean(eligible), poseData.ActualCaptureRateHz(1));
    end

    minimumSamples = max(100, floor(0.25 * ...
        options.DurationPerPoseSec * options.SampleRateHz));
    report = scopeguide.force.estimateDeadzoneThresholds(allData, ...
        Percentile=options.Percentile, ...
        ForceMarginN=options.ForceMarginN, ...
        MomentMarginNm=options.MomentMarginNm, ...
        CurrentForceDeadzoneN=cfg.force.ForceDeadzoneN, ...
        CurrentMomentDeadzoneNm=cfg.force.MomentDeadzoneNm, ...
        MinimumEligibleRatio=options.MinimumEligibleRatio, ...
        MinimumEligibleSamples=minimumSamples);
    report.CalibrationFile = string(cfg.force.CalibrationFile);
    report.AutomaticBaselineEnabled = cfg.force.AutomaticBaselineEnabled;
    report.SampleRateTargetHz = options.SampleRateHz;
    report.CaptureReturnPose = options.CaptureReturnPose;
    if options.CaptureReturnPose
        report.ReturnCheck = returnCheck(allData, captureCount);
    else
        report.ReturnCheck = struct();
    end
    report.Paths = paths;
    report.CompletedLocal = string(datetime('now'));
    report.PhysicalMotionCommandSent = false;
    report.SensorZeroCommandSent = false;

    writetable(report.PoseStatistics, paths.PoseStatisticsCsv);
    writetable(report.Excursions, paths.ExcursionsCsv);
    createSummaryFigure(allData, report, paths);
    checkpoint(allData, report, cfg, sensorCfg, options, paths);
    writeJson(paths.SummaryJson, jsonSafeReport(report));

    fprintf('\nDeadzone identification completed.\n');
    fprintf('Current force/moment deadzone: %.3f N / %.4f N*m\n', ...
        report.CurrentForceDeadzoneN, report.CurrentMomentDeadzoneNm);
    fprintf('Candidate force/moment deadzone: %.3f N / %.4f N*m\n', ...
        report.CandidateForceDeadzoneN, ...
        report.CandidateMomentDeadzoneNm);
    fprintf('Suggested activation hold: %.0f ms\n', ...
        1000 * report.SuggestedActivationHoldSec);
    fprintf('Result validity: %s\n', report.Status);
    fprintf('Results: %s\n', outputDirectory);
catch exception
    report.Status = "failed";
    report.Valid = false;
    report.ErrorIdentifier = string(exception.identifier);
    report.ErrorMessage = string(exception.message);
    try
        checkpoint(allData, report, cfg, sensorCfg, options, paths);
        writeJson(paths.SummaryJson, jsonSafeReport(report));
    catch
    end
    rethrow(exception);
end
clear cleanup;
end

function validateOptions(options)
integerNames = {'PoseCount'};
for index = 1:numel(integerNames)
    value = options.(integerNames{index});
    if value < 2 || value ~= round(value)
        error('scopeguide:force:InvalidDeadzoneCaptureOption', ...
            '%s must be an integer of at least 2.', integerNames{index});
    end
end
positiveNames = {'DurationPerPoseSec', 'SampleRateHz', 'StatusRateHz', ...
    'StableHoldSec', 'MaximumStationaryJointSpeedDegSec', ...
    'StationaryTimeoutSec', ...
    'MaximumStationaryTcpTranslationSpeedMSec', ...
    'MaximumStationaryTcpRotationSpeedRadSec'};
for index = 1:numel(positiveNames)
    value = options.(positiveNames{index});
    if ~isscalar(value) || ~isfinite(value) || value <= 0
        error('scopeguide:force:InvalidDeadzoneCaptureOption', ...
            '%s must be finite and positive.', positiveNames{index});
    end
end
if options.FilterSettleSec < 0 || ~isfinite(options.FilterSettleSec) || ...
        options.Percentile <= 50 || options.Percentile >= 100 || ...
        options.ForceMarginN < 0 || options.MomentMarginNm < 0 || ...
        options.MinimumEligibleRatio <= 0 || ...
        options.MinimumEligibleRatio > 1
    error('scopeguide:force:InvalidDeadzoneCaptureOption', ...
        'Invalid settle, percentile, margin, or eligible-ratio option.');
end
end

function printSafetyBanner(cfg, options)
fprintf('\n============================================================\n');
fprintf('FORCE / MOMENT DEADZONE IDENTIFICATION (READ ONLY)\n');
fprintf('Calibration: %s\n', cfg.force.CalibrationFile);
fprintf('Unique poses: %d | each analyzed for %.1f s\n', ...
    options.PoseCount, options.DurationPerPoseSec);
fprintf('Required checks:\n');
fprintf('  1. Robot is enabled, thermally stable, and in teach mode.\n');
fprintf('  2. Complete endoscope payload is installed and unloaded.\n');
fprintf('  3. Endoscope is outside every patient, phantom, and trocar.\n');
fprintf('  4. Cables are slack and the swept volume is clear.\n');
fprintf('  5. No one touches the robot during a static capture.\n');
fprintf('MATLAB sends no motion/enable/reset/zero/unzero command.\n');
fprintf('Automatic residual baseline is forced OFF.\n');
fprintf('============================================================\n\n');
end

function requireConfirmation(options, expected, prompt)
if options.RequireTypedConfirmation
    response = input(prompt, 's');
    if ~strcmp(response, expected)
        error('scopeguide:force:DeadzoneIdentificationCancelled', ...
            'Operator cancelled before confirmation "%s".', expected);
    end
else
    input('Press Enter to continue.', 's');
end
end

function waitUntilStationary(robot, options)
stableStart = [];
waitClock = tic;
while true
    state = robot.readState();
    if options.RequireRobotEnabled && state.RobotMode ~= "ENABLE"
        error('scopeguide:force:DeadzoneRobotNotEnabled', ...
            'RobotMode must remain ENABLE; current mode is %s.', ...
            state.RobotMode);
    end
    stationary = isStationary(state, options);
    if state.IsValid && stationary
        if isempty(stableStart)
            stableStart = tic;
        elseif toc(stableStart) >= options.StableHoldSec
            return;
        end
    else
        stableStart = [];
    end
    if toc(waitClock) >= options.StationaryTimeoutSec
        error('scopeguide:force:DeadzoneStationaryTimeout', ...
            ['Robot feedback did not remain valid and stationary for ' ...
             '%.1f s within the %.1f s timeout.'], ...
            options.StableHoldSec, options.StationaryTimeoutSec);
    end
    pause(0.02);
end
end

function stationary = isStationary(state, options)
stationary = state.IsValid && ...
    max(abs(state.JointVelocityRadSec)) <= ...
        deg2rad(options.MaximumStationaryJointSpeedDegSec) && ...
    max(abs(state.ActualTcpTwistBase(1:3))) <= ...
        options.MaximumStationaryTcpTranslationSpeedMSec && ...
    max(abs(state.ActualTcpTwistBase(4:6))) <= ...
        options.MaximumStationaryTcpRotationSpeedRadSec;
end

function data = capturePose(forceSource, robot, poseIndex, poseLabel, ...
        sessionClock, options)
periodSec = 1 / options.SampleRateHz;
totalDurationSec = options.FilterSettleSec + options.DurationPerPoseSec;
capacity = ceil(1.05 * totalDurationSec * options.SampleRateHz) + 10;
sampleTimeSec = zeros(capacity, 1);
captureElapsedSec = zeros(capacity, 1);
sensorSequence = zeros(capacity, 1, 'uint64');
sensorTimeSec = zeros(capacity, 1);
completeReadDurationSec = zeros(capacity, 1);
deviceStatus = nan(capacity, 1);
qualityValid = false(capacity, 1);
qualityStatus = strings(capacity, 1);
robotStateValid = false(capacity, 1);
robotStationary = false(capacity, 1);
robotMode = strings(capacity, 1);
analysisEligible = false(capacity, 1);
safetyWarning = false(capacity, 1);
safetyStop = false(capacity, 1);
quaternion = nan(capacity, 4);
jointsRad = nan(capacity, 6);
raw = nan(capacity, 6);
external = nan(capacity, 6);
fast = nan(capacity, 6);
slow = nan(capacity, 6);
control = zeros(capacity, 6);

captureClock = tic;
nextSampleSec = 0;
count = 0;
nextProgressSec = 10;
while toc(captureClock) < totalDurationSec
    waitUntil(captureClock, nextSampleSec);
    frame = forceSource.readProcessed(robot);
    elapsed = toc(captureClock);
    count = count + 1;
    if count > capacity
        grow = max(100, ceil(0.1 * capacity));
        [sampleTimeSec, captureElapsedSec, sensorSequence, ...
            sensorTimeSec, completeReadDurationSec, deviceStatus, ...
            qualityValid, qualityStatus, robotStateValid, ...
            robotStationary, robotMode, analysisEligible, ...
            safetyWarning, safetyStop, quaternion, jointsRad, raw, ...
            external, fast, slow, control] = growArrays(grow, ...
            sampleTimeSec, captureElapsedSec, sensorSequence, ...
            sensorTimeSec, completeReadDurationSec, deviceStatus, ...
            qualityValid, qualityStatus, robotStateValid, ...
            robotStationary, robotMode, analysisEligible, ...
            safetyWarning, safetyStop, quaternion, jointsRad, raw, ...
            external, fast, slow, control);
        capacity = capacity + grow;
    end

    processed = frame.Processed;
    robotState = frame.RobotState;
    if options.RequireRobotEnabled && robotState.RobotMode ~= "ENABLE"
        error('scopeguide:force:DeadzoneRobotNotEnabled', ...
            'RobotMode changed to %s during capture.', ...
            robotState.RobotMode);
    end
    stationary = isStationary(robotState, options);
    sampleTimeSec(count) = toc(sessionClock);
    captureElapsedSec(count) = elapsed;
    sensorSequence(count) = frame.RawSensorSample.sequence;
    sensorTimeSec(count) = frame.RawSensorSample.monotonicTime;
    completeReadDurationSec(count) = frame.CompleteReadDurationSec;
    deviceStatus(count) = frame.DeviceStatus;
    qualityValid(count) = processed.Quality.Valid;
    qualityStatus(count) = string(processed.Quality.StatusCode);
    robotStateValid(count) = robotState.IsValid;
    robotStationary(count) = stationary;
    robotMode(count) = robotState.RobotMode;
    safetyWarning(count) = processed.Safety.WarningActive;
    safetyStop(count) = processed.Safety.StopRequested;
    quaternion(count, :) = ...
        robotState.QuaternionBaseControllerWxyz(:).';
    jointsRad(count, :) = robotState.JointPositionRad(:).';
    raw(count, :) = processed.RawWrenchSensor(:).';
    external(count, :) = ...
        processed.ExternalWrenchToolAtSensorOrigin(:).';
    fast(count, :) = processed.FastWrenchToolAtSensorOrigin(:).';
    slow(count, :) = processed.SlowWrenchToolAtSensorOrigin(:).';
    control(count, :) = ...
        [processed.ControlForceTool(:); ...
         processed.ControlMomentForDiagnostics(:)].';
    analysisEligible(count) = elapsed >= options.FilterSettleSec && ...
        processed.Quality.Valid && robotState.IsValid && stationary && ...
        all(isfinite([external(count, :), slow(count, :)]));

    if elapsed >= nextProgressSec
        fprintf('  %.0f / %.0f s\n', elapsed, totalDurationSec);
        nextProgressSec = nextProgressSec + 10;
    end
    nextSampleSec = nextSampleSec + periodSec;
    if toc(captureClock) > nextSampleSec + periodSec
        nextSampleSec = toc(captureClock) + periodSec;
    end
end

indices = 1:count;
actualRateHz = count / max(captureElapsedSec(count), eps);
poseIndices = repmat(poseIndex, count, 1);
poseLabels = repmat(string(poseLabel), count, 1);
actualRates = repmat(actualRateHz, count, 1);
data = table(poseIndices, poseLabels, sampleTimeSec(indices), ...
    captureElapsedSec(indices), sensorSequence(indices), ...
    sensorTimeSec(indices), completeReadDurationSec(indices), ...
    deviceStatus(indices), qualityValid(indices), qualityStatus(indices), ...
    robotStateValid(indices), robotStationary(indices), robotMode(indices), ...
    analysisEligible(indices), safetyWarning(indices), safetyStop(indices), ...
    actualRates, ...
    'VariableNames', {'PoseIndex', 'PoseLabel', 'SampleTimeSec', ...
    'CaptureElapsedSec', 'SensorSequence', 'SensorTimeSec', ...
    'CompleteReadDurationSec', 'DeviceStatus', 'QualityValid', ...
    'QualityStatus', 'RobotStateValid', 'RobotStationary', 'RobotMode', ...
    'AnalysisEligible', 'SafetyWarning', 'SafetyStop', ...
    'ActualCaptureRateHz'});
axisNames = {'x', 'y', 'z'};
for axis = 1:3
    suffix = axisNames{axis};
    data.(['QuaternionQ' suffix]) = quaternion(indices, axis + 1);
    data.(['RawF' suffix]) = raw(indices, axis);
    data.(['RawT' suffix]) = raw(indices, axis + 3);
    data.(['ExternalF' suffix]) = external(indices, axis);
    data.(['ExternalT' suffix]) = external(indices, axis + 3);
    data.(['FastF' suffix]) = fast(indices, axis);
    data.(['FastT' suffix]) = fast(indices, axis + 3);
    data.(['SlowF' suffix]) = slow(indices, axis);
    data.(['SlowT' suffix]) = slow(indices, axis + 3);
    data.(['ControlF' suffix]) = control(indices, axis);
    data.(['DiagnosticT' suffix]) = control(indices, axis + 3);
end
data.QuaternionQw = quaternion(indices, 1);
for joint = 1:6
    data.(sprintf('Joint%dRad', joint)) = jointsRad(indices, joint);
end
end

function varargout = growArrays(grow, varargin)
varargout = cell(size(varargin));
for index = 1:numel(varargin)
    value = varargin{index};
    if isstring(value)
        addition = strings(grow, size(value, 2));
    elseif islogical(value)
        addition = false(grow, size(value, 2));
    elseif isa(value, 'uint64')
        addition = zeros(grow, size(value, 2), 'uint64');
    else
        addition = nan(grow, size(value, 2), class(value));
    end
    varargout{index} = [value; addition];
end
end

function waitUntil(timerObject, targetTime)
while true
    remaining = targetTime - toc(timerObject);
    if remaining <= 0
        return;
    elseif remaining > 0.003
        pause(remaining - 0.001);
    else
        pause(0);
    end
end
end

function check = returnCheck(data, returnPoseIndex)
firstRows = data.PoseIndex == 1 & data.AnalysisEligible;
returnRows = data.PoseIndex == returnPoseIndex & data.AnalysisEligible;
firstExternal = wrenchColumns(data, "External", firstRows);
returnExternal = wrenchColumns(data, "External", returnRows);
firstSlow = wrenchColumns(data, "Slow", firstRows);
returnSlow = wrenchColumns(data, "Slow", returnRows);
check = struct();
check.FirstPoseExternalMean = mean(firstExternal, 1);
check.ReturnPoseExternalMean = mean(returnExternal, 1);
check.ExternalMeanChange = ...
    check.ReturnPoseExternalMean - check.FirstPoseExternalMean;
check.ExternalForceChangeNormN = norm(check.ExternalMeanChange(1:3));
check.ExternalMomentChangeNormNm = norm(check.ExternalMeanChange(4:6));
check.FirstPoseSlowMean = mean(firstSlow, 1);
check.ReturnPoseSlowMean = mean(returnSlow, 1);
check.SlowMeanChange = check.ReturnPoseSlowMean - check.FirstPoseSlowMean;
check.SlowForceChangeNormN = norm(check.SlowMeanChange(1:3));
check.SlowMomentChangeNormNm = norm(check.SlowMeanChange(4:6));
end

function wrench = wrenchColumns(data, prefix, rows)
forceNames = prefix + ["Fx", "Fy", "Fz"];
momentNames = prefix + ["Tx", "Ty", "Tz"];
names = [forceNames, momentNames];
wrench = zeros(nnz(rows), 6);
for index = 1:6
    wrench(:, index) = data.(names(index))(rows);
end
end

function createSummaryFigure(data, report, paths)
eligible = data.AnalysisEligible;
t = data.SampleTimeSec;
slowForce = [data.SlowFx, data.SlowFy, data.SlowFz];
slowMoment = [data.SlowTx, data.SlowTy, data.SlowTz];
forceNorm = vecnorm(slowForce, 2, 2);
momentNorm = vecnorm(slowMoment, 2, 2);

figureHandle = figure('Visible', 'off', 'Color', 'w', ...
    'Position', [80, 80, 1500, 900]);
layout = tiledlayout(figureHandle, 2, 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf(['Deadzone identification: Q_{%.1f} + margin, ' ...
    'candidate %.2f N / %.3f N m'], report.Percentile, ...
    report.CandidateForceDeadzoneN, report.CandidateMomentDeadzoneNm));

forceAxes = nexttile(layout, 1);
plot(forceAxes, t(eligible), forceNorm(eligible), ...
    'Color', [0.05, 0.35, 0.80], 'LineWidth', 0.7);
hold(forceAxes, 'on');
yline(forceAxes, report.CurrentForceDeadzoneN, '--k', 'Current');
yline(forceAxes, report.CandidateForceDeadzoneN, '--r', 'Candidate');
ylabel(forceAxes, '|F_{slow}| [N]');
grid(forceAxes, 'on');

momentAxes = nexttile(layout, 2);
plot(momentAxes, t(eligible), momentNorm(eligible), ...
    'Color', [0.85, 0.25, 0.05], 'LineWidth', 0.7);
hold(momentAxes, 'on');
yline(momentAxes, report.CurrentMomentDeadzoneNm, '--k', 'Current');
yline(momentAxes, report.CandidateMomentDeadzoneNm, '--r', 'Candidate');
xlabel(momentAxes, 'Session time [s]');
ylabel(momentAxes, '|T_{slow}| [N m]');
grid(momentAxes, 'on');

exportgraphics(figureHandle, paths.FigurePng, 'Resolution', 180);
savefig(figureHandle, paths.FigureFig);
close(figureHandle);
end

function checkpoint(data, report, cfg, sensorCfg, options, paths)
writetable(data, paths.SignalsCsv);
save(paths.ResultsMat, 'data', 'report', 'cfg', 'sensorCfg', 'options');
end

function outputDirectory = createOutputDirectory(projectRoot)
timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
outputDirectory = fullfile(projectRoot, 'results', ...
    "force_deadzone_identification_" + timestamp);
[created, message] = mkdir(outputDirectory);
if ~created && ~isfolder(outputDirectory)
    error('scopeguide:force:DeadzoneOutputDirectory', ...
        'Cannot create %s: %s', outputDirectory, message);
end
end

function paths = outputPaths(directory)
paths = struct();
paths.SignalsCsv = string(fullfile(directory, 'signals.csv'));
paths.PoseStatisticsCsv = string(fullfile(directory, 'pose_statistics.csv'));
paths.ExcursionsCsv = string(fullfile(directory, 'excursions.csv'));
paths.ResultsMat = string(fullfile(directory, 'deadzone_identification.mat'));
paths.SummaryJson = string(fullfile(directory, 'summary.json'));
paths.ConfigJson = string(fullfile(directory, 'config_snapshot.json'));
paths.SensorConfigJson = string(fullfile(directory, ...
    'sensor_config_snapshot.json'));
paths.OptionsJson = string(fullfile(directory, 'options.json'));
paths.FigurePng = string(fullfile(directory, 'deadzone_identification.png'));
paths.FigureFig = string(fullfile(directory, 'deadzone_identification.fig'));
end

function report = initialReport(cfg, sensorCfg, options, paths)
report = struct();
report.Status = "initialized";
report.Valid = false;
report.CreatedLocal = string(datetime('now'));
report.CalibrationFile = string(cfg.force.CalibrationFile);
report.SensorIPAddress = string(sensorCfg.IPAddress);
report.Options = jsonSafeOptions(options);
report.Paths = paths;
report.PhysicalMotionCommandSent = false;
report.SensorZeroCommandSent = false;
end

function safe = jsonSafeOptions(options)
safe = options;
if isfield(safe, 'Config')
    safe = rmfield(safe, 'Config');
end
if isfield(safe, 'SensorConfig')
    safe = rmfield(safe, 'SensorConfig');
end
end

function safe = jsonSafeReport(report)
safe = report;
if isfield(safe, 'PoseStatistics') && istable(safe.PoseStatistics)
    safe.PoseStatistics = table2struct(safe.PoseStatistics);
end
if isfield(safe, 'Excursions') && istable(safe.Excursions)
    safe.Excursions = table2struct(safe.Excursions);
end
end

function writeJson(path, payload)
try
    text = jsonencode(payload, 'PrettyPrint', true);
catch
    text = jsonencode(payload);
end
file = fopen(path, 'w');
if file < 0
    error('scopeguide:force:DeadzoneJsonWrite', ...
        'Cannot open %s for writing.', path);
end
guard = onCleanup(@() fclose(file));
fwrite(file, text, 'char');
end

function disconnectHardware(forceSource, robot)
try
    forceSource.disconnect();
catch
end
try
    robot.disconnect();
catch
end
end
