function [stats, outputDirectory] = ...
        run_processed_force_live_monitor(options)
%RUN_PROCESSED_FORCE_LIVE_MONITOR Display a selected HEX-H wrench branch.
% The function reads the HEX-H and Nova5 feedback, but never calls sensor
% zero/unzero, robot Enable, ServoJ, MovJ, MovL or any motion command.

arguments
    options.Config = struct([])
    options.SensorConfig = struct([])
    options.DurationSec (1, 1) double = inf
    options.WindowSec (1, 1) double {mustBeFinite, mustBePositive} = 10
    options.PlotRateHz (1, 1) double {mustBeFinite, mustBePositive} = 10
    options.StatusRateHz (1, 1) double {mustBeFinite, mustBePositive} = 20
    options.WriteResults (1, 1) logical = true
    options.EnableAdaptiveFzBaseline (1, 1) logical = true
    options.MaximumStationaryJointSpeedDegSec (1, 1) double ...
        {mustBeFinite, mustBePositive} = 2.0
    options.MaximumStationaryTcpTranslationSpeedMmSec (1, 1) double ...
        {mustBeFinite, mustBePositive} = 3.0
    options.MaximumStationaryTcpRotationSpeedDegSec (1, 1) double ...
        {mustBeFinite, mustBePositive} = 2.0
    options.DisplaySignal (1, 1) string ...
        {mustBeMember(options.DisplaySignal, ...
        ["external", "fast", "slow", "control"])} = "control"
end

if ~(isfinite(options.DurationSec) && options.DurationSec > 0 || ...
        isinf(options.DurationSec) && options.DurationSec > 0)
    error('scopeguide:force:InvalidMonitorDuration', ...
        'DurationSec must be positive or Inf.');
end
projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'force_sensor'));
addpath(fullfile(projectRoot, 'force_calibration'));
addpath(fullfile(projectRoot, 'robot'));

if isempty(options.Config)
    cfg = defaultRcmAdmittanceConfig();
    cfg.runtime.Mode = "live_dry_run";
else
    cfg = options.Config;
end
cfg.force.AutomaticBaselineEnabled = ...
    options.EnableAdaptiveFzBaseline;
validateRcmAdmittanceConfig(cfg);
if cfg.runtime.Mode ~= "live_dry_run" || ...
        cfg.robot.EnableMotion || ~cfg.robot.DryRun
    error('scopeguide:force:LiveReadOnlyConfigurationRequired', ...
        ['The processed-force monitor requires live_dry_run, ' ...
        'EnableMotion=false and DryRun=true.']);
end
if isempty(options.SensorConfig)
    sensorCfg = onrobot.defaultConfig();
else
    sensorCfg = options.SensorConfig;
end

robot = scopeguide.io.RobotAdapter(cfg);
forceSource = scopeguide.io.ForceSensorAdapter( ...
    cfg, sensorCfg, [], StatusRateHz=options.StatusRateHz);
cleanup = onCleanup(@() disconnectHardware(forceSource, robot));

fprintf('Connecting to Nova5 feedback at %s ...\n', ...
    cfg.robot.IPAddress);
robot.connectReadOnly();
fprintf('Connecting to OnRobot HEX-H at %s:%d ...\n', ...
    sensorCfg.IPAddress, sensorCfg.Port);
forceSource.connect();
fprintf(['Connected. The monitor will not issue a HEX-H hardware-zero ' ...
    'command and will not ' ...
    'send robot commands.\n']);
if options.EnableAdaptiveFzBaseline
    fprintf([ ...
        'Startup software zero is enabled. Keep the robot still and do ' ...
        'not touch the endoscope for %.1f s.\n'], ...
        cfg.force.Baseline.StartupDurationSec);
    fprintf([ ...
        'Baseline stationary gates: joint <= %.2f deg/s, TCP <= %.2f ' ...
        'mm/s and %.2f deg/s.\n'], ...
        options.MaximumStationaryJointSpeedDegSec, ...
        options.MaximumStationaryTcpTranslationSpeedMmSec, ...
        options.MaximumStationaryTcpRotationSpeedDegSec);
    fprintf([ ...
        'After zeroing: button OFF = idle/Fz tracking; button ON = ' ...
        'manual operation/baseline frozen. E toggles, Esc turns OFF.\n']);
    fprintf([ ...
        'This software operation switch does NOT enable the robot or ' ...
        'send motion commands.\n']);
end

if options.WriteResults
    outputDirectory = createOutputDirectory(projectRoot);
    csvFile = fullfile(outputDirectory, 'signals.csv');
    [fileId, message] = fopen(csvFile, 'w');
    if fileId < 0
        error('scopeguide:force:CannotOpenLiveLog', ...
            'Cannot open %s: %s', csvFile, message);
    end
    csvCleanup = onCleanup(@() fclose(fileId));
    writeCsvHeader(fileId);
else
    outputDirectory = "";
    fileId = -1;
    csvCleanup = onCleanup(@() []);
end

plotter = scopeguide.force.ProcessedWrenchPlotter( ...
    options.WindowSec, DisplaySignal=options.DisplaySignal);
periodSec = 1 / cfg.sensor.SampleRateHz;
plotPeriodSec = 1 / options.PlotRateHz;
loopClock = tic;
nextSampleSec = 0;
nextPlotSec = 0;
sampleCount = 0;
robotInvalidCount = 0;
deadlineMissCount = 0;
readDurationSumSec = 0;
maximumReadDurationSec = 0;
warningSampleCount = 0;
stopSampleCount = 0;
qualityInvalidCount = 0;
maximumControlForceNormN = 0;
maximumDiagnosticMomentNormNm = 0;
observedWarningReasons = strings(0, 1);
observedStopReasons = strings(0, 1);
lastOperationEnabled = false;
operationEnableTransitionCount = 0;

while plotter.isOpen() && toc(loopClock) < options.DurationSec
    nowSec = toc(loopClock);
    if nowSec < nextSampleSec
        pause(min(nextSampleSec - nowSec, 0.001));
        continue;
    end

    operationEnabled = plotter.isOperationEnabled();
    if operationEnabled ~= lastOperationEnabled
        operationEnableTransitionCount = ...
            operationEnableTransitionCount + 1;
        lastOperationEnabled = operationEnabled;
    end
    contextProvider = @(robotState) monitorForceContext( ...
        robotState, operationEnabled, ...
        options.MaximumStationaryJointSpeedDegSec, ...
        options.MaximumStationaryTcpTranslationSpeedMmSec, ...
        options.MaximumStationaryTcpRotationSpeedDegSec);
    frame = forceSource.readProcessed(robot, contextProvider);
    sensorSample = frame.RawSensorSample;
    robotState = frame.RobotState;
    processed = frame.Processed;
    completeReadDurationSec = frame.CompleteReadDurationSec;
    deviceStatus = frame.DeviceStatus;
    robotInvalidCount = robotInvalidCount + ~robotState.IsValid;

    sampleCount = sampleCount + 1;
    readDurationSumSec = readDurationSumSec + completeReadDurationSec;
    maximumReadDurationSec = max( ...
        maximumReadDurationSec, completeReadDurationSec);
    qualityInvalidCount = qualityInvalidCount + ~processed.Quality.Valid;
    warningSampleCount = warningSampleCount + ...
        processed.Safety.WarningActive;
    stopSampleCount = stopSampleCount + processed.Safety.StopRequested;
    observedWarningReasons = unique([observedWarningReasons; ...
        processed.Safety.WarningReasons], 'stable');
    observedStopReasons = unique([observedStopReasons; ...
        processed.Safety.StopReasons], 'stable');
    maximumControlForceNormN = max(maximumControlForceNormN, ...
        norm(processed.ControlForceTool));
    maximumDiagnosticMomentNormNm = max( ...
        maximumDiagnosticMomentNormNm, ...
        norm(processed.ControlMomentForDiagnostics));

    if fileId >= 0
        writeCsvRow(fileId, toc(loopClock), sensorSample, ...
            completeReadDurationSec, deviceStatus, robotState, processed, ...
            operationEnabled);
    end
    if toc(loopClock) >= nextPlotSec
        plotter.update(sensorSample.monotonicTime, processed, robotState);
        nextPlotSec = toc(loopClock) + plotPeriodSec;
    end

    nextSampleSec = nextSampleSec + periodSec;
    if toc(loopClock) > nextSampleSec + periodSec
        deadlineMissCount = deadlineMissCount + 1;
        nextSampleSec = toc(loopClock) + periodSec;
    end
end

elapsedSec = toc(loopClock);
stats = struct();
stats.Status = "processed_force_live_monitor_complete";
stats.SampleCount = sampleCount;
stats.ElapsedSec = elapsedSec;
stats.ActualRateHz = sampleCount / max(elapsedSec, eps);
stats.TargetRateHz = cfg.sensor.SampleRateHz;
stats.DeviceStatusRateHz = options.StatusRateHz;
stats.DeviceStatusReadCount = double(forceSource.StatusReadCount);
stats.DeadlineMissCount = deadlineMissCount;
stats.MeanCompleteReadDurationSec = ...
    readDurationSumSec / max(sampleCount, 1);
stats.MaximumCompleteReadDurationSec = maximumReadDurationSec;
stats.QualityInvalidCount = qualityInvalidCount;
stats.RobotInvalidCount = robotInvalidCount;
stats.WarningSampleCount = warningSampleCount;
stats.StopSampleCount = stopSampleCount;
stats.ObservedWarningReasons = observedWarningReasons.';
stats.ObservedStopReasons = observedStopReasons.';
stats.MaximumControlForceNormN = maximumControlForceNormN;
stats.MaximumDiagnosticMomentNormNm = ...
    maximumDiagnosticMomentNormNm;
stats.PipelineCounters = forceSource.Pipeline.Counters;
stats.RobotCommandAttemptCount = double(robot.CommandAttemptCount);
stats.RobotCommandSentCount = double(robot.CommandSentCount);
stats.PhysicalMotionCommandSent = false;
stats.SensorZeroCommandSent = false;
stats.MomentEligibleForControl = false;
stats.CalibrationFile = string(cfg.force.CalibrationFile);
stats.DisplaySignal = options.DisplaySignal;
stats.AdaptiveFzBaselineEnabled = options.EnableAdaptiveFzBaseline;
stats.OperationEnableTransitionCount = operationEnableTransitionCount;
stats.FinalOperationEnabled = plotter.isOperationEnabled();
stats.FinalBaselineReady = forceSource.Pipeline.BaselineEstimator.Ready;
stats.FinalBaselineWrenchTool = ...
    forceSource.Pipeline.BaselineEstimator.BaselineWrench;
stats.BaselineStationaryThresholds = struct( ...
    'MaximumJointSpeedDegSec', ...
        options.MaximumStationaryJointSpeedDegSec, ...
    'MaximumTcpTranslationSpeedMmSec', ...
        options.MaximumStationaryTcpTranslationSpeedMmSec, ...
    'MaximumTcpRotationSpeedDegSec', ...
        options.MaximumStationaryTcpRotationSpeedDegSec);

clear csvCleanup;
if options.WriteResults
    writeJson(fullfile(outputDirectory, 'summary.json'), stats);
    writeJson(fullfile(outputDirectory, 'config_snapshot.json'), cfg);
    writeJson(fullfile(outputDirectory, 'sensor_config_snapshot.json'), ...
        sensorCfg);
end
clear cleanup;
fprintf('Stopped. %.1f Hz, %d samples.\n', ...
    stats.ActualRateHz, stats.SampleCount);
if options.WriteResults
    fprintf('Results: %s\n', outputDirectory);
end
end

function outputDirectory = createOutputDirectory(projectRoot)
timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
outputDirectory = fullfile(projectRoot, 'results', ...
    "processed_force_live_" + timestamp);
[created, message] = mkdir(outputDirectory);
if ~created && ~isfolder(outputDirectory)
    error('scopeguide:force:CannotCreateLiveResultDirectory', ...
        'Cannot create %s: %s', outputDirectory, message);
end
end

function writeCsvHeader(fileId)
names = ["loopTimeSec", "sensorSequence", "sensorTimeSec", ...
    "sensorReadDurationSec", "completeReadDurationSec", ...
    "deviceStatus", "robotSequence", "robotAgeSec", ...
    "qualityValid", "qualityStatus", ...
    "motionInputValid", "controlEnabled", "neutralCheckPassed", ...
    "safetyWarning", ...
    "safetyStop", "operationEnabled", "baselineReady", ...
    "baselineUpdated", "baselineFx", "baselineFy", "baselineFz", ...
    "baselineTx", "baselineTy", "baselineTz", ...
    "rawFx", "rawFy", "rawFz", ...
    "rawTx", "rawTy", "rawTz", ...
    "externalFx", "externalFy", "externalFz", ...
    "externalTx", "externalTy", "externalTz", ...
    "fastFx", "fastFy", "fastFz", "fastTx", "fastTy", "fastTz", ...
    "slowFx", "slowFy", "slowFz", "slowTx", "slowTy", "slowTz", ...
    "controlFx", "controlFy", "controlFz", ...
    "diagnosticTx", "diagnosticTy", "diagnosticTz"];
fprintf(fileId, '%s\n', strjoin(names, ','));
end

function writeCsvRow(fileId, loopTimeSec, sensorSample, ...
        completeReadDurationSec, deviceStatus, robotState, processed, ...
        operationEnabled)
[baselineReady, baselineUpdated, baseline] = ...
    baselineLogValues(processed);
values = [double(loopTimeSec), double(sensorSample.sequence), ...
    double(sensorSample.monotonicTime), ...
    double(sensorSample.readDuration), double(completeReadDurationSec), ...
    double(deviceStatus), double(robotState.FeedbackSequence), ...
    double(robotState.SampleAgeSec), double(processed.Quality.Valid), ...
    double(processed.MotionInputValid), ...
    double(processed.ControlEnabled), ...
    double(processed.NeutralCheckPassed), ...
    double(processed.Safety.WarningActive), ...
    double(processed.Safety.StopRequested), ...
    double(operationEnabled), double(baselineReady), ...
    double(baselineUpdated), baseline(:).', ...
    processed.RawWrenchSensor(:).', ...
    processed.ExternalWrenchToolAtSensorOrigin(:).', ...
    processed.FastWrenchToolAtSensorOrigin(:).', ...
    processed.SlowWrenchToolAtSensorOrigin(:).', ...
    processed.ControlForceTool(:).', ...
    processed.ControlMomentForDiagnostics(:).'];
fprintf(fileId, ...
    '%.9g,%.0f,%.9g,%.9g,%.9g,%.0f,%.0f,%.9g,%.0f,%s', ...
    values(1:9), char(processed.Quality.StatusCode));
fprintf(fileId, repmat(',%.9g', 1, numel(values) - 9), values(10:end));
fprintf(fileId, '\n');
end

function context = monitorForceContext(robotState, operationEnabled, ...
        maximumJointSpeedDegSec, maximumTcpTranslationSpeedMmSec, ...
        maximumTcpRotationSpeedDegSec)
context = scopeguide.types.forceProcessingContext();
context.HandleEnabled = logical(operationEnabled);
context.RobotStationary = ...
    scopeguide.force.isRobotStationaryForBaseline(robotState, ...
    MaximumJointSpeedDegSec=maximumJointSpeedDegSec, ...
    MaximumTcpTranslationSpeedMmSec=maximumTcpTranslationSpeedMmSec, ...
    MaximumTcpRotationSpeedDegSec=maximumTcpRotationSpeedDegSec);
% In this monitor, switch OFF is the operator's explicit assertion that
% there is no intentional contact. The residual wrench gates remain active
% as a second guard against absorbing a real force.
context.NoContactConfirmed = ~logical(operationEnabled);
context.AllowBaselineUpdate = ~logical(operationEnabled);
end

function [ready, updated, baseline] = baselineLogValues(processed)
ready = false;
updated = false;
baseline = nan(6, 1);
diagnostics = processed.BaselineDiagnostics;
if ~isstruct(diagnostics) || isempty(fieldnames(diagnostics))
    return;
end
ready = logical(diagnostics.Ready);
updated = logical(diagnostics.Updated);
baseline = double(diagnostics.BaselineWrench(:));
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

function writeJson(filePath, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end
[fileId, message] = fopen(filePath, 'w');
if fileId < 0
    error('scopeguide:force:CannotWriteLiveJson', ...
        'Cannot open %s: %s', filePath, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end
