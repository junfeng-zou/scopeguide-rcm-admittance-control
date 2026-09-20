function [summary, outputDirectory] = ...
        run_stage07_live_readonly_dryrun(options)
%RUN_STAGE07_LIVE_READONLY_DRYRUN Full sensor+robot prediction chain.
% This function connects to HEX-H and robot feedback, but RobotAdapter is
% read-only and no ServoJ/MovJ/MovL/Enable command is called anywhere.

arguments
    options.Config = struct([])
    options.SensorConfig = struct([])
    options.DurationSec (1, 1) double = 600
    options.EnablePlot (1, 1) logical = true
    options.PlotRateHz (1, 1) double = 10
    options.WindowSec (1, 1) double = 15
    options.StatusRateHz (1, 1) double = 20
    options.WriteResults (1, 1) logical = true
    options.EnableAdaptiveFzBaseline (1, 1) logical = true
    options.FaultInjectionProfile (1, 1) string ...
        {mustBeMember(options.FaultInjectionProfile, ...
        ["none", "acceptance"])} = "none"
    options.MaximumStationaryJointSpeedDegSec (1, 1) double = 2.0
    options.MaximumStationaryTcpTranslationSpeedMmSec (1, 1) double = 3.0
    options.MaximumStationaryTcpRotationSpeedDegSec (1, 1) double = 2.0
end

if ~isfinite(options.DurationSec) || options.DurationSec <= 0
    error('scopeguide:stage07:InvalidDuration', ...
        'DurationSec must be finite and positive.');
end
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
cfg.force.AutomaticBaselineEnabled = ...
    options.EnableAdaptiveFzBaseline;
validateRcmAdmittanceConfig(cfg);
if ~all(isfinite(cfg.rcm.PointBaseM)) || ...
        string(cfg.rcm.PointSource) ~= "manual_base_coordinate"
    error('scopeguide:stage07:ManualRcmPointRequired', ...
        'Stage 7 requires the supplied manual robot-base pRcmBase.');
end
if isempty(options.SensorConfig)
    sensorCfg = onrobot.defaultConfig();
else
    sensorCfg = options.SensorConfig;
end

fprintf('Stage 7 READ-ONLY dry-run. pRcmBase = [%+.4f %+.4f %+.4f] m.\n', ...
    cfg.rcm.PointBaseM);
fprintf(['CalibrationValid=false and BoundsValidated=false are retained. ' ...
    'No physical motion can be authorized.\n']);
fprintf('Warming quadprog before hardware connection ...\n');
warmup = scopeguide.control.warmupRcmQpSolver(cfg);
if ~warmup.ReadyForTimingAudit
    error('scopeguide:stage07:QpWarmupFailed', ...
        'quadprog warm-up did not reach the configured timing budget.');
end

robot = scopeguide.io.RobotAdapter(cfg);
forceSource = scopeguide.io.ForceSensorAdapter( ...
    cfg, sensorCfg, [], StatusRateHz=options.StatusRateHz);
handleAdapter = scopeguide.io.EnableHandleAdapter(cfg);
controller = scopeguide.control.Stage07ReadOnlyController(cfg);
dashboard = scopeguide.ui.Stage07DryRunDashboard( ...
    cfg, options.WindowSec, EnablePlot=options.EnablePlot, ...
    FaultInjectionProfile=options.FaultInjectionProfile);
cleanup = onCleanup(@() cleanupStage07( ...
    dashboard, forceSource, robot));

if options.WriteResults
    outputDirectory = createOutputDirectory(projectRoot);
    [forceFileId, forceCleanup] = openCsv( ...
        fullfile(outputDirectory, 'force_samples.csv'), ...
        forceHeader(), 'force'); %#ok<ASGLU>
    [controlFileId, controlCleanup] = openCsv( ...
        fullfile(outputDirectory, 'control_predictions.csv'), ...
        controlHeader(), 'control'); %#ok<ASGLU>
else
    outputDirectory = "";
    forceFileId = -1;
    controlFileId = -1;
    forceCleanup = onCleanup(@() []);
    controlCleanup = onCleanup(@() []);
end

fprintf('Connecting robot feedback at %s (read-only) ...\n', ...
    cfg.robot.IPAddress);
robot.connectReadOnly();
fprintf('Connecting HEX-H at %s:%d ...\n', ...
    sensorCfg.IPAddress, sensorCfg.Port);
forceSource.connect();
fprintf(['Connected. Keep the robot still and do not touch the endoscope ' ...
    'during the %.1f s startup software zero.\n'], ...
    cfg.force.Baseline.StartupDurationSec);
fprintf(['After baseline ready, SPACE latches prediction ON and ESC turns ' ...
    'it OFF. The blue mouse pad remains hold-to-enable.\n']);
if options.EnablePlot
    fprintf('Curve plotting is enabled at %.1f Hz.\n', ...
        options.PlotRateHz);
else
    fprintf(['Curve plotting is DISABLED. The remaining small window is ' ...
        'input/status only and creates no animated curves.\n']);
end
if options.FaultInjectionProfile == "acceptance"
    printFaultAcceptanceInstructions();
end

sensorPeriodSec = 1 / cfg.sensor.SampleRateHz;
controlPeriodSec = 1 / cfg.runtime.ControlRateHz;
plotPeriodSec = 1 / options.PlotRateHz;
clock = tic;
nextSensorSec = 0;
nextControlSec = 0;
nextPlotSec = 0;
lastDecision = struct([]);
terminationReason = "DURATION_COMPLETE";
readExceptionIdentifier = "";

sensorCount = 0;
controlCount = 0;
sensorDeadlineMissCount = 0;
controlDeadlineMissCount = 0;
qualityInvalidCount = 0;
robotInvalidCount = 0;
predictionValidCount = 0;
nonzeroPredictionCount = 0;
injectedControlCount = 0;
nonzeroDuringInjectedFaultCount = 0;
fsmStates = strings(0, 1);
qpFailureCodes = strings(0, 1);
sensorLoopTime = nan(ceil(options.DurationSec * ...
    cfg.sensor.SampleRateHz * 1.1) + 100, 1);
completeReadDuration = nan(size(sensorLoopTime));
robotAge = nan(size(sensorLoopTime));
controlDt = nan(ceil(options.DurationSec * ...
    cfg.runtime.ControlRateHz * 1.1) + 100, 1);
qpSolveTime = nan(size(controlDt));
currentRcmError = nan(size(controlDt));
predictedRcmError = nan(size(controlDt));
dashboardUpdateDuration = nan(ceil(options.DurationSec * ...
    options.PlotRateHz * 1.1) + 100, 1);
dashboardUpdateCount = 0;
lastGuidanceToken = "";

while toc(clock) < options.DurationSec && dashboard.isOpen()
    nowSec = toc(clock);
    if nowSec < nextSensorSec
        pause(min(nextSensorSec - nowSec, 0.001));
        drawnow limitrate;
        continue;
    end
    guiInput = dashboard.readInput();
    if guiInput.CloseRequested
        terminationReason = "WINDOW_CLOSED";
        break;
    end
    injection = injectionAtTime( ...
        nowSec, options.FaultInjectionProfile);
    if ~injection.HandleStale
        handleAdapter.submit(guiInput.Requested, nowSec, ...
            Source=guiInput.Source, SourceValid=true, ...
            SupportsPhysicalMotion=false);
    end
    if guiInput.ResetRequested
        controller.requestReset();
    end
    contextProvider = @(robotState) stage07ForceContext( ...
        robotState, guiInput.Requested, ...
        options.MaximumStationaryJointSpeedDegSec, ...
        options.MaximumStationaryTcpTranslationSpeedMmSec, ...
        options.MaximumStationaryTcpRotationSpeedDegSec);
    try
        frame = forceSource.readProcessed(robot, contextProvider);
    catch exception
        terminationReason = "READ_EXCEPTION";
        readExceptionIdentifier = string(exception.identifier);
        handleAdapter.forceDisable(toc(clock), "READ_EXCEPTION");
        break;
    end
    afterReadSec = toc(clock);
    sensorCount = sensorCount + 1;
    ensureCapacity(sensorCount, numel(sensorLoopTime), 'sensor');
    sensorLoopTime(sensorCount) = afterReadSec;
    completeReadDuration(sensorCount) = frame.CompleteReadDurationSec;
    robotAge(sensorCount) = frame.RobotState.SampleAgeSec;
    qualityInvalidCount = qualityInvalidCount + ...
        ~frame.Processed.Quality.Valid;
    robotInvalidCount = robotInvalidCount + ~frame.RobotState.IsValid;
    if forceFileId >= 0
        writeForceRow(forceFileId, afterReadSec, frame, ...
            guiInput.Requested, injection);
    end

    if afterReadSec >= nextControlSec
        controlCount = controlCount + 1;
        ensureCapacity(controlCount, numel(controlDt), 'control');
        handleInput = handleAdapter.read(afterReadSec);
        decision = controller.step(frame.Processed, ...
            frame.RobotState, handleInput, afterReadSec, injection);
        lastDecision = decision;
        controlDt(controlCount) = decision.DtSec;
        qpSolveTime(controlCount) = decision.Qp.SolveTimeSec;
        if decision.GeometryValid
            currentRcmError(controlCount) = ...
                decision.Geometry.RcmErrorNormM;
        end
        predictedRcmError(controlCount) = ...
            decision.Qp.PredictedRcmErrorNormM;
        predictionValidCount = predictionValidCount + ...
            decision.PredictionValid;
        isNonzero = norm(decision.QdotPredictedRadSec) > 1e-12;
        nonzeroPredictionCount = nonzeroPredictionCount + isNonzero;
        injectionActive = anyInjectionActive(injection);
        injectedControlCount = injectedControlCount + injectionActive;
        nonzeroDuringInjectedFaultCount = ...
            nonzeroDuringInjectedFaultCount + ...
            (injectionActive && isNonzero);
        fsmStates(end + 1, 1) = decision.EnableStatus.State; %#ok<AGROW>
        if strlength(decision.Qp.FailureCode) > 0
            qpFailureCodes(end + 1, 1) = ...
                decision.Qp.FailureCode; %#ok<AGROW>
        end
        if controlFileId >= 0
            writeControlRow(controlFileId, afterReadSec, frame, ...
                handleInput, decision, injection);
        end
        nextControlSec = nextControlSec + controlPeriodSec;
        if afterReadSec > nextControlSec + controlPeriodSec
            controlDeadlineMissCount = controlDeadlineMissCount + 1;
            nextControlSec = afterReadSec + controlPeriodSec;
        end
    end
    if afterReadSec >= nextPlotSec && ~isempty(lastDecision)
        dashboardUpdateCount = dashboardUpdateCount + 1;
        ensureCapacity(dashboardUpdateCount, ...
            numel(dashboardUpdateDuration), 'dashboard');
        dashboardClock = tic;
        dashboard.update(afterReadSec, frame.Processed, lastDecision);
        guidance = scopeguide.runtime.stage07FaultGuidance( ...
            afterReadSec, options.FaultInjectionProfile);
        if strlength(guidance.ConsoleToken) > 0 && ...
                guidance.ConsoleToken ~= lastGuidanceToken
            fprintf('[Stage 7 故障引导] %s\n', ...
                char(guidance.MessageZh));
            lastGuidanceToken = guidance.ConsoleToken;
        end
        dashboardUpdateDuration(dashboardUpdateCount) = ...
            toc(dashboardClock);
        nextPlotSec = afterReadSec + plotPeriodSec;
    end

    nextSensorSec = nextSensorSec + sensorPeriodSec;
    if afterReadSec > nextSensorSec + sensorPeriodSec
        sensorDeadlineMissCount = sensorDeadlineMissCount + 1;
        nextSensorSec = afterReadSec + sensorPeriodSec;
    end
end

elapsedSec = toc(clock);
finalTime = elapsedSec + cfg.runtime.NominalDtSec;
handleAdapter.forceDisable(finalTime, "STAGE07_EXIT");
controller.reset();
indicesSensor = 1:sensorCount;
indicesControl = 1:controlCount;
summary = struct();
summary.Stage = 7;
summary.Status = "live_readonly_capture_complete_not_final_accepted";
summary.TerminationReason = terminationReason;
summary.ReadExceptionIdentifier = readExceptionIdentifier;
summary.DurationSec = elapsedSec;
summary.CompletedTenMinutes = elapsedSec >= 600 - controlPeriodSec;
summary.PRcmBaseM = cfg.rcm.PointBaseM;
summary.RcmPointSource = cfg.rcm.PointSource;
summary.RcmCalibrationValid = cfg.rcm.CalibrationValid;
summary.RcmBoundsValidated = cfg.rcm.BoundsValidated;
summary.SensorSampleCount = sensorCount;
summary.ControlSampleCount = controlCount;
summary.ActualSensorRateHz = sensorCount / max(elapsedSec, eps);
summary.ActualControlRateHz = controlCount / max(elapsedSec, eps);
summary.SensorDeadlineMissCount = sensorDeadlineMissCount;
summary.ControlDeadlineMissCount = controlDeadlineMissCount;
summary.QualityInvalidCount = qualityInvalidCount;
summary.RobotInvalidCount = robotInvalidCount;
summary.PredictionValidCount = predictionValidCount;
summary.NonzeroPredictionCount = nonzeroPredictionCount;
summary.ObservedFsmStates = unique(fsmStates).';
summary.ObservedQpFailureCodes = unique(qpFailureCodes).';
summary.CompleteReadDuration = statistics( ...
    completeReadDuration(indicesSensor));
summary.RobotAge = statistics(robotAge(indicesSensor));
summary.ControlDt = statistics(controlDt(indicesControl));
summary.QpSolveTime = statistics(qpSolveTime(indicesControl));
summary.CurrentRcmErrorM = statistics( ...
    currentRcmError(indicesControl));
summary.PredictedRcmErrorM = statistics( ...
    predictedRcmError(indicesControl));
summary.FaultInjectionProfile = options.FaultInjectionProfile;
summary.FaultGuidanceEnabled = ...
    options.FaultInjectionProfile == "acceptance";
summary.FaultSchedule = scopeguide.runtime.stage07FaultSchedule();
summary.PlotEnabled = options.EnablePlot;
summary.CurvePlotUpdateCount = ...
    dashboardUpdateCount * double(options.EnablePlot);
summary.DashboardStatusUpdateCount = dashboardUpdateCount;
summary.DashboardUpdateDuration = statistics( ...
    dashboardUpdateDuration(1:dashboardUpdateCount));
summary.InjectedControlSampleCount = injectedControlCount;
summary.NonzeroPredictionDuringInjectedFaultCount = ...
    nonzeroDuringInjectedFaultCount;
summary.ManualDirectionConfirmationComplete = false;
summary.QpWarmup = warmup;
summary.RobotCommandAttemptCount = double(robot.CommandAttemptCount);
summary.RobotCommandSentCount = double(robot.CommandSentCount);
summary.PhysicalMotionCommandSent = false;
summary.SensorHardwareZeroCommandSent = false;
summary.InputSupportsPhysicalMotion = false;
summary.HardwareConnectionsCreated = true;
summary.ReadOnlyConnectionsOnly = true;
summary.RequiresManualReview = true;

clear forceCleanup controlCleanup;
if options.WriteResults
    writeJson(fullfile(outputDirectory, 'summary.json'), summary);
    writeJson(fullfile(outputDirectory, 'config_snapshot.json'), cfg);
    writeJson(fullfile(outputDirectory, 'sensor_config_snapshot.json'), ...
        sensorCfg);
    writeJson(fullfile(outputDirectory, 'rcm_point_record.json'), ...
        rcmRecord(cfg));
end
fprintf('Stage 7 read-only capture stopped: %s.\n', terminationReason);
fprintf('Sensor %.1f Hz; control %.1f Hz; robot commands sent: %d.\n', ...
    summary.ActualSensorRateHz, summary.ActualControlRateHz, ...
    summary.RobotCommandSentCount);
if options.WriteResults
    fprintf('Results: %s\n', outputDirectory);
end
clear cleanup;
cleanupStage07(dashboard, forceSource, robot);
end

function context = stage07ForceContext(robotState, handleRequested, ...
        maximumJointSpeedDegSec, maximumTcpTranslationSpeedMmSec, ...
        maximumTcpRotationSpeedDegSec)
context = scopeguide.types.forceProcessingContext();
context.HandleEnabled = logical(handleRequested);
context.RobotStationary = ...
    scopeguide.force.isRobotStationaryForBaseline(robotState, ...
    MaximumJointSpeedDegSec=maximumJointSpeedDegSec, ...
    MaximumTcpTranslationSpeedMmSec=maximumTcpTranslationSpeedMmSec, ...
    MaximumTcpRotationSpeedDegSec=maximumTcpRotationSpeedDegSec);
context.NoContactConfirmed = ~logical(handleRequested);
context.AllowBaselineUpdate = ~logical(handleRequested);
end

function printFaultAcceptanceInstructions()
schedule = scopeguide.runtime.stage07FaultSchedule();
fprintf(['Acceptance 故障引导已开启。每项故障前 5 s 会显示倒计时；' ...
    '此时按一次 Space 置 ON，进入绿色后持续施加约 3--5 N 外力。\n']);
for index = 1:numel(schedule.TimeSec)
    fprintf('  %d/5  T=%2.0f s  %s (%s)\n', index, ...
        schedule.TimeSec(index), char(schedule.NameZh(index)), ...
        char(schedule.ShortLabel(index)));
end
fprintf(['每项故障后按：撤力 -> Esc -> 等待至少 0.5 s -> R -> 确认蓝色；' ...
    '等下一次 T-5 s 提示后再按 Space ON 并施力。\n']);
end

function injection = injectionAtTime(timeSec, profile)
injection = scopeguide.runtime.stage07FaultInjectionAtTime( ...
    timeSec, profile);
end

function active = anyInjectionActive(injection)
active = injection.ForceStale || injection.RobotStale || ...
    injection.HandleStale || injection.QpFailure || ...
    injection.ControlOverrun;
end

function directory = createOutputDirectory(projectRoot)
timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
directory = fullfile(projectRoot, 'results', ...
    "stage07_live_readonly_" + timestamp);
[created, message] = mkdir(directory);
if ~created && ~isfolder(directory)
    error('scopeguide:stage07:CannotCreateResultDirectory', ...
        'Cannot create %s: %s', directory, message);
end
end

function [fileId, cleanup] = openCsv(path, header, label)
[fileId, message] = fopen(path, 'w');
if fileId < 0
    error('scopeguide:stage07:CannotOpenLog', ...
        'Cannot open %s log %s: %s', label, path, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', header);
end

function value = forceHeader()
value = strjoin(["timeSec", "sensorSeq", "sensorReadSec", ...
    "completeReadSec", "robotSeq", "robotAgeSec", ...
    "qualityValid", "robotValid", "handleRequested", ...
    "injection", "rawFx", "rawFy", "rawFz", ...
    "rawTx", "rawTy", "rawTz", "fastFx", "fastFy", "fastFz", ...
    "slowFx", "slowFy", "slowFz", "controlFx", "controlFy", ...
    "controlFz", "safetyStop", "baselineReady"], ',');
end

function writeForceRow(fileId, timeSec, frame, requested, injection)
p = frame.Processed;
baselineReady = isfield(p.BaselineDiagnostics, 'Ready') && ...
    p.BaselineDiagnostics.Ready;
fprintf(fileId, '%.9g,%.0f,%.9g,%.9g,%.0f,%.9g,%.0f,%.0f,%.0f,%s', ...
    timeSec, double(frame.RawSensorSample.sequence), ...
    frame.RawSensorSample.readDuration, frame.CompleteReadDurationSec, ...
    double(frame.RobotState.FeedbackSequence), ...
    frame.RobotState.SampleAgeSec, double(p.Quality.Valid), ...
    double(frame.RobotState.IsValid), double(requested), ...
    sanitize(injection.StatusCode));
numeric = [p.RawWrenchSensor(:); p.FastWrenchToolAtSensorOrigin(1:3); ...
    p.SlowWrenchToolAtSensorOrigin(1:3); p.ControlForceTool(:); ...
    double(p.Safety.StopRequested); double(baselineReady)];
fprintf(fileId, repmat(',%.9g', 1, numel(numeric)), numeric);
fprintf(fileId, '\n');
end

function value = controlHeader()
value = strjoin(["timeSec", "dtSec", "sensorSeq", "robotSeq", ...
    "handleEnabled", "handleValid", "handleAgeSec", "handleSource", ...
    "injection", "fsmState", "motionPermitted", "commandScale", ...
    "forceFresh", "robotFresh", "qpHealthy", "periodHealthy", ...
    "neutral", "qpValid", "qpStatus", "qpFailure", "qpExitFlag", ...
    "qpSolveSec", "currentRcmErrorM", "predictedRcmErrorM", ...
    "forceX", "forceY", "forceZ", "u1", "u2", "u3", ...
    "qdot1", "qdot2", "qdot3", "qdot4", "qdot5", "qdot6", ...
    "qtarget1", "qtarget2", "qtarget3", "qtarget4", "qtarget5", ...
    "qtarget6", "predictionValid", "hardwareCommandAuthorized", ...
    "motionCommandSent"], ',');
end

function writeControlRow(fileId, timeSec, frame, handle, decision, injection)
safety = decision.Safety;
enable = decision.EnableStatus;
qp = decision.Qp;
currentError = NaN;
if decision.GeometryValid
    currentError = decision.Geometry.RcmErrorNormM;
end
fprintf(fileId, ['%.9g,%.9g,%.0f,%.0f,%.0f,%.0f,%.9g,%s,%s,%s,' ...
    '%.0f,%.9g,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%s,%s,%.9g,' ...
    '%.9g,%.9g,%.9g'], ...
    timeSec, decision.DtSec, double(frame.RawSensorSample.sequence), ...
    double(frame.RobotState.FeedbackSequence), double(handle.Enabled), ...
    double(handle.IsValid), handle.SampleAgeSec, sanitize(handle.Source), ...
    sanitize(injection.StatusCode), sanitize(enable.State), ...
    double(enable.MotionPermitted), enable.CommandScale, ...
    double(safety.ForceFresh), double(safety.RobotFresh), ...
    double(safety.QpHealthy), double(safety.ControlPeriodHealthy), ...
    double(safety.NeutralWrench), double(qp.MotionCommandValid), ...
    sanitize(qp.StatusCode), sanitize(qp.FailureCode), qp.ExitFlag, ...
    qp.SolveTimeSec, currentError, qp.PredictedRcmErrorNormM);
numeric = [frame.Processed.ControlForceTool(:); ...
    decision.Admittance.GeneralizedVelocity(:); ...
    decision.QdotPredictedRadSec(:); ...
    decision.QTargetPredictedRad(:); double(decision.PredictionValid); ...
    double(decision.HardwareCommandAuthorized); ...
    double(decision.MotionCommandSent)];
fprintf(fileId, repmat(',%.9g', 1, numel(numeric)), numeric);
fprintf(fileId, '\n');
end

function stats = statistics(values)
values = values(isfinite(values));
stats = struct('Count', numel(values), 'Mean', NaN, 'P50', NaN, ...
    'P95', NaN, 'P99', NaN, 'Maximum', NaN);
if isempty(values)
    return;
end
stats.Mean = mean(values);
stats.P50 = percentile(values, 50);
stats.P95 = percentile(values, 95);
stats.P99 = percentile(values, 99);
stats.Maximum = max(values);
end

function value = percentile(values, percentage)
values = sort(values(:));
position = 1 + (numel(values) - 1) * percentage / 100;
lower = floor(position);
upper = ceil(position);
fraction = position - lower;
value = values(lower) * (1 - fraction) + values(upper) * fraction;
end

function value = sanitize(input)
value = char(replace(string(input), [",", newline], [";", " "]));
end

function ensureCapacity(index, capacity, label)
if index > capacity
    error('scopeguide:stage07:LogCapacityExceeded', ...
        '%s log capacity exceeded.', label);
end
end

function record = rcmRecord(cfg)
record = struct('PointBaseM', cfg.rcm.PointBaseM, ...
    'PointBaseMm', 1e3 * cfg.rcm.PointBaseM, ...
    'Frame', cfg.rcm.PointFrame, 'Source', cfg.rcm.PointSource, ...
    'SourceDescription', cfg.rcm.PointSourceDescription, ...
    'FixtureIdentifier', cfg.rcm.FixtureIdentifier, ...
    'CalibrationValid', cfg.rcm.CalibrationValid, ...
    'BoundsValidated', cfg.rcm.BoundsValidated, ...
    'PhysicalMotionAuthorized', false);
end

function writeJson(path, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end
[fileId, message] = fopen(path, 'w');
if fileId < 0
    error('scopeguide:stage07:CannotWriteJson', ...
        'Cannot open %s: %s', path, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end

function cleanupStage07(dashboard, forceSource, robot)
try
    dashboard.close();
catch
end
try
    forceSource.disconnect();
catch
end
try
    robot.disconnect();
catch
end
end
