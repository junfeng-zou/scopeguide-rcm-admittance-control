function [summary, outputDirectory] = runStage07ReadOnlyWorker( ...
        cfg, sensorCfg, options, commandQueue, displayQueue)
%RUNSTAGE07READONLYWORKER Hardware/control half of parallel Stage 7.
% This function owns all network and controller objects.  It creates no
% figures and contains no robot motion command call sites.

arguments
    cfg (1, 1) struct
    sensorCfg (1, 1) struct
    options (1, 1) struct
    commandQueue (1, 1) parallel.pool.PollableDataQueue
    displayQueue (1, 1) parallel.pool.PollableDataQueue
end

validateWorkerOptions(options);
projectRoot = string(cfg.meta.ProjectRoot);
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'force_sensor'));
addpath(fullfile(projectRoot, 'force_calibration'));
addpath(fullfile(projectRoot, 'robot'));

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

sendMessage(displayQueue, lifecycleMessage( ...
    "STARTING", "后台控制 worker 正在预热 quadprog。"));
try
    [summary, outputDirectory] = runWorkerBody( ...
        cfg, sensorCfg, options, commandQueue, displayQueue, projectRoot);
catch exception
    message = lifecycleMessage("ERROR", string(exception.message));
    message.Identifier = string(exception.identifier);
    sendMessage(displayQueue, message);
    rethrow(exception);
end
end

function [summary, outputDirectory] = runWorkerBody( ...
        cfg, sensorCfg, options, commandQueue, displayQueue, projectRoot)
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
hardwareCleanup = onCleanup(@() cleanupHardware( ...
    forceSource, robot));

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

sendMessage(displayQueue, lifecycleMessage( ...
    "CONNECTING_ROBOT", "正在连接机器人只读反馈。"));
robot.connectReadOnly();
sendMessage(displayQueue, lifecycleMessage( ...
    "CONNECTING_FORCE", "正在连接 HEX-H。"));
forceSource.connect();
sendMessage(displayQueue, lifecycleMessage( ...
    "CONNECTED", "连接完成；启动软件置零中，请保持静止。"));

sensorPeriodSec = 1 / cfg.sensor.SampleRateHz;
controlPeriodSec = 1 / cfg.runtime.ControlRateHz;
displayPeriodSec = 1 / options.DisplayRateHz;
clock = tic;
nextSensorSec = 0;
nextControlSec = 0;
nextDisplaySec = 0;
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
displaySnapshotCount = 0;
uiCommandCount = 0;
uiHeartbeatTimeoutCount = 0;
maximumUiHeartbeatAgeSec = 0;
fsmStates = strings(0, 1);
qpFailureCodes = strings(0, 1);
sensorCapacity = ceil(options.DurationSec * ...
    cfg.sensor.SampleRateHz * 1.1) + 100;
controlCapacity = ceil(options.DurationSec * ...
    cfg.runtime.ControlRateHz * 1.1) + 100;
displayCapacity = ceil(options.DurationSec * ...
    options.DisplayRateHz * 1.1) + 100;
sensorLoopTime = nan(sensorCapacity, 1);
completeReadDuration = nan(sensorCapacity, 1);
robotAge = nan(sensorCapacity, 1);
controlDt = nan(controlCapacity, 1);
qpSolveTime = nan(controlCapacity, 1);
currentRcmError = nan(controlCapacity, 1);
predictedRcmError = nan(controlCapacity, 1);
displaySendDuration = nan(displayCapacity, 1);

ui = initialUiState();
uiWasFresh = false;
while toc(clock) < options.DurationSec
    nowSec = toc(clock);
    [ui, commandCount] = drainCommands(commandQueue, ui, nowSec);
    uiCommandCount = uiCommandCount + commandCount;
    if ui.StopRequested
        terminationReason = "CLIENT_STOP_REQUESTED";
        break;
    end
    uiAgeSec = Inf;
    if isfinite(ui.LastMessageWorkerSec)
        uiAgeSec = max(0, nowSec - ui.LastMessageWorkerSec);
        maximumUiHeartbeatAgeSec = max( ...
            maximumUiHeartbeatAgeSec, uiAgeSec);
    end
    uiFresh = isfinite(uiAgeSec) && ...
        uiAgeSec <= options.InputHeartbeatTimeoutSec;
    if uiWasFresh && ~uiFresh
        uiHeartbeatTimeoutCount = uiHeartbeatTimeoutCount + 1;
    end
    uiWasFresh = uiFresh;

    if nowSec < nextSensorSec
        pause(min(nextSensorSec - nowSec, 0.001));
        continue;
    end
    injection = injectionAtTime( ...
        nowSec, options.FaultInjectionProfile);
    requested = uiFresh && ui.Requested;
    if ~injection.HandleStale
        if uiFresh
            handleAdapter.submit(requested, nowSec, ...
                Source=ui.Source, SourceValid=true, ...
                SupportsPhysicalMotion=false);
        else
            handleAdapter.submit(false, nowSec, ...
                Source="ui_heartbeat_timeout", SourceValid=false, ...
                SupportsPhysicalMotion=false);
        end
    end
    if ui.ResetPending
        controller.requestReset();
        ui.ResetPending = false;
    end
    contextProvider = @(robotState) stage07ForceContext( ...
        robotState, requested, ...
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
            requested, injection);
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
        fsmStates(end + 1, 1) = ...
            decision.EnableStatus.State; %#ok<AGROW>
        if strlength(decision.Qp.FailureCode) > 0
            qpFailureCodes(end + 1, 1) = ...
                decision.Qp.FailureCode; %#ok<AGROW>
        end
        if controlFileId >= 0
            writeControlRow(controlFileId, afterReadSec, frame, ...
                handleInput, decision, injection, uiFresh, uiAgeSec);
        end
        nextControlSec = nextControlSec + controlPeriodSec;
        if afterReadSec > nextControlSec + controlPeriodSec
            controlDeadlineMissCount = controlDeadlineMissCount + 1;
            nextControlSec = afterReadSec + controlPeriodSec;
        end
    end
    if afterReadSec >= nextDisplaySec && ~isempty(lastDecision)
        displaySnapshotCount = displaySnapshotCount + 1;
        ensureCapacity(displaySnapshotCount, ...
            numel(displaySendDuration), 'display');
        snapshot = scopeguide.runtime.buildStage07DisplaySnapshot( ...
            afterReadSec, frame.Processed, lastDecision);
        snapshot.UiHeartbeatFresh = uiFresh;
        snapshot.UiHeartbeatAgeSec = uiAgeSec;
        snapshot.WorkerSensorCount = sensorCount;
        snapshot.WorkerControlCount = controlCount;
        sendClock = tic;
        sendMessage(displayQueue, snapshot);
        displaySendDuration(displaySnapshotCount) = toc(sendClock);
        nextDisplaySec = afterReadSec + displayPeriodSec;
    end

    nextSensorSec = nextSensorSec + sensorPeriodSec;
    if afterReadSec > nextSensorSec + sensorPeriodSec
        sensorDeadlineMissCount = sensorDeadlineMissCount + 1;
        nextSensorSec = afterReadSec + sensorPeriodSec;
    end
end

elapsedSec = toc(clock);
finalTime = elapsedSec + cfg.runtime.NominalDtSec;
handleAdapter.forceDisable(finalTime, "STAGE07_PARALLEL_EXIT");
controller.reset();
indicesSensor = 1:sensorCount;
indicesControl = 1:controlCount;

summary = struct();
summary.Stage = 7;
summary.Status = "live_readonly_parallel_capture_complete_not_final_accepted";
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
summary.WorkerDisplaySnapshotCount = displaySnapshotCount;
summary.WorkerDisplaySendDuration = statistics( ...
    displaySendDuration(1:displaySnapshotCount));
summary.UiCommandCount = uiCommandCount;
summary.UiHeartbeatTimeoutSec = options.InputHeartbeatTimeoutSec;
summary.UiHeartbeatTimeoutCount = uiHeartbeatTimeoutCount;
summary.MaximumUiHeartbeatAgeSec = maximumUiHeartbeatAgeSec;
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
summary.ControlWorkerCreatedFigures = false;
summary.RequiresManualReview = true;

clear forceCleanup controlCleanup;
if options.WriteResults
    writeJson(fullfile(outputDirectory, 'summary.json'), summary);
    writeJson(fullfile(outputDirectory, 'config_snapshot.json'), cfg);
    writeJson(fullfile(outputDirectory, ...
        'sensor_config_snapshot.json'), sensorCfg);
    writeJson(fullfile(outputDirectory, 'rcm_point_record.json'), ...
        rcmRecord(cfg));
end
sendMessage(displayQueue, lifecycleMessage( ...
    "FINISHED", "后台控制 worker 已安全停止。"));
clear hardwareCleanup;
cleanupHardware(forceSource, robot);
end

function [state, count] = drainCommands(queue, state, nowSec)
count = 0;
while true
    [message, ok] = poll(queue, 0);
    if ~ok
        break;
    end
    count = count + 1;
    if ~isstruct(message) || ~isfield(message, 'Type')
        continue;
    end
    switch string(message.Type)
        case "input"
            if isfield(message, 'Requested') && ...
                    islogical(message.Requested) && ...
                    isscalar(message.Requested)
                state.Requested = message.Requested;
            else
                state.Requested = false;
            end
            if isfield(message, 'Source')
                state.Source = string(message.Source);
            else
                state.Source = "software_released";
            end
            if isfield(message, 'ResetRequested') && ...
                    logical(message.ResetRequested)
                state.ResetPending = true;
            end
            if isfield(message, 'CloseRequested') && ...
                    logical(message.CloseRequested)
                state.StopRequested = true;
                state.Requested = false;
            end
            state.LastMessageWorkerSec = nowSec;
        case "stop"
            state.StopRequested = true;
            state.Requested = false;
            state.LastMessageWorkerSec = nowSec;
    end
end
end

function state = initialUiState()
state = struct();
state.Requested = false;
state.Source = "software_released";
state.ResetPending = false;
state.StopRequested = false;
state.LastMessageWorkerSec = NaN;
end

function message = lifecycleMessage(phase, text)
message = struct('Type', "lifecycle", 'Phase', string(phase), ...
    'Text', string(text), 'Identifier', "");
end

function sendMessage(queue, message)
try
    send(queue, message);
catch
    % A closed client queue must not interrupt fail-safe controller cleanup.
end
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
    "stage07_live_readonly_parallel_" + timestamp);
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
numeric = [p.RawWrenchSensor(:); ...
    p.FastWrenchToolAtSensorOrigin(1:3); ...
    p.SlowWrenchToolAtSensorOrigin(1:3); p.ControlForceTool(:); ...
    double(p.Safety.StopRequested); double(baselineReady)];
fprintf(fileId, repmat(',%.9g', 1, numel(numeric)), numeric);
fprintf(fileId, '\n');
end

function value = controlHeader()
value = strjoin(["timeSec", "dtSec", "sensorSeq", "robotSeq", ...
    "handleEnabled", "handleValid", "handleAgeSec", "handleSource", ...
    "uiHeartbeatFresh", "uiHeartbeatAgeSec", ...
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

function writeControlRow(fileId, timeSec, frame, handle, decision, ...
        injection, uiFresh, uiAgeSec)
safety = decision.Safety;
enable = decision.EnableStatus;
qp = decision.Qp;
currentError = NaN;
if decision.GeometryValid
    currentError = decision.Geometry.RcmErrorNormM;
end
fprintf(fileId, ['%.9g,%.9g,%.0f,%.0f,%.0f,%.0f,%.9g,%s,' ...
    '%.0f,%.9g,%s,%s,%.0f,%.9g,%.0f,%.0f,%.0f,%.0f,%.0f,' ...
    '%.0f,%s,%s,%.9g,%.9g,%.9g,%.9g'], ...
    timeSec, decision.DtSec, double(frame.RawSensorSample.sequence), ...
    double(frame.RobotState.FeedbackSequence), double(handle.Enabled), ...
    double(handle.IsValid), handle.SampleAgeSec, sanitize(handle.Source), ...
    double(uiFresh), uiAgeSec, sanitize(injection.StatusCode), ...
    sanitize(enable.State), double(enable.MotionPermitted), ...
    enable.CommandScale, double(safety.ForceFresh), ...
    double(safety.RobotFresh), double(safety.QpHealthy), ...
    double(safety.ControlPeriodHealthy), double(safety.NeutralWrench), ...
    double(qp.MotionCommandValid), sanitize(qp.StatusCode), ...
    sanitize(qp.FailureCode), qp.ExitFlag, qp.SolveTimeSec, ...
    currentError, qp.PredictedRcmErrorNormM);
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

function cleanupHardware(forceSource, robot)
try
    forceSource.disconnect();
catch
end
try
    robot.disconnect();
catch
end
end

function validateWorkerOptions(options)
required = {'DurationSec', 'DisplayRateHz', 'StatusRateHz', ...
    'WriteResults', 'EnableAdaptiveFzBaseline', ...
    'FaultInjectionProfile', 'MaximumStationaryJointSpeedDegSec', ...
    'MaximumStationaryTcpTranslationSpeedMmSec', ...
    'MaximumStationaryTcpRotationSpeedDegSec', ...
    'InputHeartbeatTimeoutSec'};
for index = 1:numel(required)
    if ~isfield(options, required{index})
        error('scopeguide:stage07:IncompleteWorkerOptions', ...
            'Parallel worker option %s is missing.', required{index});
    end
end
positive = [options.DurationSec, options.DisplayRateHz, ...
    options.StatusRateHz, options.InputHeartbeatTimeoutSec];
if any(~isfinite(positive)) || any(positive <= 0) || ...
        ~ismember(string(options.FaultInjectionProfile), ...
        ["none", "acceptance"])
    error('scopeguide:stage07:InvalidWorkerOptions', ...
        'Parallel worker timing/profile options are invalid.');
end
end
