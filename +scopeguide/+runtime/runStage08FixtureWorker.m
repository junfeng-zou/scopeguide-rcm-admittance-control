function [summary, outputDirectory] = runStage08FixtureWorker( ...
        cfg, sensorCfg, options, confirmations, commandQueue, displayQueue)
%RUNSTAGE08FIXTUREWORKER Hardware/control process for Stage 8.
% Graphics never execute here.  Robot and HEX-H clients live exclusively
% in this worker, and RobotAdapter is the only ServoJ command boundary.

arguments
    cfg (1, 1) struct
    sensorCfg (1, 1) struct
    options (1, 1) struct
    confirmations (1, 1) struct
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
validateRcmAdmittanceConfig(cfg);
authorization = ...
    scopeguide.safety.evaluateStage08CommissioningAuthorization( ...
    cfg, confirmations);
if ~authorization.CommissioningAllowed
    error('scopeguide:stage08:WorkerAuthorizationBlocked', ...
        'Worker authorization blocked by: %s.', ...
        strjoin(authorization.FailedGates, ', '));
end
sendMessage(displayQueue, lifecycle( ...
    "STARTING", "正在预热 quadprog；机器人命令尚未发送。"));
try
    [summary, outputDirectory] = workerBody(cfg, sensorCfg, options, ...
        authorization, commandQueue, displayQueue, projectRoot);
catch exception
    message = lifecycle("ERROR", string(exception.message));
    message.Identifier = string(exception.identifier);
    sendMessage(displayQueue, message);
    rethrow(exception);
end
end

function [summary, outputDirectory] = workerBody(cfg, sensorCfg, options, ...
        authorization, commandQueue, displayQueue, projectRoot)
warmup = scopeguide.control.warmupRcmQpSolver(cfg);
if ~warmup.ReadyForTimingAudit
    error('scopeguide:stage08:QpWarmupFailed', ...
        'quadprog warm-up did not meet the configured timing budget.');
end
if options.WriteResults
    outputDirectory = createOutputDirectory(projectRoot, ...
        cfg.stage08.DofMode);
    [logId, logCleanup] = openCommandLog( ...
        fullfile(outputDirectory, 'drag_control.csv')); %#ok<ASGLU>
else
    outputDirectory = "";
    logId = -1;
    logCleanup = onCleanup(@() []);
end

robot = scopeguide.io.RobotAdapter(cfg);
forceSource = scopeguide.io.ForceSensorAdapter( ...
    cfg, sensorCfg, [], StatusRateHz=options.StatusRateHz);
handleAdapter = scopeguide.io.EnableHandleAdapter(cfg);
controller = scopeguide.control.Stage08FixtureController( ...
    cfg, authorization);
commandChannelHealthy = true;
wasCommanding = false;
hardwareCleanup = onCleanup(@() cleanupHardware(forceSource, robot));

sendMessage(displayQueue, lifecycle( ...
    "CONNECTING_ROBOT", sprintf([ ...
    '正在连接机器人并执行 EnableRobot(%.3f)、SpeedFactor(%d)；' ...
    '请保持急停可触及。'], cfg.stage08.EnablePayloadKg, ...
    cfg.stage08.SpeedFactorPercent)));
robot.connectForMotion(authorization);
sendMessage(displayQueue, lifecycle( ...
    "ROBOT_ENABLED", sprintf( ...
    '30004 已确认机器人使能且 SpeedRatio=%.0f。', ...
    cfg.stage08.SpeedFactorPercent)));
sendMessage(displayQueue, lifecycle( ...
    "CONNECTING_FORCE", "正在连接 HEX-H。"));
forceSource.connect();
sendMessage(displayQueue, lifecycle( ...
    "CONNECTED", "连接完成；启动软件置零中，请保持静止且不要按 Space。"));

sensorPeriodSec = 1 / cfg.sensor.SampleRateHz;
controlPeriodSec = 1 / cfg.runtime.ControlRateHz;
displayPeriodSec = 1 / options.DisplayRateHz;
clock = tic;
nextSensorSec = 0;
nextControlSec = 0;
nextDisplaySec = 0;
ui = initialUiState();
lastDecision = struct([]);
lastCommandTarget = nan(6, 1);
trackingErrorCount = 0;
terminationReason = "DURATION_COMPLETE";
faultIdentifier = "";
faultMessage = "";
commandFaultTimeSec = NaN;
commandFaultCount = 0;
commandAttemptCountAtFirstFault = NaN;
commandFaultRecord = struct();
monitorOnlyAfterCommandFault = false;

sensorCount = 0;
controlCount = 0;
commandTargetCount = 0;
holdCommandCount = 0;
controlDeadlineMissCount = 0;
uiHeartbeatTimeoutCount = 0;
uiWasFresh = false;
lastEffectiveRequested = false;
pendingStopSinceSec = NaN;
displayCount = 0;
maximumUiAgeSec = 0;
capacity = ceil(options.DurationSec * cfg.runtime.ControlRateHz * 1.2) + 100;
stopLatencySec = nan(capacity, 1);
stopLatencyCount = 0;
controlDt = nan(capacity, 1);
qpSolveSec = nan(capacity, 1);
servoSendSec = nan(capacity, 1);
transportPollSec = nan(ceil(options.DurationSec * ...
    cfg.sensor.SampleRateHz * 1.2) + 100, 1);
transportPollCount = 0;
trackingErrorRad = nan(capacity, 1);
rcmErrorM = nan(capacity, 1);
qdotCommandAbsMaxRadSec = nan(capacity, 1);
qdotActualAbsMaxRadSec = nan(capacity, 1);
qdotQpCandidateAbsMaxRadSec = nan(capacity, 1);
fsmStates = strings(capacity, 1);
forceStatusCodes = strings(capacity, 1);
wrenchSafetyStopCodes = strings(capacity, 1);

while toc(clock) < options.DurationSec
    nowSec = toc(clock);
    [ui, commandCount] = drainCommands(commandQueue, ui, nowSec); %#ok<ASGLU>
    if ui.StopRequested
        terminationReason = "CLIENT_STOP_REQUESTED";
        break;
    end
    if isfinite(ui.LastMessageWorkerSec)
        uiAgeSec = max(0, nowSec - ui.LastMessageWorkerSec);
        maximumUiAgeSec = max(maximumUiAgeSec, uiAgeSec);
    else
        uiAgeSec = Inf;
    end
    uiFresh = uiAgeSec <= options.InputHeartbeatTimeoutSec;
    if uiWasFresh && ~uiFresh
        uiHeartbeatTimeoutCount = uiHeartbeatTimeoutCount + 1;
    end
    uiWasFresh = uiFresh;
    if nowSec < nextSensorSec
        pause(min(nextSensorSec - nowSec, 0.001));
        continue;
    end

    % 30003 replies are drained without waiting.  A delayed successful
    % reply therefore cannot stall force acquisition or the control loop.
    if commandChannelHealthy
        pollClock = tic;
        try
            robot.pollMotionCommandStatus();
        catch exception
            commandChannelHealthy = false;
            wasCommanding = false;
            commandFaultCount = commandFaultCount + 1;
            latestFault = captureCommandFault( ...
                robot, exception, nowSec, "ASYNC_RESPONSE_POLL");
            if ~monitorOnlyAfterCommandFault
                commandFaultRecord = latestFault;
                commandFaultTimeSec = nowSec;
                commandAttemptCountAtFirstFault = ...
                    double(robot.CommandAttemptCount);
                faultIdentifier = string(exception.identifier);
                faultMessage = string(exception.message);
                monitorOnlyAfterCommandFault = true;
                sendMessage(displayQueue, lifecycle( ...
                    "COMMAND_FAULT_MONITOR_ONLY", ...
                    commandFaultText(latestFault)));
            end
            handleAdapter.forceDisable(toc(clock), ...
                "COMMAND_CHANNEL_FAILURE");
        end
        transportPollCount = transportPollCount + 1;
        transportPollSec(transportPollCount) = toc(pollClock);
    end

    requested = uiFresh && ui.Requested;
    if lastEffectiveRequested && ~requested
        pendingStopSinceSec = nowSec;
    end
    lastEffectiveRequested = requested;
    if uiFresh
        handleAdapter.submit(requested, nowSec, Source=ui.Source, ...
            SourceValid=true, SupportsPhysicalMotion=true);
    else
        handleAdapter.submit(false, nowSec, ...
            Source="ui_heartbeat_timeout", SourceValid=false, ...
            SupportsPhysicalMotion=false);
    end
    if ui.ResetPending
        controller.requestReset();
        ui.ResetPending = false;
    end
    contextProvider = @(robotState) forceContext( ...
        robotState, requested);
    try
        frame = forceSource.readProcessed(robot, contextProvider);
    catch exception
        terminationReason = "SENSOR_OR_FEEDBACK_READ_EXCEPTION";
        if strlength(faultIdentifier) == 0
            faultIdentifier = string(exception.identifier);
            faultMessage = string(exception.message);
        end
        handleAdapter.forceDisable(toc(clock), terminationReason);
        break;
    end
    afterReadSec = toc(clock);
    sensorCount = sensorCount + 1;
    if isfinite(pendingStopSinceSec) && ...
            max(abs(frame.RobotState.JointVelocityRadSec(:))) <= ...
            cfg.stage08.StopVelocityThresholdRadSec
        stopLatencyCount = stopLatencyCount + 1;
        stopLatencySec(stopLatencyCount) = ...
            max(0, afterReadSec - pendingStopSinceSec);
        pendingStopSinceSec = NaN;
    end

    if afterReadSec >= nextControlSec
        controlCount = controlCount + 1;
        ensureCapacity(controlCount, capacity);
        currentTrackingError = 0;
        if all(isfinite(lastCommandTarget))
            currentTrackingError = max(abs( ...
                lastCommandTarget - frame.RobotState.JointPositionRad(:)));
        end
        trackingErrorRad(controlCount) = currentTrackingError;
        if currentTrackingError > ...
                cfg.stage08.MaximumTargetFeedbackErrorRad
            trackingErrorCount = trackingErrorCount + 1;
        else
            trackingErrorCount = 0;
        end
        trackingHealthy = trackingErrorCount < ...
            cfg.stage08.MaximumConsecutiveTrackingErrors;
        externalHealth = struct( ...
            'CommandChannelHealthy', commandChannelHealthy, ...
            'TrackingHealthy', trackingHealthy, ...
            'StatusCode', healthCode(commandChannelHealthy, ...
                trackingHealthy));
        handleInput = handleAdapter.read(afterReadSec);
        decision = controller.step(frame.Processed, frame.RobotState, ...
            handleInput, afterReadSec, externalHealth);
        decision.MotionCommandSent = false;
        sendDuration = NaN;
        if decision.CommandEligible
            try
                commandResult = robot.sendServoTarget( ...
                    decision.QTargetCommandRad, authorization);
                sendDuration = commandResult.SendDurationSec;
                lastCommandTarget = decision.QTargetCommandRad;
                decision.MotionCommandSent = true;
                commandTargetCount = commandTargetCount + 1;
                wasCommanding = true;
            catch exception
                commandChannelHealthy = false;
                wasCommanding = false;
                commandFaultCount = commandFaultCount + 1;
                latestFault = captureCommandFault( ...
                    robot, exception, afterReadSec, "SERVO_TARGET");
                if ~monitorOnlyAfterCommandFault
                    commandFaultRecord = latestFault;
                    commandFaultTimeSec = afterReadSec;
                    commandAttemptCountAtFirstFault = ...
                        double(robot.CommandAttemptCount);
                    faultIdentifier = string(exception.identifier);
                    faultMessage = string(exception.message);
                    monitorOnlyAfterCommandFault = true;
                    sendMessage(displayQueue, lifecycle( ...
                        "COMMAND_FAULT_MONITOR_ONLY", ...
                        commandFaultText(latestFault)));
                end
                handleAdapter.forceDisable(toc(clock), ...
                    "COMMAND_CHANNEL_FAILURE");
            end
        elseif wasCommanding
            try
                commandResult = robot.sendCurrentPositionHold(authorization);
                sendDuration = commandResult.SendDurationSec;
                lastCommandTarget = frame.RobotState.JointPositionRad(:);
                holdCommandCount = holdCommandCount + 1;
            catch exception
                commandChannelHealthy = false;
                commandFaultCount = commandFaultCount + 1;
                latestFault = captureCommandFault( ...
                    robot, exception, afterReadSec, "RELEASE_HOLD");
                if ~monitorOnlyAfterCommandFault
                    commandFaultRecord = latestFault;
                    commandFaultTimeSec = afterReadSec;
                    commandAttemptCountAtFirstFault = ...
                        double(robot.CommandAttemptCount);
                    faultIdentifier = string(exception.identifier);
                    faultMessage = string(exception.message);
                    monitorOnlyAfterCommandFault = true;
                    sendMessage(displayQueue, lifecycle( ...
                        "COMMAND_FAULT_MONITOR_ONLY", ...
                        commandFaultText(latestFault)));
                end
                handleAdapter.forceDisable(toc(clock), ...
                    "COMMAND_CHANNEL_FAILURE");
            end
            wasCommanding = false;
        end
        lastDecision = decision;
        controlDt(controlCount) = decision.DtSec;
        qpSolveSec(controlCount) = decision.Qp.SolveTimeSec;
        servoSendSec(controlCount) = sendDuration;
        if decision.GeometryValid
            rcmErrorM(controlCount) = decision.Geometry.RcmErrorNormM;
        end
        qdotCommandAbsMaxRadSec(controlCount) = ...
            max(abs(decision.QdotCommandRadSec(:)));
        qdotActualAbsMaxRadSec(controlCount) = ...
            max(abs(frame.RobotState.JointVelocityRadSec(:)));
        qdotQpCandidateAbsMaxRadSec(controlCount) = ...
            max(abs(decision.QdotQpCandidateRadSec(:)));
        fsmStates(controlCount) = decision.EnableStatus.State;
        forceStatusCodes(controlCount) = ...
            string(decision.Safety.ForceStatusCode);
        wrenchSafetyStopCodes(controlCount) = ...
            string(decision.Safety.WrenchSafetyStopCode);
        if logId >= 0
            writeLogRow(logId, afterReadSec, frame, decision, ...
                sendDuration, currentTrackingError, uiAgeSec);
        end
        nextControlSec = nextControlSec + controlPeriodSec;
        if afterReadSec > nextControlSec + controlPeriodSec
            controlDeadlineMissCount = controlDeadlineMissCount + 1;
            nextControlSec = afterReadSec + controlPeriodSec;
        end
    end
    if afterReadSec >= nextDisplaySec && ~isempty(lastDecision)
        snapshot = scopeguide.runtime.buildStage07DisplaySnapshot( ...
            afterReadSec, frame.Processed, lastDecision);
        snapshot.UiHeartbeatFresh = uiFresh;
        snapshot.UiHeartbeatAgeSec = uiAgeSec;
        snapshot.DofMode = string(cfg.stage08.DofMode);
        snapshot.JointVelocityActualRadSec = ...
            double(frame.RobotState.JointVelocityRadSec(:));
        snapshot.WrenchSafetyStopReasons = ...
            safetyStopReasons(frame.Processed);
        sendMessage(displayQueue, snapshot);
        displayCount = displayCount + 1;
        nextDisplaySec = afterReadSec + displayPeriodSec;
    end
    nextSensorSec = nextSensorSec + sensorPeriodSec;
    if afterReadSec > nextSensorSec + sensorPeriodSec
        nextSensorSec = afterReadSec + sensorPeriodSec;
    end
end

elapsedSec = toc(clock);
if wasCommanding && commandChannelHealthy
    try
        robot.sendCurrentPositionHold(authorization);
        holdCommandCount = holdCommandCount + 1;
    catch exception
        commandChannelHealthy = false;
        commandFaultCount = commandFaultCount + 1;
        latestFault = captureCommandFault( ...
            robot, exception, elapsedSec, "FINAL_HOLD");
        if ~monitorOnlyAfterCommandFault
            commandFaultRecord = latestFault;
            commandFaultTimeSec = elapsedSec;
            commandAttemptCountAtFirstFault = ...
                double(robot.CommandAttemptCount);
            faultIdentifier = string(exception.identifier);
            faultMessage = string(exception.message);
            monitorOnlyAfterCommandFault = true;
        end
    end
end
handleAdapter.forceDisable(elapsedSec + cfg.runtime.NominalDtSec, ...
    "STAGE08_EXIT");
controller.reset();
disableFaultIdentifier = "";
disableFaultMessage = "";
try
    robot.disableProgrammatically();
catch exception
    disableFaultIdentifier = string(exception.identifier);
    disableFaultMessage = string(exception.message);
    sendMessage(displayQueue, lifecycle( ...
        "DISABLE_NOT_CONFIRMED", sprintf( ...
        'DisableRobot 未由反馈确认：%s。请立即使用示教器或急停下使能。', ...
        exception.message)));
end
indices = 1:controlCount;
summary = struct();
summary.Stage = 8;
if strlength(disableFaultIdentifier) > 0
    summary.Status = "disable_robot_not_confirmed";
elseif monitorOnlyAfterCommandFault
    summary.Status = "command_fault_latched_monitoring_completed";
else
    summary.Status = "fixture_run_complete_requires_manual_review";
end
summary.TerminationReason = terminationReason;
summary.FaultIdentifier = faultIdentifier;
summary.FaultMessage = faultMessage;
summary.DurationSec = elapsedSec;
summary.DofMode = string(cfg.stage08.DofMode);
summary.DofMask = scopeguide.control.stage08DofMask(cfg.stage08.DofMode);
summary.SpeedScale = cfg.stage08.SpeedScale;
summary.Authorization = authorization;
summary.SensorSampleCount = sensorCount;
summary.ControlSampleCount = controlCount;
summary.ActualSensorRateHz = sensorCount / max(elapsedSec, eps);
summary.ActualControlRateHz = controlCount / max(elapsedSec, eps);
summary.ControlDeadlineMissCount = controlDeadlineMissCount;
summary.UiHeartbeatTimeoutCount = uiHeartbeatTimeoutCount;
summary.MaximumUiHeartbeatAgeSec = maximumUiAgeSec;
summary.DisplaySnapshotCount = displayCount;
summary.ObservedFsmStates = unique(fsmStates(indices)).';
summary.ObservedForceStatusCodes = compactCodes( ...
    forceStatusCodes(indices), true);
summary.ObservedWrenchSafetyStopCodes = compactCodes( ...
    wrenchSafetyStopCodes(indices), false);
summary.ControlDtSec = statistics(controlDt(indices));
summary.QpSolveTimeSec = statistics(qpSolveSec(indices));
summary.ServoSendDurationSec = statistics(servoSendSec(indices));
transportDiagnostics = robot.getMotionTransportDiagnostics();
summary.MotionTransportDiagnostics = transportDiagnostics;
summary.MotionTransportHealthyAtExit = ...
    servoTransportHealthy(transportDiagnostics);
summary.ServoResponseDurationSec = statistics( ...
    transportResponseLatencies(transportDiagnostics));
summary.ServoResponseExpected = ...
    transportResponseExpected(transportDiagnostics);
% The validated legacy ServoJ path performs one immediate nonblocking read
% but does not require a per-command reply. Optional 30003 frames are
% diagnostic only and are excluded from the commissioning timing gate.
summary.ServoReplyRequiredByAcceptance = false;
summary.ServoResponseTimingSatisfied = ...
    ~summary.ServoResponseExpected;
summary.MotionTransportPollDurationSec = statistics( ...
    transportPollSec(1:transportPollCount));
summary.TargetFeedbackErrorRad = statistics(trackingErrorRad(indices));
summary.RcmErrorM = statistics(rcmErrorM(indices));
summary.JointVelocityCommandAbsMaxRadSec = statistics( ...
    qdotCommandAbsMaxRadSec(indices));
summary.JointVelocityActualAbsMaxRadSec = statistics( ...
    qdotActualAbsMaxRadSec(indices));
summary.JointVelocityQpCandidateAbsMaxRadSec = statistics( ...
    qdotQpCandidateAbsMaxRadSec(indices));
summary.StopLatencySec = statistics(stopLatencySec(1:stopLatencyCount));
summary.StopLatencyWithinLimit = stopLatencyCount > 0 && ...
    summary.StopLatencySec.Maximum <= cfg.stage08.MaximumStopLatencySec;
summary.RobotCommandAttemptCount = double(robot.CommandAttemptCount);
summary.RobotCommandSentCount = double(robot.CommandSentCount);
summary.RobotCommandFailureCount = double(robot.CommandFailureCount);
summary.MotionTargetCommandCount = commandTargetCount;
summary.HoldCommandCount = holdCommandCount;
summary.CommandChannelHealthyAtExit = commandChannelHealthy;
summary.CommandFaultCount = commandFaultCount;
summary.CommandFaultTimeSec = commandFaultTimeSec;
summary.CommandFaultRecord = commandFaultRecord;
summary.MonitorOnlyAfterCommandFault = monitorOnlyAfterCommandFault;
summary.MonitoringContinuedAfterCommandFaultSec = NaN;
if monitorOnlyAfterCommandFault && isfinite(commandFaultTimeSec)
    summary.MonitoringContinuedAfterCommandFaultSec = ...
        max(0, elapsedSec - commandFaultTimeSec);
end
summary.MotionOutputLatchedOffAfterCommandFault = ...
    monitorOnlyAfterCommandFault && ~commandChannelHealthy;
summary.NoAdditionalMotionAttemptsAfterCommandFault = ...
    monitorOnlyAfterCommandFault && ...
    double(robot.CommandAttemptCount) == commandAttemptCountAtFirstFault;
summary.ServoTimingCandidatePassed = ...
    commandTargetCount >= 30 && commandChannelHealthy && ...
    robot.ProgrammaticEnableConfirmed && ...
    robot.ProgrammaticSpeedFactorConfirmed && ...
    robot.ProgrammaticDisableConfirmed && ...
    strlength(disableFaultIdentifier) == 0 && ...
    summary.MotionTransportHealthyAtExit && ...
    summary.ServoSendDurationSec.P99 <= ...
        cfg.stage08.MaximumServoSendSec && ...
    summary.TargetFeedbackErrorRad.Maximum <= ...
        cfg.stage08.MaximumTargetFeedbackErrorRad;
summary.ServoTimingVerifiedWasNotAutomaticallyChanged = true;
summary.KeyboardInputIsNotPhysicalDeadman = true;
summary.ProgramCalledEnableRobot = robot.ProgrammaticEnableAttempted;
summary.ProgrammaticEnableConfirmed = robot.ProgrammaticEnableConfirmed;
summary.EnableRobotResponse = robot.ProgrammaticEnableResponse;
summary.ProgramCalledSpeedFactor = ...
    robot.ProgrammaticSpeedFactorAttempted;
summary.SpeedFactorConfirmed = ...
    robot.ProgrammaticSpeedFactorConfirmed;
summary.SpeedFactorResponse = ...
    robot.ProgrammaticSpeedFactorResponse;
summary.ProgramCalledDisableRobot = robot.ProgrammaticDisableAttempted;
summary.ProgrammaticDisableConfirmed = ...
    robot.ProgrammaticDisableConfirmed;
summary.DisableRobotResponse = robot.ProgrammaticDisableResponse;
summary.DisableFaultIdentifier = disableFaultIdentifier;
summary.DisableFaultMessage = disableFaultMessage;
summary.RequiresManualRcmMeasurementReview = true;

clear logCleanup;
if options.WriteResults
    writeJson(fullfile(outputDirectory, 'summary_worker.json'), summary);
    writeJson(fullfile(outputDirectory, 'config_snapshot.json'), cfg);
    writeJson(fullfile(outputDirectory, 'authorization.json'), authorization);
    if monitorOnlyAfterCommandFault
        writeJson(fullfile(outputDirectory, ...
            'command_fault_diagnostics.json'), commandFaultRecord);
    end
    save(fullfile(outputDirectory, 'drag_worker_result.mat'), ...
        'summary', 'cfg', 'authorization');
end
sendMessage(displayQueue, lifecycle( ...
    "FINISHED", ["后台控制 worker 已停止；ServoJ 输出已关闭，" ...
    "DisableRobot 已执行。"]));
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
            validPhysical = isfield(message, 'SupportsPhysicalMotion') && ...
                logical(message.SupportsPhysicalMotion);
            state.Requested = validPhysical && ...
                isfield(message, 'Requested') && logical(message.Requested);
            if isfield(message, 'Source')
                state.Source = string(message.Source);
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
state = struct('Requested', false, 'Source', "software_released", ...
    'ResetPending', false, 'StopRequested', false, ...
    'LastMessageWorkerSec', NaN);
end

function context = forceContext(robotState, handleRequested)
context = scopeguide.types.forceProcessingContext();
context.HandleEnabled = logical(handleRequested);
context.RobotStationary = ...
    scopeguide.force.isRobotStationaryForBaseline(robotState, ...
    MaximumJointSpeedDegSec=2.0, ...
    MaximumTcpTranslationSpeedMmSec=3.0, ...
    MaximumTcpRotationSpeedDegSec=2.0);
context.NoContactConfirmed = ~logical(handleRequested);
context.AllowBaselineUpdate = ~logical(handleRequested);
end

function code = healthCode(commandHealthy, trackingHealthy)
if ~commandHealthy
    code = "COMMAND_CHANNEL_FAILURE";
elseif ~trackingHealthy
    code = "TARGET_TRACKING_ERROR";
else
    code = "OK";
end
end

function directory = createOutputDirectory(projectRoot, dofMode)
timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
directory = fullfile(projectRoot, 'results', ...
    "scopeguide_3dof_drag_" + dofMode + "_" + timestamp);
[created, message] = mkdir(directory);
if ~created && ~isfolder(directory)
    error('scopeguide:stage08:CannotCreateResultDirectory', ...
        'Cannot create %s: %s', directory, message);
end
end

function [fileId, cleanup] = openCommandLog(path)
[fileId, message] = fopen(path, 'w');
if fileId < 0
    error('scopeguide:stage08:CannotOpenLog', ...
        'Cannot open %s: %s', path, message);
end
cleanup = onCleanup(@() fclose(fileId));
names = ["timeSec", "dtSec", "sensorSeq", "robotSeq", ...
    "uiAgeSec", "fsm", "fault", "commandEligible", "servoSent", ...
    "servoSendSec", "trackingErrorRad", "rcmErrorM", ...
    "forceQualityStatus", "wrenchSafetyStopCode", ...
    "wrenchSafetyStopReasons", "rawForceNormN", "rawMomentNormNm", ...
    "fastForceNormN", "fastMomentNormNm", "fastForceRateNSec", ...
    "forceX", "forceY", "forceZ", ...
    "qdotCmd1", "qdotCmd2", "qdotCmd3", ...
    "qdotCmd4", "qdotCmd5", "qdotCmd6", ...
    "qdotActual1", "qdotActual2", "qdotActual3", ...
    "qdotActual4", "qdotActual5", "qdotActual6", ...
    "qdotQpCandidate1", "qdotQpCandidate2", ...
    "qdotQpCandidate3", "qdotQpCandidate4", ...
    "qdotQpCandidate5", "qdotQpCandidate6", ...
    "q1", "q2", "q3", "q4", "q5", "q6", ...
    "target1", "target2", "target3", "target4", "target5", "target6"];
fprintf(fileId, '%s\n', strjoin(names, ','));
end

function writeLogRow(fileId, timeSec, frame, decision, ...
        responseSec, trackingError, uiAgeSec)
rcmError = NaN;
if decision.GeometryValid
    rcmError = decision.Geometry.RcmErrorNormM;
end
fprintf(fileId, ['%.9g,%.9g,%.0f,%.0f,%.9g,%s,%s,%.0f,%.0f,' ...
    '%.9g,%.9g,%.9g,%s,%s,%s,%.9g,%.9g,%.9g,%.9g,%.9g'], ...
    timeSec, decision.DtSec, ...
    double(frame.RawSensorSample.sequence), ...
    double(frame.RobotState.FeedbackSequence), uiAgeSec, ...
    sanitize(decision.EnableStatus.State), ...
    sanitize(decision.EnableStatus.FaultCode), ...
    double(decision.CommandEligible), ...
    double(decision.MotionCommandSent), responseSec, trackingError, rcmError, ...
    sanitize(decision.Safety.ForceStatusCode), ...
    sanitize(decision.Safety.WrenchSafetyStopCode), ...
    sanitize(safetyStopReasons(frame.Processed)), ...
    safetyMetric(frame.Processed, 'RawForceNormN'), ...
    safetyMetric(frame.Processed, 'RawMomentNormNm'), ...
    safetyMetric(frame.Processed, 'FastForceNormN'), ...
    safetyMetric(frame.Processed, 'FastMomentNormNm'), ...
    safetyMetric(frame.Processed, 'FastForceRateNSec'));
numeric = [frame.Processed.ControlForceTool(:); ...
    decision.QdotCommandRadSec(:); ...
    frame.RobotState.JointVelocityRadSec(:); ...
    decision.QdotQpCandidateRadSec(:); ...
    frame.RobotState.JointPositionRad(:); ...
    decision.QTargetCommandRad(:)];
fprintf(fileId, repmat(',%.9g', 1, numel(numeric)), numeric);
fprintf(fileId, '\n');
end

function value = safetyMetric(processed, fieldName)
value = NaN;
if isfield(processed, 'Safety') && isstruct(processed.Safety) && ...
        isfield(processed.Safety, fieldName)
    candidate = double(processed.Safety.(fieldName));
    if isscalar(candidate) && isfinite(candidate)
        value = candidate;
    end
end
end

function value = safetyStopReasons(processed)
value = "";
if isfield(processed, 'Safety') && isstruct(processed.Safety) && ...
        isfield(processed.Safety, 'StopReasons')
    reasons = string(processed.Safety.StopReasons(:));
    reasons = reasons(~ismissing(reasons) & strlength(reasons) > 0);
    if ~isempty(reasons)
        value = strjoin(reasons, "+");
    end
end
end

function values = compactCodes(values, retainOk)
values = string(values(:));
values = values(~ismissing(values) & strlength(values) > 0);
if ~retainOk
    values = values(values ~= "OK");
end
values = unique(values).';
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

function message = lifecycle(phase, text)
message = struct('Type', "lifecycle", 'Phase', string(phase), ...
    'Text', string(text), 'Identifier', "");
end

function record = captureCommandFault(robot, exception, timeSec, source)
record = struct();
record.Source = string(source);
record.TimeSec = double(timeSec);
record.ExceptionIdentifier = string(exception.identifier);
record.ExceptionMessage = string(exception.message);
record.RobotCommandAttemptCount = double(robot.CommandAttemptCount);
record.RobotCommandSentCount = double(robot.CommandSentCount);
record.RobotCommandFailureCount = double(robot.CommandFailureCount);
record.LastRobotState = compactRobotState(robot.LastRobotState);
record.Transport = robot.getLastMotionCommandDiagnostics();
end

function state = compactRobotState(candidate)
state = struct();
if isempty(candidate) || ~isstruct(candidate)
    return;
end
fields = {'FeedbackSequence', 'SampleAgeSec', 'RobotMode', ...
    'StatusCode', 'IsValid', 'JointPositionRad', ...
    'JointVelocityRadSec'};
for index = 1:numel(fields)
    name = fields{index};
    if isfield(candidate, name)
        state.(name) = candidate.(name);
    end
end
end

function text = commandFaultText(record)
text = sprintf(['运动命令通道已锁死，之后不再发送 ServoJ；' ...
    '力/机器人反馈、曲线和日志将继续。故障：%s'], ...
    record.ExceptionIdentifier);
end

function sendMessage(queue, message)
try
    send(queue, message);
catch
end
end

function stats = statistics(values)
values = values(isfinite(values));
stats = struct('Count', numel(values), 'Mean', NaN, 'P50', NaN, ...
    'P95', NaN, 'P99', NaN, 'Maximum', NaN);
if isempty(values)
    return;
end
values = sort(values(:));
stats.Mean = mean(values);
stats.P50 = percentile(values, 50);
stats.P95 = percentile(values, 95);
stats.P99 = percentile(values, 99);
stats.Maximum = max(values);
end

function values = transportResponseLatencies(diagnostics)
values = zeros(0, 1);
if isempty(diagnostics) || ~isstruct(diagnostics) || ...
        ~isfield(diagnostics, 'ResponseLatenciesSec')
    return;
end
candidate = double(diagnostics.ResponseLatenciesSec(:));
values = candidate(isfinite(candidate) & candidate >= 0);
end

function expected = transportResponseExpected(diagnostics)
% ServoJ itself is documented and implemented as no-reply. Missing policy
% metadata does not manufacture a per-command reply obligation; transport
% health below will independently reject an incomplete backend contract.
expected = false;
if isempty(diagnostics) || ~isstruct(diagnostics) || ...
        ~isfield(diagnostics, 'ResponseExpected') || ...
        ~isscalar(diagnostics.ResponseExpected)
    return;
end
expected = logical(diagnostics.ResponseExpected);
end

function healthy = servoTransportHealthy(diagnostics)
healthy = false;
required = {'Mode', 'ErrorCount', 'PendingResponseCount', ...
    'ResponseExpected', 'ReplyDeadlinePolicy', 'FaultIdentifier'};
if isempty(diagnostics) || ~isstruct(diagnostics) || ...
        ~all(isfield(diagnostics, required))
    return;
end
mode = string(diagnostics.Mode);
policy = string(diagnostics.ReplyDeadlinePolicy);
validMode = (mode == "legacy_immediate_optional_response" && ...
    policy == "single_immediate_read_no_wait") || ...
    (mode == "nonblocking_no_reply_expected" && ...
    policy == "not_applied_to_servoj");
healthy = validMode && ...
    ~logical(diagnostics.ResponseExpected) && ...
    double(diagnostics.ErrorCount) == 0 && ...
    strlength(string(diagnostics.FaultIdentifier)) == 0 && ...
    double(diagnostics.PendingResponseCount) == 0;
end

function value = percentile(values, percentage)
position = 1 + (numel(values) - 1) * percentage / 100;
lower = floor(position);
upper = ceil(position);
fraction = position - lower;
value = values(lower) * (1 - fraction) + values(upper) * fraction;
end

function ensureCapacity(index, capacity)
if index > capacity
    error('scopeguide:stage08:LogCapacityExceeded', ...
        'Stage 8 control log capacity exceeded.');
end
end

function value = sanitize(input)
value = char(replace(string(input), [",", newline], [";", " "]));
end

function writeJson(path, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end
[fileId, message] = fopen(path, 'w');
if fileId < 0
    error('scopeguide:stage08:CannotWriteJson', ...
        'Cannot open %s: %s', path, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end

function validateWorkerOptions(options)
required = {'DurationSec', 'DisplayRateHz', 'StatusRateHz', ...
    'WriteResults', 'EnableAdaptiveFzBaseline', ...
    'InputHeartbeatTimeoutSec'};
for index = 1:numel(required)
    if ~isfield(options, required{index})
        error('scopeguide:stage08:IncompleteWorkerOptions', ...
            'Worker option %s is missing.', required{index});
    end
end
values = [options.DurationSec, options.DisplayRateHz, ...
    options.StatusRateHz, options.InputHeartbeatTimeoutSec];
if any(~isfinite(values)) || any(values <= 0)
    error('scopeguide:stage08:InvalidWorkerOptions', ...
        'Worker timing values must be finite and positive.');
end
end
