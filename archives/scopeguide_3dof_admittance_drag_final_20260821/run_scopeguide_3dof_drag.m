function [summary, outputDirectory] = ...
        run_scopeguide_3dof_drag(options)
%RUN_SCOPEGUIDE_3DOF_DRAG Parallel physical three-DOF admittance drag.
% This entry point performs the configured software EnableRobot/SpeedFactor
% initialization. The operator must verify the independent emergency stop
% and explicitly satisfy every typed/boolean motion gate.

arguments
    options.Config = struct([])
    options.SensorConfig = struct([])
    options.DurationSec (1, 1) double = 30
    options.WindowSec (1, 1) double = 30
    options.PlotRateHz (1, 1) double = 10
    options.StatusRateHz (1, 1) double = 2
    options.InputHeartbeatRateHz (1, 1) double = 50
    options.DofMode (1, 1) string {mustBeMember(options.DofMode, ...
        ["hold", "insertion_only", "pivot1_only", "pivot2_only", ...
        "dual_pivot", "full_3dof"])} = "insertion_only"
    options.ArmPhysicalMotion (1, 1) logical = false
    options.MotionConfirmation (1, 1) string = ""
    options.SetupConfirmation (1, 1) string = ""
    options.SafetyThresholdConfirmation (1, 1) string = ""
    options.SecondObserverPresent (1, 1) logical = false
    options.FixtureAndClearanceConfirmed (1, 1) logical = false
    options.WriteResults (1, 1) logical = true
    options.EnableAdaptiveFzBaseline (1, 1) logical = true
    options.KeepFigureOpenAfterRun (1, 1) logical = true
end

validateOptions(options);
projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'force_sensor'));
addpath(fullfile(projectRoot, 'force_calibration'));
addpath(fullfile(projectRoot, 'robot'));
if isempty(options.Config)
    baseCfg = defaultRcmAdmittanceConfig();
else
    baseCfg = options.Config;
end
cfg = scopeguide.runtime.configureStage08Commissioning( ...
    baseCfg, options.DofMode);
cfg.force.AutomaticBaselineEnabled = options.EnableAdaptiveFzBaseline;
validateRcmAdmittanceConfig(cfg);
if isempty(options.SensorConfig)
    sensorCfg = onrobot.defaultConfig();
else
    sensorCfg = options.SensorConfig;
end
confirmations = struct( ...
    'MotionPhrase', options.MotionConfirmation, ...
    'SetupPhrase', options.SetupConfirmation, ...
    'SafetyThresholdPhrase', options.SafetyThresholdConfirmation, ...
    'SecondObserverPresent', options.SecondObserverPresent, ...
    'FixtureAndClearanceConfirmed', ...
        options.FixtureAndClearanceConfirmed);
authorization = ...
    scopeguide.safety.evaluateStage08CommissioningAuthorization( ...
    cfg, confirmations);
if ~options.ArmPhysicalMotion
    error('scopeguide:stage08:PhysicalMotionNotArmed', ...
        ['Physical motion defaults to OFF. Set ArmPhysicalMotion=true ' ...
         'only after ' ...
        'the fixture, independent emergency stop and exit space are ready.']);
end
if ~authorization.CommissioningAllowed
    error('scopeguide:stage08:AuthorizationBlocked', ...
        'Physical drag motion is blocked by: %s.', ...
        strjoin(authorization.FailedGates, ', '));
end
if exist('parpool', 'file') ~= 2 || exist('gcp', 'file') ~= 2
    error('scopeguide:stage08:ProcessPoolRequired', ...
        ['Three-DOF drag requires Parallel Computing Toolbox ' ...
         'process workers.']);
end
pool = gcp('nocreate');
if isempty(pool)
    pool = parpool('Processes', 1);
end
if ~isa(pool, 'parallel.ProcessPool')
    error('scopeguide:stage08:ProcessPoolRequired', ...
        'Close the current non-process pool and start a Processes pool.');
end

fprintf(['ScopeGuide 3-DOF ADMITTANCE DRAG: DOF=%s; ' ...
    'speed scale=%.0f%%; control=%.1f Hz; plot=%.1f Hz.\n'], ...
    options.DofMode, 100 * cfg.stage08.SpeedScale, ...
    cfg.runtime.ControlRateHz, options.PlotRateHz);
fprintf(['Pivot admittance feel scale=%.0f%%; target=%.3f deg/s at ' ...
    'design load; PivotMax=%.3f deg/s.\n'], ...
    100 * cfg.stage08.PivotAdmittanceSpeedScale, ...
    rad2deg(cfg.admittance.Pivot.TargetSteadySpeedRadSec), ...
    rad2deg(cfg.control.PivotMaxRadSec));
fprintf(['Relative travel recovery band: pivot=%.1f deg, ' ...
    'insertion=%.1f mm; outward motion is blocked and inward return ' ...
    'remains enabled before the outer hard boundary.\n'], ...
    rad2deg(cfg.control.RelativePivotRecoveryToleranceRad), ...
    1e3 * cfg.control.RelativeInsertionRecoveryToleranceM);
fprintf(['ServoJ parameters: t=%.4f s (= control period), ' ...
    'lookahead_time=%.1f, gain=%.1f.\n'], ...
    cfg.runtime.NominalDtSec, cfg.robot.ServoJLookaheadTime, ...
    cfg.robot.ServoJGain);
fprintf(['Input is a software latch, NOT a physical deadman: ' ...
    'Space -> ON, Esc -> immediate OFF, R -> reset after release.\n']);
fprintf(['Software robot initialization: EnableRobot(%.3f), ' ...
    'SpeedFactor(%d), wait %.1f s, then require 30004 confirmation.\n'], ...
    cfg.stage08.EnablePayloadKg, cfg.stage08.SpeedFactorPercent, ...
    cfg.stage08.PostEnablePauseSec);
fprintf(['DisableRobot() will be sent during normal exit and cleanup. ' ...
    'Keep the pendant and emergency stop reachable.\n']);
if authorization.ServoTimingBootstrapUsed
    fprintf(['ServoTiming bootstrap is active and restricted to ' ...
        'insertion_only. Review the generated timing report afterward.\n']);
end

commandQueue = parallel.pool.PollableDataQueue(Destination="any");
displayQueue = parallel.pool.PollableDataQueue(Destination="any");
dashboard = scopeguide.ui.Stage07DryRunDashboard( ...
    cfg, options.WindowSec, EnablePlot=true, ...
    PhysicalMotionMode=true, DofMode=options.DofMode);
workerOptions = struct( ...
    'DurationSec', options.DurationSec, ...
    'DisplayRateHz', options.PlotRateHz, ...
    'StatusRateHz', options.StatusRateHz, ...
    'WriteResults', options.WriteResults, ...
    'EnableAdaptiveFzBaseline', options.EnableAdaptiveFzBaseline, ...
    'InputHeartbeatTimeoutSec', cfg.stage08.InputHeartbeatTimeoutSec);
future = parfeval(pool, @scopeguide.runtime.runStage08FixtureWorker, ...
    2, cfg, sensorCfg, workerOptions, confirmations, ...
    commandQueue, displayQueue);
workerStopTimeoutSec = max(3.0, ...
    cfg.robot.InitialFeedbackTimeoutSec + 1.0);
cleanup = onCleanup(@() cleanupClient( ...
    dashboard, future, commandQueue, options.KeepFigureOpenAfterRun, ...
    workerStopTimeoutSec));

clientClock = tic;
inputPeriodSec = 1 / options.InputHeartbeatRateHz;
nextInputSec = 0;
sequence = uint64(0);
rendered = 0;
received = 0;
dropped = 0;
terminationReason = "WORKER_FINISHED";
while dashboard.isOpen()
    nowSec = toc(clientClock);
    guiInput = dashboard.readInput();
    if nowSec >= nextInputSec || guiInput.ResetRequested || ...
            guiInput.CloseRequested
        sequence = sequence + uint64(1);
        send(commandQueue, struct('Type', "input", ...
            'Sequence', sequence, 'Requested', guiInput.Requested, ...
            'Source', guiInput.Source, ...
            'ResetRequested', guiInput.ResetRequested, ...
            'CloseRequested', guiInput.CloseRequested, ...
            'SupportsPhysicalMotion', true));
        nextInputSec = nowSec + inputPeriodSec;
    end
    if guiInput.CloseRequested
        terminationReason = "WINDOW_CLOSED";
        break;
    end
    [latest, count] = drainDisplayQueue(displayQueue);
    received = received + count;
    if count > 0
        dropped = dropped + max(0, count - 1);
        dashboard.updateFromSnapshot(latest);
        rendered = rendered + 1;
    else
        drawnow limitrate;
    end
    if string(future.State) == "finished"
        break;
    end
    pause(0.001);
end
if string(future.State) ~= "finished"
    sendStop(commandQueue, terminationReason);
    wait(future, "finished", workerStopTimeoutSec);
end
if string(future.State) ~= "finished"
    cancel(future);
    error('scopeguide:stage08:WorkerStopTimeout', ...
        ['Drag worker did not stop within %.1f s and was cancelled. ' ...
         'Use the robot emergency stop and inspect the connection.'], ...
        workerStopTimeoutSec);
end
[summary, outputDirectory] = fetchOutputs(future);
summary.ClientTelemetryReceivedCount = received;
summary.ClientTelemetryRenderedCount = rendered;
summary.ClientTelemetryDroppedCount = dropped;
summary.ClientTerminationReason = terminationReason;
summary.KeyboardInputDiagnostics = dashboard.keyboardDiagnostics();
summary.ControlAndGraphicsDecoupled = true;
summary.PlotRetainedForManualReview = ...
    options.KeepFigureOpenAfterRun && dashboard.isOpen();
if options.WriteResults && strlength(outputDirectory) > 0
    writeJson(fullfile(outputDirectory, 'summary.json'), summary);
end
fprintf('ScopeGuide drag stopped: %s. ServoJ sent=%d, failures=%d.\n', ...
    summary.TerminationReason, summary.RobotCommandSentCount, ...
    summary.RobotCommandFailureCount);
fprintf(['Software robot state: EnableRobot confirmed=%d; ' ...
    'SpeedFactor confirmed=%d; DisableRobot confirmed=%d.\n'], ...
    summary.ProgrammaticEnableConfirmed, ...
    summary.SpeedFactorConfirmed, ...
    summary.ProgrammaticDisableConfirmed);
if isfield(summary, 'MonitorOnlyAfterCommandFault') && ...
        summary.MonitorOnlyAfterCommandFault
    fprintf(['Motion output was latched OFF after %s; monitoring ' ...
        'continued for %.3f s and no command retry was attempted.\n'], ...
        summary.FaultIdentifier, ...
        summary.MonitoringContinuedAfterCommandFaultSec);
    fprintf('Command diagnostics: %s\n', fullfile(outputDirectory, ...
        'command_fault_diagnostics.json'));
end
if isfield(summary, 'ServoResponseExpected') && ...
        ~summary.ServoResponseExpected
    optionalResponseCount = optionalServoResponseCount(summary);
    fprintf(['Control %.1f Hz; Servo send P99 %.3f ms; ' ...
        '30003 reply=not required (optional received=%d); ' ...
        'tracking max %.3f deg.\n'], summary.ActualControlRateHz, ...
        1e3 * summary.ServoSendDurationSec.P99, ...
        optionalResponseCount, ...
        rad2deg(summary.TargetFeedbackErrorRad.Maximum));
else
    fprintf(['Control %.1f Hz; Servo send/response P99 %.3f/%.3f ms; ' ...
        'tracking max %.3f deg.\n'], summary.ActualControlRateHz, ...
        1e3 * summary.ServoSendDurationSec.P99, ...
        1e3 * summary.ServoResponseDurationSec.P99, ...
        rad2deg(summary.TargetFeedbackErrorRad.Maximum));
end
if options.WriteResults
    fprintf('Results: %s\n', outputDirectory);
end
if options.KeepFigureOpenAfterRun && dashboard.isOpen()
    dashboard.freezeForReview(summary.TerminationReason);
    fprintf(['Final drag figure retained for manual review; ' ...
        'close it manually when finished.\n']);
end
clear cleanup;
close(commandQueue);
close(displayQueue);
end

function count = optionalServoResponseCount(summary)
count = 0;
if isfield(summary, 'MotionTransportDiagnostics') && ...
        isstruct(summary.MotionTransportDiagnostics) && ...
        isfield(summary.MotionTransportDiagnostics, ...
            'OptionalResponseCount')
    candidate = double( ...
        summary.MotionTransportDiagnostics.OptionalResponseCount);
    if isscalar(candidate) && isfinite(candidate) && candidate >= 0
        count = floor(candidate);
    end
end
end

function [latest, count] = drainDisplayQueue(queue)
latest = struct([]);
count = 0;
while true
    [message, ok] = poll(queue, 0);
    if ~ok
        break;
    end
    if ~isstruct(message) || ~isfield(message, 'Type')
        continue;
    end
    if string(message.Type) == "telemetry"
        latest = message;
        count = count + 1;
    elseif string(message.Type) == "lifecycle"
        fprintf('[ScopeGuide drag worker:%s] %s\n', ...
            string(message.Phase), string(message.Text));
    end
end
end

function sendStop(queue, reason)
try
    send(queue, struct('Type', "stop", 'Reason', string(reason), ...
        'SupportsPhysicalMotion', true));
catch
end
end

function cleanupClient(dashboard, future, commandQueue, keepFigureOpen, ...
        workerStopTimeoutSec)
sendStop(commandQueue, "CLIENT_CLEANUP");
try
    if string(future.State) ~= "finished"
        wait(future, "finished", workerStopTimeoutSec);
    end
catch
end
try
    if string(future.State) ~= "finished"
        cancel(future);
    end
catch
end
try
    if keepFigureOpen
        dashboard.freezeForReview("CLIENT_CLEANUP");
    else
        dashboard.close();
    end
catch
end
end

function validateOptions(options)
values = [options.DurationSec, options.WindowSec, options.PlotRateHz, ...
    options.StatusRateHz, options.InputHeartbeatRateHz];
if any(~isfinite(values)) || any(values <= 0)
    error('scopeguide:stage08:InvalidOptions', ...
        'Drag timing values must be finite and positive.');
end
if options.InputHeartbeatRateHz < 20
    error('scopeguide:stage08:InputHeartbeatTooSlow', ...
        'Drag input heartbeat must be at least 20 Hz.');
end
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
