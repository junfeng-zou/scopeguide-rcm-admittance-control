function [summary, outputDirectory] = run_cartesian_admittance_drag(options)
%RUN_CARTESIAN_ADMITTANCE_DRAG Physical 6D admittance without RCM.
% Space latches software motion ON, Esc turns it OFF immediately, and R
% requests a reset after the force is neutral.  This is not a physical
% deadman.  Robot/HEX-H I/O executes on one process worker; graphics and
% keyboard/mouse callbacks remain on the MATLAB client.

arguments
    options.Config = struct([])
    options.SensorConfig = struct([])
    options.DurationSec (1, 1) double = 60
    options.WindowSec (1, 1) double = 20
    options.PlotRateHz (1, 1) double = 10
    options.StatusRateHz (1, 1) double = 2
    options.InputHeartbeatRateHz (1, 1) double = 50
    options.DofMode (1, 1) string {mustBeMember(options.DofMode, ...
        ["hold", "translation_z", "translation_xyz", ...
         "rotation_x", "rotation_y", "rotation_z", ...
         "rotation_xyz", "full_6dof"])} = "translation_z"
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
    baseCfg = current_cartesian_admittance_drag_config();
else
    baseCfg = options.Config;
end
cfg = scopeguide.runtime.configureCartesianAdmittanceDrag( ...
    baseCfg, options.DofMode);
cfg.force.AutomaticBaselineEnabled = options.EnableAdaptiveFzBaseline;
validateCartesianAdmittanceDragConfig(cfg);
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
    scopeguide.safety.evaluateCartesianDragAuthorization( ...
    cfg, confirmations);
if ~options.ArmPhysicalMotion
    error('scopeguide:cartesianDrag:PhysicalMotionNotArmed', ...
        ['Physical motion defaults to OFF. Set ArmPhysicalMotion=true ' ...
         'only after checking the fixture, clearance and emergency stop.']);
end
if ~authorization.CommissioningAllowed
    error('scopeguide:cartesianDrag:AuthorizationBlocked', ...
        'Cartesian drag is blocked by: %s.', ...
        strjoin(authorization.FailedGates, ', '));
end
if exist('parpool', 'file') ~= 2 || exist('gcp', 'file') ~= 2
    error('scopeguide:cartesianDrag:ProcessPoolRequired', ...
        'Parallel Computing Toolbox with a Processes pool is required.');
end
pool = gcp('nocreate');
if isempty(pool)
    pool = parpool('Processes', 1);
end
if ~isa(pool, 'parallel.ProcessPool')
    error('scopeguide:cartesianDrag:ProcessPoolRequired', ...
        'Close the current non-process pool and use a Processes pool.');
end

parameters = scopeguide.control.deriveCartesianAdmittanceParameters(cfg);
fprintf(['NO-RCM Cartesian admittance drag: DOF=%s; control=%.1f Hz; ' ...
    'plot=%.1f Hz.\n'], options.DofMode, ...
    cfg.runtime.ControlRateHz, options.PlotRateHz);
fprintf(['RCM point, RCM error, RCM recovery and RCM QP rows are NOT ' ...
    'used in this controller.\n']);
fprintf(['Translation target/max=%.2f/%.2f mm/s at %.1f N.\n'], ...
    1e3 * parameters.TargetSteadyVelocity(1), ...
    1e3 * parameters.MaximumVelocity(1), ...
    parameters.DesignEffort(1));
fprintf(['Rotation target/max=%.2f/%.2f deg/s at %.2f N*m.\n'], ...
    rad2deg(parameters.TargetSteadyVelocity(4)), ...
    rad2deg(parameters.MaximumVelocity(4)), ...
    parameters.DesignEffort(4));
if parameters.TravelLimitsEnabled
    fprintf(['Relative travel limit=[%.1f %.1f %.1f] mm / ' ...
        '[%.1f %.1f %.1f] deg.\n'], ...
        1e3 * parameters.RelativeLimit(1:3), ...
        rad2deg(parameters.RelativeLimit(4:6)));
    fprintf(['Travel recovery band: translation=%.1f mm, ' ...
        'rotation=%.1f deg; outward motion is blocked and inward ' ...
        'return remains enabled.\n'], ...
        1e3 * cfg.cartesianDrag.Translation.RecoveryToleranceM, ...
        rad2deg(cfg.cartesianDrag.Rotation.RecoveryToleranceRad));
else
    fprintf(['Relative Cartesian travel limits: DISABLED for both ' ...
        'translation and rotation. Joint/singularity/wrench/ServoJ ' ...
        'protections remain active.\n']);
end
fprintf(['ServoJ: t=%.4f s, lookahead_time=%.1f, gain=%.1f; ' ...
    'EnableRobot(%.3f), SpeedFactor(%d).\n'], ...
    cfg.runtime.NominalDtSec, cfg.robot.ServoJLookaheadTime, ...
    cfg.robot.ServoJGain, cfg.stage08.EnablePayloadKg, ...
    cfg.stage08.SpeedFactorPercent);
fprintf(['Software latch only: Space -> ON, Esc -> immediate OFF, ' ...
    'R -> reset. Keep the pendant and emergency stop reachable.\n']);

commandQueue = parallel.pool.PollableDataQueue(Destination="any");
displayQueue = parallel.pool.PollableDataQueue(Destination="any");
dashboard = scopeguide.ui.CartesianDragDashboard( ...
    cfg, options.WindowSec, options.DofMode);
workerOptions = struct( ...
    'DurationSec', options.DurationSec, ...
    'DisplayRateHz', options.PlotRateHz, ...
    'StatusRateHz', options.StatusRateHz, ...
    'WriteResults', options.WriteResults, ...
    'EnableAdaptiveFzBaseline', options.EnableAdaptiveFzBaseline, ...
    'InputHeartbeatTimeoutSec', ...
        cfg.cartesianDrag.InputHeartbeatTimeoutSec, ...
    'ControllerFamily', "cartesian_drag");
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
    error('scopeguide:cartesianDrag:WorkerStopTimeout', ...
        ['The control worker did not stop within %.1f s. Use the ' ...
         'emergency stop and inspect the robot connection.'], ...
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
fprintf(['No-RCM Cartesian drag stopped: %s. ServoJ sent=%d, ' ...
    'failures=%d.\n'], summary.TerminationReason, ...
    summary.RobotCommandSentCount, summary.RobotCommandFailureCount);
fprintf(['Control %.1f Hz; Servo send P99 %.3f ms; tracking max ' ...
    '%.3f deg.\n'], summary.ActualControlRateHz, ...
    1e3 * summary.ServoSendDurationSec.P99, ...
    rad2deg(summary.TargetFeedbackErrorRad.Maximum));
if options.WriteResults
    fprintf('Results: %s\n', outputDirectory);
end
if options.KeepFigureOpenAfterRun && dashboard.isOpen()
    dashboard.freezeForReview(summary.TerminationReason);
    fprintf('Final figure retained; close it manually after review.\n');
end
clear cleanup;
close(commandQueue);
close(displayQueue);
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
        fprintf('[Cartesian drag worker:%s] %s\n', ...
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

function cleanupClient(dashboard, future, commandQueue, keepFigure, timeout)
sendStop(commandQueue, "CLIENT_CLEANUP");
try
    if string(future.State) ~= "finished"
        wait(future, "finished", timeout);
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
    if keepFigure
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
    error('scopeguide:cartesianDrag:InvalidOptions', ...
        'Timing values must be finite and positive.');
end
if options.InputHeartbeatRateHz < 20
    error('scopeguide:cartesianDrag:InputHeartbeatTooSlow', ...
        'Input heartbeat must be at least 20 Hz.');
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
    error('scopeguide:cartesianDrag:CannotWriteJson', ...
        'Cannot open %s: %s', path, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end
