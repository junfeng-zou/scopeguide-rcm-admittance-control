function [summary, outputDirectory] = ...
        run_stage07_live_readonly_parallel(options)
%RUN_STAGE07_LIVE_READONLY_PARALLEL Decoupled Stage 7 control and plotting.
% Hardware acquisition/control execute on a background worker.  This
% client function owns only the keyboard/mouse dashboard and rendering.
% The current project MATLAB installation uses backgroundPool threads;
% when a process pool is available, ExecutionBackend="auto" prefers it.

arguments
    options.Config = struct([])
    options.SensorConfig = struct([])
    options.DurationSec (1, 1) double = 600
    options.PlotRateHz (1, 1) double = 5
    options.WindowSec (1, 1) double = 15
    options.StatusRateHz (1, 1) double = 2
    options.InputHeartbeatRateHz (1, 1) double = 50
    options.InputHeartbeatTimeoutSec (1, 1) double = 0.250
    options.WriteResults (1, 1) logical = true
    options.EnableAdaptiveFzBaseline (1, 1) logical = true
    options.FaultInjectionProfile (1, 1) string ...
        {mustBeMember(options.FaultInjectionProfile, ...
        ["none", "acceptance"])} = "none"
    options.ExecutionBackend (1, 1) string ...
        {mustBeMember(options.ExecutionBackend, ...
        ["auto", "process", "thread"])} = "auto"
    options.MaximumStationaryJointSpeedDegSec (1, 1) double = 2.0
    options.MaximumStationaryTcpTranslationSpeedMmSec ...
        (1, 1) double = 3.0
    options.MaximumStationaryTcpRotationSpeedDegSec ...
        (1, 1) double = 2.0
end

validateTimingOptions(options);
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
if isempty(options.SensorConfig)
    sensorCfg = onrobot.defaultConfig();
else
    sensorCfg = options.SensorConfig;
end

[executor, backend, ownsProcessPool] = ...
    selectExecutor(options.ExecutionBackend);
fprintf(['Stage 7 parallel READ-ONLY dry-run. Backend=%s; ' ...
    'plot=client %.1f Hz; control=worker %.1f Hz.\n'], ...
    backend, options.PlotRateHz, cfg.runtime.ControlRateHz);
fprintf('pRcmBase = [%+.4f %+.4f %+.4f] m.\n', ...
    cfg.rcm.PointBaseM);
fprintf(['No physical motion can be authorized: EnableMotion=false, ' ...
    'DryRun=true, software input SupportsPhysicalMotion=false.\n']);
fprintf(['Input: press SPACE once to latch prediction ON; press ESC to ' ...
    'turn it OFF. Key release is intentionally ignored.\n']);
if startsWith(backend, "thread")
    fprintf(['Using MATLAB backgroundPool because a process pool is not ' ...
        'available in this installation. Graphics remain on the client.\n']);
end
if options.FaultInjectionProfile == "acceptance"
    printFaultAcceptanceInstructions();
end

commandQueue = parallel.pool.PollableDataQueue(Destination="any");
displayQueue = parallel.pool.PollableDataQueue(Destination="any");
dashboard = scopeguide.ui.Stage07DryRunDashboard( ...
    cfg, options.WindowSec, EnablePlot=true, ...
    FaultInjectionProfile=options.FaultInjectionProfile);

workerOptions = buildWorkerOptions(options);
future = parfeval(executor, ...
    @scopeguide.runtime.runStage07ReadOnlyWorker, 2, ...
    cfg, sensorCfg, workerOptions, commandQueue, displayQueue);
cleanup = onCleanup(@() cleanupParallelClient( ...
    dashboard, future, commandQueue));

inputPeriodSec = 1 / options.InputHeartbeatRateHz;
clientClock = tic;
nextInputSec = 0;
inputSequence = uint64(0);
inputMessageCount = 0;
telemetryReceivedCount = 0;
telemetryRenderedCount = 0;
telemetryDroppedCount = 0;
lifecycleMessageCount = 0;
renderDuration = nan(ceil(options.DurationSec * ...
    options.PlotRateHz * 1.2) + 100, 1);
renderCount = 0;
clientTerminationReason = "WORKER_FINISHED";
lastGuidanceToken = "";

while dashboard.isOpen()
    nowSec = toc(clientClock);
    guiInput = dashboard.readInput();
    if nowSec >= nextInputSec || guiInput.ResetRequested || ...
            guiInput.CloseRequested
        inputSequence = inputSequence + uint64(1);
        send(commandQueue, inputMessage( ...
            inputSequence, nowSec, guiInput));
        inputMessageCount = inputMessageCount + 1;
        nextInputSec = nowSec + inputPeriodSec;
    end
    if guiInput.CloseRequested
        clientTerminationReason = "WINDOW_CLOSED";
        break;
    end

    [latest, received, lifecycle] = ...
        drainDisplayQueue(displayQueue);
    telemetryReceivedCount = telemetryReceivedCount + received;
    lifecycleMessageCount = lifecycleMessageCount + lifecycle;
    if received > 0
        telemetryDroppedCount = telemetryDroppedCount + max(0, received - 1);
        renderCount = renderCount + 1;
        ensureCapacity(renderCount, numel(renderDuration), 'client render');
        renderClock = tic;
        dashboard.updateFromSnapshot(latest);
        guidance = scopeguide.runtime.stage07FaultGuidance( ...
            latest.TimeSec, options.FaultInjectionProfile);
        if strlength(guidance.ConsoleToken) > 0 && ...
                guidance.ConsoleToken ~= lastGuidanceToken
            fprintf('[Stage 7 故障引导] %s\n', ...
                char(guidance.MessageZh));
            lastGuidanceToken = guidance.ConsoleToken;
        end
        renderDuration(renderCount) = toc(renderClock);
        telemetryRenderedCount = telemetryRenderedCount + 1;
    else
        drawnow limitrate;
    end

    if string(future.State) == "finished"
        break;
    end
    pause(0.001);
end

if ~dashboard.isOpen() && string(future.State) ~= "finished" && ...
        clientTerminationReason == "WORKER_FINISHED"
    clientTerminationReason = "WINDOW_CLOSED";
end
if string(future.State) ~= "finished"
    sendStop(commandQueue, clientTerminationReason);
    wait(future, "finished", 3.0);
end
if string(future.State) ~= "finished"
    cancel(future);
    error('scopeguide:stage07:WorkerStopTimeout', ...
        ['The Stage 7 worker did not stop within 3 s. It was cancelled; ' ...
        'inspect the hardware connection before retrying.']);
end

[summary, outputDirectory] = fetchOutputs(future);
summary.ParallelExecutionBackend = backend;
summary.ProcessPoolOwnedByRunner = ownsProcessPool;
summary.ClientTerminationReason = clientTerminationReason;
summary.ClientInputMessageCount = inputMessageCount;
summary.ClientLifecycleMessageCount = lifecycleMessageCount;
summary.ClientTelemetryReceivedCount = telemetryReceivedCount;
summary.ClientTelemetryRenderedCount = telemetryRenderedCount;
summary.ClientTelemetryDroppedCount = telemetryDroppedCount;
summary.ClientRenderDuration = statistics( ...
    renderDuration(1:renderCount));
summary.ClientGraphicsCreated = true;
summary.PlotEnabled = true;
summary.CurvePlotUpdateCount = telemetryRenderedCount;
summary.DashboardStatusUpdateCount = telemetryRenderedCount;
summary.ControlAndGraphicsDecoupled = true;
summary.InputHeartbeatRateHz = options.InputHeartbeatRateHz;
summary.InputHeartbeatTimeoutSec = options.InputHeartbeatTimeoutSec;
summary.KeyboardInputDiagnostics = dashboard.keyboardDiagnostics();
summary.PhysicalMotionCommandSent = false;
summary.FaultGuidanceEnabled = ...
    options.FaultInjectionProfile == "acceptance";
summary.FaultSchedule = scopeguide.runtime.stage07FaultSchedule();

if options.WriteResults && strlength(outputDirectory) > 0
    writeJson(fullfile(outputDirectory, 'summary.json'), summary);
end
fprintf('Stage 7 parallel read-only capture stopped: %s.\n', ...
    summary.TerminationReason);
fprintf(['Worker sensor %.1f Hz; control %.1f Hz; client rendered %d ' ...
    'frames; robot commands sent: %d.\n'], ...
    summary.ActualSensorRateHz, summary.ActualControlRateHz, ...
    telemetryRenderedCount, summary.RobotCommandSentCount);
if options.WriteResults
    fprintf('Results: %s\n', outputDirectory);
end

clear cleanup;
dashboard.close();
close(commandQueue);
close(displayQueue);
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

function [executor, backend, ownsPool] = selectExecutor(requested)
ownsPool = false;
hasProcessPool = exist('parpool', 'file') == 2 && ...
    exist('gcp', 'file') == 2;
if requested == "process" && ~hasProcessPool
    error('scopeguide:stage07:ProcessPoolUnavailable', ...
        ['ExecutionBackend="process" requires Parallel Computing ' ...
        'Toolbox. Use ExecutionBackend="thread" on this installation.']);
end
if requested == "thread" || ...
        (requested == "auto" && ~hasProcessPool)
    executor = backgroundPool;
    backend = "thread_backgroundPool";
    return;
end

try
    pool = gcp('nocreate');
    if isempty(pool)
        pool = parpool('Processes', 1);
        ownsPool = true;
    end
    if ~isa(pool, 'parallel.ProcessPool')
        if requested == "process"
            error('scopeguide:stage07:ExistingPoolNotProcess', ...
                'The existing parallel pool is not process based.');
        end
        executor = backgroundPool;
        backend = "thread_backgroundPool";
        return;
    end
    executor = pool;
    backend = "process_pool";
catch exception
    if requested == "process"
        rethrow(exception);
    end
    warning('scopeguide:stage07:ProcessPoolFallback', ...
        ['Process pool unavailable (%s). Falling back to ' ...
        'backgroundPool threads.'], exception.message);
    executor = backgroundPool;
    backend = "thread_backgroundPool";
end
end

function worker = buildWorkerOptions(options)
worker = struct();
worker.DurationSec = options.DurationSec;
worker.DisplayRateHz = options.PlotRateHz;
worker.StatusRateHz = options.StatusRateHz;
worker.WriteResults = options.WriteResults;
worker.EnableAdaptiveFzBaseline = options.EnableAdaptiveFzBaseline;
worker.FaultInjectionProfile = options.FaultInjectionProfile;
worker.MaximumStationaryJointSpeedDegSec = ...
    options.MaximumStationaryJointSpeedDegSec;
worker.MaximumStationaryTcpTranslationSpeedMmSec = ...
    options.MaximumStationaryTcpTranslationSpeedMmSec;
worker.MaximumStationaryTcpRotationSpeedDegSec = ...
    options.MaximumStationaryTcpRotationSpeedDegSec;
worker.InputHeartbeatTimeoutSec = options.InputHeartbeatTimeoutSec;
end

function message = inputMessage(sequence, timeSec, guiInput)
message = struct();
message.Type = "input";
message.Sequence = sequence;
message.ClientTimeSec = timeSec;
message.Requested = logical(guiInput.Requested);
message.Source = string(guiInput.Source);
message.ResetRequested = logical(guiInput.ResetRequested);
message.CloseRequested = logical(guiInput.CloseRequested);
message.SupportsPhysicalMotion = false;
end

function sendStop(queue, reason)
try
    send(queue, struct('Type', "stop", 'Reason', string(reason), ...
        'SupportsPhysicalMotion', false));
catch
end
end

function [latest, telemetryCount, lifecycleCount] = ...
        drainDisplayQueue(queue)
latest = struct([]);
telemetryCount = 0;
lifecycleCount = 0;
while true
    [message, ok] = poll(queue, 0);
    if ~ok
        break;
    end
    if ~isstruct(message) || ~isfield(message, 'Type')
        continue;
    end
    switch string(message.Type)
        case "telemetry"
            latest = message;
            telemetryCount = telemetryCount + 1;
        case "lifecycle"
            lifecycleCount = lifecycleCount + 1;
            fprintf('[Stage 7 worker:%s] %s\n', ...
                string(message.Phase), string(message.Text));
    end
end
end

function cleanupParallelClient(dashboard, future, commandQueue)
sendStop(commandQueue, "CLIENT_CLEANUP");
try
    if string(future.State) ~= "finished"
        wait(future, "finished", 2.0);
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
    dashboard.close();
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

function ensureCapacity(index, capacity, label)
if index > capacity
    error('scopeguide:stage07:LogCapacityExceeded', ...
        '%s log capacity exceeded.', label);
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
    error('scopeguide:stage07:CannotWriteJson', ...
        'Cannot open %s: %s', path, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end

function validateTimingOptions(options)
positive = [options.DurationSec, options.PlotRateHz, ...
    options.StatusRateHz, options.InputHeartbeatRateHz, ...
    options.InputHeartbeatTimeoutSec];
if any(~isfinite(positive)) || any(positive <= 0)
    error('scopeguide:stage07:InvalidParallelTiming', ...
        'Duration and parallel timing rates must be finite and positive.');
end
if options.InputHeartbeatTimeoutSec < ...
        3 / options.InputHeartbeatRateHz
    error('scopeguide:stage07:HeartbeatMarginTooSmall', ...
        ['InputHeartbeatTimeoutSec must cover at least three nominal ' ...
        'client heartbeat periods.']);
end
end
