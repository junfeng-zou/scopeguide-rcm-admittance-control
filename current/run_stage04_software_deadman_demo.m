function [summary, outputDirectory] = ...
        run_stage04_software_deadman_demo(options)
%RUN_STAGE04_SOFTWARE_DEADMAN_DEMO Keyboard/mouse offline deadman demo.
% Hold SPACE or the on-screen pad to request enable.  Release stops in the
% next control cycle.  ESC releases; R requests FAULT reset.  This demo uses
% synthetic healthy inputs and a simulated perfect-tracking joint state. It
% never creates a robot adapter, sensor client or hardware command.

arguments
    options.Config = struct([])
    options.DurationSec (1, 1) double = 60
    options.DemoQdotRadSec = deg2rad(0.5) * [1; -1; 0; 0; 0; 0]
    options.WriteResults (1, 1) logical = true
end

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
if isempty(options.Config)
    cfg = defaultRcmAdmittanceConfig();
else
    cfg = options.Config;
end
validateRcmAdmittanceConfig(cfg);
if ~isfinite(options.DurationSec) || options.DurationSec <= 0
    error('scopeguide:stage04:InvalidDemoDuration', ...
        'DurationSec must be finite and positive.');
end
if ~isnumeric(options.DemoQdotRadSec) || ...
        numel(options.DemoQdotRadSec) ~= 6 || ...
        any(~isfinite(options.DemoQdotRadSec(:)))
    error('scopeguide:stage04:InvalidDemoVelocity', ...
        'DemoQdotRadSec must contain six finite values.');
end
qdotRequest = double(options.DemoQdotRadSec(:));

adapter = scopeguide.io.EnableHandleAdapter(cfg);
fsm = scopeguide.control.HandleEnableStateMachine(cfg);
integrator = scopeguide.control.JointCommandIntegrator(cfg);
safety = healthySyntheticSafety();

lastNowSec = 0;

% Remove any window left by an older interrupted version before creating a
% new demo.  The helper bypasses CloseRequestFcn and is safe to call even
% when the old callback workspace no longer exists.
close_stage04_software_deadman_demo();
figureHandle = figure( ...
    'Name', 'ScopeGuide Stage 4 Software Deadman (OFFLINE)', ...
    'Tag', 'ScopeGuideStage04SoftwareDeadmanDemo', ...
    'NumberTitle', 'off', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'Color', [0.94, 0.94, 0.94], ...
    'Position', [300, 220, 760, 430], ...
    'WindowKeyPressFcn', @stage04KeyPress, ...
    'WindowKeyReleaseFcn', @stage04KeyRelease, ...
    'WindowButtonUpFcn', @stage04MouseRelease, ...
    'CloseRequestFcn', @stage04CloseRequest);
guiState = defaultGuiInputState( ...
    cfg.handle.KeyboardReleaseConfirmSec, ...
    cfg.handle.KeyboardRepeatWatchdogSec);
keyboardFilter = guiState.KeyboardFilter;
setappdata(figureHandle, 'ScopeGuideStage04InputState', guiState);
cleanup = onCleanup(@() closeDemoFigure(figureHandle));

uicontrol(figureHandle, 'Style', 'text', ...
    'Units', 'normalized', ...
    'Position', [0.05, 0.88, 0.90, 0.08], ...
    'String', '阶段4：软件 deadman 离线演示（不会连接或移动机器人）', ...
    'FontSize', 14, 'FontWeight', 'bold', ...
    'BackgroundColor', get(figureHandle, 'Color'));
statusText = uicontrol(figureHandle, 'Style', 'text', ...
    'Units', 'normalized', ...
    'Position', [0.08, 0.67, 0.84, 0.13], ...
    'String', 'DISABLED', ...
    'FontSize', 20, 'FontWeight', 'bold', ...
    'ForegroundColor', [1, 1, 1], ...
    'BackgroundColor', stateColor("DISABLED"));

padAxes = axes(figureHandle, 'Units', 'normalized', ...
    'Position', [0.24, 0.30, 0.52, 0.25], ...
    'XLim', [0, 1], 'YLim', [0, 1], ...
    'XTick', [], 'YTick', [], 'Box', 'on');
hold(padAxes, 'on');
pad = patch(padAxes, [0, 1, 1, 0], [0, 0, 1, 1], ...
    [0.30, 0.55, 0.85], 'ButtonDownFcn', @stage04MousePress);
text(padAxes, 0.5, 0.5, ...
    '按住这里，或按住 SPACE', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'FontSize', 15, 'FontWeight', 'bold', ...
    'Color', [1, 1, 1], ...
    'HitTest', 'off', 'PickableParts', 'none');

detailText = uicontrol(figureHandle, 'Style', 'text', ...
    'Units', 'normalized', ...
    'Position', [0.05, 0.08, 0.90, 0.14], ...
    'String', 'SPACE/鼠标按住：使能请求；ESC：松开；R：故障复位', ...
    'FontSize', 11, ...
    'BackgroundColor', get(figureHandle, 'Color'));

nominalDtSec = cfg.runtime.NominalDtSec;
capacity = ceil(options.DurationSec * cfg.runtime.ControlRateHz) + 10;
timeSec = nan(capacity, 1);
loopDtSec = nan(capacity, 1);
requestedLog = false(capacity, 1);
inputValidLog = false(capacity, 1);
keyboardReleasePendingLog = false(capacity, 1);
stateLog = strings(capacity, 1);
motionPermittedLog = false(capacity, 1);
commandScaleLog = zeros(capacity, 1);
faultCodeLog = strings(capacity, 1);
targetLog = nan(capacity, 6);
qdotLog = zeros(capacity, 6);
sampleCount = 0;
qState = zeros(6, 1);
previousNowSec = NaN;
nextTickSec = 0;
clock = tic;

while isgraphics(figureHandle)
    drawnow limitrate;
    if ~isgraphics(figureHandle)
        break;
    end
    guiState = getGuiInputState(figureHandle);
    if guiState.CloseRequested
        break;
    end
    guiState.KeyboardFilter.tick(toc(guiState.KeyboardClock));
    nowSec = toc(clock);
    if nowSec >= options.DurationSec
        break;
    end
    if nowSec < nextTickSec
        pause(min(nextTickSec - nowSec, 0.005));
        continue;
    end
    if isfinite(previousNowSec)
        dtSec = nowSec - previousNowSec;
    else
        dtSec = nominalDtSec;
    end
    lastNowSec = nowSec;
    desired = guiState.KeyboardFilter.Held || guiState.MouseHeld;
    source = currentInputSource( ...
        guiState.KeyboardFilter.Held, guiState.MouseHeld);
    adapter.submit(desired, nowSec, Source=source, ...
        SourceValid=isgraphics(figureHandle), ...
        SupportsPhysicalMotion= ...
            cfg.handle.SoftwareInputSupportsPhysicalMotion);
    if guiState.ResetRequested
        fsm.requestReset();
        guiState.ResetRequested = false;
        setGuiInputState(figureHandle, guiState);
    end
    safety.ControlPeriodHealthy = ...
        dtSec > 0 && dtSec <= cfg.runtime.MaximumDtSec;
    handleInput = adapter.read(nowSec);
    enableStatus = fsm.step(handleInput, safety, nowSec);
    integrated = integrator.step(qState, qdotRequest, ...
        enableStatus, nowSec);
    if integrated.RequiresFault && enableStatus.State ~= "FAULT"
        safety.ControlPeriodHealthy = false;
    end
    if all(isfinite(integrated.QTargetRad))
        qState = integrated.QTargetRad;
    end

    sampleCount = sampleCount + 1;
    if sampleCount > capacity
        error('scopeguide:stage04:DemoLogCapacityExceeded', ...
            'Internal log capacity was exceeded.');
    end
    timeSec(sampleCount) = nowSec;
    loopDtSec(sampleCount) = dtSec;
    requestedLog(sampleCount) = desired;
    inputValidLog(sampleCount) = handleInput.IsValid;
    keyboardReleasePendingLog(sampleCount) = ...
        guiState.KeyboardFilter.ReleasePending;
    stateLog(sampleCount) = enableStatus.State;
    motionPermittedLog(sampleCount) = enableStatus.MotionPermitted;
    commandScaleLog(sampleCount) = enableStatus.CommandScale;
    faultCodeLog(sampleCount) = enableStatus.FaultCode;
    targetLog(sampleCount, :) = integrated.QTargetRad.';
    qdotLog(sampleCount, :) = integrated.QdotAppliedRadSec.';

    if mod(sampleCount, 5) == 0 || enableStatus.Transitioned
        updateDisplay(enableStatus, integrated, desired);
    end
    previousNowSec = nowSec;
    nextTickSec = nextTickSec + nominalDtSec;
    if nowSec - nextTickSec > cfg.runtime.MaximumDtSec
        nextTickSec = nowSec + nominalDtSec;
    end
end

finalTimeSec = max(lastNowSec + nominalDtSec, nominalDtSec);
adapter.forceDisable(finalTimeSec, "DEMO_EXIT");
safety.ControlPeriodHealthy = true;
finalStatus = fsm.step(adapter.read(finalTimeSec), safety, finalTimeSec);
finalIntegrated = integrator.step(qState, zeros(6, 1), ...
    finalStatus, finalTimeSec);

indices = 1:sampleCount;
signals = table(timeSec(indices), loopDtSec(indices), ...
    requestedLog(indices), inputValidLog(indices), ...
    keyboardReleasePendingLog(indices), stateLog(indices), ...
    motionPermittedLog(indices), commandScaleLog(indices), ...
    faultCodeLog(indices), targetLog(indices, :), qdotLog(indices, :), ...
    'VariableNames', {'TimeSec', 'LoopDtSec', 'EnableRequested', ...
    'InputValid', 'KeyboardReleasePending', 'FsmState', ...
    'MotionPermitted', 'CommandScale', ...
    'FaultCode', 'QTargetRad', 'QdotAppliedRadSec'});

summary = struct();
summary.Stage = 4;
summary.Status = "software_deadman_offline_demo_complete";
summary.SampleCount = sampleCount;
summary.DurationSec = lastNowSec;
summary.ObservedStates = unique(stateLog(indices)).';
summary.EnableRequestedSampleCount = sum(requestedLog(indices));
summary.MotionPermittedSampleCount = ...
    sum(motionPermittedLog(indices));
summary.FaultSampleCount = sum(stateLog(indices) == "FAULT");
summary.MaximumLoopDtSec = max(loopDtSec(indices));
summary.FinalState = finalStatus.State;
summary.FinalTargetReanchored = finalIntegrated.Reanchored;
summary.InputSource = "software_keyboard_or_mouse";
summary.InputSupportsPhysicalMotion = false;
summary.KeyboardReleaseConfirmSec = ...
    cfg.handle.KeyboardReleaseConfirmSec;
summary.KeyboardAutoRepeatReleaseCountSuppressed = double( ...
    keyboardFilter.SuppressedRepeatReleaseCount);
summary.KeyboardReleaseCountConfirmed = double( ...
    keyboardFilter.ConfirmedReleaseCount);
summary.HardwareConnectionsCreated = false;
summary.MotionCommandsSent = false;

if options.WriteResults
    timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
    outputDirectory = fullfile(projectRoot, 'results', ...
        "stage04_software_deadman_demo_" + timestamp);
    [created, message] = mkdir(outputDirectory);
    if ~created && ~isfolder(outputDirectory)
        error('scopeguide:stage04:CannotCreateResultDirectory', ...
            'Cannot create %s: %s', outputDirectory, message);
    end
    writeJson(fullfile(outputDirectory, 'summary.json'), summary);
    writeJson(fullfile(outputDirectory, 'config_snapshot.json'), cfg);
    writetable(signals, fullfile(outputDirectory, 'signals.csv'));
else
    outputDirectory = "";
end

fprintf('Stage 4 software deadman demo completed.\n');
fprintf('Hardware connections: 0; motion commands: 0.\n');
if strlength(outputDirectory) > 0
    fprintf('Results: %s\n', outputDirectory);
end
clear cleanup;
closeDemoFigure(figureHandle);

    function updateDisplay(enableStatus, integrated, desired)
        if ~isgraphics(figureHandle)
            return;
        end
        set(statusText, ...
            'String', sprintf('%s | scale %.2f | fault %s', ...
            enableStatus.State, enableStatus.CommandScale, ...
            enableStatus.FaultCode), ...
            'BackgroundColor', stateColor(enableStatus.State));
        if desired
            set(pad, 'FaceColor', [0.15, 0.70, 0.30]);
        else
            set(pad, 'FaceColor', [0.30, 0.55, 0.85]);
        end
        set(detailText, 'String', sprintf([ ...
            'SPACE/鼠标按住：使能请求；ESC：松开；R：故障复位\n' ...
            '模拟目标 q1 = %+.3f deg；硬件命令许可 = false'], ...
            rad2deg(integrated.QTargetRad(1))));
    end

end

function closeDemoFigure(figureHandle)
if isgraphics(figureHandle)
    % Bypass a possibly stale CloseRequestFcn.  This is essential when the
    % main function is interrupted by Ctrl+C and its workspace is unwound.
    set(figureHandle, 'CloseRequestFcn', '', ...
        'WindowKeyPressFcn', '', 'WindowKeyReleaseFcn', '', ...
        'WindowButtonUpFcn', '');
    delete(figureHandle);
end
end

function state = defaultGuiInputState(keyboardReleaseConfirmSec, ...
        keyboardRepeatWatchdogSec)
state = struct();
state.KeyboardFilter = scopeguide.io.KeyboardHoldFilter( ...
    keyboardReleaseConfirmSec, keyboardRepeatWatchdogSec);
state.KeyboardClock = tic;
state.MouseHeld = false;
state.ResetRequested = false;
state.CloseRequested = false;
end

function state = getGuiInputState(figureHandle)
if ~isgraphics(figureHandle) || ...
        ~isappdata(figureHandle, 'ScopeGuideStage04InputState')
    state = struct();
    state.CloseRequested = true;
    return;
end
state = getappdata(figureHandle, 'ScopeGuideStage04InputState');
end

function setGuiInputState(figureHandle, state)
if isgraphics(figureHandle)
    setappdata(figureHandle, 'ScopeGuideStage04InputState', state);
end
end

function stage04KeyPress(source, event)
figureHandle = callbackFigure(source);
state = getGuiInputState(figureHandle);
nowSec = toc(state.KeyboardClock);
switch string(event.Key)
    case "space"
        state.KeyboardFilter.press(nowSec);
    case "escape"
        state.KeyboardFilter.forceRelease("ESCAPE_RELEASE");
        state.MouseHeld = false;
    case "r"
        state.ResetRequested = true;
end
setGuiInputState(figureHandle, state);
end

function stage04KeyRelease(source, event)
figureHandle = callbackFigure(source);
state = getGuiInputState(figureHandle);
if string(event.Key) == "space"
    state.KeyboardFilter.release(toc(state.KeyboardClock));
end
setGuiInputState(figureHandle, state);
end

function stage04MousePress(source, ~)
figureHandle = callbackFigure(source);
state = getGuiInputState(figureHandle);
state.MouseHeld = true;
setGuiInputState(figureHandle, state);
end

function stage04MouseRelease(source, ~)
figureHandle = callbackFigure(source);
state = getGuiInputState(figureHandle);
state.MouseHeld = false;
setGuiInputState(figureHandle, state);
end

function stage04CloseRequest(source, ~)
figureHandle = callbackFigure(source);
if ~isgraphics(figureHandle)
    return;
end
state = getGuiInputState(figureHandle);
state.KeyboardFilter.forceRelease("WINDOW_CLOSE_RELEASE");
state.MouseHeld = false;
state.CloseRequested = true;
setGuiInputState(figureHandle, state);
% Delete immediately. The main loop detects the invalid handle after
% drawnow. This callback is a local function, not a closure over the main
% workspace, so it also works if Ctrl+C already interrupted the demo.
closeDemoFigure(figureHandle);
end

function figureHandle = callbackFigure(source)
if isgraphics(source, 'figure')
    figureHandle = source;
else
    figureHandle = ancestor(source, 'figure');
end
end

function safety = healthySyntheticSafety()
safety = scopeguide.types.enableSafetyStatus();
safety.NeutralWrench = true;
safety.ForceFresh = true;
safety.RobotFresh = true;
safety.QpHealthy = true;
safety.ControlPeriodHealthy = true;
safety.MotionGatesSatisfied = true;
safety.StatusCode = "SYNTHETIC_OFFLINE_HEALTHY";
end

function source = currentInputSource(keyHeld, mouseHeld)
if keyHeld && mouseHeld
    source = "keyboard_and_mouse_hold";
elseif keyHeld
    source = "keyboard_space_hold";
elseif mouseHeld
    source = "mouse_pad_hold";
else
    source = "software_released";
end
end

function color = stateColor(state)
switch string(state)
    case "DISABLED"
        color = [0.35, 0.35, 0.35];
    case "PREARM"
        color = [0.90, 0.65, 0.10];
    case "ENABLED"
        color = [0.15, 0.65, 0.25];
    case "STOPPING"
        color = [0.90, 0.40, 0.10];
    otherwise
        color = [0.75, 0.10, 0.10];
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
    error('scopeguide:stage04:CannotWriteJson', ...
        'Cannot open %s: %s', filePath, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end
