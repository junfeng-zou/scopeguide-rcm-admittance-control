classdef Stage07DryRunDashboard < handle
    %STAGE07DRYUNDASHBOARD Stage 7 prediction display and dry-run input.
    % SPACE latches the software request on; ESC latches it off. Keyboard
    % release events are intentionally unused. The mouse remains hold-only.

    properties (SetAccess = private)
        Figure
        CloseRequested (1, 1) logical = false
        PlotEnabled (1, 1) logical = true
    end

    properties (Access = private)
        Config
        WindowSec
        FaultInjectionProfile (1, 1) string = "none"
        PhysicalMotionMode (1, 1) logical = false
        DofMode (1, 1) string = "all_prediction"
        KeyboardLatched (1, 1) logical = false
        SpaceOnCommandCount (1, 1) uint64 = uint64(0)
        EscapeOffCommandCount (1, 1) uint64 = uint64(0)
        MouseHeld (1, 1) logical = false
        ResetRequested (1, 1) logical = false
        BaselineReady (1, 1) logical = false
        ForceAxes
        GeneralizedAxes
        JointVelocityAxes
        RcmAxes
        ForceLines
        DesiredLines
        AchievedLines
        JointVelocityCommandLines
        JointVelocityActualLines
        RcmLines
        LayoutTitle
        StatusText
        Pad
        LastTimeSec (1, 1) double = 0
        PreserveFigureOnDelete (1, 1) logical = false
        ReviewFrozen (1, 1) logical = false
    end

    methods
        function obj = Stage07DryRunDashboard(cfg, windowSec, options)
            arguments
                cfg (1, 1) struct
                windowSec (1, 1) double = 15
                options.EnablePlot (1, 1) logical = true
                options.FaultInjectionProfile (1, 1) string ...
                    {mustBeMember(options.FaultInjectionProfile, ...
                    ["none", "acceptance"])} = "none"
                options.PhysicalMotionMode (1, 1) logical = false
                options.DofMode (1, 1) string = "all_prediction"
            end
            obj.Config = cfg;
            obj.WindowSec = windowSec;
            obj.PlotEnabled = options.EnablePlot;
            obj.FaultInjectionProfile = options.FaultInjectionProfile;
            obj.PhysicalMotionMode = options.PhysicalMotionMode;
            obj.DofMode = options.DofMode;
            obj.createFigure();
        end

        function input = readInput(obj)
            requested = obj.BaselineReady && ...
                (obj.KeyboardLatched || obj.MouseHeld);
            if obj.KeyboardLatched && obj.MouseHeld
                source = "keyboard_latched_and_mouse_hold";
            elseif obj.KeyboardLatched
                source = "keyboard_space_latched";
            elseif obj.MouseHeld
                source = "mouse_pad_hold";
            else
                source = "software_released";
            end
            input = struct();
            input.Requested = requested;
            input.Source = source;
            input.ResetRequested = obj.ResetRequested;
            input.CloseRequested = obj.CloseRequested || ~obj.isOpen();
            obj.ResetRequested = false;
        end

        function diagnostics = keyboardDiagnostics(obj)
            diagnostics = struct();
            diagnostics.Mode = "space_on_escape_off";
            diagnostics.SpaceSetsOn = true;
            diagnostics.EscapeSetsOff = true;
            diagnostics.KeyReleaseIgnored = true;
            diagnostics.SpaceOnCommandCount = double( ...
                obj.SpaceOnCommandCount);
            diagnostics.EscapeOffCommandCount = double( ...
                obj.EscapeOffCommandCount);
            diagnostics.FinalLatched = obj.KeyboardLatched;
            diagnostics.SoftwareDryRunOnly = ~obj.PhysicalMotionMode;
            diagnostics.KeyboardInputIsNotPhysicalDeadman = ...
                obj.PhysicalMotionMode;
        end

        function update(obj, timeSec, processed, decision)
            snapshot = ...
                scopeguide.runtime.buildStage07DisplaySnapshot( ...
                timeSec, processed, decision);
            obj.updateFromSnapshot(snapshot);
        end

        function updateFromSnapshot(obj, snapshot)
            %UPDATEFROMSNAPSHOT Render a compact worker-produced snapshot.
            % Graphics remain on the MATLAB client.  A background control
            % worker can therefore send this value-only struct without
            % sharing any graphics or hardware handle objects.
            if ~obj.isOpen()
                return;
            end
            validateDisplaySnapshot(snapshot, obj.PhysicalMotionMode);
            obj.updateBaselineReady(snapshot.BaselineReady);
            timeSec = double(snapshot.TimeSec);
            obj.LastTimeSec = max(obj.LastTimeSec, timeSec);
            force = double(snapshot.ControlForceTool(:));
            desired = double(snapshot.DesiredGeneralizedVelocity(:));
            achieved = double(snapshot.AchievedGeneralizedVelocity(:));
            currentErrorMm = 1e3 * double(snapshot.CurrentRcmErrorM);
            predictedErrorMm = 1e3 * double(snapshot.PredictedRcmErrorM);
            jointVelocityCommandDegSec = zeros(6, 1);
            jointVelocityActualDegSec = zeros(6, 1);
            jointVelocityQpCandidateDegSec = zeros(6, 1);
            if obj.PhysicalMotionMode
                jointVelocityCommandDegSec = rad2deg(double( ...
                    snapshot.JointVelocityCommandRadSec(:)));
                jointVelocityActualDegSec = rad2deg(double( ...
                    snapshot.JointVelocityActualRadSec(:)));
                jointVelocityQpCandidateDegSec = rad2deg(double( ...
                    snapshot.JointVelocityQpCandidateRadSec(:)));
            end
            if obj.PlotEnabled
                for axisIndex = 1:3
                    addpoints(obj.ForceLines(axisIndex), timeSec, ...
                        force(axisIndex));
                end
                normalizer = [obj.Config.control.PivotMaxRadSec; ...
                    obj.Config.control.PivotMaxRadSec; ...
                    obj.Config.control.InsertionMaxMSec];
                desiredNormalized = desired ./ normalizer;
                achievedNormalized = achieved ./ normalizer;
                for axisIndex = 1:3
                    addpoints(obj.DesiredLines(axisIndex), timeSec, ...
                        desiredNormalized(axisIndex));
                    addpoints(obj.AchievedLines(axisIndex), timeSec, ...
                        achievedNormalized(axisIndex));
                end
                if obj.PhysicalMotionMode
                    for jointIndex = 1:6
                        addpoints(obj.JointVelocityCommandLines(jointIndex), ...
                            timeSec, ...
                            jointVelocityCommandDegSec(jointIndex));
                        addpoints(obj.JointVelocityActualLines(jointIndex), ...
                            timeSec, jointVelocityActualDegSec(jointIndex));
                    end
                end
                addpoints(obj.RcmLines(1), timeSec, currentErrorMm);
                addpoints(obj.RcmLines(2), timeSec, predictedErrorMm);
                addpoints(obj.RcmLines(3), timeSec, ...
                    1e3 * obj.Config.rcm.HardRadiusM);

                xMinimum = max(0, timeSec - obj.WindowSec);
                xMaximum = max(obj.WindowSec, timeSec);
                xlim(obj.ForceAxes, [xMinimum, xMaximum]);
                xlim(obj.GeneralizedAxes, [xMinimum, xMaximum]);
                if obj.PhysicalMotionMode
                    xlim(obj.JointVelocityAxes, [xMinimum, xMaximum]);
                end
                xlim(obj.RcmAxes, [xMinimum, xMaximum]);
            end
            state = string(snapshot.EnableState);
            guidance = scopeguide.runtime.stage07FaultGuidance( ...
                timeSec, obj.FaultInjectionProfile);
            if obj.PhysicalMotionMode
                sent = false;
                if isfield(snapshot, 'MotionCommandSent')
                    sent = logical(snapshot.MotionCommandSent);
                end
                titleText = sprintf([ ...
                    'Stage 8 FIXTURE | DOF=%s | %s | scale=%.2f | ' ...
                    'QP=%s | RCM=%.3f mm | ServoJ=%s'], ...
                    char(obj.DofMode), char(state), snapshot.CommandScale, ...
                    char(snapshot.QpStatusCode), currentErrorMm, ...
                    char(string(sent)));
            else
                titleText = sprintf([ ...
                    'Stage 7 READ-ONLY | %s | scale=%.2f | QP=%s | ' ...
                    'RCM=%.3f mm | robot command=false'], ...
                    char(state), snapshot.CommandScale, ...
                    char(snapshot.QpStatusCode), currentErrorMm);
            end
            if guidance.Enabled
                titleText = sprintf('%s | %s', titleText, ...
                    char(guidance.TitleText));
            end
            obj.LayoutTitle.String = titleText;
            controlStatus = sprintf([ ...
                'Space：软件使能 ON；Esc：立即 OFF；鼠标区域按住使能；R：故障复位\n' ...
                'F=[%+.2f,%+.2f,%+.2f] N | ' ...
                'u=[%+.3f,%+.3f,%+.3f] deg/s,deg/s,mm/s | fault=%s | inject=%s'], ...
                force(1), force(2), force(3), ...
                rad2deg(desired(1)), rad2deg(desired(2)), ...
                1e3 * desired(3), ...
                char(snapshot.FaultCode), ...
                char(snapshot.InjectionStatusCode));
            if obj.PhysicalMotionMode
                safetyCode = string(snapshot.WrenchSafetyStopCode);
                if strlength(safetyCode) == 0
                    safetyCode = "none";
                end
                controlStatus = sprintf([ ...
                    '%s\n|max qdot qp/cmd/actual|=%.3f/%.3f/%.3f deg/s | ' ...
                    'forceQuality=%s | wrenchStop=%s'], ...
                    controlStatus, ...
                    max(abs(jointVelocityQpCandidateDegSec)), ...
                    max(abs(jointVelocityCommandDegSec)), ...
                    max(abs(jointVelocityActualDegSec)), ...
                    char(snapshot.ForceStatusCode), char(safetyCode));
            end
            if guidance.Enabled
                obj.StatusText.String = sprintf('%s\n%s', ...
                    char(guidance.MessageZh), controlStatus);
                switch guidance.Phase
                    case "PREPARE"
                        obj.StatusText.BackgroundColor = [1.00, 0.93, 0.55];
                    case "ACTIVE"
                        obj.StatusText.BackgroundColor = [1.00, 0.72, 0.72];
                    case "RECOVER"
                        obj.StatusText.BackgroundColor = [0.80, 0.91, 1.00];
                    otherwise
                        obj.StatusText.BackgroundColor = [1, 1, 1];
                end
            else
                obj.StatusText.String = controlStatus;
                obj.StatusText.BackgroundColor = [1, 1, 1];
            end
            switch state
                case "PREARM"
                    obj.Pad.FaceColor = [0.95, 0.70, 0.15];
                case "ENABLED"
                    obj.Pad.FaceColor = [0.15, 0.70, 0.30];
                case "FAULT"
                    obj.Pad.FaceColor = [0.85, 0.20, 0.20];
                case "STOPPING"
                    obj.Pad.FaceColor = [0.90, 0.45, 0.15];
                otherwise
                    obj.Pad.FaceColor = [0.25, 0.50, 0.85];
            end
            if obj.PlotEnabled
                drawnow limitrate;
            end
        end

        function open = isOpen(obj)
            open = ~isempty(obj.Figure) && isgraphics(obj.Figure);
        end

        function retained = freezeForReview(obj, terminationReason)
            %FREEZEFORREVIEW Detach all control callbacks but retain plots.
            % The retained figure no longer references this dashboard
            % object, so it remains a normal MATLAB figure after the run
            % returns and can be closed manually by the operator.
            retained = false;
            if ~obj.isOpen()
                return;
            end
            if obj.ReviewFrozen
                retained = true;
                return;
            end
            obj.KeyboardLatched = false;
            obj.MouseHeld = false;
            obj.ResetRequested = false;
            obj.ReviewFrozen = true;
            obj.PreserveFigureOnDelete = true;
            set(obj.Figure, ...
                'WindowKeyPressFcn', '', ...
                'WindowKeyReleaseFcn', '', ...
                'WindowButtonUpFcn', '', ...
                'CloseRequestFcn', @(source, ~)delete(source));
            if ~isempty(obj.Pad) && isgraphics(obj.Pad)
                set(obj.Pad, 'ButtonDownFcn', '', ...
                    'FaceColor', [0.45, 0.45, 0.45]);
            end
            obj.showFullHistory();
            reason = string(terminationReason);
            if strlength(reason) == 0
                reason = "COMPLETE";
            end
            reviewText = sprintf([ ...
                '\n运行已结束（%s）；控制输入已停用。' ...
                '曲线窗口保留供分析，请手动关闭。'], char(reason));
            if ~isempty(obj.StatusText) && isgraphics(obj.StatusText)
                obj.StatusText.String = sprintf('%s%s', ...
                    char(string(obj.StatusText.String)), reviewText);
                obj.StatusText.BackgroundColor = [0.92, 0.92, 0.92];
            end
            if ~isempty(obj.LayoutTitle) && isgraphics(obj.LayoutTitle)
                obj.LayoutTitle.String = sprintf( ...
                    'Stage 8 已结束：%s | 完整曲线保留供分析', ...
                    char(reason));
            end
            drawnow;
            retained = true;
        end

        function close(obj)
            obj.CloseRequested = true;
            obj.KeyboardLatched = false;
            obj.MouseHeld = false;
            if obj.isOpen()
                set(obj.Figure, 'CloseRequestFcn', '', ...
                    'WindowKeyPressFcn', '', ...
                    'WindowButtonUpFcn', '');
                delete(obj.Figure);
            end
        end

        function delete(obj)
            if obj.PreserveFigureOnDelete && obj.isOpen()
                % The callbacks were detached by freezeForReview.  Release
                % only this object's reference; do not delete the figure.
                obj.Figure = [];
                return;
            end
            obj.close();
        end
    end

    methods (Access = private)
        function createFigure(obj)
            if obj.PhysicalMotionMode
                figureName = 'ScopeGuide Stage 8 Fixture Commissioning';
                figureTag = 'ScopeGuideStage08FixtureCommissioning';
                initialTitle = ['Stage 8 初始化：物理运动入口已打开，' ...
                    '请先确认急停和夹具'];
                padText = '鼠标按住运动使能';
            else
                figureName = 'ScopeGuide Stage 7 Read-Only Dry-Run';
                figureTag = 'ScopeGuideStage07ReadOnlyDryRun';
                initialTitle = 'Stage 7 初始化：不会发送机器人命令';
                padText = '鼠标按住预测使能';
            end
            obj.Figure = figure( ...
                'Name', figureName, ...
                'Tag', figureTag, ...
                'NumberTitle', 'off', 'Color', 'w', ...
                'WindowKeyPressFcn', @(~, event)obj.onKeyPress(event), ...
                'WindowButtonUpFcn', @(~, ~)obj.onMouseRelease(), ...
                'CloseRequestFcn', @(~, ~)obj.close());
            if obj.PhysicalMotionMode
                obj.Figure.Position(4) = max(obj.Figure.Position(4), 900);
            end
            if ~obj.PlotEnabled
                obj.createInputOnlyControls();
                return;
            end
            rowCount = 3;
            if obj.PhysicalMotionMode
                rowCount = 4;
            end
            layout = tiledlayout(obj.Figure, rowCount, 1, ...
                'TileSpacing', 'compact', 'Padding', 'compact');
            obj.LayoutTitle = title(layout, initialTitle);

            obj.ForceAxes = nexttile(layout, 1);
            hold(obj.ForceAxes, 'on');
            obj.ForceLines = coloredLines(obj.ForceAxes, '-', 1.2);
            ylabel(obj.ForceAxes, 'Control F [N]');
            legend(obj.ForceAxes, {'Fx', 'Fy', 'Fz'}, ...
                'Location', 'northeastoutside');
            grid(obj.ForceAxes, 'on');

            obj.GeneralizedAxes = nexttile(layout, 2);
            hold(obj.GeneralizedAxes, 'on');
            obj.DesiredLines = coloredLines( ...
                obj.GeneralizedAxes, '-', 1.2);
            obj.AchievedLines = coloredLines( ...
                obj.GeneralizedAxes, '--', 1.0);
            ylabel(obj.GeneralizedAxes, 'u / limit');
            ylim(obj.GeneralizedAxes, [-1.1, 1.1]);
            legend(obj.GeneralizedAxes, ...
                {'des p1', 'des p2', 'des ins', ...
                'qp p1', 'qp p2', 'qp ins'}, ...
                'Location', 'northeastoutside');
            grid(obj.GeneralizedAxes, 'on');

            if obj.PhysicalMotionMode
                obj.JointVelocityAxes = nexttile(layout, 3);
                hold(obj.JointVelocityAxes, 'on');
                obj.JointVelocityCommandLines = jointColoredLines( ...
                    obj.JointVelocityAxes, '-', 1.1);
                obj.JointVelocityActualLines = jointColoredLines( ...
                    obj.JointVelocityAxes, '--', 0.9);
                ylabel(obj.JointVelocityAxes, 'Joint speed [deg/s]');
                jointNames = "J" + string(1:6);
                legend(obj.JointVelocityAxes, ...
                    [jointNames + " cmd", jointNames + " actual"], ...
                    'Location', 'northeastoutside', 'NumColumns', 2);
                grid(obj.JointVelocityAxes, 'on');
                rcmTile = 4;
            else
                rcmTile = 3;
            end

            obj.RcmAxes = nexttile(layout, rcmTile);
            hold(obj.RcmAxes, 'on');
            obj.RcmLines = gobjects(1, 3);
            obj.RcmLines(1) = animatedline(obj.RcmAxes, ...
                'Color', [0.1, 0.3, 0.85], 'LineWidth', 1.2);
            obj.RcmLines(2) = animatedline(obj.RcmAxes, ...
                'Color', [0.1, 0.65, 0.25], 'LineWidth', 1.2);
            obj.RcmLines(3) = animatedline(obj.RcmAxes, ...
                'Color', [0.85, 0.15, 0.15], ...
                'LineStyle', '--', 'LineWidth', 1.0);
            xlabel(obj.RcmAxes, 'Time [s]');
            ylabel(obj.RcmAxes, 'RCM error [mm]');
            legend(obj.RcmAxes, {'current', 'predicted', 'hard'}, ...
                'Location', 'northeastoutside');
            grid(obj.RcmAxes, 'on');

            obj.StatusText = uicontrol(obj.Figure, 'Style', 'text', ...
                'Units', 'normalized', 'Position', [0.02, 0.003, 0.70, 0.085], ...
                'String', '启动软件置零中，请勿施力', ...
                'BackgroundColor', [1, 1, 1], ...
                'HorizontalAlignment', 'left', 'FontSize', 9);
            padAxes = axes(obj.Figure, 'Units', 'normalized', ...
                'Position', [0.75, 0.012, 0.20, 0.065], ...
                'XLim', [0, 1], 'YLim', [0, 1], ...
                'XTick', [], 'YTick', [], 'Box', 'on');
            obj.Pad = patch(padAxes, [0, 1, 1, 0], [0, 0, 1, 1], ...
                [0.25, 0.50, 0.85], ...
                'ButtonDownFcn', @(~, ~)obj.onMousePress());
            text(padAxes, 0.5, 0.5, padText, ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', ...
                'Color', 'w', 'FontWeight', 'bold', ...
                'HitTest', 'off', 'PickableParts', 'none');
        end

        function createInputOnlyControls(obj)
            obj.Figure.Name = ...
                'ScopeGuide Stage 7 Timing Test - Curves Disabled';
            obj.Figure.Position(3:4) = [900, 190];
            obj.LayoutTitle = uicontrol(obj.Figure, 'Style', 'text', ...
                'Units', 'normalized', 'Position', [0.02, 0.70, 0.96, 0.22], ...
                'String', ['Stage 7 纯控制链时序测试：' ...
                '曲线绘制已完全关闭'], ...
                'BackgroundColor', [1, 1, 1], ...
                'FontSize', 12, 'FontWeight', 'bold');
            obj.StatusText = uicontrol(obj.Figure, 'Style', 'text', ...
                'Units', 'normalized', 'Position', [0.03, 0.18, 0.67, 0.42], ...
                'String', '启动软件置零中，请勿施力', ...
                'BackgroundColor', [1, 1, 1], ...
                'HorizontalAlignment', 'left', 'FontSize', 10);
            padAxes = axes(obj.Figure, 'Units', 'normalized', ...
                'Position', [0.74, 0.18, 0.22, 0.38], ...
                'XLim', [0, 1], 'YLim', [0, 1], ...
                'XTick', [], 'YTick', [], 'Box', 'on');
            obj.Pad = patch(padAxes, [0, 1, 1, 0], [0, 0, 1, 1], ...
                [0.25, 0.50, 0.85], ...
                'ButtonDownFcn', @(~, ~)obj.onMousePress());
            text(padAxes, 0.5, 0.5, '鼠标按住预测使能', ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', ...
                'Color', 'w', 'FontWeight', 'bold', ...
                'HitTest', 'off', 'PickableParts', 'none');
        end

        function updateBaselineReady(obj, ready)
            ready = logical(ready);
            obj.BaselineReady = ready;
            if ~ready
                obj.KeyboardLatched = false;
                obj.MouseHeld = false;
            end
        end

        function showFullHistory(obj)
            if ~obj.PlotEnabled
                return;
            end
            xMaximum = max(obj.LastTimeSec, eps);
            axesHandles = [obj.ForceAxes, obj.GeneralizedAxes, ...
                obj.JointVelocityAxes, obj.RcmAxes];
            for index = 1:numel(axesHandles)
                if isgraphics(axesHandles(index), 'axes')
                    xlim(axesHandles(index), [0, xMaximum]);
                end
            end
        end

        function onKeyPress(obj, event)
            switch string(event.Key)
                case "space"
                    if obj.BaselineReady
                        obj.KeyboardLatched = true;
                        obj.SpaceOnCommandCount = ...
                            obj.SpaceOnCommandCount + uint64(1);
                    end
                case "escape"
                    obj.KeyboardLatched = false;
                    obj.EscapeOffCommandCount = ...
                        obj.EscapeOffCommandCount + uint64(1);
                    obj.MouseHeld = false;
                case "r"
                    obj.ResetRequested = true;
            end
        end

        function onMousePress(obj)
            if obj.BaselineReady
                obj.MouseHeld = true;
            end
        end

        function onMouseRelease(obj)
            obj.MouseHeld = false;
        end
    end
end

function validateDisplaySnapshot(snapshot, physicalMotionMode)
required = {'TimeSec', 'BaselineReady', 'ControlForceTool', ...
    'DesiredGeneralizedVelocity', 'AchievedGeneralizedVelocity', ...
    'CurrentRcmErrorM', 'PredictedRcmErrorM', 'EnableState', ...
    'CommandScale', 'QpStatusCode', 'FaultCode', ...
    'InjectionStatusCode'};
for index = 1:numel(required)
    if ~isstruct(snapshot) || ~isfield(snapshot, required{index})
        error('scopeguide:stage07:InvalidDisplaySnapshot', ...
            'Stage 7 display snapshot is missing %s.', required{index});
    end
end
if physicalMotionMode
    requiredPhysical = {'JointVelocityCommandRadSec', ...
        'JointVelocityActualRadSec', 'JointVelocityQpCandidateRadSec', ...
        'ForceStatusCode', 'WrenchSafetyStopCode'};
    for index = 1:numel(requiredPhysical)
        if ~isfield(snapshot, requiredPhysical{index})
            error('scopeguide:stage08:InvalidDisplaySnapshot', ...
                'Stage 8 display snapshot is missing %s.', ...
                requiredPhysical{index});
        end
    end
    jointVectors = {snapshot.JointVelocityCommandRadSec, ...
        snapshot.JointVelocityActualRadSec, ...
        snapshot.JointVelocityQpCandidateRadSec};
    if any(cellfun(@(value) ~isnumeric(value) || numel(value) ~= 6 || ...
            any(~isfinite(value(:))), jointVectors))
        error('scopeguide:stage08:InvalidDisplaySnapshot', ...
            'Stage 8 joint-speed display vectors must be finite 6-by-1.');
    end
end
vectors = {snapshot.ControlForceTool, ...
    snapshot.DesiredGeneralizedVelocity, ...
    snapshot.AchievedGeneralizedVelocity};
if any(cellfun(@(value) ~isnumeric(value) || numel(value) ~= 3 || ...
        any(~isfinite(value(:))), vectors)) || ...
        ~islogical(snapshot.BaselineReady) || ...
        ~isscalar(snapshot.BaselineReady) || ...
        ~isscalar(snapshot.TimeSec) || ~isfinite(snapshot.TimeSec)
    error('scopeguide:stage07:InvalidDisplaySnapshot', ...
        'Stage 7 display snapshot contains invalid numeric fields.');
end
end

function lines = jointColoredLines(axesHandle, lineStyle, lineWidth)
colors = linesColorOrder();
lines = gobjects(1, 6);
for index = 1:6
    lines(index) = animatedline(axesHandle, ...
        'Color', colors(index, :), 'LineStyle', lineStyle, ...
        'LineWidth', lineWidth);
end
end

function colors = linesColorOrder()
colors = [0.0000, 0.4470, 0.7410; ...
    0.8500, 0.3250, 0.0980; ...
    0.9290, 0.6940, 0.1250; ...
    0.4940, 0.1840, 0.5560; ...
    0.4660, 0.6740, 0.1880; ...
    0.3010, 0.7450, 0.9330];
end

function lines = coloredLines(axesHandle, lineStyle, lineWidth)
colors = [0.85, 0.15, 0.15; 0.10, 0.55, 0.20; 0.10, 0.25, 0.85];
lines = gobjects(1, 3);
for index = 1:3
    lines(index) = animatedline(axesHandle, ...
        'Color', colors(index, :), 'LineStyle', lineStyle, ...
        'LineWidth', lineWidth);
end
end
