classdef ProcessedWrenchPlotter < handle
    %PROCESSEDWRENCHPLOTTER Live display of a selected wrench branch.

    properties (SetAccess = private)
        Figure
    end

    properties (Access = private)
        ForceAxes
        MomentAxes
        ForceLines
        MomentLines
        LayoutTitle
        WindowSec
        DisplaySignal
        DisplayDescription
        OperationToggle
        OperationEnabled = false
        BaselineReady = false
    end

    methods
        function obj = ProcessedWrenchPlotter(windowSec, options)
            arguments
                windowSec (1, 1) double {mustBeFinite, mustBePositive} = 10
                options.DisplaySignal (1, 1) string ...
                    {mustBeMember(options.DisplaySignal, ...
                    ["external", "fast", "slow", "control"])} = "control"
            end
            obj.WindowSec = windowSec;
            obj.DisplaySignal = options.DisplaySignal;
            [~, ~, obj.DisplayDescription] = ...
                scopeguide.force.selectWrenchDisplaySignal( ...
                blankProcessedWrench(), obj.DisplaySignal);
            obj.Figure = figure( ...
                'Name', sprintf('ScopeGuide Wrench: %s', ...
                char(obj.DisplaySignal)), ...
                'NumberTitle', 'off', 'Color', 'w', ...
                'WindowKeyPressFcn', @(~, event)obj.onKeyPress(event));
            obj.OperationToggle = uicontrol(obj.Figure, ...
                'Style', 'togglebutton', 'Units', 'normalized', ...
                'Position', [0.77, 0.945, 0.21, 0.045], ...
                'String', '启动置零中，请勿施力', ...
                'FontWeight', 'bold', 'Enable', 'off', ...
                'BackgroundColor', [0.90, 0.78, 0.25], ...
                'TooltipString', ...
                'E键切换；Esc关闭。这里只使能力输入，不使能机器人。', ...
                'Callback', @(source, ~)obj.onToggleRequest(source));
            layout = tiledlayout(obj.Figure, 2, 1, ...
                'TileSpacing', 'compact', 'Padding', 'compact');
            obj.LayoutTitle = title(layout, ...
                '等待完整力觉处理数据...');

            obj.ForceAxes = nexttile(layout, 1);
            hold(obj.ForceAxes, 'on');
            obj.ForceLines = makeLines(obj.ForceAxes);
            ylabel(obj.ForceAxes, 'Force [N]');
            legend(obj.ForceAxes, {'Fx', 'Fy', 'Fz'}, ...
                'Location', 'northeastoutside');
            grid(obj.ForceAxes, 'on');

            obj.MomentAxes = nexttile(layout, 2);
            hold(obj.MomentAxes, 'on');
            obj.MomentLines = makeLines(obj.MomentAxes);
            xlabel(obj.MomentAxes, 'Time [s]');
            ylabel(obj.MomentAxes, 'Moment [N m]');
            legend(obj.MomentAxes, {'Tx', 'Ty', 'Tz'}, ...
                'Location', 'northeastoutside');
            grid(obj.MomentAxes, 'on');
        end

        function update(obj, timeSec, processed, robotState)
            if ~obj.isOpen()
                return;
            end
            [force, moment] = ...
                scopeguide.force.selectWrenchDisplaySignal( ...
                processed, obj.DisplaySignal);
            obj.updateBaselineState(processed);
            for axisIndex = 1:3
                addpoints(obj.ForceLines(axisIndex), timeSec, ...
                    force(axisIndex));
                addpoints(obj.MomentLines(axisIndex), timeSec, ...
                    moment(axisIndex));
            end
            xMinimum = max(0, timeSec - obj.WindowSec);
            xMaximum = max(obj.WindowSec, timeSec);
            xlim(obj.ForceAxes, [xMinimum, xMaximum]);
            xlim(obj.MomentAxes, [xMinimum, xMaximum]);

            title(obj.ForceAxes, sprintf( ...
                ['F = [%+.2f, %+.2f, %+.2f] N, ' ...
                '|F| = %.2f N'], force(1), force(2), force(3), ...
                norm(force)));
            title(obj.MomentAxes, sprintf( ...
                ['T = [%+.3f, %+.3f, %+.3f] N m, ' ...
                '|T| = %.3f N m'], moment(1), moment(2), moment(3), ...
                norm(moment)));
            if processed.Safety.StopRequested
                safetyText = "STOP:" + ...
                    strjoin(processed.Safety.StopReasons, '|');
            elseif processed.Safety.WarningActive
                safetyText = "WARNING:" + ...
                    strjoin(processed.Safety.WarningReasons, '|');
            else
                safetyText = "SAFETY_OK";
            end
            obj.LayoutTitle.String = sprintf( ...
                ['%s | quality=%s | %s | robot=%s, age=%.1f ms ' ...
                '| display=%s | %s'], ...
                char(obj.DisplayDescription), ...
                char(processed.Quality.StatusCode), char(safetyText), ...
                char(robotState.RobotMode), ...
                1000 * robotState.SampleAgeSec, char(obj.DisplaySignal), ...
                char(obj.baselineStatusText(processed)));
            drawnow limitrate;
        end

        function enabled = isOperationEnabled(obj)
            enabled = obj.OperationEnabled;
        end

        function open = isOpen(obj)
            open = ~isempty(obj.Figure) && isgraphics(obj.Figure);
        end
    end

    methods (Access = private)
        function updateBaselineState(obj, processed)
            diagnostics = processed.BaselineDiagnostics;
            if isstruct(diagnostics) && isfield(diagnostics, 'Ready')
                obj.BaselineReady = logical(diagnostics.Ready);
            else
                obj.BaselineReady = false;
            end
            if ~obj.BaselineReady && obj.OperationEnabled
                obj.OperationEnabled = false;
            end
            obj.refreshOperationToggle();
        end

        function onToggleRequest(obj, source)
            requested = logical(source.Value);
            if requested && ~obj.BaselineReady
                source.Value = 0;
                fprintf(['Operation enable denied: wait for startup ' ...
                    'software zero to complete.\n']);
                obj.OperationEnabled = false;
            else
                obj.OperationEnabled = requested;
            end
            obj.refreshOperationToggle();
        end

        function onKeyPress(obj, event)
            key = string(event.Key);
            if key == "e"
                if ~obj.BaselineReady && ~obj.OperationEnabled
                    fprintf(['Operation enable denied: wait for startup ' ...
                        'software zero to complete.\n']);
                    return;
                end
                obj.OperationEnabled = ~obj.OperationEnabled;
            elseif key == "escape"
                obj.OperationEnabled = false;
            else
                return;
            end
            obj.refreshOperationToggle();
        end

        function refreshOperationToggle(obj)
            if isempty(obj.OperationToggle) || ...
                    ~isgraphics(obj.OperationToggle)
                return;
            end
            if ~obj.BaselineReady
                obj.OperationToggle.Enable = 'off';
                obj.OperationToggle.Value = 0;
                obj.OperationToggle.String = '启动置零中，请勿施力';
                obj.OperationToggle.BackgroundColor = [0.90, 0.78, 0.25];
            elseif obj.OperationEnabled
                obj.OperationToggle.Enable = 'on';
                obj.OperationToggle.Value = 1;
                obj.OperationToggle.String = '操作使能：开（基线冻结）';
                obj.OperationToggle.BackgroundColor = [0.95, 0.45, 0.35];
            else
                obj.OperationToggle.Enable = 'on';
                obj.OperationToggle.Value = 0;
                obj.OperationToggle.String = '操作使能：关（空闲）';
                obj.OperationToggle.BackgroundColor = [0.45, 0.80, 0.50];
            end
        end

        function text = baselineStatusText(obj, processed)
            diagnostics = processed.BaselineDiagnostics;
            if ~isstruct(diagnostics) || ...
                    ~isfield(diagnostics, 'Enabled')
                text = "baseline=UNAVAILABLE";
            elseif ~diagnostics.Enabled
                text = "baseline=OFF";
            elseif ~diagnostics.Ready
                text = sprintf('startup_zero=%.0f%%(%s)', ...
                    100 * diagnostics.StartupProgress, ...
                    char(diagnostics.EligibilityReason));
            elseif obj.OperationEnabled
                text = sprintf('baseline=FROZEN, Fz0=%+.2f N', ...
                    diagnostics.BaselineWrench(3));
            elseif ~diagnostics.Eligible
                text = sprintf('baseline=HOLD(%s), Fz0=%+.2f N', ...
                    char(diagnostics.EligibilityReason), ...
                    diagnostics.BaselineWrench(3));
            else
                text = sprintf('baseline=Fz_TRACK, Fz0=%+.2f N', ...
                    diagnostics.BaselineWrench(3));
            end
        end
    end
end

function processed = blankProcessedWrench()
processed = struct();
processed.ExternalWrenchToolAtSensorOrigin = zeros(6, 1);
processed.FastWrenchToolAtSensorOrigin = zeros(6, 1);
processed.SlowWrenchToolAtSensorOrigin = zeros(6, 1);
processed.ControlForceTool = zeros(3, 1);
processed.ControlMomentForDiagnostics = zeros(3, 1);
end

function lines = makeLines(axesHandle)
colors = [0.85, 0.15, 0.15; 0.10, 0.55, 0.20; 0.10, 0.25, 0.85];
lines = gobjects(1, 3);
for index = 1:3
    lines(index) = animatedline(axesHandle, ...
        'Color', colors(index, :), 'LineWidth', 1.2);
end
end
