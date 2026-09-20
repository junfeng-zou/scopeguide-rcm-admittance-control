classdef CartesianDragDashboard < handle
    %CARTESIANDRAGDASHBOARD No-RCM physical drag display and latch input.
    % Space latches ON, Esc latches OFF, and mouse input is hold-only.

    properties (SetAccess = private)
        Figure
        CloseRequested (1, 1) logical = false
    end

    properties (Access = private)
        Config
        WindowSec
        DofMode (1, 1) string
        KeyboardLatched (1, 1) logical = false
        MouseHeld (1, 1) logical = false
        ResetRequested (1, 1) logical = false
        BaselineReady (1, 1) logical = false
        SpaceCount (1, 1) uint64 = uint64(0)
        EscapeCount (1, 1) uint64 = uint64(0)
        AxesHandles
        ForceLines
        MomentLines
        DesiredLines
        AchievedLines
        JointCommandLines
        JointActualLines
        RelativeLines
        PredictedRelativeLines
        TitleHandle
        StatusText
        Pad
        LastTimeSec (1, 1) double = 0
        PreserveFigureOnDelete (1, 1) logical = false
        ReviewFrozen (1, 1) logical = false
    end

    methods
        function obj = CartesianDragDashboard(cfg, windowSec, dofMode)
            arguments
                cfg (1, 1) struct
                windowSec (1, 1) double = 20
                dofMode (1, 1) string = "translation_z"
            end
            validateCartesianAdmittanceDragConfig(cfg);
            obj.Config = cfg;
            obj.WindowSec = windowSec;
            obj.DofMode = dofMode;
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
            input = struct('Requested', requested, 'Source', source, ...
                'ResetRequested', obj.ResetRequested, ...
                'CloseRequested', obj.CloseRequested || ~obj.isOpen());
            obj.ResetRequested = false;
        end

        function diagnostics = keyboardDiagnostics(obj)
            diagnostics = struct('Mode', "space_on_escape_off", ...
                'SpaceSetsOn', true, 'EscapeSetsOff', true, ...
                'KeyReleaseIgnored', true, ...
                'SpaceOnCommandCount', double(obj.SpaceCount), ...
                'EscapeOffCommandCount', double(obj.EscapeCount), ...
                'FinalLatched', obj.KeyboardLatched, ...
                'KeyboardInputIsNotPhysicalDeadman', true);
        end

        function updateFromSnapshot(obj, snapshot)
            if ~obj.isOpen()
                return;
            end
            validateSnapshot(snapshot);
            obj.BaselineReady = logical(snapshot.BaselineReady);
            if ~obj.BaselineReady
                obj.KeyboardLatched = false;
                obj.MouseHeld = false;
            end
            timeSec = double(snapshot.TimeSec);
            obj.LastTimeSec = max(obj.LastTimeSec, timeSec);
            wrench = double(snapshot.ControlWrenchDrag(:));
            desired = double(snapshot.DesiredTwistBase(:));
            achieved = double(snapshot.AchievedTwistBase(:));
            relative = double(snapshot.RelativeCoordinate(:));
            predicted = double(snapshot.PredictedRelativeCoordinate(:));
            qdotCommand = rad2deg(double( ...
                snapshot.JointVelocityCommandRadSec(:)));
            qdotActual = rad2deg(double( ...
                snapshot.JointVelocityActualRadSec(:)));
            for index = 1:3
                addpoints(obj.ForceLines(index), timeSec, wrench(index));
                addpoints(obj.MomentLines(index), timeSec, wrench(index + 3));
            end
            velocityLimit = [ ...
                obj.Config.cartesianDrag.Translation.MaximumSpeedMSec * ...
                    ones(3, 1); ...
                obj.Config.cartesianDrag.Rotation.MaximumSpeedRadSec * ...
                    ones(3, 1)];
            desiredNormalized = desired ./ velocityLimit;
            achievedNormalized = achieved ./ velocityLimit;
            for index = 1:6
                addpoints(obj.DesiredLines(index), timeSec, ...
                    desiredNormalized(index));
                addpoints(obj.AchievedLines(index), timeSec, ...
                    achievedNormalized(index));
                addpoints(obj.JointCommandLines(index), timeSec, ...
                    qdotCommand(index));
                addpoints(obj.JointActualLines(index), timeSec, ...
                    qdotActual(index));
            end
            poseDisplayScale = [ ...
                obj.Config.cartesianDrag.Translation.RelativeLimitM(:); ...
                obj.Config.cartesianDrag.Rotation.RelativeLimitRad(:)];
            relativeNormalized = relative ./ poseDisplayScale;
            predictedNormalized = predicted ./ poseDisplayScale;
            for index = 1:6
                addpoints(obj.RelativeLines(index), timeSec, ...
                    relativeNormalized(index));
                addpoints(obj.PredictedRelativeLines(index), timeSec, ...
                    predictedNormalized(index));
            end
            xMinimum = max(0, timeSec - obj.WindowSec);
            xMaximum = max(obj.WindowSec, timeSec);
            for index = 1:numel(obj.AxesHandles)
                xlim(obj.AxesHandles(index), [xMinimum, xMaximum]);
            end
            state = string(snapshot.EnableState);
            obj.TitleHandle.String = sprintf([ ...
                '无 RCM 笛卡尔导纳拖拽 | DOF=%s | %s | ' ...
                'scale=%.2f | travel=%s | QP=%s | ServoJ=%d'], ...
                char(obj.DofMode), char(state), snapshot.CommandScale, ...
                char(onOff(obj.Config.cartesianDrag.TravelLimitsEnabled)), ...
                char(snapshot.QpStatusCode), snapshot.MotionCommandSent);
            obj.StatusText.String = sprintf([ ...
                'Space：软件使能 ON；Esc：立即 OFF；R：故障复位；' ...
                '鼠标区域为按住使能\n' ...
                'F=[%+.2f,%+.2f,%+.2f] N | ' ...
                'T=[%+.3f,%+.3f,%+.3f] N m | ' ...
                '|qdot cmd/actual|=%.2f/%.2f deg/s | fault=%s | stop=%s'], ...
                wrench(1), wrench(2), wrench(3), ...
                wrench(4), wrench(5), wrench(6), ...
                max(abs(qdotCommand)), max(abs(qdotActual)), ...
                char(snapshot.FaultCode), ...
                char(orNone(snapshot.WrenchSafetyStopCode)));
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
            drawnow limitrate;
        end

        function open = isOpen(obj)
            open = ~isempty(obj.Figure) && isgraphics(obj.Figure);
        end

        function retained = freezeForReview(obj, reason)
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
            obj.ReviewFrozen = true;
            obj.PreserveFigureOnDelete = true;
            set(obj.Figure, 'WindowKeyPressFcn', '', ...
                'WindowButtonUpFcn', '', ...
                'CloseRequestFcn', @(source, ~)delete(source));
            set(obj.Pad, 'ButtonDownFcn', '', ...
                'FaceColor', [0.45, 0.45, 0.45]);
            xMaximum = max(obj.LastTimeSec, eps);
            for index = 1:numel(obj.AxesHandles)
                xlim(obj.AxesHandles(index), [0, xMaximum]);
            end
            obj.TitleHandle.String = sprintf( ...
                '无 RCM 导纳拖拽已结束：%s | 完整曲线保留', ...
                char(string(reason)));
            obj.StatusText.String = sprintf('%s\n运行已结束；请手动关闭窗口。', ...
                char(string(obj.StatusText.String)));
            drawnow;
            retained = true;
        end

        function close(obj)
            obj.CloseRequested = true;
            obj.KeyboardLatched = false;
            obj.MouseHeld = false;
            if obj.isOpen()
                set(obj.Figure, 'CloseRequestFcn', '', ...
                    'WindowKeyPressFcn', '', 'WindowButtonUpFcn', '');
                delete(obj.Figure);
            end
        end

        function delete(obj)
            if obj.PreserveFigureOnDelete && obj.isOpen()
                obj.Figure = [];
            else
                obj.close();
            end
        end
    end

    methods (Access = private)
        function createFigure(obj)
            obj.Figure = figure('Name', ...
                'ScopeGuide Cartesian Admittance Drag (No RCM)', ...
                'Tag', 'ScopeGuideCartesianDragNoRcm', ...
                'NumberTitle', 'off', 'Color', 'w', ...
                'WindowKeyPressFcn', @(~, event)obj.onKeyPress(event), ...
                'WindowButtonUpFcn', @(~, ~)obj.onMouseRelease(), ...
                'CloseRequestFcn', @(~, ~)obj.close());
            obj.Figure.Position(4) = max(obj.Figure.Position(4), 950);
            layout = tiledlayout(obj.Figure, 5, 1, ...
                'TileSpacing', 'compact', 'Padding', 'compact');
            obj.TitleHandle = title(layout, ...
                '无 RCM 笛卡尔导纳拖拽初始化：请先保持静止');
            obj.AxesHandles = gobjects(1, 5);
            obj.AxesHandles(1) = nexttile(layout, 1);
            obj.ForceLines = threeLines(obj.AxesHandles(1), '-');
            ylabel(obj.AxesHandles(1), 'Force [N]');
            legend(obj.AxesHandles(1), {'Fx','Fy','Fz'}, ...
                'Location', 'northeastoutside'); grid(obj.AxesHandles(1),'on');
            obj.AxesHandles(2) = nexttile(layout, 2);
            obj.MomentLines = threeLines(obj.AxesHandles(2), '-');
            ylabel(obj.AxesHandles(2), 'Moment [N m]');
            legend(obj.AxesHandles(2), {'Tx','Ty','Tz'}, ...
                'Location', 'northeastoutside'); grid(obj.AxesHandles(2),'on');
            obj.AxesHandles(3) = nexttile(layout, 3);
            obj.DesiredLines = sixLines(obj.AxesHandles(3), '-');
            obj.AchievedLines = sixLines(obj.AxesHandles(3), '--');
            ylabel(obj.AxesHandles(3), 'twist / limit');
            ylim(obj.AxesHandles(3), [-1.2, 1.2]);
            names = ["vx","vy","vz","wx","wy","wz"];
            legend(obj.AxesHandles(3), ["des "+names, "qp "+names], ...
                'Location','northeastoutside','NumColumns',2);
            grid(obj.AxesHandles(3),'on');
            obj.AxesHandles(4) = nexttile(layout, 4);
            obj.JointCommandLines = sixLines(obj.AxesHandles(4), '-');
            obj.JointActualLines = sixLines(obj.AxesHandles(4), '--');
            ylabel(obj.AxesHandles(4), 'Joint [deg/s]');
            joints = "J" + string(1:6);
            legend(obj.AxesHandles(4), [joints+" cmd", joints+" actual"], ...
                'Location','northeastoutside','NumColumns',2);
            grid(obj.AxesHandles(4),'on');
            obj.AxesHandles(5) = nexttile(layout, 5);
            obj.RelativeLines = sixLines(obj.AxesHandles(5), '-');
            obj.PredictedRelativeLines = sixLines(obj.AxesHandles(5), '--');
            if obj.Config.cartesianDrag.TravelLimitsEnabled
                ylabel(obj.AxesHandles(5), 'relative / limit');
                ylim(obj.AxesHandles(5), [-1.2, 1.2]);
            else
                ylabel(obj.AxesHandles(5), ...
                    'relative / display scale (travel OFF)');
            end
            xlabel(obj.AxesHandles(5), 'Time [s]');
            legend(obj.AxesHandles(5), ["cur "+names, "pred "+names], ...
                'Location','northeastoutside','NumColumns',2);
            grid(obj.AxesHandles(5),'on');
            obj.StatusText = uicontrol(obj.Figure, 'Style', 'text', ...
                'Units','normalized','Position',[0.02,0.002,0.70,0.075], ...
                'String','启动软件置零中，请勿施力', ...
                'BackgroundColor',[1,1,1], ...
                'HorizontalAlignment','left','FontSize',9);
            padAxes = axes(obj.Figure,'Units','normalized', ...
                'Position',[0.75,0.010,0.20,0.055], ...
                'XLim',[0,1],'YLim',[0,1], ...
                'XTick',[],'YTick',[],'Box','on');
            obj.Pad = patch(padAxes,[0,1,1,0],[0,0,1,1], ...
                [0.25,0.50,0.85], ...
                'ButtonDownFcn',@(~,~)obj.onMousePress());
            text(padAxes,0.5,0.5,'鼠标按住运动使能', ...
                'HorizontalAlignment','center','VerticalAlignment','middle', ...
                'Color','w','FontWeight','bold', ...
                'HitTest','off','PickableParts','none');
        end

        function onKeyPress(obj, event)
            switch string(event.Key)
                case "space"
                    if obj.BaselineReady
                        obj.KeyboardLatched = true;
                        obj.SpaceCount = obj.SpaceCount + uint64(1);
                    end
                case "escape"
                    obj.KeyboardLatched = false;
                    obj.MouseHeld = false;
                    obj.EscapeCount = obj.EscapeCount + uint64(1);
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

function validateSnapshot(snapshot)
required = {'TimeSec','BaselineReady','ControlWrenchDrag', ...
    'DesiredTwistBase','AchievedTwistBase','RelativeCoordinate', ...
    'PredictedRelativeCoordinate','EnableState','CommandScale', ...
    'QpStatusCode','FaultCode','MotionCommandSent', ...
    'JointVelocityCommandRadSec','JointVelocityActualRadSec', ...
    'WrenchSafetyStopCode','NoRcmConstraint'};
if ~isstruct(snapshot) || ~all(isfield(snapshot, required))
    error('scopeguide:cartesianDrag:InvalidDisplaySnapshot', ...
        'Cartesian drag display snapshot is incomplete.');
end
vectors = {'ControlWrenchDrag','DesiredTwistBase', ...
    'AchievedTwistBase','RelativeCoordinate', ...
    'PredictedRelativeCoordinate','JointVelocityCommandRadSec', ...
    'JointVelocityActualRadSec'};
for index = 1:numel(vectors)
    value = snapshot.(vectors{index});
    if ~isnumeric(value) || numel(value) ~= 6 || ...
            any(~isfinite(value(:)))
        error('scopeguide:cartesianDrag:InvalidDisplaySnapshot', ...
            'Display field %s must be finite 6-by-1.', vectors{index});
    end
end
if ~logical(snapshot.NoRcmConstraint)
    error('scopeguide:cartesianDrag:UnexpectedRcmSnapshot', ...
        'No-RCM dashboard refuses RCM-constrained telemetry.');
end
end

function handles = threeLines(axisHandle, style)
hold(axisHandle,'on');
colors = [0.85,0.15,0.15;0.10,0.55,0.20;0.10,0.25,0.85];
handles = gobjects(1,3);
for index=1:3
    handles(index)=animatedline(axisHandle,'Color',colors(index,:), ...
        'LineStyle',style,'LineWidth',1.1);
end
end

function handles = sixLines(axisHandle, style)
hold(axisHandle,'on');
colors = lines(6);
handles = gobjects(1,6);
for index=1:6
    handles(index)=animatedline(axisHandle,'Color',colors(index,:), ...
        'LineStyle',style,'LineWidth',1.0);
end
end

function value = orNone(input)
value = string(input);
if strlength(value) == 0
    value = "none";
end
end

function value = onOff(input)
if logical(input)
    value = "ON";
else
    value = "OFF";
end
end
