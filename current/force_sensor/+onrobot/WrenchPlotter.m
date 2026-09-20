classdef WrenchPlotter < handle
    %WRENCHPLOTTER Real-time visualization for HEX-H force and torque.

    properties (SetAccess = private)
        Figure
    end

    properties (Access = private)
        ForceAxes
        TorqueAxes
        ForceLines
        TorqueLines
        WindowSec
    end

    methods
        function obj = WrenchPlotter(windowSec)
            if nargin < 1
                windowSec = 10;
            end
            obj.WindowSec = windowSec;

            obj.Figure = figure('Name', 'OnRobot HEX-H QC Live Wrench', ...
                'NumberTitle', 'off', 'Color', 'w');
            layout = tiledlayout(obj.Figure, 2, 1, ...
                'TileSpacing', 'compact', 'Padding', 'compact');
            title(layout, 'OnRobot HEX-H QC 实时六维力/力矩');

            obj.ForceAxes = nexttile(layout, 1);
            hold(obj.ForceAxes, 'on');
            obj.ForceLines = [ ...
                animatedline(obj.ForceAxes, 'Color', [0.85 0.15 0.15], 'LineWidth', 1.2), ...
                animatedline(obj.ForceAxes, 'Color', [0.10 0.55 0.20], 'LineWidth', 1.2), ...
                animatedline(obj.ForceAxes, 'Color', [0.10 0.25 0.85], 'LineWidth', 1.2)];
            ylabel(obj.ForceAxes, 'Force [N]');
            legend(obj.ForceAxes, {'Fx', 'Fy', 'Fz'}, 'Location', 'northeastoutside');
            grid(obj.ForceAxes, 'on');

            obj.TorqueAxes = nexttile(layout, 2);
            hold(obj.TorqueAxes, 'on');
            obj.TorqueLines = [ ...
                animatedline(obj.TorqueAxes, 'Color', [0.85 0.15 0.15], 'LineWidth', 1.2), ...
                animatedline(obj.TorqueAxes, 'Color', [0.10 0.55 0.20], 'LineWidth', 1.2), ...
                animatedline(obj.TorqueAxes, 'Color', [0.10 0.25 0.85], 'LineWidth', 1.2)];
            xlabel(obj.TorqueAxes, 'Time [s]');
            ylabel(obj.TorqueAxes, 'Torque [N m]');
            legend(obj.TorqueAxes, {'Tx', 'Ty', 'Tz'}, 'Location', 'northeastoutside');
            grid(obj.TorqueAxes, 'on');
        end

        function update(obj, sample)
            if ~obj.isOpen()
                return;
            end
            t = sample.monotonicTime;
            for i = 1:3
                addpoints(obj.ForceLines(i), t, sample.force(i));
                addpoints(obj.TorqueLines(i), t, sample.torque(i));
            end

            xMin = max(0, t - obj.WindowSec);
            xMax = max(obj.WindowSec, t);
            xlim(obj.ForceAxes, [xMin, xMax]);
            xlim(obj.TorqueAxes, [xMin, xMax]);
            title(obj.ForceAxes, sprintf( ...
                'F = [%+.2f, %+.2f, %+.2f] N,  |F| = %.2f N', ...
                sample.force(1), sample.force(2), sample.force(3), ...
                sample.forceNorm));
            title(obj.TorqueAxes, sprintf( ...
                'T = [%+.3f, %+.3f, %+.3f] N m,  |T| = %.3f N m', ...
                sample.torque(1), sample.torque(2), sample.torque(3), ...
                sample.torqueNorm));
            drawnow limitrate;
        end

        function tf = isOpen(obj)
            tf = ~isempty(obj.Figure) && isgraphics(obj.Figure);
        end
    end
end
