function [force, moment, description] = ...
        selectWrenchDisplaySignal(processed, signal)
%SELECTWRENCHDISPLAYSIGNAL Select one canonical wrench branch for plotting.

arguments
    processed (1, 1) struct
    signal (1, 1) string {mustBeMember(signal, ...
        ["external", "fast", "slow", "control"])}
end

switch signal
    case "external"
        wrench = requireVector(processed, ...
            'ExternalWrenchToolAtSensorOrigin', 6);
        force = wrench(1:3);
        moment = wrench(4:6);
        description = "零偏+重力补偿后，未滤波、未死区";
    case "fast"
        wrench = requireVector(processed, ...
            'FastWrenchToolAtSensorOrigin', 6);
        force = wrench(1:3);
        moment = wrench(4:6);
        description = "零偏+重力补偿后，20 Hz快速滤波、未死区";
    case "slow"
        wrench = requireVector(processed, ...
            'SlowWrenchToolAtSensorOrigin', 6);
        force = wrench(1:3);
        moment = wrench(4:6);
        description = "零偏+重力补偿后，5 Hz慢速滤波、未死区";
    case "control"
        force = requireVector(processed, 'ControlForceTool', 3);
        moment = requireVector( ...
            processed, 'ControlMomentForDiagnostics', 3);
        description = "完整处理链死区后";
end
end

function value = requireVector(input, field, count)
if ~isfield(input, field)
    error('scopeguide:force:MissingWrenchDisplaySignal', ...
        'Processed wrench is missing field %s.', field);
end
value = double(input.(field)(:));
if numel(value) ~= count
    error('scopeguide:force:InvalidWrenchDisplaySignal', ...
        'Processed field %s must contain %d elements.', field, count);
end
end
