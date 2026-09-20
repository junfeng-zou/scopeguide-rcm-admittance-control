function [directionSensor, directionTool] = ...
        gravityDirectionFromDobotQuaternion(quaternionWxyz, varargin)
%GRAVITYDIRECTIONFROMDOBOTQUATERNION Express physical gravity in sensor/tool.

parser = inputParser;
addParameter(parser, 'RToolFromSensor', diag([-1, -1, 1]), ...
    @(x) isnumeric(x) && isequal(size(x), [3, 3]));
addParameter(parser, 'GravityBaseMps2', [0, 0, -9.80665], ...
    @(x) isnumeric(x) && numel(x) == 3 && all(isfinite(x)));
addParameter(parser, 'QuaternionConvention', 'base_from_tool', ...
    @(x) ischar(x) || isstring(x));
parse(parser, varargin{:});
opts = parser.Results;

rotationToolFromSensor = double(opts.RToolFromSensor);
if norm(rotationToolFromSensor * rotationToolFromSensor.' - eye(3), 'fro') > 1e-7 || ...
        abs(det(rotationToolFromSensor) - 1) > 1e-7
    error('calibration:InvalidRotation', ...
        'RToolFromSensor must be a right-handed rotation matrix.');
end
rotationBaseFromTool = quaternionWxyzToRotation(quaternionWxyz);
if strcmpi(char(opts.QuaternionConvention), 'base_from_tool')
    rotationToolFromBase = rotationBaseFromTool.';
elseif strcmpi(char(opts.QuaternionConvention), 'tool_from_base')
    rotationToolFromBase = rotationBaseFromTool;
else
    error('calibration:InvalidQuaternionConvention', ...
        'QuaternionConvention must be base_from_tool or tool_from_base.');
end
rotationSensorFromTool = rotationToolFromSensor.';
gravityBase = double(opts.GravityBaseMps2(:));
gravitySensor = rotationSensorFromTool * rotationToolFromBase * gravityBase;
directionSensor = (gravitySensor / norm(gravitySensor)).';
directionTool = (rotationToolFromSensor * directionSensor.').';
end
