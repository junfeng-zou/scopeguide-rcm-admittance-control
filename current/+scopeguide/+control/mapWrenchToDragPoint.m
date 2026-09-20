function mapped = mapWrenchToDragPoint( ...
        wrenchToolAtSensorOrigin, TBaseFlange, cfg)
%MAPWRENCHTODRAGPOINT Shift wrench to the configured no-RCM drag point.
% The input components are expressed in flange/tool axes and referenced at
% the HEX-H measurement origin.  Output control components are expressed
% in the drag-point axes.

validateCartesianAdmittanceDragConfig(cfg);
wrench = validateVector(wrenchToolAtSensorOrigin, 6, ...
    'wrenchToolAtSensorOrigin');
TBaseFlange = validateTransform(TBaseFlange, 'TBaseFlange');
TFlangeDrag = double(cfg.cartesianDrag.TFlangeDragPoint);
pSensorFlange = double(cfg.tool.TFlangeSensor(1:3, 4));
pDragFlange = TFlangeDrag(1:3, 4);
rotationFlangeFromDrag = TFlangeDrag(1:3, 1:3);

forceFlange = wrench(1:3);
momentSensorFlange = wrench(4:6);
sensorToDragFlange = pSensorFlange - pDragFlange;
momentDragFlange = momentSensorFlange + ...
    cross(sensorToDragFlange, forceFlange);
forceDrag = rotationFlangeFromDrag' * forceFlange;
momentDrag = rotationFlangeFromDrag' * momentDragFlange;
rotationBaseFromDrag = TBaseFlange(1:3, 1:3) * ...
    rotationFlangeFromDrag;

mapped = struct();
mapped.WrenchToolAtSensorOrigin = wrench;
mapped.WrenchDragAtDragPoint = [forceDrag; momentDrag];
mapped.ForceFlangeN = forceFlange;
mapped.MomentFlangeAtSensorOriginNm = momentSensorFlange;
mapped.MomentFlangeAtDragPointNm = momentDragFlange;
mapped.ForceDragN = forceDrag;
mapped.MomentDragNm = momentDrag;
mapped.SensorToDragLeverFlangeM = sensorToDragFlange;
mapped.PSensorFlangeM = pSensorFlange;
mapped.PDragFlangeM = pDragFlange;
mapped.RotationBaseFromDrag = rotationBaseFromDrag;
mapped.TBaseDragPoint = TBaseFlange * TFlangeDrag;
mapped.NoRcmPointUsed = true;
end

function value = validateVector(input, count, name)
if ~isnumeric(input) || numel(input) ~= count || ...
        any(~isfinite(input(:)))
    error('scopeguide:cartesianDrag:InvalidWrenchInput', ...
        '%s must contain %d finite numeric values.', name, count);
end
value = double(input(:));
end

function transform = validateTransform(value, name)
if ~isnumeric(value) || ~isequal(size(value), [4, 4]) || ...
        any(~isfinite(value(:))) || ...
        norm(double(value(4, :)) - [0, 0, 0, 1], inf) > 1e-12
    error('scopeguide:cartesianDrag:InvalidWrenchGeometry', ...
        '%s must be a finite homogeneous transform.', name);
end
transform = double(value);
rotation = transform(1:3, 1:3);
if norm(rotation' * rotation - eye(3), 'fro') > 1e-9 || ...
        abs(det(rotation) - 1) > 1e-9
    error('scopeguide:cartesianDrag:InvalidWrenchGeometry', ...
        '%s rotation must be right-handed and orthonormal.', name);
end
end
