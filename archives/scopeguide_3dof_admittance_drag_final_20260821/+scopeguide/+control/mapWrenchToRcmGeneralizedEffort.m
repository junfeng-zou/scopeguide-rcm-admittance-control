function mapped = mapWrenchToRcmGeneralizedEffort( ...
        wrenchToolAtSensorOrigin, TBaseEndoscope, pRcmBaseM, cfg)
%MAPWRENCHTORCMGENERALIZEDEFFORT Shift a measured wrench to the RCM.
% Input order is [Fx; Fy; Fz; Tx; Ty; Tz].  Components are expressed in
% flange/tool axes, while the wrench reference point is the HEX-H
% measurement origin specified by cfg.tool.TFlangeSensor.

validateRcmAdmittanceConfig(cfg);
wrenchTool = validateWrench(wrenchToolAtSensorOrigin);
pRcm = validateVector(pRcmBaseM, 3, 'pRcmBaseM');
TBaseEndoscope = validateRigidTransform(TBaseEndoscope, ...
    'TBaseEndoscope');
TFlangeEndoscope = double(cfg.tool.TFlangeEndoscope);
TFlangeSensor = double(cfg.tool.TFlangeSensor);

% TBaseEndoscope = TBaseFlange * TFlangeEndoscope.
TBaseFlange = TBaseEndoscope * rigidInverse(TFlangeEndoscope);
TBaseSensor = TBaseFlange * TFlangeSensor;
rotationBaseFromTool = TBaseFlange(1:3, 1:3);
pSensorBase = TBaseSensor(1:3, 4);

forceBase = rotationBaseFromTool * wrenchTool(1:3);
momentSensorBase = rotationBaseFromTool * wrenchTool(4:6);
sensorToRcmLeverBase = pSensorBase - pRcm;
momentRcmBase = momentSensorBase + ...
    cross(sensorToRcmLeverBase, forceBase);

basis = scopeguide.geometry.computeRcmMotionBasis( ...
    TBaseEndoscope, pRcm, cfg);
generalizedEffort = [ ...
    dot(basis.PivotAxis1Base, momentRcmBase); ...
    dot(basis.PivotAxis2Base, momentRcmBase); ...
    dot(basis.InsertionDirectionBase, forceBase)];

mapped = struct();
mapped.WrenchToolAtSensorOrigin = wrenchTool;
mapped.ForceToolN = wrenchTool(1:3);
mapped.MomentToolAtSensorOriginNm = wrenchTool(4:6);
mapped.ForceBaseN = forceBase;
mapped.MomentBaseAtSensorOriginNm = momentSensorBase;
mapped.MomentBaseAtRcmNm = momentRcmBase;
mapped.PSensorBaseM = pSensorBase;
mapped.SensorToRcmLeverBaseM = sensorToRcmLeverBase;
mapped.SensorToRcmDistanceM = norm(sensorToRcmLeverBase);
mapped.TBaseFlange = TBaseFlange;
mapped.TBaseSensor = TBaseSensor;
mapped.GeneralizedEffort = generalizedEffort;
mapped.GeneralizedEffortWithRoll = [generalizedEffort; 0];
mapped.Basis = basis;
mapped.MomentUsedForControl = true;
mapped.RollEffort = 0;
end

function wrench = validateWrench(value)
if ~isnumeric(value) || numel(value) ~= 6 || ...
        any(~isfinite(value(:)))
    error('scopeguide:control:InvalidToolWrench', ...
        ['wrenchToolAtSensorOrigin must contain six finite values in ' ...
         '[Fx,Fy,Fz,Tx,Ty,Tz] order.']);
end
wrench = double(value(:));
end

function vector = validateVector(value, count, name)
if ~isnumeric(value) || numel(value) ~= count || ...
        any(~isfinite(value(:)))
    error('scopeguide:control:InvalidRcmWrenchGeometry', ...
        '%s must contain %d finite numeric values.', name, count);
end
vector = double(value(:));
end

function transform = validateRigidTransform(value, name)
if ~isnumeric(value) || ~isequal(size(value), [4, 4]) || ...
        any(~isfinite(value(:))) || ...
        norm(double(value(4, :)) - [0, 0, 0, 1], inf) > 1e-12
    error('scopeguide:control:InvalidRcmWrenchGeometry', ...
        '%s must be a finite rigid 4-by-4 transform.', name);
end
transform = double(value);
rotation = transform(1:3, 1:3);
if norm(rotation' * rotation - eye(3), 'fro') > 1e-9 || ...
        abs(det(rotation) - 1) > 1e-9
    error('scopeguide:control:InvalidRcmWrenchGeometry', ...
        '%s rotation must be right-handed and orthonormal.', name);
end
end

function inverse = rigidInverse(transform)
rotation = transform(1:3, 1:3);
translation = transform(1:3, 4);
inverse = [rotation.', -rotation.' * translation; 0, 0, 0, 1];
end
