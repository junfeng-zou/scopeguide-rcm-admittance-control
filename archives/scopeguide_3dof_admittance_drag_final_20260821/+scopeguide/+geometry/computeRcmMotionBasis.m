function basis = computeRcmMotionBasis(TBaseEndoscope, pRcmBaseM, cfg)
%COMPUTERCMMOTIONBASIS Construct a stable force-only 3-DOF RCM basis.
% Columns are pivot about b1, pivot about b2 and shaft insertion.  The
% output twist is ordered [linear velocity; angular velocity] in robot base.

validateRcmAdmittanceConfig(cfg);
validateTransform(TBaseEndoscope);
pRcm = validateVector(pRcmBaseM, 3, 'pRcmBaseM');

rotation = double(TBaseEndoscope(1:3, 1:3));
tipHomogeneous = double(TBaseEndoscope) * ...
    [double(cfg.tool.TipInEndoscopeM(:)); 1];
pTip = tipHomogeneous(1:3);

shaftTool = double(cfg.tool.ShaftAxisEndoscope(:));
shaftTool = shaftTool / norm(shaftTool);
% Choose once in the tool frame.  Because the shaft definition is fixed in
% that frame, the selected canonical reference cannot switch with pose.
canonicalAxes = eye(3);
[~, referenceIndex] = min(abs(canonicalAxes' * shaftTool));
referenceTool = canonicalAxes(:, referenceIndex);
b1Tool = referenceTool - shaftTool * dot(shaftTool, referenceTool);
b1Tool = b1Tool / norm(b1Tool);
b2Tool = cross(shaftTool, b1Tool);
b2Tool = b2Tool / norm(b2Tool);

shaftBase = rotation * shaftTool;
b1Base = rotation * b1Tool;
b2Base = rotation * b2Tool;
insertionBase = double(cfg.tool.InsertionAxisSign) * shaftBase;
rBase = pTip - pRcm;

linearBasis = [cross(b1Base, rBase), ...
    cross(b2Base, rBase), insertionBase];
angularBasis = [b1Base, b2Base, zeros(3, 1)];

basis = struct();
basis.PivotAxis1Tool = b1Tool;
basis.PivotAxis2Tool = b2Tool;
basis.ShaftAxisTool = shaftTool;
basis.PivotAxis1Base = b1Base;
basis.PivotAxis2Base = b2Base;
basis.ShaftAxisBase = shaftBase;
basis.InsertionDirectionBase = insertionBase;
basis.PRcmBaseM = pRcm;
basis.PTipBaseM = pTip;
basis.RcmToTipBaseM = rBase;
basis.RcmToTipDistanceM = norm(rBase);
basis.LinearVelocityBasisBase = linearBasis;
basis.AngularVelocityBasisBase = angularBasis;
basis.TwistBasisBase = [linearBasis; angularBasis];
basis.RollRateRadSec = 0;
end

function validateTransform(transform)
if ~isnumeric(transform) || ~isequal(size(transform), [4, 4]) || ...
        any(~isfinite(transform(:))) || ...
        norm(transform(4, :) - [0, 0, 0, 1], inf) > 1e-12
    error('scopeguide:geometry:InvalidEndoscopeTransform', ...
        'TBaseEndoscope must be a finite rigid 4-by-4 transform.');
end
rotation = double(transform(1:3, 1:3));
if norm(rotation' * rotation - eye(3), 'fro') > 1e-9 || ...
        abs(det(rotation) - 1) > 1e-9
    error('scopeguide:geometry:InvalidEndoscopeTransform', ...
        'TBaseEndoscope rotation must be right-handed and orthonormal.');
end
end

function vector = validateVector(value, count, name)
if ~isnumeric(value) || numel(value) ~= count || ...
        any(~isfinite(value(:)))
    error('scopeguide:geometry:InvalidRcmGeometryInput', ...
        '%s must contain %d finite numeric values.', name, count);
end
vector = double(value(:));
end
