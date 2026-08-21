function geometry = computeRcmConstraintKinematics( ...
        jointPositionRad, pRcmBaseM, cfg)
%COMPUTERCMCONSTRAINTKINEMATICS RCM error and velocity Jacobians.
% The two independent RCM rows describe lateral velocity of the shaft point
% closest to pRcm.  Axial insertion is therefore not constrained.

validateRcmAdmittanceConfig(cfg);
q = double(jointPositionRad(:));
pRcm = double(pRcmBaseM(:));
if numel(q) ~= 6 || any(~isfinite(q)) || ...
        numel(pRcm) ~= 3 || any(~isfinite(pRcm))
    error('scopeguide:geometry:InvalidRcmConstraintInput', ...
        'Joint position and pRcm must contain 6 and 3 finite values.');
end

jacobians = scopeguide.geometry.cr5GeometricJacobian(q, cfg);
kinematics = jacobians.Kinematics;
basis = scopeguide.geometry.computeRcmMotionBasis( ...
    kinematics.TBaseEndoscope, pRcm, cfg);
pTip = basis.PTipBaseM;
shaftAxis = basis.ShaftAxisBase;
transverseBasis = [basis.PivotAxis1Base, basis.PivotAxis2Base];
shaftCoordinate = dot(shaftAxis, pRcm - pTip);
tipToClosestPoint = shaftCoordinate * shaftAxis;
pClosest = pTip + tipToClosestPoint;
errorBase = pRcm - pClosest;
error2 = transverseBasis' * errorBase;

tipJacobian = jacobians.EndoscopeTip;
linePointJacobian = [eye(3), -skew(tipToClosestPoint)] * ...
    tipJacobian;
rcmJacobian = transverseBasis' * linePointJacobian;
generalizedRateProjection = [ ...
    zeros(1, 3), basis.PivotAxis1Base'; ...
    zeros(1, 3), basis.PivotAxis2Base'; ...
    basis.InsertionDirectionBase', zeros(1, 3)];
generalizedRateJacobian = generalizedRateProjection * tipJacobian;

geometry = struct();
geometry.JointPositionRad = q;
geometry.PRcmBaseM = pRcm;
geometry.PTipBaseM = pTip;
geometry.PClosestBaseM = pClosest;
geometry.TipToClosestPointBaseM = tipToClosestPoint;
geometry.ShaftCoordinateM = shaftCoordinate;
geometry.ShaftAxisBase = shaftAxis;
geometry.TransverseBasisBase = transverseBasis;
geometry.RcmErrorBaseM = errorBase;
geometry.RcmError2M = error2;
geometry.RcmErrorNormM = norm(errorBase);
geometry.TipTwistJacobianBase = tipJacobian;
geometry.ClosestLinePointJacobianBase = linePointJacobian;
geometry.RcmLateralJacobian = rcmJacobian;
geometry.GeneralizedRateProjection = generalizedRateProjection;
geometry.GeneralizedRateJacobian = generalizedRateJacobian;
geometry.MotionBasis = basis;
geometry.Kinematics = kinematics;
end

function matrix = skew(vector)
vector = double(vector(:));
matrix = [ ...
    0, -vector(3), vector(2); ...
    vector(3), 0, -vector(1); ...
    -vector(2), vector(1), 0];
end
