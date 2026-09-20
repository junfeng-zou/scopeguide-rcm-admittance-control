function geometry = computeDragPointKinematics(jointPositionRad, cfg)
%COMPUTEDRAGPOINTKINEMATICS Pose and spatial Jacobian of the drag point.
% Twist order is [linear; angular], expressed in the robot base frame.

validateCartesianAdmittanceDragConfig(cfg);
jacobians = scopeguide.geometry.cr5GeometricJacobian( ...
    jointPositionRad, cfg);
kinematics = jacobians.Kinematics;
TBaseFlange = kinematics.TBaseFlange;
TFlangeDrag = double(cfg.cartesianDrag.TFlangeDragPoint);
TBaseDrag = TBaseFlange * TFlangeDrag;

offsetBase = TBaseFlange(1:3, 1:3) * TFlangeDrag(1:3, 4);
JFlange = jacobians.Flange;
JDrag = [JFlange(1:3, :) - skew(offsetBase) * JFlange(4:6, :); ...
    JFlange(4:6, :)];

geometry = struct();
geometry.JointPositionRad = double(jointPositionRad(:));
geometry.TBaseFlange = TBaseFlange;
geometry.TBaseDragPoint = TBaseDrag;
geometry.PDragPointBaseM = TBaseDrag(1:3, 4);
geometry.RotationBaseFromDrag = TBaseDrag(1:3, 1:3);
geometry.DragPointTwistJacobianBase = JDrag;
geometry.FlangeTwistJacobianBase = JFlange;
geometry.FlangeToDragOffsetBaseM = offsetBase;
geometry.Kinematics = kinematics;
geometry.NoRcmConstraint = true;
end

function matrix = skew(vector)
vector = double(vector(:));
matrix = [0, -vector(3), vector(2); ...
    vector(3), 0, -vector(1); ...
    -vector(2), vector(1), 0];
end
