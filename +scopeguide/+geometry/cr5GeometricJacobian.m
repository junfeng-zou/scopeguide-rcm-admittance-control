function jacobians = cr5GeometricJacobian(jointPositionRad, cfg)
%CR5GEOMETRICJACOBIAN Spatial geometric Jacobians in the base frame.
% Linear rows refer respectively to the flange origin and endoscope tip.

kinematics = scopeguide.geometry.cr5ForwardKinematics( ...
    jointPositionRad, cfg);
flangePosition = kinematics.FlangePositionBaseM;
tipPosition = kinematics.EndoscopeTipPositionBaseM;
flangeJacobian = zeros(6, 6);
tipJacobian = zeros(6, 6);
for index = 1:6
    axis = kinematics.JointAxesBase(:, index);
    origin = kinematics.JointOriginsBaseM(:, index);
    flangeJacobian(1:3, index) = cross( ...
        axis, flangePosition - origin);
    flangeJacobian(4:6, index) = axis;
    tipJacobian(1:3, index) = cross(axis, tipPosition - origin);
    tipJacobian(4:6, index) = axis;
end

jacobians = struct();
jacobians.Flange = flangeJacobian;
jacobians.EndoscopeTip = tipJacobian;
jacobians.ExpressedIn = "base";
jacobians.AngularVelocityConvention = "spatial";
jacobians.Kinematics = kinematics;
end
