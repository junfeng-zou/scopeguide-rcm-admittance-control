function kinematics = cr5ForwardKinematics(jointPositionRad, cfg)
%CR5FORWARDKINEMATICS Standard-DH FK for flange and endoscope tip.

validateRcmAdmittanceConfig(cfg);
q = double(jointPositionRad(:));
if numel(q) ~= 6 || any(~isfinite(q))
    error('scopeguide:kinematics:InvalidJointPosition', ...
        'jointPositionRad must contain six finite radians.');
end
if any(abs(q) > cfg.kinematics.MaximumAbsJointAngleRad)
    error('scopeguide:kinematics:JointUnitOrRangeError', ...
        ['Joint magnitude exceeds MaximumAbsJointAngleRad. Check that ' ...
        'the input is in radians, not degrees.']);
end

theta = q + cfg.kinematics.ThetaOffsetRad(:);
d = cfg.kinematics.DM(:);
a = cfg.kinematics.AM(:);
alpha = cfg.kinematics.AlphaRad(:);
transforms = zeros(4, 4, 6);
jointOrigins = zeros(3, 6);
jointAxes = zeros(3, 6);
transform = eye(4);
for index = 1:6
    jointOrigins(:, index) = transform(1:3, 4);
    jointAxes(:, index) = transform(1:3, 3);
    transform = transform * standardDhTransform( ...
        theta(index), d(index), a(index), alpha(index));
    transforms(:, :, index) = transform;
end

kinematics = struct();
kinematics.JointPositionRad = q;
kinematics.TBaseLink = transforms;
kinematics.TBaseFlange = transform;
kinematics.TBaseEndoscope = transform * cfg.tool.TFlangeEndoscope;
kinematics.JointOriginsBaseM = jointOrigins;
kinematics.JointAxesBase = jointAxes;
kinematics.FlangePositionBaseM = transform(1:3, 4);
tipHomogeneous = kinematics.TBaseEndoscope * ...
    [cfg.tool.TipInEndoscopeM(:); 1];
kinematics.EndoscopeTipPositionBaseM = tipHomogeneous(1:3);
kinematics.ShaftAxisBase = ...
    kinematics.TBaseEndoscope(1:3, 1:3) * ...
    cfg.tool.ShaftAxisEndoscope(:);
end

function transform = standardDhTransform(theta, d, a, alpha)
ct = cos(theta); st = sin(theta);
ca = cos(alpha); sa = sin(alpha);
transform = [ ...
    ct, -st*ca, st*sa, a*ct; ...
    st, ct*ca, -ct*sa, a*st; ...
    0, sa, ca, d; ...
    0, 0, 0, 1];
end
