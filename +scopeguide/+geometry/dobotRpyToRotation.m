function rotation = dobotRpyToRotation(rpy, angleScaleToRad)
%DOBOTRPYTOROTATION Convert Dobot fixed-axis XYZ RPY to a rotation.
% The convention is R = Rz(rz)*Ry(ry)*Rx(rx). The live validation script
% compares this candidate interpretation against ActualQuaternion.

if nargin < 2
    angleScaleToRad = pi / 180;
end
angles = double(rpy(:)) * angleScaleToRad;
if numel(angles) ~= 3 || any(~isfinite(angles)) || ...
        ~isscalar(angleScaleToRad) || ~isfinite(angleScaleToRad) || ...
        angleScaleToRad <= 0
    error('scopeguide:geometry:InvalidRpy', ...
        'RPY and angle scale must be finite and valid.');
end
rx = angles(1); ry = angles(2); rz = angles(3);
cx = cos(rx); sx = sin(rx);
cy = cos(ry); sy = sin(ry);
cz = cos(rz); sz = sin(rz);
rotation = [ ...
    cy*cz, cz*sx*sy - cx*sz, cx*cz*sy + sx*sz; ...
    cy*sz, cx*cz + sx*sy*sz, -cz*sx + cx*sy*sz; ...
    -sy, cy*sx, cx*cy];
end
