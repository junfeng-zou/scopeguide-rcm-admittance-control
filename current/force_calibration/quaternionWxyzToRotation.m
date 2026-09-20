function rotation = quaternionWxyzToRotation(quaternionWxyz)
%QUATERNIONWXYZTOROTATION Convert [qw,qx,qy,qz] to a rotation matrix.
% This implementation uses no Robotics System Toolbox functions.

quaternion = double(quaternionWxyz(:).');
if numel(quaternion) ~= 4 || any(~isfinite(quaternion))
    error('calibration:InvalidQuaternion', ...
        'Quaternion must be a finite [qw,qx,qy,qz] vector.');
end
quaternionNorm = norm(quaternion);
if quaternionNorm < 1e-9
    error('calibration:InvalidQuaternion', ...
        'Quaternion norm is too small.');
end
quaternion = quaternion / quaternionNorm;
w = quaternion(1);
x = quaternion(2);
y = quaternion(3);
z = quaternion(4);

rotation = [ ...
    1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w); ...
    2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w); ...
    2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)];
end
