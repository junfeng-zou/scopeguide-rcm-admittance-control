function coordinate = relativeDragPose(TBaseAnchor, TBaseCurrent)
%RELATIVEDRAGPOSE Smallest relative pose in anchor-frame coordinates.
% Output order is [dx;dy;dz;rx;ry;rz].  Rotation is the SO(3) logarithm.

TBaseAnchor = validateTransform(TBaseAnchor, 'TBaseAnchor');
TBaseCurrent = validateTransform(TBaseCurrent, 'TBaseCurrent');
R0 = TBaseAnchor(1:3, 1:3);
relativePosition = R0' * ( ...
    TBaseCurrent(1:3, 4) - TBaseAnchor(1:3, 4));
relativeRotation = R0' * TBaseCurrent(1:3, 1:3);
coordinate = [relativePosition; rotationLog(relativeRotation)];
end

function vector = rotationLog(rotation)
cosAngle = min(max((trace(rotation) - 1) / 2, -1), 1);
angle = acos(cosAngle);
if angle < 1e-9
    vector = 0.5 * [rotation(3, 2) - rotation(2, 3); ...
        rotation(1, 3) - rotation(3, 1); ...
        rotation(2, 1) - rotation(1, 2)];
elseif pi - angle < 1e-6
    [vectors, values] = eig((rotation + eye(3)) / 2);
    [~, index] = max(real(diag(values)));
    axis = real(vectors(:, index));
    axis = axis / max(norm(axis), eps);
    skewVector = [rotation(3, 2) - rotation(2, 3); ...
        rotation(1, 3) - rotation(3, 1); ...
        rotation(2, 1) - rotation(1, 2)];
    if dot(axis, skewVector) < 0
        axis = -axis;
    end
    vector = angle * axis;
else
    vector = angle / (2 * sin(angle)) * [ ...
        rotation(3, 2) - rotation(2, 3); ...
        rotation(1, 3) - rotation(3, 1); ...
        rotation(2, 1) - rotation(1, 2)];
end
end

function transform = validateTransform(value, name)
if ~isnumeric(value) || ~isequal(size(value), [4, 4]) || ...
        any(~isfinite(value(:))) || ...
        norm(double(value(4, :)) - [0, 0, 0, 1], inf) > 1e-10
    error('scopeguide:cartesianDrag:InvalidPoseTransform', ...
        '%s must be a finite homogeneous transform.', name);
end
transform = double(value);
rotation = transform(1:3, 1:3);
if norm(rotation' * rotation - eye(3), 'fro') > 1e-8 || ...
        abs(det(rotation) - 1) > 1e-8
    error('scopeguide:cartesianDrag:InvalidPoseTransform', ...
        '%s rotation is invalid.', name);
end
end
