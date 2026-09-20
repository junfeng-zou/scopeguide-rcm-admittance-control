function vector = rotationLogVector(rotation)
%ROTATIONLOGVECTOR Return the SO(3) logarithm as an axis-angle vector.

validateRotation(rotation);
cosine = max(-1, min(1, (trace(rotation) - 1) / 2));
angle = acos(cosine);
skewVector = 0.5 * [ ...
    rotation(3, 2) - rotation(2, 3); ...
    rotation(1, 3) - rotation(3, 1); ...
    rotation(2, 1) - rotation(1, 2)];
if angle < 1e-8
    vector = skewVector;
elseif pi - angle < 1e-6
    % This branch is not used by small Jacobian perturbations, but keeps
    % pose-error reporting finite near 180 degrees.
    axisMatrix = (rotation + eye(3)) / 2;
    [~, index] = max(diag(axisMatrix));
    axis = zeros(3, 1);
    axis(index) = sqrt(max(axisMatrix(index, index), 0));
    if axis(index) < 1e-8
        axis = [1; 0; 0];
    else
        other = setdiff(1:3, index);
        axis(other) = axisMatrix(other, index) / axis(index);
        axis = axis / norm(axis);
    end
    if dot(axis, skewVector) < 0
        axis = -axis;
    end
    vector = angle * axis;
else
    vector = (angle / sin(angle)) * skewVector;
end
end

function validateRotation(rotation)
if ~isnumeric(rotation) || ~isequal(size(rotation), [3, 3]) || ...
        any(~isfinite(rotation), 'all') || ...
        norm(rotation' * rotation - eye(3), 'fro') > 1e-7 || ...
        abs(det(rotation) - 1) > 1e-7
    error('scopeguide:geometry:InvalidRotation', ...
        'Input must be a finite right-handed rotation matrix.');
end
end
