function plan = buildThreePoseCartesianTargets( ...
        currentPose, poseBOrientationDeltaDeg, ...
        poseCOrientationDeltaDeg, maximumOrientationTransitionDeg)
%BUILDTHREEPOSECARTESIANTARGETS Build guarded A/B/C/A Cartesian targets.
% Controller X/Y/Z are held fixed. Only RPY orientation targets change.

current = rowVector(currentPose, 6, 'currentPose');
deltaB = rowVector(poseBOrientationDeltaDeg, 3, ...
    'poseBOrientationDeltaDeg');
deltaC = rowVector(poseCOrientationDeltaDeg, 3, ...
    'poseCOrientationDeltaDeg');
if ~isscalar(maximumOrientationTransitionDeg) || ...
        ~isfinite(maximumOrientationTransitionDeg) || ...
        maximumOrientationTransitionDeg <= 0
    error('scopeguide:diagnostics:InvalidMaximumTransition', ...
        ['maximumOrientationTransitionDeg must be a finite positive ' ...
         'scalar.']);
end

targets = repmat(current, 4, 1);
targets(2, 4:6) = wrapDegrees(current(4:6) + deltaB);
targets(3, 4:6) = wrapDegrees(current(4:6) + deltaC);

transitionRpyDelta = zeros(3, 3);
maximumByTransition = zeros(3, 1);
for index = 1:3
    transitionRpyDelta(index, :) = wrappedDifferenceDeg( ...
        targets(index + 1, 4:6), targets(index, 4:6));
    rotationA = scopeguide.geometry.dobotRpyToRotation( ...
        targets(index, 4:6), pi / 180);
    rotationB = scopeguide.geometry.dobotRpyToRotation( ...
        targets(index + 1, 4:6), pi / 180);
    maximumByTransition(index) = rad2deg( ...
        scopeguide.geometry.rotationDistance(rotationA, rotationB));
end
if any(maximumByTransition > maximumOrientationTransitionDeg)
    failed = find(maximumByTransition > ...
        maximumOrientationTransitionDeg, 1);
    error('scopeguide:diagnostics:TransitionTooLarge', ...
        ['Transition %d requests an orientation change of %.3f deg, ' ...
         'exceeding the configured %.3f deg limit.'], ...
        failed, maximumByTransition(failed), ...
        maximumOrientationTransitionDeg);
end

plan = struct();
plan.Labels = ["A_initial"; "B"; "C"; "A_return"];
plan.TargetCartesianPose = targets;
plan.TransitionRpyDeltaDeg = transitionRpyDelta;
plan.OrientationTransitionDeg = maximumByTransition;
plan.PoseBOrientationDeltaDeg = deltaB;
plan.PoseCOrientationDeltaDeg = deltaC;
plan.ControllerPoseReferenceAssumption = "flange";
end

function value = rowVector(input, count, name)
value = double(input(:).');
if numel(value) ~= count || any(~isfinite(value))
    error('scopeguide:diagnostics:InvalidCartesianVector', ...
        '%s must contain %d finite values.', name, count);
end
end

function value = wrapDegrees(value)
value = mod(double(value) + 180, 360) - 180;
end

function difference = wrappedDifferenceDeg(target, actual)
difference = mod(double(target) - double(actual) + 180, 360) - 180;
end
