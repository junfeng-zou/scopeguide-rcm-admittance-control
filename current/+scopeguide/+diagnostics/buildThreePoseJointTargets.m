function plan = buildThreePoseJointTargets( ...
        currentJointDeg, poseBDeltaDeg, poseCDeltaDeg, maximumTransitionDeg)
%BUILDTHREEPOSEJOINTTARGETS Build guarded A/B/C/A joint-space targets.

current = rowSix(currentJointDeg, 'currentJointDeg');
deltaB = rowSix(poseBDeltaDeg, 'poseBDeltaDeg');
deltaC = rowSix(poseCDeltaDeg, 'poseCDeltaDeg');
if ~isscalar(maximumTransitionDeg) || ~isfinite(maximumTransitionDeg) || ...
        maximumTransitionDeg <= 0
    error('scopeguide:diagnostics:InvalidMaximumTransition', ...
        'maximumTransitionDeg must be a finite positive scalar.');
end

targets = [current; current + deltaB; current + deltaC; current];
transition = zeros(3, 6);
for index = 1:3
    transition(index, :) = wrappedDifferenceDeg( ...
        targets(index + 1, :), targets(index, :));
end
maximumByTransition = max(abs(transition), [], 2);
if any(maximumByTransition > maximumTransitionDeg)
    failed = find(maximumByTransition > maximumTransitionDeg, 1);
    error('scopeguide:diagnostics:TransitionTooLarge', ...
        ['Transition %d requests a maximum joint change of %.3f deg, ' ...
         'exceeding the configured %.3f deg limit.'], ...
        failed, maximumByTransition(failed), maximumTransitionDeg);
end

plan = struct();
plan.Labels = ["A_initial"; "B"; "C"; "A_return"];
plan.TargetJointDeg = targets;
plan.TransitionDeltaDeg = transition;
plan.MaximumTransitionDeg = maximumByTransition;
plan.PoseBDeltaDeg = deltaB;
plan.PoseCDeltaDeg = deltaC;
end

function value = rowSix(input, name)
value = double(input(:).');
if numel(value) ~= 6 || any(~isfinite(value))
    error('scopeguide:diagnostics:InvalidJointVector', ...
        '%s must contain six finite values.', name);
end
end

function difference = wrappedDifferenceDeg(target, actual)
difference = mod(double(target) - double(actual) + 180, 360) - 180;
end
