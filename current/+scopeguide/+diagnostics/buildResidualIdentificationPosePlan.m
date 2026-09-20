function plan = buildResidualIdentificationPosePlan( ...
        currentCartesianPose, orientationOffsetsDeg, trainingIndices, ...
        validationIndices, maximumTransitionDeg)
%BUILDRESIDUALIDENTIFICATIONPOSEPLAN Build fixed train/validation pose plan.

current = double(currentCartesianPose(:).');
offsets = double(orientationOffsetsDeg);
if numel(current) ~= 6 || any(~isfinite(current)) || ...
        size(offsets, 2) ~= 3 || size(offsets, 1) < 9 || ...
        any(~isfinite(offsets), 'all') || any(abs(offsets(1, :)) > 1e-12)
    error('scopeguide:residualPlan:InvalidPoseDefinition', ...
        ['Current pose must have six finite values; offsets must contain ' ...
         'at least nine finite rows and start with [0 0 0].']);
end
uniquePoseCount = size(offsets, 1);
training = validateIndices(trainingIndices, uniquePoseCount, 'trainingIndices');
validation = validateIndices( ...
    validationIndices, uniquePoseCount, 'validationIndices');
if numel(training) < 6 || numel(validation) < 3 || ...
        ~isempty(intersect(training, validation)) || ...
        ~isequal(sort([training validation]), 1:uniquePoseCount)
    error('scopeguide:residualPlan:InvalidSplit', ...
        ['Training and validation indices must be disjoint, cover every ' ...
         'unique pose, and contain at least six/three poses respectively.']);
end
if ~isscalar(maximumTransitionDeg) || ...
        ~isfinite(maximumTransitionDeg) || maximumTransitionDeg <= 0
    error('scopeguide:residualPlan:InvalidTransitionLimit', ...
        'maximumTransitionDeg must be finite and positive.');
end

targets = repmat(current, uniquePoseCount + 1, 1);
for index = 1:uniquePoseCount
    targets(index, 4:6) = wrapDegrees(current(4:6) + offsets(index, :));
end
targets(end, :) = current;

transitionDeg = zeros(uniquePoseCount, 1);
for index = 1:uniquePoseCount
    rotationA = scopeguide.geometry.dobotRpyToRotation( ...
        targets(index, 4:6), pi / 180);
    rotationB = scopeguide.geometry.dobotRpyToRotation( ...
        targets(index + 1, 4:6), pi / 180);
    transitionDeg(index) = rad2deg( ...
        scopeguide.geometry.rotationDistance(rotationA, rotationB));
end
if any(transitionDeg > maximumTransitionDeg)
    failed = find(transitionDeg > maximumTransitionDeg, 1);
    error('scopeguide:residualPlan:TransitionTooLarge', ...
        ['Transition %d is %.3f deg, exceeding the configured %.3f ' ...
         'deg limit.'], failed, transitionDeg(failed), maximumTransitionDeg);
end

labels = strings(uniquePoseCount + 1, 1);
for index = 1:uniquePoseCount
    if ismember(index, training)
        role = "train";
    else
        role = "validation";
    end
    labels(index) = sprintf('P%02d_%s', index, role);
end
labels(end) = "A_return";

plan = struct();
plan.Labels = labels;
plan.TargetCartesianPose = targets;
plan.OrientationOffsetsDeg = [offsets; zeros(1, 3)];
plan.OrientationTransitionDeg = transitionDeg;
plan.TrainingIndices = training;
plan.ValidationIndices = validation;
plan.UniquePoseCount = uniquePoseCount;
plan.ReturnPoseIndex = uniquePoseCount + 1;
plan.ControllerPoseReferenceAssumption = "flange";
end

function indices = validateIndices(value, maximum, name)
indices = double(value(:).');
if isempty(indices) || any(~isfinite(indices)) || ...
        any(indices ~= round(indices)) || ...
        any(indices < 1 | indices > maximum) || ...
        numel(unique(indices)) ~= numel(indices)
    error('scopeguide:residualPlan:InvalidIndices', ...
        '%s contains invalid indices.', name);
end
end

function value = wrapDegrees(value)
value = mod(double(value) + 180, 360) - 180;
end
