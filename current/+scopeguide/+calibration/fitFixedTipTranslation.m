function result = fitFixedTipTranslation( ...
        flangePositionBaseM, rotationBaseFromFlange, options)
%FITFIXEDTIPTRANSLATION Estimate flange-to-tip translation from pivot poses.
% Each input pose must observe the same physical tip point in the robot
% base frame.  The linear model is
%
%   pFixedBase = pFlangeBase_i + RBaseFlange_i * pTipFlange.
%
% The fixed-point experiment cannot identify the orientation of the tool
% frame.  RotationFlangeToEndoscope is therefore treated as a separately
% known value and copied into the returned homogeneous transform.

arguments
    flangePositionBaseM (:, 3) double
    rotationBaseFromFlange double
    options.RotationFlangeToEndoscope (3, 3) double = eye(3)
    options.MinimumOrientationSpanRad (1, 1) double = deg2rad(30)
    options.MaximumConditionNumber (1, 1) double = 1e4
    options.MaximumRmsResidualM (1, 1) double = 0.001
    options.MaximumResidualM (1, 1) double = 0.002
end

position = double(flangePositionBaseM);
rotation = double(rotationBaseFromFlange);
poseCount = size(position, 1);
if poseCount < 4
    error('scopeguide:calibration:InsufficientFixedTipPoses', ...
        'At least four fixed-tip poses are required.');
end
if any(~isfinite(position), 'all') || ...
        ndims(rotation) > 3 || size(rotation, 1) ~= 3 || ...
        size(rotation, 2) ~= 3 || size(rotation, 3) ~= poseCount || ...
        any(~isfinite(rotation), 'all')
    error('scopeguide:calibration:InvalidFixedTipPoseData', ...
        ['Positions must be finite N-by-3 values and rotations must be ' ...
         'finite 3-by-3-by-N values.']);
end
validateRotation(options.RotationFlangeToEndoscope, ...
    'RotationFlangeToEndoscope');
for index = 1:poseCount
    validateRotation(rotation(:, :, index), sprintf( ...
        'rotationBaseFromFlange(:,:,%.0f)', index));
end
validatePositive(options.MinimumOrientationSpanRad, ...
    'MinimumOrientationSpanRad');
validatePositive(options.MaximumConditionNumber, ...
    'MaximumConditionNumber');
validatePositive(options.MaximumRmsResidualM, ...
    'MaximumRmsResidualM');
validatePositive(options.MaximumResidualM, ...
    'MaximumResidualM');
if options.MaximumResidualM < options.MaximumRmsResidualM
    error('scopeguide:calibration:InvalidFixedTipResidualBounds', ...
        'MaximumResidualM must not be smaller than MaximumRmsResidualM.');
end

design = zeros(3 * poseCount, 6);
observation = zeros(3 * poseCount, 1);
for index = 1:poseCount
    rows = (3 * index - 2):(3 * index);
    design(rows, :) = [rotation(:, :, index), -eye(3)];
    observation(rows) = -position(index, :).';
end

singularValues = svd(design, 'econ');
rankTolerance = max(size(design)) * eps(max(singularValues));
designRank = sum(singularValues > rankTolerance);
if designRank < 6
    estimate = pinv(design) * observation;
    conditionNumber = Inf;
else
    estimate = design \ observation;
    conditionNumber = singularValues(1) / singularValues(end);
end

tipTranslationFlangeM = estimate(1:3);
fixedTipPointBaseM = estimate(4:6);
predictedTipPointBaseM = zeros(poseCount, 3);
for index = 1:poseCount
    predictedTipPointBaseM(index, :) = (position(index, :).' + ...
        rotation(:, :, index) * tipTranslationFlangeM).';
end
residualBaseM = predictedTipPointBaseM - fixedTipPointBaseM.';
residualNormM = vecnorm(residualBaseM, 2, 2);
rmsResidualM = sqrt(mean(residualNormM.^2));
maximumResidualM = max(residualNormM);
orientationSpanRad = maximumPairwiseRotationDistance(rotation);

failureReasons = strings(0, 1);
if designRank < 6
    failureReasons(end + 1, 1) = "DESIGN_MATRIX_RANK_DEFICIENT";
end
if ~isfinite(conditionNumber) || ...
        conditionNumber > options.MaximumConditionNumber
    failureReasons(end + 1, 1) = "DESIGN_MATRIX_ILL_CONDITIONED";
end
if orientationSpanRad < options.MinimumOrientationSpanRad
    failureReasons(end + 1, 1) = "ORIENTATION_SPAN_TOO_SMALL";
end
if rmsResidualM > options.MaximumRmsResidualM
    failureReasons(end + 1, 1) = "RMS_FIXED_POINT_RESIDUAL_TOO_LARGE";
end
if maximumResidualM > options.MaximumResidualM
    failureReasons(end + 1, 1) = ...
        "MAXIMUM_FIXED_POINT_RESIDUAL_TOO_LARGE";
end

transform = eye(4);
transform(1:3, 1:3) = options.RotationFlangeToEndoscope;
transform(1:3, 4) = tipTranslationFlangeM;

result = struct();
result.Valid = isempty(failureReasons);
result.FailureReasons = failureReasons;
result.PoseCount = poseCount;
result.DesignRank = designRank;
result.DesignSingularValues = singularValues;
result.DesignConditionNumber = conditionNumber;
result.OrientationSpanRad = orientationSpanRad;
result.MinimumRequiredOrientationSpanRad = ...
    options.MinimumOrientationSpanRad;
result.TipTranslationFlangeM = tipTranslationFlangeM;
result.FixedTipPointBaseM = fixedTipPointBaseM;
result.RotationFlangeToEndoscope = ...
    options.RotationFlangeToEndoscope;
result.RotationEstimated = false;
result.TFlangeEndoscope = transform;
result.PredictedTipPointBaseM = predictedTipPointBaseM;
result.ResidualBaseM = residualBaseM;
result.ResidualNormM = residualNormM;
result.RmsResidualM = rmsResidualM;
result.MaximumResidualM = maximumResidualM;
result.MaximumAllowedRmsResidualM = options.MaximumRmsResidualM;
result.MaximumAllowedResidualM = options.MaximumResidualM;
result.DoesNotEstimateToolRotation = true;
result.DoesNotSendHardwareCommands = true;
end

function distance = maximumPairwiseRotationDistance(rotations)
count = size(rotations, 3);
distance = 0;
for first = 1:(count - 1)
    for second = (first + 1):count
        candidate = scopeguide.geometry.rotationDistance( ...
            rotations(:, :, first), rotations(:, :, second));
        distance = max(distance, candidate);
    end
end
end

function validateRotation(rotation, name)
if any(~isfinite(rotation), 'all') || ...
        norm(rotation' * rotation - eye(3), 'fro') > 1e-6 || ...
        abs(det(rotation) - 1) > 1e-6
    error('scopeguide:calibration:InvalidFixedTipRotation', ...
        '%s must be a finite proper rotation matrix.', name);
end
end

function validatePositive(value, name)
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || value <= 0
    error('scopeguide:calibration:InvalidFixedTipOption', ...
        '%s must be a finite positive scalar.', name);
end
end
