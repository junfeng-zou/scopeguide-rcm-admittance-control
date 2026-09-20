function solution = solveCartesianVelocityQp(jointPositionRad, ...
        qdotPreviousRadSec, desiredTwistBase, relativeCoordinate, ...
        rotationBaseFromAnchor, dtSec, cfg, options)
%SOLVECARTESIANVELOCITYQP Joint velocity QP without any RCM constraint.

arguments
    jointPositionRad
    qdotPreviousRadSec
    desiredTwistBase
    relativeCoordinate
    rotationBaseFromAnchor
    dtSec (1, 1) double
    cfg (1, 1) struct
    options.DiagnosticExitFlagOverride (1, 1) double = NaN
    options.DiagnosticSolveTimeOverrideSec (1, 1) double = NaN
end

validateCartesianAdmittanceDragConfig(cfg);
solution = blankSolution();
[valid, q, previous, desired, relative, anchorRotation] = ...
    validateInputs(jointPositionRad, qdotPreviousRadSec, ...
    desiredTwistBase, relativeCoordinate, rotationBaseFromAnchor, ...
    dtSec, cfg);
if ~valid
    solution.FailureCode = "INVALID_QP_INPUT";
    return;
end
if exist('quadprog', 'file') ~= 2 || ...
        ~licenseAvailable('Optimization_Toolbox')
    solution.FailureCode = "QUADPROG_UNAVAILABLE";
    return;
end

travelLimitsEnabled = logical(cfg.cartesianDrag.TravelLimitsEnabled);
limits = [double(cfg.cartesianDrag.Translation.RelativeLimitM(:)); ...
    double(cfg.cartesianDrag.Rotation.RelativeLimitRad(:))];
recoveryTolerance = [ ...
    cfg.cartesianDrag.Translation.RecoveryToleranceM * ones(3, 1); ...
    cfg.cartesianDrag.Rotation.RecoveryToleranceRad * ones(3, 1)];
hardLimits = limits + recoveryTolerance;
solution.TravelLimitsEnabled = travelLimitsEnabled;
if travelLimitsEnabled
    solution.SoftRelativeLimit = limits;
    solution.RecoveryTolerance = recoveryTolerance;
    solution.HardRelativeLimit = hardLimits;
    solution.RecoveryActiveAxes = abs(relative) > limits;
    solution.TravelRecoveryActive = any(solution.RecoveryActiveAxes);
    if any(abs(relative) > hardLimits + cfg.qp.ConstraintTolerance)
        solution.FailureCode = ...
            "CURRENT_CARTESIAN_TRAVEL_OUTSIDE_HARD_BOUND";
        return;
    end
end
try
    geometry = scopeguide.geometry.computeDragPointKinematics(q, cfg);
catch exception
    if startsWith(string(exception.identifier), "scopeguide:")
        solution.FailureCode = "INVALID_DRAG_POINT_GEOMETRY";
        solution.InputErrorIdentifier = string(exception.identifier);
        return;
    end
    rethrow(exception);
end
solution.Geometry = geometry;
J = geometry.DragPointTwistJacobianBase;
lengthScale = cfg.cartesianDrag.Qp.TaskCharacteristicLengthM;
taskScale = diag([1, 1, 1, lengthScale, lengthScale, lengthScale]);
scaledJacobian = taskScale * J;
singularValues = svd(scaledJacobian);
sigmaMinimum = min(singularValues);
sigmaMaximum = max(singularValues);
solution.ScaledJacobianSingularValues = singularValues;
solution.ScaledJacobianSigmaMinimum = sigmaMinimum;
solution.ScaledJacobianConditionNumber = ...
    sigmaMaximum / max(sigmaMinimum, eps);
singularityScale = smoothScale(sigmaMinimum, ...
    cfg.cartesianDrag.Qp.SingularityStopSigma, ...
    cfg.cartesianDrag.Qp.SingularityFullSpeedSigma);
solution.SingularityScale = singularityScale;
if singularityScale == 0
    solution.FailureCode = "JACOBIAN_SINGULAR_STOP";
    return;
end

qCenter = 0.5 * (double(cfg.kinematics.JointLowerLimitRad(:)) + ...
    double(cfg.kinematics.JointUpperLimitRad(:)));
qdotCentering = -cfg.cartesianDrag.Qp.JointCenteringGainSecInv * ...
    (q - qCenter);
qdotCentering = limitNorm(qdotCentering, ...
    cfg.cartesianDrag.Qp.JointCenteringMaximumRadSec);
qdotCentering = singularityScale * qdotCentering;

taskWeight = cfg.cartesianDrag.Qp.TaskTrackingWeight;
regularization = cfg.cartesianDrag.Qp.JointVelocityRegularization;
smoothing = cfg.cartesianDrag.Qp.CommandSmoothingWeight;
objectiveMatrix = sqrt(taskWeight) * taskScale * J;
objectiveTarget = sqrt(taskWeight) * taskScale * ...
    (singularityScale * desired);
if regularization > 0
    objectiveMatrix = [objectiveMatrix; ...
        sqrt(regularization) * eye(6)];
    objectiveTarget = [objectiveTarget; ...
        sqrt(regularization) * qdotCentering];
end
if smoothing > 0
    objectiveMatrix = [objectiveMatrix; sqrt(smoothing) * eye(6)];
    objectiveTarget = [objectiveTarget; sqrt(smoothing) * previous];
end
H = objectiveMatrix' * objectiveMatrix;
H = (H + H') / 2;
f = -objectiveMatrix' * objectiveTarget;

[lower, upper, boundCode] = jointVelocityBounds( ...
    q, previous, dtSec, cfg);
solution.JointVelocityLowerBoundRadSec = lower;
solution.JointVelocityUpperBoundRadSec = upper;
if strlength(boundCode) > 0
    solution.FailureCode = boundCode;
    return;
end
anchorProjection = blkdiag(anchorRotation', anchorRotation');
JAnchor = anchorProjection * J;
if travelLimitsEnabled
    relativeLowerBound = -limits;
    relativeUpperBound = limits;
    aboveNormalRange = relative > limits;
    belowNormalRange = relative < -limits;
    % Inside the recovery band, keep the bound at the current feedback
    % pose. This blocks outward velocity while retaining inward return.
    relativeUpperBound(aboveNormalRange) = relative(aboveNormalRange);
    relativeLowerBound(belowNormalRange) = relative(belowNormalRange);
    A = dtSec * [JAnchor; -JAnchor];
    b = [relativeUpperBound - relative; ...
        relative - relativeLowerBound];
else
    % No Cartesian pose/travel inequality is passed to quadprog. Joint,
    % velocity, acceleration, singularity and ServoJ protections remain.
    relativeLowerBound = nan(6, 1);
    relativeUpperBound = nan(6, 1);
    A = zeros(0, 6);
    b = zeros(0, 1);
end
solution.RelativeTwistJacobianAnchor = JAnchor;
solution.ActiveRelativeLowerBound = relativeLowerBound;
solution.ActiveRelativeUpperBound = relativeUpperBound;

qpOptions = optimoptions('quadprog', 'Display', 'off', ...
    'Algorithm', 'interior-point-convex', ...
    'ConstraintTolerance', cfg.qp.ConstraintTolerance, ...
    'OptimalityTolerance', cfg.qp.OptimalityTolerance, ...
    'MaxIterations', cfg.qp.MaximumIterations);
start = tic;
try
    [qdot, objectiveValue, exitFlag, qpOutput] = quadprog( ...
        H, f, A, b, [], [], lower, upper, [], qpOptions);
    solveTime = toc(start);
catch exception
    solution.SolveTimeSec = toc(start);
    solution.FailureCode = "QUADPROG_EXCEPTION";
    solution.InputErrorIdentifier = string(exception.identifier);
    return;
end
if isfinite(options.DiagnosticExitFlagOverride)
    exitFlag = options.DiagnosticExitFlagOverride;
end
if isfinite(options.DiagnosticSolveTimeOverrideSec)
    solveTime = options.DiagnosticSolveTimeOverrideSec;
end
solution.SolveTimeSec = solveTime;
solution.ExitFlag = exitFlag;
solution.ObjectiveValue = objectiveValue;
solution.Iterations = iterationCount(qpOutput);
if solveTime > cfg.qp.MaximumSolveTimeSec
    solution.FailureCode = "QP_TIMEOUT";
    return;
end
if exitFlag <= 0 || isempty(qdot) || any(~isfinite(qdot))
    if exitFlag == -2
        solution.FailureCode = "QP_INFEASIBLE";
    else
        solution.FailureCode = "QP_ABNORMAL_EXIT";
    end
    return;
end

diagnostics = constraintDiagnostics(qdot, q, previous, relative, ...
    JAnchor, A, b, dtSec, limits, hardLimits, ...
    relativeLowerBound, relativeUpperBound, travelLimitsEnabled, cfg);
solution.ConstraintDiagnostics = diagnostics;
if diagnostics.MaximumViolation > cfg.qp.ConstraintTolerance
    solution.FailureCode = "QP_CONSTRAINT_VIOLATION";
    return;
end
solution.QdotCommandRadSec = qdot;
solution.AchievedTwistBase = J * qdot;
solution.AchievedTwistAnchor = JAnchor * qdot;
solution.PredictedJointPositionRad = q + qdot * dtSec;
solution.PredictedRelativeCoordinate = relative + ...
    solution.AchievedTwistAnchor * dtSec;
solution.MotionCommandValid = true;
solution.QpHealthy = true;
solution.FailureCode = "";
if ~travelLimitsEnabled
    solution.StatusCode = "QP_SOLVED_NO_RCM_NO_TRAVEL_LIMIT";
elseif solution.TravelRecoveryActive
    solution.StatusCode = "QP_SOLVED_TRAVEL_RECOVERY_NO_RCM";
else
    solution.StatusCode = "QP_SOLVED_NO_RCM";
end
end

function solution = blankSolution()
solution = struct();
solution.StatusCode = "QP_FAILED_ZERO_OUTPUT";
solution.FailureCode = "UNASSESSED";
solution.MotionCommandValid = false;
solution.QpHealthy = false;
solution.QdotCommandRadSec = zeros(6, 1);
solution.AchievedTwistBase = zeros(6, 1);
solution.AchievedTwistAnchor = zeros(6, 1);
solution.PredictedJointPositionRad = nan(6, 1);
solution.PredictedRelativeCoordinate = nan(6, 1);
solution.SolveTimeSec = NaN;
solution.ExitFlag = NaN;
solution.Iterations = NaN;
solution.ObjectiveValue = NaN;
solution.SingularityScale = 0;
solution.ScaledJacobianSingularValues = nan(6, 1);
solution.ScaledJacobianSigmaMinimum = NaN;
solution.ScaledJacobianConditionNumber = Inf;
solution.JointVelocityLowerBoundRadSec = nan(6, 1);
solution.JointVelocityUpperBoundRadSec = nan(6, 1);
solution.RelativeTwistJacobianAnchor = nan(6, 6);
solution.SoftRelativeLimit = nan(6, 1);
solution.RecoveryTolerance = nan(6, 1);
solution.HardRelativeLimit = nan(6, 1);
solution.ActiveRelativeLowerBound = nan(6, 1);
solution.ActiveRelativeUpperBound = nan(6, 1);
solution.RecoveryActiveAxes = false(6, 1);
solution.TravelRecoveryActive = false;
solution.TravelLimitsEnabled = false;
solution.ConstraintDiagnostics = struct();
solution.Geometry = struct([]);
solution.InputErrorIdentifier = "";
solution.Solver = "quadprog";
solution.NoFallbackSolverUsed = true;
solution.NoRcmConstraint = true;
solution.DoesNotSendHardwareCommands = true;
end

function [valid, q, previous, desired, relative, rotation] = ...
        validateInputs(qInput, previousInput, desiredInput, ...
        relativeInput, rotationInput, dtSec, cfg)
q = double(qInput(:));
previous = double(previousInput(:));
desired = double(desiredInput(:));
relative = double(relativeInput(:));
rotation = double(rotationInput);
valid = numel(q) == 6 && all(isfinite(q)) && ...
    all(q >= double(cfg.kinematics.JointLowerLimitRad(:))) && ...
    all(q <= double(cfg.kinematics.JointUpperLimitRad(:))) && ...
    numel(previous) == 6 && all(isfinite(previous)) && ...
    numel(desired) == 6 && all(isfinite(desired)) && ...
    numel(relative) == 6 && all(isfinite(relative)) && ...
    isequal(size(rotation), [3, 3]) && all(isfinite(rotation(:))) && ...
    norm(rotation' * rotation - eye(3), 'fro') <= 1e-8 && ...
    abs(det(rotation) - 1) <= 1e-8 && ...
    isfinite(dtSec) && dtSec > 0 && ...
    dtSec <= cfg.runtime.MaximumDtSec;
end

function available = licenseAvailable(feature)
try
    available = logical(license('test', feature));
catch
    available = false;
end
end

function scale = smoothScale(value, lower, upper)
normalized = min(max((value - lower) / (upper - lower), 0), 1);
scale = normalized^2 * (3 - 2 * normalized);
end

function limited = limitNorm(value, maximumNorm)
current = norm(value);
if current > maximumNorm
    limited = value * maximumNorm / current;
else
    limited = value;
end
end

function [lower, upper, code] = jointVelocityBounds(q, previous, dtSec, cfg)
maximumVelocity = double(cfg.control.MaximumJointCommandRadSec(:));
maximumAcceleration = ...
    double(cfg.control.MaximumJointAccelerationRadSec2(:));
[positionLower, positionUpper] = jointPositionBounds(cfg);
lower = max([-maximumVelocity, ...
    previous - maximumAcceleration * dtSec, ...
    (positionLower - q) / dtSec], [], 2);
upper = min([maximumVelocity, ...
    previous + maximumAcceleration * dtSec, ...
    (positionUpper - q) / dtSec], [], 2);
code = "";
if any(lower > upper)
    code = "JOINT_BOUNDS_INFEASIBLE";
end
end

function [lower, upper] = jointPositionBounds(cfg)
margin = max(cfg.qp.JointPositionMarginRad, ...
    cfg.stage08.JointLimitMarginRad);
lower = double(cfg.kinematics.JointLowerLimitRad(:)) + margin;
upper = double(cfg.kinematics.JointUpperLimitRad(:)) - margin;
end

function count = iterationCount(output)
if isstruct(output) && isfield(output, 'iterations') && ...
        isfinite(output.iterations)
    count = double(output.iterations);
else
    count = NaN;
end
end

function diagnostics = constraintDiagnostics(qdot, q, previous, ...
        relative, JAnchor, A, b, dtSec, softLimits, hardLimits, ...
        relativeLowerBound, relativeUpperBound, ...
        travelLimitsEnabled, cfg)
inequalityViolation = max([A * qdot - b; 0]);
speedViolation = max([abs(qdot) - ...
    cfg.control.MaximumJointCommandRadSec(:); 0]);
accelerationViolation = max([abs(qdot - previous) - ...
    cfg.control.MaximumJointAccelerationRadSec2(:) * dtSec; 0]);
[positionLower, positionUpper] = jointPositionBounds(cfg);
predictedPosition = q + qdot * dtSec;
positionViolation = max([positionLower - predictedPosition; ...
    predictedPosition - positionUpper; 0]);
predictedRelative = relative + JAnchor * qdot * dtSec;
if travelLimitsEnabled
    activeRelativeViolation = max([ ...
        relativeLowerBound - predictedRelative; ...
        predictedRelative - relativeUpperBound; 0]);
    softRelativeExcursion = max([ ...
        abs(predictedRelative) - softLimits; 0]);
    hardRelativeViolation = max([ ...
        abs(predictedRelative) - hardLimits; 0]);
    minimumInequalityMargin = min(b - A * qdot);
else
    activeRelativeViolation = 0;
    softRelativeExcursion = 0;
    hardRelativeViolation = 0;
    minimumInequalityMargin = NaN;
end
diagnostics = struct();
diagnostics.MaximumInequalityViolation = inequalityViolation;
diagnostics.MaximumSpeedViolation = speedViolation;
diagnostics.MaximumAccelerationViolation = accelerationViolation;
diagnostics.MaximumPositionViolation = positionViolation;
diagnostics.MaximumRelativeTravelViolation = activeRelativeViolation;
diagnostics.MaximumSoftTravelExcursion = softRelativeExcursion;
diagnostics.MaximumHardTravelViolation = hardRelativeViolation;
diagnostics.MaximumViolation = max([inequalityViolation; speedViolation; ...
    accelerationViolation; positionViolation; ...
    activeRelativeViolation; hardRelativeViolation]);
diagnostics.MinimumInequalityMargin = minimumInequalityMargin;
diagnostics.TravelLimitsEnabled = travelLimitsEnabled;
diagnostics.NoRcmConstraint = true;
end
