function solution = solveRcmConstrainedQp(jointPositionRad, ...
        qdotPreviousRadSec, desiredTipTwistBase, ...
        relativeCoordinate, pRcmBaseM, dtSec, cfg, options)
%SOLVERCMCONSTRAINEDQP Pure Stage 6 quadprog solve with fail-closed output.
% Diagnostic overrides exist only to exercise timeout/exit-flag handling in
% offline tests; normal callers leave them as NaN.

arguments
    jointPositionRad
    qdotPreviousRadSec
    desiredTipTwistBase
    relativeCoordinate
    pRcmBaseM
    dtSec (1, 1) double
    cfg (1, 1) struct
    options.DiagnosticExitFlagOverride (1, 1) double = NaN
    options.DiagnosticSolveTimeOverrideSec (1, 1) double = NaN
end

validateRcmAdmittanceConfig(cfg);
solution = blankSolution(cfg);
[valid, q, qdotPrevious, desiredTwist, relative] = ...
    validateInputs(jointPositionRad, qdotPreviousRadSec, ...
    desiredTipTwistBase, relativeCoordinate, pRcmBaseM, dtSec, cfg);
if ~valid
    solution.FailureCode = "INVALID_QP_INPUT";
    return;
end
pRcm = double(pRcmBaseM(:));
if exist('quadprog', 'file') ~= 2 || ...
        ~licenseAvailable('Optimization_Toolbox')
    solution.FailureCode = "QUADPROG_UNAVAILABLE";
    return;
end

try
    geometry = scopeguide.geometry.computeRcmConstraintKinematics( ...
        q, pRcm, cfg);
catch exception
    if startsWith(string(exception.identifier), "scopeguide:")
        solution.FailureCode = "INVALID_RCM_GEOMETRY";
        solution.InputErrorIdentifier = string(exception.identifier);
        return;
    end
    rethrow(exception);
end
solution.Geometry = geometry;
solution.CurrentRcmErrorNormM = geometry.RcmErrorNormM;
if geometry.RcmErrorNormM > cfg.rcm.HardRadiusM + ...
        cfg.qp.ConstraintTolerance
    solution.FailureCode = "CURRENT_RCM_OUTSIDE_HARD_BOUND";
    return;
end
if norm(relative(1:2)) > cfg.control.RelativePivotLimitRad + ...
        cfg.qp.ConstraintTolerance || ...
        abs(relative(3)) > cfg.control.RelativeInsertionLimitM + ...
        cfg.qp.ConstraintTolerance
    solution.FailureCode = "CURRENT_RELATIVE_TRAVEL_OUTSIDE_BOUND";
    return;
end

taskScale = diag([1, 1, 1, ...
    cfg.qp.TaskCharacteristicLengthM * ones(1, 3)]);
scaledJacobian = taskScale * geometry.TipTwistJacobianBase;
singularValues = svd(scaledJacobian);
sigmaMinimum = min(singularValues);
sigmaMaximum = max(singularValues);
solution.ScaledJacobianSingularValues = singularValues;
solution.ScaledJacobianSigmaMinimum = sigmaMinimum;
solution.ScaledJacobianConditionNumber = ...
    sigmaMaximum / max(sigmaMinimum, eps);
singularityScale = smoothScale(sigmaMinimum, ...
    cfg.qp.SingularityStopSigma, ...
    cfg.qp.SingularityFullSpeedSigma);
solution.SingularityScale = singularityScale;
if singularityScale == 0
    solution.FailureCode = "JACOBIAN_SINGULAR_STOP";
    return;
end

[rcmWeight, recoveryGain] = rcmModeParameters( ...
    cfg.rcm.Mode, geometry.RcmErrorNormM, cfg);
solution.RcmMode = string(cfg.rcm.Mode);
solution.RcmObjectiveWeight = rcmWeight;
solution.RcmRecoveryGainSecInv = recoveryGain;
desiredScaled = singularityScale * desiredTwist;
recoveryTarget = singularityScale * recoveryGain * ...
    geometry.RcmError2M;

taskMatrix = sqrt(cfg.qp.TaskTrackingWeight) * ...
    taskScale * geometry.TipTwistJacobianBase;
taskTarget = sqrt(cfg.qp.TaskTrackingWeight) * ...
    taskScale * desiredScaled;
objectiveMatrix = taskMatrix;
objectiveTarget = taskTarget;
if string(cfg.rcm.Mode) ~= "hard"
    objectiveMatrix = [objectiveMatrix; ...
        sqrt(rcmWeight) * geometry.RcmLateralJacobian];
    objectiveTarget = [objectiveTarget; ...
        sqrt(rcmWeight) * recoveryTarget];
end
qdotCentering = -cfg.qp.JointCenteringGainSecInv * q;
qdotCentering = limitNorm(qdotCentering, ...
    cfg.qp.JointCenteringMaximumRadSec);
qdotCentering = singularityScale * qdotCentering;
objectiveMatrix = [objectiveMatrix; ...
    sqrt(cfg.qp.JointVelocityRegularization) * eye(6)];
objectiveTarget = [objectiveTarget; ...
    sqrt(cfg.qp.JointVelocityRegularization) * qdotCentering];
H = objectiveMatrix' * objectiveMatrix;
H = (H + H') / 2;
f = -objectiveMatrix' * objectiveTarget;

[lowerBound, upperBound, boundCode] = jointVelocityBounds( ...
    q, qdotPrevious, dtSec, cfg);
solution.JointVelocityLowerBoundRadSec = lowerBound;
solution.JointVelocityUpperBoundRadSec = upperBound;
if strlength(boundCode) > 0
    solution.FailureCode = boundCode;
    return;
end
[A, b, constraintGroups] = inequalityConstraints( ...
    geometry, relative, dtSec, cfg);
if string(cfg.rcm.Mode) == "hard"
    Aeq = geometry.RcmLateralJacobian;
    beq = recoveryTarget;
else
    Aeq = zeros(0, 6);
    beq = zeros(0, 1);
end
solution.InequalityConstraintGroups = constraintGroups;

qpOptions = optimoptions('quadprog', 'Display', 'off', ...
    'Algorithm', 'interior-point-convex', ...
    'ConstraintTolerance', cfg.qp.ConstraintTolerance, ...
    'OptimalityTolerance', cfg.qp.OptimalityTolerance, ...
    'MaxIterations', cfg.qp.MaximumIterations);
startTime = tic;
try
    [qdot, objectiveValue, exitFlag, qpOutput] = quadprog( ...
        H, f, A, b, Aeq, beq, lowerBound, upperBound, [], qpOptions);
    solveTimeSec = toc(startTime);
catch exception
    solveTimeSec = toc(startTime);
    solution.SolveTimeSec = solveTimeSec;
    solution.FailureCode = "QUADPROG_EXCEPTION";
    solution.InputErrorIdentifier = string(exception.identifier);
    return;
end
if isfinite(options.DiagnosticExitFlagOverride)
    exitFlag = options.DiagnosticExitFlagOverride;
end
if isfinite(options.DiagnosticSolveTimeOverrideSec)
    solveTimeSec = options.DiagnosticSolveTimeOverrideSec;
end
solution.SolveTimeSec = solveTimeSec;
solution.ExitFlag = exitFlag;
solution.ObjectiveValue = objectiveValue;
solution.Iterations = getIterations(qpOutput);
if solveTimeSec > cfg.qp.MaximumSolveTimeSec
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

diagnostics = constraintDiagnostics(qdot, q, qdotPrevious, ...
    relative, geometry, A, b, Aeq, beq, dtSec, cfg);
solution.ConstraintDiagnostics = diagnostics;
if diagnostics.MaximumViolation > cfg.qp.ConstraintTolerance
    solution.FailureCode = "QP_CONSTRAINT_VIOLATION";
    return;
end

solution.QdotCommandRadSec = qdot;
solution.AchievedTipTwistBase = ...
    geometry.TipTwistJacobianBase * qdot;
solution.AchievedGeneralizedVelocity = ...
    geometry.GeneralizedRateJacobian * qdot;
solution.PredictedJointPositionRad = q + qdot * dtSec;
solution.PredictedRelativeCoordinate = relative + ...
    solution.AchievedGeneralizedVelocity * dtSec;
solution.PredictedRcmError2M = geometry.RcmError2M - ...
    geometry.RcmLateralJacobian * qdot * dtSec;
solution.PredictedRcmErrorNormM = ...
    norm(solution.PredictedRcmError2M);
solution.MotionCommandValid = true;
solution.QpHealthy = true;
solution.FailureCode = "";
solution.StatusCode = "QP_SOLVED";
end

function solution = blankSolution(cfg)
solution = struct();
solution.StatusCode = "QP_FAILED_ZERO_OUTPUT";
solution.FailureCode = "UNASSESSED";
solution.MotionCommandValid = false;
solution.QpHealthy = false;
solution.QdotCommandRadSec = zeros(6, 1);
solution.AchievedTipTwistBase = zeros(6, 1);
solution.AchievedGeneralizedVelocity = zeros(3, 1);
solution.PredictedJointPositionRad = nan(6, 1);
solution.PredictedRelativeCoordinate = nan(3, 1);
solution.PredictedRcmError2M = nan(2, 1);
solution.PredictedRcmErrorNormM = NaN;
solution.CurrentRcmErrorNormM = NaN;
solution.SolveTimeSec = NaN;
solution.ExitFlag = NaN;
solution.Iterations = NaN;
solution.ObjectiveValue = NaN;
solution.RcmMode = string(cfg.rcm.Mode);
solution.RcmObjectiveWeight = NaN;
solution.RcmRecoveryGainSecInv = NaN;
solution.SingularityScale = 0;
solution.ScaledJacobianSingularValues = nan(6, 1);
solution.ScaledJacobianSigmaMinimum = NaN;
solution.ScaledJacobianConditionNumber = Inf;
solution.JointVelocityLowerBoundRadSec = nan(6, 1);
solution.JointVelocityUpperBoundRadSec = nan(6, 1);
solution.InequalityConstraintGroups = struct();
solution.ConstraintDiagnostics = struct();
solution.Geometry = struct([]);
solution.InputErrorIdentifier = "";
solution.Solver = "quadprog";
solution.NoFallbackSolverUsed = true;
solution.DoesNotSendHardwareCommands = true;
end

function [valid, q, previous, twist, relative] = validateInputs( ...
        qInput, previousInput, twistInput, relativeInput, pRcm, dtSec, cfg)
q = double(qInput(:));
previous = double(previousInput(:));
twist = double(twistInput(:));
relative = double(relativeInput(:));
valid = numel(q) == 6 && all(isfinite(q)) && ...
    all(q >= double(cfg.kinematics.JointLowerLimitRad(:))) && ...
    all(q <= double(cfg.kinematics.JointUpperLimitRad(:))) && ...
    numel(previous) == 6 && all(isfinite(previous)) && ...
    numel(twist) == 6 && all(isfinite(twist)) && ...
    numel(relative) == 3 && all(isfinite(relative)) && ...
    isnumeric(pRcm) && numel(pRcm) == 3 && ...
    all(isfinite(double(pRcm(:)))) && isfinite(dtSec) && dtSec > 0 && ...
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

function [weight, gain] = rcmModeParameters(mode, errorNorm, cfg)
mode = string(mode);
if mode == "hard"
    weight = 0;
    gain = cfg.qp.RcmRecoveryGainMaximumSecInv;
elseif mode == "soft"
    weight = cfg.qp.SoftRcmWeight;
    gain = cfg.qp.RcmRecoveryGainMinimumSecInv;
else
    normalized = min(max((errorNorm - cfg.rcm.SoftRadiusM) / ...
        (cfg.rcm.HardRadiusM - cfg.rcm.SoftRadiusM), 0), 1);
    blend = normalized^2 * (3 - 2 * normalized);
    weight = cfg.qp.HybridRcmWeightMinimum + blend * ...
        (cfg.qp.HybridRcmWeightMaximum - ...
        cfg.qp.HybridRcmWeightMinimum);
    gain = cfg.qp.RcmRecoveryGainMinimumSecInv + blend * ...
        (cfg.qp.RcmRecoveryGainMaximumSecInv - ...
        cfg.qp.RcmRecoveryGainMinimumSecInv);
end
end

function limited = limitNorm(vector, maximumNorm)
vectorNorm = norm(vector);
if vectorNorm > maximumNorm
    limited = vector * maximumNorm / vectorNorm;
else
    limited = vector;
end
end

function [lower, upper, code] = jointVelocityBounds(q, previous, dtSec, cfg)
maximumVelocity = cfg.control.MaximumJointCommandRadSec(:);
maximumAcceleration = cfg.control.MaximumJointAccelerationRadSec2(:);
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

function [A, b, groups] = inequalityConstraints( ...
        geometry, relative, dtSec, cfg)
[rcmNormals, rcmRadius] = inscribedPolygon( ...
    cfg.qp.HardConstraintPolygonSides, cfg.rcm.HardRadiusM);
rcmA = -dtSec * rcmNormals' * geometry.RcmLateralJacobian;
rcmB = rcmRadius - rcmNormals' * geometry.RcmError2M;

[pivotNormals, pivotRadius] = inscribedPolygon( ...
    cfg.qp.RelativePivotPolygonSides, ...
    cfg.control.RelativePivotLimitRad);
pivotJacobian = geometry.GeneralizedRateJacobian(1:2, :);
pivotA = dtSec * pivotNormals' * pivotJacobian;
pivotB = pivotRadius - pivotNormals' * relative(1:2);
insertionJacobian = geometry.GeneralizedRateJacobian(3, :);
insertionA = dtSec * [insertionJacobian; -insertionJacobian];
insertionB = [cfg.control.RelativeInsertionLimitM - relative(3); ...
    cfg.control.RelativeInsertionLimitM + relative(3)];
A = [rcmA; pivotA; insertionA];
b = [rcmB; pivotB; insertionB];
groups = struct('RcmRows', size(rcmA, 1), ...
    'RelativePivotRows', size(pivotA, 1), ...
    'RelativeInsertionRows', size(insertionA, 1));
end

function [normals, radius] = inscribedPolygon(sideCount, circleRadius)
angles = 2 * pi * (0:(sideCount - 1)) / sideCount;
normals = [cos(angles); sin(angles)];
radius = circleRadius * cos(pi / sideCount);
end

function count = getIterations(output)
if isstruct(output) && isfield(output, 'iterations') && ...
        isfinite(output.iterations)
    count = double(output.iterations);
else
    count = NaN;
end
end

function diagnostics = constraintDiagnostics(qdot, q, previous, ...
        relative, geometry, A, b, Aeq, beq, dtSec, cfg)
inequalityViolation = max([A * qdot - b; 0]);
if isempty(Aeq)
    equalityViolation = 0;
else
    equalityViolation = norm(Aeq * qdot - beq, inf);
end
speedViolation = max([abs(qdot) - ...
    cfg.control.MaximumJointCommandRadSec(:); 0]);
accelerationViolation = max([abs(qdot - previous) - ...
    cfg.control.MaximumJointAccelerationRadSec2(:) * dtSec; 0]);
[positionLower, positionUpper] = jointPositionBounds(cfg);
predictedPosition = q + qdot * dtSec;
positionViolation = max([positionLower - predictedPosition; ...
    predictedPosition - positionUpper; 0]);
predictedRelative = relative + ...
    geometry.GeneralizedRateJacobian * qdot * dtSec;
relativeViolation = max([ ...
    norm(predictedRelative(1:2)) - cfg.control.RelativePivotLimitRad; ...
    abs(predictedRelative(3)) - cfg.control.RelativeInsertionLimitM; 0]);
predictedRcmError = geometry.RcmError2M - ...
    geometry.RcmLateralJacobian * qdot * dtSec;
rcmViolation = max(norm(predictedRcmError) - cfg.rcm.HardRadiusM, 0);
diagnostics = struct();
diagnostics.MaximumInequalityViolation = inequalityViolation;
diagnostics.MaximumEqualityViolation = equalityViolation;
diagnostics.MaximumSpeedViolation = speedViolation;
diagnostics.MaximumAccelerationViolation = accelerationViolation;
diagnostics.MaximumPositionViolation = positionViolation;
diagnostics.MaximumRelativeTravelViolation = relativeViolation;
diagnostics.MaximumRcmViolation = rcmViolation;
diagnostics.MaximumViolation = max([inequalityViolation; ...
    equalityViolation; speedViolation; accelerationViolation; ...
    positionViolation; relativeViolation; rcmViolation]);
diagnostics.MinimumInequalityMargin = min(b - A * qdot);
end

function [lower, upper] = jointPositionBounds(cfg)
margin = max(cfg.qp.JointPositionMarginRad, ...
    cfg.stage08.JointLimitMarginRad);
lower = double(cfg.kinematics.JointLowerLimitRad(:)) + margin;
upper = double(cfg.kinematics.JointUpperLimitRad(:)) - margin;
end
