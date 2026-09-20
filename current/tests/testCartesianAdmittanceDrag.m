function tests = testCartesianAdmittanceDrag
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
addpath(fullfile(root, 'config'));
addpath(fullfile(root, 'robot'));
testCase.TestData.Root = root;
end

function testEditableConfigurationAndDerivedParameters(testCase)
[cfg, profile] = current_cartesian_admittance_drag_config();
verifyWarningFree(testCase, ...
    @() validateCartesianAdmittanceDragConfig(cfg));
parameters = scopeguide.control.deriveCartesianAdmittanceParameters(cfg);
verifyEqual(testCase, cfg.cartesianDrag.DofMode, "translation_z");
verifyEqual(testCase, cfg.cartesianDrag.TFlangeDragPoint(1:3, 4), ...
    cfg.tool.TFlangeSensor(1:3, 4), 'AbsTol', 0);
verifyEqual(testCase, parameters.Damping(1), ...
    cfg.cartesianDrag.Translation.DesignForceN / ...
    cfg.cartesianDrag.Translation.TargetSteadySpeedMSec, ...
    'RelTol', 1e-14);
verifyEqual(testCase, parameters.VirtualMass(1), ...
    parameters.Damping(1) * ...
    cfg.cartesianDrag.Translation.TimeConstantSec, 'RelTol', 1e-14);
verifyEqual(testCase, profile.TranslationDampingNSecPerM, ...
    parameters.Damping(1), 'AbsTol', 0);
verifyEqual(testCase, ...
    cfg.cartesianDrag.Translation.RecoveryToleranceM, 0.0010, ...
    'AbsTol', 0);
verifyEqual(testCase, ...
    cfg.cartesianDrag.Rotation.RecoveryToleranceRad, deg2rad(0.3), ...
    'AbsTol', 1e-15);
verifyEqual(testCase, cfg.safety.RawForceStopN, ...
    profile.RawForceStopN, 'AbsTol', 0);
verifyEqual(testCase, cfg.safety.FastForceStopN, ...
    profile.FastForceStopN, 'AbsTol', 0);
verifyEqual(testCase, cfg.safety.RawMomentStopNm, ...
    profile.RawMomentStopNm, 'AbsTol', 0);
verifyEqual(testCase, cfg.safety.FastMomentStopNm, ...
    profile.FastMomentStopNm, 'AbsTol', 0);
verifyEqual(testCase, ...
    cfg.cartesianDrag.SafetyThresholdConfirmationPhrase, ...
    profile.SafetyThresholdConfirmationPhrase);
verifyTrue(testCase, cfg.robot.ServoTimingVerified);
verifyFalse(testCase, cfg.cartesianDrag.TravelLimitsEnabled);
verifyFalse(testCase, parameters.TravelLimitsEnabled);
end

function testDofMasks(testCase)
verifyEqual(testCase, ...
    scopeguide.control.cartesianDragDofMask("translation_z"), ...
    logical([0; 0; 1; 0; 0; 0]));
verifyEqual(testCase, ...
    scopeguide.control.cartesianDragDofMask("rotation_xyz"), ...
    logical([0; 0; 0; 1; 1; 1]));
verifyEqual(testCase, ...
    scopeguide.control.cartesianDragDofMask("full_6dof"), ...
    true(6, 1));
verifyError(testCase, ...
    @() scopeguide.control.cartesianDragDofMask("rcm"), ...
    'scopeguide:cartesianDrag:UnknownDofMode');
end

function testWrenchShiftUsesDragPointNotRcm(testCase)
[cfg, ~] = current_cartesian_admittance_drag_config();
cfg.cartesianDrag.TFlangeDragPoint(1:3, 4) = [0; 0; 0.120];
validateCartesianAdmittanceDragConfig(cfg);
wrench = [10; 0; 0; 0; 0; 0];
mapped = scopeguide.control.mapWrenchToDragPoint(wrench, eye(4), cfg);
verifyEqual(testCase, mapped.ForceDragN, [10; 0; 0], ...
    'AbsTol', 1e-12);
verifyEqual(testCase, mapped.MomentDragNm, [0; -1; 0], ...
    'AbsTol', 1e-12);
verifyTrue(testCase, mapped.NoRcmPointUsed);
end

function testDragPointJacobianFiniteDifference(testCase)
[cfg, ~] = current_cartesian_admittance_drag_config();
q = deg2rad([123.7; -17.6; -109.2; 82.2; 92.2; 4.1]);
geometry = scopeguide.geometry.computeDragPointKinematics(q, cfg);
epsilon = 1e-7;
for joint = 1:6
    displaced = scopeguide.geometry.computeDragPointKinematics( ...
        q + unitVector(joint) * epsilon, cfg);
    relative = scopeguide.geometry.relativeDragPose( ...
        geometry.TBaseDragPoint, displaced.TBaseDragPoint) / epsilon;
    numericBase = [ ...
        geometry.RotationBaseFromDrag * relative(1:3); ...
        geometry.RotationBaseFromDrag * relative(4:6)];
    verifyEqual(testCase, numericBase, ...
        geometry.DragPointTwistJacobianBase(:, joint), ...
        'AbsTol', 2e-6);
end
end

function testAdmittanceSteadySpeedAndReset(testCase)
[cfg, ~] = current_cartesian_admittance_drag_config();
cfg.cartesianDrag.DofMode = "full_6dof";
validateCartesianAdmittanceDragConfig(cfg);
controller = scopeguide.control.CartesianAdmittance6D(cfg);
enabled = enableStatus(true, false, 1);
for index = 1:200
    output = controller.step( ...
        [cfg.cartesianDrag.Translation.DesignForceN; zeros(5, 1)], eye(3), ...
        0.05, enabled);
end
verifyEqual(testCase, output.VelocityDrag(1), ...
    cfg.cartesianDrag.Translation.TargetSteadySpeedMSec, ...
    'AbsTol', 5e-6);
verifyEqual(testCase, output.VelocityDrag(2:6), zeros(5, 1), ...
    'AbsTol', 1e-12);
disabled = enableStatus(false, true, 0);
output = controller.step(ones(6, 1), eye(3), 0.05, disabled);
verifyEqual(testCase, output.VelocityDrag, zeros(6, 1), ...
    'AbsTol', 0);
verifyEqual(testCase, output.StatusCode, "DYNAMIC_STATE_RESET");
end

function testTravelLimiterBrakesBeforeBoundary(testCase)
[cfg, ~] = current_cartesian_admittance_drag_config();
cfg.cartesianDrag.TravelLimitsEnabled = true;
relative = [cfg.cartesianDrag.Translation.RelativeLimitM(1) - 0.0001; ...
    zeros(5, 1)];
[limited, diagnostics] = ...
    scopeguide.control.limitCartesianTwistForTravel( ...
    [0.006; zeros(5, 1)], relative, eye(3), 0.05, cfg);
verifyLessThan(testCase, limited(1), 0.006);
verifyGreaterThanOrEqual(testCase, limited(1), 0);
verifyTrue(testCase, diagnostics.ActiveAxes(1));
verifyLessThanOrEqual(testCase, ...
    relative(1) + limited(1) * 0.05, ...
    cfg.cartesianDrag.Translation.RelativeLimitM(1) + 1e-12);
end

function testTravelRecoveryLimiterBlocksOutwardAndAllowsInward(testCase)
[cfg, ~] = current_cartesian_admittance_drag_config();
cfg.cartesianDrag.TravelLimitsEnabled = true;
limit = cfg.cartesianDrag.Translation.RelativeLimitM(1);
relative = [limit + 0.0002; zeros(5, 1)];
[outward, outwardDiagnostics] = ...
    scopeguide.control.limitCartesianTwistForTravel( ...
    [0.002; zeros(5, 1)], relative, eye(3), 0.05, cfg);
verifyEqual(testCase, outward(1), 0, 'AbsTol', 1e-15);
verifyTrue(testCase, outwardDiagnostics.RecoveryActiveAxes(1));
verifyTrue(testCase, outwardDiagnostics.AnyRecoveryActive);
verifyFalse(testCase, outwardDiagnostics.AnyOutsideHardBoundary);

[inward, inwardDiagnostics] = ...
    scopeguide.control.limitCartesianTwistForTravel( ...
    [-0.002; zeros(5, 1)], relative, eye(3), 0.05, cfg);
verifyEqual(testCase, inward(1), -0.002, 'AbsTol', 1e-15);
verifyTrue(testCase, inwardDiagnostics.AnyRecoveryActive);
end

function testDisabledTravelLimiterLeavesRequestedTwistUnchanged(testCase)
[cfg, ~] = current_cartesian_admittance_drag_config();
verifyFalse(testCase, cfg.cartesianDrag.TravelLimitsEnabled);
requested = [0.006; -0.004; 0.003; ...
    deg2rad(1.0); -deg2rad(2.0); deg2rad(0.5)];
relative = [1; -2; 3; deg2rad(170); -deg2rad(120); deg2rad(90)];
[limited, diagnostics] = ...
    scopeguide.control.limitCartesianTwistForTravel( ...
    requested, relative, eye(3), 0.05, cfg);
verifyEqual(testCase, limited, requested, 'AbsTol', 0);
verifyFalse(testCase, diagnostics.TravelLimitsEnabled);
verifyFalse(testCase, diagnostics.AnyBoundaryActive);
verifyFalse(testCase, diagnostics.AnyRecoveryActive);
verifyFalse(testCase, diagnostics.AnyOutsideHardBoundary);
end

function testQpTravelRecoveryBandForTranslationAndRotation(testCase)
[cfg, ~] = current_cartesian_admittance_drag_config();
cfg.cartesianDrag.TravelLimitsEnabled = true;
cfg = scopeguide.runtime.configureCartesianAdmittanceDrag( ...
    cfg, "full_6dof");
cfg.qp.MaximumSolveTimeSec = cfg.runtime.NominalDtSec;
q = deg2rad([123.7; -17.6; -109.2; 82.2; 92.2; 4.1]);
geometry = scopeguide.geometry.computeDragPointKinematics(q, cfg);
rotation = geometry.RotationBaseFromDrag;
softLimits = [cfg.cartesianDrag.Translation.RelativeLimitM(:); ...
    cfg.cartesianDrag.Rotation.RelativeLimitRad(:)];
recovery = [ ...
    cfg.cartesianDrag.Translation.RecoveryToleranceM * ones(3, 1); ...
    cfg.cartesianDrag.Rotation.RecoveryToleranceRad * ones(3, 1)];
anchorProjectionToBase = blkdiag(rotation, rotation);
for warmupIndex = 1:5
    scopeguide.control.solveCartesianVelocityQp( ...
        q, zeros(6, 1), zeros(6, 1), zeros(6, 1), ...
        rotation, 0.05, cfg);
end

for axis = [1, 6]
    relative = zeros(6, 1);
    relative(axis) = softLimits(axis) + 0.4 * recovery(axis);
    requestedAnchor = zeros(6, 1);
    if axis <= 3
        requestedAnchor(axis) = 0.002;
    else
        requestedAnchor(axis) = deg2rad(1.0);
    end
    priorOutward = scopeguide.control.solveCartesianVelocityQp( ...
        q, zeros(6, 1), anchorProjectionToBase * requestedAnchor, ...
        zeros(6, 1), rotation, 0.05, cfg);
    verifyTrue(testCase, priorOutward.MotionCommandValid, ...
        priorOutward.FailureCode);
    outward = scopeguide.control.solveCartesianVelocityQp( ...
        q, priorOutward.QdotCommandRadSec, ...
        anchorProjectionToBase * requestedAnchor, ...
        relative, rotation, 0.05, cfg);
    assumeFalse(testCase, ...
        outward.FailureCode == "QUADPROG_UNAVAILABLE");
    verifyTrue(testCase, outward.MotionCommandValid, ...
        outward.FailureCode);
    verifyTrue(testCase, outward.TravelRecoveryActive);
    verifyTrue(testCase, outward.RecoveryActiveAxes(axis));
    verifyLessThanOrEqual(testCase, ...
        outward.AchievedTwistAnchor(axis), 1e-9);
    verifyLessThanOrEqual(testCase, ...
        outward.PredictedRelativeCoordinate(axis), ...
        relative(axis) + 1e-10);
    verifyEqual(testCase, outward.StatusCode, ...
        "QP_SOLVED_TRAVEL_RECOVERY_NO_RCM");

    requestedAnchor(axis) = -requestedAnchor(axis);
    inward = scopeguide.control.solveCartesianVelocityQp( ...
        q, zeros(6, 1), anchorProjectionToBase * requestedAnchor, ...
        relative, rotation, 0.05, cfg);
    verifyTrue(testCase, inward.MotionCommandValid, ...
        inward.FailureCode);
    verifyLessThan(testCase, inward.AchievedTwistAnchor(axis), 0);
    verifyLessThan(testCase, ...
        inward.PredictedRelativeCoordinate(axis), relative(axis));
end
end

function testQpFaultsOnlyOutsideHardTravelBoundary(testCase)
[cfg, ~] = current_cartesian_admittance_drag_config();
cfg.cartesianDrag.TravelLimitsEnabled = true;
cfg = scopeguide.runtime.configureCartesianAdmittanceDrag( ...
    cfg, "full_6dof");
q = deg2rad([123.7; -17.6; -109.2; 82.2; 92.2; 4.1]);
geometry = scopeguide.geometry.computeDragPointKinematics(q, cfg);
relative = zeros(6, 1);
relative(1) = cfg.cartesianDrag.Translation.RelativeLimitM(1) + ...
    cfg.cartesianDrag.Translation.RecoveryToleranceM + 1e-5;
translationFailure = ...
    scopeguide.control.solveCartesianVelocityQp( ...
    q, zeros(6, 1), zeros(6, 1), relative, ...
    geometry.RotationBaseFromDrag, 0.05, cfg);
assumeFalse(testCase, ...
    translationFailure.FailureCode == "QUADPROG_UNAVAILABLE");
verifyFalse(testCase, translationFailure.MotionCommandValid);
verifyEqual(testCase, translationFailure.FailureCode, ...
    "CURRENT_CARTESIAN_TRAVEL_OUTSIDE_HARD_BOUND");

relative(:) = 0;
relative(6) = cfg.cartesianDrag.Rotation.RelativeLimitRad(3) + ...
    cfg.cartesianDrag.Rotation.RecoveryToleranceRad + deg2rad(0.01);
rotationFailure = scopeguide.control.solveCartesianVelocityQp( ...
    q, zeros(6, 1), zeros(6, 1), relative, ...
    geometry.RotationBaseFromDrag, 0.05, cfg);
verifyFalse(testCase, rotationFailure.MotionCommandValid);
verifyEqual(testCase, rotationFailure.FailureCode, ...
    "CURRENT_CARTESIAN_TRAVEL_OUTSIDE_HARD_BOUND");
end

function testAuthorizationDoesNotEvaluateRcmGates(testCase)
[cfg, ~] = current_cartesian_admittance_drag_config();
cfg.rcm.CalibrationValid = false;
cfg.rcm.BoundsValidated = false;
cfg.rcm.PointBaseM(:) = NaN;
cfg = scopeguide.runtime.configureCartesianAdmittanceDrag( ...
    cfg, "translation_z");
confirmations = validConfirmations(cfg);
authorization = ...
    scopeguide.safety.evaluateCartesianDragAuthorization( ...
    cfg, confirmations);
verifyTrue(testCase, authorization.CommissioningAllowed);
verifyFalse(testCase, authorization.RcmGatesEvaluated);
verifyFalse(testCase, isfield(authorization.Gates, ...
    'RcmCalibrationValid'));
end

function testNoRcmQpSolvesAndHonorsTravelBound(testCase)
[cfg, ~] = current_cartesian_admittance_drag_config();
cfg.cartesianDrag.TravelLimitsEnabled = true;
cfg.rcm.CalibrationValid = false;
cfg.rcm.BoundsValidated = false;
cfg.rcm.PointBaseM(:) = NaN;
cfg = scopeguide.runtime.configureCartesianAdmittanceDrag( ...
    cfg, "full_6dof");
% This unit test verifies the mathematical constraints rather than the
% production timing audit.  The real worker performs a dedicated warm-up
% before enforcing the normal 15 ms solve budget.
cfg.qp.MaximumSolveTimeSec = cfg.runtime.NominalDtSec;
q = deg2rad([123.7; -17.6; -109.2; 82.2; 92.2; 4.1]);
geometry = scopeguide.geometry.computeDragPointKinematics(q, cfg);
relative = zeros(6, 1);
desired = [0; 0; 0.002; 0; 0; 0];
for warmupIndex = 1:5
    scopeguide.control.solveCartesianVelocityQp( ...
        q, zeros(6, 1), desired, relative, ...
        geometry.RotationBaseFromDrag, 0.05, cfg);
end

solution = scopeguide.control.solveCartesianVelocityQp( ...
    q, zeros(6, 1), desired, relative, ...
    geometry.RotationBaseFromDrag, 0.05, cfg);
assumeFalse(testCase, solution.FailureCode == "QUADPROG_UNAVAILABLE");
verifyTrue(testCase, solution.MotionCommandValid, ...
    solution.FailureCode);
verifyTrue(testCase, solution.NoRcmConstraint);
verifyLessThanOrEqual(testCase, ...
    max(abs(solution.QdotCommandRadSec)), ...
    max(cfg.control.MaximumJointCommandRadSec) + 1e-10);
verifyLessThanOrEqual(testCase, ...
    max(abs(solution.PredictedRelativeCoordinate)), ...
    max([cfg.cartesianDrag.Translation.RelativeLimitM(:); ...
    cfg.cartesianDrag.Rotation.RelativeLimitRad(:)]) + 1e-10);
end

function testNoRcmQpAllowsPoseOutsideInactiveTravelReferences(testCase)
[cfg, ~] = current_cartesian_admittance_drag_config();
cfg = scopeguide.runtime.configureCartesianAdmittanceDrag( ...
    cfg, "full_6dof");
cfg.qp.MaximumSolveTimeSec = cfg.runtime.NominalDtSec;
q = deg2rad([123.7; -17.6; -109.2; 82.2; 92.2; 4.1]);
geometry = scopeguide.geometry.computeDragPointKinematics(q, cfg);
relative = [0.25; -0.20; 0.15; ...
    deg2rad(90); -deg2rad(80); deg2rad(70)];
desired = [0; 0; 0.002; 0; 0; 0];
for warmupIndex = 1:5
    scopeguide.control.solveCartesianVelocityQp( ...
        q, zeros(6, 1), desired, relative, ...
        geometry.RotationBaseFromDrag, 0.05, cfg);
end
solution = scopeguide.control.solveCartesianVelocityQp( ...
    q, zeros(6, 1), desired, relative, ...
    geometry.RotationBaseFromDrag, 0.05, cfg);
assumeFalse(testCase, solution.FailureCode == "QUADPROG_UNAVAILABLE");
verifyTrue(testCase, solution.MotionCommandValid, solution.FailureCode);
verifyFalse(testCase, solution.TravelLimitsEnabled);
verifyFalse(testCase, solution.TravelRecoveryActive);
verifyEqual(testCase, solution.StatusCode, ...
    "QP_SOLVED_NO_RCM_NO_TRAVEL_LIMIT");
verifyTrue(testCase, all(isnan(solution.SoftRelativeLimit)));
end

function testControllerDisabledPathIsZeroAndOfflineSafe(testCase)
[cfg, ~] = current_cartesian_admittance_drag_config();
cfg = scopeguide.runtime.configureCartesianAdmittanceDrag( ...
    cfg, "full_6dof");
authorization = ...
    scopeguide.safety.evaluateCartesianDragAuthorization( ...
    cfg, validConfirmations(cfg));
controller = scopeguide.control.CartesianDragController( ...
    cfg, authorization);
processed = validProcessed();
robot = validRobotState();
handle = releasedHandle();
output = controller.step(processed, robot, handle, 0.05);
verifyFalse(testCase, output.CommandEligible);
verifyEqual(testCase, output.QdotCommandRadSec, zeros(6, 1), ...
    'AbsTol', 0);
verifyTrue(testCase, output.NoRcmConstraint);
verifyTrue(testCase, output.DoesNotSendHardwareCommands);
end

function vector = unitVector(index)
vector = zeros(6, 1);
vector(index) = 1;
end

function status = enableStatus(permitted, reset, scale)
status = struct('MotionPermitted', logical(permitted), ...
    'ResetDynamicState', logical(reset), 'CommandScale', double(scale));
end

function confirmations = validConfirmations(cfg)
confirmations = struct( ...
    'MotionPhrase', cfg.robot.MotionConfirmationPhrase, ...
    'SetupPhrase', cfg.cartesianDrag.SetupConfirmationPhrase, ...
    'SafetyThresholdPhrase', ...
        cfg.cartesianDrag.SafetyThresholdConfirmationPhrase, ...
    'SecondObserverPresent', true, ...
    'FixtureAndClearanceConfirmed', true);
end

function processed = validProcessed()
processed = struct();
processed.ControlWrenchToolAtSensorOrigin = zeros(6, 1);
processed.NeutralCheckPassed = true;
processed.Quality = struct('Valid', true, 'StatusCode', "OK");
processed.BaselineDiagnostics = struct('Ready', true);
processed.Safety = struct('StopRequested', false, ...
    'StopReasons', strings(0, 1));
end

function robot = validRobotState()
robot = struct();
robot.JointPositionRad = ...
    deg2rad([123.7; -17.6; -109.2; 82.2; 92.2; 4.1]);
robot.JointVelocityRadSec = zeros(6, 1);
robot.IsValid = true;
end

function input = releasedHandle()
input = struct('Enabled', false, 'SampleAgeSec', 0, ...
    'Source', "test", 'SupportsPhysicalMotion', true, ...
    'IsValid', true, 'StatusCode', "OK");
end
