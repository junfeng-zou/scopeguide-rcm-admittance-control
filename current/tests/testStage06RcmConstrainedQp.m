function tests = testStage06RcmConstrainedQp
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
cfg = defaultRcmAdmittanceConfig();
testCase.TestData.Warmup = warmupActualProblem(cfg);
end

function testQuadprogRouteIsActuallyAvailable(testCase)
cfg = defaultRcmAdmittanceConfig();
verifyTrue(testCase, contains(string(which('quadprog')), "quadprog"));
verifyTrue(testCase, logical(license('test', 'Optimization_Toolbox')));
verifyEqual(testCase, cfg.qp.Solver, "quadprog");
verifyTrue(testCase, testCase.TestData.Warmup.FinalSolved);
end

function testRcmJacobianMatchesFiniteDifference(testCase)
cfg = defaultRcmAdmittanceConfig();
q = safeConfiguration(1);
[pAxis, basis] = axisPoint(q, cfg);
pRcm = pAxis + 4e-4 * basis.PivotAxis1Base - ...
    2e-4 * basis.PivotAxis2Base;
geometry = scopeguide.geometry.computeRcmConstraintKinematics( ...
    q, pRcm, cfg);
qdot = [0.01; -0.02; 0.015; 0.008; -0.012; 0.006];
h = 1e-6;
plus = scopeguide.geometry.computeRcmConstraintKinematics( ...
    q + h * qdot, pRcm, cfg);
minus = scopeguide.geometry.computeRcmConstraintKinematics( ...
    q - h * qdot, pRcm, cfg);
errorDerivativeBase = (plus.RcmErrorBaseM - ...
    minus.RcmErrorBaseM) / (2 * h);
numeric = geometry.TransverseBasisBase' * errorDerivativeBase;
analytic = -geometry.RcmLateralJacobian * qdot;
verifyEqual(testCase, numeric, analytic, 'AbsTol', 2e-8);
end

function testHardSoftAndHybridModesSolveIdealGeometry(testCase)
for mode = ["hard", "soft", "hybrid"]
    cfg = defaultRcmAdmittanceConfig();
    cfg.rcm.Mode = mode;
    validateRcmAdmittanceConfig(cfg);
    q = safeConfiguration(1);
    [pRcm, basis] = axisPoint(q, cfg);
    desired = basis.TwistBasisBase * ...
        [deg2rad(0.05); -deg2rad(0.04); 0.0002];
    solved = scopeguide.control.solveRcmConstrainedQp( ...
        q, zeros(6, 1), desired, zeros(3, 1), ...
        pRcm, 0.01, cfg);
    verifyTrue(testCase, solved.MotionCommandValid, mode);
    verifyGreaterThan(testCase, solved.ExitFlag, 0);
    verifyLessThanOrEqual(testCase, ...
        solved.ConstraintDiagnostics.MaximumViolation, ...
        cfg.qp.ConstraintTolerance);
    verifyLessThanOrEqual(testCase, solved.PredictedRcmErrorNormM, ...
        cfg.rcm.HardRadiusM);
    verifyEqual(testCase, solved.NoFallbackSolverUsed, true);
    if mode == "hard"
        verifyLessThanOrEqual(testCase, ...
            solved.ConstraintDiagnostics.MaximumEqualityViolation, ...
            cfg.qp.ConstraintTolerance);
    end
end
end

function testHybridRecoveryStrengthIncreasesWithError(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.rcm.Mode = "hybrid";
q = safeConfiguration(1);
[pAxis, basis] = axisPoint(q, cfg);
errors = [0.2, 0.8, 1.3] * 1e-3;
weights = zeros(size(errors));
gains = zeros(size(errors));
recovery = zeros(size(errors));
for index = 1:numel(errors)
    pRcm = pAxis + errors(index) * basis.PivotAxis1Base;
    solved = scopeguide.control.solveRcmConstrainedQp( ...
        q, zeros(6, 1), zeros(6, 1), zeros(3, 1), ...
        pRcm, 0.01, cfg);
    verifyTrue(testCase, solved.MotionCommandValid);
    weights(index) = solved.RcmObjectiveWeight;
    gains(index) = solved.RcmRecoveryGainSecInv;
    recovery(index) = norm( ...
        solved.Geometry.RcmLateralJacobian * ...
        solved.QdotCommandRadSec);
end
verifyGreaterThanOrEqual(testCase, diff(weights), zeros(1, 2));
verifyGreaterThanOrEqual(testCase, diff(gains), zeros(1, 2));
verifyGreaterThan(testCase, recovery(2), recovery(1));
verifyGreaterThan(testCase, recovery(3), recovery(2));
end

function testSoftRcmRecoveryDeadzoneAndSmoothActivation(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.rcm.Mode = "soft";
q = safeConfiguration(1);
[pAxis, basis] = axisPoint(q, cfg);
errorsM = [0.00020, 0.000375, 0.00060];
expectedScales = [0, 0.5, 1];

for index = 1:numel(errorsM)
    pRcm = pAxis + errorsM(index) * basis.PivotAxis1Base;
    solved = scopeguide.control.solveRcmConstrainedQp( ...
        q, zeros(6, 1), zeros(6, 1), zeros(3, 1), ...
        pRcm, 0.01, cfg);
    verifyTrue(testCase, solved.MotionCommandValid);
    verifyEqual(testCase, solved.RcmRecoveryActivationScale, ...
        expectedScales(index), 'AbsTol', 1e-10);
    expectedTargetNorm = cfg.qp.RcmRecoveryGainMinimumSecInv * ...
        expectedScales(index) * errorsM(index);
    verifyEqual(testCase, norm(solved.RcmRecoveryTarget2MSec), ...
        expectedTargetNorm, 'AbsTol', 1e-12);
end
end

function testHardRcmRecoveryDoesNotUseDeadzone(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.rcm.Mode = "hard";
q = safeConfiguration(1);
[pAxis, basis] = axisPoint(q, cfg);
errorM = 0.00020;
pRcm = pAxis + errorM * basis.PivotAxis1Base;
solved = scopeguide.control.solveRcmConstrainedQp( ...
    q, zeros(6, 1), zeros(6, 1), zeros(3, 1), ...
    pRcm, 0.01, cfg);

verifyTrue(testCase, solved.MotionCommandValid);
verifyEqual(testCase, solved.RcmRecoveryActivationScale, 1, ...
    'AbsTol', 0);
verifyEqual(testCase, norm(solved.RcmRecoveryTarget2MSec), ...
    cfg.qp.RcmRecoveryGainMaximumSecInv * errorM, ...
    'AbsTol', 1e-12);
end

function testJointSpeedAccelerationAndPredictionBounds(testCase)
cfg = defaultRcmAdmittanceConfig();
q = safeConfiguration(2);
[pRcm, basis] = axisPoint(q, cfg);
desired = basis.TwistBasisBase * ...
    [deg2rad(0.2); deg2rad(0.1); 0.0005];
previous = deg2rad(0.1) * [1; -1; 1; -1; 1; -1];
solved = scopeguide.control.solveRcmConstrainedQp( ...
    q, previous, desired, zeros(3, 1), pRcm, 0.01, cfg);
verifyTrue(testCase, solved.MotionCommandValid);
verifyLessThanOrEqual(testCase, max(abs(solved.QdotCommandRadSec)), ...
    max(cfg.control.MaximumJointCommandRadSec) + 1e-12);
verifyLessThanOrEqual(testCase, ...
    max(abs(solved.QdotCommandRadSec - previous)), ...
    max(cfg.control.MaximumJointAccelerationRadSec2) * 0.01 + 1e-12);
positionLimit = cfg.kinematics.MaximumAbsJointAngleRad - ...
    cfg.qp.JointPositionMarginRad;
verifyLessThanOrEqual(testCase, ...
    max(abs(solved.PredictedJointPositionRad)), ...
    positionLimit + 1e-12);
end

function testHybridHardEnvelopeAndRelativeBoundsAreIndependent(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.rcm.Mode = "hybrid";
q = safeConfiguration(1);
[pAxis, basis] = axisPoint(q, cfg);
pRcm = pAxis + 1.40e-3 * basis.PivotAxis1Base;
relative = [0.98 * cfg.control.RelativePivotLimitRad; 0; ...
    0.98 * cfg.control.RelativeInsertionLimitM];
desired = basis.TwistBasisBase * ...
    [cfg.control.PivotMaxRadSec; 0; cfg.control.InsertionMaxMSec];
solved = scopeguide.control.solveRcmConstrainedQp( ...
    q, zeros(6, 1), desired, relative, pRcm, 0.01, cfg);
verifyTrue(testCase, solved.MotionCommandValid);
verifyLessThanOrEqual(testCase, solved.PredictedRcmErrorNormM, ...
    cfg.rcm.HardRadiusM + cfg.qp.ConstraintTolerance);
verifyLessThanOrEqual(testCase, ...
    norm(solved.PredictedRelativeCoordinate(1:2)), ...
    cfg.control.RelativePivotLimitRad + cfg.qp.ConstraintTolerance);
verifyLessThanOrEqual(testCase, ...
    abs(solved.PredictedRelativeCoordinate(3)), ...
    cfg.control.RelativeInsertionLimitM + cfg.qp.ConstraintTolerance);
verifyGreaterThan(testCase, solved.RcmObjectiveWeight, ...
    cfg.qp.HybridRcmWeightMinimum);
verifyGreaterThan(testCase, ...
    solved.InequalityConstraintGroups.RcmRows, 0);
end

function testRelativeTravelRecoveryBlocksOutwardAndAllowsInward(testCase)
cfg = defaultRcmAdmittanceConfig();
q = safeConfiguration(1);
[pRcm, basis] = axisPoint(q, cfg);
relative = [ ...
    cfg.control.RelativePivotLimitRad + ...
        0.40 * cfg.control.RelativePivotRecoveryToleranceRad; ...
    0; ...
    cfg.control.RelativeInsertionLimitM + ...
        0.40 * cfg.control.RelativeInsertionRecoveryToleranceM];

outwardDesired = basis.TwistBasisBase * [ ...
    0.25 * cfg.control.PivotMaxRadSec; 0; ...
    0.25 * cfg.control.InsertionMaxMSec];
outward = scopeguide.control.solveRcmConstrainedQp( ...
    q, zeros(6, 1), outwardDesired, relative, pRcm, 0.01, cfg);
verifyTrue(testCase, outward.MotionCommandValid);
verifyEqual(testCase, outward.StatusCode, ...
    "QP_SOLVED_TRAVEL_RECOVERY_RCM");
verifyTrue(testCase, outward.RelativeTravelRecoveryActive);
verifyTrue(testCase, outward.PivotTravelRecoveryActive);
verifyTrue(testCase, outward.InsertionTravelRecoveryActive);
verifyLessThanOrEqual(testCase, ...
    dot(relative(1:2), outward.AchievedGeneralizedVelocity(1:2)), ...
    1e-12);
verifyLessThanOrEqual(testCase, ...
    outward.AchievedGeneralizedVelocity(3), 1e-12);
verifyLessThanOrEqual(testCase, ...
    norm(outward.PredictedRelativeCoordinate(1:2)), ...
    norm(relative(1:2)) + 1e-12);
verifyLessThanOrEqual(testCase, ...
    outward.PredictedRelativeCoordinate(3), relative(3) + 1e-12);

inwardDesired = basis.TwistBasisBase * [ ...
    -0.25 * cfg.control.PivotMaxRadSec; 0; ...
    -0.25 * cfg.control.InsertionMaxMSec];
inward = scopeguide.control.solveRcmConstrainedQp( ...
    q, zeros(6, 1), inwardDesired, relative, pRcm, 0.01, cfg);
verifyTrue(testCase, inward.MotionCommandValid);
verifyLessThan(testCase, ...
    dot(relative(1:2), inward.AchievedGeneralizedVelocity(1:2)), 0);
verifyLessThan(testCase, inward.AchievedGeneralizedVelocity(3), 0);
end

function testRelativeTravelOutsideRecoveryBandFailsClosed(testCase)
cfg = defaultRcmAdmittanceConfig();
q = safeConfiguration(1);
[pRcm, ~] = axisPoint(q, cfg);

pivotOutside = [ ...
    cfg.control.RelativePivotLimitRad + ...
        cfg.control.RelativePivotRecoveryToleranceRad + deg2rad(0.01); ...
    0; 0];
pivot = scopeguide.control.solveRcmConstrainedQp( ...
    q, zeros(6, 1), zeros(6, 1), pivotOutside, ...
    pRcm, 0.01, cfg);
verifyFalse(testCase, pivot.MotionCommandValid);
verifyEqual(testCase, pivot.FailureCode, ...
    "CURRENT_RELATIVE_TRAVEL_OUTSIDE_HARD_BOUND");
verifyEqual(testCase, pivot.QdotCommandRadSec, zeros(6, 1), ...
    'AbsTol', 0);

insertionOutside = [0; 0; ...
    cfg.control.RelativeInsertionLimitM + ...
        cfg.control.RelativeInsertionRecoveryToleranceM + 1e-5];
insertion = scopeguide.control.solveRcmConstrainedQp( ...
    q, zeros(6, 1), zeros(6, 1), insertionOutside, ...
    pRcm, 0.01, cfg);
verifyFalse(testCase, insertion.MotionCommandValid);
verifyEqual(testCase, insertion.FailureCode, ...
    "CURRENT_RELATIVE_TRAVEL_OUTSIDE_HARD_BOUND");
verifyEqual(testCase, insertion.QdotCommandRadSec, zeros(6, 1), ...
    'AbsTol', 0);
end

function testInfeasibleBoundsReturnExactZero(testCase)
cfg = defaultRcmAdmittanceConfig();
q = safeConfiguration(1);
[pRcm, ~] = axisPoint(q, cfg);
solved = scopeguide.control.solveRcmConstrainedQp( ...
    q, 10 * ones(6, 1), zeros(6, 1), zeros(3, 1), ...
    pRcm, 0.01, cfg);
verifyFalse(testCase, solved.MotionCommandValid);
verifyEqual(testCase, solved.FailureCode, "JOINT_BOUNDS_INFEASIBLE");
verifyEqual(testCase, solved.QdotCommandRadSec, zeros(6, 1), ...
    'AbsTol', 0);
end

function testTimeoutAndAbnormalExitReturnExactZero(testCase)
cfg = defaultRcmAdmittanceConfig();
q = safeConfiguration(1);
[pRcm, ~] = axisPoint(q, cfg);
timeout = scopeguide.control.solveRcmConstrainedQp( ...
    q, zeros(6, 1), zeros(6, 1), zeros(3, 1), pRcm, ...
    0.01, cfg, DiagnosticSolveTimeOverrideSec= ...
    cfg.qp.MaximumSolveTimeSec + 0.001);
verifyEqual(testCase, timeout.FailureCode, "QP_TIMEOUT");
verifyEqual(testCase, timeout.QdotCommandRadSec, zeros(6, 1), ...
    'AbsTol', 0);

abnormal = scopeguide.control.solveRcmConstrainedQp( ...
    q, zeros(6, 1), zeros(6, 1), zeros(3, 1), pRcm, ...
    0.01, cfg, DiagnosticExitFlagOverride=-7);
verifyEqual(testCase, abnormal.FailureCode, "QP_ABNORMAL_EXIT");
verifyEqual(testCase, abnormal.QdotCommandRadSec, zeros(6, 1), ...
    'AbsTol', 0);
end

function testSingularityScaleShrinksAndStops(testCase)
cfg = defaultRcmAdmittanceConfig();
qSingular = zeros(6, 1);
[pSingular, ~] = axisPoint(qSingular, cfg);
stopped = scopeguide.control.solveRcmConstrainedQp( ...
    qSingular, zeros(6, 1), ones(6, 1), zeros(3, 1), ...
    pSingular, 0.01, cfg);
verifyEqual(testCase, stopped.FailureCode, "JACOBIAN_SINGULAR_STOP");
verifyEqual(testCase, stopped.QdotCommandRadSec, zeros(6, 1), ...
    'AbsTol', 0);
verifyEqual(testCase, stopped.SingularityScale, 0, 'AbsTol', 0);

qMid = safeConfiguration(3);
[pMid, basisMid] = axisPoint(qMid, cfg);
mid = scopeguide.control.solveRcmConstrainedQp( ...
    qMid, zeros(6, 1), basisMid.TwistBasisBase * ...
    [deg2rad(0.05); 0; 0], zeros(3, 1), pMid, 0.01, cfg);
qFull = safeConfiguration(1);
[pFull, basisFull] = axisPoint(qFull, cfg);
full = scopeguide.control.solveRcmConstrainedQp( ...
    qFull, zeros(6, 1), basisFull.TwistBasisBase * ...
    [deg2rad(0.05); 0; 0], zeros(3, 1), pFull, 0.01, cfg);
verifyTrue(testCase, mid.MotionCommandValid);
verifyTrue(testCase, full.MotionCommandValid);
verifyGreaterThan(testCase, mid.SingularityScale, 0);
verifyLessThan(testCase, mid.SingularityScale, 1);
verifyEqual(testCase, full.SingularityScale, 1, 'AbsTol', 0);
end

function testConsecutiveFailuresEscalateAndReset(testCase)
cfg = defaultRcmAdmittanceConfig();
controller = scopeguide.control.RcmConstrainedQpController(cfg);
q = safeConfiguration(1);
[pAxis, basis] = axisPoint(q, cfg);
pOutside = pAxis + 5e-3 * basis.PivotAxis1Base;
status = enabledStatus();
for index = 1:cfg.qp.MaximumConsecutiveFailures
    output = controller.step(q, zeros(6, 1), pOutside, 0.01, status);
    verifyEqual(testCase, output.QdotCommandRadSec, zeros(6, 1), ...
        'AbsTol', 0);
    if index < cfg.qp.MaximumConsecutiveFailures
        verifyFalse(testCase, output.RequiresFault);
    end
end
verifyTrue(testCase, output.RequiresFault);
verifyFalse(testCase, output.QpHealthy);

released = status;
released.MotionPermitted = false;
released.ResetDynamicState = true;
resetOutput = controller.step(q, zeros(6, 1), pAxis, ...
    0.01, released);
verifyTrue(testCase, resetOutput.DynamicStateReset);
verifyEqual(testCase, controller.ConsecutiveFailureCount, uint32(0));
verifyEqual(testCase, controller.AppliedRelativeCoordinate, ...
    zeros(3, 1), 'AbsTol', 0);
end

function testRepresentativeWorkspaceScanPasses(testCase)
[scan, outputDirectory] = run_stage06_qp_workspace_scan( ...
    SampleCount=60, RandomSeed=607, WriteResults=false);
verifyEqual(testCase, outputDirectory, "");
verifyTrue(testCase, scan.Summary.WorkspaceScanPassed);
verifyGreaterThanOrEqual(testCase, scan.Summary.FeasibilityRate, ...
    scan.Summary.MinimumAcceptedFeasibilityRate);
verifyEqual(testCase, scan.Summary.UnexpectedFailureCount, 0);
verifyTrue(testCase, scan.Summary.TimingBudgetPassed);
verifyFalse(testCase, scan.Summary.HardwareConnectionsCreated);
verifyFalse(testCase, scan.Summary.MotionCommandsSent);
end

function report = warmupActualProblem(cfg)
scopeguide.control.warmupRcmQpSolver(cfg, Iterations=12);
q = safeConfiguration(1);
[pAxis, basis] = axisPoint(q, cfg);
desired = basis.TwistBasisBase * [deg2rad(0.05); 0; 0.0001];
solved = false;
lastTime = NaN;
for mode = ["hard", "soft", "hybrid"]
    modeConfig = cfg;
    modeConfig.rcm.Mode = mode;
    if mode == "hard"
        pRcm = pAxis;
    else
        pRcm = pAxis + 8e-4 * basis.PivotAxis1Base;
    end
    for index = 1:12
        result = scopeguide.control.solveRcmConstrainedQp( ...
            q, zeros(6, 1), desired, zeros(3, 1), ...
            pRcm, 0.01, modeConfig);
        solved = result.MotionCommandValid;
        lastTime = result.SolveTimeSec;
    end
end
report = struct('FinalSolved', solved, 'LastSolveTimeSec', lastTime);
end

function q = safeConfiguration(index)
configurations = [ ...
    0.2, -0.4, 0.5; ...
    -0.5, 0.6, 0.3; ...
    0.8, -0.7, -0.6; ...
    -0.4, 0.5, -0.4; ...
    0.3, -0.3, 0.7; ...
    -0.2, 0.4, -0.5];
q = configurations(:, index);
end

function [pRcm, basis] = axisPoint(q, cfg)
kinematics = scopeguide.geometry.cr5ForwardKinematics(q, cfg);
pRcm = kinematics.EndoscopeTipPositionBaseM - ...
    0.15 * kinematics.ShaftAxisBase;
basis = scopeguide.geometry.computeRcmMotionBasis( ...
    kinematics.TBaseEndoscope, pRcm, cfg);
end

function status = enabledStatus()
status = struct('MotionPermitted', true, ...
    'ResetDynamicState', false, 'CommandScale', 1);
end
