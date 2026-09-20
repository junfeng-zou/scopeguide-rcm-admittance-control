function tests = testStage05ForceOnlyRcmAdmittance
tests = functiontests(localfunctions);
end

function setupOnce(~)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
end

function testStableOrthonormalRcmBasisAndNoSignSwitch(testCase)
cfg = defaultRcmAdmittanceConfig();
pRcm = [0.3; -0.1; 0.2];
referenceTool = [];
rng(505);
for index = 1:100
    [q, ~] = qr(randn(3));
    if det(q) < 0
        q(:, 1) = -q(:, 1);
    end
    transform = eye(4);
    transform(1:3, 1:3) = q;
    transform(1:3, 4) = pRcm + q * [0; 0; 0.15];
    basis = scopeguide.geometry.computeRcmMotionBasis( ...
        transform, pRcm, cfg);
    axesTool = [basis.PivotAxis1Tool, basis.PivotAxis2Tool, ...
        basis.ShaftAxisTool];
    verifyEqual(testCase, axesTool' * axesTool, eye(3), ...
        'AbsTol', 2e-12);
    verifyGreaterThan(testCase, det(axesTool), 1 - 2e-12);
    if isempty(referenceTool)
        referenceTool = axesTool;
    else
        verifyEqual(testCase, axesTool, referenceTool, 'AbsTol', 0);
    end
    verifyEqual(testCase, ...
        basis.LinearVelocityBasisBase(:, 1:2) - ...
        cross(basis.AngularVelocityBasisBase(:, 1:2), ...
        repmat(basis.RcmToTipBaseM, 1, 2), 1), ...
        zeros(3, 2), 'AbsTol', 3e-12);
end
end

function testPositiveAndNegativeDirectionsForAllThreeDofs(testCase)
[cfg, transform, pRcm] = syntheticGeometry(0.15);
basis = scopeguide.geometry.computeRcmMotionBasis( ...
    transform, pRcm, cfg);
for axisIndex = 1:3
    directionBase = basis.LinearVelocityBasisBase(:, axisIndex);
    directionBase = directionBase / norm(directionBase);
    directionTool = transform(1:3, 1:3)' * directionBase;
    positive = scopeguide.control.mapWrenchToRcmGeneralizedEffort( ...
        [5 * directionTool; zeros(3, 1)], transform, pRcm, cfg);
    negative = scopeguide.control.mapWrenchToRcmGeneralizedEffort( ...
        [-5 * directionTool; zeros(3, 1)], transform, pRcm, cfg);
    verifyGreaterThan(testCase, positive.GeneralizedEffort(axisIndex), 0);
    verifyLessThan(testCase, negative.GeneralizedEffort(axisIndex), 0);
    verifyEqual(testCase, negative.GeneralizedEffort, ...
        -positive.GeneralizedEffort, 'AbsTol', 2e-12);
    verifyEqual(testCase, positive.GeneralizedEffortWithRoll(4), 0, ...
        'AbsTol', 0);
    verifyTrue(testCase, positive.MomentUsedForControl);
end
end

function testMeasuredWrenchIsShiftedFromSensorOriginToRcm(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.tool.TFlangeEndoscope = eye(4);
cfg.tool.TFlangeSensor = eye(4);
cfg.tool.TFlangeSensor(3, 4) = -0.20;
pRcm = zeros(3, 1);
TBaseEndoscope = eye(4);

force = [0; 5; 0];
mapped = scopeguide.control.mapWrenchToRcmGeneralizedEffort( ...
    [force; zeros(3, 1)], TBaseEndoscope, pRcm, cfg);

verifyEqual(testCase, mapped.PSensorBaseM, [0; 0; -0.20], ...
    'AbsTol', 1e-12);
verifyEqual(testCase, mapped.MomentBaseAtRcmNm, [1; 0; 0], ...
    'AbsTol', 1e-12);
verifyEqual(testCase, mapped.GeneralizedEffort, [1; 0; 0], ...
    'AbsTol', 1e-12);
end

function testDirectSensorMomentAndForceMomentAreCombined(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.tool.TFlangeEndoscope = eye(4);
cfg.tool.TFlangeSensor = eye(4);
cfg.tool.TFlangeSensor(3, 4) = -0.20;
pRcm = zeros(3, 1);

mapped = scopeguide.control.mapWrenchToRcmGeneralizedEffort( ...
    [0; 5; 2; -0.25; 0; 0], eye(4), pRcm, cfg);

verifyEqual(testCase, mapped.MomentBaseAtRcmNm, [0.75; 0; 0], ...
    'AbsTol', 1e-12);
verifyEqual(testCase, mapped.GeneralizedEffort, [0.75; 0; 2], ...
    'AbsTol', 1e-12);
end

function testParametersAreDerivedFromDesignQuantities(testCase)
cfg = defaultRcmAdmittanceConfig();
parameters = scopeguide.control.deriveForceOnlyAdmittanceParameters(cfg);
expectedPivotDamping = ...
    cfg.admittance.Pivot.DesignForceN * ...
    cfg.admittance.Pivot.NominalLeverArmM / ...
    cfg.admittance.Pivot.TargetSteadySpeedRadSec;
expectedInsertionDamping = ...
    cfg.admittance.Insertion.DesignForceN / ...
    cfg.admittance.Insertion.TargetSteadySpeedMSec;
verifyEqual(testCase, parameters.Damping(1:2), ...
    expectedPivotDamping * ones(2, 1), 'RelTol', 2e-15);
verifyEqual(testCase, parameters.Damping(3), ...
    expectedInsertionDamping, 'RelTol', 2e-15);
verifyEqual(testCase, parameters.VirtualMass, ...
    parameters.Damping .* parameters.TimeConstantSec, ...
    'RelTol', 2e-15);
verifyFalse(testCase, parameters.RollEnabled);
end

function testDesignInputApproachesTargetAndZeroInputDecays(testCase)
[cfg, transform, pRcm] = syntheticGeometry(0.15);
controller = scopeguide.control.ForceOnlyRcmAdmittance(cfg);
status = enabledStatus(1);
force = [0; -cfg.admittance.Pivot.DesignForceN; 0; 0; 0; 0];
output = simulate(controller, force, transform, pRcm, ...
    3.0, 0.01, status);
verifyEqual(testCase, output.GeneralizedVelocity(1), ...
    cfg.admittance.Pivot.TargetSteadySpeedRadSec, 'RelTol', 2e-3);
output = simulate(controller, zeros(6, 1), transform, pRcm, ...
    4.5, 0.01, status);
verifyLessThanOrEqual(testCase, norm(output.GeneralizedVelocity), ...
    2 * max(controller.Parameters.VelocityZeroTolerance));
verifyEqual(testCase, output.RollRateRadSec, 0, 'AbsTol', 0);
end

function testMeasuredDtJitterIsStable(testCase)
[cfg, transform, pRcm] = syntheticGeometry(0.15);
constant = scopeguide.control.ForceOnlyRcmAdmittance(cfg);
jittered = scopeguide.control.ForceOnlyRcmAdmittance(cfg);
status = enabledStatus(1);
force = [0; 0; 2.5; 0; 0; 0];
for index = 1:100
    constantOutput = constant.step(force, transform, pRcm, ...
        0.010, status);
end
for index = 1:100
    dtSec = 0.006 + 0.008 * mod(index, 2);
    jitteredOutput = jittered.step(force, transform, pRcm, ...
        dtSec, status);
end
verifyEqual(testCase, sum(repmat([0.014, 0.006], 1, 50)), 1, ...
    'AbsTol', 1e-12);
verifyEqual(testCase, jitteredOutput.GeneralizedVelocity(3), ...
    constantOutput.GeneralizedVelocity(3), 'RelTol', 0.02);
verifyEqual(testCase, jitteredOutput.RelativeCoordinate(3), ...
    constantOutput.RelativeCoordinate(3), 'RelTol', 0.02);
end

function testLargeInputRespectsAccelerationSpeedAndTravel(testCase)
[cfg, transform, pRcm] = syntheticGeometry(0.15);
controller = scopeguide.control.ForceOnlyRcmAdmittance(cfg);
status = enabledStatus(1);
force = [1000; -1000; 1000; 0; 0; 0];
previousVelocity = zeros(3, 1);
maxPivotAcceleration = 0;
maxInsertionAcceleration = 0;
for index = 1:3000
    output = controller.step(force, transform, pRcm, 0.01, status);
    acceleration = (output.GeneralizedVelocity - previousVelocity) / 0.01;
    previousVelocity = output.GeneralizedVelocity;
    maxPivotAcceleration = max(maxPivotAcceleration, ...
        norm(acceleration(1:2)));
    maxInsertionAcceleration = max(maxInsertionAcceleration, ...
        abs(acceleration(3)));
    verifyLessThanOrEqual(testCase, ...
        norm(output.GeneralizedVelocity(1:2)), ...
        cfg.control.PivotMaxRadSec + 1e-12);
    verifyLessThanOrEqual(testCase, abs(output.GeneralizedVelocity(3)), ...
        cfg.control.InsertionMaxMSec + 1e-12);
    verifyLessThanOrEqual(testCase, ...
        norm(output.RelativeCoordinate(1:2)), ...
        cfg.control.RelativePivotLimitRad + 1e-12);
    verifyLessThanOrEqual(testCase, abs(output.RelativeCoordinate(3)), ...
        cfg.control.RelativeInsertionLimitM + 1e-12);
    verifyEqual(testCase, output.RollRateRadSec, 0, 'AbsTol', 0);
end
verifyLessThanOrEqual(testCase, maxPivotAcceleration, ...
    cfg.admittance.Pivot.MaximumAccelerationRadSec2 + 2e-10);
verifyLessThanOrEqual(testCase, maxInsertionAcceleration, ...
    cfg.admittance.Insertion.MaximumAccelerationMSec2 + 2e-10);
verifyGreaterThan(testCase, ...
    norm(output.RelativeCoordinate(1:2)), ...
    0.95 * cfg.control.RelativePivotLimitRad);
verifyGreaterThan(testCase, abs(output.RelativeCoordinate(3)), ...
    0.95 * cfg.control.RelativeInsertionLimitM);

released = controller.step(zeros(6, 1), transform, pRcm, 0.01, status);
verifyLessThanOrEqual(testCase, norm(released.GeneralizedVelocity), ...
    norm(previousVelocity) + 1e-12);
verifyLessThanOrEqual(testCase, ...
    norm(controller.GeneralizedVelocity - released.GeneralizedVelocity), ...
    1e-15);
end

function testPivotTipSpeedLimitTracksRcmTipDistance(testCase)
[cfg, shortTransform, shortRcm] = syntheticGeometry(0.10);
[~, longTransform, longRcm] = syntheticGeometry(1.00);
short = scopeguide.control.ForceOnlyRcmAdmittance(cfg);
long = scopeguide.control.ForceOnlyRcmAdmittance(cfg);
status = enabledStatus(1);
force = [0; -100; 0; 0; 0; 0];
for index = 1:100
    shortOutput = short.step(force, shortTransform, shortRcm, ...
        0.01, status);
    longOutput = long.step(force, longTransform, longRcm, ...
        0.01, status);
end
limit = cfg.admittance.Pivot.MaximumTipSpeedMSec;
verifyLessThanOrEqual(testCase, ...
    shortOutput.PivotTipLinearSpeedMSec, limit + 1e-12);
verifyLessThanOrEqual(testCase, ...
    longOutput.PivotTipLinearSpeedMSec, limit + 1e-12);
verifyLessThan(testCase, norm(longOutput.GeneralizedVelocity(1:2)), ...
    norm(shortOutput.GeneralizedVelocity(1:2)));
verifyEqual(testCase, longOutput.EffectivePivotSpeedLimitRadSec, ...
    limit, 'RelTol', 1e-12);
end

function testStage04ResetAndSoftStartAreImmediate(testCase)
[cfg, transform, pRcm] = syntheticGeometry(0.15);
controller = scopeguide.control.ForceOnlyRcmAdmittance(cfg);
force = [0; -100; 0; 0; 0; 0];

transition = enabledStatus(0);
transition.ResetDynamicState = true;
output = controller.step(force, transform, pRcm, 0.01, transition);
verifyTrue(testCase, output.DynamicStateReset);
verifyEqual(testCase, output.GeneralizedVelocity, zeros(3, 1), ...
    'AbsTol', 0);

zeroScale = controller.step(force, transform, pRcm, 0.01, ...
    enabledStatus(0));
verifyEqual(testCase, zeroScale.GeneralizedVelocity, zeros(3, 1), ...
    'AbsTol', 0);
smallScale = controller.step(force, transform, pRcm, 0.01, ...
    enabledStatus(0.05));
verifyGreaterThan(testCase, norm(smallScale.GeneralizedVelocity), 0);
verifyLessThanOrEqual(testCase, ...
    norm(smallScale.GeneralizedAcceleration(1:2)), ...
    cfg.admittance.Pivot.MaximumAccelerationRadSec2 + 1e-12);

released = enabledStatus(0);
released.MotionPermitted = false;
released.ResetDynamicState = true;
stopped = controller.step(force, transform, pRcm, 0.01, released);
verifyEqual(testCase, stopped.GeneralizedVelocity, zeros(3, 1), ...
    'AbsTol', 0);
verifyEqual(testCase, stopped.RelativeCoordinate, zeros(3, 1), ...
    'AbsTol', 0);
verifyEqual(testCase, stopped.DesiredTipTwistBase, zeros(6, 1), ...
    'AbsTol', 0);
end

function testInvalidInputAndLongPeriodFailClosed(testCase)
[cfg, transform, pRcm] = syntheticGeometry(0.15);
controller = scopeguide.control.ForceOnlyRcmAdmittance(cfg);
status = enabledStatus(1);
controller.step([0; -5; 0; 0; 0; 0], ...
    transform, pRcm, 0.01, status);
invalid = controller.step([NaN; zeros(5, 1)], transform, pRcm, ...
    0.01, status);
verifyTrue(testCase, invalid.RequiresFault);
verifyEqual(testCase, invalid.GeneralizedVelocity, zeros(3, 1), ...
    'AbsTol', 0);
late = controller.step(zeros(6, 1), transform, pRcm, ...
    cfg.runtime.MaximumDtSec + 0.001, status);
verifyTrue(testCase, late.RequiresFault);
verifyEqual(testCase, late.StatusCode, "INVALID_CONTROL_PERIOD");
verifyEqual(testCase, controller.RelativeCoordinate, zeros(3, 1), ...
    'AbsTol', 0);
end

function testCurrentStaticRecordHasNoSustainedAdmittanceDrift(testCase)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
source = fullfile(projectRoot, 'force_calibration', 'data', ...
    'hex_h_tool_calibration_20260812_154348', 'raw_samples.csv');
assumeTrue(testCase, isfile(source));
[replay, outputDirectory] = ...
    run_stage05_force_only_admittance_replay( ...
    SourceFile=string(source), MaximumSamples=1200, ...
    WriteResults=false);
verifyEqual(testCase, outputDirectory, "");
verifyGreaterThan(testCase, ...
    replay.Summary.EligibleAdmittanceSampleCount, 0);
verifyTrue(testCase, replay.Summary.NoSustainedDrift);
verifyTrue(testCase, replay.Summary.MomentUsedForControl);
verifyFalse(testCase, replay.Summary.HardwareConnectionsCreated);
verifyFalse(testCase, replay.Summary.MotionCommandsSent);
end

function output = simulate(controller, force, transform, pRcm, ...
        durationSec, dtSec, status)
count = round(durationSec / dtSec);
for index = 1:count
    output = controller.step(force, transform, pRcm, dtSec, status);
end
end

function [cfg, transform, pRcm] = syntheticGeometry(distanceM)
cfg = defaultRcmAdmittanceConfig();
cfg.tool.TFlangeEndoscope = eye(4);
cfg.tool.TFlangeSensor = eye(4);
pRcm = [0; 0; 0];
transform = eye(4);
transform(3, 4) = distanceM;
end

function status = enabledStatus(scale)
status = struct('MotionPermitted', true, ...
    'ResetDynamicState', false, 'CommandScale', scale);
end
