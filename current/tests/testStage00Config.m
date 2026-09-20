function tests = testStage00Config
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
testCase.TestData.ProjectRoot = projectRoot;
end

function testSafeDefaults(testCase)
cfg = defaultRcmAdmittanceConfig();

verifyEqual(testCase, cfg.runtime.Mode, "offline_replay");
verifyFalse(testCase, cfg.robot.EnableMotion);
verifyTrue(testCase, cfg.robot.DryRun);
verifyFalse(testCase, cfg.control.EnableRoll);
verifyEqual(testCase, cfg.control.DofCount, 3);
verifyTrue(testCase, cfg.tool.GeometryVerified);
verifyTrue(testCase, cfg.force.ParametersValidated);
verifyTrue(testCase, cfg.rcm.CalibrationValid);
verifyTrue(testCase, cfg.rcm.BoundsValidated);
verifyTrue(testCase, isfile(cfg.meta.UserAttestationRecord));
attestation = jsondecode(fileread(cfg.meta.UserAttestationRecord));
verifyTrue(testCase, attestation.toolGeometry.verified);
verifyTrue(testCase, attestation.rcm.calibrationValid);
verifyTrue(testCase, attestation.rcm.boundsValidated);
verifyTrue(testCase, attestation.forceProcessing.parametersValidated);
verifyFalse(testCase, ...
    attestation.operatorInput.physicalDeadmanVerified);
verifyFalse(testCase, cfg.safety.ThresholdsValidated);
verifyEqual(testCase, cfg.tool.TFlangeEndoscope(1:3, 4), ...
    [-0.002840; 0.113051; 0.355046], 'AbsTol', 1e-12);
verifyEqual(testCase, cfg.tool.TFlangeSensor(1:3, 4), ...
    [0; 0; 0.020], 'AbsTol', 1e-12);
verifyEqual(testCase, cfg.tool.TFlangeSensor(1:3, 1:3), ...
    diag([-1, -1, 1]), 'AbsTol', 0);
verifyEqual(testCase, cfg.stage08.SpeedScale, 2.5, 'AbsTol', 0);
verifyEqual(testCase, ...
    rad2deg(cfg.control.RelativePivotRecoveryToleranceRad), ...
    0.3, 'AbsTol', 10 * eps);
verifyEqual(testCase, ...
    1e3 * cfg.control.RelativeInsertionRecoveryToleranceM, ...
    0.5, 'AbsTol', 10 * eps);
verifyEqual(testCase, ...
    rad2deg(cfg.stage08.MaximumServoTargetStepRad), 0.30, ...
    'AbsTol', 10 * eps);
verifyWarningFree(testCase, @() validateRcmAdmittanceConfig(cfg));
end

function testRelativeTravelRecoveryBandValidation(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.control.RelativePivotRecoveryToleranceRad = ...
    cfg.control.RelativePivotLimitRad;
verifyError(testCase, @() validateRcmAdmittanceConfig(cfg), ...
    'scopeguide:config:PivotRecoveryBandTooLarge');

cfg = defaultRcmAdmittanceConfig();
cfg.control.RelativeInsertionRecoveryToleranceM = ...
    cfg.control.RelativeInsertionLimitM;
verifyError(testCase, @() validateRcmAdmittanceConfig(cfg), ...
    'scopeguide:config:InsertionRecoveryBandTooLarge');
end

function testThirtyHertzCommunicationAlignedTimingDefaults(testCase)
cfg = defaultRcmAdmittanceConfig();

verifyEqual(testCase, cfg.runtime.ControlRateHz, 30, 'AbsTol', 0);
verifyEqual(testCase, cfg.runtime.NominalDtSec, 1 / 30, ...
    'AbsTol', eps);
verifyEqual(testCase, cfg.robot.ExpectedFeedbackRateHz, 33, ...
    'AbsTol', 0);
verifyGreaterThan(testCase, cfg.robot.FeedbackStaleSec, 2 / 33);
verifyLessThan(testCase, cfg.runtime.MaximumDtSec, 3 / 30);
verifyLessThan(testCase, cfg.qp.MaximumSolveTimeSec, ...
    0.5 * cfg.runtime.NominalDtSec);
end

function testInconsistentControlRateRejected(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.runtime.NominalDtSec = 0.010;

verifyError(testCase, @() validateRcmAdmittanceConfig(cfg), ...
    'scopeguide:config:InconsistentControlRate');
end

function testInvalidModeRejected(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.runtime.Mode = "unknown";
verifyError(testCase, @() validateRcmAdmittanceConfig(cfg), ...
    'scopeguide:config:InvalidMode');
end

function testUnsafeOfflineFlagsRejected(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.robot.EnableMotion = true;
cfg.robot.DryRun = false;
verifyError(testCase, @() validateRcmAdmittanceConfig(cfg), ...
    'scopeguide:config:UnsafeDryRunConfiguration');
end

function testInvalidToolTransformRejected(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.tool.TFlangeEndoscope(1:3, 1:3) = diag([1, 1, -1]);
verifyError(testCase, @() validateRcmAdmittanceConfig(cfg), ...
    'scopeguide:config:InvalidTransform');
end

function testInvalidSensorTransformRejected(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.tool.TFlangeSensor(1:3, 1:3) = diag([1, 1, -1]);
verifyError(testCase, @() validateRcmAdmittanceConfig(cfg), ...
    'scopeguide:config:InvalidTransform');
end

function testLegacyStage08ConfigMigrationPreservesSpeedScale(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.tool = rmfield(cfg.tool, ...
    {'TFlangeSensor', 'SensorOriginSource'});
cfg.stage08.SpeedScale = 2.5;
cfg.stage08.MaximumServoTargetStepRad = deg2rad(0.10);

upgraded = scopeguide.runtime.upgradeRcmAdmittanceConfig(cfg);

verifyEqual(testCase, upgraded.stage08.SpeedScale, 2.5, 'AbsTol', 0);
verifyEqual(testCase, ...
    rad2deg(upgraded.stage08.MaximumServoTargetStepRad), 0.30, ...
    'AbsTol', 10 * eps);
verifyEqual(testCase, upgraded.tool.TFlangeSensor(1:3, 4), ...
    [0; 0; 0.020], 'AbsTol', 0);
verifyWarningFree(testCase, ...
    @() validateRcmAdmittanceConfig(upgraded));
end

function testAccepted015DegreeStage08ConfigMigratesTo030Degree(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.stage08.MaximumServoTargetStepRad = deg2rad(0.15);

upgraded = scopeguide.runtime.upgradeRcmAdmittanceConfig(cfg);

verifyEqual(testCase, ...
    rad2deg(upgraded.stage08.MaximumServoTargetStepRad), 0.30, ...
    'AbsTol', 10 * eps);
verifyWarningFree(testCase, ...
    @() validateRcmAdmittanceConfig(upgraded));
end

function testInvalidRcmBoundaryOrderingRejected(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.rcm.SoftRadiusM = cfg.rcm.HardRadiusM;
verifyError(testCase, @() validateRcmAdmittanceConfig(cfg), ...
    'scopeguide:config:InvalidRcmBounds');
end

function testLegacyConfigAddsRcmRecoveryShaping(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.qp = rmfield(cfg.qp, ...
    {'RcmRecoveryDeadzoneM', 'RcmRecoveryFullActivationM'});

upgraded = scopeguide.runtime.upgradeRcmAdmittanceConfig(cfg);

verifyEqual(testCase, upgraded.qp.RcmRecoveryDeadzoneM, ...
    0.00025, 'AbsTol', 0);
verifyEqual(testCase, upgraded.qp.RcmRecoveryFullActivationM, ...
    0.00050, 'AbsTol', 0);
verifyWarningFree(testCase, ...
    @() validateRcmAdmittanceConfig(upgraded));
end

function testInvalidRcmRecoveryActivationOrderingRejected(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.qp.RcmRecoveryFullActivationM = ...
    cfg.qp.RcmRecoveryDeadzoneM;
verifyError(testCase, @() validateRcmAdmittanceConfig(cfg), ...
    'scopeguide:config:InvalidRcmRecoveryActivationBounds');
end

function testDefaultMotionAuthorizationDenied(testCase)
cfg = defaultRcmAdmittanceConfig();
authorization = scopeguide.safety.evaluateMotionAuthorization(cfg, "");

verifyFalse(testCase, authorization.Requested);
verifyFalse(testCase, authorization.Allowed);
verifyTrue(testCase, authorization.DoesNotSendHardwareCommands);
verifyTrue(testCase, any(authorization.FailedGates == ...
    "MotionModeSelected"));
end

function testMotionAuthorizationRequiresTypedPhrase(testCase)
cfg = motionReadyConfiguration();
authorization = scopeguide.safety.evaluateMotionAuthorization(cfg, ...
    "WRONG_PHRASE");

verifyFalse(testCase, authorization.Allowed);
verifyTrue(testCase, any(authorization.FailedGates == ...
    "TypedConfirmationMatches"));
end

function testMotionAuthorizationPassesOnlyAllGates(testCase)
cfg = motionReadyConfiguration();
authorization = scopeguide.safety.evaluateMotionAuthorization(cfg, ...
    cfg.robot.MotionConfirmationPhrase);

verifyTrue(testCase, authorization.Requested);
verifyTrue(testCase, authorization.Allowed);
verifyEmpty(testCase, authorization.FailedGates);
end

function testDataContractsAreFailSafe(testCase)
robotState = scopeguide.types.robotState();
forceSample = scopeguide.types.forceSample();
handleSample = scopeguide.types.handleSample();
decision = scopeguide.types.controlDecision();

verifySize(testCase, robotState.JointPositionRad, [6, 1]);
verifyFalse(testCase, robotState.IsValid);
verifySize(testCase, forceSample.RawWrenchSensor, [6, 1]);
verifyFalse(testCase, forceSample.IsValid);
verifyFalse(testCase, handleSample.Enabled);
verifyFalse(testCase, handleSample.IsValid);
verifyFalse(testCase, decision.AllowCommand);
verifyEqual(testCase, decision.QdotCommandRadSec, zeros(6, 1));
verifyEqual(testCase, decision.FsmState, "DISABLED");
end

function testOfflineMainInitializesAndCleansUp(testCase)
result = main_rcm_admittance_drag(GenerateEnvironmentReport=false);

verifyEqual(testCase, result.Status, "stage00_offline_initialized");
verifyTrue(testCase, result.CleanupExecuted);
verifyFalse(testCase, result.HardwareConnectionsCreated);
verifyFalse(testCase, result.MotionCommandsSent);
verifyFalse(testCase, result.MotionAuthorization.Allowed);
verifyEqual(testCase, result.ReportDirectory, "");
end

function testLiveDryRunCannotConnectDuringStage00(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.runtime.Mode = "live_dry_run";
verifyWarningFree(testCase, @() callExpectingError( ...
    @() main_rcm_admittance_drag( ...
        Config=cfg, GenerateEnvironmentReport=false), ...
    'scopeguide:runtime:StageNotImplemented'));
end

function testPhysicalModeStillHasNoImplementation(testCase)
cfg = motionReadyConfiguration();
verifyWarningFree(testCase, @() callExpectingError( ...
    @() main_rcm_admittance_drag( ...
        Config=cfg, ...
        MotionConfirmation=cfg.robot.MotionConfirmationPhrase, ...
        GenerateEnvironmentReport=false), ...
    'scopeguide:runtime:PhysicalMotionNotImplemented'));
end

function cfg = motionReadyConfiguration()
cfg = defaultRcmAdmittanceConfig();
cfg.runtime.Mode = "fixture_motion";
cfg.robot.EnableMotion = true;
cfg.robot.DryRun = false;
cfg.robot.ModelVerified = true;
cfg.robot.ServoTimingVerified = true;
cfg.tool.GeometryVerified = true;
cfg.force.ParametersValidated = true;
cfg.rcm.PointBaseM = [0.4; 0.0; 0.3];
cfg.rcm.PointSource = "manual_base_coordinate";
cfg.rcm.PointSourceDescription = "synthetic test point";
cfg.rcm.FixtureIdentifier = "test_fixture";
cfg.rcm.CalibrationValid = true;
cfg.rcm.BoundsValidated = true;
cfg.safety.ThresholdsValidated = true;
cfg.safety.JointLimitsValidated = true;
validateRcmAdmittanceConfig(cfg);
end

function callExpectingError(action, expectedIdentifier)
try
    action();
catch exception
    if strcmp(exception.identifier, expectedIdentifier)
        return;
    end
    rethrow(exception);
end
error('scopeguide:tests:ExpectedErrorNotThrown', ...
    'Expected error %s was not thrown.', expectedIdentifier);
end
