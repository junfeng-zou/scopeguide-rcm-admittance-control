function tests = testStage08FixtureCommissioning
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
addpath(fullfile(root, 'config'));
addpath(fullfile(root, 'robot'));
addpath(fullfile(root, 'tests'));
testCase.TestData.Root = root;
end

function testCr5ManufacturerJointLimitsAndMargin(testCase)
cfg = defaultRcmAdmittanceConfig();
verifyEqual(testCase, rad2deg(cfg.kinematics.JointLowerLimitRad), ...
    [-360; -360; -160; -360; -360; -360], 'AbsTol', 1e-12);
verifyEqual(testCase, rad2deg(cfg.kinematics.JointUpperLimitRad), ...
    [360; 360; 160; 360; 360; 360], 'AbsTol', 1e-12);
verifyEqual(testCase, rad2deg(cfg.stage08.JointLimitMarginRad), 5, ...
    'AbsTol', 1e-12);
verifyTrue(testCase, cfg.safety.JointLimitsValidated);
end

function testStage08ForceStopThresholdsAndConfirmationPhrase(testCase)
cfg = defaultRcmAdmittanceConfig();
verifyEqual(testCase, cfg.safety.RawForceStopN, 60, 'AbsTol', 0);
verifyEqual(testCase, cfg.safety.FastForceStopN, 40, 'AbsTol', 0);
verifyEqual(testCase, ...
    cfg.stage08.SafetyThresholdConfirmationPhrase, ...
    "CONFIRM_LIMITS_60N_40N_4NM_2NM");
end

function testCurrentDualPivotProfileFixesReviewedParameters(testCase)
sourceFile = fullfile(testCase.TestData.Root, 'resources', ...
    'validated_pivot2_config.mat');
assumeTrue(testCase, isfile(sourceFile));

[cfg, profile] = current_stage08_dual_pivot_config( ...
    SourceAcceptedConfigFile=string(sourceFile));
parameters = ...
    scopeguide.control.deriveForceOnlyAdmittanceParameters(cfg);

verifyEqual(testCase, profile.PivotVirtualMassScale, 0.40, ...
    'AbsTol', 0);
verifyEqual(testCase, cfg.admittance.Pivot.TimeConstantSec, ...
    0.12, 'AbsTol', 10 * eps);
verifyEqual(testCase, rad2deg(cfg.control.RelativePivotLimitRad), ...
    7.5, 'AbsTol', 10 * eps);
verifyEqual(testCase, cfg.rcm.Mode, "soft");
verifyEqual(testCase, cfg.rcm.HardRadiusM, 0.002, 'AbsTol', 0);
verifyEqual(testCase, parameters.Damping(1), ...
    profile.PivotDampingNmSecPerRad, 'AbsTol', 0);
verifyEqual(testCase, parameters.VirtualMass(1), ...
    parameters.Damping(1) * 0.12, 'RelTol', 2e-15);
verifyEqual(testCase, cfg.meta.ActiveTuningProfile, profile.Name);
end

function testCurrentFull3DofProfileMatchesLastUsedParameters(testCase)
sourceFile = fullfile(testCase.TestData.Root, 'resources', ...
    'validated_base_config.mat');
assumeTrue(testCase, isfile(sourceFile));

[cfg, profile] = current_stage08_full_3dof_config( ...
    SourceAcceptedConfigFile=string(sourceFile));
parameters = ...
    scopeguide.control.deriveForceOnlyAdmittanceParameters(cfg);

verifyEqual(testCase, cfg.stage08.DofMode, "full_3dof");
verifyTrue(testCase, any(string( ...
    cfg.stage08.CompletedDofModes(:)) == "dual_pivot"));
verifyTrue(testCase, cfg.stage08.ProfileApplied);
verifyEqual(testCase, cfg.stage08.SpeedScale, 1.2, 'AbsTol', 0);
verifyEqual(testCase, rad2deg( ...
    cfg.admittance.Pivot.TargetSteadySpeedRadSec), ...
    2.133333333333333, 'AbsTol', 1e-12);
verifyEqual(testCase, cfg.admittance.Pivot.TimeConstantSec, ...
    0.08, 'AbsTol', 10 * eps);
verifyEqual(testCase, rad2deg(cfg.control.PivotMaxRadSec), ...
    3.0, 'AbsTol', 1e-12);
verifyEqual(testCase, rad2deg(cfg.control.RelativePivotLimitRad), ...
    7.5, 'AbsTol', 1e-12);
verifyEqual(testCase, ...
    rad2deg(cfg.control.RelativePivotRecoveryToleranceRad), ...
    0.3, 'AbsTol', 1e-12);
verifyEqual(testCase, ...
    cfg.admittance.Insertion.TargetSteadySpeedMSec, ...
    0.006, 'AbsTol', 0);
verifyEqual(testCase, ...
    cfg.admittance.Insertion.TimeConstantSec, 0.15, 'AbsTol', 0);
verifyEqual(testCase, cfg.control.InsertionMaxMSec, ...
    0.008, 'AbsTol', 0);
verifyEqual(testCase, cfg.control.RelativeInsertionLimitM, ...
    0.030, 'AbsTol', 0);
verifyEqual(testCase, ...
    1e3 * cfg.control.RelativeInsertionRecoveryToleranceM, ...
    1.0, 'AbsTol', 1e-12);
verifyEqual(testCase, cfg.rcm.Mode, "soft");
verifyEqual(testCase, cfg.rcm.HardRadiusM, 0.002, 'AbsTol', 0);
verifyEqual(testCase, parameters.Damping(1), ...
    profile.PivotDampingNmSecPerRad, 'AbsTol', 0);
verifyEqual(testCase, parameters.Damping(3), 6 / 0.006, ...
    'RelTol', 2e-15);
verifyEqual(testCase, parameters.VirtualMass(3), ...
    parameters.Damping(3) * 0.15, 'RelTol', 2e-15);

configured = scopeguide.runtime.configureStage08Commissioning( ...
    cfg, "full_3dof");
verifyEqual(testCase, configured.control.PivotMaxRadSec, ...
    cfg.control.PivotMaxRadSec, 'AbsTol', 0);
verifyEqual(testCase, configured.control.InsertionMaxMSec, ...
    cfg.control.InsertionMaxMSec, 'AbsTol', 0);
verifyEqual(testCase, ...
    configured.admittance.Pivot.TargetSteadySpeedRadSec, ...
    cfg.admittance.Pivot.TargetSteadySpeedRadSec, 'AbsTol', 0);
verifyEqual(testCase, ...
    configured.admittance.Insertion.TargetSteadySpeedMSec, ...
    cfg.admittance.Insertion.TargetSteadySpeedMSec, 'AbsTol', 0);
end

function testStage08ProfileScalesAllMotionLimits(testCase)
base = defaultRcmAdmittanceConfig();
cfg = scopeguide.runtime.configureStage08Commissioning( ...
    base, "insertion_only");
verifyEqual(testCase, cfg.runtime.Mode, "fixture_motion");
verifyEqual(testCase, cfg.runtime.ControlRateHz, 20, 'AbsTol', 0);
verifyEqual(testCase, cfg.runtime.NominalDtSec, 1 / 20, ...
    'AbsTol', 10 * eps);
verifyTrue(testCase, cfg.robot.EnableMotion);
verifyFalse(testCase, cfg.robot.DryRun);
verifyEqual(testCase, cfg.control.PivotMaxRadSec, ...
    base.stage08.SpeedScale * ...
    base.stage08.PivotMaximumSpeedMultiplier * ...
    base.control.PivotMaxRadSec, ...
    'AbsTol', 1e-15);
verifyEqual(testCase, ...
    cfg.admittance.Pivot.TargetSteadySpeedRadSec, ...
    base.stage08.PivotAdmittanceSpeedScale * ...
    base.stage08.PivotTargetSpeedMultiplier * ...
    base.admittance.Pivot.TargetSteadySpeedRadSec, ...
    'AbsTol', 1e-15);
verifyTrue(testCase, cfg.stage08.PivotSpeedRetuneApplied);
verifyEqual(testCase, cfg.control.InsertionMaxMSec, ...
    base.stage08.SpeedScale * base.control.InsertionMaxMSec, ...
    'AbsTol', 1e-15);
verifyEqual(testCase, cfg.control.MaximumJointCommandRadSec, ...
    base.stage08.SpeedScale * base.control.MaximumJointCommandRadSec, ...
    'AbsTol', 1e-15);
cfgSecond = scopeguide.runtime.configureStage08Commissioning( ...
    cfg, "pivot1_only");
verifyEqual(testCase, cfgSecond.runtime.ControlRateHz, 20, 'AbsTol', 0);
verifyEqual(testCase, cfgSecond.runtime.NominalDtSec, 1 / 20, ...
    'AbsTol', 10 * eps);
verifyEqual(testCase, cfgSecond.control.MaximumJointCommandRadSec, ...
    cfg.control.MaximumJointCommandRadSec, 'AbsTol', 1e-15);
verifyEqual(testCase, cfgSecond.control.PivotMaxRadSec, ...
    cfg.control.PivotMaxRadSec, 'AbsTol', 1e-15);
verifyEqual(testCase, ...
    cfgSecond.admittance.Pivot.TargetSteadySpeedRadSec, ...
    cfg.admittance.Pivot.TargetSteadySpeedRadSec, 'AbsTol', 1e-15);
end

function testSaved120PercentProfileMigratesPivotSpeedsOnce(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.stage08 = rmfield(cfg.stage08, ...
    {'PivotAdmittanceSpeedScale', 'PivotTargetSpeedMultiplier', ...
     'PivotMaximumSpeedMultiplier', ...
     'PivotSpeedRetuneApplied', 'PivotSpeedRetuneRevision'});
cfg.stage08.SpeedScale = 1.2;
cfg.stage08.ProfileApplied = true;
cfg.control.PivotMaxRadSec = deg2rad(1.2);
cfg.admittance.Pivot.TargetSteadySpeedRadSec = deg2rad(0.6);

upgraded = scopeguide.runtime.upgradeRcmAdmittanceConfig(cfg);
verifyEqual(testCase, rad2deg(upgraded.control.PivotMaxRadSec), ...
    3.0, 'AbsTol', 1e-12);
verifyEqual(testCase, rad2deg( ...
    upgraded.admittance.Pivot.TargetSteadySpeedRadSec), ...
    2.133333333333333, 'AbsTol', 1e-12);
verifyTrue(testCase, upgraded.stage08.PivotSpeedRetuneApplied);
verifyEqual(testCase, upgraded.stage08.PivotAdmittanceSpeedScale, ...
    1.6, 'AbsTol', 0);
verifyEqual(testCase, upgraded.stage08.PivotSpeedRetuneRevision, 3);

upgradedAgain = scopeguide.runtime.upgradeRcmAdmittanceConfig(upgraded);
verifyEqual(testCase, upgradedAgain.control.PivotMaxRadSec, ...
    upgraded.control.PivotMaxRadSec, 'AbsTol', 0);
verifyEqual(testCase, ...
    upgradedAgain.admittance.Pivot.TargetSteadySpeedRadSec, ...
    upgraded.admittance.Pivot.TargetSteadySpeedRadSec, 'AbsTol', 0);
verifyWarningFree(testCase, ...
    @() validateRcmAdmittanceConfig(upgradedAgain));
end

function testFirstPivotRetuneMigratesToDoubledRetuneOnce(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.stage08 = rmfield(cfg.stage08, 'PivotSpeedRetuneRevision');
cfg.stage08.SpeedScale = 1.2;
cfg.stage08.ProfileApplied = true;
cfg.stage08.PivotSpeedRetuneApplied = true;
cfg.stage08.PivotTargetSpeedMultiplier = 4 / 3;
cfg.stage08.PivotMaximumSpeedMultiplier = 1.25;
cfg.control.PivotMaxRadSec = deg2rad(1.5);
cfg.admittance.Pivot.TargetSteadySpeedRadSec = deg2rad(0.8);

upgraded = scopeguide.runtime.upgradeRcmAdmittanceConfig(cfg);
verifyEqual(testCase, rad2deg(upgraded.control.PivotMaxRadSec), ...
    3.0, 'AbsTol', 1e-12);
verifyEqual(testCase, rad2deg( ...
    upgraded.admittance.Pivot.TargetSteadySpeedRadSec), ...
    2.133333333333333, 'AbsTol', 1e-12);
verifyEqual(testCase, upgraded.stage08.PivotMaximumSpeedMultiplier, ...
    2.5, 'AbsTol', 0);
verifyEqual(testCase, upgraded.stage08.PivotTargetSpeedMultiplier, ...
    8 / 3, 'AbsTol', 10 * eps);
verifyEqual(testCase, upgraded.stage08.PivotAdmittanceSpeedScale, ...
    1.6, 'AbsTol', 0);
verifyEqual(testCase, upgraded.stage08.PivotSpeedRetuneRevision, 3);

upgradedAgain = scopeguide.runtime.upgradeRcmAdmittanceConfig(upgraded);
verifyEqual(testCase, upgradedAgain.control.PivotMaxRadSec, ...
    upgraded.control.PivotMaxRadSec, 'AbsTol', 0);
verifyEqual(testCase, ...
    upgradedAgain.admittance.Pivot.TargetSteadySpeedRadSec, ...
    upgraded.admittance.Pivot.TargetSteadySpeedRadSec, 'AbsTol', 0);
end

function testRevision2PivotRetuneMigratesFeelScaleOnce(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.stage08 = rmfield(cfg.stage08, 'PivotAdmittanceSpeedScale');
cfg.stage08.SpeedScale = 1.2;
cfg.stage08.ProfileApplied = true;
cfg.stage08.PivotSpeedRetuneApplied = true;
cfg.stage08.PivotSpeedRetuneRevision = 2;
cfg.stage08.PivotTargetSpeedMultiplier = 8 / 3;
cfg.stage08.PivotMaximumSpeedMultiplier = 2.5;
cfg.control.PivotMaxRadSec = deg2rad(3.0);
cfg.admittance.Pivot.TargetSteadySpeedRadSec = deg2rad(1.6);

upgraded = scopeguide.runtime.upgradeRcmAdmittanceConfig(cfg);
verifyEqual(testCase, rad2deg(upgraded.control.PivotMaxRadSec), ...
    3.0, 'AbsTol', 1e-12);
verifyEqual(testCase, rad2deg( ...
    upgraded.admittance.Pivot.TargetSteadySpeedRadSec), ...
    2.133333333333333, 'AbsTol', 1e-12);
verifyEqual(testCase, upgraded.stage08.PivotAdmittanceSpeedScale, ...
    1.6, 'AbsTol', 0);
verifyEqual(testCase, upgraded.stage08.PivotSpeedRetuneRevision, 3);

upgradedAgain = scopeguide.runtime.upgradeRcmAdmittanceConfig(upgraded);
verifyEqual(testCase, upgradedAgain.control.PivotMaxRadSec, ...
    upgraded.control.PivotMaxRadSec, 'AbsTol', 0);
verifyEqual(testCase, ...
    upgradedAgain.admittance.Pivot.TargetSteadySpeedRadSec, ...
    upgraded.admittance.Pivot.TargetSteadySpeedRadSec, 'AbsTol', 0);
end

function testStage08SpeedScaleAllowsFourPointFiveAndRejectsLargerValues(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.stage08.SpeedScale = 4.5;
validateRcmAdmittanceConfig(cfg);
configured = scopeguide.runtime.configureStage08Commissioning( ...
    cfg, "insertion_only");
authorization = ...
    scopeguide.safety.evaluateStage08CommissioningAuthorization( ...
        configured, validConfirmations(configured));
verifyTrue(testCase, authorization.Gates.SpeedScaleConservative);

cfg.stage08.SpeedScale = 4.5 + eps(4.5);
verifyError(testCase, @() validateRcmAdmittanceConfig(cfg), ...
    'scopeguide:config:UnsafeStage08SpeedScale');
end

function testDofMasksAreExact(testCase)
verifyEqual(testCase, scopeguide.control.stage08DofMask("hold"), ...
    logical([0; 0; 0]));
verifyEqual(testCase, ...
    scopeguide.control.stage08DofMask("insertion_only"), ...
    logical([0; 0; 1]));
verifyEqual(testCase, ...
    scopeguide.control.stage08DofMask("pivot1_only"), ...
    logical([1; 0; 0]));
verifyEqual(testCase, ...
    scopeguide.control.stage08DofMask("pivot2_only"), ...
    logical([0; 1; 0]));
verifyEqual(testCase, ...
    scopeguide.control.stage08DofMask("dual_pivot"), ...
    logical([1; 1; 0]));
verifyEqual(testCase, ...
    scopeguide.control.stage08DofMask("full_3dof"), ...
    logical([1; 1; 1]));
end

function testAuthorizationFailsClosedAndAllowsInsertionBootstrap(testCase)
cfg = stage08Config("insertion_only");
bad = confirmations();
authorization = ...
    scopeguide.safety.evaluateStage08CommissioningAuthorization(cfg, bad);
verifyFalse(testCase, authorization.CommissioningAllowed);
verifyTrue(testCase, any(authorization.FailedGates == ...
    "MotionPhraseMatches"));

good = validConfirmations(cfg);
authorization = ...
    scopeguide.safety.evaluateStage08CommissioningAuthorization(cfg, good);
verifyTrue(testCase, authorization.CommissioningAllowed);
verifyTrue(testCase, authorization.ServoTimingBootstrapUsed);
verifyTrue(testCase, ...
    authorization.SafetyThresholdRuntimeConfirmationUsed);
verifyTrue(testCase, authorization.KeyboardInputIsNotPhysicalDeadman);
end

function testServoBootstrapCannotOpenPivot(testCase)
cfg = stage08Config("pivot1_only");
authorization = ...
    scopeguide.safety.evaluateStage08CommissioningAuthorization( ...
    cfg, validConfirmations(cfg));
verifyFalse(testCase, authorization.CommissioningAllowed);
verifyTrue(testCase, any(authorization.FailedGates == ...
    "ServoTimingVerifiedOrBootstrap"));
end

function testRobotAdapterSendsGuardedRadiansAsDegrees(testCase)
cfg = stage08Config("insertion_only");
authorization = scopeguide.safety.evaluateStage08CommissioningAuthorization( ...
    cfg, validConfirmations(cfg));
backend = MockDobotMotionBackend(validSnapshot());
adapter = scopeguide.io.RobotAdapter(cfg, backend);
cleanup = onCleanup(@() adapter.disconnect());
adapter.connectForMotion(authorization);
verifyEqual(testCase, backend.EnableCallCount, uint64(1));
verifyEqual(testCase, backend.LastEnablePayloadKg, ...
    cfg.stage08.EnablePayloadKg, 'AbsTol', 1e-12);
verifyEqual(testCase, backend.SetSpeedRatioCallCount, uint64(1));
verifyEqual(testCase, backend.LastSpeedRatioPercent, ...
    cfg.stage08.SpeedFactorPercent, 'AbsTol', 1e-12);
verifyTrue(testCase, adapter.ProgrammaticEnableConfirmed);
verifyTrue(testCase, adapter.ProgrammaticSpeedFactorConfirmed);
state = adapter.readState();
target = state.JointPositionRad;
target(2) = target(2) + deg2rad(0.05);
result = adapter.sendServoTarget(target, authorization);
verifyTrue(testCase, result.Submitted);
verifyTrue(testCase, result.Accepted);
verifyTrue(testCase, result.ResponseConfirmed);
verifyEqual(testCase, result.Transport.Mode, ...
    "legacy_immediate_optional_response");
verifyFalse(testCase, result.Transport.ResponseExpected);
verifyEqual(testCase, result.Transport.ReplyDeadlinePolicy, ...
    "single_immediate_read_no_wait");
verifyEqual(testCase, result.Transport.PendingResponseCount, 0);
verifyEqual(testCase, backend.ServoJCallCount, uint64(1));
verifyEqual(testCase, backend.LastServoTargetDeg(:), ...
    rad2deg(target), 'AbsTol', 1e-9);
verifyEqual(testCase, backend.LastServoTimeSec, ...
    cfg.runtime.NominalDtSec, 'AbsTol', 1e-12);
verifyEqual(testCase, backend.LastServoLookaheadTime, ...
    cfg.robot.ServoJLookaheadTime, 'AbsTol', 1e-12);
verifyEqual(testCase, backend.LastServoGain, ...
    cfg.robot.ServoJGain, 'AbsTol', 1e-12);
verifyEqual(testCase, result.ServoTimeSec, ...
    cfg.runtime.NominalDtSec, 'AbsTol', 1e-12);
verifyEqual(testCase, result.ServoLookaheadTime, ...
    cfg.robot.ServoJLookaheadTime, 'AbsTol', 1e-12);
verifyEqual(testCase, result.ServoGain, ...
    cfg.robot.ServoJGain, 'AbsTol', 1e-12);
verifyTrue(testCase, contains(backend.LastServoCommand, ...
    "t=0.050000,lookahead_time=60.000000,gain=400.000000"));
verifyEqual(testCase, adapter.CommandSentCount, uint64(1));
verifyFalse(testCase, backend.LogEveryCommand);
for index = 1:10
    diagnostics = adapter.pollMotionCommandStatus();
    verifyFalse(testCase, diagnostics.ResponseExpected);
    verifyEqual(testCase, diagnostics.ReplyDeadlinePolicy, ...
        "single_immediate_read_no_wait");
    verifyEqual(testCase, diagnostics.PendingResponseCount, 0);
    verifyEqual(testCase, diagnostics.MaximumAllowedPendingResponses, 0);
    verifyTrue(testCase, isnan(diagnostics.HardResponseTimeoutSec));
    verifyEqual(testCase, diagnostics.ErrorCount, 0);
    verifyEqual(testCase, diagnostics.FaultIdentifier, "");
end
clear cleanup;
verifyEqual(testCase, backend.DisableCallCount, uint64(1));
verifyEqual(testCase, backend.Snapshot.robotMode, "DISABLED");
end

function testProgrammaticEnableRequiresDisableOnExit(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.stage08.DisableRobotOnExit = false;
verifyError(testCase, @() validateRcmAdmittanceConfig(cfg), ...
    'scopeguide:config:Stage08DisableOnExitRequired');
end

function testRealBackendUsesValidatedLegacyServoJ(testCase)
source = fileread(fullfile(testCase.TestData.Root, ...
    'robot', 'ZJFDobotCR5.m'));

verifyFalse(testCase, contains(source, ...
    'AsynchronousCommandResponseTimeout'));
verifyFalse(testCase, contains(source, 'PendingMoveResponses'));
verifyFalse(testCase, contains(source, ...
    'checkAsynchronousMoveTimeout'));
servoStart = strfind(source, ...
    'function response = ServoJ(self,joints,t,lookahead_time,gain)');
servoEnd = strfind(source, '% --- 私有方法 ---');
verifyNotEmpty(testCase, servoStart);
verifyNotEmpty(testCase, servoEnd);
servoSource = source(servoStart(1):servoEnd(1) - 1);
verifyTrue(testCase, contains(servoSource, ...
    'cmd = sprintf("ServoJ(%f,%f,%f,%f,%f,%f,t=%f,'));
verifyTrue(testCase, contains(servoSource, ...
    'response = self.sendMoveCommand(cmd)'));
verifyFalse(testCase, contains(servoSource, ...
    'sendMoveCommandNonblocking'));
verifyFalse(testCase, contains(source, 'ServoJNonblocking'));
verifyTrue(testCase, contains(source, ...
    'if self.MoveClient.NumBytesAvailable > 0'));
verifyFalse(testCase, contains(source, ...
    'waitForV3CommandResponse'));
end

function testRobotAdapterPreservesMotionTransportDiagnostics(testCase)
cfg = stage08Config("insertion_only");
authorization = scopeguide.safety.evaluateStage08CommissioningAuthorization( ...
    cfg, validConfirmations(cfg));
backend = MockDobotMotionBackend(validSnapshot());
backend.ThrowOnServo = true;
adapter = scopeguide.io.RobotAdapter(cfg, backend);
cleanup = onCleanup(@() adapter.disconnect());
adapter.connectForMotion(authorization);
state = adapter.readState();

verifyError(testCase, @() adapter.sendServoTarget( ...
    state.JointPositionRad, authorization), ...
    'scopeguide:tests:InjectedServoFailure');
diagnostics = adapter.getLastMotionCommandDiagnostics();
verifyTrue(testCase, diagnostics.WriteFailed);
verifyFalse(testCase, diagnostics.ResponseExpected);
verifyEqual(testCase, diagnostics.PendingResponseCount, 0);
verifyEqual(testCase, adapter.CommandAttemptCount, uint64(1));
verifyEqual(testCase, adapter.CommandSentCount, uint64(0));
verifyEqual(testCase, adapter.CommandFailureCount, uint64(1));
clear cleanup;
end

function testServoTargetStepAndJointMarginRejectBeforeSend(testCase)
cfg = stage08Config("insertion_only");
authorization = scopeguide.safety.evaluateStage08CommissioningAuthorization( ...
    cfg, validConfirmations(cfg));
backend = MockDobotMotionBackend(validSnapshot());
adapter = scopeguide.io.RobotAdapter(cfg, backend);
cleanup = onCleanup(@() adapter.disconnect());
adapter.connectForMotion(authorization);
state = adapter.readState();
tooFar = state.JointPositionRad;
tooFar(1) = tooFar(1) + 2 * cfg.stage08.MaximumServoTargetStepRad;
verifyError(testCase, @() adapter.sendServoTarget(tooFar, authorization), ...
    'scopeguide:robot:ServoTargetStepTooLarge');
verifyEqual(testCase, backend.ServoJCallCount, uint64(0));
clear cleanup;
end

function testSubsequentServoStepUsesLastSubmittedCommand(testCase)
cfg = stage08Config("insertion_only");
authorization = scopeguide.safety.evaluateStage08CommissioningAuthorization( ...
    cfg, validConfirmations(cfg));
backend = MockDobotMotionBackend(validSnapshot());
adapter = scopeguide.io.RobotAdapter(cfg, backend);
cleanup = onCleanup(@() adapter.disconnect());
adapter.connectForMotion(authorization);
state = adapter.readState();

% The mock feedback deliberately remains at zero, representing transport
% and servo lag.  Each of the first two command increments is legal even
% though the second target is more than one configured step ahead of feedback.
first = state.JointPositionRad;
first(1) = first(1) + 0.8 * cfg.stage08.MaximumServoTargetStepRad;
adapter.sendServoTarget(first, authorization);
second = first;
second(1) = second(1) + 0.8 * cfg.stage08.MaximumServoTargetStepRad;
adapter.sendServoTarget(second, authorization);
verifyEqual(testCase, backend.ServoJCallCount, uint64(2));
verifyEqual(testCase, adapter.LastCommandTargetRad, second, ...
    'AbsTol', 1e-15);

% A real command-to-command jump above 0.30 deg remains fail-closed.
jump = second;
jump(1) = jump(1) + 1.1 * cfg.stage08.MaximumServoTargetStepRad;
verifyError(testCase, @() adapter.sendServoTarget(jump, authorization), ...
    'scopeguide:robot:ServoTargetStepTooLarge');
verifyEqual(testCase, backend.ServoJCallCount, uint64(2));
clear cleanup;
end

function testFeedbackHoldUsesFreshFeedbackNotCommandStep(testCase)
cfg = stage08Config("insertion_only");
authorization = scopeguide.safety.evaluateStage08CommissioningAuthorization( ...
    cfg, validConfirmations(cfg));
backend = MockDobotMotionBackend(validSnapshot());
adapter = scopeguide.io.RobotAdapter(cfg, backend);
cleanup = onCleanup(@() adapter.disconnect());
adapter.connectForMotion(authorization);
state = adapter.readState();

% Build 0.30 deg command-to-feedback lag using four legal 0.075 deg
% command increments. The release hold targets fresh feedback and must
% not be rejected by the ordinary 0.30 deg command-step guard.
target = state.JointPositionRad;
for index = 1:4
    target(1) = target(1) + ...
        0.25 * cfg.stage08.MaximumServoTargetStepRad;
    adapter.sendServoTarget(target, authorization);
end
backend.Snapshot.jointAnglesDeg = zeros(1, 6);
adapter.readState();
result = adapter.sendCurrentPositionHold(authorization);
verifyTrue(testCase, result.Submitted);
verifyEqual(testCase, backend.ServoJCallCount, uint64(5));
verifyEqual(testCase, backend.LastServoTargetDeg, zeros(1, 6), ...
    'AbsTol', 1e-12);
clear cleanup;
end

function testRobotModeRejectsServoBeforeSend(testCase)
cfg = stage08Config("insertion_only");
authorization = scopeguide.safety.evaluateStage08CommissioningAuthorization( ...
    cfg, validConfirmations(cfg));
snapshot = validSnapshot();
snapshot.robotMode = "DISABLED";
backend = MockDobotMotionBackend(snapshot);
backend.EnableChangesMode = false;
adapter = scopeguide.io.RobotAdapter(cfg, backend);
cleanup = onCleanup(@() adapter.disconnect());
verifyError(testCase, @() adapter.connectForMotion(authorization), ...
    'scopeguide:robot:MotionConnectionFailed');
verifyEqual(testCase, backend.ServoJCallCount, uint64(0));
clear cleanup;
end

function testDryRunStillCannotSend(testCase)
cfg = defaultRcmAdmittanceConfig();
backend = MockDobotBackend(validSnapshot());
adapter = scopeguide.io.RobotAdapter(cfg, backend);
verifyError(testCase, @() adapter.sendServoTarget( ...
    zeros(6, 1), struct()), 'scopeguide:robot:DryRunMotionRejected');
verifyEqual(testCase, backend.ServoJCallCount, uint64(0));
end

function testStage8SourceUsesGuardedProgrammaticEnableAndDisable(testCase)
runner = fileread(fullfile(testCase.TestData.Root, ...
    'run_stage08_fixture_commissioning.m'));
worker = fileread(fullfile(testCase.TestData.Root, ...
    '+scopeguide', '+runtime', 'runStage08FixtureWorker.m'));
adapter = fileread(fullfile(testCase.TestData.Root, ...
    '+scopeguide', '+io', 'RobotAdapter.m'));
source = [runner, newline, worker, newline, adapter];
verifyTrue(testCase, contains(adapter, ...
    'obj.Backend.Enable(stage08.EnablePayloadKg)'));
verifyTrue(testCase, contains(adapter, ...
    'obj.Backend.SetSpeedRatio(stage08.SpeedFactorPercent)'));
verifyTrue(testCase, contains(adapter, 'obj.Backend.Disable()'));
verifyTrue(testCase, contains(adapter, ...
    'obj.waitForRobotMode("DISABLED"'));
verifyEqual(testCase, count(source, 'robot.sendServoTarget('), 1);
verifyTrue(testCase, contains(worker, '"qdotCmd1"'));
verifyTrue(testCase, contains(worker, '"qdotActual1"'));
verifyTrue(testCase, contains(worker, '"wrenchSafetyStopReasons"'));
verifyTrue(testCase, contains(worker, ...
    '"COMMAND_FAULT_MONITOR_ONLY"'));
verifyTrue(testCase, contains(worker, ...
    'command_fault_diagnostics.json'));
verifyTrue(testCase, contains(worker, ...
    'NoAdditionalMotionAttemptsAfterCommandFault'));
verifyTrue(testCase, contains(worker, ...
    'robot.pollMotionCommandStatus()'));
verifyTrue(testCase, contains(worker, 'ServoSendDurationSec'));
end

function testWrenchSafetyReasonIsPreservedByFsm(testCase)
cfg = defaultRcmAdmittanceConfig();
fsm = scopeguide.control.HandleEnableStateMachine(cfg);
handle = scopeguide.types.handleSample();
handle.Enabled = true;
handle.IsValid = true;
handle.SampleAgeSec = 0;
handle.Source = "stage08_test";
handle.SupportsPhysicalMotion = true;
handle.StatusCode = "OK";
safety = healthySafety();
safety.WrenchSafetyHealthy = false;
safety.WrenchSafetyStopCode = "FAST_FORCE_STOP";
status = fsm.step(handle, safety, 0.01);
verifyEqual(testCase, status.State, "FAULT");
verifyEqual(testCase, status.FaultCode, "FAST_FORCE_STOP");
end

function testForceQualityReasonIsPreservedByFsm(testCase)
cfg = defaultRcmAdmittanceConfig();
fsm = scopeguide.control.HandleEnableStateMachine(cfg);
handle = scopeguide.types.handleSample();
handle.Enabled = true;
handle.IsValid = true;
handle.SampleAgeSec = 0;
handle.Source = "stage08_test";
handle.SupportsPhysicalMotion = true;
handle.StatusCode = "OK";
safety = healthySafety();
safety.ForceFresh = false;
safety.ForceStatusCode = "FORCE_STALE_SAMPLE";
status = fsm.step(handle, safety, 0.01);
verifyEqual(testCase, status.State, "FAULT");
verifyEqual(testCase, status.FaultCode, "FORCE_STALE_SAMPLE");
end

function testPhysicalDashboardShowsJointSpeedDiagnostics(testCase)
originalVisibility = get(groot, 'defaultFigureVisible');
visibilityCleanup = onCleanup(@() set( ...
    groot, 'defaultFigureVisible', originalVisibility));
set(groot, 'defaultFigureVisible', 'off');
cfg = stage08Config("insertion_only");
dashboard = scopeguide.ui.Stage07DryRunDashboard( ...
    cfg, 15, EnablePlot=true, PhysicalMotionMode=true, ...
    DofMode="insertion_only");
dashboardCleanup = onCleanup(@() dashboard.close());
snapshot = physicalDisplaySnapshot();

dashboard.updateFromSnapshot(snapshot);

verifyTrue(testCase, dashboard.isOpen());
verifyEqual(testCase, numel(findall(dashboard.Figure, ...
    'Type', 'animatedline')), 24);
verifyTrue(testCase, contains(string(dashboard.Figure.Name), ...
    "Stage 8"));
clear dashboardCleanup visibilityCleanup;
end

function testPhysicalDashboardCanFreezeAndRetainFigure(testCase)
originalVisibility = get(groot, 'defaultFigureVisible');
visibilityCleanup = onCleanup(@() set( ...
    groot, 'defaultFigureVisible', originalVisibility));
set(groot, 'defaultFigureVisible', 'off');
cfg = stage08Config("insertion_only");
dashboard = scopeguide.ui.Stage07DryRunDashboard( ...
    cfg, 15, EnablePlot=true, PhysicalMotionMode=true, ...
    DofMode="insertion_only");
snapshot = physicalDisplaySnapshot();
snapshot.TimeSec = 23;
dashboard.updateFromSnapshot(snapshot);
figureHandle = dashboard.Figure;
figureCleanup = onCleanup(@() deleteIfGraphics(figureHandle));

verifyTrue(testCase, dashboard.freezeForReview("DURATION_COMPLETE"));
verifyEmpty(testCase, get(figureHandle, 'WindowKeyPressFcn'));
verifyEmpty(testCase, get(figureHandle, 'WindowButtonUpFcn'));
delete(dashboard);
verifyTrue(testCase, isgraphics(figureHandle));
axesHandles = findall(figureHandle, 'Type', 'axes');
fullHistoryAxesCount = 0;
for index = 1:numel(axesHandles)
    limits = xlim(axesHandles(index));
    if abs(limits(2) - 23) < 1e-9
        verifyEqual(testCase, limits, [0, 23], 'AbsTol', 1e-9);
        fullHistoryAxesCount = fullHistoryAxesCount + 1;
    else
        % The mouse-enable pad is also implemented as an axes and keeps
        % its fixed normalized [0,1] interaction coordinates.
        verifyEqual(testCase, limits, [0, 1], 'AbsTol', 1e-9);
    end
end
verifyEqual(testCase, fullHistoryAxesCount, 4);

clear figureCleanup visibilityCleanup;
end

function testAcceptedInsertionProducesNextPhaseConfig(testCase)
directory = string(tempname);
mkdir(directory);
cleanup = onCleanup(@() rmdir(directory, 's'));
cfg = stage08Config("insertion_only");
authorization = scopeguide.safety.evaluateStage08CommissioningAuthorization( ...
    cfg, validConfirmations(cfg));
summary = fakeAcceptedSummary("insertion_only");
save(fullfile(directory, 'stage08_worker_result.mat'), ...
    'summary', 'cfg', 'authorization');
[cfgAccepted, acceptance] = accept_stage08_fixture_result( ...
    directory, DirectionCorrect=true, ...
    PhysicalRcmWithinHardBound=true, PhysicalRcmMaximumMm=0.8, ...
    ReleaseStopWasImmediate=true, NoUnexpectedMotion=true, ...
    Confirmation="ACCEPT_STAGE8_PHASE");
verifyTrue(testCase, acceptance.Accepted);
verifyTrue(testCase, cfgAccepted.robot.ServoTimingVerified);
verifyEqual(testCase, cfgAccepted.stage08.CompletedDofModes, ...
    "insertion_only");
clear cleanup;
end

function cfg = stage08Config(mode)
cfg = scopeguide.runtime.configureStage08Commissioning( ...
    defaultRcmAdmittanceConfig(), mode);
end

function value = confirmations()
value = struct('MotionPhrase', "", 'SetupPhrase', "", ...
    'SafetyThresholdPhrase', "", 'SecondObserverPresent', false, ...
    'FixtureAndClearanceConfirmed', false);
end

function value = validConfirmations(cfg)
value = struct( ...
    'MotionPhrase', cfg.robot.MotionConfirmationPhrase, ...
    'SetupPhrase', cfg.stage08.SetupConfirmationPhrase, ...
    'SafetyThresholdPhrase', ...
        cfg.stage08.SafetyThresholdConfirmationPhrase, ...
    'SecondObserverPresent', true, ...
    'FixtureAndClearanceConfirmed', true);
end

function snapshot = validSnapshot()
snapshot = struct();
snapshot.feedbackSequence = uint64(10);
snapshot.hostMonotonicSec = 1.0;
snapshot.feedbackAgeSec = 0.001;
snapshot.invalidFeedbackByteCount = uint64(0);
snapshot.robotMode = "ENABLE";
snapshot.currentSpeedRatio = 20;
snapshot.jointAnglesDeg = zeros(1, 6);
snapshot.cartesianPose = zeros(1, 6);
snapshot.actualJointSpeedsDegSec = zeros(1, 6);
snapshot.actualTCPSpeed = zeros(1, 6);
snapshot.actualQuaternionWxyz = [1, 0, 0, 0];
end

function summary = fakeAcceptedSummary(mode)
summary = struct();
summary.DofMode = string(mode);
summary.FaultIdentifier = "";
summary.RobotCommandFailureCount = 0;
summary.CommandChannelHealthyAtExit = true;
summary.TargetFeedbackErrorRad = struct('Maximum', deg2rad(0.1));
summary.RcmErrorM = struct('Maximum', 0.0008);
summary.StopLatencySec = struct('Maximum', 0.05);
summary.StopLatencyWithinLimit = true;
summary.ServoTimingCandidatePassed = true;
summary.ProgrammaticEnableConfirmed = true;
summary.SpeedFactorConfirmed = true;
summary.ProgrammaticDisableConfirmed = true;
summary.DisableFaultIdentifier = "";
end

function safety = healthySafety()
safety = scopeguide.types.enableSafetyStatus();
safety.NeutralWrench = true;
safety.ForceFresh = true;
safety.ForceStatusCode = "OK";
safety.ForceControlReady = true;
safety.WrenchSafetyHealthy = true;
safety.WrenchSafetyStopCode = "";
safety.RobotFresh = true;
safety.QpHealthy = true;
safety.ControlPeriodHealthy = true;
safety.MotionGatesSatisfied = true;
safety.CommandChannelHealthy = true;
safety.TrackingHealthy = true;
end

function snapshot = physicalDisplaySnapshot()
snapshot = struct();
snapshot.TimeSec = 1;
snapshot.BaselineReady = true;
snapshot.ControlForceTool = [1; 2; 3];
snapshot.ControlWrenchToolAtSensorOrigin = [1; 2; 3; 0.1; 0.2; 0.3];
snapshot.DesiredGeneralizedVelocity = [0.01; -0.01; 0.001];
snapshot.AchievedGeneralizedVelocity = [0.009; -0.009; 0.0009];
snapshot.CurrentRcmErrorM = 0.0002;
snapshot.PredictedRcmErrorM = 0.0003;
snapshot.EnableState = "ENABLED";
snapshot.CommandScale = 1;
snapshot.QpStatusCode = "SOLVED";
snapshot.FaultCode = "";
snapshot.InjectionStatusCode = "NONE";
snapshot.MotionCommandSent = true;
snapshot.JointVelocityCommandRadSec = deg2rad((1:6).');
snapshot.JointVelocityActualRadSec = deg2rad((0.5:0.5:3).');
snapshot.JointVelocityQpCandidateRadSec = deg2rad((2:2:12).');
snapshot.ForceStatusCode = "OK";
snapshot.WrenchSafetyStopCode = "";
end

function deleteIfGraphics(handle)
if isgraphics(handle)
    delete(handle);
end
end
