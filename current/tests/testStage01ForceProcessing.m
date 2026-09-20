function tests = testStage01ForceProcessing
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'force_sensor'));
addpath(fullfile(projectRoot, 'force_calibration'));
testCase.TestData.ProjectRoot = projectRoot;
end

function testSmoothVectorDeadzoneDirectionAndContinuity(testCase)
threshold = 1.3;
[below, activeBelow] = scopeguide.force.smoothVectorDeadzone( ...
    [threshold - 1e-9; 0; 0], threshold, 0.8 * threshold, false);
[above, activeAbove] = scopeguide.force.smoothVectorDeadzone( ...
    [threshold + 1e-9; 0; 0], threshold, 0.8 * threshold, false);
[output, ~] = scopeguide.force.smoothVectorDeadzone( ...
    [3; 4; 0], threshold, 0.8 * threshold, false);

verifyEqual(testCase, below, zeros(3, 1), 'AbsTol', 1e-15);
verifyFalse(testCase, activeBelow);
verifyTrue(testCase, activeAbove);
verifyLessThan(testCase, norm(above), 2e-9);
verifyEqual(testCase, output / norm(output), [3; 4; 0] / 5, ...
    'AbsTol', 1e-12);
verifyEqual(testCase, norm(output), 5 - threshold, 'AbsTol', 1e-12);
end

function testGuardedBaselineUpdatesAndFreezes(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.force.AutomaticBaselineEnabled = true;
cfg.force.Baseline.StartupDurationSec = 0.2;
cfg.force.Baseline.MinimumEligibleDurationSec = 0;
estimator = scopeguide.force.BaselineEstimator(cfg.force);
context = eligibleBaselineContext();
wrench = [1; 0; 0; 0.1; 0; 0];

for index = 1:20
    estimator.step(wrench, 0.1, context);
end
updatedBaseline = estimator.BaselineWrench;
verifyGreaterThan(testCase, norm(updatedBaseline), 0);

context.HandleEnabled = true;
for index = 1:20
    estimator.step(1.5 * wrench, 0.1, context);
end
verifyEqual(testCase, estimator.BaselineWrench, updatedBaseline, ...
    'AbsTol', 0);
end

function testStartupZeroAndIdleTrackingAreFzOnly(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.force.AutomaticBaselineEnabled = true;
cfg.force.Baseline.StartupDurationSec = 0.2;
cfg.force.Baseline.MinimumEligibleDurationSec = 0;
cfg.force.Baseline.UpdateTimeConstantSec = 0.1;
cfg.force.Baseline.MaximumForceUpdateRateNSec = 10;
estimator = scopeguide.force.BaselineEstimator(cfg.force);
context = eligibleBaselineContext();
startupWrench = [0.4; -0.2; 14; 0.05; -0.02; 0.03];

[~, firstDiagnostics] = estimator.step( ...
    startupWrench, 0.1, context);
[startupBaseline, readyDiagnostics] = estimator.step( ...
    startupWrench, 0.1, context);

verifyFalse(testCase, firstDiagnostics.Ready);
verifyTrue(testCase, readyDiagnostics.Ready);
verifyTrue(testCase, readyDiagnostics.StartupJustCompleted);
verifyEqual(testCase, startupBaseline, startupWrench, 'AbsTol', 1e-12);

trackingWrench = startupWrench + [0.2; 0; 1.0; 0; 0; 0];
for index = 1:20
    estimator.step(trackingWrench, 0.1, context);
end
trackedBaseline = estimator.BaselineWrench;
verifyEqual(testCase, trackedBaseline(1), startupWrench(1), ...
    'AbsTol', 0);
verifyGreaterThan(testCase, trackedBaseline(3), startupWrench(3));
verifyEqual(testCase, trackedBaseline(4:6), startupWrench(4:6), ...
    'AbsTol', 0);

context.HandleEnabled = true;
frozenBaseline = estimator.BaselineWrench;
for index = 1:20
    estimator.step(trackingWrench + [0; 0; 0.5; 0; 0; 0], ...
        0.1, context);
end
verifyEqual(testCase, estimator.BaselineWrench, frozenBaseline, ...
    'AbsTol', 0);
end

function testControlForceRequiresOperationEnable(testCase)
[pipeline, cfg] = createPipeline();
desiredForce = [cfg.force.ForceDeadzoneN + 1; 0; 0];
[sample, quaternion] = syntheticSample( ...
    pipeline, [desiredForce; zeros(3, 1)], 1, 0.005);
context = scopeguide.types.forceProcessingContext();
context.HandleEnabled = false;
processed = pipeline.step(sample, quaternion, 0.005, context);

verifyTrue(testCase, processed.MotionInputValid);
verifyFalse(testCase, processed.ControlEnabled);
verifyEqual(testCase, processed.ControlForceTool, zeros(3, 1), ...
    'AbsTol', 0);
end

function testDefaultBaselineIsDisabled(testCase)
cfg = defaultRcmAdmittanceConfig();
estimator = scopeguide.force.BaselineEstimator(cfg.force);
[baseline, diagnostics] = estimator.step( ...
    [1; 0; 0; 0; 0; 0], 0.1, eligibleBaselineContext());
verifyEqual(testCase, baseline, zeros(6, 1));
verifyFalse(testCase, diagnostics.Updated);
verifyEqual(testCase, diagnostics.EligibilityReason, "DISABLED");
end

function testRawSafetySpikeCannotBeHidden(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.safety.RawForceStopN = 5;
cfg.safety.RawMomentStopNm = 2;
cfg.safety.FastForceWarningN = 2;
cfg.safety.FastForceStopN = 4;
cfg.safety.FastMomentWarningNm = 0.5;
cfg.safety.FastMomentStopNm = 1;
monitor = scopeguide.force.WrenchSafetyMonitor(cfg.safety);
decision = monitor.step([6; 0; 0; 0; 0; 0], zeros(6, 1), 0.005);

verifyTrue(testCase, decision.StopRequested);
verifyTrue(testCase, any(decision.StopReasons == "RAW_FORCE_STOP"));
verifyFalse(testCase, decision.AllowControlInput);
end

function testPipelineSyntheticForceOnlyOutput(testCase)
for axis = 1:3
    for signValue = [-1, 1]
        [pipeline, cfg] = createPipeline();
        desiredForce = zeros(3, 1);
        desiredForce(axis) = signValue * ...
            (cfg.force.ForceDeadzoneN + 1);
        desiredToolWrench = [desiredForce; zeros(3, 1)];
        [sample, quaternion] = syntheticSample( ...
            pipeline, desiredToolWrench, 1, 0.005);
        processed = pipeline.step(sample, quaternion, 0.005, ...
            scopeguide.types.forceProcessingContext());

        expectedControlForce = zeros(3, 1);
        expectedControlForce(axis) = signValue;
        verifyTrue(testCase, processed.Quality.Valid);
        verifyTrue(testCase, processed.MotionInputValid);
        verifyEqual(testCase, ...
            processed.ExternalWrenchToolAtSensorOrigin, ...
            desiredToolWrench, 'AbsTol', 1e-10);
        verifyEqual(testCase, processed.ControlForceTool, ...
            expectedControlForce, 'AbsTol', 1e-10);
        verifyTrue(testCase, processed.MomentEligibleForControl);
        verifyEqual(testCase, ...
            processed.ControlWrenchToolAtSensorOrigin, ...
            [expectedControlForce; zeros(3, 1)], 'AbsTol', 1e-10);
    end
end
end

function testFastBranchRespondsBeforeSlowBranch(testCase)
[pipeline, ~] = createPipeline();
[initial, quaternion] = syntheticSample( ...
    pipeline, zeros(6, 1), 1, 0.005);
pipeline.step(initial, quaternion, 0.005, ...
    scopeguide.types.forceProcessingContext());
[stepSample, quaternion] = syntheticSample( ...
    pipeline, [5; 0; 0; 0; 0; 0], 2, 0.010);
processed = pipeline.step(stepSample, quaternion, 0.010, ...
    scopeguide.types.forceProcessingContext());

verifyGreaterThan(testCase, ...
    norm(processed.FastWrenchToolAtSensorOrigin(1:3)), ...
    norm(processed.SlowWrenchToolAtSensorOrigin(1:3)));
verifyFalse(testCase, processed.NeutralCheckPassed);
end

function testStaleSampleFailsClosed(testCase)
[pipeline, ~] = createPipeline();
[sample, quaternion] = syntheticSample( ...
    pipeline, zeros(6, 1), 1, 0);
processed = pipeline.step(sample, quaternion, 0.1, ...
    scopeguide.types.forceProcessingContext());

verifyFalse(testCase, processed.Quality.Valid);
verifyEqual(testCase, processed.Quality.StatusCode, "STALE_SAMPLE");
verifyFalse(testCase, processed.MotionInputValid);
verifyEqual(testCase, processed.ControlForceTool, zeros(3, 1));
verifyTrue(testCase, processed.Safety.StopRequested);
end

function testDuplicateSequenceFailsClosed(testCase)
[pipeline, ~] = createPipeline();
[sample1, quaternion] = syntheticSample( ...
    pipeline, zeros(6, 1), 1, 0.005);
first = pipeline.step(sample1, quaternion, 0.005, ...
    scopeguide.types.forceProcessingContext());
sample2 = sample1;
sample2.HostMonotonicSec = 0.010;
second = pipeline.step(sample2, quaternion, 0.010, ...
    scopeguide.types.forceProcessingContext());

verifyTrue(testCase, first.Quality.Valid);
verifyFalse(testCase, second.Quality.Valid);
verifyEqual(testCase, second.Quality.StatusCode, "DUPLICATE_SEQUENCE");
verifyEqual(testCase, second.ControlForceTool, zeros(3, 1));
end

function testNonfiniteWrenchAndDeviceErrorFailClosed(testCase)
[pipeline, ~] = createPipeline();
[sample, quaternion] = syntheticSample( ...
    pipeline, zeros(6, 1), 1, 0.005);
sample.RawWrenchSensor(2) = NaN;
nonfinite = pipeline.step(sample, quaternion, 0.005, ...
    scopeguide.types.forceProcessingContext());
verifyEqual(testCase, nonfinite.Quality.StatusCode, "NONFINITE_WRENCH");
verifyFalse(testCase, nonfinite.MotionInputValid);

pipeline.reset();
[sample, quaternion] = syntheticSample( ...
    pipeline, zeros(6, 1), 1, 0.005);
sample.DeviceStatus = 3;
deviceError = pipeline.step(sample, quaternion, 0.005, ...
    scopeguide.types.forceProcessingContext());
verifyEqual(testCase, deviceError.Quality.StatusCode, ...
    "DEVICE_STATUS_ERROR");
verifyFalse(testCase, deviceError.MotionInputValid);
end

function testCalibrationReplayRetainsAcceptedPoses(testCase)
cfg = historicalCalibrationReplayConfig( ...
    testCase.TestData.ProjectRoot);
source = calibrationReplayFile(testCase.TestData.ProjectRoot);
replay = scopeguide.force.replayCalibrationDataset(cfg, source);

verifyEqual(testCase, replay.Summary.SampleCount, 7200);
verifyEqual(testCase, replay.Summary.PoseIndices, [1:8, 10]);
verifyEqual(testCase, replay.Summary.InvalidCount, 0);
verifyEqual(testCase, replay.Summary.SafetyStopCount, 0);
verifyTrue(testCase, all(replay.QualityValid));
verifyTrue(testCase, all(isfinite(replay.ControlForceTool), 'all'));
verifyTrue(testCase, replay.Summary.MomentEligibleForControl);
% The retained calibration poses must remain inside the small residual
% control allowance after the 1.3 N radial deadzone.
verifyLessThan(testCase, replay.Summary.MaximumControlForceNormN, 0.30);
end

function testCalibrationReplayIsDeterministic(testCase)
cfg = historicalCalibrationReplayConfig( ...
    testCase.TestData.ProjectRoot);
source = calibrationReplayFile(testCase.TestData.ProjectRoot);
first = scopeguide.force.replayCalibrationDataset( ...
    cfg, source, MaximumSamples=200);
second = scopeguide.force.replayCalibrationDataset( ...
    cfg, source, MaximumSamples=200);

verifyEqual(testCase, first.ExternalWrenchToolAtSensorOrigin, ...
    second.ExternalWrenchToolAtSensorOrigin, 'AbsTol', 0);
verifyEqual(testCase, first.FastWrenchToolAtSensorOrigin, ...
    second.FastWrenchToolAtSensorOrigin, 'AbsTol', 0);
verifyEqual(testCase, first.ControlForceTool, ...
    second.ControlForceTool, 'AbsTol', 0);
verifyEqual(testCase, first.StatusCode, second.StatusCode);
end

function testLegacyRawCsvDoesNotPretendToHavePose(testCase)
staticFile = fullfile(testCase.TestData.ProjectRoot, 'force_sensor', ...
    'data', 'hex_h_static_20260718_202428.csv');
data = readtable(staticFile, 'VariableNamingRule', 'preserve');
names = string(data.Properties.VariableNames);
verifyTrue(testCase, all(ismember( ...
    ["Fx_N", "Fy_N", "Fz_N", "Tx_Nm", "Ty_Nm", "Tz_Nm"], names)));
verifyFalse(testCase, any(ismember(["qw", "qx", "qy", "qz"], names)));
end

function testStaticAndContactRawReplayRemainControlIneligible(testCase)
cfg = defaultRcmAdmittanceConfig();
staticFile = string(fullfile(testCase.TestData.ProjectRoot, ...
    'force_sensor', 'data', 'hex_h_static_20260718_202428.csv'));
contactFile = string(fullfile(testCase.TestData.ProjectRoot, ...
    'force_sensor', 'data', 'hex_h_contact_20260718_210529.csv'));
staticReplay = scopeguide.force.replayPoseFreeRawDataset( ...
    cfg, staticFile);
contactReplay = scopeguide.force.replayPoseFreeRawDataset( ...
    cfg, contactFile);

verifyFalse(testCase, staticReplay.CompensationApplied);
verifyFalse(testCase, staticReplay.ControlEligible);
verifyFalse(testCase, contactReplay.CompensationApplied);
verifyFalse(testCase, contactReplay.ControlEligible);
verifyTrue(testCase, all(isfinite(staticReplay.FastRawWrenchSensor), 'all'));
verifyTrue(testCase, all(isfinite(contactReplay.SlowRawWrenchSensor), 'all'));
verifyGreaterThan(testCase, ...
    contactReplay.Summary.MaximumRawForceNormN, ...
    staticReplay.Summary.MaximumRawForceNormN);
end

function [pipeline, cfg] = createPipeline()
cfg = defaultRcmAdmittanceConfig();
pipeline = scopeguide.force.ForceProcessingPipeline(cfg);
end

function [sample, quaternion] = syntheticSample( ...
        pipeline, externalToolWrench, sequence, timestamp)
quaternion = [1; 0; 0; 0];
calibration = pipeline.Calibration;
reference = compensateHexHWrench( ...
    calibration.biasSensor, quaternion, calibration);
rotationToolFromSensor = calibration.rotationToolFromSensor;
externalSensor = [ ...
    rotationToolFromSensor.' * externalToolWrench(1:3); ...
    rotationToolFromSensor.' * externalToolWrench(4:6)];

sample = scopeguide.types.forceSample();
sample.RawWrenchSensor = calibration.biasSensor + ...
    reference.gravitySensor + externalSensor;
sample.Sequence = uint64(sequence);
sample.HostMonotonicSec = timestamp;
sample.ReadDurationSec = 0.001;
sample.SampleAgeSec = 0;
sample.DeviceStatus = 0;
sample.IsValid = true;
sample.StatusCode = "SYNTHETIC";
end

function context = eligibleBaselineContext()
context = scopeguide.types.forceProcessingContext();
context.HandleEnabled = false;
context.RobotStationary = true;
context.NoContactConfirmed = true;
context.AllowBaselineUpdate = true;
end

function file = calibrationReplayFile(projectRoot)
file = string(fullfile(projectRoot, 'force_calibration', 'data', ...
    'hex_h_tool_calibration_20260807_164630', 'raw_samples.csv'));
end

function cfg = historicalCalibrationReplayConfig(projectRoot)
cfg = defaultRcmAdmittanceConfig();
cfg.force.CalibrationFile = string(fullfile(projectRoot, ...
    'force_calibration', 'data', ...
    'hex_h_tool_calibration_20260807_164630', ...
    'calibration_without_pose09.json'));
end
