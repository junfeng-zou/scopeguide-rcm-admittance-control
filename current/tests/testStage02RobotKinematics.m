function tests = testStage02RobotKinematics
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'robot'));
addpath(fullfile(projectRoot, 'force_calibration'));
addpath(fullfile(projectRoot, 'tests'));
testCase.TestData.ProjectRoot = projectRoot;
end

function testZeroPoseForwardKinematicsRegression(testCase)
cfg = defaultRcmAdmittanceConfig();
result = scopeguide.geometry.cr5ForwardKinematics(zeros(6, 1), cfg);
expectedRotation = [-1, 0, 0; 0, 0, -1; 0, -1, 0];
expectedFlangePosition = [0; -0.223; 1.090];
expectedTipPosition = [0.002840; -0.578046; 0.976949];

verifyEqual(testCase, result.TBaseFlange(1:3, 1:3), ...
    expectedRotation, 'AbsTol', 1e-12);
verifyEqual(testCase, result.FlangePositionBaseM, ...
    expectedFlangePosition, 'AbsTol', 1e-12);
verifyEqual(testCase, result.EndoscopeTipPositionBaseM, ...
    expectedTipPosition, 'AbsTol', 1e-12);
verifyEqual(testCase, result.TBaseEndoscope, ...
    result.TBaseFlange * cfg.tool.TFlangeEndoscope, 'AbsTol', 1e-12);
end

function testForwardKinematicsProducesRigidTransforms(testCase)
cfg = defaultRcmAdmittanceConfig();
rng(23);
for sample = 1:30
    q = -1.5 + 3 * rand(6, 1);
    result = scopeguide.geometry.cr5ForwardKinematics(q, cfg);
    for link = 1:6
        verifyRigidTransform(testCase, result.TBaseLink(:, :, link));
    end
    verifyRigidTransform(testCase, result.TBaseEndoscope);
    verifyEqual(testCase, norm(result.ShaftAxisBase), 1, ...
        'AbsTol', 1e-12);
end
end

function testEndoscopeJacobianMatchesCentralDifference(testCase)
cfg = defaultRcmAdmittanceConfig();
rng(47);
step = 1e-7;
maximumRelativeError = 0;
for sample = 1:20
    q = -1.2 + 2.4 * rand(6, 1);
    jacobians = scopeguide.geometry.cr5GeometricJacobian(q, cfg);
    numerical = zeros(6, 6);
    for joint = 1:6
        perturbation = zeros(6, 1);
        perturbation(joint) = step;
        plus = scopeguide.geometry.cr5ForwardKinematics( ...
            q + perturbation, cfg);
        minus = scopeguide.geometry.cr5ForwardKinematics( ...
            q - perturbation, cfg);
        numerical(1:3, joint) = ...
            (plus.EndoscopeTipPositionBaseM - ...
            minus.EndoscopeTipPositionBaseM) / (2 * step);
        numerical(4:6, joint) = ...
            scopeguide.geometry.rotationLogVector( ...
            plus.TBaseEndoscope(1:3, 1:3) * ...
            minus.TBaseEndoscope(1:3, 1:3)') / (2 * step);
    end
    relativeError = norm(jacobians.EndoscopeTip - numerical, 'fro') / ...
        max(norm(numerical, 'fro'), eps);
    maximumRelativeError = max(maximumRelativeError, relativeError);
end
verifyLessThan(testCase, maximumRelativeError, 1e-6);
end

function testToolPointJacobianShiftIdentity(testCase)
cfg = defaultRcmAdmittanceConfig();
q = [0.2; -0.6; 0.4; 0.1; -0.3; 0.8];
jacobians = scopeguide.geometry.cr5GeometricJacobian(q, cfg);
offsetBase = jacobians.Kinematics.EndoscopeTipPositionBaseM - ...
    jacobians.Kinematics.FlangePositionBaseM;
expectedLinear = jacobians.Flange(1:3, :) - ...
    skew(offsetBase) * jacobians.Flange(4:6, :);
verifyEqual(testCase, jacobians.EndoscopeTip(1:3, :), ...
    expectedLinear, 'AbsTol', 1e-12);
end

function testDegreeInputIsRejected(testCase)
cfg = defaultRcmAdmittanceConfig();
verifyError(testCase, @() scopeguide.geometry.cr5ForwardKinematics( ...
    [0; 90; 0; 0; 0; 0], cfg), ...
    'scopeguide:kinematics:JointUnitOrRangeError');
end

function testRobotAdapterConvertsUnitsAndIsReadOnly(testCase)
cfg = liveDryRunConfig();
backend = MockDobotBackend(validSnapshot());
adapter = scopeguide.io.RobotAdapter(cfg, backend);
cleanup = onCleanup(@() adapter.disconnect());
adapter.connectReadOnly();
state = adapter.readState();

verifyTrue(testCase, state.IsValid);
verifyEqual(testCase, state.JointPositionRad(2), pi/2, ...
    'AbsTol', 1e-12);
verifyEqual(testCase, state.JointVelocityRadSec(1), pi/180, ...
    'AbsTol', 1e-12);
verifyEqual(testCase, state.ControllerPosePositionM, ...
    [0.1; 0.2; 0.3], 'AbsTol', 1e-12);
verifyEqual(testCase, state.ActualTcpTwistBase(1), 0.010, ...
    'AbsTol', 1e-12);
verifyEqual(testCase, state.ActualTcpTwistBase(6), pi/2, ...
    'AbsTol', 1e-12);
verifyLessThan(testCase, state.ControllerRpyQuaternionMismatchRad, 1e-12);
verifyEqual(testCase, state.ControllerPoseReference, "flange");
verifyTrue(testCase, state.ControllerPoseUnitsVerified);
verifyEqual(testCase, state.TBaseFlange, ...
    state.TBaseControllerPose, 'AbsTol', 1e-12);
verifyEqual(testCase, state.QuaternionBaseFlangeWxyz, ...
    state.QuaternionBaseControllerWxyz, 'AbsTol', 1e-12);
verifyError(testCase, @() adapter.sendServoTarget(zeros(6, 1)), ...
    'scopeguide:robot:DryRunMotionRejected');
verifyEqual(testCase, backend.ServoJCallCount, uint64(0));
verifyEqual(testCase, adapter.CommandSentCount, uint64(0));
clear cleanup;
end

function testRobotAdapterMarksStaleFeedbackInvalid(testCase)
cfg = liveDryRunConfig();
snapshot = validSnapshot();
snapshot.feedbackAgeSec = 2 * cfg.robot.FeedbackStaleSec;
backend = MockDobotBackend(snapshot);
adapter = scopeguide.io.RobotAdapter(cfg, backend);
cleanup = onCleanup(@() adapter.disconnect());
adapter.connectReadOnly();
state = adapter.readState();
verifyFalse(testCase, state.IsValid);
verifyEqual(testCase, state.StatusCode, "STALE_FEEDBACK");
clear cleanup;
end

function testRecordedMultiPoseDataInfersFlangeReference(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.force.CalibrationFile = string(fullfile(testCase.TestData.ProjectRoot, ...
    'force_calibration', 'data', ...
    'hex_h_tool_calibration_20260807_164630', ...
    'calibration_without_pose09.json'));
source = string(fullfile(testCase.TestData.ProjectRoot, ...
    'force_calibration', 'data', ...
    'hex_h_tool_calibration_20260807_164630', 'raw_samples.csv'));
validation = scopeguide.geometry.replayRecordedRobotKinematics(cfg, source);

verifyEqual(testCase, validation.Summary.PoseIndices, [1:8, 10]);
verifyGreaterThan(testCase, validation.Summary.UniqueFeedbackCount, 100);
verifyEqual(testCase, ...
    validation.Summary.InferredHistoricalControllerPoseReference, "flange");
verifyLessThan(testCase, validation.Summary.FlangePositionRmseM, ...
    validation.Summary.EndoscopePositionRmseM);
verifyTrue(testCase, isfinite(validation.Summary.FeedbackPeriodP99Sec));
verifyTrue(testCase, ...
    validation.Summary.CurrentLiveSemanticsStillRequireVerification);
end

function testAggregateLiveReadOnlyReports(testCase)
temporaryRoot = string(tempname);
mkdir(temporaryRoot);
cleanup = onCleanup(@() rmdir(temporaryRoot, 's'));
directories = strings(3, 1);
for index = 1:3
    directories(index) = fullfile(temporaryRoot, "pose_" + index);
    mkdir(directories(index));
    runSummary = struct( ...
        'PoseLabel', "pose_" + index, ...
        'PhysicalMotionCommandSent', false, ...
        'AdapterCommandSentCount', 0, ...
        'FeedbackPeriodP99Sec', 0.009, ...
        'FeedbackAgeP99Sec', 0.008, ...
        'ValidReadRatio', 1.0);
    writeTestJson(fullfile(directories(index), 'summary.json'), ...
        runSummary);
    jointOffset = 0.1 * index;
    signals = table( ...
        [jointOffset; jointOffset], zeros(2, 1), zeros(2, 1), ...
        zeros(2, 1), zeros(2, 1), zeros(2, 1), ...
        [0.0008; 0.0009], [0.001; 0.001], ...
        [0.371; 0.371], [0.001; 0.001], [0; 0], ...
        'VariableNames', {'J1Rad', 'J2Rad', 'J3Rad', 'J4Rad', ...
        'J5Rad', 'J6Rad', 'FlangePositionErrorM', ...
        'FlangeOrientationErrorRad', 'EndoscopePositionErrorM', ...
        'EndoscopeOrientationErrorRad', 'RpyQuaternionMismatchRad'});
    writetable(signals, fullfile(directories(index), 'signals.csv'));
end

[summary, outputDirectory] = ...
    aggregate_stage02_robot_readonly_validations( ...
    directories, WriteResults=false);
verifyEqual(testCase, summary.PoseRunCount, 3);
verifyTrue(testCase, summary.AllPoseLabelsUnique);
verifyTrue(testCase, summary.AllPhysicalMotionCommandSentFalse);
verifyEqual(testCase, summary.TotalAdapterCommandSentCount, 0);
verifyEqual(testCase, summary.InferredControllerPoseReference, "flange");
verifyEqual(testCase, outputDirectory, "");
clear cleanup;
end

function cfg = liveDryRunConfig()
cfg = defaultRcmAdmittanceConfig();
cfg.runtime.Mode = "live_dry_run";
validateRcmAdmittanceConfig(cfg);
end

function snapshot = validSnapshot()
snapshot = struct();
snapshot.feedbackSequence = uint64(10);
snapshot.hostMonotonicSec = 1.0;
snapshot.feedbackAgeSec = 0.001;
snapshot.invalidFeedbackByteCount = uint64(0);
snapshot.robotMode = "DISABLED";
snapshot.jointAnglesDeg = [0, 90, 0, 0, 0, 0];
snapshot.cartesianPose = [100, 200, 300, 0, 0, 90];
snapshot.actualJointSpeedsDegSec = [1, 0, 0, 0, 0, 0];
snapshot.actualTCPSpeed = [10, 0, 0, 0, 0, 90];
snapshot.actualQuaternionWxyz = [sqrt(0.5), 0, 0, sqrt(0.5)];
end

function verifyRigidTransform(testCase, transform)
verifyEqual(testCase, transform(4, :), [0, 0, 0, 1], ...
    'AbsTol', 1e-12);
rotation = transform(1:3, 1:3);
verifyEqual(testCase, rotation' * rotation, eye(3), ...
    'AbsTol', 1e-12);
verifyEqual(testCase, det(rotation), 1, 'AbsTol', 1e-12);
end

function matrix = skew(vector)
matrix = [0, -vector(3), vector(2); ...
    vector(3), 0, -vector(1); ...
    -vector(2), vector(1), 0];
end

function writeTestJson(filePath, value)
fileId = fopen(filePath, 'w');
if fileId < 0
    error('scopeguide:tests:CannotWriteTemporaryJson', ...
        'Cannot create temporary test file %s.', filePath);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', jsonencode(value));
clear cleanup;
end
