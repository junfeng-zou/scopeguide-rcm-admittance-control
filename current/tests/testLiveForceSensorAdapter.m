function tests = testLiveForceSensorAdapter
tests = functiontests(localfunctions);
end

function setupOnce(~)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'force_sensor'));
addpath(fullfile(projectRoot, 'force_calibration'));
addpath(fullfile(projectRoot, 'robot'));
addpath(fullfile(projectRoot, 'tests'));
end

function testCanonicalLivePathUsesHexClientAndPipeline(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.runtime.Mode = "live_dry_run";
sensorClient = MockHexHClient();
sensorCfg = onrobot.defaultConfig();
forceSource = scopeguide.io.ForceSensorAdapter( ...
    cfg, sensorCfg, sensorClient);

robotBackend = MockDobotBackend(validRobotSnapshot());
robot = scopeguide.io.RobotAdapter(cfg, robotBackend);
cleanup = onCleanup(@() disconnectAll(forceSource, robot));
robot.connectReadOnly();
forceSource.connect();
frame = forceSource.readProcessed(robot);

verifyEqual(testCase, frame.RawSensorSample.sequence, uint64(1));
verifyEqual(testCase, frame.ForceSample.RawWrenchSensor, ...
    sensorClient.Wrench, 'AbsTol', 0);
verifyTrue(testCase, frame.Processed.Quality.Valid);
verifyTrue(testCase, frame.Processed.MotionInputValid);
verifyFalse(testCase, frame.Processed.ControlEnabled);
verifyEqual(testCase, frame.Processed.ControlForceTool, zeros(3, 1), ...
    'AbsTol', 0);
verifyTrue(testCase, frame.MomentEligibleForControl);
verifyEqual(testCase, ...
    frame.Processed.ControlWrenchToolAtSensorOrigin, zeros(6, 1), ...
    'AbsTol', 0);
verifyEqual(testCase, forceSource.Pipeline.Counters.Accepted, uint64(1));
verifyEqual(testCase, sensorClient.ConnectCallCount, uint64(1));
verifyEqual(testCase, sensorClient.StatusReadCount, uint64(1));
verifyEqual(testCase, robot.CommandSentCount, uint64(0));
clear cleanup;
end

function snapshot = validRobotSnapshot()
snapshot = struct();
snapshot.feedbackSequence = uint64(10);
snapshot.hostMonotonicSec = 1.0;
snapshot.feedbackAgeSec = 0.001;
snapshot.invalidFeedbackByteCount = uint64(0);
snapshot.robotMode = "DISABLED";
snapshot.jointAnglesDeg = zeros(1, 6);
snapshot.cartesianPose = [0, -222.9, 1090.7, -90, 0, -180];
snapshot.actualJointSpeedsDegSec = zeros(1, 6);
snapshot.actualTCPSpeed = zeros(1, 6);
snapshot.actualQuaternionWxyz = [0.000148750374033193, ...
    -0.000149047407491279, 0.707482916575648, ...
    -0.706730414239979];
end

function disconnectAll(forceSource, robot)
forceSource.disconnect();
robot.disconnect();
end
