function [result, outputDirectory] = ...
        run_three_pose_force_diagnostic(options)
%RUN_THREE_POSE_FORCE_DIAGNOSTIC Guarded A/B/C/A force diagnostic.
%
% Pose A is the controller Cartesian pose at startup. The default candidates
% keep controller X/Y/Z fixed and apply two small, independent orientation
% changes:
%   B orientation = A orientation + [8 0 0] deg
%   C orientation = A orientation + [0 8 0] deg
% Motion is sent through the existing ZJFDobotCR5.MovJ method. These are
% diagnostic poses, not calibrated surgical motions. The endoscope must be
% fully outside the patient/phantom and every printed target/path must be
% checked by the operator before confirmation.
%
% The routine never calls Enable, Reset, ClearError, HEX-H zero or unzero.
% The robot must already be enabled and idle. On any runtime failure it
% sends StopMove, saves the partial log, and does not attempt an automatic
% recovery motion.

arguments
    options.EnableMotion (1, 1) logical = false
    options.RequireTypedConfirmation (1, 1) logical = true
    options.PoseBOrientationDeltaDeg (1, 3) double = [8 0 0]
    options.PoseCOrientationDeltaDeg (1, 3) double = [0 8 0]
    options.MaximumOrientationTransitionDeg (1, 1) double = 15
    options.MaximumPredictedTipDisplacementM (1, 1) double = 0.15
    options.SpeedRatioPercent (1, 1) double = 20
    options.SampleDurationSec (1, 1) double = 20
    options.SampleRateHz (1, 1) double = 200
    options.StableHoldSec (1, 1) double = 1.0
    options.MoveTimeoutSec (1, 1) double = 60
    options.MotionStartTimeoutSec (1, 1) double = 3.0
    options.CartesianPositionToleranceMm (1, 1) double = 1.0
    options.CartesianOrientationToleranceDeg (1, 1) double = 0.5
    % Nova5 QDActual has shown stationary quantization around 0.102 deg/s.
    % Keep margin above that floor while retaining TCP-speed and consecutive
    % unique-feedback checks.
    options.MaximumStationaryJointSpeedDegSec (1, 1) double = 0.15
    options.MaximumStationaryTcpTranslationSpeed (1, 1) double = 0.20
    options.MaximumStationaryTcpRotationSpeed (1, 1) double = 0.20
    options.MinimumConsecutiveMovingFeedbackFrames (1, 1) double = 2
    options.MaximumFeedbackAgeSec (1, 1) double = 0.10
    options.MotionRawForceStopN (1, 1) double = 40
    options.MotionRawMomentStopNm (1, 1) double = 4
    options.MotionExternalForceStopN (1, 1) double = 25
    options.MotionExternalMomentStopNm (1, 1) double = 2
end

validateOptions(options);
if ~options.EnableMotion
    error('scopeguide:diagnostics:MotionNotExplicitlyEnabled', ...
        ['This diagnostic sends MovJ commands. Re-run with ' ...
         'EnableMotion=true only after checking the cleared workspace.']);
end

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'force_sensor'));
addpath(fullfile(projectRoot, 'force_calibration'));
addpath(fullfile(projectRoot, 'robot'));

cfg = defaultRcmAdmittanceConfig();
sensorCfg = onrobot.defaultConfig();
sensorCfg.SampleRateHz = options.SampleRateHz;
[calibration, calibrationProvenance] = ...
    loadHexHGravityCalibration(cfg.force.CalibrationFile);

outputDirectory = createOutputDirectory(projectRoot);
csvFile = fullfile(outputDirectory, 'signals.csv');
matFile = fullfile(outputDirectory, 'diagnostic.mat');
summaryFile = fullfile(outputDirectory, 'summary.json');
writeJson(fullfile(outputDirectory, 'options.json'), options);

robot = ZJFDobotCR5(cfg.robot.IPAddress);
sensor = onrobot.HexHClient(sensorCfg);
hardwareCleanup = onCleanup(@() disconnectHardware(robot, sensor));
allData = table();
result = initialResult(cfg, calibrationProvenance, options, ...
    csvFile, matFile, summaryFile);

try
    printSafetyBanner(options);
    robot.Connect();
    sensor.connect();
    waitForInitialFeedback(robot, cfg.robot.InitialFeedbackTimeoutSec);
    requireEnabledAndIdle(robot);
    requireSensorHealthy(sensor);
    waitUntilStationary(robot, options);

    snapshotA = robot.GetStateSnapshot();
    plan = scopeguide.diagnostics.buildThreePoseCartesianTargets( ...
        snapshotA.cartesianPose, options.PoseBOrientationDeltaDeg, ...
        options.PoseCOrientationDeltaDeg, ...
        options.MaximumOrientationTransitionDeg);
    [tipPositions, maximumTipDisplacementM] = ...
        previewTipPositions(plan.TargetCartesianPose, cfg);
    if maximumTipDisplacementM > options.MaximumPredictedTipDisplacementM
        error('scopeguide:diagnostics:PredictedTipMotionTooLarge', ...
            ['Nominal FK predicts %.1f mm maximum endpoint displacement ' ...
             'from A, exceeding the configured %.1f mm limit. Reduce the ' ...
             'orientation deltas or explicitly revise the limit after review.'], ...
            1000 * maximumTipDisplacementM, ...
            1000 * options.MaximumPredictedTipDisplacementM);
    end
    result.MotionPlan = plan;
    result.NominalEndoscopeTipPositionBaseM = tipPositions;
    result.MaximumPredictedTipDisplacementM = maximumTipDisplacementM;
    printMotionPlan(plan, tipPositions, maximumTipDisplacementM);
    requireConfirmation(options, 'RUN THREE POSES', ...
        ['Type "RUN THREE POSES" only after checking all four targets, ' ...
         'the swept volume, cable slack, and physical E-stop access: ']);

    robot.SetSpeedRatio(options.SpeedRatioPercent);
    sessionClock = tic;

    requireConfirmation(options, 'CAPTURE A', ...
        'Type "CAPTURE A" to record the current unloaded pose A: ');
    [poseData, poseSummary] = captureStaticPose( ...
        robot, sensor, calibration, "A_initial", 1, ...
        sessionClock, options);
    allData = poseData;
    result.PoseSummaries.A_initial = poseSummary;
    saveCheckpoint(allData, result, csvFile, matFile, summaryFile);

    moveAndWait(robot, sensor, calibration, ...
        plan.TargetCartesianPose(2, :), ...
        "B", options);
    [poseData, poseSummary] = captureStaticPose( ...
        robot, sensor, calibration, "B", 2, sessionClock, options);
    allData = [allData; poseData];
    result.PoseSummaries.B = poseSummary;
    saveCheckpoint(allData, result, csvFile, matFile, summaryFile);

    moveAndWait(robot, sensor, calibration, ...
        plan.TargetCartesianPose(3, :), ...
        "C", options);
    [poseData, poseSummary] = captureStaticPose( ...
        robot, sensor, calibration, "C", 3, sessionClock, options);
    allData = [allData; poseData];
    result.PoseSummaries.C = poseSummary;
    saveCheckpoint(allData, result, csvFile, matFile, summaryFile);

    moveAndWait(robot, sensor, calibration, ...
        plan.TargetCartesianPose(4, :), ...
        "A_return", options);
    [poseData, poseSummary] = captureStaticPose( ...
        robot, sensor, calibration, "A_return", 4, ...
        sessionClock, options);
    allData = [allData; poseData];
    result.PoseSummaries.A_return = poseSummary;
    result.ReturnResidualChangeTool = ...
        result.PoseSummaries.A_return.ExternalToolMean - ...
        result.PoseSummaries.A_initial.ExternalToolMean;
    result.Completed = true;
    result.Status = "complete";
    result.SampleCount = height(allData);
    saveCheckpoint(allData, result, csvFile, matFile, summaryFile);

    fprintf('\nThree-pose diagnostic completed and returned to A.\n');
    fprintf('A-return external wrench change: [%s]\n', ...
        vectorText(result.ReturnResidualChangeTool));
    fprintf('Results: %s\n', outputDirectory);
catch exception
    result.Status = "failed";
    result.ErrorIdentifier = string(exception.identifier);
    result.ErrorMessage = string(exception.message);
    result.SampleCount = height(allData);
    try
        if robot.IsConnected
            robot.StopMove();
        end
    catch stopException
        result.StopMoveError = string(stopException.message);
    end
    saveCheckpoint(allData, result, csvFile, matFile, summaryFile);
    rethrow(exception);
end
end

function validateOptions(options)
positiveNames = {'MaximumOrientationTransitionDeg', ...
    'MaximumPredictedTipDisplacementM', 'SpeedRatioPercent', ...
    'SampleDurationSec', 'SampleRateHz', 'StableHoldSec', ...
    'MoveTimeoutSec', 'MotionStartTimeoutSec', ...
    'CartesianPositionToleranceMm', ...
    'CartesianOrientationToleranceDeg', ...
    'MaximumStationaryJointSpeedDegSec', ...
    'MaximumStationaryTcpTranslationSpeed', ...
    'MaximumStationaryTcpRotationSpeed', 'MaximumFeedbackAgeSec', ...
    'MotionRawForceStopN', 'MotionRawMomentStopNm', ...
    'MotionExternalForceStopN', 'MotionExternalMomentStopNm'};
for index = 1:numel(positiveNames)
    value = options.(positiveNames{index});
    if ~isscalar(value) || ~isfinite(value) || value <= 0
        error('scopeguide:diagnostics:InvalidOption', ...
            '%s must be finite and positive.', positiveNames{index});
    end
end
if options.MinimumConsecutiveMovingFeedbackFrames < 1 || ...
        options.MinimumConsecutiveMovingFeedbackFrames ~= ...
        round(options.MinimumConsecutiveMovingFeedbackFrames)
    error('scopeguide:diagnostics:InvalidMovingFrameCount', ...
        'MinimumConsecutiveMovingFeedbackFrames must be a positive integer.');
end
if options.SpeedRatioPercent > 20
    error('scopeguide:diagnostics:UnsafeSpeedRatio', ...
        'SpeedRatioPercent is limited to 20%% for this diagnostic.');
end
end

function printSafetyBanner(options)
fprintf('\n============================================================\n');
fprintf('Nova5 / HEX-H THREE-POSE FORCE DIAGNOSTIC\n');
fprintf('MATLAB WILL SEND MovJ Cartesian commands at %.1f%% speed.\n', ...
    options.SpeedRatioPercent);
fprintf('Required before continuing:\n');
fprintf('  1. Endoscope is fully outside every patient/phantom.\n');
fprintf('  2. The full wrist/endoscope swept volume is clear.\n');
fprintf('  3. Cables are slack and a physical E-stop is reachable.\n');
fprintf('  4. Robot is already enabled and idle.\n');
fprintf('  5. No HEX-H ZERO/Auto-calibration will be used.\n');
fprintf('The script never enables or resets the robot.\n');
fprintf('============================================================\n\n');
end

function result = initialResult(cfg, provenance, options, ...
        csvFile, matFile, summaryFile)
result = struct();
result.Status = "initialized";
result.Completed = false;
result.CreatedLocal = string(datetime('now'));
result.RobotIPAddress = string(cfg.robot.IPAddress);
result.SensorIPAddress = "192.168.50.201";
result.CalibrationFile = string(cfg.force.CalibrationFile);
result.CalibrationProvenance = provenance;
result.Options = options;
result.PoseSummaries = struct();
result.CsvFile = string(csvFile);
result.MatFile = string(matFile);
result.SummaryFile = string(summaryFile);
result.SampleCount = 0;
end

function [positions, maximumDisplacement] = ...
        previewTipPositions(targetCartesianPose, cfg)
positions = zeros(size(targetCartesianPose, 1), 3);
for index = 1:size(targetCartesianPose, 1)
    pose = targetCartesianPose(index, :);
    transform = eye(4);
    transform(1:3, 1:3) = scopeguide.geometry.dobotRpyToRotation( ...
        pose(4:6), cfg.robot.ControllerPoseAngleScaleToRad);
    transform(1:3, 4) = pose(1:3).' * ...
        cfg.robot.ControllerPoseTranslationScaleToM;
    endoscopeTransform = transform * cfg.tool.TFlangeEndoscope;
    positions(index, :) = endoscopeTransform(1:3, 4).';
end
maximumDisplacement = max(vecnorm(positions - positions(1, :), 2, 2));
end

function printMotionPlan(plan, tipPositions, maximumDisplacement)
fprintf('Captured pose A and generated the following candidate plan:\n');
for index = 1:numel(plan.Labels)
    fprintf('  %-9s pose_mm_deg=[%s]  nominal_tip_m=[%s]\n', ...
        plan.Labels(index), ...
        vectorText(plan.TargetCartesianPose(index, :)), ...
        vectorText(tipPositions(index, :)));
end
fprintf('Maximum endpoint displacement from A: %.1f mm\n', ...
    1000 * maximumDisplacement);
fprintf(['The nominal tip preview assumes controller CartesianPose is the ' ...
    'flange pose, as inferred in stage 2.\n']);
fprintf(['These endpoint checks are not collision detection. MovJ follows ' ...
    'a controller-planned path whose entire swept volume must be clear.\n']);
end

function moveAndWait(robot, sensor, calibration, targetPose, label, options)
requireEnabledAndIdle(robot);
requireSensorHealthy(sensor);
snapshot = robot.GetStateSnapshot();
orientationStepDeg = orientationErrorDeg( ...
    targetPose(4:6), snapshot.actualQuaternionWxyz);
if orientationStepDeg > options.MaximumOrientationTransitionDeg
    error('scopeguide:diagnostics:LiveTransitionTooLarge', ...
        ['Live orientation transition to %s is %.2f deg, exceeding ' ...
         'the configured %.1f deg.'], label, orientationStepDeg, ...
        options.MaximumOrientationTransitionDeg);
end
expected = "MOVE " + label;
requireConfirmation(options, expected, sprintf( ...
    'Type "%s" to send MovJ to %s: ', expected, label));
fprintf('Moving to %s at %.1f%% speed...\n', label, ...
    options.SpeedRatioPercent);
moveResponse = robot.MovJ(targetPose);

moveClock = tic;
stableClock = [];
motionStarted = false;
while true
    sample = sensor.readSample();
    snapshot = robot.GetStateSnapshot();
    assertFreshFeedback(snapshot, options);
    mode = string(snapshot.robotMode);
    if any(mode == ["ERROR", "PAUSE", "DISABLED", "POWER_OFF"])
        safeStop(robot);
        error('scopeguide:diagnostics:RobotModeDuringMove', ...
            'Robot entered mode %s while moving to %s.', mode, label);
    end
    compensated = compensateHexHWrench( ...
        sample.wrench, snapshot.actualQuaternionWxyz, calibration);
    enforceMotionWrenchLimits(sample.wrench, compensated.externalTool, ...
        robot, label, options);

    positionErrorMm = norm(double(snapshot.cartesianPose(1:3)) - ...
        double(targetPose(1:3)));
    orientationError = orientationErrorDeg( ...
        targetPose(4:6), snapshot.actualQuaternionWxyz);
    targetReached = ...
        positionErrorMm <= options.CartesianPositionToleranceMm && ...
        orientationError <= options.CartesianOrientationToleranceDeg;
    stationary = isStationary(snapshot, options);
    % Nova5 V3 may keep RunQueuedCmd=1 while enabled and stationary, so it
    % is diagnostic context only. Actual motion start requires RUNNING or
    % measured joint/TCP velocity.
    motionStarted = motionStarted || mode == "RUNNING" || ~stationary;
    if targetReached && stationary && mode == "ENABLE"
        if isempty(stableClock)
            stableClock = tic;
        elseif toc(stableClock) >= options.StableHoldSec
            fprintf('Reached %s and remained stationary for %.1f s.\n', ...
                label, options.StableHoldSec);
            return;
        end
    else
        stableClock = [];
    end
    if ~motionStarted && toc(moveClock) > options.MotionStartTimeoutSec
        controllerErrors = robot.GetErrorID();
        safeStop(robot);
        error('scopeguide:diagnostics:MotionQueueDidNotStart', ...
            ['MovJ was accepted but motion to %s did not start within ' ...
             '%.1f s. Move response: %s Dashboard GetErrorID: %s ' ...
             'RobotMode=%s, RunQueuedCmd=%d, PauseCmdFlag=%d.'], ...
            label, options.MotionStartTimeoutSec, strtrim(moveResponse), ...
            strtrim(controllerErrors), mode, ...
            snapshot.runQueuedCommand, snapshot.pauseCommandFlag);
    end
    if toc(moveClock) > options.MoveTimeoutSec
        safeStop(robot);
        error('scopeguide:diagnostics:MoveTimeout', ...
            ['Move to %s did not settle within %.1f s. Last Cartesian ' ...
             'errors: %.3f mm, %.3f deg.'], label, ...
            options.MoveTimeoutSec, positionErrorMm, orientationError);
    end
    pause(0.005);
end
end

function enforceMotionWrenchLimits(raw, externalTool, robot, label, options)
raw = double(raw(:));
external = double(externalTool(:));
reason = "";
if norm(raw(1:3)) >= options.MotionRawForceStopN
    reason = "RAW_FORCE";
elseif norm(raw(4:6)) >= options.MotionRawMomentStopNm
    reason = "RAW_MOMENT";
elseif norm(external(1:3)) >= options.MotionExternalForceStopN
    reason = "EXTERNAL_FORCE";
elseif norm(external(4:6)) >= options.MotionExternalMomentStopNm
    reason = "EXTERNAL_MOMENT";
end
if strlength(reason) > 0
    safeStop(robot);
    error('scopeguide:diagnostics:MotionWrenchStop', ...
        ['%s limit was reached while moving to %s. Raw norms %.2f N/' ...
         '%.3f Nm; compensated norms %.2f N/%.3f Nm.'], ...
        reason, label, norm(raw(1:3)), norm(raw(4:6)), ...
        norm(external(1:3)), norm(external(4:6)));
end
end

function [data, summary] = captureStaticPose( ...
        robot, sensor, calibration, label, poseIndex, sessionClock, options)
requireEnabledAndIdle(robot);
requireSensorHealthy(sensor);
waitUntilStationary(robot, options);
count = max(2, round(options.SampleDurationSec * options.SampleRateHz));
fprintf('Recording %s: %d samples at %.1f Hz for %.1f s...\n', ...
    label, count, options.SampleRateHz, options.SampleDurationSec);

poseLabel = repmat(string(label), count, 1);
poseNumber = repmat(double(poseIndex), count, 1);
sampleInPose = (1:count).';
hostSessionTimeSec = zeros(count, 1);
sensorSequence = zeros(count, 1, 'uint64');
sensorTimeSec = zeros(count, 1);
sensorReadDurationSec = zeros(count, 1);
robotSequence = zeros(count, 1, 'uint64');
robotFeedbackAgeSec = zeros(count, 1);
raw = zeros(count, 6);
gravity = zeros(count, 6);
unbiased = zeros(count, 6);
externalSensor = zeros(count, 6);
externalTool = zeros(count, 6);
joints = zeros(count, 6);
quaternion = zeros(count, 4);
stationary = false(count, 1);

period = 1 / options.SampleRateHz;
recordClock = tic;
lastRobotSequence = uint64(0);
lastMotionSequence = uint64(0);
lastRobotUpdateClock = tic;
movingFrames = 0;
for index = 1:count
    waitUntil(recordClock, (index - 1) * period);
    wrenchSample = sensor.readSample();
    snapshot = robot.GetStateSnapshot();
    assertFreshFeedback(snapshot, options);
    if snapshot.feedbackSequence ~= lastRobotSequence
        lastRobotSequence = snapshot.feedbackSequence;
        lastRobotUpdateClock = tic;
    elseif toc(lastRobotUpdateClock) > options.MaximumFeedbackAgeSec
        error('scopeguide:diagnostics:FrozenFeedback', ...
            'Robot feedback froze while recording %s.', label);
    end
    still = isStationary(snapshot, options);
    [lastMotionSequence, movingFrames, isNewFeedback] = ...
        scopeguide.diagnostics.updateUniqueMotionCounter( ...
        lastMotionSequence, movingFrames, snapshot.feedbackSequence, ~still);
    if isNewFeedback && movingFrames >= ...
            options.MinimumConsecutiveMovingFeedbackFrames
        error('scopeguide:diagnostics:RobotMovedDuringCapture', ...
            ['Robot moved for %d consecutive unique feedback frames during ' ...
             '%s. Max joint/TCP speeds: %.4f deg/s, %.4f, %.4f.'], ...
            movingFrames, label, ...
            max(abs(snapshot.actualJointSpeedsDegSec)), ...
            max(abs(snapshot.actualTCPSpeed(1:3))), ...
            max(abs(snapshot.actualTCPSpeed(4:6))));
    end
    compensated = compensateHexHWrench( ...
        wrenchSample.wrench, snapshot.actualQuaternionWxyz, calibration);

    hostSessionTimeSec(index) = toc(sessionClock);
    sensorSequence(index) = wrenchSample.sequence;
    sensorTimeSec(index) = wrenchSample.monotonicTime;
    sensorReadDurationSec(index) = wrenchSample.readDuration;
    robotSequence(index) = snapshot.feedbackSequence;
    robotFeedbackAgeSec(index) = snapshot.feedbackAgeSec;
    raw(index, :) = compensated.rawSensor.';
    gravity(index, :) = compensated.gravitySensor.';
    unbiased(index, :) = compensated.unbiasedSensor.';
    externalSensor(index, :) = compensated.externalSensor.';
    externalTool(index, :) = compensated.externalTool.';
    joints(index, :) = snapshot.jointAnglesDeg;
    quaternion(index, :) = snapshot.actualQuaternionWxyz;
    stationary(index) = still;
end

data = table(poseLabel, poseNumber, sampleInPose, hostSessionTimeSec, ...
    sensorSequence, sensorTimeSec, sensorReadDurationSec, robotSequence, ...
    robotFeedbackAgeSec, stationary);
data = addSixColumns(data, raw, 'raw');
data = addSixColumns(data, gravity, 'gravity');
data = addSixColumns(data, unbiased, 'unbiased');
data = addSixColumns(data, externalSensor, 'externalSensor');
data = addSixColumns(data, externalTool, 'externalTool');
for axis = 1:6
    data.(sprintf('J%dDeg', axis)) = joints(:, axis);
end
for axis = 1:4
    data.(sprintf('q%dWxyz', axis)) = quaternion(:, axis);
end

summary = struct();
summary.Label = string(label);
summary.SampleCount = count;
summary.RawMean = mean(raw, 1);
summary.RawStd = std(raw, 0, 1);
summary.GravityMean = mean(gravity, 1);
summary.ExternalSensorMean = mean(externalSensor, 1);
summary.ExternalSensorStd = std(externalSensor, 0, 1);
summary.ExternalToolMean = mean(externalTool, 1);
summary.ExternalToolStd = std(externalTool, 0, 1);
summary.ExternalToolForceNormN = norm(summary.ExternalToolMean(1:3));
summary.ExternalToolMomentNormNm = norm(summary.ExternalToolMean(4:6));
summary.JointMeanDeg = mean(joints, 1);
summary.QuaternionMeanWxyz = averageQuaternion(quaternion);
fprintf('%s external_tool mean=[%s], |F|=%.3f N, |T|=%.4f Nm\n', ...
    label, vectorText(summary.ExternalToolMean), ...
    summary.ExternalToolForceNormN, summary.ExternalToolMomentNormNm);
end

function data = addSixColumns(data, values, prefix)
suffix = {'FxN','FyN','FzN','TxNm','TyNm','TzNm'};
for index = 1:6
    data.([prefix suffix{index}]) = values(:, index);
end
end

function average = averageQuaternion(values)
normalized = values ./ vecnorm(values, 2, 2);
reference = normalized(1, :);
for index = 2:size(normalized, 1)
    if dot(normalized(index, :), reference) < 0
        normalized(index, :) = -normalized(index, :);
    end
end
average = mean(normalized, 1);
average = average / norm(average);
end

function waitForInitialFeedback(robot, timeoutSec)
clock = tic;
while robot.FeedbackSequence == 0
    if toc(clock) > timeoutSec
        error('scopeguide:diagnostics:FeedbackTimeout', ...
            'No robot feedback arrived within %.1f s.', timeoutSec);
    end
    pause(0.02);
end
snapshot = robot.GetStateSnapshot();
if any(~isfinite(snapshot.actualQuaternionWxyz))
    error('scopeguide:diagnostics:InvalidQuaternion', ...
        'Initial robot quaternion is invalid.');
end
end

function requireEnabledAndIdle(robot)
mode = string(robot.RobotMode);
if mode ~= "ENABLE"
    error('scopeguide:diagnostics:RobotNotEnabledAndIdle', ...
        ['RobotMode must be ENABLE before capture/motion. Current mode: ' ...
         '%s. Enable and verify the robot manually.'], mode);
end
end

function requireSensorHealthy(sensor)
status = double(sensor.readStatus());
if status ~= 0
    error('scopeguide:diagnostics:SensorStatus', ...
        'HEX-H status register is %d.', status);
end
end

function waitUntilStationary(robot, options)
clock = tic;
stableClock = [];
while true
    snapshot = robot.GetStateSnapshot();
    assertFreshFeedback(snapshot, options);
    if isStationary(snapshot, options) && ...
            string(snapshot.robotMode) == "ENABLE"
        if isempty(stableClock)
            stableClock = tic;
        elseif toc(stableClock) >= options.StableHoldSec
            return;
        end
    else
        stableClock = [];
    end
    if toc(clock) > options.MoveTimeoutSec
        error('scopeguide:diagnostics:StationaryTimeout', ...
            'Robot did not become stationary within %.1f s.', ...
            options.MoveTimeoutSec);
    end
    pause(0.02);
end
end

function stationary = isStationary(snapshot, options)
stationary = ...
    max(abs(snapshot.actualJointSpeedsDegSec)) <= ...
        options.MaximumStationaryJointSpeedDegSec && ...
    max(abs(snapshot.actualTCPSpeed(1:3))) <= ...
        options.MaximumStationaryTcpTranslationSpeed && ...
    max(abs(snapshot.actualTCPSpeed(4:6))) <= ...
        options.MaximumStationaryTcpRotationSpeed;
end

function assertFreshFeedback(snapshot, options)
if ~isfinite(snapshot.feedbackAgeSec) || ...
        snapshot.feedbackAgeSec > options.MaximumFeedbackAgeSec
    error('scopeguide:diagnostics:StaleFeedback', ...
        'Robot feedback age %.3f s exceeds %.3f s.', ...
        snapshot.feedbackAgeSec, options.MaximumFeedbackAgeSec);
end
if any(~isfinite(snapshot.actualQuaternionWxyz)) || ...
        abs(norm(snapshot.actualQuaternionWxyz) - 1) > 1e-3
    error('scopeguide:diagnostics:InvalidQuaternion', ...
        'Robot quaternion is invalid.');
end
end

function requireConfirmation(options, expected, prompt)
if ~options.RequireTypedConfirmation
    return;
end
response = string(input(prompt, 's'));
if response ~= string(expected)
    error('scopeguide:diagnostics:OperatorCancelled', ...
        'Expected "%s"; operation cancelled.', expected);
end
end

function saveCheckpoint(data, result, csvFile, matFile, summaryFile)
if ~isempty(data)
    writetable(data, csvFile);
end
save(matFile, 'data', 'result');
writeJson(summaryFile, result);
end

function outputDirectory = createOutputDirectory(projectRoot)
timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
outputDirectory = fullfile(projectRoot, 'results', ...
    "three_pose_force_diagnostic_" + timestamp);
[created, message] = mkdir(outputDirectory);
if ~created && ~isfolder(outputDirectory)
    error('scopeguide:diagnostics:OutputDirectory', ...
        'Could not create %s: %s', outputDirectory, message);
end
end

function writeJson(path, payload)
try
    text = jsonencode(payload, 'PrettyPrint', true);
catch
    text = jsonencode(payload);
end
file = fopen(path, 'w');
if file < 0
    error('scopeguide:diagnostics:JsonWriteFailed', ...
        'Could not open %s.', path);
end
guard = onCleanup(@() fclose(file));
fwrite(file, text, 'char');
end

function waitUntil(clock, targetSec)
while true
    remaining = targetSec - toc(clock);
    if remaining <= 0
        return;
    elseif remaining > 0.003
        pause(remaining - 0.001);
    else
        pause(0);
    end
end
end

function errorDeg = orientationErrorDeg(targetRpyDeg, actualQuaternionWxyz)
targetRotation = scopeguide.geometry.dobotRpyToRotation(targetRpyDeg, pi / 180);
actualRotation = scopeguide.geometry.rotationMatrixFromQuaternionWxyz( ...
    actualQuaternionWxyz);
errorDeg = rad2deg(scopeguide.geometry.rotationDistance( ...
    targetRotation, actualRotation));
end

function safeStop(robot)
try
    robot.StopMove();
catch
end
end

function text = vectorText(value)
text = strtrim(sprintf('%+.6f ', double(value)));
end

function disconnectHardware(robot, sensor)
try
    sensor.disconnect();
catch
end
try
    robot.Disconnect();
catch
end
end
