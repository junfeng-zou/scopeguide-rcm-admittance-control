function result = calibrateHexHToolLoadWithDobot(cfg)
%CALIBRATEHEXHTOOLLOADWITHDOBOT Collect static poses and fit HEX-H tool load.
%
% Safe default (no MATLAB motion command):
%   cfg = defaultHexHToolCalibrationConfig();
%   result = calibrateHexHToolLoadWithDobot(cfg);
%
% Guarded automatic joint motion (only with workcell-validated targets):
%   cfg = defaultHexHToolCalibrationConfig();
%   cfg.MotionMode = 'automatic_joint';
%   cfg.AutomaticJointTargetsDeg = [ ... ]; % at least 7 safe rows
%   result = calibrateHexHToolLoadWithDobot(cfg);
%
% The routine never calls HEX-H zero()/unzero(), Enable(), Reset(), or
% ClearError(). In automatic mode the robot must already be enabled, every
% target requires typed confirmation by default, and motion uses JointMovJ.

if nargin < 1 || isempty(cfg)
    cfg = defaultHexHToolCalibrationConfig();
end
validateConfiguration(cfg);

calibrationDirectory = fileparts(mfilename('fullpath'));
projectRoot = fileparts(calibrationDirectory);
addpath(calibrationDirectory);
addpath(fullfile(projectRoot, 'robot'));
addpath(fullfile(projectRoot, 'force_sensor'));

automaticMotion = strcmpi(cfg.MotionMode, 'automatic_joint');
if automaticMotion
    targets = double(cfg.AutomaticJointTargetsDeg);
    poseCount = size(targets, 1);
else
    targets = zeros(0, 6);
    poseCount = cfg.OperatorGuidedPoseCount;
end

printSafetyBanner(cfg, automaticMotion, poseCount);
if cfg.RequireTypedConfirmation
    confirmation = input( ...
        'Type CALIBRATE (uppercase) after completing the checks: ', 's');
    if ~strcmp(confirmation, 'CALIBRATE')
        error('calibration:OperatorCancelled', ...
            'Calibration cancelled before connecting to hardware.');
    end
end

robot = ZJFDobotCR5(cfg.RobotIPAddress);
sensorCfg = onrobot.defaultConfig();
sensorCfg.SampleRateHz = cfg.SampleRateHz;
sensor = onrobot.HexHClient(sensorCfg);
cleanupGuard = onCleanup(@() disconnectHardware(robot, sensor)); %#ok<NASGU>

try
    robot.Connect();
    sensor.connect();
    waitForInitialFeedback(robot, 5.0);
    if automaticMotion
        requireEnabledAndIdle(robot);
        robot.SetSpeedRatio(cfg.RobotSpeedRatioPercent);
    end

    samplesPerPose = max(20, round(cfg.SampleRateHz * cfg.SamplesPerPoseSec));
    totalSamples = poseCount * samplesPerPose;
    poseIndex = zeros(totalSamples, 1);
    sampleInPose = zeros(totalSamples, 1);
    hostSessionTimeSec = zeros(totalSamples, 1);
    sensorSequence = zeros(totalSamples, 1, 'uint64');
    sensorMonotonicSec = zeros(totalSamples, 1);
    sensorReadDurationSec = zeros(totalSamples, 1);
    robotFeedbackSequence = zeros(totalSamples, 1, 'uint64');
    robotFeedbackTimeSec = zeros(totalSamples, 1);
    wrenchSensor = zeros(totalSamples, 6);
    quaternionWxyz = zeros(totalSamples, 4);
    jointAnglesDeg = zeros(totalSamples, 6);
    cartesianPose = zeros(totalSamples, 6);
    jointSpeedDegSec = zeros(totalSamples, 6);
    tcpSpeed = zeros(totalSamples, 6);
    stationary = false(totalSamples, 1);

    sessionTimer = tic;
    rowOffset = 0;
    gravityReferenceValidated = false;
    gravityReferenceValidation = struct();
    for pose = 1:poseCount
        fprintf('\n=== Calibration pose %d / %d ===\n', pose, poseCount);
        if automaticMotion
            target = targets(pose, :);
            snapshot = robot.GetStateSnapshot();
            jointStep = abs(wrappedDifferenceDeg( ...
                target, snapshot.jointAnglesDeg));
            if any(jointStep > cfg.MaximumJointStepDeg)
                error('calibration:JointStepTooLarge', ...
                    ['Pose %d exceeds MaximumJointStepDeg=%.1f. ' ...
                     'Largest requested step is %.1f deg.'], ...
                    pose, cfg.MaximumJointStepDeg, max(jointStep));
            end
            fprintf('Current joints [deg]: %s\n', vectorText(snapshot.jointAnglesDeg));
            fprintf('Target joints  [deg]: %s\n', vectorText(target));
            if cfg.RequireTypedConfirmation
                expected = sprintf('MOVE %d', pose);
                response = input(sprintf('Type "%s" to send JointMovJ: ', expected), 's');
                if ~strcmp(response, expected)
                    error('calibration:OperatorCancelled', ...
                        'Operator cancelled before pose %d.', pose);
                end
            end
            requireEnabledAndIdle(robot);
            robot.JointMovJ(target);
            waitUntilStationary(robot, target, cfg);
        else
            fprintf(['Move the robot with the teach pendant/backdrive to a new ' ...
                'collision-free orientation. Keep the endoscope suspended, ' ...
                'with no contact and slack cables.\n']);
            if cfg.RequireTypedConfirmation
                expected = sprintf('SAMPLE %d', pose);
                response = input(sprintf( ...
                    'Type "%s" when the robot is at the pose: ', expected), 's');
                if ~strcmp(response, expected)
                    error('calibration:OperatorCancelled', ...
                        'Operator cancelled before pose %d.', pose);
                end
            else
                input('Press Enter when the robot is at the pose.', 's');
            end
            waitUntilStationary(robot, [], cfg);
        end

        staticSnapshot = robot.GetStateSnapshot();
        isZeroJointReference = all(abs(wrappedDifferenceDeg( ...
            cfg.ZeroJointReferenceDeg, staticSnapshot.jointAnglesDeg)) <= ...
            cfg.ZeroJointToleranceDeg);
        if cfg.RequireZeroJointGravityReference && ...
                ~automaticMotion && pose == 1 && ~isZeroJointReference
            error('calibration:MissingZeroJointReference', ...
                ['Operator-guided pose 1 must be the zero-joint gravity ' ...
                 'reference within %.2f deg. Current joints: %s'], ...
                cfg.ZeroJointToleranceDeg, ...
                vectorText(staticSnapshot.jointAnglesDeg));
        end
        if isZeroJointReference
            gravityReferenceValidation = validateZeroJointGravityDirection( ...
                staticSnapshot.actualQuaternionWxyz, ...
                'RToolFromSensor', cfg.RToolFromSensor, ...
                'GravityBaseMps2', cfg.GravityBaseMps2, ...
                'QuaternionConvention', cfg.QuaternionConvention, ...
                'ExpectedSensorDirection', ...
                    cfg.ExpectedGravityDirectionSensor, ...
                'ExpectedToolDirection', cfg.ExpectedGravityDirectionTool, ...
                'MaximumDirectionErrorDeg', ...
                    cfg.MaximumGravityDirectionErrorDeg);
            gravityReferenceValidated = true;
            fprintf(['Zero-joint gravity direction verified: sensor error ' ...
                '%.3f deg, tool error %.3f deg.\n'], ...
                gravityReferenceValidation.sensorDirectionErrorDeg, ...
                gravityReferenceValidation.toolDirectionErrorDeg);
        end

        sensorStatus = sensor.readStatus();
        if sensorStatus ~= 0
            error('calibration:HexHStatus', ...
                'HEX-H status is %d before pose %d; aborting.', sensorStatus, pose);
        end
        fprintf('Recording %.1f s at %.1f Hz without hardware tare...\n', ...
            cfg.SamplesPerPoseSec, cfg.SampleRateHz);
        indices = rowOffset + (1:samplesPerPose);
        [poseSamples, motionReport] = recordOnePose( ...
            robot, sensor, cfg, samplesPerPose, sessionTimer);
        if motionReport.confirmedMovingRatio > cfg.MaximumMovingSampleRatio
            error('calibration:RobotMovedDuringSampling', ...
                [sprintf(['Pose %d confirmed moving-feedback ratio %.2f%% ' ...
                 'exceeds %.2f%% (%d/%d unique 30004 frames). '], ...
                 pose, 100*motionReport.confirmedMovingRatio, ...
                 100*cfg.MaximumMovingSampleRatio, ...
                 motionReport.confirmedMovingFrameCount, ...
                 motionReport.uniqueFeedbackCount), ...
                 sprintf(['Raw threshold hits: %d; isolated hits ignored: %d. ' ...
                 'Triggers joint/TCP-translation/TCP-rotation: %d/%d/%d. ' ...
                 'Maxima: %.4g deg/s, %.4g, %.4g.'], ...
                 motionReport.rawMovingFrameCount, ...
                 motionReport.ignoredIsolatedFrameCount, ...
                 motionReport.jointTriggerCount, ...
                 motionReport.translationTriggerCount, ...
                 motionReport.rotationTriggerCount, ...
                 motionReport.maximumJointSpeedDegSec, ...
                 motionReport.maximumTcpTranslationSpeed, ...
                 motionReport.maximumTcpRotationSpeed)]);
        end
        fprintf(['Stationarity: %d unique robot frames; raw/confirmed ' ...
            'motion %.2f%%/%.2f%%; %d isolated hits ignored.\n'], ...
            motionReport.uniqueFeedbackCount, ...
            100*motionReport.rawMovingRatio, ...
            100*motionReport.confirmedMovingRatio, ...
            motionReport.ignoredIsolatedFrameCount);

        poseIndex(indices) = pose;
        sampleInPose(indices) = (1:samplesPerPose).';
        hostSessionTimeSec(indices) = poseSamples.hostSessionTimeSec;
        sensorSequence(indices) = poseSamples.sensorSequence;
        sensorMonotonicSec(indices) = poseSamples.sensorMonotonicSec;
        sensorReadDurationSec(indices) = poseSamples.sensorReadDurationSec;
        robotFeedbackSequence(indices) = poseSamples.robotFeedbackSequence;
        robotFeedbackTimeSec(indices) = poseSamples.robotFeedbackTimeSec;
        wrenchSensor(indices, :) = poseSamples.wrenchSensor;
        quaternionWxyz(indices, :) = poseSamples.quaternionWxyz;
        jointAnglesDeg(indices, :) = poseSamples.jointAnglesDeg;
        cartesianPose(indices, :) = poseSamples.cartesianPose;
        jointSpeedDegSec(indices, :) = poseSamples.jointSpeedDegSec;
        tcpSpeed(indices, :) = poseSamples.tcpSpeed;
        stationary(indices) = poseSamples.stationary;
        rowOffset = rowOffset + samplesPerPose;
    end

    if cfg.RequireZeroJointGravityReference && ~gravityReferenceValidated
        error('calibration:MissingZeroJointReference', ...
            ['No sampled pose matched the zero-joint reference. No ' ...
             'calibration parameters will be accepted.']);
    end

    rawData = buildRawTable(poseIndex, sampleInPose, hostSessionTimeSec, ...
        sensorSequence, sensorMonotonicSec, sensorReadDurationSec, ...
        robotFeedbackSequence, robotFeedbackTimeSec, wrenchSensor, ...
        quaternionWxyz, jointAnglesDeg, cartesianPose, jointSpeedDegSec, ...
        tcpSpeed, stationary);
    [poseSummary, poseWrench, poseQuaternion] = summarizePoses( ...
        rawData, poseCount);

    [calibration, fitReport] = fitHexHToolLoadCalibration( ...
        poseWrench, poseQuaternion, ...
        'RToolFromSensor', cfg.RToolFromSensor, ...
        'GravityBaseMps2', cfg.GravityBaseMps2, ...
        'GravityForceSign', cfg.GravityForceSign, ...
        'QuaternionConvention', cfg.QuaternionConvention, ...
        'MaximumConditionNumber', cfg.MaximumConditionNumber, ...
        'ComputeLeaveOneOut', true);
    calibration.sensorOriginInToolM = cfg.SensorOriginInToolM;
    calibration.robotIPAddress = cfg.RobotIPAddress;
    calibration.sensorIPAddress = sensorCfg.IPAddress;
    calibration.createdUTC = char(datetime('now', 'TimeZone', 'UTC', ...
        'Format', 'yyyy-MM-dd''T''HH:mm:ss.SSSXXX'));
    calibration.zeroJointGravityReference = gravityReferenceValidation;

    outputDirectory = createOutputDirectory(cfg.OutputDirectory);
    rawCsv = fullfile(outputDirectory, 'raw_samples.csv');
    poseCsv = fullfile(outputDirectory, 'pose_summary.csv');
    matFile = fullfile(outputDirectory, 'calibration.mat');
    jsonFile = fullfile(outputDirectory, 'calibration.json');
    writetable(rawData, rawCsv);
    writetable(poseSummary, poseCsv);
    save(matFile, 'calibration', 'fitReport', 'poseSummary', 'rawData', 'cfg');
    writeJson(jsonFile, struct( ...
        'calibration', calibration, 'fitReport', fitReport));
    figureFile = '';
    if cfg.SaveFigure
        figureFile = fullfile(outputDirectory, 'calibration_residuals.png');
        createCalibrationFigure(poseWrench, fitReport, figureFile);
    end

    result = struct();
    result.calibration = calibration;
    result.fitReport = fitReport;
    result.poseSummary = poseSummary;
    result.outputDirectory = outputDirectory;
    result.rawCsv = rawCsv;
    result.poseCsv = poseCsv;
    result.matFile = matFile;
    result.jsonFile = jsonFile;
    result.figureFile = figureFile;

    fprintf('\nCalibration completed without changing HEX-H hardware bias.\n');
    fprintf('Mass: %.6f kg\n', calibration.massKg);
    fprintf('COM in HEX-H sensor frame [m]: %s\n', ...
        vectorText(calibration.comSensorM));
    fprintf('Bias [N,N,N,N*m,N*m,N*m]: %s\n', ...
        vectorText(calibration.biasSensor));
    fprintf('Gravity force sign: %+d\n', calibration.gravityForceSign);
    fprintf('LOO force RMSE: %.4f N; torque RMSE: %.5f N*m\n', ...
        fitReport.leaveOneOutForceRmseN, ...
        fitReport.leaveOneOutTorqueRmseNm);
    fprintf('Results: %s\n', outputDirectory);
catch calibrationError
    if automaticMotion && robot.IsConnected
        try
            robot.StopMove();
        catch stopError
            warning('calibration:StopFailed', ...
                'Could not send StopMove after failure: %s', stopError.message);
        end
    end
    rethrow(calibrationError);
end
end

function validateConfiguration(cfg)
required = {'RobotIPAddress','MotionMode','AutomaticJointTargetsDeg', ...
    'OperatorGuidedPoseCount','RequireTypedConfirmation', ...
    'RobotSpeedRatioPercent','MaximumJointStepDeg', ...
    'JointTargetToleranceDeg','MoveTimeoutSec','StableHoldSec', ...
    'MaximumStationaryJointSpeedDegSec', ...
    'MaximumStationaryTcpTranslationSpeed', ...
    'MaximumStationaryTcpRotationSpeed', ...
    'MinimumConsecutiveMovingFeedbackFrames','SampleRateHz', ...
    'SamplesPerPoseSec','MaximumMovingSampleRatio', ...
    'MaximumFeedbackStaleSec','RToolFromSensor','SensorOriginInToolM', ...
    'RequireZeroJointGravityReference','ZeroJointReferenceDeg', ...
    'ZeroJointToleranceDeg','ExpectedGravityDirectionTool', ...
    'ExpectedGravityDirectionSensor','MaximumGravityDirectionErrorDeg', ...
    'GravityBaseMps2','GravityForceSign','QuaternionConvention', ...
    'MaximumConditionNumber','OutputDirectory','SaveFigure'};
for index = 1:numel(required)
    if ~isfield(cfg, required{index})
        error('calibration:MissingConfiguration', ...
            'Configuration field %s is missing.', required{index});
    end
end
mode = lower(char(cfg.MotionMode));
if ~ismember(mode, {'operator_guided','automatic_joint'})
    error('calibration:InvalidMotionMode', ...
        'MotionMode must be operator_guided or automatic_joint.');
end
if strcmp(mode, 'automatic_joint')
    targets = cfg.AutomaticJointTargetsDeg;
    if size(targets, 2) ~= 6 || size(targets, 1) < 7 || ...
            any(~isfinite(targets), 'all')
        error('calibration:InvalidAutomaticTargets', ...
            ['AutomaticJointTargetsDeg must contain at least seven finite ' ...
             'collision-checked 1-by-6 joint targets.']);
    end
    if cfg.RequireZeroJointGravityReference
        referenceDifference = abs(wrappedDifferenceDeg( ...
            targets, cfg.ZeroJointReferenceDeg));
        if ~any(all(referenceDifference <= cfg.ZeroJointToleranceDeg, 2))
            error('calibration:MissingZeroJointReferenceTarget', ...
                ['Automatic targets must include the zero-joint gravity ' ...
                 'reference within %.2f deg.'], cfg.ZeroJointToleranceDeg);
        end
    end
elseif cfg.OperatorGuidedPoseCount < 7
    error('calibration:TooFewPoses', ...
        'At least seven poses are required for leave-one-out validation.');
end
if cfg.RobotSpeedRatioPercent < 1 || cfg.RobotSpeedRatioPercent > 10
    error('calibration:UnsafeSpeedRatio', ...
        'Calibration RobotSpeedRatioPercent must be between 1 and 10.');
end
if cfg.SampleRateHz <= 0 || cfg.SamplesPerPoseSec <= 0 || ...
        cfg.MaximumMovingSampleRatio < 0
    error('calibration:InvalidSamplingConfig', ...
        'Sampling configuration values are invalid.');
end
if cfg.MinimumConsecutiveMovingFeedbackFrames < 1 || ...
        cfg.MinimumConsecutiveMovingFeedbackFrames ~= ...
        round(cfg.MinimumConsecutiveMovingFeedbackFrames)
    error('calibration:InvalidStationarityConfiguration', ...
        'MinimumConsecutiveMovingFeedbackFrames must be a positive integer.');
end
end

function printSafetyBanner(cfg, automaticMotion, poseCount)
fprintf('\n============================================================\n');
fprintf('HEX-H / Dobot static tool-load calibration\n');
fprintf('Robot: %s | poses: %d | mode: %s\n', ...
    cfg.RobotIPAddress, poseCount, cfg.MotionMode);
fprintf('Required checks:\n');
fprintf('  1. Clear workspace; keep a hand on the physical E-stop.\n');
fprintf('  2. Endoscope and all rigid connectors are fully installed.\n');
fprintf('  3. Endoscope is suspended, not touching any object.\n');
fprintf('  4. Cables are slack and do not support/pull the tool.\n');
fprintf('  5. Every pose is collision-checked in this exact workcell.\n');
fprintf('  6. HEX-H is warmed up; no hardware zero will be issued.\n');
if cfg.RequireZeroJointGravityReference
    fprintf(['  7. Include J=[0,0,0,0,0,0] deg to verify gravity is ' ...
        'tool +Y / sensor -Y.\n']);
end
if automaticMotion
    fprintf('MATLAB WILL SEND JointMovJ at %d%% speed after each confirmation.\n', ...
        cfg.RobotSpeedRatioPercent);
    fprintf('The robot must already be enabled; this script never calls Enable().\n');
else
    fprintf('Operator-guided mode sends no robot motion command.\n');
end
fprintf('============================================================\n\n');
end

function waitForInitialFeedback(robot, timeoutSec)
timer = tic;
while robot.FeedbackSequence == 0
    if toc(timer) > timeoutSec
        error('calibration:FeedbackTimeout', ...
            'No Dobot 30004 feedback arrived within %.1f s.', timeoutSec);
    end
    pause(0.02);
end
snapshot = robot.GetStateSnapshot();
if any(~isfinite(snapshot.actualQuaternionWxyz))
    error('calibration:InvalidRobotQuaternion', ...
        'Dobot ActualQuaternion is invalid.');
end
end

function requireEnabledAndIdle(robot)
mode = char(robot.RobotMode);
if ~strcmp(mode, 'ENABLE')
    error('calibration:RobotNotEnabledAndIdle', ...
        ['Automatic calibration requires RobotMode=ENABLE (enabled and idle). ' ...
         'Current mode is %s. Enable and verify the robot manually.'], mode);
end
end

function waitUntilStationary(robot, targetJointDeg, cfg)
timeoutTimer = tic;
stableStart = [];
while true
    snapshot = robot.GetStateSnapshot();
    if strcmp(char(snapshot.robotMode), 'ERROR')
        error('calibration:RobotError', ...
            'Robot entered ERROR mode while waiting for a static pose.');
    end
    if isempty(targetJointDeg)
        targetReached = true;
    else
        difference = abs(wrappedDifferenceDeg( ...
            targetJointDeg, snapshot.jointAnglesDeg));
        targetReached = all(difference <= cfg.JointTargetToleranceDeg);
    end
    stationary = isStationarySnapshot(snapshot, cfg);
    if targetReached && stationary
        if isempty(stableStart)
            stableStart = tic;
        elseif toc(stableStart) >= cfg.StableHoldSec
            return;
        end
    else
        stableStart = [];
    end
    if toc(timeoutTimer) > cfg.MoveTimeoutSec
        error('calibration:PoseTimeout', ...
            'Robot did not reach a stable pose within %.1f s.', cfg.MoveTimeoutSec);
    end
    pause(0.02);
end
end

function [samples, motionReport] = recordOnePose( ...
        robot, sensor, cfg, count, sessionTimer)
period = 1 / cfg.SampleRateHz;
samples = struct();
samples.hostSessionTimeSec = zeros(count, 1);
samples.sensorSequence = zeros(count, 1, 'uint64');
samples.sensorMonotonicSec = zeros(count, 1);
samples.sensorReadDurationSec = zeros(count, 1);
samples.robotFeedbackSequence = zeros(count, 1, 'uint64');
samples.robotFeedbackTimeSec = zeros(count, 1);
samples.wrenchSensor = zeros(count, 6);
samples.quaternionWxyz = zeros(count, 4);
samples.jointAnglesDeg = zeros(count, 6);
samples.cartesianPose = zeros(count, 6);
samples.jointSpeedDegSec = zeros(count, 6);
samples.tcpSpeed = zeros(count, 6);
samples.stationary = false(count, 1);

recordTimer = tic;
lastFeedbackSequence = uint64(0);
feedbackChangeTimer = tic;
for index = 1:count
    waitUntil(recordTimer, (index - 1) * period);
    wrenchSample = sensor.readSample();
    snapshot = robot.GetStateSnapshot();
    if snapshot.feedbackSequence ~= lastFeedbackSequence
        lastFeedbackSequence = snapshot.feedbackSequence;
        feedbackChangeTimer = tic;
    elseif toc(feedbackChangeTimer) > cfg.MaximumFeedbackStaleSec
        error('calibration:StaleRobotFeedback', ...
            'Dobot feedback did not update for %.3f s.', ...
            cfg.MaximumFeedbackStaleSec);
    end
    samples.hostSessionTimeSec(index) = toc(sessionTimer);
    samples.sensorSequence(index) = wrenchSample.sequence;
    samples.sensorMonotonicSec(index) = wrenchSample.monotonicTime;
    samples.sensorReadDurationSec(index) = wrenchSample.readDuration;
    samples.robotFeedbackSequence(index) = snapshot.feedbackSequence;
    samples.robotFeedbackTimeSec(index) = snapshot.hostMonotonicSec;
    samples.wrenchSensor(index, :) = wrenchSample.wrench.';
    samples.quaternionWxyz(index, :) = snapshot.actualQuaternionWxyz;
    samples.jointAnglesDeg(index, :) = snapshot.jointAnglesDeg;
    samples.cartesianPose(index, :) = snapshot.cartesianPose;
    samples.jointSpeedDegSec(index, :) = snapshot.actualJointSpeedsDegSec;
    samples.tcpSpeed(index, :) = snapshot.actualTCPSpeed;
    samples.stationary(index) = isStationarySnapshot(snapshot, cfg);
end
motionReport = summarizeRobotStationarity( ...
    samples.robotFeedbackSequence, samples.jointSpeedDegSec, samples.tcpSpeed, ...
    'MaximumJointSpeedDegSec', cfg.MaximumStationaryJointSpeedDegSec, ...
    'MaximumTcpTranslationSpeed', ...
        cfg.MaximumStationaryTcpTranslationSpeed, ...
    'MaximumTcpRotationSpeed', cfg.MaximumStationaryTcpRotationSpeed, ...
    'MinimumConsecutiveMovingFrames', ...
        cfg.MinimumConsecutiveMovingFeedbackFrames);
end

function stationary = isStationarySnapshot(snapshot, cfg)
stationary = ...
    max(abs(snapshot.actualJointSpeedsDegSec)) <= ...
        cfg.MaximumStationaryJointSpeedDegSec && ...
    max(abs(snapshot.actualTCPSpeed(1:3))) <= ...
        cfg.MaximumStationaryTcpTranslationSpeed && ...
    max(abs(snapshot.actualTCPSpeed(4:6))) <= ...
        cfg.MaximumStationaryTcpRotationSpeed;
end

function rawData = buildRawTable(poseIndex, sampleInPose, ...
        hostSessionTimeSec, sensorSequence, sensorMonotonicSec, ...
        sensorReadDurationSec, robotFeedbackSequence, robotFeedbackTimeSec, ...
        wrench, quaternion, joints, tcpPose, jointSpeed, tcpSpeed, stationary)
rawData = table(poseIndex, sampleInPose, hostSessionTimeSec, ...
    sensorSequence, sensorMonotonicSec, sensorReadDurationSec, ...
    robotFeedbackSequence, robotFeedbackTimeSec, stationary);
wrenchNames = {'Fx_N','Fy_N','Fz_N','Tx_Nm','Ty_Nm','Tz_Nm'};
quaternionNames = {'qw','qx','qy','qz'};
for index = 1:6
    rawData.(wrenchNames{index}) = wrench(:, index);
    rawData.(sprintf('J%d_deg', index)) = joints(:, index);
    rawData.(sprintf('tcpPose%d', index)) = tcpPose(:, index);
    rawData.(sprintf('jointSpeed%d', index)) = jointSpeed(:, index);
    rawData.(sprintf('tcpSpeed%d', index)) = tcpSpeed(:, index);
end
for index = 1:4
    rawData.(quaternionNames{index}) = quaternion(:, index);
end
end

function [summary, poseWrench, poseQuaternion] = summarizePoses(rawData, poseCount)
poseWrench = zeros(poseCount, 6);
poseQuaternion = zeros(poseCount, 4);
wrenchStd = zeros(poseCount, 6);
meanJoints = zeros(poseCount, 6);
wrenchNames = {'Fx_N','Fy_N','Fz_N','Tx_Nm','Ty_Nm','Tz_Nm'};
quaternionNames = {'qw','qx','qy','qz'};
for pose = 1:poseCount
    rows = rawData.poseIndex == pose;
    wrench = zeros(sum(rows), 6);
    quaternions = zeros(sum(rows), 4);
    joints = zeros(sum(rows), 6);
    for axis = 1:6
        wrench(:, axis) = rawData.(wrenchNames{axis})(rows);
        joints(:, axis) = rawData.(sprintf('J%d_deg', axis))(rows);
    end
    for axis = 1:4
        quaternions(:, axis) = rawData.(quaternionNames{axis})(rows);
    end
    poseWrench(pose, :) = median(wrench, 1);
    wrenchStd(pose, :) = std(wrench, 0, 1);
    poseQuaternion(pose, :) = averageQuaternion(quaternions);
    meanJoints(pose, :) = mean(joints, 1);
end
summary = table((1:poseCount).', 'VariableNames', {'poseIndex'});
for axis = 1:6
    summary.(wrenchNames{axis}) = poseWrench(:, axis);
    summary.([wrenchNames{axis} '_std']) = wrenchStd(:, axis);
    summary.(sprintf('J%d_mean_deg', axis)) = meanJoints(:, axis);
end
for axis = 1:4
    summary.(quaternionNames{axis}) = poseQuaternion(:, axis);
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

function directory = createOutputDirectory(rootDirectory)
if ~isfolder(rootDirectory)
    mkdir(rootDirectory);
end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
directory = fullfile(rootDirectory, ['hex_h_tool_calibration_' timestamp]);
mkdir(directory);
end

function writeJson(path, payload)
try
    text = jsonencode(payload, 'PrettyPrint', true);
catch
    text = jsonencode(payload);
end
file = fopen(path, 'w');
if file < 0
    error('calibration:JsonWriteFailed', 'Could not open %s.', path);
end
guard = onCleanup(@() fclose(file)); %#ok<NASGU>
fwrite(file, text, 'char');
end

function createCalibrationFigure(measured, report, outputFile)
labels = {'Fx (N)','Fy (N)','Fz (N)','Tx (N*m)','Ty (N*m)','Tz (N*m)'};
figureHandle = figure('Name', 'HEX-H tool-load calibration', ...
    'Color', 'white', 'Visible', 'off');
guard = onCleanup(@() close(figureHandle)); %#ok<NASGU>
layout = tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
title(layout, 'Static pose wrench: measured, predicted, and residual');
for axis = 1:6
    plotAxis = nexttile(layout);
    pose = 1:size(measured, 1);
    plot(plotAxis, pose, measured(:, axis), 'o-', 'LineWidth', 1.2);
    hold(plotAxis, 'on');
    plot(plotAxis, pose, report.predictedWrenchSensor(:, axis), ...
        'x--', 'LineWidth', 1.2);
    plot(plotAxis, pose, report.compensatedResidual(:, axis), ...
        '.-', 'LineWidth', 1.0);
    grid(plotAxis, 'on');
    xlabel(plotAxis, 'pose');
    ylabel(plotAxis, labels{axis});
    legend(plotAxis, {'measured','predicted','compensated'}, ...
        'Location', 'best');
end
saveas(figureHandle, outputFile);
end

function waitUntil(timerObject, targetTime)
while true
    remaining = targetTime - toc(timerObject);
    if remaining <= 0
        return;
    elseif remaining > 0.003
        pause(remaining - 0.001);
    else
        pause(0);
    end
end
end

function difference = wrappedDifferenceDeg(target, actual)
difference = mod(double(target) - double(actual) + 180, 360) - 180;
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
