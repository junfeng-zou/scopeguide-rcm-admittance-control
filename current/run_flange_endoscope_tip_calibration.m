function [report, outputDirectory] = ...
        run_flange_endoscope_tip_calibration(options)
%RUN_FLANGE_ENDOSCOPE_TIP_CALIBRATION Read-only fixed-tip pivot calibration.
% The operator keeps the physical endoscope tip in one rigid divot and
% changes only the robot/flange orientation in BACKDRIVE mode.  This
% program reads 30004 feedback and never enables, disables or commands the
% robot.  Fixed-point pivot data identify the flange-to-tip translation;
% the tool-frame rotation is copied from the supplied/current config.

arguments
    options.Config = struct([])
    options.PoseCount (1, 1) double = 10
    options.SamplesPerPose (1, 1) double = 50
    options.SampleIntervalSec (1, 1) double = 0.02
    options.MinimumPoseSeparationDeg (1, 1) double = 8
    options.MinimumOrientationSpanDeg (1, 1) double = 30
    options.MaximumConditionNumber (1, 1) double = 1e4
    options.MaximumRmsResidualMm (1, 1) double = 1.0
    options.MaximumResidualMm (1, 1) double = 2.0
    options.MaximumFeedbackAgeSec (1, 1) double = 0.10
    options.MaximumJointSpeedDegSec (1, 1) double = 0.20
    options.MaximumTcpTranslationSpeedMSec (1, 1) double = 0.001
    options.MaximumTcpRotationSpeedRadSec (1, 1) double = deg2rad(1.0)
    options.MaximumMovingSampleRatio (1, 1) double = 0.05
    options.MaximumPositionSpreadMm (1, 1) double = 0.25
    options.MaximumOrientationSpreadDeg (1, 1) double = 0.15
    options.RequireTypedConfirmation (1, 1) logical = true
    options.WriteResults (1, 1) logical = true
    options.SaveFigure (1, 1) logical = true
    options.OutputRoot (1, 1) string = ""
end

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'robot'));
if isempty(options.Config)
    cfg = defaultRcmAdmittanceConfig();
else
    cfg = scopeguide.runtime.upgradeRcmAdmittanceConfig(options.Config);
end
validateRcmAdmittanceConfig(cfg);
validateOptions(options);
if string(cfg.robot.ControllerPoseReference) ~= "flange" || ...
        ~cfg.robot.ControllerPoseUnitsVerified
    error('scopeguide:calibration:FlangeFeedbackReferenceRequired', ...
        ['The calibration requires verified 30004 Cartesian feedback ' ...
         'referenced to the robot flange.']);
end

rotationFlangeToEndoscope = ...
    double(cfg.tool.TFlangeEndoscope(1:3, 1:3));
outputDirectory = "";
if options.WriteResults
    outputRoot = options.OutputRoot;
    if strlength(outputRoot) == 0
        outputRoot = fullfile(projectRoot, 'results');
    end
    if ~isfolder(outputRoot)
        mkdir(outputRoot);
    end
    stamp = char(datetime('now', ...
        'Format', 'yyyyMMdd_HHmmss_SSS'));
    outputDirectory = fullfile(outputRoot, ...
        "flange_endoscope_tip_calibration_" + string(stamp));
    mkdir(outputDirectory);
end

printSafetyBanner(cfg, options, rotationFlangeToEndoscope, ...
    outputDirectory);
if options.RequireTypedConfirmation
    confirmation = input([ ...
        'Type CALIBRATE_FLANGE_TIP after the fixed divot and emergency ' ...
        'stop are ready: '], 's');
    if ~strcmp(confirmation, 'CALIBRATE_FLANGE_TIP')
        error('scopeguide:calibration:OperatorCancelled', ...
            'Tip calibration cancelled before connecting to the robot.');
    end
end

robot = ZJFDobotCR5(char(cfg.robot.IPAddress), ...
    cfg.robot.DashboardPort, cfg.robot.MovePort, ...
    cfg.robot.FeedbackPort);
cleanup = onCleanup(@() disconnectRobot(robot));
robot.Connect();
waitForInitialFeedback(robot, cfg.robot.InitialFeedbackTimeoutSec);

captures = repmat(blankCapture(), options.PoseCount, 1);
acceptedRotations = zeros(3, 3, 0);
for poseIndex = 1:options.PoseCount
    fprintf('\n=== Fixed-tip pose %d / %d ===\n', ...
        poseIndex, options.PoseCount);
    fprintf(['Keep the physical tip seated in the SAME rigid divot. ' ...
        'Change flange orientation without slipping the tip, then let ' ...
        'the robot settle.\n']);
    expected = sprintf('SAMPLE %d', poseIndex);
    response = input(sprintf('Type "%s" to capture this pose: ', ...
        expected), 's');
    if ~strcmp(response, expected)
        error('scopeguide:calibration:OperatorCancelled', ...
            'Calibration cancelled before pose %d.', poseIndex);
    end

    capture = captureOnePose(robot, cfg, options, poseIndex);
    if ~capture.Stationary
        printRejectedCapture(capture);
        error('scopeguide:calibration:PoseNotStationary', ...
            ['Pose %d was not stationary. No result was accepted for ' ...
             'this pose; rerun the calibration.'], poseIndex);
    end
    if poseIndex > 1
        separationRad = minimumRotationDistance( ...
            capture.MeanRotationBaseFromFlange, acceptedRotations);
        capture.MinimumSeparationFromEarlierRad = separationRad;
        if separationRad < deg2rad(options.MinimumPoseSeparationDeg)
            error('scopeguide:calibration:PoseOrientationTooSimilar', ...
                ['Pose %d is only %.3f deg from an earlier pose; at ' ...
                 'least %.3f deg is required. Rerun and use more varied ' ...
                 'flange orientations.'], poseIndex, rad2deg(separationRad), ...
                options.MinimumPoseSeparationDeg);
        end
    end
    captures(poseIndex) = capture;
    acceptedRotations(:, :, end + 1) = ...
        capture.MeanRotationBaseFromFlange; %#ok<AGROW>
    fprintf(['Accepted pose %d: position spread %.4f mm; orientation ' ...
        'spread %.4f deg; moving samples %.2f%%.\n'], poseIndex, ...
        1e3 * capture.MaximumPositionDeviationM, ...
        rad2deg(capture.MaximumOrientationDeviationRad), ...
        100 * capture.MovingSampleRatio);
    if options.WriteResults
        checkpointFile = fullfile(outputDirectory, ...
            'capture_checkpoint.mat');
        save(checkpointFile, 'captures', 'cfg', 'options', ...
            'rotationFlangeToEndoscope');
    end
end

flangePositionBaseM = vertcat(captures.MeanPositionBaseM);
rotationBaseFromFlange = cat(3, ...
    captures.MeanRotationBaseFromFlange);
fit = scopeguide.calibration.fitFixedTipTranslation( ...
    flangePositionBaseM, rotationBaseFromFlange, ...
    RotationFlangeToEndoscope=rotationFlangeToEndoscope, ...
    MinimumOrientationSpanRad=deg2rad( ...
        options.MinimumOrientationSpanDeg), ...
    MaximumConditionNumber=options.MaximumConditionNumber, ...
    MaximumRmsResidualM=1e-3 * options.MaximumRmsResidualMm, ...
    MaximumResidualM=1e-3 * options.MaximumResidualMm);

report = fit;
report.CreatedUTC = char(datetime('now', 'TimeZone', 'UTC', ...
    'Format', 'yyyy-MM-dd''T''HH:mm:ss.SSSXXX'));
report.RobotIPAddress = string(cfg.robot.IPAddress);
report.ControllerPoseReference = string( ...
    cfg.robot.ControllerPoseReference);
report.CaptureMode = "operator_guided_fixed_tip_read_only";
report.RobotModeWasRecordedButNotUsedAsGate = true;
report.ReadOnlyRobotFeedback = true;
report.ProgramCalledEnableRobot = false;
report.ProgramCalledDisableRobot = false;
report.MotionCommandsSent = 0;
report.RotationSource = ...
    "copied_from_input_config_not_observable_by_fixed_point_pivot";
report.PoseSummaries = buildPoseSummary(captures, fit);

if options.WriteResults
    rawSamples = buildRawSampleTable(captures);
    poseSummary = report.PoseSummaries;
    writetable(rawSamples, fullfile(outputDirectory, 'raw_samples.csv'));
    writetable(poseSummary, fullfile(outputDirectory, 'pose_summary.csv'));
    save(fullfile(outputDirectory, 'calibration.mat'), ...
        'report', 'captures', 'rawSamples', 'poseSummary', 'cfg', 'options');
    jsonReport = report;
    jsonReport.PoseSummaries = table2struct(poseSummary);
    writeJson(fullfile(outputDirectory, 'calibration.json'), jsonReport);
    if options.SaveFigure
        writeResidualFigure(report, fullfile(outputDirectory, ...
            'fixed_tip_residuals.png'));
    end
end

printResult(report, outputDirectory);
end

function capture = captureOnePose(robot, cfg, options, poseIndex)
count = options.SamplesPerPose;
positionBaseM = zeros(count, 3);
rotationBaseFromFlange = zeros(3, 3, count);
quaternionWxyz = zeros(count, 4);
jointAnglesDeg = zeros(count, 6);
jointSpeedDegSec = zeros(count, 6);
tcpSpeed = zeros(count, 6);
feedbackSequence = zeros(count, 1, 'uint64');
feedbackAgeSec = zeros(count, 1);
robotMode = strings(count, 1);
lastSequence = uint64(0);
for sampleIndex = 1:count
    snapshot = waitForNewSnapshot(robot, lastSequence, ...
        max(0.5, 5 * options.SampleIntervalSec));
    lastSequence = snapshot.feedbackSequence;
    if snapshot.feedbackAgeSec > options.MaximumFeedbackAgeSec
        error('scopeguide:calibration:StaleCalibrationFeedback', ...
            'Pose %d sample %d feedback age %.4f s exceeds %.4f s.', ...
            poseIndex, sampleIndex, snapshot.feedbackAgeSec, ...
            options.MaximumFeedbackAgeSec);
    end
    cartesianPose = double(snapshot.cartesianPose(:)).';
    quaternion = double(snapshot.actualQuaternionWxyz(:)).';
    if numel(cartesianPose) ~= 6 || any(~isfinite(cartesianPose)) || ...
            numel(quaternion) ~= 4 || any(~isfinite(quaternion))
        error('scopeguide:calibration:InvalidCalibrationFeedback', ...
            'Pose %d sample %d contains invalid Cartesian feedback.', ...
            poseIndex, sampleIndex);
    end
    positionBaseM(sampleIndex, :) = cartesianPose(1:3) * ...
        cfg.robot.ControllerPoseTranslationScaleToM;
    rotationBaseFromFlange(:, :, sampleIndex) = ...
        scopeguide.geometry.rotationMatrixFromQuaternionWxyz(quaternion);
    quaternionWxyz(sampleIndex, :) = quaternion;
    jointAnglesDeg(sampleIndex, :) = ...
        double(snapshot.jointAnglesDeg(:)).';
    jointSpeedDegSec(sampleIndex, :) = ...
        double(snapshot.actualJointSpeedsDegSec(:)).';
    tcpSpeed(sampleIndex, :) = double(snapshot.actualTCPSpeed(:)).';
    feedbackSequence(sampleIndex) = snapshot.feedbackSequence;
    feedbackAgeSec(sampleIndex) = snapshot.feedbackAgeSec;
    robotMode(sampleIndex) = string(snapshot.robotMode);
    pause(options.SampleIntervalSec);
end

meanPosition = mean(positionBaseM, 1);
meanRotation = meanRotationMatrix(rotationBaseFromFlange);
positionDeviationM = vecnorm(positionBaseM - meanPosition, 2, 2);
orientationDeviationRad = zeros(count, 1);
for index = 1:count
    orientationDeviationRad(index) = ...
        scopeguide.geometry.rotationDistance( ...
        rotationBaseFromFlange(:, :, index), meanRotation);
end
jointSpeedMaximumDegSec = max(abs(jointSpeedDegSec), [], 2);
tcpTranslationSpeedMSec = vecnorm(tcpSpeed(:, 1:3), 2, 2) * ...
    cfg.robot.TcpLinearVelocityScaleToMSec;
tcpRotationSpeedRadSec = vecnorm(tcpSpeed(:, 4:6), 2, 2) * ...
    cfg.robot.TcpAngularVelocityScaleToRadSec;
moving = jointSpeedMaximumDegSec > ...
    options.MaximumJointSpeedDegSec | ...
    tcpTranslationSpeedMSec > ...
    options.MaximumTcpTranslationSpeedMSec | ...
    tcpRotationSpeedRadSec > options.MaximumTcpRotationSpeedRadSec;

capture = blankCapture();
capture.PoseIndex = poseIndex;
capture.PositionBaseM = positionBaseM;
capture.RotationBaseFromFlange = rotationBaseFromFlange;
capture.QuaternionWxyz = quaternionWxyz;
capture.JointAnglesDeg = jointAnglesDeg;
capture.JointSpeedDegSec = jointSpeedDegSec;
capture.TcpSpeed = tcpSpeed;
capture.FeedbackSequence = feedbackSequence;
capture.FeedbackAgeSec = feedbackAgeSec;
capture.RobotMode = robotMode;
capture.MeanPositionBaseM = meanPosition;
capture.MeanRotationBaseFromFlange = meanRotation;
capture.MeanJointAnglesDeg = mean(jointAnglesDeg, 1);
capture.MaximumPositionDeviationM = max(positionDeviationM);
capture.MaximumOrientationDeviationRad = max(orientationDeviationRad);
capture.MaximumJointSpeedDegSec = max(jointSpeedMaximumDegSec);
capture.MaximumTcpTranslationSpeedMSec = ...
    max(tcpTranslationSpeedMSec);
capture.MaximumTcpRotationSpeedRadSec = max(tcpRotationSpeedRadSec);
capture.MovingSampleRatio = mean(moving);
capture.Stationary = capture.MovingSampleRatio <= ...
    options.MaximumMovingSampleRatio && ...
    capture.MaximumPositionDeviationM <= ...
    1e-3 * options.MaximumPositionSpreadMm && ...
    capture.MaximumOrientationDeviationRad <= ...
    deg2rad(options.MaximumOrientationSpreadDeg);
end

function snapshot = waitForNewSnapshot(robot, previousSequence, timeoutSec)
clock = tic;
while toc(clock) <= timeoutSec
    try
        candidate = robot.GetStateSnapshot();
        if candidate.feedbackSequence > previousSequence
            snapshot = candidate;
            return;
        end
    catch exception
        if ~strcmp(exception.identifier, 'ZJFDobotCR5:NoFeedback')
            rethrow(exception);
        end
    end
    pause(0.002);
end
error('scopeguide:calibration:CalibrationFeedbackTimeout', ...
    'No new validated 30004 feedback arrived within %.3f s.', timeoutSec);
end

function waitForInitialFeedback(robot, timeoutSec)
waitForNewSnapshot(robot, uint64(0), timeoutSec);
end

function capture = blankCapture()
capture = struct( ...
    'PoseIndex', 0, ...
    'PositionBaseM', zeros(0, 3), ...
    'RotationBaseFromFlange', zeros(3, 3, 0), ...
    'QuaternionWxyz', zeros(0, 4), ...
    'JointAnglesDeg', zeros(0, 6), ...
    'JointSpeedDegSec', zeros(0, 6), ...
    'TcpSpeed', zeros(0, 6), ...
    'FeedbackSequence', zeros(0, 1, 'uint64'), ...
    'FeedbackAgeSec', zeros(0, 1), ...
    'RobotMode', strings(0, 1), ...
    'MeanPositionBaseM', nan(1, 3), ...
    'MeanRotationBaseFromFlange', nan(3, 3), ...
    'MeanJointAnglesDeg', nan(1, 6), ...
    'MaximumPositionDeviationM', NaN, ...
    'MaximumOrientationDeviationRad', NaN, ...
    'MaximumJointSpeedDegSec', NaN, ...
    'MaximumTcpTranslationSpeedMSec', NaN, ...
    'MaximumTcpRotationSpeedRadSec', NaN, ...
    'MovingSampleRatio', NaN, ...
    'MinimumSeparationFromEarlierRad', NaN, ...
    'Stationary', false);
end

function rotation = meanRotationMatrix(rotations)
matrixSum = sum(rotations, 3);
[left, ~, right] = svd(matrixSum);
correction = diag([1, 1, det(left * right')]);
rotation = left * correction * right';
end

function distance = minimumRotationDistance(rotation, earlier)
distance = Inf;
for index = 1:size(earlier, 3)
    distance = min(distance, scopeguide.geometry.rotationDistance( ...
        rotation, earlier(:, :, index)));
end
end

function summary = buildPoseSummary(captures, fit)
count = numel(captures);
poseIndex = (1:count).';
position = vertcat(captures.MeanPositionBaseM);
joint = vertcat(captures.MeanJointAnglesDeg);
positionSpreadMm = 1e3 * vertcat( ...
    captures.MaximumPositionDeviationM);
orientationSpreadDeg = rad2deg(vertcat( ...
    captures.MaximumOrientationDeviationRad));
movingRatio = vertcat(captures.MovingSampleRatio);
minimumSeparationDeg = rad2deg(vertcat( ...
    captures.MinimumSeparationFromEarlierRad));
residual = fit.ResidualBaseM;
residualNormMm = 1e3 * fit.ResidualNormM;
rotationRows = zeros(count, 9);
for index = 1:count
    rotationRows(index, :) = reshape( ...
        captures(index).MeanRotationBaseFromFlange.', 1, 9);
end
summary = table(poseIndex, position(:, 1), position(:, 2), ...
    position(:, 3), rotationRows(:, 1), rotationRows(:, 2), ...
    rotationRows(:, 3), rotationRows(:, 4), rotationRows(:, 5), ...
    rotationRows(:, 6), rotationRows(:, 7), rotationRows(:, 8), ...
    rotationRows(:, 9), joint(:, 1), joint(:, 2), joint(:, 3), ...
    joint(:, 4), joint(:, 5), joint(:, 6), positionSpreadMm, ...
    orientationSpreadDeg, movingRatio, minimumSeparationDeg, ...
    1e3 * residual(:, 1), 1e3 * residual(:, 2), ...
    1e3 * residual(:, 3), residualNormMm, ...
    'VariableNames', {'PoseIndex','FlangeXBaseM','FlangeYBaseM', ...
    'FlangeZBaseM','R11','R12','R13','R21','R22','R23', ...
    'R31','R32','R33','J1Deg','J2Deg','J3Deg','J4Deg','J5Deg', ...
    'J6Deg','PositionSpreadMm','OrientationSpreadDeg', ...
    'MovingSampleRatio','MinimumEarlierSeparationDeg', ...
    'ResidualXmm','ResidualYmm','ResidualZmm','ResidualNormMm'});
end

function output = buildRawSampleTable(captures)
poseCount = numel(captures);
sampleCount = size(captures(1).PositionBaseM, 1);
rowCount = poseCount * sampleCount;
poseIndex = zeros(rowCount, 1);
sampleIndex = zeros(rowCount, 1);
sequence = zeros(rowCount, 1, 'uint64');
feedbackAgeSec = zeros(rowCount, 1);
position = zeros(rowCount, 3);
quaternion = zeros(rowCount, 4);
joint = zeros(rowCount, 6);
jointSpeed = zeros(rowCount, 6);
tcpSpeed = zeros(rowCount, 6);
row = 0;
for pose = 1:poseCount
    rows = row + (1:sampleCount);
    poseIndex(rows) = pose;
    sampleIndex(rows) = (1:sampleCount).';
    sequence(rows) = captures(pose).FeedbackSequence;
    feedbackAgeSec(rows) = captures(pose).FeedbackAgeSec;
    position(rows, :) = captures(pose).PositionBaseM;
    quaternion(rows, :) = captures(pose).QuaternionWxyz;
    joint(rows, :) = captures(pose).JointAnglesDeg;
    jointSpeed(rows, :) = captures(pose).JointSpeedDegSec;
    tcpSpeed(rows, :) = captures(pose).TcpSpeed;
    row = row + sampleCount;
end
output = table(poseIndex, sampleIndex, sequence, feedbackAgeSec, ...
    position(:, 1), position(:, 2), position(:, 3), ...
    quaternion(:, 1), quaternion(:, 2), quaternion(:, 3), ...
    quaternion(:, 4), joint(:, 1), joint(:, 2), joint(:, 3), ...
    joint(:, 4), joint(:, 5), joint(:, 6), ...
    jointSpeed(:, 1), jointSpeed(:, 2), jointSpeed(:, 3), ...
    jointSpeed(:, 4), jointSpeed(:, 5), jointSpeed(:, 6), ...
    tcpSpeed(:, 1), tcpSpeed(:, 2), tcpSpeed(:, 3), ...
    tcpSpeed(:, 4), tcpSpeed(:, 5), tcpSpeed(:, 6), ...
    'VariableNames', {'PoseIndex','SampleIndex','FeedbackSequence', ...
    'FeedbackAgeSec','FlangeXBaseM','FlangeYBaseM','FlangeZBaseM', ...
    'Qw','Qx','Qy','Qz','J1Deg','J2Deg','J3Deg','J4Deg','J5Deg', ...
    'J6Deg','J1SpeedDegSec','J2SpeedDegSec','J3SpeedDegSec', ...
    'J4SpeedDegSec','J5SpeedDegSec','J6SpeedDegSec', ...
    'TcpVx','TcpVy','TcpVz','TcpWx','TcpWy','TcpWz'});
end

function writeResidualFigure(report, filePath)
figureHandle = figure('Visible', 'off', 'Color', 'w', ...
    'Name', 'Fixed-tip flange calibration residuals');
cleanup = onCleanup(@() close(figureHandle));
tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile;
bar(1e3 * report.ResidualBaseM, 'grouped');
grid on;
xlabel('Pose index');
ylabel('Residual [mm]');
legend({'X','Y','Z'}, 'Location', 'best');
title('Fixed-tip residual components');
nexttile;
bar(1e3 * report.ResidualNormM);
hold on;
yline(1e3 * report.MaximumAllowedResidualM, 'r--', ...
    'Maximum allowed');
grid on;
xlabel('Pose index');
ylabel('Residual norm [mm]');
title(sprintf('RMS %.3f mm; max %.3f mm; valid=%d', ...
    1e3 * report.RmsResidualM, 1e3 * report.MaximumResidualM, ...
    report.Valid));
exportgraphics(figureHandle, filePath, 'Resolution', 180);
end

function printSafetyBanner(cfg, options, rotation, outputDirectory)
fprintf('\n=== Flange-to-endoscope-tip fixed-point calibration ===\n');
fprintf('Robot: %s; poses: %d; samples/pose: %d.\n', ...
    string(cfg.robot.IPAddress), options.PoseCount, ...
    options.SamplesPerPose);
fprintf(['READ-ONLY: no EnableRobot, DisableRobot, ServoJ, MovJ or other ' ...
    'motion command will be sent.\n']);
fprintf(['Use the pendant/manual guidance while repositioning and let the ' ...
    'robot settle before each capture.\n']);
fprintf(['RobotMode is recorded for traceability but is NOT used as a ' ...
    'capture gate.\n']);
fprintf(['Keep the physical tip in one rigid divot and vary the flange ' ...
    'orientation by at least %.1f deg overall.\n'], ...
    options.MinimumOrientationSpanDeg);
fprintf(['This experiment estimates translation only. The following known ' ...
    'flange-to-endoscope rotation will be retained:\n']);
disp(rotation);
if strlength(outputDirectory) > 0
    fprintf('Checkpoint/results directory: %s\n', outputDirectory);
end
end

function printRejectedCapture(capture)
fprintf(['Rejected capture: position spread %.4f mm; orientation spread ' ...
    '%.4f deg; moving samples %.2f%%; max joint speed %.3f deg/s.\n'], ...
    1e3 * capture.MaximumPositionDeviationM, ...
    rad2deg(capture.MaximumOrientationDeviationRad), ...
    100 * capture.MovingSampleRatio, ...
    capture.MaximumJointSpeedDegSec);
end

function printResult(report, outputDirectory)
fprintf('\n=== Flange-to-endoscope-tip calibration result ===\n');
fprintf('Validity: %s\n', string(report.Valid));
fprintf('Tip translation in flange frame [m]: [%+.9f %+.9f %+.9f]\n', ...
    report.TipTranslationFlangeM);
fprintf('Tip translation in flange frame [mm]: [%+.3f %+.3f %+.3f]\n', ...
    1e3 * report.TipTranslationFlangeM);
fprintf('Fixed tip point in robot base [m]: [%+.6f %+.6f %+.6f]\n', ...
    report.FixedTipPointBaseM);
fprintf(['Orientation span %.2f deg; design rank %d/6; condition ' ...
    'number %.3g.\n'], rad2deg(report.OrientationSpanRad), ...
    report.DesignRank, report.DesignConditionNumber);
fprintf('Fixed-point residual RMS/max: %.3f / %.3f mm.\n', ...
    1e3 * report.RmsResidualM, 1e3 * report.MaximumResidualM);
fprintf('TFlangeEndoscope = [\n');
for row = 1:4
    values = report.TFlangeEndoscope(row, :);
    suffix = conditional(row < 4, ';', '');
    fprintf('  %.9f %.9f %.9f %.9f%s\n', ...
        values(1), values(2), values(3), values(4), suffix);
end
fprintf('];\n');
if ~report.Valid
    fprintf('NOT ACCEPTED: %s\n', strjoin(report.FailureReasons, ', '));
end
if strlength(outputDirectory) > 0
    fprintf('Results: %s\n', outputDirectory);
end
fprintf(['Rotation was not estimated by this fixed-point experiment; do ' ...
    'not interpret the output as a full six-DOF hand-eye calibration.\n']);
end

function value = conditional(condition, trueValue, falseValue)
if condition
    value = trueValue;
else
    value = falseValue;
end
end

function validateOptions(options)
integerNames = {'PoseCount','SamplesPerPose'};
for index = 1:numel(integerNames)
    name = integerNames{index};
    value = options.(name);
    if ~isfinite(value) || value ~= fix(value) || value < 1
        error('scopeguide:calibration:InvalidTipCalibrationOption', ...
            '%s must be a positive integer.', name);
    end
end
if options.PoseCount < 6 || options.SamplesPerPose < 10
    error('scopeguide:calibration:InsufficientTipCalibrationData', ...
        'Use at least 6 poses and at least 10 samples per pose.');
end
positiveNames = {'SampleIntervalSec','MinimumPoseSeparationDeg', ...
    'MinimumOrientationSpanDeg','MaximumConditionNumber', ...
    'MaximumRmsResidualMm','MaximumResidualMm', ...
    'MaximumFeedbackAgeSec','MaximumJointSpeedDegSec', ...
    'MaximumTcpTranslationSpeedMSec', ...
    'MaximumTcpRotationSpeedRadSec','MaximumPositionSpreadMm', ...
    'MaximumOrientationSpreadDeg'};
for index = 1:numel(positiveNames)
    name = positiveNames{index};
    value = options.(name);
    if ~isfinite(value) || value <= 0
        error('scopeguide:calibration:InvalidTipCalibrationOption', ...
            '%s must be a finite positive scalar.', name);
    end
end
if options.MaximumMovingSampleRatio < 0 || ...
        options.MaximumMovingSampleRatio > 1
    error('scopeguide:calibration:InvalidTipCalibrationOption', ...
        'MaximumMovingSampleRatio must lie in [0,1].');
end
if options.MaximumResidualMm < options.MaximumRmsResidualMm
    error('scopeguide:calibration:InvalidTipCalibrationOption', ...
        'MaximumResidualMm must not be smaller than RMS bound.');
end
end

function writeJson(path, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end
file = fopen(path, 'w');
if file < 0
    error('scopeguide:calibration:CannotWriteTipCalibrationJson', ...
        'Cannot write %s.', path);
end
cleanup = onCleanup(@() fclose(file));
fwrite(file, encoded, 'char');
end

function disconnectRobot(robot)
if isempty(robot) || ~isvalid(robot) || ~robot.IsConnected
    return;
end
try
    robot.Disconnect();
catch exception
    warning('scopeguide:calibration:TipCalibrationDisconnectFailed', ...
        'Robot disconnect failed: %s', exception.message);
end
end
