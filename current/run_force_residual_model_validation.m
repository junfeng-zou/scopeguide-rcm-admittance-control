function [result, outputDirectory] = ...
        run_force_residual_model_validation(options)
%RUN_FORCE_RESIDUAL_MODEL_VALIDATION Identify residual-wrench source.
%
% The routine performs the following guarded experiment:
%   1. Capture nine contact-free static orientations plus a return to A.
%   2. Fit Original, BiasOnly, LoadOnly and Joint(bias+mass+CoG) models.
%   3. Fit only on the six predeclared training poses.
%   4. Judge the hypotheses on three independent validation poses.
%   5. Check A-return repeatability.
%   6. Write a versioned bias-only candidate only when every gate passes.
%
% The candidate is never activated. This routine never overwrites the
% source calibration or defaultRcmAdmittanceConfig.m, and never calls robot
% Enable/Reset/ClearError/Continue or HEX-H ZERO/Auto-calibration. A safety
% stop uses the V3 Pause() command; the program never resumes it itself.

arguments
    options.EnableMotion (1, 1) logical = false
    options.RequireTypedConfirmation (1, 1) logical = true
    options.ResumeDirectory (1, 1) string = ""
    options.SourceCalibrationFile (1, 1) string = ""
    options.OrientationOffsetsDeg (:, 3) double = defaultOffsets()
    options.TrainingIndices (1, :) double = [1 2 3 5 6 8]
    options.ValidationIndices (1, :) double = [4 7 9]
    options.MaximumOrientationTransitionDeg (1, 1) double = 15
    options.MaximumPredictedTipDisplacementM (1, 1) double = 0.15
    options.SpeedRatioPercent (1, 1) double = 10
    options.SampleDurationSec (1, 1) double = 20
    options.SampleRateHz (1, 1) double = 200
    options.StableHoldSec (1, 1) double = 1.0
    options.MoveTimeoutSec (1, 1) double = 60
    options.MotionStartTimeoutSec (1, 1) double = 3.0
    options.CartesianPositionToleranceMm (1, 1) double = 1.0
    options.CartesianOrientationToleranceDeg (1, 1) double = 0.5
    options.MaximumStationaryJointSpeedDegSec (1, 1) double = 0.15
    options.MaximumStationaryTcpTranslationSpeed (1, 1) double = 0.20
    options.MaximumStationaryTcpRotationSpeed (1, 1) double = 0.20
    options.MinimumConsecutiveMovingFeedbackFrames (1, 1) double = 2
    options.CaptureDerivedSpeedThresholdMultiplier (1, 1) double = 1.5
    options.MaximumFeedbackAgeSec (1, 1) double = 0.10
    options.MotionRawForceStopN (1, 1) double = 40
    options.MotionRawMomentStopNm (1, 1) double = 4
    options.MotionExternalForceStopN (1, 1) double = 25
    options.MotionExternalMomentStopNm (1, 1) double = 2
    options.MaximumConditionNumber (1, 1) double = 1e6
    options.MaximumValidationForceNormN (1, 1) double = 1.3
    options.MaximumValidationMomentNormNm (1, 1) double = 0.18
    options.MaximumValidationForceVectorRmseN (1, 1) double = 0.9
    options.MaximumValidationMomentVectorRmseNm (1, 1) double = 0.12
    options.MaximumTrainingCenteredForceNormN (1, 1) double = 1.0
    options.MaximumTrainingCenteredMomentNormNm (1, 1) double = 0.12
    options.MaximumReturnForceChangeN (1, 1) double = 0.5
    options.MaximumReturnMomentChangeNm (1, 1) double = 0.05
    options.MaximumBiasShiftForceNormN (1, 1) double = 15
    options.MaximumBiasShiftMomentNormNm (1, 1) double = 0.5
    options.BiasVsJointRelativeRatio (1, 1) double = 1.25
    options.BiasVsJointForceMarginN (1, 1) double = 0.2
    options.BiasVsJointMomentMarginNm (1, 1) double = 0.03
    options.SaveFigure (1, 1) logical = true
end

validateOptions(options);
if ~options.EnableMotion
    error('scopeguide:residualValidation:MotionNotExplicitlyEnabled', ...
        ['This experiment sends V3 MovJ commands. Re-run with ' ...
         'EnableMotion=true only after clearing and reviewing the complete ' ...
         'swept volume.']);
end

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'force_sensor'));
addpath(fullfile(projectRoot, 'force_calibration'));
addpath(fullfile(projectRoot, 'robot'));

cfg = defaultRcmAdmittanceConfig();
resumeMode = strlength(options.ResumeDirectory) > 0;
if resumeMode
    checkpoint = ...
        scopeguide.diagnostics.loadResidualValidationCheckpoint( ...
        projectRoot, options.ResumeDirectory, options);
    outputDirectory = checkpoint.Directory;
    paths = outputPaths(outputDirectory);
    allData = checkpoint.Data;
    poseSummaries = checkpoint.PoseSummaries;
    result = checkpoint.Result;
    plan = checkpoint.Plan;
    options = checkpoint.Options;
    sourceCalibrationFile = checkpoint.SourceCalibrationFile;
    startPoseIndex = checkpoint.NextPoseIndex;
    result.Paths = paths;
    timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
    writeJson(fullfile(outputDirectory, ...
        "resume_options_" + timestamp + ".json"), options);
    if isempty(allData)
        sessionTimeOffsetSec = 0;
    else
        sessionTimeOffsetSec = max(allData.HostSessionTimeSec);
    end
else
    if strlength(options.SourceCalibrationFile) == 0
        sourceCalibrationFile = string(cfg.force.CalibrationFile);
    else
        sourceCalibrationFile = options.SourceCalibrationFile;
    end
    [~, calibrationProvenance] = ...
        loadHexHGravityCalibration(sourceCalibrationFile);
    outputDirectory = createOutputDirectory(projectRoot);
    paths = outputPaths(outputDirectory);
    writeJson(paths.OptionsJson, options);
    allData = table();
    poseSummaries = repmat(emptyPoseSummary(), 0, 1);
    plan = struct();
    startPoseIndex = 1;
    sessionTimeOffsetSec = 0;
    result = initialResult(cfg, sourceCalibrationFile, ...
        calibrationProvenance, options, paths);
end
validateOptions(options);
[sourceCalibration, calibrationProvenance] = ...
    loadHexHGravityCalibration(sourceCalibrationFile);
sensorCfg = onrobot.defaultConfig();
sensorCfg.SampleRateHz = options.SampleRateHz;

robot = ZJFDobotCR5(cfg.robot.IPAddress);
sensor = onrobot.HexHClient(sensorCfg);
hardwareCleanup = onCleanup(@() disconnectHardware(robot, sensor));
motionCommandMayBePending = false;

try
    printSafetyBanner(options);
    robot.Connect();
    sensor.connect();
    waitForInitialFeedback(robot, cfg.robot.InitialFeedbackTimeoutSec);
    requireEnabledAndIdle(robot);
    requireSensorHealthy(sensor);
    waitUntilStationary(robot, options);
    requireMotionQueueReady(robot);

    if ~resumeMode
        snapshotA = robot.GetStateSnapshot();
        plan = scopeguide.diagnostics.buildResidualIdentificationPosePlan( ...
            snapshotA.cartesianPose, options.OrientationOffsetsDeg, ...
            options.TrainingIndices, options.ValidationIndices, ...
            options.MaximumOrientationTransitionDeg);
    end
    [tipPositions, maximumTipDisplacementM] = ...
        previewTipPositions(plan.TargetCartesianPose, cfg);
    if maximumTipDisplacementM > options.MaximumPredictedTipDisplacementM
        error('scopeguide:residualValidation:PredictedTipMotionTooLarge', ...
            ['Nominal preview predicts %.1f mm maximum endpoint ' ...
             'displacement from A, exceeding the configured %.1f mm.'], ...
            1000 * maximumTipDisplacementM, ...
            1000 * options.MaximumPredictedTipDisplacementM);
    end
    result.MotionPlan = plan;
    result.NominalEndoscopeTipPositionBaseM = tipPositions;
    result.MaximumPredictedTipDisplacementM = maximumTipDisplacementM;
    printMotionPlan(plan, tipPositions, maximumTipDisplacementM);
    if resumeMode
        fprintf(['Resuming existing dataset after %d completed poses. ' ...
            'Next pose: %s.\n'], startPoseIndex - 1, ...
            plan.Labels(startPoseIndex));
        requireConfirmation(options, 'RESUME RESIDUAL VALIDATION', ...
            ['Type "RESUME RESIDUAL VALIDATION" only after confirming ' ...
             'the original plan, current tool/cable state, and E-stop: ']);
    else
        requireConfirmation(options, 'RUN RESIDUAL VALIDATION', ...
            ['Type "RUN RESIDUAL VALIDATION" only after checking all ten ' ...
             'targets, the complete swept volume, cable slack, and E-stop: ']);
    end

    speedResponse = robot.SetSpeedRatio(options.SpeedRatioPercent);
    result.SpeedCommandResponse = string(speedResponse);
    sessionClock = tic;
    for poseIndex = startPoseIndex:numel(plan.Labels)
        label = plan.Labels(poseIndex);
        snapshot = robot.GetStateSnapshot();
        alreadyAtTarget = targetPoseReached(snapshot, ...
            plan.TargetCartesianPose(poseIndex, :), options);
        if poseIndex > 1 && ~alreadyAtTarget
            motionCommandMayBePending = true;
            moveAndWait(robot, sensor, sourceCalibration, ...
                plan.TargetCartesianPose(poseIndex, :), label, options);
            motionCommandMayBePending = false;
        elseif resumeMode && poseIndex == startPoseIndex
            fprintf('Current feedback already matches %s; no MovJ sent.\n', ...
                label);
        end
        requireConfirmation(options, "CAPTURE " + label, sprintf( ...
            ['Verify the endoscope is unloaded and cables are slack. ' ...
             'Type "CAPTURE %s": '], label));
        [poseData, poseSummary] = captureStaticPose( ...
            robot, sensor, sourceCalibration, label, poseIndex, ...
            sessionClock, sessionTimeOffsetSec, options);
        allData = [allData; poseData]; %#ok<AGROW>
        poseSummaries(end + 1, 1) = poseSummary; %#ok<AGROW>
        result.PoseSummaries = poseSummaries;
        result.SampleCount = height(allData);
        result.CompletedPoseCount = numel(poseSummaries);
        saveCheckpoint(allData, poseSummaries, result, paths);
    end

    uniqueCount = plan.UniquePoseCount;
    poseWrench = vertcat(poseSummaries(1:uniqueCount).RawMedian);
    poseQuaternion = vertcat( ...
        poseSummaries(1:uniqueCount).QuaternionMeanWxyz);
    comparison = compareHexHResidualModels( ...
        poseWrench, poseQuaternion, sourceCalibration, ...
        plan.TrainingIndices, plan.ValidationIndices, ...
        'MaximumConditionNumber', options.MaximumConditionNumber);

    returnRaw = poseSummaries(plan.ReturnPoseIndex).RawMedian;
    returnQuaternion = ...
        poseSummaries(plan.ReturnPoseIndex).QuaternionMeanWxyz;
    returnResidual = residualForCalibration( ...
        returnRaw, returnQuaternion, ...
        comparison.Models.BiasOnly.Calibration);
    initialResidual = comparison.Models.BiasOnly.ResidualSensor(1, :);
    returnResidualChange = returnResidual - initialResidual;
    assessment = assessCandidate(comparison, returnResidualChange, options);

    modelTable = buildModelComparisonTable(comparison);
    poseTable = buildPoseSummaryTable(poseSummaries, plan, comparison, ...
        returnResidual);
    writetable(modelTable, paths.ModelCsv);
    writetable(poseTable, paths.PoseCsv);
    result.ModelComparison = comparison;
    result.ModelComparisonTable = modelTable;
    result.Assessment = assessment;
    result.ReturnResidualChangeSensor = returnResidualChange;
    result.Completed = true;
    result.Status = assessment.Status;

    if assessment.Passed
        [candidateJson, candidateMat] = writeCandidate( ...
            outputDirectory, comparison.Models.BiasOnly.Calibration, ...
            sourceCalibrationFile, calibrationProvenance, plan, ...
            assessment, comparison);
        result.CandidateCalibrationJson = string(candidateJson);
        result.CandidateCalibrationMat = string(candidateMat);
    else
        result.CandidateCalibrationJson = "";
        result.CandidateCalibrationMat = "";
    end
    if options.SaveFigure
        saveResidualFigure(comparison, plan, paths.FigurePng);
    end
    saveCheckpoint(allData, poseSummaries, result, paths);
    printFinalResult(result, modelTable, outputDirectory);
catch exception
    result.Status = "failed";
    result.ErrorIdentifier = string(exception.identifier);
    result.ErrorMessage = string(exception.message);
    result.SampleCount = height(allData);
    result.PoseSummaries = poseSummaries;
    try
        if robot.IsConnected && motionCommandMayBePending
            robot.StopMove();
        end
    catch stopException
        result.StopMoveError = string(stopException.message);
    end
    saveCheckpoint(allData, poseSummaries, result, paths);
    rethrow(exception);
end
end

function offsets = defaultOffsets()
offsets = [0 0 0; 10 0 0; 10 10 0; 0 10 0; -10 10 0; ...
    -10 0 0; -10 -10 0; 0 -10 0; 10 -10 0];
end

function validateOptions(options)
positive = {'MaximumOrientationTransitionDeg', ...
    'MaximumPredictedTipDisplacementM','SpeedRatioPercent', ...
    'SampleDurationSec','SampleRateHz','StableHoldSec','MoveTimeoutSec', ...
    'MotionStartTimeoutSec','CartesianPositionToleranceMm', ...
    'CartesianOrientationToleranceDeg', ...
    'MaximumStationaryJointSpeedDegSec', ...
    'MaximumStationaryTcpTranslationSpeed', ...
    'MaximumStationaryTcpRotationSpeed','MaximumFeedbackAgeSec', ...
    'CaptureDerivedSpeedThresholdMultiplier', ...
    'MotionRawForceStopN','MotionRawMomentStopNm', ...
    'MotionExternalForceStopN','MotionExternalMomentStopNm', ...
    'MaximumConditionNumber','MaximumValidationForceNormN', ...
    'MaximumValidationMomentNormNm', ...
    'MaximumValidationForceVectorRmseN', ...
    'MaximumValidationMomentVectorRmseNm', ...
    'MaximumTrainingCenteredForceNormN', ...
    'MaximumTrainingCenteredMomentNormNm','MaximumReturnForceChangeN', ...
    'MaximumReturnMomentChangeNm','MaximumBiasShiftForceNormN', ...
    'MaximumBiasShiftMomentNormNm','BiasVsJointRelativeRatio', ...
    'BiasVsJointForceMarginN','BiasVsJointMomentMarginNm'};
for index = 1:numel(positive)
    value = options.(positive{index});
    if ~isscalar(value) || ~isfinite(value) || value <= 0
        error('scopeguide:residualValidation:InvalidOption', ...
            '%s must be finite and positive.', positive{index});
    end
end
if options.SpeedRatioPercent > 20
    error('scopeguide:residualValidation:UnsafeSpeedRatio', ...
        'SpeedRatioPercent is limited to 20%%.');
end
if options.MinimumConsecutiveMovingFeedbackFrames < 1 || ...
        options.MinimumConsecutiveMovingFeedbackFrames ~= ...
        round(options.MinimumConsecutiveMovingFeedbackFrames)
    error('scopeguide:residualValidation:InvalidMovingFrameCount', ...
        'MinimumConsecutiveMovingFeedbackFrames must be an integer.');
end
end

function printSafetyBanner(options)
fprintf('\n============================================================\n');
fprintf('Nova5 / HEX-H RESIDUAL MODEL VALIDATION\n');
fprintf('V3 TCP/IP MovJ commands will run at %.1f%% speed.\n', ...
    options.SpeedRatioPercent);
fprintf('Nine unique poses: 6 fixed training + 3 held-out validation.\n');
fprintf('Each pose records %.1f s at %.1f Hz; pose 10 returns to A.\n', ...
    options.SampleDurationSec, options.SampleRateHz);
fprintf('Required before continuing:\n');
fprintf('  1. Endoscope is outside every patient/phantom.\n');
fprintf('  2. Every target and the complete swept volume are clear.\n');
fprintf('  3. Tool is unloaded; cables remain slack at every pose.\n');
fprintf('  4. Robot is already enabled/idle; E-stop is reachable.\n');
fprintf('  5. Do not use HEX-H ZERO or Auto-calibration.\n');
fprintf('No source/default calibration file will be overwritten.\n');
fprintf('============================================================\n\n');
end

function result = initialResult(cfg, sourceFile, provenance, options, paths)
result = struct();
result.Status = "initialized";
result.Completed = false;
result.CreatedLocal = string(datetime('now'));
result.RobotIPAddress = string(cfg.robot.IPAddress);
result.SourceCalibrationFile = string(sourceFile);
result.CalibrationProvenance = provenance;
result.Options = options;
result.Paths = paths;
result.SampleCount = 0;
result.CompletedPoseCount = 0;
result.CandidateActivationAuthorized = false;
end

function paths = outputPaths(directory)
paths = struct();
paths.SignalsCsv = string(fullfile(directory, 'raw_samples.csv'));
paths.PoseCsv = string(fullfile(directory, 'pose_summary.csv'));
paths.ModelCsv = string(fullfile(directory, 'model_comparison.csv'));
paths.Mat = string(fullfile(directory, 'diagnostic.mat'));
paths.SummaryJson = string(fullfile(directory, 'summary.json'));
paths.OptionsJson = string(fullfile(directory, 'options.json'));
paths.FigurePng = string(fullfile(directory, 'residual_models.png'));
end

function [positions, maximumDisplacement] = ...
        previewTipPositions(targetPose, cfg)
positions = zeros(size(targetPose, 1), 3);
for index = 1:size(targetPose, 1)
    transform = eye(4);
    transform(1:3, 1:3) = scopeguide.geometry.dobotRpyToRotation( ...
        targetPose(index, 4:6), ...
        cfg.robot.ControllerPoseAngleScaleToRad);
    transform(1:3, 4) = targetPose(index, 1:3).' * ...
        cfg.robot.ControllerPoseTranslationScaleToM;
    tipTransform = transform * cfg.tool.TFlangeEndoscope;
    positions(index, :) = tipTransform(1:3, 4).';
end
maximumDisplacement = max(vecnorm(positions - positions(1, :), 2, 2));
end

function printMotionPlan(plan, tipPositions, maximumDisplacement)
fprintf('Fixed pose plan (chosen before any fitting):\n');
for index = 1:numel(plan.Labels)
    if index <= plan.UniquePoseCount && ...
            ismember(index, plan.TrainingIndices)
        role = 'TRAIN';
    elseif index <= plan.UniquePoseCount
        role = 'HELD-OUT';
    else
        role = 'RETURN CHECK';
    end
    fprintf('  %-14s %-12s pose_mm_deg=[%s] tip_m=[%s]\n', ...
        plan.Labels(index), role, ...
        vectorText(plan.TargetCartesianPose(index, :)), ...
        vectorText(tipPositions(index, :)));
end
fprintf('Maximum nominal tip displacement from A: %.1f mm\n', ...
    1000 * maximumDisplacement);
fprintf(['Preview assumes controller CartesianPose is the flange pose. ' ...
    'It is not collision detection.\n']);
end

function moveAndWait(robot, sensor, calibration, target, label, options)
requireEnabledAndIdle(robot);
requireMotionQueueReady(robot);
requireSensorHealthy(sensor);
snapshot = robot.GetStateSnapshot();
stepDeg = orientationErrorDeg(target(4:6), ...
    snapshot.actualQuaternionWxyz);
if stepDeg > options.MaximumOrientationTransitionDeg
    error('scopeguide:residualValidation:LiveTransitionTooLarge', ...
        'Transition to %s is %.2f deg; limit is %.2f deg.', ...
        label, stepDeg, options.MaximumOrientationTransitionDeg);
end
expected = "MOVE " + label;
requireConfirmation(options, expected, sprintf( ...
    'Type "%s" to send V3 MovJ to %s: ', expected, label));
fprintf('Moving to %s at %.1f%% speed...\n', label, ...
    options.SpeedRatioPercent);
response = robot.MovJ(target);
fprintf('MovJ response: %s\n', strtrim(response));

moveClock = tic;
stableClock = [];
motionStarted = false;
while true
    wrenchSample = sensor.readSample();
    snapshot = robot.GetStateSnapshot();
    assertFreshFeedback(snapshot, options);
    mode = string(snapshot.robotMode);
    if any(mode == ["ERROR","PAUSE","DISABLED","POWER_OFF"])
        safeStop(robot);
        error('scopeguide:residualValidation:RobotModeDuringMove', ...
            'Robot entered %s while moving to %s.', mode, label);
    end
    compensated = compensateHexHWrench(wrenchSample.wrench, ...
        snapshot.actualQuaternionWxyz, calibration);
    enforceMotionWrenchLimits(wrenchSample.wrench, ...
        compensated.externalTool, robot, label, options);
    positionErrorMm = norm(snapshot.cartesianPose(1:3) - target(1:3));
    rotationErrorDeg = orientationErrorDeg( ...
        target(4:6), snapshot.actualQuaternionWxyz);
    stationary = isStationary(snapshot, options);
    motionStarted = motionStarted || mode == "RUNNING" || ~stationary;
    reached = positionErrorMm <= options.CartesianPositionToleranceMm && ...
        rotationErrorDeg <= options.CartesianOrientationToleranceDeg;
    if reached && stationary && mode == "ENABLE"
        if isempty(stableClock)
            stableClock = tic;
        elseif toc(stableClock) >= options.StableHoldSec
            fprintf('Reached %s; stationary for %.1f s.\n', ...
                label, options.StableHoldSec);
            return;
        end
    else
        stableClock = [];
    end
    if ~motionStarted && toc(moveClock) > options.MotionStartTimeoutSec
        controllerErrors = robot.GetErrorID();
        safeStop(robot);
        error('scopeguide:residualValidation:MotionQueueDidNotStart', ...
            ['MovJ accepted but %s did not start in %.1f s. Response=%s; ' ...
             'GetErrorID=%s; Mode=%s; RunQueuedCmd=%d; PauseFlag=%d.'], ...
            label, options.MotionStartTimeoutSec, strtrim(response), ...
            strtrim(controllerErrors), mode, snapshot.runQueuedCommand, ...
            snapshot.pauseCommandFlag);
    end
    if toc(moveClock) > options.MoveTimeoutSec
        safeStop(robot);
        error('scopeguide:residualValidation:MoveTimeout', ...
            ['Move to %s did not settle in %.1f s. Last errors: ' ...
             '%.3f mm, %.3f deg.'], label, options.MoveTimeoutSec, ...
            positionErrorMm, rotationErrorDeg);
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
    error('scopeguide:residualValidation:MotionWrenchStop', ...
        ['%s at %s. Raw %.2f N/%.3f Nm; current-compensated ' ...
         '%.2f N/%.3f Nm.'], reason, label, norm(raw(1:3)), ...
        norm(raw(4:6)), norm(external(1:3)), norm(external(4:6)));
end
end

function [data, summary] = captureStaticPose( ...
        robot, sensor, calibration, label, poseIndex, sessionClock, ...
        sessionTimeOffsetSec, options)
requireEnabledAndIdle(robot);
requireSensorHealthy(sensor);
waitUntilStationary(robot, options);
count = max(2, round(options.SampleDurationSec * options.SampleRateHz));
fprintf('Recording %s: %d samples at %.1f Hz for %.1f s...\n', ...
    label, count, options.SampleRateHz, options.SampleDurationSec);

raw = zeros(count, 6);
externalSensor = zeros(count, 6);
externalTool = zeros(count, 6);
joints = zeros(count, 6);
quaternion = zeros(count, 4);
cartesianPose = zeros(count, 6);
hostSessionTimeSec = zeros(count, 1);
sensorSequence = zeros(count, 1, 'uint64');
sensorReadDurationSec = zeros(count, 1);
robotSequence = zeros(count, 1, 'uint64');
robotFeedbackAgeSec = zeros(count, 1);
stationary = false(count, 1);

period = 1 / options.SampleRateHz;
recordClock = tic;
lastRobotSequence = uint64(0);
lastRobotUpdateClock = tic;
motionState = struct();
for index = 1:count
    waitUntil(recordClock, (index - 1) * period);
    wrenchSample = sensor.readSample();
    snapshot = robot.GetStateSnapshot();
    assertFreshFeedback(snapshot, options);
    if snapshot.feedbackSequence ~= lastRobotSequence
        lastRobotSequence = snapshot.feedbackSequence;
        lastRobotUpdateClock = tic;
    elseif toc(lastRobotUpdateClock) > options.MaximumFeedbackAgeSec
        error('scopeguide:residualValidation:FrozenFeedback', ...
            'Robot feedback froze while recording %s.', label);
    end
    still = isStationary(snapshot, options);
    [motionState, motionEvidence] = ...
        scopeguide.diagnostics.updateCaptureMotionEvidence( ...
        motionState, snapshot, ...
        'MaximumJointSpeedDegSec', ...
            options.MaximumStationaryJointSpeedDegSec, ...
        'MaximumTcpTranslationSpeedMmSec', ...
            options.MaximumStationaryTcpTranslationSpeed, ...
        'MaximumTcpRotationSpeedDegSec', ...
            options.MaximumStationaryTcpRotationSpeed, ...
        'DerivedSpeedThresholdMultiplier', ...
            options.CaptureDerivedSpeedThresholdMultiplier);
    if motionEvidence.IsNewFeedback && ...
            motionEvidence.ConsecutiveMovingFrames >= ...
            options.MinimumConsecutiveMovingFeedbackFrames
        safeStop(robot);
        error('scopeguide:residualValidation:RobotMovedDuringCapture', ...
            ['Robot moved for %d consecutive unique feedback frames during ' ...
             '%s. Reported joint/TCP speeds: %.4f deg/s, %.4f, %.4f; ' ...
             'derived joint/TCP speeds: %.4f deg/s, %.4f mm/s, ' ...
             '%.4f deg/s.'], ...
            motionEvidence.ConsecutiveMovingFrames, label, ...
            motionEvidence.ReportedJointSpeedDegSec, ...
            motionEvidence.ReportedTcpTranslationSpeed, ...
            motionEvidence.ReportedTcpRotationSpeed, ...
            motionEvidence.DerivedJointSpeedDegSec, ...
            motionEvidence.DerivedTcpTranslationSpeedMmSec, ...
            motionEvidence.DerivedTcpRotationSpeedDegSec);
    end
    compensated = compensateHexHWrench(wrenchSample.wrench, ...
        snapshot.actualQuaternionWxyz, calibration);
    hostSessionTimeSec(index) = sessionTimeOffsetSec + toc(sessionClock);
    sensorSequence(index) = wrenchSample.sequence;
    sensorReadDurationSec(index) = wrenchSample.readDuration;
    robotSequence(index) = snapshot.feedbackSequence;
    robotFeedbackAgeSec(index) = snapshot.feedbackAgeSec;
    raw(index, :) = compensated.rawSensor.';
    externalSensor(index, :) = compensated.externalSensor.';
    externalTool(index, :) = compensated.externalTool.';
    joints(index, :) = snapshot.jointAnglesDeg;
    quaternion(index, :) = snapshot.actualQuaternionWxyz;
    cartesianPose(index, :) = snapshot.cartesianPose;
    stationary(index) = still;
end

data = table(repmat(string(label), count, 1), ...
    repmat(double(poseIndex), count, 1), (1:count).', ...
    hostSessionTimeSec, sensorSequence, sensorReadDurationSec, ...
    robotSequence, robotFeedbackAgeSec, stationary, ...
    'VariableNames', {'PoseLabel','PoseNumber','SampleInPose', ...
    'HostSessionTimeSec','SensorSequence','SensorReadDurationSec', ...
    'RobotSequence','RobotFeedbackAgeSec','Stationary'});
data = addSixColumns(data, raw, 'raw');
data = addSixColumns(data, externalSensor, 'sourceExternalSensor');
data = addSixColumns(data, externalTool, 'sourceExternalTool');
for axis = 1:6
    data.(sprintf('J%dDeg', axis)) = joints(:, axis);
    data.(sprintf('Cartesian%d', axis)) = cartesianPose(:, axis);
end
for axis = 1:4
    data.(sprintf('q%dWxyz', axis)) = quaternion(:, axis);
end

summary = emptyPoseSummary();
summary.Label = string(label);
summary.PoseIndex = poseIndex;
summary.SampleCount = count;
summary.RawMedian = median(raw, 1);
summary.RawMean = mean(raw, 1);
summary.RawStd = std(raw, 0, 1);
summary.SourceExternalSensorMean = mean(externalSensor, 1);
summary.SourceExternalSensorStd = std(externalSensor, 0, 1);
summary.SourceExternalToolMean = mean(externalTool, 1);
summary.SourceExternalToolStd = std(externalTool, 0, 1);
summary.JointMeanDeg = mean(joints, 1);
summary.CartesianPoseMean = mean(cartesianPose, 1);
summary.QuaternionMeanWxyz = averageQuaternion(quaternion);
fprintf('%s source residual=[%s], |F|=%.3f N, |T|=%.4f Nm\n', ...
    label, vectorText(summary.SourceExternalToolMean), ...
    norm(summary.SourceExternalToolMean(1:3)), ...
    norm(summary.SourceExternalToolMean(4:6)));
end

function value = emptyPoseSummary()
value = struct('Label', "", 'PoseIndex', 0, 'SampleCount', 0, ...
    'RawMedian', zeros(1, 6), 'RawMean', zeros(1, 6), ...
    'RawStd', zeros(1, 6), ...
    'SourceExternalSensorMean', zeros(1, 6), ...
    'SourceExternalSensorStd', zeros(1, 6), ...
    'SourceExternalToolMean', zeros(1, 6), ...
    'SourceExternalToolStd', zeros(1, 6), ...
    'JointMeanDeg', zeros(1, 6), ...
    'CartesianPoseMean', zeros(1, 6), ...
    'QuaternionMeanWxyz', zeros(1, 4));
end

function assessment = assessCandidate(comparison, returnChange, options)
assessment = assessHexHBiasOnlyCandidate(comparison, returnChange, ...
    'MaximumValidationForceNormN', ...
        options.MaximumValidationForceNormN, ...
    'MaximumValidationMomentNormNm', ...
        options.MaximumValidationMomentNormNm, ...
    'MaximumValidationForceVectorRmseN', ...
        options.MaximumValidationForceVectorRmseN, ...
    'MaximumValidationMomentVectorRmseNm', ...
        options.MaximumValidationMomentVectorRmseNm, ...
    'MaximumTrainingCenteredForceNormN', ...
        options.MaximumTrainingCenteredForceNormN, ...
    'MaximumTrainingCenteredMomentNormNm', ...
        options.MaximumTrainingCenteredMomentNormNm, ...
    'MaximumReturnForceChangeN', options.MaximumReturnForceChangeN, ...
    'MaximumReturnMomentChangeNm', options.MaximumReturnMomentChangeNm, ...
    'MaximumBiasShiftForceNormN', options.MaximumBiasShiftForceNormN, ...
    'MaximumBiasShiftMomentNormNm', options.MaximumBiasShiftMomentNormNm, ...
    'BiasVsJointRelativeRatio', options.BiasVsJointRelativeRatio, ...
    'BiasVsJointForceMarginN', options.BiasVsJointForceMarginN, ...
    'BiasVsJointMomentMarginNm', options.BiasVsJointMomentMarginNm);
end

function residual = residualForCalibration(raw, quaternion, calibration)
compensated = compensateHexHWrench(raw(:), quaternion, calibration);
residual = compensated.externalSensor.';
end

function output = buildModelComparisonTable(comparison)
names = ["Original";"BiasOnly";"LoadOnly";"Joint"];
count = numel(names);
trainForceRmseN = zeros(count, 1);
trainMomentRmseNm = zeros(count, 1);
validationForceRmseN = zeros(count, 1);
validationMomentRmseNm = zeros(count, 1);
validationMaximumForceN = zeros(count, 1);
validationMaximumMomentNm = zeros(count, 1);
massKg = zeros(count, 1);
comX = zeros(count, 1); comY = zeros(count, 1); comZ = zeros(count, 1);
for index = 1:count
    model = comparison.Models.(char(names(index)));
    trainForceRmseN(index) = model.Training.ForceVectorRmseN;
    trainMomentRmseNm(index) = model.Training.MomentVectorRmseNm;
    validationForceRmseN(index) = model.Validation.ForceVectorRmseN;
    validationMomentRmseNm(index) = model.Validation.MomentVectorRmseNm;
    validationMaximumForceN(index) = ...
        model.Validation.MaximumForceNormN;
    validationMaximumMomentNm(index) = ...
        model.Validation.MaximumMomentNormNm;
    massKg(index) = model.Calibration.massKg;
    center = model.Calibration.comSensorM(:);
    comX(index) = center(1); comY(index) = center(2); comZ(index) = center(3);
end
output = table(names, trainForceRmseN, trainMomentRmseNm, ...
    validationForceRmseN, validationMomentRmseNm, ...
    validationMaximumForceN, validationMaximumMomentNm, massKg, ...
    comX, comY, comZ);
end

function output = buildPoseSummaryTable(summaries, plan, comparison, returnResidual)
count = numel(summaries);
poseIndex = (1:count).';
poseLabel = string({summaries.Label}).';
role = repmat("return", count, 1);
role(plan.TrainingIndices) = "training";
role(plan.ValidationIndices) = "validation";
raw = vertcat(summaries.RawMedian);
sourceResidual = zeros(count, 6);
biasResidual = zeros(count, 6);
sourceResidual(1:plan.UniquePoseCount, :) = ...
    comparison.Models.Original.ResidualSensor;
biasResidual(1:plan.UniquePoseCount, :) = ...
    comparison.Models.BiasOnly.ResidualSensor;
sourceResidual(end, :) = residualForCalibration( ...
    summaries(end).RawMedian, summaries(end).QuaternionMeanWxyz, ...
    comparison.Models.Original.Calibration);
biasResidual(end, :) = returnResidual;
output = table(poseIndex, poseLabel, role);
output = addSixColumns(output, raw, 'rawMedian');
output = addSixColumns(output, sourceResidual, 'sourceResidualSensor');
output = addSixColumns(output, biasResidual, 'biasResidualSensor');
end

function [jsonPath, matPath] = writeCandidate(directory, calibration, ...
        sourceFile, provenance, plan, assessment, comparison)
timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
jsonPath = fullfile(directory, ...
    "candidate_bias_only_calibration_" + timestamp + ".json");
matPath = fullfile(directory, ...
    "candidate_bias_only_calibration_" + timestamp + ".mat");
payload = struct();
payload.status = 'VALIDATED_CANDIDATE_NOT_ACTIVATED';
payload.activationAuthorized = false;
payload.createdLocal = char(datetime('now'));
payload.sourceCalibrationFile = char(sourceFile);
payload.sourceCalibrationProvenance = provenance;
payload.trainingIndices = plan.TrainingIndices;
payload.validationIndices = plan.ValidationIndices;
payload.calibration = calibration;
payload.assessment = assessment;
payload.biasCorrectionSensor = comparison.BiasCorrectionSensor;
payload.notes = { ...
    'This file was validated on held-out poses but is not activated.', ...
    'The source calibration and default configuration were not modified.', ...
    'Review physical preload, cable routing, and residual plots before use.'};
writeJson(jsonPath, payload);
save(matPath, 'payload');
end

function saveResidualFigure(comparison, plan, figurePath)
figureHandle = figure('Visible', 'off', 'Color', 'w', ...
    'Name', 'HEX-H residual model validation');
cleanup = onCleanup(@() close(figureHandle));
models = {'Original','BiasOnly','LoadOnly','Joint'};
colors = lines(numel(models));
for panel = 1:2
    subplot(2, 1, panel); hold on; grid on;
    for modelIndex = 1:numel(models)
        residual = comparison.Models.(models{modelIndex}).ResidualSensor;
        if panel == 1
            values = vecnorm(residual(:, 1:3), 2, 2);
        else
            values = vecnorm(residual(:, 4:6), 2, 2);
        end
        plot(1:numel(values), values, '-o', ...
            'Color', colors(modelIndex, :), ...
            'DisplayName', models{modelIndex});
    end
    xline(plan.ValidationIndices, ':k', 'HandleVisibility', 'off');
    xlabel('Pose index');
    if panel == 1
        ylabel('|F residual| [N]');
        title('Training and held-out force residuals');
    else
        ylabel('|T residual| [N m]');
        title('Training and held-out moment residuals');
    end
    legend('Location', 'best');
end
exportgraphics(figureHandle, figurePath, 'Resolution', 150);
end

function printFinalResult(result, modelTable, outputDirectory)
fprintf('\nModel comparison (vector-norm RMSE on held-out poses):\n');
disp(modelTable(:, {'names','validationForceRmseN', ...
    'validationMomentRmseNm','validationMaximumForceN', ...
    'validationMaximumMomentNm'}));
fprintf('Decision: %s\n', result.Assessment.Status);
if result.Assessment.Passed
    fprintf('Candidate written but NOT activated: %s\n', ...
        result.CandidateCalibrationJson);
else
    fprintf('No candidate calibration was generated. Failed gates: %s\n', ...
        strjoin(result.Assessment.FailedGates, ', '));
end
fprintf('Recommended action: %s\n', ...
    result.Assessment.RecommendedAction);
fprintf('Results: %s\n', outputDirectory);
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
        error('scopeguide:residualValidation:FeedbackTimeout', ...
            'No robot feedback arrived in %.1f s.', timeoutSec);
    end
    pause(0.02);
end
assertFiniteQuaternion(robot.GetStateSnapshot().actualQuaternionWxyz);
end

function requireEnabledAndIdle(robot)
mode = string(robot.RobotMode);
if mode ~= "ENABLE"
    error('scopeguide:residualValidation:RobotNotEnabledAndIdle', ...
        'RobotMode must be ENABLE. Current mode: %s.', mode);
end
end

function requireMotionQueueReady(robot)
snapshot = robot.GetStateSnapshot();
if snapshot.pauseCommandFlag || ~snapshot.runQueuedCommand
    error('scopeguide:residualValidation:MotionQueueNotRunning', ...
        ['The V3 TCP motion queue is not running: RunQueuedCmd=%d, ' ...
         'PauseCmdFlag=%d. Do NOT send Continue() blindly because an ' ...
         'accepted target may still be queued. With the swept volume ' ...
         'clear, clear the old queue using ResetRobot from the controller/' ...
         'DobotStudio, then manually re-enable the robot and rerun.'], ...
        snapshot.runQueuedCommand, snapshot.pauseCommandFlag);
end
end

function requireSensorHealthy(sensor)
status = double(sensor.readStatus());
if status ~= 0
    error('scopeguide:residualValidation:SensorStatus', ...
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
        error('scopeguide:residualValidation:StationaryTimeout', ...
            'Robot did not become stationary in %.1f s.', ...
            options.MoveTimeoutSec);
    end
    pause(0.02);
end
end

function stationary = isStationary(snapshot, options)
stationary = max(abs(snapshot.actualJointSpeedsDegSec)) <= ...
        options.MaximumStationaryJointSpeedDegSec && ...
    max(abs(snapshot.actualTCPSpeed(1:3))) <= ...
        options.MaximumStationaryTcpTranslationSpeed && ...
    max(abs(snapshot.actualTCPSpeed(4:6))) <= ...
        options.MaximumStationaryTcpRotationSpeed;
end

function assertFreshFeedback(snapshot, options)
if ~isfinite(snapshot.feedbackAgeSec) || ...
        snapshot.feedbackAgeSec > options.MaximumFeedbackAgeSec
    error('scopeguide:residualValidation:StaleFeedback', ...
        'Robot feedback age %.3f s exceeds %.3f s.', ...
        snapshot.feedbackAgeSec, options.MaximumFeedbackAgeSec);
end
assertFiniteQuaternion(snapshot.actualQuaternionWxyz);
end

function assertFiniteQuaternion(quaternion)
if any(~isfinite(quaternion)) || abs(norm(quaternion) - 1) > 1e-3
    error('scopeguide:residualValidation:InvalidQuaternion', ...
        'Robot ActualQuaternion is invalid.');
end
end

function requireConfirmation(options, expected, prompt)
if ~options.RequireTypedConfirmation
    return;
end
while true
    response = strtrim(string(input(prompt, 's')));
    if response == string(expected)
        return;
    end
    if strcmpi(response, "CANCEL")
        error('scopeguide:residualValidation:OperatorCancelled', ...
            'Operator entered CANCEL while waiting for "%s".', expected);
    end
    fprintf(2, ['Input did not match "%s". No command was sent. ' ...
        'Try again, or type "CANCEL" to stop safely.\n'], expected);
end
end

function saveCheckpoint(data, poseSummaries, result, paths)
if ~isempty(data)
    writetable(data, paths.SignalsCsv);
end
save(paths.Mat, 'data', 'poseSummaries', 'result');
writeJson(paths.SummaryJson, jsonSafeResult(result));
end

function value = jsonSafeResult(result)
value = result;
if isfield(value, 'ModelComparison')
    value.ModelComparison = rmfield(value.ModelComparison, 'Models');
end
if isfield(value, 'ModelComparisonTable')
    value = rmfield(value, 'ModelComparisonTable');
end
end

function directory = createOutputDirectory(projectRoot)
timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
directory = fullfile(projectRoot, 'results', ...
    "force_residual_model_validation_" + timestamp);
[created, message] = mkdir(directory);
if ~created && ~isfolder(directory)
    error('scopeguide:residualValidation:OutputDirectory', ...
        'Could not create %s: %s', directory, message);
end
end

function writeJson(path, payload)
try
    contents = jsonencode(payload, 'PrettyPrint', true);
catch
    contents = jsonencode(payload);
end
file = fopen(path, 'w');
if file < 0
    error('scopeguide:residualValidation:JsonWriteFailed', ...
        'Could not open %s.', path);
end
guard = onCleanup(@() fclose(file));
fwrite(file, contents, 'char');
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

function errorDeg = orientationErrorDeg(targetRpyDeg, quaternionWxyz)
targetRotation = scopeguide.geometry.dobotRpyToRotation( ...
    targetRpyDeg, pi / 180);
actualRotation = scopeguide.geometry.rotationMatrixFromQuaternionWxyz( ...
    quaternionWxyz);
errorDeg = rad2deg(scopeguide.geometry.rotationDistance( ...
    targetRotation, actualRotation));
end

function reached = targetPoseReached(snapshot, targetPose, options)
positionErrorMm = norm(double(snapshot.cartesianPose(1:3)) - ...
    double(targetPose(1:3)));
rotationErrorDeg = orientationErrorDeg( ...
    targetPose(4:6), snapshot.actualQuaternionWxyz);
reached = positionErrorMm <= options.CartesianPositionToleranceMm && ...
    rotationErrorDeg <= options.CartesianOrientationToleranceDeg;
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
