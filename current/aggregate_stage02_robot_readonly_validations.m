function [summary, outputDirectory] = ...
        aggregate_stage02_robot_readonly_validations(resultDirectories, options)
%AGGREGATE_STAGE02_ROBOT_READONLY_VALIDATIONS Combine static-pose runs.
% Pass at least three stage02_live_readonly_* result directories collected
% at manually selected, mechanically safe and distinct robot poses.

arguments
    resultDirectories (:, 1) string
    options.WriteResults (1, 1) logical = true
end

if numel(resultDirectories) < 3
    error('scopeguide:stage02:AtLeastThreePosesRequired', ...
        'At least three live read-only pose result directories are required.');
end
projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
flangePositionErrorM = zeros(0, 1);
flangeOrientationErrorRad = zeros(0, 1);
endoscopePositionErrorM = zeros(0, 1);
endoscopeOrientationErrorRad = zeros(0, 1);
rpyQuaternionMismatchRad = zeros(0, 1);
meanJointPositionRad = nan(numel(resultDirectories), 6);
poseLabels = strings(numel(resultDirectories), 1);
motionCommandSent = false(numel(resultDirectories), 1);
commandSentCount = nan(numel(resultDirectories), 1);
feedbackPeriodP99Sec = nan(numel(resultDirectories), 1);
feedbackAgeP99Sec = nan(numel(resultDirectories), 1);
validReadRatio = nan(numel(resultDirectories), 1);

for index = 1:numel(resultDirectories)
    directory = resultDirectories(index);
    summaryFile = fullfile(directory, 'summary.json');
    signalsFile = fullfile(directory, 'signals.csv');
    if ~isfile(summaryFile) || ~isfile(signalsFile)
        error('scopeguide:stage02:IncompleteLiveResult', ...
            'Directory %s must contain summary.json and signals.csv.', ...
            directory);
    end
    runSummary = jsondecode(fileread(summaryFile));
    signals = readtable(signalsFile, 'VariableNamingRule', 'preserve');
    required = ["J1Rad", "J2Rad", "J3Rad", "J4Rad", "J5Rad", ...
        "J6Rad", "FlangePositionErrorM", ...
        "FlangeOrientationErrorRad", "EndoscopePositionErrorM", ...
        "EndoscopeOrientationErrorRad", "RpyQuaternionMismatchRad"];
    missing = setdiff(required, string(signals.Properties.VariableNames));
    if ~isempty(missing)
        error('scopeguide:stage02:IncompleteLiveSignals', ...
            'Directory %s is missing columns: %s.', directory, ...
            strjoin(missing, ', '));
    end
    poseLabels(index) = string(runSummary.PoseLabel);
    motionCommandSent(index) = logical(runSummary.PhysicalMotionCommandSent);
    commandSentCount(index) = double(runSummary.AdapterCommandSentCount);
    feedbackPeriodP99Sec(index) = double(runSummary.FeedbackPeriodP99Sec);
    feedbackAgeP99Sec(index) = double(runSummary.FeedbackAgeP99Sec);
    validReadRatio(index) = double(runSummary.ValidReadRatio);
    jointColumnNames = cellstr(compose("J%dRad", 1:6));
    meanJointPositionRad(index, :) = mean( ...
        signals{:, jointColumnNames}, 1, 'omitnan');
    flangePositionErrorM = [flangePositionErrorM; ...
        signals.FlangePositionErrorM]; %#ok<AGROW>
    flangeOrientationErrorRad = [flangeOrientationErrorRad; ...
        signals.FlangeOrientationErrorRad]; %#ok<AGROW>
    endoscopePositionErrorM = [endoscopePositionErrorM; ...
        signals.EndoscopePositionErrorM]; %#ok<AGROW>
    endoscopeOrientationErrorRad = [endoscopeOrientationErrorRad; ...
        signals.EndoscopeOrientationErrorRad]; %#ok<AGROW>
    rpyQuaternionMismatchRad = [rpyQuaternionMismatchRad; ...
        signals.RpyQuaternionMismatchRad]; %#ok<AGROW>
end

cfg = defaultRcmAdmittanceConfig();
flangeRmseM = rootMeanSquare(flangePositionErrorM);
endoscopeRmseM = rootMeanSquare(endoscopePositionErrorM);
pairwiseJointSeparationRad = pairwiseDistances(meanJointPositionRad);
summary = struct();
summary.Stage = 2;
summary.Status = "multi_pose_readonly_aggregate_complete";
summary.ResultDirectories = resultDirectories.';
summary.PoseLabels = poseLabels.';
summary.PoseRunCount = numel(resultDirectories);
summary.AllPoseLabelsUnique = numel(unique(poseLabels)) == numel(poseLabels);
summary.MinimumMeanJointPoseSeparationRad = ...
    min(pairwiseJointSeparationRad);
summary.AllPhysicalMotionCommandSentFalse = ~any(motionCommandSent);
summary.TotalAdapterCommandSentCount = sum(commandSentCount);
summary.MinimumValidReadRatio = min(validReadRatio);
summary.MaximumFeedbackPeriodP99Sec = max(feedbackPeriodP99Sec);
summary.MaximumFeedbackAgeP99Sec = max(feedbackAgeP99Sec);
summary.SuggestedFeedbackStaleSec = max( ...
    3 * summary.MaximumFeedbackPeriodP99Sec, ...
    summary.MaximumFeedbackAgeP99Sec + ...
    summary.MaximumFeedbackPeriodP99Sec);
summary.FlangePositionRmseM = flangeRmseM;
summary.FlangePositionMaximumM = max(flangePositionErrorM);
summary.FlangeOrientationRmseRad = ...
    rootMeanSquare(flangeOrientationErrorRad);
summary.FlangeOrientationMaximumRad = max(flangeOrientationErrorRad);
summary.EndoscopePositionRmseM = endoscopeRmseM;
summary.EndoscopePositionMaximumM = max(endoscopePositionErrorM);
summary.EndoscopeOrientationRmseRad = ...
    rootMeanSquare(endoscopeOrientationErrorRad);
summary.EndoscopeOrientationMaximumRad = ...
    max(endoscopeOrientationErrorRad);
summary.RpyQuaternionMismatchRmseRad = ...
    rootMeanSquare(rpyQuaternionMismatchRad);
summary.RpyQuaternionMismatchMaximumRad = ...
    max(rpyQuaternionMismatchRad);
if flangeRmseM < endoscopeRmseM
    summary.InferredControllerPoseReference = "flange";
else
    summary.InferredControllerPoseReference = "endoscope";
end
summary.FlangeErrorToSoftRcmRatio = ...
    summary.FlangePositionMaximumM / cfg.rcm.SoftRadiusM;
summary.FlangeErrorToHardRcmRatio = ...
    summary.FlangePositionMaximumM / cfg.rcm.HardRadiusM;
summary.NominalModelSupportsCurrentHardRcmCandidate = ...
    summary.FlangePositionMaximumM < cfg.rcm.HardRadiusM;
summary.ServoTimingMeasured = false;
summary.ModelAutomaticallyMarkedVerified = false;

if options.WriteResults
    timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
    outputDirectory = fullfile(projectRoot, 'results', ...
        "stage02_live_readonly_aggregate_" + timestamp);
    [created, message] = mkdir(outputDirectory);
    if ~created && ~isfolder(outputDirectory)
        error('scopeguide:stage02:CannotCreateResultDirectory', ...
            'Cannot create %s: %s', outputDirectory, message);
    end
    writeJson(fullfile(outputDirectory, 'summary.json'), summary);
    poseTable = table(poseLabels, meanJointPositionRad, validReadRatio, ...
        feedbackPeriodP99Sec, feedbackAgeP99Sec, ...
        'VariableNames', {'PoseLabel', 'MeanJointPositionRad', ...
        'ValidReadRatio', 'FeedbackPeriodP99Sec', 'FeedbackAgeP99Sec'});
    writetable(poseTable, fullfile(outputDirectory, 'included_runs.csv'));
else
    outputDirectory = "";
end
end

function distances = pairwiseDistances(rows)
count = size(rows, 1);
distances = nan(count * (count - 1) / 2, 1);
outputIndex = 0;
for first = 1:count-1
    for second = first+1:count
        outputIndex = outputIndex + 1;
        distances(outputIndex) = norm(rows(first, :) - rows(second, :));
    end
end
end

function value = rootMeanSquare(values)
value = sqrt(mean(values.^2, 'omitnan'));
end

function writeJson(filePath, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end
[fileId, message] = fopen(filePath, 'w');
if fileId < 0
    error('scopeguide:stage02:CannotWriteJson', ...
        'Cannot open %s: %s', filePath, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end
