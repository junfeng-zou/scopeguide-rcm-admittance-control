function [review, outputDirectory] = ...
        run_stage08_final_review(resultDirectories, options)
%RUN_STAGE08_FINAL_REVIEW Aggregate accepted Stage 8 fixture phases.
% Read-only review: no robot/sensor connection and no motion command.

arguments
    resultDirectories (:, 1) string
    options.WriteResults (1, 1) logical = true
end

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
requiredModes = ["insertion_only"; "pivot1_only"; "pivot2_only"; ...
    "dual_pivot"; "full_3dof"];
if numel(resultDirectories) ~= numel(requiredModes)
    error('scopeguide:stage08:IncompleteFinalReview', ...
        'Provide exactly five accepted result directories in phase order.');
end
phase = repmat(struct(), numel(requiredModes), 1);
allPassed = true;
for index = 1:numel(requiredModes)
    resultPath = fullfile(resultDirectories(index), ...
        'stage08_worker_result.mat');
    acceptedPath = fullfile(resultDirectories(index), ...
        'stage08_accepted_config.mat');
    if ~isfile(resultPath) || ~isfile(acceptedPath)
        error('scopeguide:stage08:ReviewFileMissing', ...
            'Stage 8 result or acceptance file is missing in %s.', ...
            resultDirectories(index));
    end
    worker = load(resultPath, 'summary');
    accepted = load(acceptedPath, 'acceptance', 'cfgAccepted');
    phase(index).Directory = resultDirectories(index);
    phase(index).ExpectedMode = requiredModes(index);
    phase(index).ObservedMode = string(worker.summary.DofMode);
    phase(index).ModeCorrect = ...
        phase(index).ObservedMode == requiredModes(index);
    phase(index).Accepted = accepted.acceptance.Accepted;
    phase(index).NoCommandFailure = ...
        worker.summary.RobotCommandFailureCount == 0;
    phase(index).ProgrammaticEnableConfirmed = ...
        worker.summary.ProgrammaticEnableConfirmed;
    phase(index).SpeedFactorConfirmed = ...
        worker.summary.SpeedFactorConfirmed;
    phase(index).ProgrammaticDisableConfirmed = ...
        worker.summary.ProgrammaticDisableConfirmed;
    phase(index).StopLatencyWithinLimit = ...
        worker.summary.StopLatencyWithinLimit;
    phase(index).TrackingWithinLimit = ...
        worker.summary.TargetFeedbackErrorRad.Maximum <= ...
        accepted.cfgAccepted.stage08.MaximumTargetFeedbackErrorRad;
    phase(index).ModelRcmWithinHardBound = ...
        worker.summary.RcmErrorM.Maximum <= ...
        accepted.cfgAccepted.rcm.HardRadiusM;
    phase(index).PhysicalRcmMaximumMm = ...
        accepted.acceptance.PhysicalRcmMaximumMm;
    fields = {'ModeCorrect', 'Accepted', 'NoCommandFailure', ...
        'ProgrammaticEnableConfirmed', 'SpeedFactorConfirmed', ...
        'ProgrammaticDisableConfirmed', ...
        'StopLatencyWithinLimit', 'TrackingWithinLimit', ...
        'ModelRcmWithinHardBound'};
    phasePassed = all(cellfun(@(name) phase(index).(name), fields));
    phase(index).Passed = phasePassed;
    allPassed = allPassed && phasePassed;
end
finalAccepted = load(fullfile(resultDirectories(end), ...
    'stage08_accepted_config.mat'), 'cfgAccepted');
completed = string(finalAccepted.cfgAccepted.stage08.CompletedDofModes(:));
allModesRecorded = all(ismember(requiredModes, completed));
allPassed = allPassed && allModesRecorded && ...
    finalAccepted.cfgAccepted.robot.ServoTimingVerified;
review = struct();
review.Stage = 8;
review.Status = "BLOCKED";
if allPassed
    review.Status = "PASSED_FIXTURE";
end
review.Passed = allPassed;
review.RequiredModes = requiredModes;
review.CompletedModes = completed;
review.AllModesRecorded = allModesRecorded;
review.ServoTimingVerified = ...
    finalAccepted.cfgAccepted.robot.ServoTimingVerified;
review.Phases = phase;
review.HardwareConnectionsCreated = 0;
review.MotionCommandsSent = 0;
review.KeyboardInputWasNotPhysicalDeadman = true;
review.Stage09StillRequiresPhantomSpecificSafetyReview = true;

if options.WriteResults
    timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
    outputDirectory = fullfile(projectRoot, 'results', ...
        "stage08_final_review_" + timestamp);
    [created, message] = mkdir(outputDirectory);
    if ~created && ~isfolder(outputDirectory)
        error('scopeguide:stage08:CannotCreateReviewDirectory', ...
            'Cannot create %s: %s', outputDirectory, message);
    end
    writeJson(fullfile(outputDirectory, 'review.json'), review);
    cfgStage08Accepted = finalAccepted.cfgAccepted;
    save(fullfile(outputDirectory, 'stage08_accepted_config.mat'), ...
        'cfgStage08Accepted', 'review');
else
    outputDirectory = "";
end
fprintf('Stage 8 final review: %s (%d/%d phases passed).\n', ...
    review.Status, sum([phase.Passed]), numel(phase));
if options.WriteResults
    fprintf('Results: %s\n', outputDirectory);
end
end

function writeJson(path, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end
[fileId, message] = fopen(path, 'w');
if fileId < 0
    error('scopeguide:stage08:CannotWriteJson', ...
        'Cannot open %s: %s', path, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end
