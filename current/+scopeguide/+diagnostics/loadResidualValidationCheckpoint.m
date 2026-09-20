function checkpoint = loadResidualValidationCheckpoint( ...
        projectRoot, requestedDirectory, currentOptions)
%LOADRESIDUALVALIDATIONCHECKPOINT Validate one partial experiment.
% This function is offline only and does not communicate with hardware.

arguments
    projectRoot (1, 1) string
    requestedDirectory (1, 1) string
    currentOptions (1, 1) struct
end

directory = requestedDirectory;
if ~isfolder(directory)
    candidate = fullfile(projectRoot, directory);
    if isfolder(candidate)
        directory = string(candidate);
    else
        error('scopeguide:residualValidation:ResumeDirectoryMissing', ...
            'ResumeDirectory does not exist: %s', requestedDirectory);
    end
end
matFile = fullfile(directory, 'diagnostic.mat');
if ~isfile(matFile)
    error('scopeguide:residualValidation:ResumeCheckpointMissing', ...
        'Resume checkpoint is missing: %s', matFile);
end
loaded = load(matFile, 'data', 'poseSummaries', 'result');
if ~isfield(loaded, 'data') || ~isfield(loaded, 'poseSummaries') || ...
        ~isfield(loaded, 'result') || ...
        ~isfield(loaded.result, 'MotionPlan') || ...
        ~isfield(loaded.result, 'Options') || ...
        ~isfield(loaded.result, 'SourceCalibrationFile')
    error('scopeguide:residualValidation:InvalidResumeCheckpoint', ...
        'The checkpoint does not contain the required plan and data.');
end

data = loaded.data;
poseSummaries = loaded.poseSummaries;
result = loaded.result;
plan = result.MotionPlan;
completedCount = numel(poseSummaries);
expectedSamplesPerPose = round( ...
    result.Options.SampleDurationSec * result.Options.SampleRateHz);
if result.Completed || completedCount < 1 || ...
        completedCount >= numel(plan.Labels) || ...
        height(data) ~= completedCount * expectedSamplesPerPose
    error('scopeguide:residualValidation:ResumeCheckpointNotPartial', ...
        'Resume requires a consistent, incomplete checkpoint.');
end
for index = 1:completedCount
    if string(poseSummaries(index).Label) ~= string(plan.Labels(index))
        error('scopeguide:residualValidation:ResumePoseMismatch', ...
            'Saved pose %d does not match the fixed motion plan.', index);
    end
end

options = currentOptions;
preserveFromCurrent = {'EnableMotion','RequireTypedConfirmation', ...
    'ResumeDirectory','CaptureDerivedSpeedThresholdMultiplier'};
previousOptions = result.Options;
names = fieldnames(previousOptions);
for index = 1:numel(names)
    name = names{index};
    if isfield(options, name) && ~ismember(name, preserveFromCurrent)
        options.(name) = previousOptions.(name);
    end
end

result.Status = "resuming";
result.Completed = false;
result.Options = options;
result.ResumeCount = fieldOrDefault(result, 'ResumeCount', 0) + 1;
result.LastResumeLocal = string(datetime('now'));
for name = {'ErrorIdentifier','ErrorMessage','StopMoveError'}
    if isfield(result, name{1})
        result = rmfield(result, name{1});
    end
end

checkpoint = struct();
checkpoint.Directory = directory;
checkpoint.Data = data;
checkpoint.PoseSummaries = poseSummaries;
checkpoint.Result = result;
checkpoint.Plan = plan;
checkpoint.Options = options;
checkpoint.SourceCalibrationFile = string(result.SourceCalibrationFile);
checkpoint.NextPoseIndex = completedCount + 1;
end

function value = fieldOrDefault(input, name, defaultValue)
if isfield(input, name)
    value = input.(name);
else
    value = defaultValue;
end
end
