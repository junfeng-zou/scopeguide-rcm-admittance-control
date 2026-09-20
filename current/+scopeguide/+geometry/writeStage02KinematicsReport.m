function outputDirectory = writeStage02KinematicsReport( ...
    projectRoot, cfg, validation)
%WRITESTAGE02KINEMATICSREPORT Save historical Stage 2 validation results.

timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
outputDirectory = fullfile(projectRoot, 'results', ...
    "stage02_recorded_kinematics_" + timestamp);
[created, message] = mkdir(outputDirectory);
if ~created && ~isfolder(outputDirectory)
    error('scopeguide:kinematics:CannotCreateResultDirectory', ...
        'Cannot create %s: %s', outputDirectory, message);
end

summary = validation.Summary;
summary.Stage = 2;
summary.Status = "historical_readonly_validation_complete";
summary.PhysicalMotionCommandSent = false;
summary.ModelAutomaticallyMarkedVerified = false;
writeJson(fullfile(outputDirectory, 'summary.json'), summary);
writeJson(fullfile(outputDirectory, 'config_snapshot.json'), cfg);
save(fullfile(outputDirectory, 'validation.mat'), 'validation', '-v7.3');

signals = table(validation.PoseIndex, ...
    double(validation.FeedbackSequence), validation.FeedbackTimeSec, ...
    validation.FlangePositionErrorM, ...
    validation.FlangeOrientationErrorRad, ...
    validation.EndoscopePositionErrorM, ...
    validation.EndoscopeOrientationErrorRad, ...
    validation.RpyQuaternionMismatchRad, ...
    'VariableNames', {'PoseIndex', 'FeedbackSequence', ...
    'FeedbackTimeSec', 'FlangePositionErrorM', ...
    'FlangeOrientationErrorRad', 'EndoscopePositionErrorM', ...
    'EndoscopeOrientationErrorRad', 'RpyQuaternionMismatchRad'});
writetable(signals, fullfile(outputDirectory, 'validation.csv'));
end

function writeJson(filePath, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end
[fileId, message] = fopen(filePath, 'w');
if fileId < 0
    error('scopeguide:kinematics:CannotWriteJson', ...
        'Cannot open %s: %s', filePath, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end
