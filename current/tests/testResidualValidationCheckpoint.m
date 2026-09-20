function tests = testResidualValidationCheckpoint
tests = functiontests(localfunctions);
end

function testSevenCompletedPosesResumeAtEight(testCase)
directory = string(tempname);
mkdir(directory);
cleanup = onCleanup(@() rmdir(directory, 's'));
[data, poseSummaries, result] = syntheticCheckpoint();
save(fullfile(directory, 'diagnostic.mat'), ...
    'data', 'poseSummaries', 'result');
current = result.Options;
current.ResumeDirectory = directory;
current.CaptureDerivedSpeedThresholdMultiplier = 1.5;
checkpoint = ...
    scopeguide.diagnostics.loadResidualValidationCheckpoint( ...
    string(tempdir), directory, current);
verifyEqual(testCase, checkpoint.NextPoseIndex, 8);
verifyEqual(testCase, checkpoint.Plan.Labels(8), "P08_train");
verifyEqual(testCase, height(checkpoint.Data), 28);
verifyEqual(testCase, checkpoint.Result.Status, "resuming");
verifyEqual(testCase, checkpoint.Result.ResumeCount, 1);
clear cleanup
end

function testCompletedCheckpointCannotResume(testCase)
directory = string(tempname);
mkdir(directory);
cleanup = onCleanup(@() rmdir(directory, 's'));
[data, poseSummaries, result] = syntheticCheckpoint();
result.Completed = true;
save(fullfile(directory, 'diagnostic.mat'), ...
    'data', 'poseSummaries', 'result');
current = result.Options;
verifyError(testCase, @() ...
    scopeguide.diagnostics.loadResidualValidationCheckpoint( ...
    string(tempdir), directory, current), ...
    'scopeguide:residualValidation:ResumeCheckpointNotPartial');
clear cleanup
end

function [data, poseSummaries, result] = syntheticCheckpoint()
labels = ["P01_train";"P02_train";"P03_train";"P04_validation"; ...
    "P05_train";"P06_train";"P07_validation";"P08_train"; ...
    "P09_validation";"A_return"];
poseSummaries = repmat(struct('Label', ""), 7, 1);
for index = 1:7
    poseSummaries(index).Label = labels(index);
end
data = table((1:28).', 'VariableNames', {'HostSessionTimeSec'});
result = struct();
result.Completed = false;
result.Status = "failed";
result.SourceCalibrationFile = "/tmp/source.json";
result.CalibrationProvenance = struct();
result.MotionPlan = struct('Labels', labels);
result.Options = struct('EnableMotion', true, ...
    'RequireTypedConfirmation', true, ...
    'SampleDurationSec', 2, 'SampleRateHz', 2);
end
