function [validation, outputDirectory] = ...
        run_stage02_recorded_kinematics_validation(options)
%RUN_STAGE02_RECORDED_KINEMATICS_VALIDATION Offline multi-pose CR5 check.

arguments
    options.Config = struct([])
    options.SourceFile = ""
    options.WriteResults (1, 1) logical = true
end

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'force_calibration'));
if isempty(options.Config)
    cfg = defaultRcmAdmittanceConfig();
else
    cfg = options.Config;
end
sourceOption = string(options.SourceFile);
if strlength(sourceOption) == 0
    sourceFile = fullfile(projectRoot, 'force_calibration', 'data', ...
        'hex_h_tool_calibration_20260812_154348', 'raw_samples.csv');
else
    sourceFile = sourceOption;
end

validation = scopeguide.geometry.replayRecordedRobotKinematics( ...
    cfg, sourceFile);
if options.WriteResults
    outputDirectory = ...
        scopeguide.geometry.writeStage02KinematicsReport( ...
        projectRoot, cfg, validation);
else
    outputDirectory = "";
end
end
