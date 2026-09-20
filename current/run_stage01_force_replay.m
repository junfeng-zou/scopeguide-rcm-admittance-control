function [replay, outputDirectory] = run_stage01_force_replay(options)
%RUN_STAGE01_FORCE_REPLAY Run full pose-aware offline force replay.
% No robot or sensor connection is created by this function.

arguments
    options.Config = struct([])
    options.SourceFile = ""
    options.MaximumSamples (1, 1) double = inf
    options.WriteResults (1, 1) logical = true
end

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'force_sensor'));
addpath(fullfile(projectRoot, 'force_calibration'));

if isempty(options.Config)
    cfg = defaultRcmAdmittanceConfig();
else
    cfg = options.Config;
end
validateRcmAdmittanceConfig(cfg);
sourceOption = string(options.SourceFile);
if ~isscalar(sourceOption)
    error('scopeguide:force:InvalidReplaySource', ...
        'SourceFile must be a scalar path.');
end
if strlength(sourceOption) == 0
    sourceFile = fullfile(projectRoot, 'force_calibration', 'data', ...
        'hex_h_tool_calibration_20260807_164630', 'raw_samples.csv');
else
    sourceFile = sourceOption;
end

replay = scopeguide.force.replayCalibrationDataset( ...
    cfg, sourceFile, MaximumSamples=options.MaximumSamples, ...
    RetainedPosesOnly=true);
if options.WriteResults
    outputDirectory = scopeguide.force.writeStage01ReplayReport( ...
        projectRoot, cfg, replay);
else
    outputDirectory = "";
end
end
