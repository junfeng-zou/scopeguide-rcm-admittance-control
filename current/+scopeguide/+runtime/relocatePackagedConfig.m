function cfg = relocatePackagedConfig(cfg, projectRoot)
%RELOCATEPACKAGEDCONFIG Resolve bundled runtime resources after cloning.
% Hardware parameters, tuning and acceptance records remain unchanged.

arguments
    cfg (1, 1) struct
    projectRoot (1, 1) string
end

cfg.meta.ProjectRoot = projectRoot;
cfg.meta.UserAttestationRecord = fullfile(projectRoot, 'config', ...
    'stage08_user_attestation_20260814.json');
cfg.logging.ResultsRoot = fullfile(projectRoot, 'results');
calibrationFile = fullfile(projectRoot, 'force_calibration', 'data', ...
    'hex_h_tool_calibration_20260812_154348', ...
    'calibration_without_pose01_pose07_pose10.json');
if ~isfile(calibrationFile)
    error('scopeguide:config:PackagedCalibrationMissing', ...
        'Packaged force calibration was not found: %s', calibrationFile);
end
cfg.force.CalibrationFile = calibrationFile;
end
