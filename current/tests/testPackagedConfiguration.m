function tests = testPackagedConfiguration
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = string(fileparts(fileparts(mfilename('fullpath'))));
addpath(root, fullfile(root, 'config'), fullfile(root, 'force_calibration'));
testCase.TestData.Root = root;
end

function testDefaultProfilesResolveBundledResources(testCase)
root = testCase.TestData.Root;
profiles = {@current_stage08_full_3dof_config, ...
    @current_cartesian_admittance_drag_config, ...
    @current_stage08_dual_pivot_config};
for index = 1:numel(profiles)
    [cfg, profile] = profiles{index}();
    verifyEqual(testCase, cfg.meta.ProjectRoot, root);
    verifyEqual(testCase, cfg.logging.ResultsRoot, fullfile(root, 'results'));
    verifyTrue(testCase, isfile(cfg.meta.UserAttestationRecord));
    verifyTrue(testCase, startsWith(profile.SourceAcceptedConfigFile, ...
        fullfile(root, 'resources')));
    verifyTrue(testCase, isfile(profile.SourceAcceptedConfigFile));
    verifyTrue(testCase, startsWith(cfg.force.CalibrationFile, root));
    [calibration, ~] = loadHexHGravityCalibration(cfg.force.CalibrationFile);
    verifyTrue(testCase, calibration.valid);
end
end

function testExternalConfigKeepsItsOwnCalibration(testCase)
root = testCase.TestData.Root;
directory = string(tempname);
mkdir(directory);
cleanup = onCleanup(@() rmdir(directory, 's')); %#ok<NASGU>
loaded = load(fullfile(root, 'resources', 'validated_base_config.mat'));
cfgAccepted = loaded.cfgAccepted;
cfgAccepted.force.CalibrationFile = fullfile(directory, 'external.json');
file = fullfile(directory, 'accepted.mat');
save(file, 'cfgAccepted');
cfg = current_cartesian_admittance_drag_config(SourceAcceptedConfigFile=file);
verifyEqual(testCase, cfg.force.CalibrationFile, cfgAccepted.force.CalibrationFile);
end
