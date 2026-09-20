function outputDirectory = writeStage00EnvironmentReport( ...
    projectRoot, cfg, environment)
%WRITESTAGE00ENVIRONMENTREPORT Save Stage 0 configuration and environment.

arguments
    projectRoot (1, 1) string
    cfg (1, 1) struct
    environment (1, 1) struct
end

timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
outputDirectory = fullfile(projectRoot, 'results', ...
    "stage00_environment_" + timestamp);
if ~isfolder(outputDirectory)
    [created, message] = mkdir(outputDirectory);
    if ~created
        error('scopeguide:runtime:CannotCreateResultsDirectory', ...
            'Cannot create %s: %s', outputDirectory, message);
    end
end

summary = struct();
summary.Stage = 0;
summary.Topic = "environment";
summary.Status = "initialized_offline";
summary.PhysicalMotionCommandSent = false;
summary.DefaultMotionAuthorized = false;
summary.QpRoute = environment.Optimization.QpRoute;
summary.Notes = [ ...
    "Stage 0 performs no robot or sensor connection."; ...
    "A dry-run DLS route is not permission for physical motion."];

writeJson(fullfile(outputDirectory, 'summary.json'), summary);
writeJson(fullfile(outputDirectory, 'config_snapshot.json'), cfg);
writeJson(fullfile(outputDirectory, 'environment.json'), environment);
end

function writeJson(filePath, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end

[fileId, message] = fopen(filePath, 'w');
if fileId < 0
    error('scopeguide:runtime:CannotWriteJson', ...
        'Cannot open %s: %s', filePath, message);
end
cleanup = onCleanup(@() fclose(fileId));
fwrite(fileId, encoded, 'char');
fwrite(fileId, newline, 'char');
clear cleanup;
end
