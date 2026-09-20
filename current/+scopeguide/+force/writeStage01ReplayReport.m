function outputDirectory = writeStage01ReplayReport(projectRoot, cfg, replay)
%WRITESTAGE01REPLAYREPORT Save deterministic Stage 1 replay artifacts.

arguments
    projectRoot (1, 1) string
    cfg (1, 1) struct
    replay (1, 1) struct
end

timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
outputDirectory = fullfile(projectRoot, 'results', ...
    "stage01_force_replay_" + timestamp);
[created, message] = mkdir(outputDirectory);
if ~created && ~isfolder(outputDirectory)
    error('scopeguide:force:CannotCreateReplayDirectory', ...
        'Cannot create %s: %s', outputDirectory, message);
end

summary = replay.Summary;
summary.Stage = 1;
summary.Status = "offline_replay_complete";
summary.PhysicalMotionCommandSent = false;
summary.AutomaticBaselineEnabled = cfg.force.AutomaticBaselineEnabled;
summary.SafetyThresholdsValidatedForMotion = ...
    cfg.safety.ThresholdsValidated;

writeJson(fullfile(outputDirectory, 'summary.json'), summary);
writeJson(fullfile(outputDirectory, 'config_snapshot.json'), cfg);
save(fullfile(outputDirectory, 'signals.mat'), 'replay', '-v7.3');

signalTable = table(replay.PoseIndex, double(replay.RecordedSequence), ...
    replay.TimestampSec, replay.DtSec, replay.SampleAgeSec, ...
    replay.QualityValid, replay.MotionInputValid, ...
    replay.NeutralCheckPassed, replay.SafetyWarning, replay.SafetyStop, ...
    replay.StatusCode, replay.SafetyWarningReasons, ...
    replay.SafetyStopReasons, ...
    'VariableNames', {'PoseIndex', 'Sequence', 'TimestampSec', 'DtSec', ...
    'SampleAgeSec', 'QualityValid', 'MotionInputValid', ...
    'NeutralCheckPassed', 'SafetyWarning', 'SafetyStop', 'StatusCode', ...
    'SafetyWarningReasons', 'SafetyStopReasons'});
signalTable = addWrenchColumns(signalTable, ...
    replay.RawWrenchSensor, 'RawSensor');
signalTable = addWrenchColumns(signalTable, ...
    replay.ExternalWrenchToolAtSensorOrigin, 'ExternalToolAtSensorOrigin');
signalTable = addWrenchColumns(signalTable, ...
    replay.FastWrenchToolAtSensorOrigin, 'FastToolAtSensorOrigin');
signalTable = addWrenchColumns(signalTable, ...
    replay.SlowWrenchToolAtSensorOrigin, 'SlowToolAtSensorOrigin');
signalTable = addWrenchColumns(signalTable, ...
    replay.BaselineWrenchTool, 'BaselineTool');
signalTable.ControlFxToolN = replay.ControlForceTool(1, :).';
signalTable.ControlFyToolN = replay.ControlForceTool(2, :).';
signalTable.ControlFzToolN = replay.ControlForceTool(3, :).';
writetable(signalTable, fullfile(outputDirectory, 'signals.csv'));
end

function output = addWrenchColumns(input, wrench, prefix)
output = input;
suffixes = {'FxN', 'FyN', 'FzN', 'TxNm', 'TyNm', 'TzNm'};
for index = 1:6
    output.([prefix, suffixes{index}]) = wrench(index, :).';
end
end

function writeJson(filePath, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end
[fileId, message] = fopen(filePath, 'w');
if fileId < 0
    error('scopeguide:force:CannotWriteReplayJson', ...
        'Cannot open %s: %s', filePath, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end
