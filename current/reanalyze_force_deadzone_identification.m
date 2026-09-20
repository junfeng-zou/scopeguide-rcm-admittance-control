function [report, outputDirectory] = ...
        reanalyze_force_deadzone_identification(resultsMatFile)
%REANALYZE_FORCE_DEADZONE_IDENTIFICATION Reanalyze a saved checkpoint.
% This function performs file I/O and numerical analysis only. It does not
% construct robot/sensor adapters and cannot connect to or command hardware.

arguments
    resultsMatFile (1, 1) string
end

if ~isfile(resultsMatFile)
    error('scopeguide:force:DeadzoneCheckpointMissing', ...
        'Saved checkpoint does not exist: %s', resultsMatFile);
end
source = load(resultsMatFile);
required = {'data', 'report', 'cfg', 'sensorCfg', 'options'};
for index = 1:numel(required)
    if ~isfield(source, required{index})
        error('scopeguide:force:InvalidDeadzoneCheckpoint', ...
            'Saved checkpoint is missing variable %s.', required{index});
    end
end
if ~istable(source.data) || isempty(source.data)
    error('scopeguide:force:InvalidDeadzoneCheckpoint', ...
        'Saved checkpoint contains no signal table rows.');
end

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
addpath(fullfile(projectRoot, 'force_calibration'));

[data, renamed] = ...
    scopeguide.force.normalizeDeadzoneSignalColumnNames(source.data);
cfg = source.cfg;
sensorCfg = source.sensorCfg;
options = source.options;
sourceReport = source.report;
outputDirectory = string(fileparts(resultsMatFile));
paths = outputPaths(outputDirectory);

minimumSamples = max(100, floor(0.25 * ...
    options.DurationPerPoseSec * options.SampleRateHz));
report = scopeguide.force.estimateDeadzoneThresholds(data, ...
    Percentile=options.Percentile, ...
    ForceMarginN=options.ForceMarginN, ...
    MomentMarginNm=options.MomentMarginNm, ...
    CurrentForceDeadzoneN=cfg.force.ForceDeadzoneN, ...
    CurrentMomentDeadzoneNm=cfg.force.MomentDeadzoneNm, ...
    MinimumEligibleRatio=options.MinimumEligibleRatio, ...
    MinimumEligibleSamples=minimumSamples);
report.CalibrationFile = string(cfg.force.CalibrationFile);
report.AutomaticBaselineEnabled = cfg.force.AutomaticBaselineEnabled;
report.SampleRateTargetHz = options.SampleRateHz;
report.CaptureReturnPose = options.CaptureReturnPose;
if options.CaptureReturnPose
    report.ReturnCheck = returnCheck(data, max(data.PoseIndex));
else
    report.ReturnCheck = struct();
end
report.Paths = paths;
report.CompletedLocal = string(datetime('now'));
report.OfflineReanalysis = true;
report.ReusedSavedAcquisition = true;
report.SourceResultsMat = resultsMatFile;
report.SourceCheckpointStatus = string(sourceReport.Status);
if isfield(sourceReport, 'ErrorIdentifier')
    report.SourceErrorIdentifier = string(sourceReport.ErrorIdentifier);
else
    report.SourceErrorIdentifier = "";
end
report.RenamedColumnCount = size(renamed, 1);
report.PhysicalMotionCommandSent = false;
report.SensorZeroCommandSent = false;

writetable(data, paths.SignalsCsv);
writetable(report.PoseStatistics, paths.PoseStatisticsCsv);
writetable(report.Excursions, paths.ExcursionsCsv);
if isempty(renamed)
    renameTable = table(strings(0, 1), strings(0, 1), ...
        'VariableNames', {'LegacyName', 'CanonicalName'});
else
    renameTable = table(renamed(:, 1), renamed(:, 2), ...
        'VariableNames', {'LegacyName', 'CanonicalName'});
end
writetable(renameTable, paths.ColumnRenamesCsv);
createSummaryFigure(data, report, paths);
sourceResultsMat = resultsMatFile;
save(paths.ResultsMat, 'data', 'report', 'cfg', 'sensorCfg', ...
    'options', 'sourceReport', 'sourceResultsMat', 'renamed');
writeJson(paths.SummaryJson, jsonSafeReport(report));

fprintf('\nOffline deadzone reanalysis completed.\n');
fprintf('No robot or sensor connection was opened.\n');
fprintf('Corrected %d legacy column names.\n', size(renamed, 1));
fprintf('Candidate force/moment deadzone: %.3f N / %.4f N*m\n', ...
    report.CandidateForceDeadzoneN, report.CandidateMomentDeadzoneNm);
fprintf('Suggested activation hold: %.0f ms\n', ...
    1000 * report.SuggestedActivationHoldSec);
fprintf('Result validity: %s\n', report.Status);
fprintf('Results: %s\n', outputDirectory);
end

function check = returnCheck(data, returnPoseIndex)
firstRows = data.PoseIndex == 1 & data.AnalysisEligible;
returnRows = data.PoseIndex == returnPoseIndex & data.AnalysisEligible;
if ~any(firstRows) || ~any(returnRows)
    error('scopeguide:force:DeadzoneReturnCheckMissing', ...
        'First or return pose contains no eligible samples.');
end
firstExternal = wrenchColumns(data, "External", firstRows);
returnExternal = wrenchColumns(data, "External", returnRows);
firstSlow = wrenchColumns(data, "Slow", firstRows);
returnSlow = wrenchColumns(data, "Slow", returnRows);
check = struct();
check.FirstPoseExternalMean = mean(firstExternal, 1);
check.ReturnPoseExternalMean = mean(returnExternal, 1);
check.ExternalMeanChange = ...
    check.ReturnPoseExternalMean - check.FirstPoseExternalMean;
check.ExternalForceChangeNormN = norm(check.ExternalMeanChange(1:3));
check.ExternalMomentChangeNormNm = norm(check.ExternalMeanChange(4:6));
check.FirstPoseSlowMean = mean(firstSlow, 1);
check.ReturnPoseSlowMean = mean(returnSlow, 1);
check.SlowMeanChange = check.ReturnPoseSlowMean - check.FirstPoseSlowMean;
check.SlowForceChangeNormN = norm(check.SlowMeanChange(1:3));
check.SlowMomentChangeNormNm = norm(check.SlowMeanChange(4:6));
end

function wrench = wrenchColumns(data, prefix, rows)
names = prefix + ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"];
wrench = zeros(nnz(rows), 6);
for index = 1:6
    wrench(:, index) = data.(names(index))(rows);
end
end

function createSummaryFigure(data, report, paths)
eligible = data.AnalysisEligible;
t = data.SampleTimeSec;
slowForce = [data.SlowFx, data.SlowFy, data.SlowFz];
slowMoment = [data.SlowTx, data.SlowTy, data.SlowTz];
forceNorm = vecnorm(slowForce, 2, 2);
momentNorm = vecnorm(slowMoment, 2, 2);

figureHandle = figure('Visible', 'off', 'Color', 'w', ...
    'Position', [80, 80, 1500, 900]);
figureGuard = onCleanup(@() close(figureHandle));
layout = tiledlayout(figureHandle, 2, 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf(['Deadzone identification: Q_{%.1f} + margin, ' ...
    'candidate %.2f N / %.3f N m'], report.Percentile, ...
    report.CandidateForceDeadzoneN, report.CandidateMomentDeadzoneNm));

forceAxes = nexttile(layout, 1);
plot(forceAxes, t(eligible), forceNorm(eligible), ...
    'Color', [0.05, 0.35, 0.80], 'LineWidth', 0.7);
hold(forceAxes, 'on');
yline(forceAxes, report.CurrentForceDeadzoneN, '--k', 'Current');
yline(forceAxes, report.CandidateForceDeadzoneN, '--r', 'Candidate');
ylabel(forceAxes, '|F_{slow}| [N]');
grid(forceAxes, 'on');

momentAxes = nexttile(layout, 2);
plot(momentAxes, t(eligible), momentNorm(eligible), ...
    'Color', [0.85, 0.25, 0.05], 'LineWidth', 0.7);
hold(momentAxes, 'on');
yline(momentAxes, report.CurrentMomentDeadzoneNm, '--k', 'Current');
yline(momentAxes, report.CandidateMomentDeadzoneNm, '--r', 'Candidate');
xlabel(momentAxes, 'Session time [s]');
ylabel(momentAxes, '|T_{slow}| [N m]');
grid(momentAxes, 'on');

exportgraphics(figureHandle, paths.FigurePng, 'Resolution', 180);
savefig(figureHandle, paths.FigureFig);
clear figureGuard;
end

function paths = outputPaths(directory)
paths = struct();
paths.SignalsCsv = string(fullfile(directory, 'signals_corrected.csv'));
paths.PoseStatisticsCsv = string(fullfile(directory, ...
    'pose_statistics.csv'));
paths.ExcursionsCsv = string(fullfile(directory, 'excursions.csv'));
paths.ColumnRenamesCsv = string(fullfile(directory, ...
    'column_renames.csv'));
paths.ResultsMat = string(fullfile(directory, ...
    'deadzone_identification_reanalyzed.mat'));
paths.SummaryJson = string(fullfile(directory, ...
    'summary_reanalyzed.json'));
paths.FigurePng = string(fullfile(directory, ...
    'deadzone_identification.png'));
paths.FigureFig = string(fullfile(directory, ...
    'deadzone_identification.fig'));
end

function safe = jsonSafeReport(report)
safe = report;
if isfield(safe, 'PoseStatistics') && istable(safe.PoseStatistics)
    safe.PoseStatistics = table2struct(safe.PoseStatistics);
end
if isfield(safe, 'Excursions') && istable(safe.Excursions)
    safe.Excursions = table2struct(safe.Excursions);
end
end

function writeJson(path, payload)
try
    text = jsonencode(payload, 'PrettyPrint', true);
catch
    text = jsonencode(payload);
end
file = fopen(path, 'w');
if file < 0
    error('scopeguide:force:DeadzoneJsonWrite', ...
        'Cannot open %s for writing.', path);
end
fileGuard = onCleanup(@() fclose(file));
fwrite(file, text, 'char');
clear fileGuard;
end
