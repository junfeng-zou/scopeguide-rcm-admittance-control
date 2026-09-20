function [data, stats, driftSummary, outputDirectory] = ...
        run_hex_h_10min_drift_test(varargin)
%RUN_HEX_H_10MIN_DRIFT_TEST Record and plot a stationary HEX-H for 10 min.
%
% Run from the ScopeGuide project directory:
%   [data, stats, driftSummary, outputDirectory] = ...
%       run_hex_h_10min_drift_test();
%
% The sensor and everything mounted after it should remain stationary and
% untouched throughout the recording. This function is read-only and does
% not call sensor.zero() or sensor.unzero().

parser = inputParser;
addParameter(parser, 'DurationSec', 10 * 60, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
addParameter(parser, 'RateHz', 200, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
addParameter(parser, 'MeanWindowSec', 30, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
parse(parser, varargin{:});
opts = parser.Results;

projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot, 'force_sensor'));

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
outputDirectory = fullfile(projectRoot, 'results', ...
    ['hex_h_10min_drift_' timestamp]);
if ~isfolder(outputDirectory)
    mkdir(outputDirectory);
end

csvFile = fullfile(outputDirectory, 'wrench_6axis.csv');
matFile = fullfile(outputDirectory, 'wrench_6axis.mat');
summaryFile = fullfile(outputDirectory, 'drift_summary.csv');
pngFile = fullfile(outputDirectory, 'wrench_6axis.png');
figFile = fullfile(outputDirectory, 'wrench_6axis.fig');

cfg = onrobot.defaultConfig();
cfg.SampleRateHz = opts.RateHz;
cfg.Backend = 'auto';

sensor = onrobot.HexHClient(cfg);
sensorCleanup = onCleanup(@() sensor.disconnect());

fprintf('\nHEX-H six-axis stationary drift test\n');
fprintf('Duration: %.1f min | target rate: %.1f Hz\n', ...
    opts.DurationSec / 60, opts.RateHz);
fprintf('Please keep the robot, sensor, cable and mounted tool stationary.\n');
fprintf('The program will NOT change the sensor bias.\n\n');

sensor.connect();
[data, stats] = onrobot.recordWrench(sensor, ...
    'RateHz', opts.RateHz, ...
    'DurationSec', opts.DurationSec, ...
    'CsvFile', csvFile, ...
    'MatFile', matFile, ...
    'Verbose', true);
sensor.disconnect();

timeSec = data.readStartTimeSec - data.readStartTimeSec(1);
timeMin = timeSec / 60;
wrench = [data.Fx_N, data.Fy_N, data.Fz_N, ...
    data.Tx_Nm, data.Ty_Nm, data.Tz_Nm];

% Compare the first and last stationary windows. Averaging prevents random
% sample noise from being mistaken for long-term drift.
meanWindowSec = min(opts.MeanWindowSec, max(timeSec(end) / 3, eps));
firstWindow = timeSec <= timeSec(1) + meanWindowSec;
lastWindow = timeSec >= timeSec(end) - meanWindowSec;
firstMean = mean(wrench(firstWindow, :), 1);
lastMean = mean(wrench(lastWindow, :), 1);
meanChange = lastMean - firstMean;

linearDriftPerMin = zeros(1, 6);
for axisIndex = 1:6
    coefficients = polyfit(timeMin, wrench(:, axisIndex), 1);
    linearDriftPerMin(axisIndex) = coefficients(1);
end
peakToPeak = max(wrench, [], 1) - min(wrench, [], 1);

axisName = {'Fx'; 'Fy'; 'Fz'; 'Tx'; 'Ty'; 'Tz'};
unit = {'N'; 'N'; 'N'; 'N*m'; 'N*m'; 'N*m'};
driftSummary = table(axisName, unit, firstMean.', lastMean.', ...
    meanChange.', linearDriftPerMin.', peakToPeak.', ...
    'VariableNames', {'Axis', 'Unit', 'FirstWindowMean', ...
    'LastWindowMean', 'LastMinusFirst', 'LinearDriftPerMin', ...
    'PeakToPeak'});
writetable(driftSummary, summaryFile);

% A one-second moving mean makes slow drift visible while the light trace
% retains the raw measurements. Limit plotted points to keep the FIG small.
smoothingSamples = max(1, round(stats.actualRateHz));
smoothedWrench = movmean(wrench, smoothingSamples, 1, ...
    'Endpoints', 'shrink');
plotStride = max(1, ceil(height(data) / 12000));
plotIndices = 1:plotStride:height(data);

figureHandle = figure('Color', 'w', 'Position', [80, 60, 1500, 950]);
layout = tiledlayout(figureHandle, 3, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf(['HEX-H stationary six-axis recording ' ...
    '(%.2f min, %.1f Hz)'], timeMin(end), stats.actualRateHz));
xlabel(layout, 'Time [min]');

axisLabels = {'F_x [N]', 'F_y [N]', 'F_z [N]', ...
    'T_x [N m]', 'T_y [N m]', 'T_z [N m]'};
for axisIndex = 1:6
    axesHandle = nexttile(layout, axisIndex);
    plot(axesHandle, timeMin(plotIndices), ...
        wrench(plotIndices, axisIndex), ...
        'Color', [0.76, 0.76, 0.76], 'LineWidth', 0.5);
    hold(axesHandle, 'on');
    plot(axesHandle, timeMin(plotIndices), ...
        smoothedWrench(plotIndices, axisIndex), ...
        'Color', [0.00, 0.35, 0.80], 'LineWidth', 1.2);
    grid(axesHandle, 'on');
    box(axesHandle, 'on');
    xlim(axesHandle, [timeMin(1), timeMin(end)]);
    ylabel(axesHandle, axisLabels{axisIndex}, 'Interpreter', 'tex');
    title(axesHandle, sprintf('%s: change = %+.4f %s', ...
        axisName{axisIndex}, meanChange(axisIndex), unit{axisIndex}), ...
        'Interpreter', 'none');
    legend(axesHandle, {'Raw', '1 s moving mean'}, ...
        'Location', 'best');
end

exportgraphics(figureHandle, pngFile, 'Resolution', 180);
savefig(figureHandle, figFile);

stats.outputDirectory = outputDirectory;
stats.meanWindowSec = meanWindowSec;
stats.csvFile = csvFile;
stats.matFile = matFile;
stats.summaryFile = summaryFile;
stats.pngFile = pngFile;
stats.figFile = figFile;
save(matFile, 'data', 'stats', 'driftSummary', 'cfg');

fprintf('\nFirst/last %.1f s mean comparison:\n', meanWindowSec);
disp(driftSummary(:, {'Axis', 'Unit', 'FirstWindowMean', ...
    'LastWindowMean', 'LastMinusFirst', 'LinearDriftPerMin'}));
fprintf('Results saved to:\n%s\n', outputDirectory);
end
