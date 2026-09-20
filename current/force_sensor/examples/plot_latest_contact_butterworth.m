function outputs = plot_latest_contact_butterworth()
%PLOT_LATEST_CONTACT_BUTTERWORTH Compare contact data before/after filtering.

% Uses the newest hex_h_contact_*.mat recording and applies a causal
% second-order 10 Hz Butterworth low-pass at 200 Hz, sample by sample.

exampleDir = fileparts(mfilename('fullpath'));
packageRoot = fileparts(exampleDir);
addpath(packageRoot);
dataDir = fullfile(packageRoot, 'data');

files = dir(fullfile(dataDir, 'hex_h_contact_*.mat'));
if isempty(files)
    error('No contact-force MAT recording found in %s.', dataDir);
end
[~, newestIndex] = max([files.datenum]);
inputFile = fullfile(files(newestIndex).folder, files(newestIndex).name);
recording = load(inputFile);
data = recording.data;

sampleRateHz = 200;
cutoffHz = 10;
rawWrench = [data.Fx_N, data.Fy_N, data.Fz_N, ...
    data.Tx_Nm, data.Ty_Nm, data.Tz_Nm];
timeSec = data.readStartTimeSec - data.readStartTimeSec(1);

filterBank = onrobot.WrenchFilterBank(sampleRateHz, 20, cutoffHz);
filteredWrench = zeros(size(rawWrench));
filterBank.reset(rawWrench(1, :).');
for sampleIndex = 1:size(rawWrench, 1)
    [~, filteredSample] = filterBank.step(rawWrench(sampleIndex, :).');
    filteredWrench(sampleIndex, :) = filteredSample.';
end

filteredData = data;
filteredData.Fx_filtered_N = filteredWrench(:, 1);
filteredData.Fy_filtered_N = filteredWrench(:, 2);
filteredData.Fz_filtered_N = filteredWrench(:, 3);
filteredData.Tx_filtered_Nm = filteredWrench(:, 4);
filteredData.Ty_filtered_Nm = filteredWrench(:, 5);
filteredData.Tz_filtered_Nm = filteredWrench(:, 6);

[~, baseName] = fileparts(inputFile);
outputBase = fullfile(dataDir, [baseName '_butter2_10Hz']);
csvFile = [outputBase '.csv'];
pngFile = [outputBase '.png'];
figFile = [outputBase '.fig'];
writetable(filteredData, csvFile);

figureHandle = figure('Visible', 'off', 'Color', 'w', ...
    'Position', [100, 100, 1500, 950]);
layout = tiledlayout(figureHandle, 3, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf([ ...
    'OnRobot HEX-H QC contact test: Raw vs causal Butterworth ' ...
    '(f_s = %d Hz, f_c = %d Hz)'], sampleRateHz, cutoffHz), ...
    'FontWeight', 'bold');
xlabel(layout, 'Time [s]');

axisIndices = [1 4; 2 5; 3 6];
axisLabels = {'F_x', '\tau_x'; 'F_y', '\tau_y'; 'F_z', '\tau_z'};
units = {'N', 'N m'; 'N', 'N m'; 'N', 'N m'};
filteredColors = {[0.00 0.32 0.85], [0.85 0.25 0.05]};

for row = 1:3
    for column = 1:2
        axisIndex = axisIndices(row, column);
        axesHandle = nexttile(layout, (row - 1) * 2 + column);
        plot(axesHandle, timeSec, rawWrench(:, axisIndex), ...
            'Color', [0.72 0.72 0.72], 'LineWidth', 0.65);
        hold(axesHandle, 'on');
        plot(axesHandle, timeSec, filteredWrench(:, axisIndex), ...
            'Color', filteredColors{column}, 'LineWidth', 1.25);
        grid(axesHandle, 'on');
        box(axesHandle, 'on');
        xlim(axesHandle, [timeSec(1), timeSec(end)]);
        ylabel(axesHandle, sprintf('%s [%s]', ...
            axisLabels{row, column}, units{row, column}), ...
            'Interpreter', 'tex');
        title(axesHandle, axisLabels{row, column}, 'Interpreter', 'tex');
        legend(axesHandle, {'Raw', 'Butterworth 10 Hz'}, ...
            'Location', 'best');
    end
end

exportgraphics(figureHandle, pngFile, 'Resolution', 180);
savefig(figureHandle, figFile);
close(figureHandle);

outputs = struct('inputFile', inputFile, 'csvFile', csvFile, ...
    'pngFile', pngFile, 'figFile', figFile, ...
    'sampleRateHz', sampleRateHz, 'cutoffHz', cutoffHz);

fprintf('Input: %s\n', inputFile);
fprintf('Filtered CSV: %s\n', csvFile);
fprintf('Comparison PNG: %s\n', pngFile);
fprintf('Editable FIG: %s\n', figFile);
end
