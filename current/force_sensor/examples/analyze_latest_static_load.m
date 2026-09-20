function report = analyze_latest_static_load()
%ANALYZE_LATEST_STATIC_LOAD Analyze the newest static-load MAT recording.

exampleDir = fileparts(mfilename('fullpath'));
packageRoot = fileparts(exampleDir);
addpath(packageRoot);
dataDir = fullfile(packageRoot, 'data');

files = dir(fullfile(dataDir, 'hex_h_static_*.mat'));
if isempty(files)
    error('No static-load MAT recording found in %s.', dataDir);
end
[~, newestIndex] = max([files.datenum]);
inputFile = fullfile(files(newestIndex).folder, files(newestIndex).name);
recording = load(inputFile);

report = onrobot.analyzeStaticNoise(recording.data);
disp(report.rawStatistics);
disp(report.spectrumSummary);
disp(report.filterComparison);

[~, baseName] = fileparts(inputFile);
statisticsFile = fullfile(dataDir, [baseName '_statistics.csv']);
filtersFile = fullfile(dataDir, [baseName '_filters.csv']);
analysisFile = fullfile(dataDir, [baseName '_analysis.mat']);
writetable(report.rawStatistics, statisticsFile);
writetable(report.filterComparison, filtersFile);
save(analysisFile, 'report');

fprintf('Analyzed: %s\n', inputFile);
fprintf('Statistics: %s\n', statisticsFile);
fprintf('Filters: %s\n', filtersFile);
fprintf('Analysis MAT: %s\n', analysisFile);
end
