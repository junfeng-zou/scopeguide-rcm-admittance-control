function [data, stats] = record_static_load(durationSec)
%RECORD_STATIC_LOAD Record stationary fixed-load HEX-H data without plots.

if nargin < 1
    durationSec = 60;
end

exampleDir = fileparts(mfilename('fullpath'));
packageRoot = fileparts(exampleDir);
addpath(packageRoot);

cfg = onrobot.defaultConfig();
cfg.SampleRateHz = 200;
cfg.Backend = 'auto';

sensor = onrobot.HexHClient(cfg);
cleanup = onCleanup(@() sensor.disconnect());
sensor.connect();

outputDir = fullfile(packageRoot, 'data');
if ~isfolder(outputDir)
    mkdir(outputDir);
end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
csvFile = fullfile(outputDir, ['hex_h_static_' timestamp '.csv']);
matFile = fullfile(outputDir, ['hex_h_static_' timestamp '.mat']);

[data, stats] = onrobot.recordWrench(sensor, ...
    'RateHz', cfg.SampleRateHz, ...
    'DurationSec', durationSec, ...
    'CsvFile', csvFile, ...
    'MatFile', matFile, ...
    'Verbose', true);

fprintf('CSV: %s\n', csvFile);
fprintf('MAT: %s\n', matFile);
end
