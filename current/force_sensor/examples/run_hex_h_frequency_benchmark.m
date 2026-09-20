function report = run_hex_h_frequency_benchmark()
%RUN_HEX_H_FREQUENCY_BENCHMARK Find the stable HEX-H polling frequency.
% No plots are created and the sensor bias is not changed.

exampleDir = fileparts(mfilename('fullpath'));
packageRoot = fileparts(exampleDir);
addpath(packageRoot);

cfg = onrobot.defaultConfig();
cfg.Backend = 'auto';
cfg.Timeout = 1.0;

sensor = onrobot.HexHClient(cfg);
cleanup = onCleanup(@() sensor.disconnect());
sensor.connect();

report = onrobot.benchmarkFrequency(sensor, ...
    'RatesHz', [], ...              % []: choose rates from burst throughput
    'DurationPerRateSec', 5, ...
    'WarmupSamples', 20, ...
    'BurstSamples', 100, ...
    'MaxAutoRateHz', 500, ...
    'MinRateRatio', 0.98, ...
    'MaxDeadlineMissRatio', 0.01, ...
    'MaxP95JitterFraction', 0.25, ...
    'Verbose', true);

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
csvFile = fullfile(pwd, ['hex_h_frequency_' timestamp '.csv']);
matFile = fullfile(pwd, ['hex_h_frequency_' timestamp '.mat']);
writetable(report.results, csvFile);
save(matFile, 'report');

fprintf('CSV report: %s\n', csvFile);
fprintf('MAT report: %s\n', matFile);
end
