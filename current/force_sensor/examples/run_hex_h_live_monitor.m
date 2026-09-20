function run_hex_h_live_monitor()
%RUN_HEX_H_LIVE_MONITOR Connect, read, visualize, and log HEX-H data.

exampleDir = fileparts(mfilename('fullpath'));
packageRoot = fileparts(exampleDir);
addpath(packageRoot);

cfg = onrobot.defaultConfig();
cfg.IPAddress = '192.168.50.201';
cfg.SampleRateHz = 200;
cfg.PlotWindowSec = 10;
cfg.Backend = 'auto';

sensor = onrobot.HexHClient(cfg);
cleanup = onCleanup(@() sensor.disconnect());

fprintf('Connecting to OnRobot Compute Box at %s:%d ...\n', ...
    cfg.IPAddress, cfg.Port);
sensor.connect();
fprintf('Connected using backend: %s\n', sensor.BackendInUse);

% Do not call sensor.zero() automatically. Zero only when the sensor is
% unloaded and stable, and when changing the device bias is intentional.
sample = sensor.readSample();
fprintf(['Initial wrench: F=[%+.2f %+.2f %+.2f] N, ' ...
    'T=[%+.3f %+.3f %+.3f] N m\n'], sample.wrench);

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
logFile = fullfile(pwd, ['hex_h_' timestamp '.csv']);
stats = onrobot.monitor(sensor, ...
    'RateHz', cfg.SampleRateHz, ...
    'WindowSec', cfg.PlotWindowSec, ...
    'DurationSec', inf, ...
    'CsvFile', logFile);

fprintf('Stopped. %.1f Hz, %d samples, CSV: %s\n', ...
    stats.actualRateHz, stats.sampleCount, logFile);
end
