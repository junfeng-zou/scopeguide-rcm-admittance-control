function stats = monitor(sensor, varargin)
%MONITOR Read and visualize HEX-H data at a controlled rate.
%
% stats = onrobot.monitor(sensor, 'RateHz', 200, 'DurationSec', inf, ...
%     'WindowSec', 10, 'CsvFile', '', 'SampleCallback', []);
%
% SampleCallback receives each sample struct. This is useful for passing the
% same reader directly into a later safety-state estimator.

parser = inputParser;
addParameter(parser, 'RateHz', 200, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'DurationSec', inf, ...
    @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'WindowSec', 10, ...
    @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'CsvFile', '', @(x) ischar(x) || isstring(x));
addParameter(parser, 'SampleCallback', [], ...
    @(x) isempty(x) || isa(x, 'function_handle'));
parse(parser, varargin{:});
opts = parser.Results;

if ~sensor.IsConnected
    sensor.connect();
end

plotter = onrobot.WrenchPlotter(opts.WindowSec);
period = 1 / opts.RateHz;
loopTimer = tic;
nextSampleTime = 0;
sampleCount = 0;
droppedDeadlines = 0;
readTimeSum = 0;

fid = -1;
if ~isempty(char(opts.CsvFile))
    [fid, message] = fopen(char(opts.CsvFile), 'w');
    if fid < 0
        error('onrobot:monitor:CsvOpenFailed', ...
            'Cannot open CSV file: %s', message);
    end
    csvCleanup = onCleanup(@() fclose(fid));
    fprintf(fid, ['sequence,monotonic_time_s,read_duration_s,' ...
        'Fx_N,Fy_N,Fz_N,Tx_Nm,Ty_Nm,Tz_Nm\n']);
end

while plotter.isOpen() && toc(loopTimer) < opts.DurationSec
    nowTime = toc(loopTimer);
    if nowTime < nextSampleTime
        pause(min(nextSampleTime - nowTime, 0.002));
        continue;
    end

    sample = sensor.readSample();
    sampleCount = sampleCount + 1;
    readTimeSum = readTimeSum + sample.readDuration;
    plotter.update(sample);

    if fid >= 0
        fprintf(fid, '%u,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
            sample.sequence, sample.monotonicTime, sample.readDuration, ...
            sample.wrench(1), sample.wrench(2), sample.wrench(3), ...
            sample.wrench(4), sample.wrench(5), sample.wrench(6));
    end

    if ~isempty(opts.SampleCallback)
        opts.SampleCallback(sample);
    end

    nextSampleTime = nextSampleTime + period;
    if toc(loopTimer) > nextSampleTime + period
        droppedDeadlines = droppedDeadlines + 1;
        nextSampleTime = toc(loopTimer) + period;
    end
end

elapsed = toc(loopTimer);
stats = struct();
stats.sampleCount = sampleCount;
stats.elapsedSec = elapsed;
stats.actualRateHz = sampleCount / max(elapsed, eps);
stats.droppedDeadlines = droppedDeadlines;
stats.meanReadDurationSec = readTimeSum / max(sampleCount, 1);
end
