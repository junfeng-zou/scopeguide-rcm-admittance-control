function [data, stats] = recordWrench(sensor, varargin)
%RECORDWRENCH Record HEX-H data at a fixed polling rate without plotting.
%
% [data, stats] = onrobot.recordWrench(sensor, ...
%     'RateHz', 200, 'DurationSec', 60, 'CsvFile', 'static.csv');
%
% The function is read-only: it never changes the sensor bias or filter.

parser = inputParser;
addParameter(parser, 'RateHz', 200, ...
    @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'DurationSec', 60, ...
    @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'CsvFile', '', @(x) ischar(x) || isstring(x));
addParameter(parser, 'MatFile', '', @(x) ischar(x) || isstring(x));
addParameter(parser, 'Verbose', true, ...
    @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
parse(parser, varargin{:});
opts = parser.Results;

if ~sensor.IsConnected
    sensor.connect();
end

period = 1 / opts.RateHz;
count = max(2, round(opts.DurationSec * opts.RateHz));

sequence = zeros(count, 1, 'uint64');
scheduledTimeSec = (0:count-1).' * period;
readStartTimeSec = zeros(count, 1);
sampleTimeSec = zeros(count, 1);
readDurationSec = zeros(count, 1);
deadlineMiss = false(count, 1);
wrench = zeros(count, 6);

if opts.Verbose
    fprintf('Recording %d samples at %.1f Hz for %.1f s from %s ...\n', ...
        count, opts.RateHz, opts.DurationSec, sensor.Config.IPAddress);
end

recordTimer = tic;
for i = 1:count
    waitUntil(recordTimer, scheduledTimeSec(i));
    readStartTimeSec(i) = toc(recordTimer);
    sample = sensor.readSample();
    readEndTime = toc(recordTimer);

    sequence(i) = sample.sequence;
    sampleTimeSec(i) = sample.monotonicTime;
    readDurationSec(i) = sample.readDuration;
    deadlineMiss(i) = readEndTime > scheduledTimeSec(i) + period;
    wrench(i, :) = sample.wrench.';
end

elapsedSec = toc(recordTimer);
actualRateHz = (count - 1) / ...
    max(readStartTimeSec(end) - readStartTimeSec(1), eps);
startLatenessSec = readStartTimeSec - scheduledTimeSec;

data = table(sequence, scheduledTimeSec, readStartTimeSec, ...
    sampleTimeSec, readDurationSec, startLatenessSec, deadlineMiss, ...
    wrench(:, 1), wrench(:, 2), wrench(:, 3), ...
    wrench(:, 4), wrench(:, 5), wrench(:, 6), ...
    'VariableNames', {'sequence', 'scheduledTimeSec', 'readStartTimeSec', ...
    'sampleTimeSec', 'readDurationSec', 'startLatenessSec', ...
    'deadlineMiss', 'Fx_N', 'Fy_N', 'Fz_N', 'Tx_Nm', 'Ty_Nm', 'Tz_Nm'});

stats = struct();
stats.requestedRateHz = opts.RateHz;
stats.actualRateHz = actualRateHz;
stats.sampleCount = count;
stats.elapsedSec = elapsedSec;
stats.deadlineMissRatio = mean(deadlineMiss);
stats.meanReadDurationMs = 1000 * mean(readDurationSec);
stats.p95ReadDurationMs = 1000 * percentile(readDurationSec, 95);
stats.p99ReadDurationMs = 1000 * percentile(readDurationSec, 99);
stats.timestampUTC = datetime('now', 'TimeZone', 'UTC');

if ~isempty(char(opts.CsvFile))
    writetable(data, char(opts.CsvFile));
end
if ~isempty(char(opts.MatFile))
    save(char(opts.MatFile), 'data', 'stats');
end

if opts.Verbose
    fprintf(['Recorded %.2f Hz, deadline miss %.3f%%, ' ...
        'read P95 %.2f ms.\n'], stats.actualRateHz, ...
        100 * stats.deadlineMissRatio, stats.p95ReadDurationMs);
end
end

function waitUntil(timerObject, targetTime)
while true
    remaining = targetTime - toc(timerObject);
    if remaining <= 0
        return;
    elseif remaining > 0.003
        pause(remaining - 0.001);
    else
        pause(0);
    end
end
end

function value = percentile(data, percentage)
data = sort(double(data(:)));
position = 1 + (numel(data) - 1) * percentage / 100;
lowerIndex = floor(position);
upperIndex = ceil(position);
if lowerIndex == upperIndex
    value = data(lowerIndex);
else
    weight = position - lowerIndex;
    value = data(lowerIndex) * (1 - weight) + data(upperIndex) * weight;
end
end
