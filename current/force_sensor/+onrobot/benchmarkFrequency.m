function report = benchmarkFrequency(sensor, varargin)
%BENCHMARKFREQUENCY Measure the highest stable HEX-H polling frequency.
%
% report = onrobot.benchmarkFrequency(sensor)
% report = onrobot.benchmarkFrequency(sensor, 'RatesHz', [25 50 75 100])
%
% A rate is considered stable when all configured conditions hold:
%   1. achieved rate >= MinRateRatio * requested rate;
%   2. deadline miss ratio <= MaxDeadlineMissRatio;
%   3. P95 absolute period jitter <= MaxP95JitterFraction * period.
%
% This function performs no visualization and does not zero the sensor.

parser = inputParser;
addParameter(parser, 'RatesHz', [], ...
    @(x) isempty(x) || (isnumeric(x) && isvector(x) && all(x > 0)));
addParameter(parser, 'DurationPerRateSec', 5, ...
    @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'WarmupSamples', 20, ...
    @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(parser, 'BurstSamples', 100, ...
    @(x) isnumeric(x) && isscalar(x) && x >= 2);
addParameter(parser, 'MaxAutoRateHz', 500, ...
    @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'MinRateRatio', 0.98, ...
    @(x) isnumeric(x) && isscalar(x) && x > 0 && x <= 1);
addParameter(parser, 'MaxDeadlineMissRatio', 0.01, ...
    @(x) isnumeric(x) && isscalar(x) && x >= 0 && x <= 1);
addParameter(parser, 'MaxP95JitterFraction', 0.25, ...
    @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(parser, 'Verbose', true, ...
    @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
parse(parser, varargin{:});
opts = parser.Results;

if ~sensor.IsConnected
    sensor.connect();
end

if opts.Verbose
    fprintf('HEX-H frequency benchmark using backend: %s\n', ...
        sensor.BackendInUse);
    fprintf('Warm-up: %d samples\n', opts.WarmupSamples);
end

for i = 1:opts.WarmupSamples
    sensor.readSample();
end

% First measure the upper bound without intentional waiting.
burstStart = zeros(opts.BurstSamples, 1);
burstEnd = zeros(opts.BurstSamples, 1);
burstReadSec = zeros(opts.BurstSamples, 1);
burstTimer = tic;
for i = 1:opts.BurstSamples
    burstStart(i) = toc(burstTimer);
    sample = sensor.readSample();
    burstEnd(i) = toc(burstTimer);
    burstReadSec(i) = sample.readDuration;
end
burstRateHz = opts.BurstSamples / max(burstEnd(end) - burstStart(1), eps);

rates = opts.RatesHz;
if isempty(rates)
    upperRate = min(opts.MaxAutoRateHz, ...
        max(20, ceil(burstRateHz / 10) * 10));
    standardRates = [20 25 30 40 50 60 75 100 125 150 200 250 ...
        300 350 400 450 500];
    rates = standardRates(standardRates <= upperRate);
    if isempty(rates) || rates(end) < upperRate
        rates = [rates, upperRate];
    end
end
rates = unique(sort(double(rates(:).')));

if opts.Verbose
    fprintf(['Burst upper bound: %.1f Hz ' ...
        '(read P95 %.2f ms, P99 %.2f ms)\n'], ...
        burstRateHz, 1000 * localPercentile(burstReadSec, 95), ...
        1000 * localPercentile(burstReadSec, 99));
    fprintf('Testing requested rates: %s Hz\n\n', num2str(rates));
end

nRates = numel(rates);
requestedRateHz = rates(:);
achievedRateHz = zeros(nRates, 1);
rateRatio = zeros(nRates, 1);
meanReadMs = zeros(nRates, 1);
p95ReadMs = zeros(nRates, 1);
p99ReadMs = zeros(nRates, 1);
p95AbsJitterMs = zeros(nRates, 1);
deadlineMissRatio = zeros(nRates, 1);
sampleCount = zeros(nRates, 1);
stable = false(nRates, 1);

for rateIndex = 1:nRates
    targetRate = rates(rateIndex);
    period = 1 / targetRate;
    count = max(2, ceil(opts.DurationPerRateSec * targetRate));
    starts = zeros(count, 1);
    ends = zeros(count, 1);
    readSec = zeros(count, 1);
    missed = false(count, 1);

    testTimer = tic;
    scheduledStart = 0;
    for sampleIndex = 1:count
        waitUntil(testTimer, scheduledStart);
        starts(sampleIndex) = toc(testTimer);
        sample = sensor.readSample();
        ends(sampleIndex) = toc(testTimer);
        readSec(sampleIndex) = sample.readDuration;

        nextDeadline = scheduledStart + period;
        missed(sampleIndex) = ends(sampleIndex) > nextDeadline;
        scheduledStart = nextDeadline;
    end

    intervals = diff(starts);
    achieved = (count - 1) / max(starts(end) - starts(1), eps);
    absoluteJitter = abs(intervals - period);

    sampleCount(rateIndex) = count;
    achievedRateHz(rateIndex) = achieved;
    rateRatio(rateIndex) = achieved / targetRate;
    meanReadMs(rateIndex) = 1000 * mean(readSec);
    p95ReadMs(rateIndex) = 1000 * localPercentile(readSec, 95);
    p99ReadMs(rateIndex) = 1000 * localPercentile(readSec, 99);
    p95AbsJitterMs(rateIndex) = 1000 * localPercentile(absoluteJitter, 95);
    deadlineMissRatio(rateIndex) = mean(missed);

    stable(rateIndex) = ...
        rateRatio(rateIndex) >= opts.MinRateRatio && ...
        deadlineMissRatio(rateIndex) <= opts.MaxDeadlineMissRatio && ...
        p95AbsJitterMs(rateIndex) <= ...
            1000 * opts.MaxP95JitterFraction * period;

    if opts.Verbose
        fprintf(['%6.1f Hz -> actual %6.1f Hz | read P95 %6.2f ms | ' ...
            'jitter P95 %6.2f ms | miss %6.2f %% | %s\n'], ...
            targetRate, achievedRateHz(rateIndex), p95ReadMs(rateIndex), ...
            p95AbsJitterMs(rateIndex), 100 * deadlineMissRatio(rateIndex), ...
            stableLabel(stable(rateIndex)));
    end
end

results = table(requestedRateHz, achievedRateHz, rateRatio, sampleCount, ...
    meanReadMs, p95ReadMs, p99ReadMs, p95AbsJitterMs, ...
    deadlineMissRatio, stable);

if any(stable)
    maxStableHz = max(requestedRateHz(stable));
else
    maxStableHz = NaN;
end

report = struct();
report.timestampUTC = datetime('now', 'TimeZone', 'UTC');
report.backend = sensor.BackendInUse;
report.ipAddress = sensor.Config.IPAddress;
report.burstRateHz = burstRateHz;
report.burstMeanReadMs = 1000 * mean(burstReadSec);
report.burstP95ReadMs = 1000 * localPercentile(burstReadSec, 95);
report.burstP99ReadMs = 1000 * localPercentile(burstReadSec, 99);
report.maxStableHz = maxStableHz;
report.criteria = struct( ...
    'MinRateRatio', opts.MinRateRatio, ...
    'MaxDeadlineMissRatio', opts.MaxDeadlineMissRatio, ...
    'MaxP95JitterFraction', opts.MaxP95JitterFraction);
report.results = results;

if opts.Verbose
    fprintf('\n');
    if isnan(maxStableHz)
        fprintf('No tested rate satisfied the stability criteria.\n');
    else
        fprintf('Highest stable tested rate: %.1f Hz\n', maxStableHz);
    end
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

function value = localPercentile(data, percentage)
data = sort(double(data(:)));
if isempty(data)
    value = NaN;
    return;
end
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

function label = stableLabel(isStable)
if isStable
    label = 'STABLE';
else
    label = 'UNSTABLE';
end
end
