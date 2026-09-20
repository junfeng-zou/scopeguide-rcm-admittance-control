function report = analyzeStaticNoise(data, varargin)
%ANALYZESTATICNOISE Analyze stationary HEX-H noise and causal filters.
%
% report = onrobot.analyzeStaticNoise(data)
%
% No Signal Processing Toolbox is required. The function contains its own
% Welch PSD and second-order Butterworth low-pass implementations.

parser = inputParser;
addParameter(parser, 'SampleRateHz', [], ...
    @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
addParameter(parser, 'SettleTimeSec', 2, ...
    @(x) isnumeric(x) && isscalar(x) && x >= 0);
parse(parser, varargin{:});
opts = parser.Results;

required = {'readStartTimeSec', 'Fx_N', 'Fy_N', 'Fz_N', ...
    'Tx_Nm', 'Ty_Nm', 'Tz_Nm'};
for i = 1:numel(required)
    if ~ismember(required{i}, data.Properties.VariableNames)
        error('onrobot:analyzeStaticNoise:MissingVariable', ...
            'Input table is missing %s.', required{i});
    end
end

t = double(data.readStartTimeSec(:));
X = [data.Fx_N, data.Fy_N, data.Fz_N, ...
     data.Tx_Nm, data.Ty_Nm, data.Tz_Nm];
axisName = {'Fx'; 'Fy'; 'Fz'; 'Tx'; 'Ty'; 'Tz'};
unit = {'N'; 'N'; 'N'; 'N*m'; 'N*m'; 'N*m'};

if isempty(opts.SampleRateHz)
    fs = 1 / median(diff(t));
else
    fs = opts.SampleRateHz;
end

meanValue = mean(X, 1).';
stdValue = std(X, 0, 1).';
peakToPeak = (max(X, [], 1) - min(X, [], 1)).';
robustSigma = zeros(6, 1);
driftPerMin = zeros(6, 1);
for axisIndex = 1:6
    robustSigma(axisIndex) = 1.4826 * ...
        median(abs(X(:, axisIndex) - median(X(:, axisIndex))));
    coefficients = polyfit(t, X(:, axisIndex), 1);
    driftPerMin(axisIndex) = 60 * coefficients(1);
end

rawStatistics = table(axisName, unit, meanValue, stdValue, ...
    robustSigma, peakToPeak, driftPerMin);

Xd = detrendLinear(t, X);
noiseStd = std(Xd, 0, 1);
[psdValue, frequencyHz] = welchPsd(Xd, fs);

bandsHz = [0.1 1; 1 5; 5 10; 10 20; 20 50; 50 fs/2];
validBands = bandsHz(:, 1) < bandsHz(:, 2);
bandsHz = bandsHz(validBands, :);
bandRms = zeros(size(bandsHz, 1), 6);
for bandIndex = 1:size(bandsHz, 1)
    selected = frequencyHz >= bandsHz(bandIndex, 1) & ...
        frequencyHz < bandsHz(bandIndex, 2);
    if nnz(selected) >= 2
        for axisIndex = 1:6
            bandRms(bandIndex, axisIndex) = sqrt(trapz( ...
                frequencyHz(selected), psdValue(selected, axisIndex)));
        end
    end
end

dominantFrequencyHz = zeros(6, 1);
selected = frequencyHz >= 0.2;
for axisIndex = 1:6
    selectedPsd = psdValue(selected, axisIndex);
    selectedFrequency = frequencyHz(selected);
    [~, maximumIndex] = max(selectedPsd);
    dominantFrequencyHz(axisIndex) = selectedFrequency(maximumIndex);
end
spectrumSummary = table(axisName, dominantFrequencyHz);

settleSamples = min(size(X, 1) - 2, round(opts.SettleTimeSec * fs));
valid = (settleSamples + 1):size(X, 1);

filterNames = {'Raw'; 'EMA_5Hz'; 'EMA_10Hz'; 'EMA_20Hz'; 'EMA_30Hz'; ...
    'Butter2_5Hz'; 'Butter2_10Hz'; 'Butter2_20Hz'; 'Butter2_30Hz'; ...
    'MovingAverage_5'};
cutoffHz = [NaN; 5; 10; 20; 30; 5; 10; 20; 30; NaN];
nFilters = numel(filterNames);
delayMs = zeros(nFilters, 1);
stdRatio = zeros(nFilters, 6);
filteredStd = zeros(nFilters, 6);

for filterIndex = 1:nFilters
    name = filterNames{filterIndex};
    if strcmp(name, 'Raw')
        Y = X;
        delayMs(filterIndex) = 0;
    elseif strncmp(name, 'EMA_', 4)
        fc = cutoffHz(filterIndex);
        alpha = 1 - exp(-2 * pi * fc / fs);
        Y = filter(alpha, [1, -(1 - alpha)], X);
        delayMs(filterIndex) = 1000 * (1 - alpha) / (alpha * fs);
    elseif strncmp(name, 'Butter2_', 8)
        fc = cutoffHz(filterIndex);
        [b, a] = butterworth2Lowpass(fc, fs);
        Y = filter(b, a, X);
        delaySamplesAtDc = sqrt(2) / (2 * tan(pi * fc / fs));
        delayMs(filterIndex) = 1000 * delaySamplesAtDc / fs;
    else
        windowLength = 5;
        Y = filter(ones(1, windowLength) / windowLength, 1, X);
        delayMs(filterIndex) = 1000 * (windowLength - 1) / (2 * fs);
    end

    Yd = detrendLinear(t(valid), Y(valid, :));
    filteredStd(filterIndex, :) = std(Yd, 0, 1);
    stdRatio(filterIndex, :) = filteredStd(filterIndex, :) ./ noiseStd;
end

medianStdReductionPct = 100 * (1 - median(stdRatio, 2));
filterComparison = table(filterNames, cutoffHz, delayMs, ...
    medianStdReductionPct, ...
    stdRatio(:, 1), stdRatio(:, 2), stdRatio(:, 3), ...
    stdRatio(:, 4), stdRatio(:, 5), stdRatio(:, 6), ...
    'VariableNames', {'filterName', 'cutoffHz', 'estimatedDelayMs', ...
    'medianStdReductionPct', 'FxStdRatio', 'FyStdRatio', 'FzStdRatio', ...
    'TxStdRatio', 'TyStdRatio', 'TzStdRatio'});

report = struct();
report.sampleRateHz = fs;
report.rawStatistics = rawStatistics;
report.detrendedNoiseStd = noiseStd;
report.frequencyHz = frequencyHz;
report.psd = psdValue;
report.bandsHz = bandsHz;
report.bandRms = bandRms;
report.spectrumSummary = spectrumSummary;
report.filterComparison = filterComparison;
end

function Y = detrendLinear(t, X)
Y = zeros(size(X));
for axisIndex = 1:size(X, 2)
    coefficients = polyfit(t, X(:, axisIndex), 1);
    Y(:, axisIndex) = X(:, axisIndex) - polyval(coefficients, t);
end
end

function [b, a] = butterworth2Lowpass(cutoffHz, sampleRateHz)
if cutoffHz <= 0 || cutoffHz >= sampleRateHz / 2
    error('onrobot:analyzeStaticNoise:InvalidCutoff', ...
        'Cutoff frequency must lie between 0 and Nyquist frequency.');
end
k = tan(pi * cutoffHz / sampleRateHz);
normalizer = 1 / (1 + sqrt(2) * k + k^2);
b0 = k^2 * normalizer;
b = [b0, 2 * b0, b0];
a = [1, 2 * (k^2 - 1) * normalizer, ...
    (1 - sqrt(2) * k + k^2) * normalizer];
end

function [powerSpectralDensity, frequencyHz] = welchPsd(X, fs)
n = size(X, 1);
windowLength = min(n, max(128, round(4 * fs)));
if mod(windowLength, 2) ~= 0
    windowLength = windowLength - 1;
end
hop = max(1, floor(windowLength / 2));
nfft = 2^nextpow2(max(1024, 4 * windowLength));
window = 0.5 - 0.5 * cos(2 * pi * (0:windowLength-1).' / ...
    (windowLength - 1));
windowPower = sum(window.^2);
starts = 1:hop:(n - windowLength + 1);
powerSpectralDensity = zeros(nfft / 2 + 1, size(X, 2));

for startIndex = starts
    segment = X(startIndex:startIndex + windowLength - 1, :) .* window;
    spectrum = fft(segment, nfft, 1);
    periodogram = abs(spectrum(1:nfft/2+1, :)).^2 / ...
        (fs * windowPower);
    periodogram(2:end-1, :) = 2 * periodogram(2:end-1, :);
    powerSpectralDensity = powerSpectralDensity + periodogram;
end
powerSpectralDensity = powerSpectralDensity / numel(starts);
frequencyHz = (0:nfft/2).' * fs / nfft;
end
