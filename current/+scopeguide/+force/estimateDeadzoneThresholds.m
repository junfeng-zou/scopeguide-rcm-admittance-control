function report = estimateDeadzoneThresholds(signals, options)
%ESTIMATEDEADZONETHRESHOLDS Estimate radial force/moment deadzones.
% Thresholds are estimated from the exact slow branch used by the online
% controller. External (compensated but unfiltered) wrench statistics are
% retained to distinguish compensation residuals from filtered noise.

arguments
    signals table
    options.Percentile (1, 1) double = 99.5
    options.ForceMarginN (1, 1) double = 0.15
    options.MomentMarginNm (1, 1) double = 0.015
    options.ForceRoundingN (1, 1) double = 0.05
    options.MomentRoundingNm (1, 1) double = 0.005
    options.CurrentForceDeadzoneN (1, 1) double = 1.3
    options.CurrentMomentDeadzoneNm (1, 1) double = 0.18
    options.MinimumEligibleRatio (1, 1) double = 0.90
    options.MinimumEligibleSamples (1, 1) double = 100
    options.MaximumContinuityGapSec (1, 1) double = 0.05
end

validateOptions(options);
required = ["PoseIndex", "PoseLabel", "SampleTimeSec", ...
    "AnalysisEligible", ...
    "ExternalFx", "ExternalFy", "ExternalFz", ...
    "ExternalTx", "ExternalTy", "ExternalTz", ...
    "SlowFx", "SlowFy", "SlowFz", ...
    "SlowTx", "SlowTy", "SlowTz"];
missing = setdiff(required, string(signals.Properties.VariableNames));
if ~isempty(missing)
    error('scopeguide:force:DeadzoneIdentificationMissingVariable', ...
        'Signals table is missing: %s.', strjoin(missing, ', '));
end
if isempty(signals)
    error('scopeguide:force:DeadzoneIdentificationEmpty', ...
        'Signals table must not be empty.');
end

poseIndex = double(signals.PoseIndex(:));
sampleTimeSec = double(signals.SampleTimeSec(:));
analysisEligible = logical(signals.AnalysisEligible(:));
externalForce = tableColumns(signals, ...
    ["ExternalFx", "ExternalFy", "ExternalFz"]);
externalMoment = tableColumns(signals, ...
    ["ExternalTx", "ExternalTy", "ExternalTz"]);
slowForce = tableColumns(signals, ["SlowFx", "SlowFy", "SlowFz"]);
slowMoment = tableColumns(signals, ["SlowTx", "SlowTy", "SlowTz"]);
finiteRows = all(isfinite([sampleTimeSec, externalForce, ...
    externalMoment, slowForce, slowMoment]), 2);
analysisEligible = analysisEligible & finiteRows;

poses = unique(poseIndex(isfinite(poseIndex)), 'stable');
poseCount = numel(poses);
if poseCount < 1
    error('scopeguide:force:DeadzoneIdentificationNoPose', ...
        'No finite pose indices are available.');
end

poseLabels = strings(poseCount, 1);
totalSamples = zeros(poseCount, 1);
eligibleSamples = zeros(poseCount, 1);
eligibleRatio = zeros(poseCount, 1);
eligibleDurationSec = zeros(poseCount, 1);
externalForceMean = zeros(poseCount, 3);
externalForceStd = zeros(poseCount, 3);
externalMomentMean = zeros(poseCount, 3);
externalMomentStd = zeros(poseCount, 3);
slowForceMean = zeros(poseCount, 3);
slowForceStd = zeros(poseCount, 3);
slowMomentMean = zeros(poseCount, 3);
slowMomentStd = zeros(poseCount, 3);
forceNormMedian = zeros(poseCount, 1);
forceNormP95 = zeros(poseCount, 1);
forceNormP99 = zeros(poseCount, 1);
forceNormTargetPercentile = zeros(poseCount, 1);
forceNormMaximum = zeros(poseCount, 1);
momentNormMedian = zeros(poseCount, 1);
momentNormP95 = zeros(poseCount, 1);
momentNormP99 = zeros(poseCount, 1);
momentNormTargetPercentile = zeros(poseCount, 1);
momentNormMaximum = zeros(poseCount, 1);

for index = 1:poseCount
    pose = poses(index);
    rows = poseIndex == pose;
    eligible = rows & analysisEligible;
    totalSamples(index) = nnz(rows);
    eligibleSamples(index) = nnz(eligible);
    eligibleRatio(index) = eligibleSamples(index) / ...
        max(totalSamples(index), 1);
    labels = string(signals.PoseLabel(rows));
    if isempty(labels)
        poseLabels(index) = "pose_" + pose;
    else
        poseLabels(index) = labels(1);
    end
    requireEligibleSamples(eligibleSamples(index), pose, options);

    t = sampleTimeSec(eligible);
    eligibleDurationSec(index) = max(t) - min(t);
    externalForceMean(index, :) = mean(externalForce(eligible, :), 1);
    externalForceStd(index, :) = std(externalForce(eligible, :), 0, 1);
    externalMomentMean(index, :) = mean(externalMoment(eligible, :), 1);
    externalMomentStd(index, :) = std(externalMoment(eligible, :), 0, 1);
    slowForceMean(index, :) = mean(slowForce(eligible, :), 1);
    slowForceStd(index, :) = std(slowForce(eligible, :), 0, 1);
    slowMomentMean(index, :) = mean(slowMoment(eligible, :), 1);
    slowMomentStd(index, :) = std(slowMoment(eligible, :), 0, 1);

    forceNorm = vecnorm(slowForce(eligible, :), 2, 2);
    momentNorm = vecnorm(slowMoment(eligible, :), 2, 2);
    forceNormMedian(index) = percentile(forceNorm, 50);
    forceNormP95(index) = percentile(forceNorm, 95);
    forceNormP99(index) = percentile(forceNorm, 99);
    forceNormTargetPercentile(index) = ...
        percentile(forceNorm, options.Percentile);
    forceNormMaximum(index) = max(forceNorm);
    momentNormMedian(index) = percentile(momentNorm, 50);
    momentNormP95(index) = percentile(momentNorm, 95);
    momentNormP99(index) = percentile(momentNorm, 99);
    momentNormTargetPercentile(index) = ...
        percentile(momentNorm, options.Percentile);
    momentNormMaximum(index) = max(momentNorm);
end

candidateForceUnrounded = max(forceNormTargetPercentile) + ...
    options.ForceMarginN;
candidateMomentUnrounded = max(momentNormTargetPercentile) + ...
    options.MomentMarginNm;
candidateForce = roundUp(candidateForceUnrounded, options.ForceRoundingN);
candidateMoment = roundUp(candidateMomentUnrounded, ...
    options.MomentRoundingNm);

poseStatistics = table(poses, poseLabels, totalSamples, eligibleSamples, ...
    eligibleRatio, eligibleDurationSec, ...
    externalForceMean(:, 1), externalForceMean(:, 2), ...
    externalForceMean(:, 3), ...
    externalForceStd(:, 1), externalForceStd(:, 2), ...
    externalForceStd(:, 3), ...
    externalMomentMean(:, 1), externalMomentMean(:, 2), ...
    externalMomentMean(:, 3), ...
    externalMomentStd(:, 1), externalMomentStd(:, 2), ...
    externalMomentStd(:, 3), ...
    slowForceMean(:, 1), slowForceMean(:, 2), slowForceMean(:, 3), ...
    slowForceStd(:, 1), slowForceStd(:, 2), slowForceStd(:, 3), ...
    slowMomentMean(:, 1), slowMomentMean(:, 2), slowMomentMean(:, 3), ...
    slowMomentStd(:, 1), slowMomentStd(:, 2), slowMomentStd(:, 3), ...
    forceNormMedian, forceNormP95, forceNormP99, ...
    forceNormTargetPercentile, forceNormMaximum, ...
    momentNormMedian, momentNormP95, momentNormP99, ...
    momentNormTargetPercentile, momentNormMaximum, ...
    'VariableNames', {'PoseIndex', 'PoseLabel', 'TotalSamples', ...
    'EligibleSamples', 'EligibleRatio', 'EligibleDurationSec', ...
    'ExternalFxMean', 'ExternalFyMean', 'ExternalFzMean', ...
    'ExternalFxStd', 'ExternalFyStd', 'ExternalFzStd', ...
    'ExternalTxMean', 'ExternalTyMean', 'ExternalTzMean', ...
    'ExternalTxStd', 'ExternalTyStd', 'ExternalTzStd', ...
    'SlowFxMean', 'SlowFyMean', 'SlowFzMean', ...
    'SlowFxStd', 'SlowFyStd', 'SlowFzStd', ...
    'SlowTxMean', 'SlowTyMean', 'SlowTzMean', ...
    'SlowTxStd', 'SlowTyStd', 'SlowTzStd', ...
    'ForceNormMedian', 'ForceNormP95', 'ForceNormP99', ...
    'ForceNormTargetPercentile', 'ForceNormMaximum', ...
    'MomentNormMedian', 'MomentNormP95', 'MomentNormP99', ...
    'MomentNormTargetPercentile', 'MomentNormMaximum'});

excursions = buildExcursionTable(poseIndex, sampleTimeSec, ...
    analysisEligible, slowForce, slowMoment, poses, ...
    options.CurrentForceDeadzoneN, options.CurrentMomentDeadzoneNm, ...
    candidateForce, candidateMoment, options.MaximumContinuityGapSec);
candidateForceRows = excursions.Signal == "force" & ...
    excursions.ThresholdKind == "candidate";
maximumCandidateForceExcursionSec = ...
    max(excursions.MaxDurationSec(candidateForceRows), [], 'omitnan');
if isempty(maximumCandidateForceExcursionSec) || ...
        ~isfinite(maximumCandidateForceExcursionSec)
    maximumCandidateForceExcursionSec = 0;
end
suggestedHoldSec = min(0.20, max(0.05, ...
    roundUp(maximumCandidateForceExcursionSec + 0.02, 0.01)));

report = struct();
report.Status = "valid";
if any(eligibleRatio < options.MinimumEligibleRatio)
    report.Status = "insufficient_eligible_ratio";
end
report.Valid = report.Status == "valid";
report.Percentile = options.Percentile;
report.ForceMarginN = options.ForceMarginN;
report.MomentMarginNm = options.MomentMarginNm;
report.CurrentForceDeadzoneN = options.CurrentForceDeadzoneN;
report.CurrentMomentDeadzoneNm = options.CurrentMomentDeadzoneNm;
report.CandidateForceDeadzoneN = candidateForce;
report.CandidateMomentDeadzoneNm = candidateMoment;
report.CandidateForceBeforeRoundingN = candidateForceUnrounded;
report.CandidateMomentBeforeRoundingNm = candidateMomentUnrounded;
report.SuggestedForceReleaseThresholdN = 0.8 * candidateForce;
report.SuggestedMomentReleaseThresholdNm = 0.8 * candidateMoment;
report.SuggestedActivationHoldSec = suggestedHoldSec;
report.MaximumCandidateForceExcursionSec = ...
    maximumCandidateForceExcursionSec;
report.MaximumPoseMeanSlowForceNormN = ...
    max(vecnorm(slowForceMean, 2, 2));
report.MaximumPoseMeanSlowMomentNormNm = ...
    max(vecnorm(slowMomentMean, 2, 2));
report.PoseStatistics = poseStatistics;
report.Excursions = excursions;
end

function validateOptions(options)
if options.Percentile <= 50 || options.Percentile >= 100
    error('scopeguide:force:InvalidDeadzonePercentile', ...
        'Percentile must lie strictly between 50 and 100.');
end
positiveNames = {'ForceRoundingN', 'MomentRoundingNm', ...
    'CurrentForceDeadzoneN', 'CurrentMomentDeadzoneNm', ...
    'MinimumEligibleSamples', 'MaximumContinuityGapSec'};
for index = 1:numel(positiveNames)
    value = options.(positiveNames{index});
    if ~isscalar(value) || ~isfinite(value) || value <= 0
        error('scopeguide:force:InvalidDeadzoneOption', ...
            '%s must be finite and positive.', positiveNames{index});
    end
end
if options.ForceMarginN < 0 || options.MomentMarginNm < 0 || ...
        options.MinimumEligibleRatio <= 0 || ...
        options.MinimumEligibleRatio > 1
    error('scopeguide:force:InvalidDeadzoneOption', ...
        'Margins must be nonnegative and eligible ratio must lie in (0,1].');
end
end

function values = tableColumns(input, names)
values = zeros(height(input), numel(names));
for index = 1:numel(names)
    values(:, index) = double(input.(names(index)));
end
end

function requireEligibleSamples(count, pose, options)
if count < options.MinimumEligibleSamples
    error('scopeguide:force:DeadzoneIdentificationTooFewSamples', ...
        ['Pose %d has only %d eligible samples; at least %d are ' ...
         'required.'], pose, count, options.MinimumEligibleSamples);
end
end

function value = percentile(data, percentage)
values = sort(double(data(:)));
if isempty(values)
    value = NaN;
    return;
end
position = 1 + (numel(values) - 1) * percentage / 100;
lowerIndex = floor(position);
upperIndex = ceil(position);
if lowerIndex == upperIndex
    value = values(lowerIndex);
else
    weight = position - lowerIndex;
    value = values(lowerIndex) * (1 - weight) + ...
        values(upperIndex) * weight;
end
end

function value = roundUp(value, increment)
scaled = value / increment;
tolerance = 1e-10 * max(abs(scaled), 1);
value = ceil(scaled - tolerance) * increment;
end

function output = buildExcursionTable(poseIndex, sampleTimeSec, eligible, ...
        slowForce, slowMoment, poses, currentForce, currentMoment, ...
        candidateForce, candidateMoment, maximumGapSec)
rowCount = numel(poses) * 4;
poseColumn = zeros(rowCount, 1);
signalColumn = strings(rowCount, 1);
thresholdKindColumn = strings(rowCount, 1);
thresholdColumn = zeros(rowCount, 1);
countColumn = zeros(rowCount, 1);
maximumColumn = zeros(rowCount, 1);
p95Column = zeros(rowCount, 1);
totalColumn = zeros(rowCount, 1);
row = 0;
for poseOffset = 1:numel(poses)
    pose = poses(poseOffset);
    poseRows = poseIndex == pose;
    t = sampleTimeSec(poseRows);
    poseEligible = eligible(poseRows);
    forceNorm = vecnorm(slowForce(poseRows, :), 2, 2);
    momentNorm = vecnorm(slowMoment(poseRows, :), 2, 2);
    definitions = { ...
        "force", "current", currentForce, forceNorm; ...
        "force", "candidate", candidateForce, forceNorm; ...
        "moment", "current", currentMoment, momentNorm; ...
        "moment", "candidate", candidateMoment, momentNorm};
    for definition = 1:size(definitions, 1)
        row = row + 1;
        threshold = definitions{definition, 3};
        durations = excursionDurations(t, poseEligible, ...
            definitions{definition, 4}, threshold, maximumGapSec);
        poseColumn(row) = pose;
        signalColumn(row) = definitions{definition, 1};
        thresholdKindColumn(row) = definitions{definition, 2};
        thresholdColumn(row) = threshold;
        countColumn(row) = numel(durations);
        if isempty(durations)
            maximumColumn(row) = 0;
            p95Column(row) = 0;
            totalColumn(row) = 0;
        else
            maximumColumn(row) = max(durations);
            p95Column(row) = percentile(durations, 95);
            totalColumn(row) = sum(durations);
        end
    end
end
output = table(poseColumn, signalColumn, thresholdKindColumn, ...
    thresholdColumn, countColumn, maximumColumn, p95Column, totalColumn, ...
    'VariableNames', {'PoseIndex', 'Signal', 'ThresholdKind', ...
    'Threshold', 'ExcursionCount', 'MaxDurationSec', ...
    'P95DurationSec', 'TotalDurationSec'});
end

function durations = excursionDurations(time, eligible, magnitude, ...
        threshold, maximumGapSec)
time = double(time(:));
eligible = logical(eligible(:));
magnitude = double(magnitude(:));
above = eligible & isfinite(time) & isfinite(magnitude) & ...
    magnitude > threshold;
validTime = time(eligible & isfinite(time));
timeDifferences = diff(validTime);
timeDifferences = timeDifferences(timeDifferences > 0 & ...
    timeDifferences <= maximumGapSec);
if isempty(timeDifferences)
    nominalStep = 0;
else
    nominalStep = median(timeDifferences);
end
durations = zeros(0, 1);
runStart = NaN;
previousTime = NaN;
for index = 1:numel(time)
    if above(index) && (isnan(previousTime) || ...
            time(index) - previousTime <= maximumGapSec)
        if isnan(runStart)
            runStart = time(index);
        end
        previousTime = time(index);
    else
        if ~isnan(runStart)
            durations(end + 1, 1) = ...
                max(nominalStep, previousTime - runStart + nominalStep); %#ok<AGROW>
        end
        if above(index)
            runStart = time(index);
            previousTime = time(index);
        else
            runStart = NaN;
            previousTime = NaN;
        end
    end
end
if ~isnan(runStart)
    durations(end + 1, 1) = ...
        max(nominalStep, previousTime - runStart + nominalStep);
end
end
