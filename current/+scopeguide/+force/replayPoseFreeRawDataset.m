function replay = replayPoseFreeRawDataset(cfg, csvFile, options)
%REPLAYPOSEFREERAWDATASET Replay legacy wrench CSV without robot pose.
% Gravity compensation and control eligibility are intentionally disabled.
% This path validates only raw data integrity and dual filter behaviour.

arguments
    cfg (1, 1) struct
    csvFile (1, 1) string
    options.MaximumSamples (1, 1) double = inf
end

validateRcmAdmittanceConfig(cfg);
if ~isfile(csvFile)
    error('scopeguide:force:ReplayFileMissing', ...
        'Replay CSV does not exist: %s', csvFile);
end
data = readtable(csvFile, 'VariableNamingRule', 'preserve');
required = ["Fx_N", "Fy_N", "Fz_N", "Tx_Nm", "Ty_Nm", "Tz_Nm"];
missing = setdiff(required, string(data.Properties.VariableNames));
if ~isempty(missing)
    error('scopeguide:force:ReplayColumnsMissing', ...
        'Raw replay CSV is missing columns: %s.', strjoin(missing, ', '));
end
if ~(options.MaximumSamples == inf || ...
        isfinite(options.MaximumSamples) && ...
        options.MaximumSamples >= 1 && ...
        options.MaximumSamples == fix(options.MaximumSamples))
    error('scopeguide:force:InvalidMaximumSamples', ...
        'MaximumSamples must be a positive integer or Inf.');
end
if isfinite(options.MaximumSamples)
    data = data(1:min(height(data), options.MaximumSamples), :);
end

raw = [data.Fx_N.'; data.Fy_N.'; data.Fz_N.'; ...
    data.Tx_Nm.'; data.Ty_Nm.'; data.Tz_Nm.'];
if isempty(raw) || any(~isfinite(raw), 'all')
    error('scopeguide:force:InvalidRawReplayData', ...
        'Raw replay data must be nonempty and finite.');
end
filters = onrobot.WrenchFilterBank(cfg.sensor.SampleRateHz, ...
    cfg.force.FastCutoffHz, cfg.force.ControlCutoffHz);
count = size(raw, 2);
fast = nan(6, count);
slow = nan(6, count);
for index = 1:count
    [fast(:, index), slow(:, index)] = filters.step(raw(:, index));
end

rawForceNorm = columnNorm(raw(1:3, :));
rawMomentNorm = columnNorm(raw(4:6, :));
replay = struct();
replay.SourceFile = csvFile;
replay.RawWrenchSensor = raw;
replay.FastRawWrenchSensor = fast;
replay.SlowRawWrenchSensor = slow;
replay.CompensationApplied = false;
replay.ControlEligible = false;
replay.MissingRequiredPose = true;
replay.Summary = struct( ...
    'SampleCount', count, ...
    'MaximumRawForceNormN', max(rawForceNorm), ...
    'MaximumRawMomentNormNm', max(rawMomentNorm), ...
    'RawForceStopCount', nnz(rawForceNorm >= ...
        cfg.safety.RawForceStopN), ...
    'RawMomentStopCount', nnz(rawMomentNorm >= ...
        cfg.safety.RawMomentStopNm));
end

function values = columnNorm(matrix)
values = sqrt(sum(matrix.^2, 1));
end
