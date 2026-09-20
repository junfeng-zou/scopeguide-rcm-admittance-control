function [cfgAccepted, acceptance] = ...
        accept_stage08_fixture_result(resultDirectory, options)
%ACCEPT_STAGE08_FIXTURE_RESULT Manual review gate between Stage 8 phases.
% It never connects to hardware.  A reviewed MAT result is converted into
% a new config value; the project default configuration is not overwritten.

arguments
    resultDirectory (1, 1) string
    options.DirectionCorrect (1, 1) logical = false
    options.PhysicalRcmWithinHardBound (1, 1) logical = false
    options.PhysicalRcmMaximumMm (1, 1) double = NaN
    options.AcceptedRcmHardBoundMm (1, 1) double = NaN
    options.PhysicalRcmEvidenceKind (1, 1) string {mustBeMember( ...
        options.PhysicalRcmEvidenceKind, ...
        ["physical_measurement", "user_declared_upper_bound", ...
        "model_proxy"])} = "physical_measurement"
    options.ReleaseStopWasImmediate (1, 1) logical = false
    options.NoUnexpectedMotion (1, 1) logical = false
    options.Confirmation (1, 1) string = ""
end

expected = "ACCEPT_STAGE8_PHASE";
if options.Confirmation ~= expected
    error('scopeguide:stage08:AcceptanceConfirmationRequired', ...
        'Set Confirmation="%s" after manual review.', expected);
end
matPath = fullfile(resultDirectory, 'stage08_worker_result.mat');
if ~isfile(matPath)
    error('scopeguide:stage08:ResultFileMissing', ...
        'Missing Stage 8 worker result: %s', matPath);
end
loaded = load(matPath, 'summary', 'cfg', 'authorization');
summary = loaded.summary;
cfgAccepted = scopeguide.runtime.upgradeRcmAdmittanceConfig(loaded.cfg);
authorization = loaded.authorization;
originalHardRcmBoundMm = 1e3 * cfgAccepted.rcm.HardRadiusM;
hardRcmBoundOverrideApplied = ...
    isfinite(options.AcceptedRcmHardBoundMm);
if hardRcmBoundOverrideApplied
    if options.AcceptedRcmHardBoundMm <= 0
        error('scopeguide:stage08:InvalidAcceptedRcmHardBound', ...
            'AcceptedRcmHardBoundMm must be positive when supplied.');
    end
    cfgAccepted.rcm.HardRadiusM = ...
        1e-3 * options.AcceptedRcmHardBoundMm;
    validateRcmAdmittanceConfig(cfgAccepted);
end
requiredSummary = {'DofMode', 'FaultIdentifier', ...
    'RobotCommandFailureCount', 'CommandChannelHealthyAtExit', ...
    'TargetFeedbackErrorRad', 'RcmErrorM', ...
    'StopLatencySec', 'StopLatencyWithinLimit', ...
    'ServoTimingCandidatePassed', 'ProgrammaticEnableConfirmed', ...
    'SpeedFactorConfirmed', 'ProgrammaticDisableConfirmed', ...
    'DisableFaultIdentifier'};
for index = 1:numel(requiredSummary)
    if ~isfield(summary, requiredSummary{index})
        error('scopeguide:stage08:IncompleteResult', ...
            'Result summary is missing %s.', requiredSummary{index});
    end
end
automatic = struct();
automatic.AuthorizationWasAllowed = authorization.CommissioningAllowed;
automatic.NoFaultIdentifier = strlength(string(summary.FaultIdentifier)) == 0;
automatic.NoCommandFailure = summary.RobotCommandFailureCount == 0;
automatic.CommandChannelHealthy = summary.CommandChannelHealthyAtExit;
automatic.ProgrammaticEnableConfirmed = ...
    summary.ProgrammaticEnableConfirmed;
automatic.SpeedFactorConfirmed = summary.SpeedFactorConfirmed;
automatic.ProgrammaticDisableConfirmed = ...
    summary.ProgrammaticDisableConfirmed;
automatic.NoDisableFault = ...
    strlength(string(summary.DisableFaultIdentifier)) == 0;
automatic.TrackingWithinLimit = ...
    summary.TargetFeedbackErrorRad.Maximum <= ...
    cfgAccepted.stage08.MaximumTargetFeedbackErrorRad;
automatic.ModelRcmWithinHardBound = ...
    summary.RcmErrorM.Maximum <= cfgAccepted.rcm.HardRadiusM;
automatic.MeasuredStopLatencyWithinLimit = ...
    summary.StopLatencyWithinLimit;
manual = struct( ...
    'DirectionCorrect', options.DirectionCorrect, ...
    'PhysicalRcmWithinHardBound', options.PhysicalRcmWithinHardBound, ...
    'PhysicalRcmMeasurementFinite', ...
        isfinite(options.PhysicalRcmMaximumMm) && ...
        options.PhysicalRcmMaximumMm >= 0, ...
    'PhysicalRcmMeasurementWithinHardBound', ...
        isfinite(options.PhysicalRcmMaximumMm) && ...
        options.PhysicalRcmMaximumMm <= 1e3 * cfgAccepted.rcm.HardRadiusM, ...
    'ReleaseStopWasImmediate', options.ReleaseStopWasImmediate, ...
    'NoUnexpectedMotion', options.NoUnexpectedMotion);
automaticNames = fieldnames(automatic);
manualNames = fieldnames(manual);
automaticPassed = all(cellfun(@(name) automatic.(name), automaticNames));
manualPassed = all(cellfun(@(name) manual.(name), manualNames));
if ~automaticPassed || ~manualPassed
    failed = [string(automaticNames(~cellfun( ...
        @(name) automatic.(name), automaticNames))); ...
        string(manualNames(~cellfun(@(name) manual.(name), manualNames)))];
    error('scopeguide:stage08:ResultAcceptanceBlocked', ...
        'Stage 8 result cannot be accepted; failed: %s.', ...
        strjoin(failed, ', '));
end
mode = string(summary.DofMode);
completed = string(cfgAccepted.stage08.CompletedDofModes(:));
if ~any(completed == mode)
    completed(end + 1, 1) = mode;
end
cfgAccepted.stage08.CompletedDofModes = completed;
if mode == "insertion_only"
    if ~summary.ServoTimingCandidatePassed
        error('scopeguide:stage08:ServoTimingCandidateFailed', ...
            'Insertion result did not pass the ServoJ timing candidate gate.');
    end
    cfgAccepted.robot.ServoTimingVerified = true;
end
validateRcmAdmittanceConfig(cfgAccepted);
acceptance = struct();
acceptance.Accepted = true;
acceptance.AcceptedAtLocal = string(datetime('now', ...
    'TimeZone', 'Asia/Shanghai', ...
    'Format', 'yyyy-MM-dd HH:mm:ss Z'));
acceptance.ResultDirectory = resultDirectory;
acceptance.DofMode = mode;
acceptance.AutomaticChecks = automatic;
acceptance.ManualChecks = manual;
acceptance.PhysicalRcmMaximumMm = options.PhysicalRcmMaximumMm;
acceptance.PhysicalRcmEvidenceKind = ...
    options.PhysicalRcmEvidenceKind;
acceptance.HardRcmBoundOverrideApplied = ...
    hardRcmBoundOverrideApplied;
acceptance.OriginalHardRcmBoundMm = originalHardRcmBoundMm;
acceptance.AcceptedHardRcmBoundMm = ...
    1e3 * cfgAccepted.rcm.HardRadiusM;
acceptance.CompletedDofModes = completed;
acceptance.ServoTimingVerified = cfgAccepted.robot.ServoTimingVerified;
acceptance.KeyboardInputWasNotPhysicalDeadman = true;
save(fullfile(resultDirectory, 'stage08_accepted_config.mat'), ...
    'cfgAccepted', 'acceptance');
writeJson(fullfile(resultDirectory, 'stage08_acceptance.json'), acceptance);
fprintf('Stage 8 phase accepted: %s. Completed: %s\n', ...
    mode, strjoin(completed, ', '));
fprintf('Accepted config: %s\n', ...
    fullfile(resultDirectory, 'stage08_accepted_config.mat'));
end

function writeJson(path, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end
[fileId, message] = fopen(path, 'w');
if fileId < 0
    error('scopeguide:stage08:CannotWriteJson', ...
        'Cannot open %s: %s', path, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end
