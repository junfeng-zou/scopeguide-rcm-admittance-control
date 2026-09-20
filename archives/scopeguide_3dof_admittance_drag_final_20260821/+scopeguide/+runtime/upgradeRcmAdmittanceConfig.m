function cfg = upgradeRcmAdmittanceConfig(cfg)
%UPGRADERCMADMITTANCECONFIG Add explicit sensor-origin geometry to legacy cfg.
% This is a narrow, deterministic migration for configurations saved before
% the 2026-08-16 RCM wrench-shift, ServoJ step-guard and Stage 8 pivot-speed
% updates.  It does not change wrench thresholds, RCM coordinates or
% acceptance records.

arguments
    cfg (1, 1) struct
end

if ~isfield(cfg, 'tool') || ~isstruct(cfg.tool)
    error('scopeguide:config:MissingToolConfiguration', ...
        'Legacy configuration is missing cfg.tool.');
end
if ~isfield(cfg.tool, 'TFlangeSensor')
    cfg.tool.TFlangeSensor = [ ...
       -1  0  0  0; ...
        0 -1  0  0; ...
        0  0  1  0.020; ...
        0  0  0  1];
end
if ~isfield(cfg.tool, 'SensorOriginSource')
    cfg.tool.SensorOriginSource = ...
        "legacy_config_migrated_to_user_estimate_flange_plus_z_20mm";
end
if ~isfield(cfg, 'control') || ~isstruct(cfg.control)
    error('scopeguide:config:MissingControlConfiguration', ...
        'Legacy configuration is missing cfg.control.');
end
if ~isfield(cfg.control, 'RelativePivotRecoveryToleranceRad')
    cfg.control.RelativePivotRecoveryToleranceRad = deg2rad(0.3);
end
if ~isfield(cfg.control, 'RelativeInsertionRecoveryToleranceM')
    cfg.control.RelativeInsertionRecoveryToleranceM = 0.0005;
end
if isfield(cfg, 'stage08') && isstruct(cfg.stage08)
    maximumMultiplier = 2.50;
    targetMultiplier = 8 / 3;
    admittanceSpeedScale = 1.60;
    currentRevision = 3;
    hadMaximumMultiplier = isfield(cfg.stage08, ...
        'PivotMaximumSpeedMultiplier');
    hadTargetMultiplier = isfield(cfg.stage08, ...
        'PivotTargetSpeedMultiplier');
    hasRetuneMarker = isfield(cfg.stage08, ...
        'PivotSpeedRetuneApplied');
    hasRetuneRevision = isfield(cfg.stage08, ...
        'PivotSpeedRetuneRevision');
    profileWasApplied = isfield(cfg.stage08, 'ProfileApplied') && ...
        logical(cfg.stage08.ProfileApplied);
    revisionIsCurrent = hasRetuneRevision && ...
        double(cfg.stage08.PivotSpeedRetuneRevision) == currentRevision;
    if ~revisionIsCurrent
        previousRetuneApplied = profileWasApplied && hasRetuneMarker && ...
            logical(cfg.stage08.PivotSpeedRetuneApplied) && ...
            hadMaximumMultiplier && hadTargetMultiplier;
        if profileWasApplied
            speedScale = double(cfg.stage08.SpeedScale);
            if previousRetuneApplied
                appliedMaximumMultiplier = double( ...
                    cfg.stage08.PivotMaximumSpeedMultiplier);
                appliedTargetMultiplier = double( ...
                    cfg.stage08.PivotTargetSpeedMultiplier);
            else
                appliedMaximumMultiplier = 1;
                appliedTargetMultiplier = 1;
            end
            basePivotMaximum = cfg.control.PivotMaxRadSec / ...
                (speedScale * appliedMaximumMultiplier);
            basePivotTarget = ...
                cfg.admittance.Pivot.TargetSteadySpeedRadSec / ...
                (speedScale * appliedTargetMultiplier);
            cfg.control.PivotMaxRadSec = basePivotMaximum * ...
                speedScale * maximumMultiplier;
            cfg.admittance.Pivot.TargetSteadySpeedRadSec = ...
                basePivotTarget * admittanceSpeedScale * ...
                targetMultiplier;
        end
        cfg.stage08.PivotAdmittanceSpeedScale = admittanceSpeedScale;
        cfg.stage08.PivotMaximumSpeedMultiplier = maximumMultiplier;
        cfg.stage08.PivotTargetSpeedMultiplier = targetMultiplier;
        cfg.stage08.PivotSpeedRetuneApplied = profileWasApplied;
        cfg.stage08.PivotSpeedRetuneRevision = currentRevision;
    elseif ~isfield(cfg.stage08, 'PivotAdmittanceSpeedScale')
        cfg.stage08.PivotAdmittanceSpeedScale = admittanceSpeedScale;
    end
    if ~hasRetuneMarker
        cfg.stage08.PivotSpeedRetuneApplied = profileWasApplied;
    end
end
if isfield(cfg, 'stage08') && isstruct(cfg.stage08) && ...
        isfield(cfg.stage08, 'MaximumServoTargetStepRad')
    oldStepRad = double(cfg.stage08.MaximumServoTargetStepRad);
    previousStepRad = deg2rad([0.10, 0.15]);
    isPreviousStep = any(abs(oldStepRad - previousStepRad) <= ...
        100 * eps(max(previousStepRad)));
    if isPreviousStep
        % Migrate both historical guards so saved Stage 8 accepted configs
        % use the explicitly requested 0.30 deg per-command bound.  At the
        % 20 Hz Stage 8 loop this corresponds to a nominal 6 deg/s target
        % increment boundary; it remains a fail-closed discontinuity check,
        % not a velocity limiter.
        cfg.stage08.MaximumServoTargetStepRad = deg2rad(0.30);
    end
end
end
