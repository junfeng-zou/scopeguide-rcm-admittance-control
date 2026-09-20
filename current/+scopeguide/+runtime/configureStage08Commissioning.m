function cfg = configureStage08Commissioning(cfg, dofMode)
%CONFIGURESTAGE08COMMISSIONING Apply the conservative fixture profile.

arguments
    cfg (1, 1) struct
    dofMode (1, 1) string
end

cfg = scopeguide.runtime.upgradeRcmAdmittanceConfig(cfg);
cfg.runtime.Mode = "fixture_motion";
% The user's CR5 has shown intermittent motion-port response timeouts at
% the previous 30 Hz commissioning rate.  Stage 8 therefore uses a
% dedicated 20 Hz command loop; Stage 7 and the offline defaults remain at
% 30 Hz.  All dynamics continue to use measured dt.
cfg.runtime.ControlRateHz = 20;
cfg.runtime.NominalDtSec = 1 / cfg.runtime.ControlRateHz;
cfg.robot.EnableMotion = true;
cfg.robot.DryRun = false;
cfg.stage08.DofMode = dofMode;
scopeguide.control.stage08DofMask(dofMode);
if ~cfg.stage08.ProfileApplied
    scale = cfg.stage08.SpeedScale;
    cfg.control.PivotMaxRadSec = cfg.control.PivotMaxRadSec * scale * ...
        cfg.stage08.PivotMaximumSpeedMultiplier;
    cfg.control.InsertionMaxMSec = cfg.control.InsertionMaxMSec * scale;
    cfg.control.MaximumJointCommandRadSec = ...
        cfg.control.MaximumJointCommandRadSec * scale;
    cfg.control.MaximumJointAccelerationRadSec2 = ...
        cfg.control.MaximumJointAccelerationRadSec2 * scale;
    cfg.admittance.Pivot.TargetSteadySpeedRadSec = ...
        cfg.admittance.Pivot.TargetSteadySpeedRadSec * ...
        cfg.stage08.PivotAdmittanceSpeedScale * ...
        cfg.stage08.PivotTargetSpeedMultiplier;
    cfg.admittance.Pivot.MaximumAccelerationRadSec2 = ...
        cfg.admittance.Pivot.MaximumAccelerationRadSec2 * scale;
    cfg.admittance.Pivot.MaximumTipSpeedMSec = ...
        cfg.admittance.Pivot.MaximumTipSpeedMSec * scale;
    cfg.admittance.Insertion.TargetSteadySpeedMSec = ...
        cfg.admittance.Insertion.TargetSteadySpeedMSec * scale;
    cfg.admittance.Insertion.MaximumAccelerationMSec2 = ...
        cfg.admittance.Insertion.MaximumAccelerationMSec2 * scale;
    cfg.stage08.PivotSpeedRetuneApplied = true;
    cfg.stage08.ProfileApplied = true;
end
validateRcmAdmittanceConfig(cfg);
end
