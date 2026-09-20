function cfg = configureCartesianAdmittanceDrag(cfg, dofMode)
%CONFIGURECARTESIANADMITTANCEDRAG Select physical no-RCM drag operation.

arguments
    cfg (1, 1) struct
    dofMode (1, 1) string
end

cfg = scopeguide.runtime.upgradeRcmAdmittanceConfig(cfg);
if ~isfield(cfg, 'cartesianDrag')
    error('scopeguide:cartesianDrag:MissingConfiguration', ...
        'Use current_cartesian_admittance_drag_config first.');
end
cfg.cartesianDrag.DofMode = dofMode;
scopeguide.control.cartesianDragDofMask(dofMode);
cfg.runtime.Mode = "fixture_motion";
cfg.runtime.ControlRateHz = 20;
cfg.runtime.NominalDtSec = 1 / cfg.runtime.ControlRateHz;
cfg.robot.EnableMotion = true;
cfg.robot.DryRun = false;
validateCartesianAdmittanceDragConfig(cfg);
end
