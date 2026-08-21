function parameters = deriveForceOnlyAdmittanceParameters(cfg)
%DERIVEFORCEONLYADMITTANCEPARAMETERS Explainable Stage 5 M/D parameters.

validateRcmAdmittanceConfig(cfg);
pivot = cfg.admittance.Pivot;
insertion = cfg.admittance.Insertion;

pivotDesignEffortNm = pivot.DesignForceN * pivot.NominalLeverArmM;
pivotDamping = pivotDesignEffortNm / ...
    pivot.TargetSteadySpeedRadSec;
pivotMass = pivotDamping * pivot.TimeConstantSec;
insertionDamping = insertion.DesignForceN / ...
    insertion.TargetSteadySpeedMSec;
insertionMass = insertionDamping * insertion.TimeConstantSec;

parameters = struct();
parameters.Damping = [pivotDamping; pivotDamping; insertionDamping];
parameters.VirtualMass = [pivotMass; pivotMass; insertionMass];
parameters.DesignGeneralizedEffort = [pivotDesignEffortNm; ...
    pivotDesignEffortNm; insertion.DesignForceN];
parameters.TargetSteadyVelocity = [pivot.TargetSteadySpeedRadSec; ...
    pivot.TargetSteadySpeedRadSec; insertion.TargetSteadySpeedMSec];
parameters.TimeConstantSec = [pivot.TimeConstantSec; ...
    pivot.TimeConstantSec; insertion.TimeConstantSec];
parameters.MaximumVelocity = [cfg.control.PivotMaxRadSec; ...
    cfg.control.PivotMaxRadSec; cfg.control.InsertionMaxMSec];
parameters.MaximumAcceleration = [pivot.MaximumAccelerationRadSec2; ...
    pivot.MaximumAccelerationRadSec2; ...
    insertion.MaximumAccelerationMSec2];
parameters.MaximumRelativePosition = [cfg.control.RelativePivotLimitRad; ...
    cfg.control.RelativePivotLimitRad; ...
    cfg.control.RelativeInsertionLimitM];
parameters.VelocityZeroTolerance = ...
    double(cfg.admittance.VelocityZeroTolerance(:));
parameters.MaximumPivotTipSpeedMSec = pivot.MaximumTipSpeedMSec;
parameters.RollEnabled = false;
end
