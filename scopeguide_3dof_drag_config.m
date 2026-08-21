function [cfg, profile] = scopeguide_3dof_drag_config(options)
%SCOPEGUIDE_3DOF_DRAG_CONFIG Final pivot + insertion drag parameters.
% The packaged base configuration supplies the validated hardware,
% calibration and prerequisite records. The block marked USER-EDITABLE
% contains the final, already-scaled three-DOF drag values.
%
% The returned generalized order is [pivot1; pivot2; insertion].  Roll is
% kept disabled. run_scopeguide_3dof_drag still requires all physical-motion
% confirmation arguments and explicitly selecting DofMode="full_3dof".

arguments
    options.SourceAcceptedConfigFile (1, 1) string = ""
end

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'config'));
sourceFile = options.SourceAcceptedConfigFile;
if strlength(sourceFile) == 0
    sourceFile = fullfile(projectRoot, 'resources', ...
        'validated_base_config.mat');
end
if ~isfile(sourceFile)
    error('scopeguide:stage08:AcceptedConfigNotFound', ...
        'Accepted dual-pivot configuration was not found: %s', ...
        sourceFile);
end

loaded = load(sourceFile, 'cfgAccepted');
if ~isfield(loaded, 'cfgAccepted') || ~isstruct(loaded.cfgAccepted)
    error('scopeguide:stage08:InvalidAcceptedConfigFile', ...
        'File does not contain a scalar cfgAccepted structure: %s', ...
        sourceFile);
end

cfg = scopeguide.runtime.upgradeRcmAdmittanceConfig( ...
    loaded.cfgAccepted);
% The accepted MAT file was created in the working project. Redirect every
% runtime path owned by this package to the archive root so the process
% worker neither imports code from nor writes results into that project.
cfg.meta.ProjectRoot = projectRoot;
cfg.meta.UserAttestationRecord = "";
cfg.logging.ResultsRoot = fullfile(projectRoot, 'results');
calibrationFile = fullfile(projectRoot, 'resources', ...
    'force_calibration.json');
if ~isfile(calibrationFile)
    error('scopeguide:drag:CalibrationFileNotFound', ...
        'Packaged force calibration was not found: %s', ...
        calibrationFile);
end
cfg.force.CalibrationFile = calibrationFile;
completedModes = string(cfg.stage08.CompletedDofModes(:));
if ~any(completedModes == "dual_pivot")
    error('scopeguide:stage08:Full3DofPrerequisiteMissing', ...
        ['The editable full-3DOF profile requires an accepted ' ...
         'dual_pivot prerequisite.']);
end

% =====================================================================
% USER-EDITABLE FINAL VALUES
% These are final values, not unscaled defaults. ProfileApplied
% remains true so the internal configuration layer will not multiply them
% a second time. Changing SpeedScale alone therefore changes only the
% recorded/displayed scale; edit the explicit target/limit below when
% changing the actual motion response.
% =====================================================================
profile = struct();
profile.Name = "scopeguide_3dof_admittance_drag_final_20260821";
profile.SourceAcceptedConfigFile = sourceFile;

% Runtime and ServoJ.
profile.ControlRateHz = 20.0;
profile.SpeedScale = 1.20;
profile.ServoJLookaheadTime = 60.0;
profile.ServoJGain = 400.0;
profile.SpeedFactorPercent = 20;
profile.EnablePayloadKg = 1.3;

% Pivot: values used in the 2026-08-17 full-3DOF runs.
profile.PivotDesignForceN = 5.0;
profile.PivotNominalLeverArmM = 0.15;
profile.PivotTargetSteadySpeedRadSec = ...
    deg2rad(2.133333333333333);
profile.PivotTimeConstantSec = 0.12;
profile.PivotMaximumAccelerationRadSec2 = deg2rad(6.0);
profile.PivotMaximumTipSpeedMSec = 0.006;
profile.PivotMaximumSpeedRadSec = deg2rad(3.0);
profile.RelativePivotLimitRad = deg2rad(7.5);

% Insertion.
profile.InsertionDesignForceN = 5.0;
profile.InsertionTargetSteadySpeedMSec = 0.006;
profile.InsertionTimeConstantSec = 0.30;
profile.InsertionMaximumAccelerationMSec2 = 0.012;
profile.InsertionMaximumSpeedMSec = 0.008;
profile.RelativeInsertionLimitM = 0.015;

% Joint-command and RCM limits.
profile.MaximumJointCommandRadSec = deg2rad(6.0);
profile.MaximumJointAccelerationRadSec2 = deg2rad(24.0);
profile.MaximumServoTargetStepRad = deg2rad(0.30);
profile.MaximumTargetFeedbackErrorRad = deg2rad(0.50);
profile.RcmMode = "soft";
profile.RcmSoftRadiusM = 0.0005;
profile.RcmHardRadiusM = 0.002;

% Force processing and the currently selected stop thresholds.
profile.ForceDeadzoneN = 1.3;
profile.MomentDeadzoneNm = 0.18;
profile.RawForceStopN = 60.0;
profile.FastForceStopN = 40.0;
profile.RawMomentStopNm = 16.0;
profile.FastMomentStopNm = 8.0;

% Apply the editable values.  Geometry, RCM coordinates, force calibration
% path, joint limits and completed-mode evidence stay sourced from the
% accepted dual-pivot MAT file above.
cfg.runtime.ControlRateHz = profile.ControlRateHz;
cfg.runtime.NominalDtSec = 1 / profile.ControlRateHz;
cfg.robot.ServoJLookaheadTime = profile.ServoJLookaheadTime;
cfg.robot.ServoJGain = profile.ServoJGain;

cfg.stage08.DofMode = "full_3dof";
cfg.stage08.ProfileApplied = true;
cfg.stage08.SpeedScale = profile.SpeedScale;
cfg.stage08.SpeedFactorPercent = profile.SpeedFactorPercent;
cfg.stage08.EnablePayloadKg = profile.EnablePayloadKg;
cfg.stage08.PivotAdmittanceSpeedScale = 1.60;
cfg.stage08.PivotMaximumSpeedMultiplier = 2.50;
cfg.stage08.PivotTargetSpeedMultiplier = 8 / 3;
cfg.stage08.PivotSpeedRetuneApplied = true;
cfg.stage08.PivotSpeedRetuneRevision = 3;
cfg.stage08.MaximumServoTargetStepRad = ...
    profile.MaximumServoTargetStepRad;
cfg.stage08.MaximumTargetFeedbackErrorRad = ...
    profile.MaximumTargetFeedbackErrorRad;

cfg.admittance.Pivot.DesignForceN = profile.PivotDesignForceN;
cfg.admittance.Pivot.NominalLeverArmM = ...
    profile.PivotNominalLeverArmM;
cfg.admittance.Pivot.TargetSteadySpeedRadSec = ...
    profile.PivotTargetSteadySpeedRadSec;
cfg.admittance.Pivot.TimeConstantSec = ...
    profile.PivotTimeConstantSec;
cfg.admittance.Pivot.MaximumAccelerationRadSec2 = ...
    profile.PivotMaximumAccelerationRadSec2;
cfg.admittance.Pivot.MaximumTipSpeedMSec = ...
    profile.PivotMaximumTipSpeedMSec;
cfg.control.PivotMaxRadSec = profile.PivotMaximumSpeedRadSec;
cfg.control.RelativePivotLimitRad = profile.RelativePivotLimitRad;

cfg.admittance.Insertion.DesignForceN = ...
    profile.InsertionDesignForceN;
cfg.admittance.Insertion.TargetSteadySpeedMSec = ...
    profile.InsertionTargetSteadySpeedMSec;
cfg.admittance.Insertion.TimeConstantSec = ...
    profile.InsertionTimeConstantSec;
cfg.admittance.Insertion.MaximumAccelerationMSec2 = ...
    profile.InsertionMaximumAccelerationMSec2;
cfg.control.InsertionMaxMSec = profile.InsertionMaximumSpeedMSec;
cfg.control.RelativeInsertionLimitM = ...
    profile.RelativeInsertionLimitM;

cfg.control.MaximumJointCommandRadSec(:) = ...
    profile.MaximumJointCommandRadSec;
cfg.control.MaximumJointAccelerationRadSec2(:) = ...
    profile.MaximumJointAccelerationRadSec2;
cfg.rcm.Mode = profile.RcmMode;
cfg.rcm.SoftRadiusM = profile.RcmSoftRadiusM;
cfg.rcm.HardRadiusM = profile.RcmHardRadiusM;

cfg.force.ForceDeadzoneN = profile.ForceDeadzoneN;
cfg.force.MomentDeadzoneNm = profile.MomentDeadzoneNm;
cfg.force.NeutralForceLimitN = profile.ForceDeadzoneN;
cfg.force.NeutralMomentLimitNm = profile.MomentDeadzoneNm;
cfg.safety.RawForceStopN = profile.RawForceStopN;
cfg.safety.FastForceStopN = profile.FastForceStopN;
cfg.safety.RawMomentStopNm = profile.RawMomentStopNm;
cfg.safety.FastMomentStopNm = profile.FastMomentStopNm;
cfg.stage08.SetupConfirmationPhrase = ...
    "SCOPEGUIDE_3DOF_DRAG_READY";
cfg.stage08.SafetyThresholdConfirmationPhrase = ...
    "CONFIRM_LIMITS_60N_40N_16NM_8NM";

cfg.meta.ActiveTuningProfile = profile.Name;
cfg.meta.ActiveTuningProfileSource = sourceFile;

validateRcmAdmittanceConfig(cfg);

parameters = ...
    scopeguide.control.deriveForceOnlyAdmittanceParameters(cfg);
profile.PivotDampingNmSecPerRad = parameters.Damping(1);
profile.PivotVirtualMassKgM2 = parameters.VirtualMass(1);
profile.InsertionDampingNSecPerM = parameters.Damping(3);
profile.InsertionVirtualMassKg = parameters.VirtualMass(3);
end
