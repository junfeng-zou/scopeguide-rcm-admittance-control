function [cfg, profile] = current_stage08_full_3dof_config(options)
%CURRENT_STAGE08_FULL_3DOF_CONFIG Editable pivot + insertion parameters.
% The accepted dual-pivot MAT file supplies the validated hardware,
% calibration and prerequisite records.  The block marked USER-EDITABLE
% contains the final, already-scaled values used by the most recent
% full-3DOF run.  MAT acceptance evidence is never modified by this file.
%
% The returned generalized order is [pivot1; pivot2; insertion].  Roll is
% kept disabled.  run_stage08_fixture_commissioning still requires all
% physical-motion confirmation arguments and explicitly selecting
% DofMode="full_3dof".

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
if strlength(options.SourceAcceptedConfigFile) == 0
    cfg = scopeguide.runtime.relocatePackagedConfig(cfg, projectRoot);
end
completedModes = string(cfg.stage08.CompletedDofModes(:));
if ~any(completedModes == "dual_pivot")
    error('scopeguide:stage08:Full3DofPrerequisiteMissing', ...
        ['The editable full-3DOF profile requires an accepted ' ...
         'dual_pivot prerequisite.']);
end

% =====================================================================
% USER-EDITABLE FINAL VALUES
% These are final Stage 8 values, not unscaled defaults.  ProfileApplied
% remains true so configureStage08Commissioning will not multiply them a
% second time.  Changing SpeedScale alone therefore changes only the
% recorded/displayed scale; edit the explicit target/limit below when
% changing the actual motion response.
% =====================================================================
profile = struct();
profile.Name = "stage08_full_3dof_editable_20260820";
profile.SourceAcceptedConfigFile = sourceFile;

% Runtime and ServoJ.
profile.ControlRateHz = 20.0;
profile.SpeedScale = 1.20;
profile.ServoJLookaheadTime = 60.0;
profile.ServoJGain = 400.0;
profile.SpeedFactorPercent = 30;
profile.EnablePayloadKg = 1.3;

% Flange-to-endoscope-tip geometry from the 10-pose fixed-tip pivot
% calibration on 2026-08-25.  The user accepted the 1.262 mm RMS and
% 2.037 mm maximum fixed-point residual for the present application.
% The fixed-point experiment estimates translation only, so the existing
% identity flange-to-endoscope rotation is retained.
profile.TFlangeEndoscope = [ ...
    1 0 0 0.003122985; ...
    0 1 0 0.118179746; ...
    0 0 1 0.428666535; ...
    0 0 0 1];
profile.ToolGeometrySource = ...
    "fixed_tip_user_accepted_20260825_142503";

% Latest physical RCM point supplied by the user after updating the
% flange-to-endoscope-tip transform.  Coordinates are in the robot base
% frame and metres.
profile.RcmPointBaseM = [-0.3707, 0.7433, 0.1040];
profile.RcmPointSourceDescription = ...
    "get_rcm_target_user_accepted_20260825";

% Pivot: values used in the 2026-08-17 full-3DOF runs.
profile.PivotDesignForceN = 20.0;
profile.PivotNominalLeverArmM = 0.15;
profile.PivotTargetSteadySpeedRadSec = ...
    deg2rad(2.133333333333333);
profile.PivotTimeConstantSec = 0.08;
profile.PivotMaximumAccelerationRadSec2 = deg2rad(30.0);
profile.PivotMaximumTipSpeedMSec = 0.006;
profile.PivotMaximumSpeedRadSec = deg2rad(3.0);
profile.RelativePivotLimitRad = deg2rad(7.5);
profile.RelativePivotRecoveryToleranceRad = deg2rad(0.3);

% Insertion: final values accepted for the current full-3DOF drag feel.
profile.InsertionDesignForceN = 6.0;
profile.InsertionTargetSteadySpeedMSec = 0.006;
profile.InsertionTimeConstantSec = 0.15;
profile.InsertionMaximumAccelerationMSec2 = 0.020;
profile.InsertionMaximumSpeedMSec = 0.008;
profile.RelativeInsertionLimitM = 0.030;
profile.RelativeInsertionRecoveryToleranceM = 0.001;

% Joint-command and RCM limits.
profile.MaximumJointCommandRadSec = deg2rad(6.0);
profile.MaximumJointAccelerationRadSec2 = deg2rad(24.0);
profile.MaximumServoTargetStepRad = deg2rad(0.30);
profile.MaximumTargetFeedbackErrorRad = deg2rad(0.50);
profile.RcmMode = "soft";
profile.RcmSoftRadiusM = 0.0005;
profile.RcmHardRadiusM = 0.002;
% Soft-RCM delayed-feedback suppression: no recovery below 0.25 mm,
% smooth activation from 0.25 to 0.50 mm, then the original recovery gain.
profile.RcmRecoveryDeadzoneM = 0.00025;
profile.RcmRecoveryFullActivationM = 0.00050;

% Force processing and the currently confirmed Stage 8 stop thresholds.
profile.ForceDeadzoneN = 1.3;
profile.MomentDeadzoneNm = 0.18;
profile.RawForceStopN = 60.0;
profile.FastForceStopN = 40.0;
profile.RawMomentStopNm = 16.0;
profile.FastMomentStopNm = 8.0;

% Apply the editable values.  RCM coordinates, force calibration path,
% joint limits and completed-mode evidence stay sourced from the accepted
% dual-pivot MAT file above.  Tool geometry is explicitly overridden by
% the accepted fixed-tip result above.
cfg.runtime.ControlRateHz = profile.ControlRateHz;
cfg.runtime.NominalDtSec = 1 / profile.ControlRateHz;
cfg.robot.ServoJLookaheadTime = profile.ServoJLookaheadTime;
cfg.robot.ServoJGain = profile.ServoJGain;
cfg.tool.TFlangeEndoscope = profile.TFlangeEndoscope;
cfg.tool.GeometryVerified = true;
cfg.tool.GeometrySource = profile.ToolGeometrySource;
cfg.rcm.PointBaseM = profile.RcmPointBaseM;
cfg.rcm.PointFrame = "robot_base";
cfg.rcm.PointSource = "manual_base_coordinate";
cfg.rcm.PointSourceDescription = ...
    profile.RcmPointSourceDescription;
cfg.rcm.PointRecordedAtLocal = "2026-08-25 Asia/Shanghai";
cfg.rcm.CalibrationValid = true;
cfg.rcm.BoundsValidated = true;

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
cfg.control.RelativePivotRecoveryToleranceRad = ...
    profile.RelativePivotRecoveryToleranceRad;

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
cfg.control.RelativeInsertionRecoveryToleranceM = ...
    profile.RelativeInsertionRecoveryToleranceM;

cfg.control.MaximumJointCommandRadSec(:) = ...
    profile.MaximumJointCommandRadSec;
cfg.control.MaximumJointAccelerationRadSec2(:) = ...
    profile.MaximumJointAccelerationRadSec2;
cfg.rcm.Mode = profile.RcmMode;
cfg.rcm.SoftRadiusM = profile.RcmSoftRadiusM;
cfg.rcm.HardRadiusM = profile.RcmHardRadiusM;
cfg.qp.RcmRecoveryDeadzoneM = profile.RcmRecoveryDeadzoneM;
cfg.qp.RcmRecoveryFullActivationM = ...
    profile.RcmRecoveryFullActivationM;

cfg.force.ForceDeadzoneN = profile.ForceDeadzoneN;
cfg.force.MomentDeadzoneNm = profile.MomentDeadzoneNm;
cfg.force.NeutralForceLimitN = profile.ForceDeadzoneN;
cfg.force.NeutralMomentLimitNm = profile.MomentDeadzoneNm;
cfg.safety.RawForceStopN = profile.RawForceStopN;
cfg.safety.FastForceStopN = profile.FastForceStopN;
cfg.safety.RawMomentStopNm = profile.RawMomentStopNm;
cfg.safety.FastMomentStopNm = profile.FastMomentStopNm;
cfg.stage08.SafetyThresholdConfirmationPhrase = ...
    "CONFIRM_LIMITS_60N_40N_4NM_2NM";

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
