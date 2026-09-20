function [cfg, profile] = current_cartesian_admittance_drag_config(options)
%CURRENT_CARTESIAN_ADMITTANCE_DRAG_CONFIG Editable no-RCM drag profile.
% The accepted Stage 8 file is used only as evidence for the already
% verified robot transport, force calibration and joint limits.  Cartesian
% drag parameters below are independent of the RCM controller and can be
% edited directly in this M-file.

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
    error('scopeguide:cartesianDrag:AcceptedConfigNotFound', ...
        ['The accepted robot/force configuration was not found: %s. ' ...
         'Pass SourceAcceptedConfigFile explicitly.'], sourceFile);
end

loaded = load(sourceFile, 'cfgAccepted');
if ~isfield(loaded, 'cfgAccepted') || ~isstruct(loaded.cfgAccepted)
    error('scopeguide:cartesianDrag:InvalidAcceptedConfigFile', ...
        'File does not contain a scalar cfgAccepted structure: %s', ...
        sourceFile);
end
cfg = scopeguide.runtime.upgradeRcmAdmittanceConfig(loaded.cfgAccepted);
if strlength(options.SourceAcceptedConfigFile) == 0
    cfg = scopeguide.runtime.relocatePackagedConfig(cfg, projectRoot);
end

% =====================================================================
% USER-EDITABLE NO-RCM CARTESIAN DRAG PARAMETERS
% Twist/wrench order is [x; y; z; rx; ry; rz].  Linear units are m/s and
% N; angular units are rad/s and N*m.
% =====================================================================
profile = struct();
profile.Name = "cartesian_admittance_drag_initial_20260821";
profile.SourceAcceptedConfigFile = sourceFile;
profile.ControlRateHz = 20.0;
% Explicitly override the accepted MAT value so the current no-RCM drag
% always initializes the Dobot controller with SpeedFactor(30).
profile.SpeedFactorPercent = 30;

% Keep the accepted endoscope-tip geometry synchronized with the RCM
% controller.  This does not move the no-RCM drag point: the latter remains
% the HEX-H measurement origin below so the accepted 6-DOF hand feel is
% unchanged.
profile.TFlangeEndoscope = [ ...
    1 0 0 0.003122985; ...
    0 1 0 0.118179746; ...
    0 0 1 0.428666535; ...
    0 0 0 1];
profile.ToolGeometrySource = ...
    "fixed_tip_user_accepted_20260825_142503";

% Retain the latest user-accepted RCM point as shared geometry metadata.
% The no-RCM Cartesian controller does not read this point or impose an
% RCM constraint.
profile.RcmPointBaseM = [-0.3213, 0.5618, 0.1358];
profile.RcmPointSourceDescription = ...
    "get_rcm_target_user_accepted_20260825";

% The default drag point is the HEX-H measurement origin: flange +Z 20 mm.
% Its axes are deliberately aligned with the flange/tool control axes.
profile.TFlangeDragPoint = eye(4);
profile.TFlangeDragPoint(1:3, 4) = cfg.tool.TFlangeSensor(1:3, 4);

profile.TranslationDesignForceN = 6.0;
profile.TranslationTargetSpeedMSec = 0.006;
profile.TranslationTimeConstantSec = 0.15;
profile.TranslationMaximumSpeedMSec = 0.008;
profile.TranslationMaximumAccelerationMSec2 = 0.030;
% Disable both translation and rotation limits relative to the pose captured
% at each Space-enable event.  The values below are retained only as display
% reference scales and can be re-enabled by setting this flag true.
profile.TravelLimitsEnabled = false;
profile.RelativeTranslationLimitM = 0.020 * ones(3, 1);
% This recovery tolerance is inactive while TravelLimitsEnabled is false.
profile.TranslationRecoveryToleranceM = 0.0010;

profile.RotationDesignMomentNm = 5.00;
profile.RotationTargetSpeedRadSec = deg2rad(3.0);
profile.RotationTimeConstantSec = 0.08;
profile.RotationMaximumSpeedRadSec = deg2rad(4.0);
profile.RotationMaximumAccelerationRadSec2 = deg2rad(30.0);
% These rotation values are inactive display/reference values while the
% shared TravelLimitsEnabled flag is false.
profile.RelativeRotationLimitRad = deg2rad(10.0) * ones(3, 1);
profile.RotationRecoveryToleranceRad = deg2rad(0.3);

% These saturate the wrench used by admittance; they are not safety-stop
% thresholds.  The independent raw/fast wrench monitor remains active.
profile.ControlForceSaturationN = 15.0;
profile.ControlMomentSaturationNm = 5.0;

% Independent raw/fast wrench safety-stop thresholds.  Keep the typed
% confirmation phrase synchronized whenever any threshold is changed.
profile.RawForceStopN = 60.0;
profile.FastForceStopN = 40.0;
profile.RawMomentStopNm = 20.0;
profile.FastMomentStopNm = 10.0;
profile.SafetyThresholdConfirmationPhrase = ...
    "CONFIRM_LIMITS_60N_40N_20NM_10NM";

profile.MaximumJointCommandRadSec = deg2rad(6.0);
profile.MaximumJointAccelerationRadSec2 = deg2rad(24.0);
profile.MaximumServoTargetStepRad = deg2rad(0.30);
profile.MaximumTargetFeedbackErrorRad = deg2rad(0.50);

cfg.runtime.ControlRateHz = profile.ControlRateHz;
cfg.runtime.NominalDtSec = 1 / profile.ControlRateHz;
cfg.stage08.SpeedFactorPercent = profile.SpeedFactorPercent;
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
cfg.control.MaximumJointCommandRadSec(:) = ...
    profile.MaximumJointCommandRadSec;
cfg.control.MaximumJointAccelerationRadSec2(:) = ...
    profile.MaximumJointAccelerationRadSec2;
cfg.stage08.MaximumServoTargetStepRad = ...
    profile.MaximumServoTargetStepRad;
cfg.stage08.MaximumTargetFeedbackErrorRad = ...
    profile.MaximumTargetFeedbackErrorRad;
cfg.safety.RawForceStopN = profile.RawForceStopN;
cfg.safety.FastForceStopN = profile.FastForceStopN;
cfg.safety.RawMomentStopNm = profile.RawMomentStopNm;
cfg.safety.FastMomentStopNm = profile.FastMomentStopNm;
cfg.stage08.SafetyThresholdConfirmationPhrase = ...
    profile.SafetyThresholdConfirmationPhrase;

cfg.cartesianDrag = struct();
cfg.cartesianDrag.DofMode = "translation_z";
cfg.cartesianDrag.AllowedDofModes = [ ...
    "hold", "translation_z", "translation_xyz", ...
    "rotation_x", "rotation_y", "rotation_z", ...
    "rotation_xyz", "full_6dof"];
cfg.cartesianDrag.TravelLimitsEnabled = ...
    profile.TravelLimitsEnabled;
cfg.cartesianDrag.TFlangeDragPoint = profile.TFlangeDragPoint;
cfg.cartesianDrag.DragPointSource = ...
    "hex_h_measurement_origin_flange_plus_z_20mm";
cfg.cartesianDrag.Translation.DesignForceN = ...
    profile.TranslationDesignForceN;
cfg.cartesianDrag.Translation.TargetSteadySpeedMSec = ...
    profile.TranslationTargetSpeedMSec;
cfg.cartesianDrag.Translation.TimeConstantSec = ...
    profile.TranslationTimeConstantSec;
cfg.cartesianDrag.Translation.MaximumSpeedMSec = ...
    profile.TranslationMaximumSpeedMSec;
cfg.cartesianDrag.Translation.MaximumAccelerationMSec2 = ...
    profile.TranslationMaximumAccelerationMSec2;
cfg.cartesianDrag.Translation.RelativeLimitM = ...
    profile.RelativeTranslationLimitM;
cfg.cartesianDrag.Translation.RecoveryToleranceM = ...
    profile.TranslationRecoveryToleranceM;
cfg.cartesianDrag.Rotation.DesignMomentNm = ...
    profile.RotationDesignMomentNm;
cfg.cartesianDrag.Rotation.TargetSteadySpeedRadSec = ...
    profile.RotationTargetSpeedRadSec;
cfg.cartesianDrag.Rotation.TimeConstantSec = ...
    profile.RotationTimeConstantSec;
cfg.cartesianDrag.Rotation.MaximumSpeedRadSec = ...
    profile.RotationMaximumSpeedRadSec;
cfg.cartesianDrag.Rotation.MaximumAccelerationRadSec2 = ...
    profile.RotationMaximumAccelerationRadSec2;
cfg.cartesianDrag.Rotation.RelativeLimitRad = ...
    profile.RelativeRotationLimitRad;
cfg.cartesianDrag.Rotation.RecoveryToleranceRad = ...
    profile.RotationRecoveryToleranceRad;
cfg.cartesianDrag.ControlForceSaturationN = ...
    profile.ControlForceSaturationN;
cfg.cartesianDrag.ControlMomentSaturationNm = ...
    profile.ControlMomentSaturationNm;
cfg.cartesianDrag.VelocityZeroTolerance = ...
    [1e-8 * ones(3, 1); 1e-7 * ones(3, 1)];
cfg.cartesianDrag.Qp.TaskCharacteristicLengthM = 0.15;
cfg.cartesianDrag.Qp.TaskTrackingWeight = 1.0;
cfg.cartesianDrag.Qp.JointVelocityRegularization = 1e-3;
cfg.cartesianDrag.Qp.CommandSmoothingWeight = 2e-3;
cfg.cartesianDrag.Qp.JointCenteringGainSecInv = 0.03;
cfg.cartesianDrag.Qp.JointCenteringMaximumRadSec = deg2rad(0.3);
cfg.cartesianDrag.Qp.SingularityStopSigma = 0.002;
cfg.cartesianDrag.Qp.SingularityFullSpeedSigma = 0.020;
cfg.cartesianDrag.SetupConfirmationPhrase = ...
    "CARTESIAN_DRAG_FIXTURE_READY";
cfg.cartesianDrag.SafetyThresholdConfirmationPhrase = ...
    cfg.stage08.SafetyThresholdConfirmationPhrase;
cfg.cartesianDrag.RequireSecondObserver = true;
cfg.cartesianDrag.RequireFixtureAndClearanceConfirmation = true;
cfg.cartesianDrag.InputHeartbeatTimeoutSec = ...
    cfg.stage08.InputHeartbeatTimeoutSec;

cfg.meta.ActiveTuningProfile = profile.Name;
cfg.meta.ActiveTuningProfileSource = sourceFile;

validateCartesianAdmittanceDragConfig(cfg);
parameters = scopeguide.control.deriveCartesianAdmittanceParameters(cfg);
profile.TranslationDampingNSecPerM = parameters.Damping(1);
profile.TranslationVirtualMassKg = parameters.VirtualMass(1);
profile.RotationDampingNmSecPerRad = parameters.Damping(4);
profile.RotationVirtualMassKgM2 = parameters.VirtualMass(4);
end
