function cfg = defaultRcmAdmittanceConfig()
%DEFAULTRCMADMITTANCECONFIG Safe defaults for the ScopeGuide controller.
% All controller units are SI: metre, radian, second, newton and N*m.
% The returned configuration cannot authorize physical robot motion.

projectRoot = fileparts(fileparts(mfilename('fullpath')));

cfg = struct();
cfg.meta.SchemaVersion = "0.18.0-stage08-pivot-feel-160pct";
cfg.meta.ProjectName = "ScopeGuide";
cfg.meta.ProjectRoot = string(projectRoot);
cfg.meta.UserAttestationRecord = string(fullfile(projectRoot, 'config', ...
    'stage08_user_attestation_20260814.json'));

cfg.runtime.Mode = "offline_replay";
% The CR5 TCP/IP feedback stream is expected to remain stable near 33 Hz.
% Run the controller slightly below that rate so each control update normally
% receives a new robot sample instead of repeatedly reusing the same frame.
cfg.runtime.ControlRateHz = 30;
cfg.runtime.NominalDtSec = 1 / cfg.runtime.ControlRateHz;
% Allow bounded host/network jitter, while still latching a fault before
% three nominal 30 Hz control periods have elapsed.
cfg.runtime.MaximumDtSec = 0.075;
cfg.runtime.StopAfterOneIteration = true;

cfg.robot.IPAddress = "192.168.50.105";
cfg.robot.DashboardPort = 29999;
cfg.robot.MovePort = 30003;
cfg.robot.FeedbackPort = 30004;
% ServoJ t is not stored independently: every command derives it from
% cfg.runtime.NominalDtSec so it always follows the configured control rate.
cfg.robot.ServoJLookaheadTime = 60.0;
cfg.robot.ServoJGain = 400.0;
cfg.robot.EnableMotion = false;
cfg.robot.DryRun = true;
cfg.robot.RequireTypedConfirmation = true;
cfg.robot.MotionConfirmationPhrase = "ENABLE_SCOPEGUIDE_MOTION";
cfg.robot.ExpectedFeedbackRateHz = 33;
% One 33 Hz feedback interval is about 30.3 ms.  This threshold tolerates
% one missed update plus scheduling jitter, but not a sustained disconnect.
cfg.robot.FeedbackStaleSec = 0.080;
cfg.robot.InitialFeedbackTimeoutSec = 5.0;
cfg.robot.JointPositionScaleToRad = pi / 180;
cfg.robot.JointVelocityScaleToRadSec = pi / 180;
cfg.robot.ControllerPoseTranslationScaleToM = 1e-3;
cfg.robot.ControllerPoseAngleScaleToRad = pi / 180;
cfg.robot.TcpLinearVelocityScaleToMSec = 1e-3;
cfg.robot.TcpAngularVelocityScaleToRadSec = pi / 180;
% Stage 2 close-out (2026-08-13): three live read-only poses and the
% 12-pose final-assembly replay both support a flange-referenced
% CartesianPose in mm/deg.  The nominal DH model remains below the current
% 2.0 mm hard-RCM bound, but not the 0.5 mm soft candidate.
cfg.robot.ControllerPoseReference = "flange";
cfg.robot.ControllerPoseUnitsVerified = true;
cfg.robot.ModelVerified = true;
% Command-to-feedback timing requires actual low-speed motion and therefore
% remains a later fixture-stage gate; feedback timing alone is not enough.
cfg.robot.ServoTimingVerified = false;

cfg.sensor.SampleRateHz = 200;
cfg.sensor.StaleSec = 0.020;
cfg.sensor.MaximumReadDurationSec = 0.020;
cfg.sensor.FutureTimestampToleranceSec = 0.002;
cfg.sensor.StatusCheckEnabled = true;

cfg.handle.StaleSec = 0.020;
cfg.handle.FutureTimestampToleranceSec = 0.002;
cfg.handle.RisingDebounceSec = 0.040;
cfg.handle.SoftStartSec = 0.200;
% GUI keyboard only: Linux/X11 auto-repeat may emit a synthetic release
% followed by a delayed press.  At the 30 Hz control rate, retain the
% release for 50 ms so the pair cannot create a one-cycle false release.
% This is never a physical-motion deadman parameter.
cfg.handle.KeyboardReleaseConfirmSec = 0.050;
% Once Linux key repeat has started, repeated KeyPress events act as a
% software-only heartbeat.  If they stop and the final KeyRelease event is
% missing, force a release after this interval.
cfg.handle.KeyboardRepeatWatchdogSec = 0.200;
cfg.handle.SoftwareInputSupportsPhysicalMotion = false;

cfg.tool.TFlangeEndoscope = [ ...
    1 0 0 -0.002840; ...
    0 1 0  0.113051; ...
    0 0 1  0.355046; ...
    0 0 0  1];
% Estimated pose of the HEX-H wrench measurement frame in the robot flange
% frame.  The measured wrench is rotated into flange/tool axes by the force
% calibration before control; this transform supplies the measurement
% origin needed to shift that wrench to the RCM.  The user estimated the
% origin at flange +Z 20 mm.  diag([-1 -1 1]) matches the calibrated HEX-H
% sensor-to-tool axis convention and preserves a right-handed frame.
cfg.tool.TFlangeSensor = [ ...
   -1  0  0  0; ...
    0 -1  0  0; ...
    0  0  1  0.020; ...
    0  0  0  1];
cfg.tool.SensorOriginSource = ...
    "user_estimate_flange_plus_z_20mm_2026-08-16";
cfg.tool.TipInEndoscopeM = [0; 0; 0];
cfg.tool.ShaftAxisEndoscope = [0; 0; 1];
cfg.tool.InsertionAxisSign = 1;
% User attested on 2026-08-14 that this final-assembly transform, shaft
% direction and insertion sign are usable for free-space commissioning.
cfg.tool.GeometryVerified = true;

cfg.kinematics.Convention = "standard-DH";
cfg.kinematics.ThetaOffsetRad = [0, pi/2, 0, pi/2, 0, 0];
cfg.kinematics.DM = [0.240, 0.135, 0, 0, 0.120, 0.088];
cfg.kinematics.AM = [0, 0.400, 0.330, 0, 0, 0];
cfg.kinematics.AlphaRad = [pi/2, 0, 0, pi/2, -pi/2, 0];
cfg.kinematics.MaximumAbsJointAngleRad = 2 * pi;

% Manufacturer CR5 joint travel.  These are absolute model limits, not a
% recommendation to operate near them.  Stage 8 keeps an additional 5 deg
% QP/command margin from every limit; joint 3 is the asymmetric-range joint
% in the CR5 specification (plus/minus 160 deg).
cfg.kinematics.JointLowerLimitRad = deg2rad( ...
    [-360; -360; -160; -360; -360; -360]);
cfg.kinematics.JointUpperLimitRad = deg2rad( ...
    [ 360;  360;  160;  360;  360;  360]);
cfg.kinematics.JointLimitSource = ...
    "Dobot CR5 product specification, accessed 2026-08-14";

cfg.force.CalibrationFile = string(fullfile(projectRoot, ...
    'force_calibration', 'data', 'hex_h_tool_calibration_20260812_154348', ...
    'calibration_without_pose01_pose07_pose10.json'));
cfg.force.FastCutoffHz = 20;
cfg.force.ControlCutoffHz = 5;
cfg.force.ForceDeadzoneN = 1.3;
cfg.force.MomentDeadzoneNm = 0.18;
cfg.force.NeutralForceLimitN = 1.3;
cfg.force.NeutralMomentLimitNm = 0.18;
cfg.force.DeadzoneReleaseRatio = 0.80;
cfg.force.AutomaticBaselineEnabled = false;
% The user accepts the current final-load calibration, filters, 1.3 N /
% 0.18 N*m deadzones and adaptive Fz baseline strategy for free-space
% endoscope micro-adjustment commissioning.
cfg.force.ParametersValidated = true;
% When AutomaticBaselineEnabled is explicitly enabled, collect one
% contiguous, stationary no-contact interval before allowing control.  The
% initial software zero is six-axis; subsequent adaptation is Fz-only.
cfg.force.Baseline.StartupZeroEnabled = true;
cfg.force.Baseline.StartupDurationSec = 3.0;
cfg.force.Baseline.StartupMaximumForceNormN = 25.0;
cfg.force.Baseline.StartupMaximumMomentNormNm = 2.0;
cfg.force.Baseline.TrackingComponentMask = ...
    logical([0; 0; 1; 0; 0; 0]);
cfg.force.Baseline.MinimumEligibleDurationSec = 1.0;
cfg.force.Baseline.UpdateTimeConstantSec = 30.0;
cfg.force.Baseline.MaximumForceUpdateRateNSec = 0.05;
cfg.force.Baseline.MaximumMomentUpdateRateNmSec = 0.005;
% These limits apply to slow tracking relative to the startup software
% zero, rather than to the absolute startup baseline itself.
cfg.force.Baseline.MaximumForceOffsetN = 15.0;
cfg.force.Baseline.MaximumMomentOffsetNm = 0.30;
cfg.force.Baseline.EligibilityForceNormN = 1.8;
cfg.force.Baseline.EligibilityMomentNormNm = 0.30;

cfg.control.DofCount = 3;
cfg.control.EnableRoll = false;
cfg.control.PivotMaxRadSec = deg2rad(1);
cfg.control.InsertionMaxMSec = 0.002;
cfg.control.RollMaxRadSec = 0;
cfg.control.RelativePivotLimitRad = deg2rad(5);
cfg.control.RelativeInsertionLimitM = 0.008;
% A small recovery band separates the normal travel limit from the hard
% fault boundary.  Inside this band the QP blocks further outward motion
% while retaining velocity back toward the normal range.
cfg.control.RelativePivotRecoveryToleranceRad = deg2rad(0.3);
cfg.control.RelativeInsertionRecoveryToleranceM = 0.0005;
cfg.control.MaximumJointCommandRadSec = ...
    deg2rad(5) * ones(6, 1);
cfg.control.MaximumJointAccelerationRadSec2 = ...
    deg2rad(20) * ones(6, 1);
cfg.control.MaximumTargetStateErrorRad = ...
    deg2rad(2) * ones(6, 1);

% Stage 5 force-only 3-DOF admittance.  Damping and virtual mass are
% derived, rather than tuned independently:
%   damping = design generalized effort / target steady speed
%   virtual mass = damping * time constant
% Pivot effort uses DesignForceN * NominalLeverArmM; insertion effort uses
% DesignForceN directly.  The limits remain in cfg.control so there is one
% authoritative set of task-space safety bounds.
cfg.admittance.Pivot.DesignForceN = 5.0;
cfg.admittance.Pivot.NominalLeverArmM = 0.15;
cfg.admittance.Pivot.TargetSteadySpeedRadSec = deg2rad(0.5);
cfg.admittance.Pivot.TimeConstantSec = 0.30;
cfg.admittance.Pivot.MaximumAccelerationRadSec2 = deg2rad(5);
cfg.admittance.Pivot.MaximumTipSpeedMSec = 0.005;
cfg.admittance.Insertion.DesignForceN = 5.0;
cfg.admittance.Insertion.TargetSteadySpeedMSec = 0.001;
cfg.admittance.Insertion.TimeConstantSec = 0.30;
cfg.admittance.Insertion.MaximumAccelerationMSec2 = 0.010;
cfg.admittance.VelocityZeroTolerance = [1e-7; 1e-7; 1e-8];

% Stage 6 offline RCM-constrained QP.  The current RCM radii and symmetric
% joint-position range are conservative algorithm-test values only; real
% motion remains blocked until their validation gates are explicitly met.
cfg.qp.Solver = "quadprog";
cfg.qp.TaskCharacteristicLengthM = 0.15;
cfg.qp.TaskTrackingWeight = 1.0;
cfg.qp.JointVelocityRegularization = 1e-3;
cfg.qp.SoftRcmWeight = 25.0;
cfg.qp.HybridRcmWeightMinimum = 10.0;
cfg.qp.HybridRcmWeightMaximum = 250.0;
cfg.qp.RcmRecoveryGainMinimumSecInv = 0.5;
cfg.qp.RcmRecoveryGainMaximumSecInv = 2.0;
% Suppress small delayed-feedback RCM corrections and smoothly restore the
% original recovery target between 0.25 and 0.50 mm.  Hard RCM mode does
% not use this shaping, so its equality-constraint semantics are unchanged.
cfg.qp.RcmRecoveryDeadzoneM = 0.00025;
cfg.qp.RcmRecoveryFullActivationM = 0.00050;
cfg.qp.JointCenteringGainSecInv = 0.05;
cfg.qp.JointCenteringMaximumRadSec = deg2rad(0.5);
cfg.qp.JointPositionMarginRad = deg2rad(2.0);
cfg.qp.HardConstraintPolygonSides = 32;
cfg.qp.RelativePivotPolygonSides = 32;
cfg.qp.SingularityStopSigma = 0.002;
cfg.qp.SingularityFullSpeedSigma = 0.020;
cfg.qp.ConstraintTolerance = 1e-8;
cfg.qp.OptimalityTolerance = 1e-8;
cfg.qp.MaximumIterations = 100;
% Keep the QP budget below half of the 33.3 ms control period.  This also
% accommodates the measured 11.6 ms first solve after a long idle interval.
cfg.qp.MaximumSolveTimeSec = 0.015;
cfg.qp.MaximumConsecutiveFailures = 3;


% User-supplied robot-base coordinate and bounds, explicitly accepted by
% the user on 2026-08-14 for the current unchanged free-space setup.
cfg.rcm.PointBaseM = [-0.2869, 0.6260, 0.1446];
cfg.rcm.PointFrame = "robot_base";
cfg.rcm.PointSource = "manual_base_coordinate";
cfg.rcm.PointSourceDescription = ...
    "User-corrected Stage 7 robot-base coordinate; units are metres";
cfg.rcm.FixtureIdentifier = "stage07_current_setup";
cfg.rcm.PointRecordedAtLocal = "2026-08-13 Asia/Shanghai";
cfg.rcm.Mode = "soft";
cfg.rcm.SoftRadiusM = 0.0005;
cfg.rcm.HardRadiusM = 0.0020;
cfg.rcm.CalibrationValid = true;
cfg.rcm.BoundsValidated = true;

cfg.safety.ThresholdsValidated = false;
% The numerical joint limits above are taken from the CR5 manufacturer
% specification and are exercised by the Stage 8 offline boundary tests.
cfg.safety.JointLimitsValidated = true;
cfg.safety.RequireFreshSensor = true;
cfg.safety.RequireFreshRobot = true;
cfg.safety.RequireFreshHandle = true;
% Stage 1 offline starting points only. Physical motion remains denied
% until ThresholdsValidated is set after fixture/phantom experiments.
cfg.safety.RawForceStopN = 60.0;
cfg.safety.RawMomentStopNm = 4.0;
cfg.safety.FastForceWarningN = 6.0;
cfg.safety.FastForceStopN = 40.0;
cfg.safety.FastMomentWarningNm = 0.8;
cfg.safety.FastMomentStopNm = 2.0;
cfg.safety.FastForceRateStopNSec = 1000.0;

% Stage 8 is deliberately a mechanical-fixture commissioning profile.  It
% does not authorize tissue/phantom operation.  SpeedScale multiplies all
% task limits, joint limits and admittance acceleration/speed parameters.
cfg.stage08.DofMode = "insertion_only";
cfg.stage08.ProfileApplied = false;
cfg.stage08.AllowedDofModes = ["hold", "insertion_only", ...
    "pivot1_only", "pivot2_only", "dual_pivot", "full_3dof"];
cfg.stage08.SpeedScale = 2.5;
% Stage 8 pivot-only retuning is explicit and symmetric for pivot1/pivot2.
% PivotAdmittanceSpeedScale is deliberately independent of the global
% SpeedScale: it changes the force-to-speed sensitivity without raising
% joint, insertion or PivotMax safety caps.  With SpeedScale=1.2 and the
% values below, the 5 N design target is 2.133 deg/s and PivotMax is
% 3.0 deg/s.
cfg.stage08.PivotAdmittanceSpeedScale = 1.60;
cfg.stage08.PivotTargetSpeedMultiplier = 8 / 3;
cfg.stage08.PivotMaximumSpeedMultiplier = 2.50;
cfg.stage08.PivotSpeedRetuneApplied = false;
cfg.stage08.PivotSpeedRetuneRevision = 3;
% Stage 8 uses the application-validated software initialization sequence
% from dobot_HW_control2. Dashboard replies are optional/delayed; transition
% is authorized only after 30004 confirms ENABLE/RUNNING and SpeedRatio=20.
cfg.stage08.ProgrammaticRobotEnable = true;
cfg.stage08.EnablePayloadKg = 1.3;
cfg.stage08.SpeedFactorPercent = 20;
cfg.stage08.PostEnablePauseSec = 2.0;
cfg.stage08.SpeedFactorFeedbackTolerance = 0.5;
cfg.stage08.DisableRobotOnExit = true;
cfg.stage08.KeyboardInputMode = "space_on_escape_off";
cfg.stage08.InputHeartbeatTimeoutSec = 0.250;
cfg.stage08.JointLimitMarginRad = deg2rad(5.0);
% This is a command-to-command discontinuity guard.  The first ServoJ
% target is compared with current feedback; later targets are compared
% with the last submitted ServoJ target.  Tracking lag has the
% separate 0.50 deg / 3-consecutive-cycle watchdog below.
cfg.stage08.MaximumServoTargetStepRad = deg2rad(0.30);
% Maximum local duration allowed for writing one ServoJ command to the
% operating-system TCP buffer. ServoJ has no required 30003 reply, so there
% is deliberately no per-command reply timeout or pending-reply limit.
cfg.stage08.MaximumServoSendSec = 0.060;
cfg.stage08.MaximumTargetFeedbackErrorRad = deg2rad(0.50);
cfg.stage08.MaximumConsecutiveTrackingErrors = 3;
cfg.stage08.StopVelocityThresholdRadSec = deg2rad(0.05);
cfg.stage08.MaximumStopLatencySec = 0.300;
cfg.stage08.RequiredRobotModes = ["ENABLE", "RUNNING"];
cfg.stage08.RequireIndependentEmergencyStop = true;
% The user explicitly reported that the robot emergency stop is available.
% This records that statement; it does not test the emergency-stop circuit.
cfg.stage08.EmergencyStopAttested = true;
cfg.stage08.EmergencyStopAttestationSource = ...
    "User statement on 2026-08-14";
cfg.stage08.RequireSecondObserver = true;
cfg.stage08.RequireFixtureAndClearanceConfirmation = true;
cfg.stage08.SetupConfirmationPhrase = "STAGE8_FIXTURE_READY";
cfg.stage08.SafetyThresholdConfirmationPhrase = ...
    "CONFIRM_LIMITS_60N_40N_4NM_2NM";
cfg.stage08.AllowServoTimingBootstrap = true;
cfg.stage08.BootstrapDofMode = "insertion_only";
cfg.stage08.CompletedDofModes = strings(0, 1);

cfg.logging.ResultsRoot = string(fullfile(projectRoot, 'results'));
cfg.logging.WriteEnvironmentReport = true;

validateRcmAdmittanceConfig(cfg);
end
