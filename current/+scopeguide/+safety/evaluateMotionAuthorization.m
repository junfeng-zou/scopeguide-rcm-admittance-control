function authorization = evaluateMotionAuthorization(cfg, typedConfirmation)
%EVALUATEMOTIONAUTHORIZATION Pure, non-interactive physical-motion gate.
% Passing this gate never sends a command. Hardware adapters must still
% check authorization.Allowed immediately before every motion command.

arguments
    cfg (1, 1) struct
    typedConfirmation = ""
end

validateRcmAdmittanceConfig(cfg);
mode = string(cfg.runtime.Mode);
motionMode = any(mode == ["fixture_motion", "phantom_motion"]);

gates = struct();
gates.MotionModeSelected = motionMode;
gates.EnableMotionSet = cfg.robot.EnableMotion;
gates.DryRunDisabled = ~cfg.robot.DryRun;
gates.TypedConfirmationMatches = ...
    ~cfg.robot.RequireTypedConfirmation || ...
    strcmp(string(typedConfirmation), ...
        string(cfg.robot.MotionConfirmationPhrase));
gates.ToolGeometryVerified = cfg.tool.GeometryVerified;
gates.RobotModelVerified = cfg.robot.ModelVerified;
gates.ServoTimingVerified = cfg.robot.ServoTimingVerified;
gates.RcmPointInRobotBase = ...
    string(cfg.rcm.PointFrame) == "robot_base";
gates.RcmPointSourceDeclared = ...
    string(cfg.rcm.PointSource) ~= "unset";
gates.RcmCalibrationValid = cfg.rcm.CalibrationValid && ...
    all(isfinite(cfg.rcm.PointBaseM));
gates.RcmBoundsValidated = cfg.rcm.BoundsValidated;
gates.ForceParametersValidated = cfg.force.ParametersValidated;
gates.SafetyThresholdsValidated = cfg.safety.ThresholdsValidated;
gates.JointLimitsValidated = cfg.safety.JointLimitsValidated;
gates.RollDisabled = ~cfg.control.EnableRoll && ...
    cfg.control.RollMaxRadSec == 0;

names = fieldnames(gates);
passed = false(size(names));
for index = 1:numel(names)
    passed(index) = gates.(names{index});
end

authorization = struct();
authorization.Requested = motionMode || cfg.robot.EnableMotion || ...
    ~cfg.robot.DryRun;
authorization.Allowed = all(passed);
authorization.Gates = gates;
authorization.FailedGates = string(names(~passed));
authorization.Mode = mode;
authorization.DoesNotSendHardwareCommands = true;
end
