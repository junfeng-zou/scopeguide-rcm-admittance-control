function authorization = evaluateStage08CommissioningAuthorization( ...
        cfg, confirmations)
%EVALUATESTAGE08COMMISSIONINGAUTHORIZATION Pure Stage 8 fixture gate.
% This function sends no command.  It permits one narrowly-scoped timing
% bootstrap while ServoTimingVerified is still false; every other physical
% and configuration gate remains mandatory.

arguments
    cfg (1, 1) struct
    confirmations (1, 1) struct
end

validateRcmAdmittanceConfig(cfg);
required = {'MotionPhrase', 'SetupPhrase', 'SafetyThresholdPhrase', ...
    'SecondObserverPresent', 'FixtureAndClearanceConfirmed'};
for index = 1:numel(required)
    if ~isfield(confirmations, required{index})
        error('scopeguide:stage08:IncompleteConfirmations', ...
            'Stage 8 confirmation %s is missing.', required{index});
    end
end
logicalConfirmation(confirmations.SecondObserverPresent, ...
    'SecondObserverPresent');
logicalConfirmation(confirmations.FixtureAndClearanceConfirmed, ...
    'FixtureAndClearanceConfirmed');

mode = string(cfg.stage08.DofMode);
bootstrapUsed = ~cfg.robot.ServoTimingVerified && ...
    cfg.stage08.AllowServoTimingBootstrap && ...
    mode == string(cfg.stage08.BootstrapDofMode);
safetyThresholdsConfirmed = cfg.safety.ThresholdsValidated || ...
    strcmp(string(confirmations.SafetyThresholdPhrase), ...
    string(cfg.stage08.SafetyThresholdConfirmationPhrase));

gates = struct();
gates.FixtureMotionMode = string(cfg.runtime.Mode) == "fixture_motion";
gates.EnableMotionSet = cfg.robot.EnableMotion;
gates.DryRunDisabled = ~cfg.robot.DryRun;
gates.MotionPhraseMatches = ~cfg.robot.RequireTypedConfirmation || ...
    strcmp(string(confirmations.MotionPhrase), ...
    string(cfg.robot.MotionConfirmationPhrase));
gates.SetupPhraseMatches = strcmp(string(confirmations.SetupPhrase), ...
    string(cfg.stage08.SetupConfirmationPhrase));
gates.ToolGeometryVerified = cfg.tool.GeometryVerified;
gates.RobotModelVerified = cfg.robot.ModelVerified;
gates.ServoTimingVerifiedOrBootstrap = ...
    cfg.robot.ServoTimingVerified || bootstrapUsed || mode == "hold";
gates.RcmCalibrationValid = cfg.rcm.CalibrationValid && ...
    all(isfinite(cfg.rcm.PointBaseM));
gates.RcmBoundsValidated = cfg.rcm.BoundsValidated;
gates.ForceParametersValidated = cfg.force.ParametersValidated;
gates.SafetyThresholdsValidatedOrConfirmed = ...
    safetyThresholdsConfirmed;
gates.JointLimitsValidated = cfg.safety.JointLimitsValidated;
gates.EmergencyStopAttested = ...
    ~cfg.stage08.RequireIndependentEmergencyStop || ...
    cfg.stage08.EmergencyStopAttested;
gates.SecondObserverPresent = ~cfg.stage08.RequireSecondObserver || ...
    confirmations.SecondObserverPresent;
gates.FixtureAndClearanceConfirmed = ...
    ~cfg.stage08.RequireFixtureAndClearanceConfirmation || ...
    confirmations.FixtureAndClearanceConfirmed;
gates.KeyboardLatchModeSelected = ...
    string(cfg.stage08.KeyboardInputMode) == "space_on_escape_off";
gates.DofModeAllowed = any(mode == string(cfg.stage08.AllowedDofModes));
gates.DofPrerequisitesComplete = all(ismember( ...
    requiredCompletedModes(mode), ...
    string(cfg.stage08.CompletedDofModes)));
gates.SpeedScaleConservative = cfg.stage08.SpeedScale > 0 && ...
    cfg.stage08.SpeedScale <= 4.5;
gates.RollDisabled = ~cfg.control.EnableRoll && ...
    cfg.control.RollMaxRadSec == 0;

names = fieldnames(gates);
passed = false(size(names));
for index = 1:numel(names)
    passed(index) = gates.(names{index});
end
authorization = struct();
authorization.Requested = true;
authorization.Allowed = all(passed);
authorization.CommissioningAllowed = authorization.Allowed;
authorization.Gates = gates;
authorization.FailedGates = string(names(~passed));
authorization.Mode = string(cfg.runtime.Mode);
authorization.DofMode = mode;
authorization.ServoTimingBootstrapUsed = bootstrapUsed;
authorization.SafetyThresholdRuntimeConfirmationUsed = ...
    ~cfg.safety.ThresholdsValidated && safetyThresholdsConfirmed;
authorization.KeyboardInputIsNotPhysicalDeadman = true;
authorization.DoesNotSendHardwareCommands = true;
end

function required = requiredCompletedModes(mode)
switch string(mode)
    case {"hold", "insertion_only"}
        required = strings(0, 1);
    case "pivot1_only"
        required = "insertion_only";
    case "pivot2_only"
        required = ["insertion_only"; "pivot1_only"];
    case "dual_pivot"
        required = ["insertion_only"; "pivot1_only"; "pivot2_only"];
    case "full_3dof"
        required = ["insertion_only"; "pivot1_only"; ...
            "pivot2_only"; "dual_pivot"];
    otherwise
        required = "INVALID";
end
end

function logicalConfirmation(value, name)
if ~islogical(value) || ~isscalar(value)
    error('scopeguide:stage08:InvalidConfirmation', ...
        '%s must be a scalar logical.', name);
end
end
