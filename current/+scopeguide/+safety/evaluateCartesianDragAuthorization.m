function authorization = evaluateCartesianDragAuthorization(cfg, confirmations)
%EVALUATECARTESIANDRAGAUTHORIZATION Physical gate with no RCM conditions.
% Passing this pure check sends no hardware command.

arguments
    cfg (1, 1) struct
    confirmations (1, 1) struct
end
validateCartesianAdmittanceDragConfig(cfg);
required = {'MotionPhrase', 'SetupPhrase', 'SafetyThresholdPhrase', ...
    'SecondObserverPresent', 'FixtureAndClearanceConfirmed'};
if ~all(isfield(confirmations, required))
    missing = required(~isfield(confirmations, required));
    error('scopeguide:cartesianDrag:IncompleteConfirmations', ...
        'Missing confirmation fields: %s.', strjoin(string(missing), ', '));
end
logicalConfirmation(confirmations.SecondObserverPresent, ...
    'SecondObserverPresent');
logicalConfirmation(confirmations.FixtureAndClearanceConfirmed, ...
    'FixtureAndClearanceConfirmed');
c = cfg.cartesianDrag;
safetyConfirmed = cfg.safety.ThresholdsValidated || strcmp( ...
    string(confirmations.SafetyThresholdPhrase), ...
    string(c.SafetyThresholdConfirmationPhrase));

gates = struct();
gates.FixtureMotionMode = string(cfg.runtime.Mode) == "fixture_motion";
gates.EnableMotionSet = cfg.robot.EnableMotion;
gates.DryRunDisabled = ~cfg.robot.DryRun;
gates.MotionPhraseMatches = ~cfg.robot.RequireTypedConfirmation || ...
    strcmp(string(confirmations.MotionPhrase), ...
    string(cfg.robot.MotionConfirmationPhrase));
gates.SetupPhraseMatches = strcmp(string(confirmations.SetupPhrase), ...
    string(c.SetupConfirmationPhrase));
gates.ToolGeometryVerified = cfg.tool.GeometryVerified;
gates.RobotModelVerified = cfg.robot.ModelVerified;
gates.ServoTimingVerified = cfg.robot.ServoTimingVerified;
gates.ForceParametersValidated = cfg.force.ParametersValidated;
gates.SafetyThresholdsValidatedOrConfirmed = safetyConfirmed;
gates.JointLimitsValidated = cfg.safety.JointLimitsValidated;
gates.EmergencyStopAttested = cfg.stage08.EmergencyStopAttested;
gates.SecondObserverPresent = ~c.RequireSecondObserver || ...
    confirmations.SecondObserverPresent;
gates.FixtureAndClearanceConfirmed = ...
    ~c.RequireFixtureAndClearanceConfirmation || ...
    confirmations.FixtureAndClearanceConfirmed;
gates.KeyboardLatchModeSelected = ...
    string(cfg.stage08.KeyboardInputMode) == "space_on_escape_off";
gates.DofModeAllowed = any(string(c.DofMode) == ...
    string(c.AllowedDofModes));
gates.DragPointTransformFinite = ...
    all(isfinite(c.TFlangeDragPoint(:)));

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
authorization.DofMode = string(c.DofMode);
authorization.CartesianDofMode = string(c.DofMode);
authorization.ControllerFamily = "cartesian_drag";
authorization.SafetyThresholdRuntimeConfirmationUsed = ...
    ~cfg.safety.ThresholdsValidated && safetyConfirmed;
authorization.KeyboardInputIsNotPhysicalDeadman = true;
authorization.RcmGatesEvaluated = false;
authorization.DoesNotSendHardwareCommands = true;
end

function logicalConfirmation(value, name)
if ~islogical(value) || ~isscalar(value)
    error('scopeguide:cartesianDrag:InvalidConfirmation', ...
        '%s must be a scalar logical.', name);
end
end
