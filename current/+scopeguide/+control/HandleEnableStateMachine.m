classdef HandleEnableStateMachine < handle
    %HANDLEENABLESTATEMACHINE Fail-safe deadman state machine.
    % The class is pure control logic: it creates no GUI, network client or
    % hardware command.  A rising edge must pass PREARM/debounce; release
    % and every runtime fault remove motion permission immediately.

    properties (SetAccess = private)
        Config
        State (1, 1) string = "DISABLED"
        StateEnteredTimeSec (1, 1) double = NaN
        ReadySinceSec (1, 1) double = NaN
        EnabledSinceSec (1, 1) double = NaN
        LastStepTimeSec (1, 1) double = NaN
        FaultCode (1, 1) string = ""
        ResetRequested (1, 1) logical = false
        StopReason (1, 1) string = "INITIALIZED_DISABLED"
    end

    methods
        function obj = HandleEnableStateMachine(cfg)
            arguments
                cfg (1, 1) struct
            end
            validateRcmAdmittanceConfig(cfg);
            obj.Config = cfg;
        end

        function requestReset(obj)
            % A reset is only consumed after release and health recovery.
            obj.ResetRequested = true;
        end

        function status = step(obj, handleInput, safety, nowSec)
            arguments
                obj
                handleInput (1, 1) struct
                safety (1, 1) struct
                nowSec (1, 1) double
            end
            validateHandleInput(handleInput);
            validateSafetyStatus(safety);
            if ~isfinite(nowSec)
                error('scopeguide:control:InvalidFsmTime', ...
                    'nowSec must be finite.');
            end

            previousState = obj.State;
            if isnan(obj.StateEnteredTimeSec)
                obj.StateEnteredTimeSec = nowSec;
            end
            if isfinite(obj.LastStepTimeSec) && ...
                    nowSec <= obj.LastStepTimeSec
                obj.enterFault("CONTROL_CLOCK_REGRESSION", nowSec);
            end
            obj.LastStepTimeSec = nowSec;

            inputFresh = handleInput.IsValid && ...
                isfinite(handleInput.SampleAgeSec) && ...
                handleInput.SampleAgeSec >= ...
                    -obj.Config.handle.FutureTimestampToleranceSec && ...
                handleInput.SampleAgeSec <= obj.Config.handle.StaleSec;
            requested = inputFresh && handleInput.Enabled;
            runtimeFault = firstRuntimeFault(safety);

            switch obj.State
                case "DISABLED"
                    obj.ReadySinceSec = NaN;
                    obj.EnabledSinceSec = NaN;
                    if requested
                        if strlength(runtimeFault) > 0
                            obj.enterFault(runtimeFault, nowSec);
                        else
                            obj.transitionTo("PREARM", nowSec, ...
                                "RISING_EDGE_PREARM");
                            if safety.NeutralWrench
                                obj.ReadySinceSec = nowSec;
                            end
                        end
                    elseif ~inputFresh
                        obj.StopReason = "HANDLE_NOT_FRESH";
                    else
                        obj.StopReason = "HANDLE_RELEASED";
                    end

                case "PREARM"
                    if ~inputFresh
                        obj.enterFault(handleFaultCode(handleInput), nowSec);
                    elseif ~handleInput.Enabled
                        obj.transitionTo("STOPPING", nowSec, ...
                            "HANDLE_RELEASED_DURING_PREARM");
                    elseif strlength(runtimeFault) > 0
                        obj.enterFault(runtimeFault, nowSec);
                    elseif ~safety.NeutralWrench
                        obj.ReadySinceSec = NaN;
                        obj.StopReason = "PRELOAD_NOT_NEUTRAL";
                    else
                        if isnan(obj.ReadySinceSec)
                            obj.ReadySinceSec = nowSec;
                        end
                        if nowSec - obj.ReadySinceSec >= ...
                                obj.Config.handle.RisingDebounceSec
                            obj.transitionTo("ENABLED", nowSec, ...
                                "PREARM_PASSED");
                            obj.EnabledSinceSec = nowSec;
                        else
                            obj.StopReason = "PREARM_DEBOUNCE";
                        end
                    end

                case "ENABLED"
                    if ~inputFresh
                        obj.enterFault(handleFaultCode(handleInput), nowSec);
                    elseif ~handleInput.Enabled
                        obj.transitionTo("STOPPING", nowSec, ...
                            "HANDLE_RELEASED");
                    elseif strlength(runtimeFault) > 0
                        obj.enterFault(runtimeFault, nowSec);
                    else
                        obj.StopReason = "";
                    end

                case "STOPPING"
                    if ~inputFresh || ~handleInput.Enabled
                        obj.transitionTo("DISABLED", nowSec, ...
                            "STOP_COMPLETE_REANCHORED");
                    else
                        obj.StopReason = "RELEASE_REQUIRED_BEFORE_REARM";
                    end

                case "FAULT"
                    recovered = inputFresh && ~handleInput.Enabled && ...
                        strlength(runtimeFault) == 0 && ...
                        safety.NeutralWrench;
                    if obj.ResetRequested && recovered
                        obj.FaultCode = "";
                        obj.ResetRequested = false;
                        obj.transitionTo("DISABLED", nowSec, ...
                            "EXPLICIT_RESET_ACCEPTED");
                    else
                        obj.StopReason = "FAULT_LATCHED";
                    end

                otherwise
                    obj.enterFault("UNKNOWN_FSM_STATE", nowSec);
            end

            transitioned = obj.State ~= previousState;
            motionPermitted = obj.State == "ENABLED" && inputFresh && ...
                handleInput.Enabled && strlength(firstRuntimeFault(safety)) == 0;
            softStartScale = 0;
            if motionPermitted
                if obj.Config.handle.SoftStartSec == 0
                    softStartScale = 1;
                else
                    softStartScale = min(max( ...
                        (nowSec - obj.EnabledSinceSec) / ...
                        obj.Config.handle.SoftStartSec, 0), 1);
                end
            end

            status = struct();
            status.State = obj.State;
            status.PreviousState = previousState;
            status.Transitioned = transitioned;
            status.MotionPermitted = motionPermitted;
            status.CommandScale = softStartScale;
            status.AllowBaselineUpdate = obj.State == "DISABLED" && ...
                inputFresh && ~handleInput.Enabled && ...
                strlength(firstRuntimeFault(safety)) == 0;
            status.ResetDynamicState = ~motionPermitted || transitioned;
            status.InputFresh = inputFresh;
            status.HandleRequested = requested;
            status.HandleSource = string(handleInput.Source);
            status.InputSupportsPhysicalMotion = ...
                handleInput.SupportsPhysicalMotion;
            status.NeutralWrench = safety.NeutralWrench;
            status.FaultLatched = obj.State == "FAULT";
            status.FaultCode = obj.FaultCode;
            status.ResetRequested = obj.ResetRequested;
            status.StopReason = obj.StopReason;
            status.StateEnteredTimeSec = obj.StateEnteredTimeSec;
            status.EnabledElapsedSec = 0;
            if motionPermitted
                status.EnabledElapsedSec = nowSec - obj.EnabledSinceSec;
            end
            status.DoesNotSendHardwareCommands = true;
        end
    end

    methods (Access = private)
        function transitionTo(obj, state, nowSec, reason)
            obj.State = state;
            obj.StateEnteredTimeSec = nowSec;
            obj.StopReason = reason;
            if state ~= "PREARM"
                obj.ReadySinceSec = NaN;
            end
            if state ~= "ENABLED"
                obj.EnabledSinceSec = NaN;
            end
        end

        function enterFault(obj, code, nowSec)
            obj.State = "FAULT";
            obj.StateEnteredTimeSec = nowSec;
            obj.ReadySinceSec = NaN;
            obj.EnabledSinceSec = NaN;
            obj.FaultCode = code;
            obj.ResetRequested = false;
            obj.StopReason = "FAULT_LATCHED";
        end
    end
end

function validateHandleInput(input)
required = {'Enabled', 'SampleAgeSec', 'Source', ...
    'SupportsPhysicalMotion', 'IsValid', 'StatusCode'};
for index = 1:numel(required)
    if ~isfield(input, required{index})
        error('scopeguide:control:IncompleteHandleInput', ...
            'Handle input is missing %s.', required{index});
    end
end
if ~islogical(input.Enabled) || ~isscalar(input.Enabled) || ...
        ~islogical(input.IsValid) || ~isscalar(input.IsValid) || ...
        ~islogical(input.SupportsPhysicalMotion) || ...
        ~isscalar(input.SupportsPhysicalMotion)
    error('scopeguide:control:InvalidHandleInput', ...
        'Handle logical fields must be scalar logical values.');
end
end

function validateSafetyStatus(status)
required = {'NeutralWrench', 'ForceFresh', 'RobotFresh', ...
    'QpHealthy', 'ControlPeriodHealthy', 'MotionGatesSatisfied'};
for index = 1:numel(required)
    name = required{index};
    if ~isfield(status, name) || ~islogical(status.(name)) || ...
            ~isscalar(status.(name))
        error('scopeguide:control:InvalidEnableSafetyStatus', ...
            'Safety field %s must be a scalar logical.', name);
    end
end
optional = {'ForceControlReady', 'WrenchSafetyHealthy', ...
    'CommandChannelHealthy', 'TrackingHealthy'};
for index = 1:numel(optional)
    name = optional{index};
    if isfield(status, name) && (~islogical(status.(name)) || ...
            ~isscalar(status.(name)))
        error('scopeguide:control:InvalidEnableSafetyStatus', ...
            'Optional safety field %s must be a scalar logical.', name);
    end
end
optionalStrings = {'ForceStatusCode', 'WrenchSafetyStopCode'};
for index = 1:numel(optionalStrings)
    name = optionalStrings{index};
    if isfield(status, name)
        value = string(status.(name));
        if ~isscalar(value) || ismissing(value)
            error('scopeguide:control:InvalidEnableSafetyStatus', ...
                'Optional safety code %s must be a scalar string.', name);
        end
    end
end
end

function code = firstRuntimeFault(safety)
if ~safety.ForceFresh
    if isfield(safety, 'ForceStatusCode') && ...
            strlength(string(safety.ForceStatusCode)) > 0
        code = string(safety.ForceStatusCode);
    else
        code = "FORCE_STALE";
    end
elseif isfield(safety, 'ForceControlReady') && ...
        ~safety.ForceControlReady
    code = "FORCE_BASELINE_NOT_READY";
elseif isfield(safety, 'WrenchSafetyHealthy') && ...
        ~safety.WrenchSafetyHealthy
    if isfield(safety, 'WrenchSafetyStopCode') && ...
            strlength(string(safety.WrenchSafetyStopCode)) > 0
        code = string(safety.WrenchSafetyStopCode);
    else
        code = "WRENCH_SAFETY_STOP";
    end
elseif ~safety.RobotFresh
    code = "ROBOT_FEEDBACK_STALE";
elseif ~safety.QpHealthy
    code = "QP_FAILURE";
elseif ~safety.ControlPeriodHealthy
    code = "CONTROL_PERIOD_OVERRUN";
elseif isfield(safety, 'CommandChannelHealthy') && ...
        ~safety.CommandChannelHealthy
    code = "COMMAND_CHANNEL_FAILURE";
elseif isfield(safety, 'TrackingHealthy') && ~safety.TrackingHealthy
    code = "TARGET_TRACKING_ERROR";
elseif ~safety.MotionGatesSatisfied
    code = "MOTION_GATE_BLOCKED";
else
    code = "";
end
end

function code = handleFaultCode(input)
status = string(input.StatusCode);
if strlength(status) == 0 || status == "OK"
    code = "HANDLE_INVALID";
else
    code = "HANDLE_" + status;
end
end
