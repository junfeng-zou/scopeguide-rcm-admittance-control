classdef JointCommandIntegrator < handle
    %JOINTCOMMANDINTEGRATOR Fail-safe measured-dt joint target integrator.
    % It applies soft-start, velocity/acceleration limits and a bounded
    % target-to-state error.  Stop/fault paths reanchor and clear all stored
    % rate state.  The output never authorizes a hardware command.

    properties (SetAccess = private)
        Config
        Initialized (1, 1) logical = false
        TargetRad (6, 1) double = nan(6, 1)
        PreviousAppliedVelocityRadSec (6, 1) double = zeros(6, 1)
        PreviousTimeSec (1, 1) double = NaN
    end

    methods
        function obj = JointCommandIntegrator(cfg)
            arguments
                cfg (1, 1) struct
            end
            validateRcmAdmittanceConfig(cfg);
            obj.Config = cfg;
        end

        function result = step(obj, qStateRad, qdotRequestedRadSec, ...
                enableStatus, nowSec)
            arguments
                obj
                qStateRad
                qdotRequestedRadSec
                enableStatus (1, 1) struct
                nowSec (1, 1) double
            end
            result = defaultResult();
            if ~validSixVector(qStateRad) || ~isfinite(nowSec)
                obj.clearInternalState();
                result.StatusCode = "INVALID_STATE_OR_TIME";
                result.RequiresFault = true;
                return;
            end
            qStateRad = double(qStateRad(:));
            if ~validSixVector(qdotRequestedRadSec)
                obj.reanchor(qStateRad, nowSec);
                result = anchoredResult(obj, "INVALID_QDOT_REQUEST");
                result.RequiresFault = true;
                return;
            end
            qdotRequestedRadSec = double(qdotRequestedRadSec(:));
            validateEnableStatus(enableStatus);

            if ~obj.Initialized
                obj.reanchor(qStateRad, nowSec);
                result = anchoredResult(obj, "INITIAL_ANCHOR");
                return;
            end

            dtSec = nowSec - obj.PreviousTimeSec;
            if ~isfinite(dtSec) || dtSec <= 0 || ...
                    dtSec > obj.Config.runtime.MaximumDtSec
                obj.reanchor(qStateRad, nowSec);
                result = anchoredResult(obj, "CONTROL_PERIOD_INVALID");
                result.DtSec = dtSec;
                result.RequiresFault = true;
                return;
            end

            if ~enableStatus.MotionPermitted || ...
                    enableStatus.ResetDynamicState
                obj.reanchor(qStateRad, nowSec);
                result = anchoredResult(obj, "REANCHORED_STOP_PATH");
                result.DtSec = dtSec;
                return;
            end

            maximumVelocity = ...
                obj.Config.control.MaximumJointCommandRadSec(:);
            velocityLimited = min(max(qdotRequestedRadSec, ...
                -maximumVelocity), maximumVelocity);
            desiredVelocity = velocityLimited * ...
                min(max(double(enableStatus.CommandScale), 0), 1);
            maximumVelocityDelta = ...
                obj.Config.control.MaximumJointAccelerationRadSec2(:) * ...
                dtSec;
            velocityDelta = desiredVelocity - ...
                obj.PreviousAppliedVelocityRadSec;
            velocityDeltaLimited = min(max(velocityDelta, ...
                -maximumVelocityDelta), maximumVelocityDelta);
            rateLimitedVelocity = ...
                obj.PreviousAppliedVelocityRadSec + velocityDeltaLimited;

            previousTarget = obj.TargetRad;
            proposedTarget = previousTarget + rateLimitedVelocity * dtSec;
            maximumError = ...
                obj.Config.control.MaximumTargetStateErrorRad(:);
            proposedError = proposedTarget - qStateRad;
            limitedError = min(max(proposedError, ...
                -maximumError), maximumError);
            obj.TargetRad = qStateRad + limitedError;
            appliedVelocity = (obj.TargetRad - previousTarget) / dtSec;
            obj.PreviousAppliedVelocityRadSec = appliedVelocity;
            obj.PreviousTimeSec = nowSec;

            result.QTargetRad = obj.TargetRad;
            result.QdotAppliedRadSec = appliedVelocity;
            result.DtSec = dtSec;
            result.Reanchored = false;
            result.VelocityClamped = any(abs(velocityLimited - ...
                qdotRequestedRadSec) > 10 * eps);
            result.AccelerationClamped = any(abs(velocityDeltaLimited - ...
                velocityDelta) > 10 * eps);
            result.TargetErrorClamped = any(abs(limitedError - ...
                proposedError) > 10 * eps);
            result.IsValid = true;
            result.StatusCode = "OK";
            result.RequiresFault = false;
        end

        function reanchor(obj, qStateRad, nowSec)
            arguments
                obj
                qStateRad
                nowSec (1, 1) double
            end
            if ~validSixVector(qStateRad) || ~isfinite(nowSec)
                error('scopeguide:control:InvalidIntegratorAnchor', ...
                    'Anchor state and time must be finite.');
            end
            obj.TargetRad = double(qStateRad(:));
            obj.PreviousAppliedVelocityRadSec = zeros(6, 1);
            obj.PreviousTimeSec = nowSec;
            obj.Initialized = true;
        end

        function reset(obj)
            obj.clearInternalState();
        end
    end

    methods (Access = private)
        function clearInternalState(obj)
            obj.Initialized = false;
            obj.TargetRad = nan(6, 1);
            obj.PreviousAppliedVelocityRadSec = zeros(6, 1);
            obj.PreviousTimeSec = NaN;
        end
    end
end

function result = defaultResult()
result = struct();
result.QTargetRad = nan(6, 1);
result.QdotAppliedRadSec = zeros(6, 1);
result.DtSec = NaN;
result.Reanchored = true;
result.VelocityClamped = false;
result.AccelerationClamped = false;
result.TargetErrorClamped = false;
result.IsValid = false;
result.StatusCode = "UNINITIALIZED";
result.RequiresFault = false;
result.HardwareCommandAuthorized = false;
end

function result = anchoredResult(obj, statusCode)
result = defaultResult();
result.QTargetRad = obj.TargetRad;
result.QdotAppliedRadSec = zeros(6, 1);
result.Reanchored = true;
result.IsValid = true;
result.StatusCode = statusCode;
end

function valid = validSixVector(value)
valid = isnumeric(value) && isreal(value) && numel(value) == 6 && ...
    all(isfinite(value(:)));
end

function validateEnableStatus(status)
required = {'MotionPermitted', 'ResetDynamicState', 'CommandScale'};
for index = 1:numel(required)
    if ~isfield(status, required{index})
        error('scopeguide:control:IncompleteEnableStatus', ...
            'Enable status is missing %s.', required{index});
    end
end
if ~islogical(status.MotionPermitted) || ...
        ~isscalar(status.MotionPermitted) || ...
        ~islogical(status.ResetDynamicState) || ...
        ~isscalar(status.ResetDynamicState) || ...
        ~isnumeric(status.CommandScale) || ...
        ~isscalar(status.CommandScale) || ...
        ~isfinite(status.CommandScale)
    error('scopeguide:control:InvalidEnableStatus', ...
        'Enable status fields have invalid types or values.');
end
end
