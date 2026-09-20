classdef CartesianAdmittance6D < handle
    %CARTESIANADMITTANCE6D Diagonal 6D admittance without an RCM model.
    % State and input order is [vx vy vz wx wy wz] in drag-point axes.

    properties (SetAccess = private)
        Config
        Parameters
        DofMask (6, 1) logical
        VelocityDrag (6, 1) double = zeros(6, 1)
        StepCount (1, 1) uint64 = uint64(0)
        ResetCount (1, 1) uint64 = uint64(0)
    end

    methods
        function obj = CartesianAdmittance6D(cfg)
            arguments
                cfg (1, 1) struct
            end
            validateCartesianAdmittanceDragConfig(cfg);
            obj.Config = cfg;
            obj.Parameters = ...
                scopeguide.control.deriveCartesianAdmittanceParameters(cfg);
            obj.DofMask = scopeguide.control.cartesianDragDofMask( ...
                cfg.cartesianDrag.DofMode);
        end

        function reset(obj)
            obj.VelocityDrag(:) = 0;
            obj.ResetCount = obj.ResetCount + uint64(1);
        end

        function output = step(obj, wrenchDrag, rotationBaseFromDrag, ...
                dtSec, enableStatus)
            obj.StepCount = obj.StepCount + uint64(1);
            if ~validEnable(enableStatus)
                obj.reset();
                output = obj.zeroOutput("INVALID_ENABLE_STATUS", true);
                return;
            end
            if ~isfinite(dtSec) || dtSec <= 0 || ...
                    dtSec > obj.Config.runtime.MaximumDtSec
                obj.reset();
                output = obj.zeroOutput("INVALID_CONTROL_PERIOD", true);
                return;
            end
            if ~enableStatus.MotionPermitted || ...
                    enableStatus.ResetDynamicState
                obj.reset();
                output = obj.zeroOutput("DYNAMIC_STATE_RESET", false);
                return;
            end
            if ~validSix(wrenchDrag) || ...
                    ~validRotation(rotationBaseFromDrag)
                obj.reset();
                output = obj.zeroOutput("INVALID_ADMITTANCE_INPUT", true);
                return;
            end

            wrench = double(wrenchDrag(:));
            [wrench(1:3), forceClamped] = limitNorm( ...
                wrench(1:3), ...
                obj.Config.cartesianDrag.ControlForceSaturationN);
            [wrench(4:6), momentClamped] = limitNorm( ...
                wrench(4:6), ...
                obj.Config.cartesianDrag.ControlMomentSaturationNm);
            scale = min(max(double(enableStatus.CommandScale), 0), 1);
            effort = scale * wrench .* double(obj.DofMask);
            previous = obj.VelocityDrag;
            accelerationRequested = (effort - ...
                obj.Parameters.Damping .* previous) ./ ...
                obj.Parameters.VirtualMass;
            accelerationRequested(~obj.DofMask) = 0;
            acceleration = accelerationRequested;
            [acceleration(1:3), translationAccelerationClamped] = ...
                limitNorm(acceleration(1:3), ...
                obj.Parameters.MaximumAcceleration(1));
            [acceleration(4:6), rotationAccelerationClamped] = ...
                limitNorm(acceleration(4:6), ...
                obj.Parameters.MaximumAcceleration(4));
            velocity = previous + acceleration * dtSec;
            [velocity(1:3), translationSpeedClamped] = limitNorm( ...
                velocity(1:3), obj.Parameters.MaximumVelocity(1));
            [velocity(4:6), rotationSpeedClamped] = limitNorm( ...
                velocity(4:6), obj.Parameters.MaximumVelocity(4));
            velocity(~obj.DofMask) = 0;
            zeroMask = abs(velocity) <= ...
                obj.Parameters.VelocityZeroTolerance & ...
                abs(effort) <= obj.Parameters.Damping .* ...
                obj.Parameters.VelocityZeroTolerance;
            velocity(zeroMask) = 0;
            obj.VelocityDrag = velocity;

            rotation = double(rotationBaseFromDrag);
            desiredTwistBase = [rotation * velocity(1:3); ...
                rotation * velocity(4:6)];
            output = struct();
            output.StatusCode = "OK";
            output.Valid = true;
            output.RequiresFault = false;
            output.DynamicStateReset = false;
            output.CommandScale = scale;
            output.DofMask = obj.DofMask;
            output.UnscaledWrenchDrag = double(wrenchDrag(:));
            output.SaturatedWrenchDrag = wrench;
            output.AppliedEffortDrag = effort;
            output.VelocityDrag = velocity;
            output.AccelerationDrag = (velocity - previous) / dtSec;
            output.DesiredTwistBase = desiredTwistBase;
            output.ForceClamped = forceClamped;
            output.MomentClamped = momentClamped;
            output.TranslationAccelerationClamped = ...
                translationAccelerationClamped;
            output.RotationAccelerationClamped = ...
                rotationAccelerationClamped;
            output.TranslationSpeedClamped = translationSpeedClamped;
            output.RotationSpeedClamped = rotationSpeedClamped;
            output.Parameters = obj.Parameters;
            output.NoRcmConstraint = true;
            output.DoesNotSendHardwareCommands = true;
        end
    end

    methods (Access = private)
        function output = zeroOutput(obj, code, requiresFault)
            output = struct();
            output.StatusCode = string(code);
            output.Valid = ~requiresFault;
            output.RequiresFault = logical(requiresFault);
            output.DynamicStateReset = true;
            output.CommandScale = 0;
            output.DofMask = obj.DofMask;
            output.UnscaledWrenchDrag = zeros(6, 1);
            output.SaturatedWrenchDrag = zeros(6, 1);
            output.AppliedEffortDrag = zeros(6, 1);
            output.VelocityDrag = zeros(6, 1);
            output.AccelerationDrag = zeros(6, 1);
            output.DesiredTwistBase = zeros(6, 1);
            output.ForceClamped = false;
            output.MomentClamped = false;
            output.TranslationAccelerationClamped = false;
            output.RotationAccelerationClamped = false;
            output.TranslationSpeedClamped = false;
            output.RotationSpeedClamped = false;
            output.Parameters = obj.Parameters;
            output.NoRcmConstraint = true;
            output.DoesNotSendHardwareCommands = true;
        end
    end
end

function valid = validEnable(status)
required = {'MotionPermitted', 'ResetDynamicState', 'CommandScale'};
valid = isstruct(status) && all(isfield(status, required)) && ...
    islogical(status.MotionPermitted) && isscalar(status.MotionPermitted) && ...
    islogical(status.ResetDynamicState) && ...
    isscalar(status.ResetDynamicState) && ...
    isnumeric(status.CommandScale) && isscalar(status.CommandScale) && ...
    isfinite(status.CommandScale) && status.CommandScale >= 0 && ...
    status.CommandScale <= 1;
end

function valid = validSix(value)
valid = isnumeric(value) && numel(value) == 6 && ...
    all(isfinite(value(:)));
end

function valid = validRotation(value)
valid = isnumeric(value) && isequal(size(value), [3, 3]) && ...
    all(isfinite(value(:))) && ...
    norm(double(value)' * double(value) - eye(3), 'fro') <= 1e-8 && ...
    abs(det(double(value)) - 1) <= 1e-8;
end

function [limited, clamped] = limitNorm(value, maximum)
current = norm(value);
clamped = current > maximum;
if clamped
    limited = value * maximum / current;
else
    limited = value;
end
end
