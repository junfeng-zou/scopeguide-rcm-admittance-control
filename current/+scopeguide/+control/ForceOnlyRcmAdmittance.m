classdef ForceOnlyRcmAdmittance < handle
    %FORCEONLYRCMAdMITTANCE Offline-safe 3-DOF wrench admittance.
    % Generalized state order is [pivot1; pivot2; insertion].  Roll is
    % structurally absent and every public output reports it as exactly zero.
    % The historical class name is retained for API/result compatibility;
    % pivot input now uses the full six-axis wrench shifted to the RCM.

    properties (SetAccess = private)
        Config
        Parameters
        GeneralizedVelocity (3, 1) double = zeros(3, 1)
        RelativeCoordinate (3, 1) double = zeros(3, 1)
        StepCount (1, 1) uint64 = uint64(0)
        ResetCount (1, 1) uint64 = uint64(0)
    end

    methods
        function obj = ForceOnlyRcmAdmittance(cfg)
            arguments
                cfg (1, 1) struct
            end
            validateRcmAdmittanceConfig(cfg);
            obj.Config = cfg;
            obj.Parameters = ...
                scopeguide.control.deriveForceOnlyAdmittanceParameters(cfg);
        end

        function reset(obj)
            obj.GeneralizedVelocity(:) = 0;
            obj.RelativeCoordinate(:) = 0;
            obj.ResetCount = obj.ResetCount + 1;
        end

        function output = step(obj, wrenchToolAtSensorOrigin, ...
                TBaseEndoscope, ...
                pRcmBaseM, dtSec, enableStatus)
            arguments
                obj
                wrenchToolAtSensorOrigin
                TBaseEndoscope
                pRcmBaseM
                dtSec (1, 1) double
                enableStatus (1, 1) struct
            end
            obj.StepCount = obj.StepCount + 1;
            [statusValid, statusCode] = validateEnableStatus(enableStatus);
            periodValid = isfinite(dtSec) && dtSec > 0 && ...
                dtSec <= obj.Config.runtime.MaximumDtSec;
            if ~statusValid || ~periodValid
                obj.reset();
                if ~periodValid
                    statusCode = "INVALID_CONTROL_PERIOD";
                end
                output = obj.zeroOutput(statusCode, true, false);
                return;
            end
            if ~enableStatus.MotionPermitted || ...
                    enableStatus.ResetDynamicState
                obj.reset();
                output = obj.zeroOutput("DYNAMIC_STATE_RESET", ...
                    false, true);
                return;
            end

            try
                mapped = scopeguide.control.mapWrenchToRcmGeneralizedEffort( ...
                    wrenchToolAtSensorOrigin, TBaseEndoscope, ...
                    pRcmBaseM, obj.Config);
            catch exception
                if startsWith(string(exception.identifier), "scopeguide:")
                    obj.reset();
                    output = obj.zeroOutput("INVALID_ADMITTANCE_INPUT", ...
                        true, false);
                    output.InputErrorIdentifier = string(exception.identifier);
                    return;
                end
                rethrow(exception);
            end

            scale = min(max(double(enableStatus.CommandScale), 0), 1);
            generalizedEffort = scale * mapped.GeneralizedEffort;
            previousVelocity = obj.GeneralizedVelocity;
            previousCoordinate = obj.RelativeCoordinate;
            accelerationRequested = (generalizedEffort - ...
                obj.Parameters.Damping .* previousVelocity) ./ ...
                obj.Parameters.VirtualMass;
            [accelerationLimited, accelerationClamped] = ...
                limitAcceleration(accelerationRequested, ...
                obj.Parameters.MaximumAcceleration);
            velocity = previousVelocity + accelerationLimited * dtSec;

            [velocity, speedClamped] = limitTaskSpeed(velocity, ...
                obj.Parameters.MaximumVelocity);
            tipSpeedLimit = obj.Parameters.MaximumPivotTipSpeedMSec / ...
                max(mapped.Basis.RcmToTipDistanceM, eps);
            pivotLimit = min(obj.Parameters.MaximumVelocity(1), ...
                tipSpeedLimit);
            [velocity(1:2), tipSpeedClamped] = ...
                limitVectorNorm(velocity(1:2), pivotLimit);

            [velocity, boundaryActive] = limitForRelativeTravel( ...
                velocity, previousCoordinate, ...
                obj.Parameters.MaximumRelativePosition, ...
                obj.Parameters.MaximumAcceleration, dtSec);
            coordinate = previousCoordinate + velocity * dtSec;
            coordinate = enforceNumericTravelBounds(coordinate, ...
                obj.Parameters.MaximumRelativePosition);
            % Synchronize all limited values back into the actual state.
            velocity = (coordinate - previousCoordinate) / dtSec;
            zeroMask = abs(velocity) <= ...
                obj.Parameters.VelocityZeroTolerance & ...
                abs(generalizedEffort) <= ...
                obj.Parameters.Damping .* ...
                obj.Parameters.VelocityZeroTolerance;
            velocity(zeroMask) = 0;
            coordinate = previousCoordinate + velocity * dtSec;
            obj.GeneralizedVelocity = velocity;
            obj.RelativeCoordinate = coordinate;

            output = struct();
            output.StatusCode = "OK";
            output.Valid = true;
            output.RequiresFault = false;
            output.DynamicStateReset = false;
            output.CommandScale = scale;
            output.GeneralizedEffort = generalizedEffort;
            output.UnscaledGeneralizedEffort = mapped.GeneralizedEffort;
            output.GeneralizedVelocity = velocity;
            output.GeneralizedVelocityWithRoll = [velocity; 0];
            output.RelativeCoordinate = coordinate;
            output.RelativeCoordinateWithRoll = [coordinate; 0];
            output.GeneralizedAcceleration = ...
                (velocity - previousVelocity) / dtSec;
            output.DesiredTipTwistBase = ...
                mapped.Basis.TwistBasisBase * velocity;
            output.RollRateRadSec = 0;
            output.RollCoordinateRad = 0;
            output.MomentUsedForControl = true;
            output.AccelerationClamped = accelerationClamped;
            output.SpeedClamped = speedClamped;
            output.TipSpeedClamped = tipSpeedClamped;
            output.TravelBoundaryActive = boundaryActive;
            output.EffectivePivotSpeedLimitRadSec = pivotLimit;
            output.PivotTipLinearSpeedMSec = ...
                norm(velocity(1:2)) * ...
                mapped.Basis.RcmToTipDistanceM;
            output.Mapping = mapped;
            output.Parameters = obj.Parameters;
            output.InputErrorIdentifier = "";
            output.DoesNotSendHardwareCommands = true;
        end
    end

    methods (Access = private)
        function output = zeroOutput(obj, code, requiresFault, resetApplied)
            output = struct();
            output.StatusCode = string(code);
            output.Valid = ~requiresFault;
            output.RequiresFault = logical(requiresFault);
            output.DynamicStateReset = logical(resetApplied);
            output.CommandScale = 0;
            output.GeneralizedEffort = zeros(3, 1);
            output.UnscaledGeneralizedEffort = zeros(3, 1);
            output.GeneralizedVelocity = zeros(3, 1);
            output.GeneralizedVelocityWithRoll = zeros(4, 1);
            output.RelativeCoordinate = zeros(3, 1);
            output.RelativeCoordinateWithRoll = zeros(4, 1);
            output.GeneralizedAcceleration = zeros(3, 1);
            output.DesiredTipTwistBase = zeros(6, 1);
            output.RollRateRadSec = 0;
            output.RollCoordinateRad = 0;
            output.MomentUsedForControl = false;
            output.AccelerationClamped = false;
            output.SpeedClamped = false;
            output.TipSpeedClamped = false;
            output.TravelBoundaryActive = false;
            output.EffectivePivotSpeedLimitRadSec = 0;
            output.PivotTipLinearSpeedMSec = 0;
            output.Mapping = struct([]);
            output.Parameters = obj.Parameters;
            output.InputErrorIdentifier = "";
            output.DoesNotSendHardwareCommands = true;
        end
    end
end

function [valid, code] = validateEnableStatus(status)
required = {'MotionPermitted', 'ResetDynamicState', 'CommandScale'};
valid = isstruct(status) && all(isfield(status, required));
code = "INVALID_ENABLE_STATUS";
if ~valid
    return;
end
valid = islogical(status.MotionPermitted) && ...
    isscalar(status.MotionPermitted) && ...
    islogical(status.ResetDynamicState) && ...
    isscalar(status.ResetDynamicState) && ...
    isnumeric(status.CommandScale) && isscalar(status.CommandScale) && ...
    isfinite(status.CommandScale) && status.CommandScale >= 0 && ...
    status.CommandScale <= 1;
if valid
    code = "OK";
end
end

function [limited, clamped] = limitAcceleration(value, limits)
limited = double(value(:));
[limited(1:2), pivotClamped] = ...
    limitVectorNorm(limited(1:2), limits(1));
insertionBefore = limited(3);
limited(3) = min(max(limited(3), -limits(3)), limits(3));
clamped = pivotClamped || limited(3) ~= insertionBefore;
end

function [limited, clamped] = limitTaskSpeed(value, limits)
limited = double(value(:));
[limited(1:2), pivotClamped] = ...
    limitVectorNorm(limited(1:2), limits(1));
insertionBefore = limited(3);
limited(3) = min(max(limited(3), -limits(3)), limits(3));
clamped = pivotClamped || limited(3) ~= insertionBefore;
end

function [value, clamped] = limitVectorNorm(value, maximumNorm)
currentNorm = norm(value);
clamped = currentNorm > maximumNorm;
if clamped
    value = value * (maximumNorm / currentNorm);
end
end

function [velocity, active] = limitForRelativeTravel(velocity, ...
        coordinate, limits, accelerationLimits, dtSec)
active = false;

pivotSpeed = norm(velocity(1:2));
if pivotSpeed > 0
    direction = velocity(1:2) / pivotSpeed;
    position = coordinate(1:2);
    positionNorm = norm(position);
    if positionNorm == 0 || dot(position, direction) >= 0
        remaining = max(limits(1) - positionNorm, 0);
        safeSpeed = stoppingSafeSpeed(remaining, ...
            accelerationLimits(1), dtSec);
        geometricSpeed = maximumBallSpeed(position, direction, ...
            limits(1), dtSec);
        allowed = min(safeSpeed, geometricSpeed);
        if pivotSpeed > allowed
            velocity(1:2) = direction * allowed;
            active = true;
        end
    else
        geometricSpeed = maximumBallSpeed(position, direction, ...
            limits(1), dtSec);
        if pivotSpeed > geometricSpeed
            velocity(1:2) = direction * geometricSpeed;
            active = true;
        end
    end
end

if velocity(3) ~= 0
    direction = sign(velocity(3));
    outward = coordinate(3) == 0 || ...
        sign(coordinate(3)) == direction;
    geometricSpeed = max((limits(3) - ...
        direction * coordinate(3)) / dtSec, 0);
    allowed = geometricSpeed;
    if outward
        remaining = max(limits(3) - abs(coordinate(3)), 0);
        allowed = min(allowed, stoppingSafeSpeed(remaining, ...
            accelerationLimits(3), dtSec));
    end
    if abs(velocity(3)) > allowed
        velocity(3) = direction * allowed;
        active = true;
    end
end
end

function speed = stoppingSafeSpeed(remaining, acceleration, dtSec)
speed = -acceleration * dtSec + sqrt( ...
    (acceleration * dtSec)^2 + 2 * acceleration * remaining);
speed = max(speed, 0);
end

function speed = maximumBallSpeed(position, direction, limit, dtSec)
projection = dot(position, direction);
discriminant = projection^2 + limit^2 - dot(position, position);
speed = max((-projection + sqrt(max(discriminant, 0))) / dtSec, 0);
end

function coordinate = enforceNumericTravelBounds(coordinate, limits)
pivotNorm = norm(coordinate(1:2));
if pivotNorm > limits(1)
    coordinate(1:2) = coordinate(1:2) * limits(1) / pivotNorm;
end
coordinate(3) = min(max(coordinate(3), -limits(3)), limits(3));
end
