function [twistBase, diagnostics] = limitCartesianTwistForTravel( ...
        requestedTwistBase, relativeCoordinate, ...
        rotationBaseFromAnchor, dtSec, cfg)
%LIMITCARTESIANTWISTFORTRAVEL Braking-aware no-RCM workspace limiter.
% Relative coordinates and limits are axis-aligned in the enable-anchor
% frame.  This limiter acts before the QP; the QP repeats the pose bound as
% a hard one-step inequality.

validateCartesianAdmittanceDragConfig(cfg);
twist = validateSix(requestedTwistBase, 'requestedTwistBase');
relative = validateSix(relativeCoordinate, 'relativeCoordinate');
rotation = validateRotation(rotationBaseFromAnchor);
if ~isfinite(dtSec) || dtSec <= 0 || ...
        dtSec > cfg.runtime.MaximumDtSec
    error('scopeguide:cartesianDrag:InvalidTravelLimiterPeriod', ...
        'dtSec is outside the configured control-period range.');
end

twistAnchor = [rotation' * twist(1:3); rotation' * twist(4:6)];
requestedAnchor = twistAnchor;
limits = [double(cfg.cartesianDrag.Translation.RelativeLimitM(:)); ...
    double(cfg.cartesianDrag.Rotation.RelativeLimitRad(:))];
recoveryTolerance = [ ...
    cfg.cartesianDrag.Translation.RecoveryToleranceM * ones(3, 1); ...
    cfg.cartesianDrag.Rotation.RecoveryToleranceRad * ones(3, 1)];
hardLimits = limits + recoveryTolerance;
acceleration = [ ...
    cfg.cartesianDrag.Translation.MaximumAccelerationMSec2 * ones(3, 1); ...
    cfg.cartesianDrag.Rotation.MaximumAccelerationRadSec2 * ones(3, 1)];
active = false(6, 1);
travelLimitsEnabled = logical(cfg.cartesianDrag.TravelLimitsEnabled);
if travelLimitsEnabled
    for index = 1:6
        rate = twistAnchor(index);
        if rate == 0
            continue;
        end
        direction = sign(rate);
        remainingGeometric = max( ...
            limits(index) - direction * relative(index), 0);
        allowed = remainingGeometric / dtSec;
        outward = relative(index) == 0 || ...
            sign(relative(index)) == direction;
        if outward
            remaining = max(limits(index) - abs(relative(index)), 0);
            allowed = min(allowed, stoppingSafeSpeed( ...
                remaining, acceleration(index), dtSec));
        end
        if abs(rate) > allowed
            twistAnchor(index) = direction * allowed;
            active(index) = true;
        end
    end
end
twistBase = [rotation * twistAnchor(1:3); ...
    rotation * twistAnchor(4:6)];
diagnostics = struct();
diagnostics.RequestedTwistAnchor = requestedAnchor;
diagnostics.LimitedTwistAnchor = twistAnchor;
diagnostics.ActiveAxes = active;
diagnostics.AnyBoundaryActive = any(active);
diagnostics.TravelLimitsEnabled = travelLimitsEnabled;
diagnostics.RelativeCoordinate = relative;
diagnostics.RelativeLimit = limits;
diagnostics.RecoveryTolerance = recoveryTolerance;
diagnostics.HardRelativeLimit = hardLimits;
diagnostics.RecoveryActiveAxes = travelLimitsEnabled & ...
    abs(relative) > limits;
diagnostics.AnyRecoveryActive = any(diagnostics.RecoveryActiveAxes);
diagnostics.OutsideHardBoundaryAxes = travelLimitsEnabled & ...
    abs(relative) > hardLimits;
diagnostics.AnyOutsideHardBoundary = ...
    any(diagnostics.OutsideHardBoundaryAxes);
diagnostics.NoRcmConstraint = true;
end

function speed = stoppingSafeSpeed(remaining, acceleration, dtSec)
speed = -acceleration * dtSec + sqrt( ...
    (acceleration * dtSec)^2 + 2 * acceleration * remaining);
speed = max(speed, 0);
end

function value = validateSix(input, name)
if ~isnumeric(input) || numel(input) ~= 6 || ...
        any(~isfinite(input(:)))
    error('scopeguide:cartesianDrag:InvalidTravelLimiterInput', ...
        '%s must contain six finite numeric values.', name);
end
value = double(input(:));
end

function rotation = validateRotation(input)
rotation = double(input);
if ~isequal(size(rotation), [3, 3]) || any(~isfinite(rotation(:))) || ...
        norm(rotation' * rotation - eye(3), 'fro') > 1e-8 || ...
        abs(det(rotation) - 1) > 1e-8
    error('scopeguide:cartesianDrag:InvalidAnchorRotation', ...
        'rotationBaseFromAnchor must be a valid rotation matrix.');
end
end
