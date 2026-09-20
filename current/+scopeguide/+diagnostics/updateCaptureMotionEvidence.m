function [state, evidence] = updateCaptureMotionEvidence( ...
        state, snapshot, options)
%UPDATECAPTUREMOTIONEVIDENCE Confirm motion from independent feedback.
% QDActual alone is retained as diagnostic evidence but cannot abort a
% static capture. Motion is confirmed by TCP speed or by finite differences
% of actual joint/TCP pose across consecutive unique feedback frames.

arguments
    state (1, 1) struct
    snapshot (1, 1) struct
    options.MaximumJointSpeedDegSec (1, 1) double = 0.15
    options.MaximumTcpTranslationSpeedMmSec (1, 1) double = 0.20
    options.MaximumTcpRotationSpeedDegSec (1, 1) double = 0.20
    options.DerivedSpeedThresholdMultiplier (1, 1) double = 1.5
end

validateThresholds(options);
state = normalizeState(state);
required = {'feedbackSequence','hostMonotonicSec','jointAnglesDeg', ...
    'cartesianPose','actualQuaternionWxyz','actualJointSpeedsDegSec', ...
    'actualTCPSpeed'};
for index = 1:numel(required)
    if ~isfield(snapshot, required{index})
        error('scopeguide:captureMotion:InvalidSnapshot', ...
            'snapshot is missing %s.', required{index});
    end
end

evidence = struct();
evidence.IsNewFeedback = ...
    uint64(snapshot.feedbackSequence) ~= state.LastSequence;
evidence.ReportedJointSpeedDegSec = ...
    max(abs(double(snapshot.actualJointSpeedsDegSec)));
evidence.ReportedTcpTranslationSpeed = ...
    max(abs(double(snapshot.actualTCPSpeed(1:3))));
evidence.ReportedTcpRotationSpeed = ...
    max(abs(double(snapshot.actualTCPSpeed(4:6))));
evidence.DerivedJointSpeedDegSec = 0;
evidence.DerivedTcpTranslationSpeedMmSec = 0;
evidence.DerivedTcpRotationSpeedDegSec = 0;
evidence.ConfirmedMoving = false;
evidence.ConsecutiveMovingFrames = state.ConsecutiveMovingFrames;
if ~evidence.IsNewFeedback
    return;
end

sequence = uint64(snapshot.feedbackSequence);
hostTime = double(snapshot.hostMonotonicSec);
joints = double(snapshot.jointAnglesDeg(:).');
cartesian = double(snapshot.cartesianPose(:).');
quaternion = double(snapshot.actualQuaternionWxyz(:).');
if ~isfinite(hostTime) || numel(joints) ~= 6 || numel(cartesian) ~= 6 || ...
        numel(quaternion) ~= 4 || any(~isfinite(joints)) || ...
        any(~isfinite(cartesian)) || any(~isfinite(quaternion))
    error('scopeguide:captureMotion:InvalidSnapshot', ...
        'Feedback pose/timing fields must be finite.');
end

if state.LastSequence ~= 0
    dt = hostTime - state.LastHostMonotonicSec;
    if ~isfinite(dt) || dt <= 0
        error('scopeguide:captureMotion:InvalidFeedbackTiming', ...
            'Unique feedback timestamps must increase.');
    end
    evidence.DerivedJointSpeedDegSec = ...
        max(abs(joints - state.LastJointAnglesDeg)) / dt;
    evidence.DerivedTcpTranslationSpeedMmSec = ...
        norm(cartesian(1:3) - state.LastCartesianPose(1:3)) / dt;
    previousRotation = ...
        scopeguide.geometry.rotationMatrixFromQuaternionWxyz( ...
        state.LastQuaternionWxyz);
    currentRotation = ...
        scopeguide.geometry.rotationMatrixFromQuaternionWxyz(quaternion);
    evidence.DerivedTcpRotationSpeedDegSec = rad2deg( ...
        scopeguide.geometry.rotationDistance( ...
        previousRotation, currentRotation)) / dt;

    multiplier = options.DerivedSpeedThresholdMultiplier;
    evidence.ConfirmedMoving = ...
        evidence.ReportedTcpTranslationSpeed > ...
            options.MaximumTcpTranslationSpeedMmSec || ...
        evidence.ReportedTcpRotationSpeed > ...
            options.MaximumTcpRotationSpeedDegSec || ...
        evidence.DerivedJointSpeedDegSec > multiplier * ...
            options.MaximumJointSpeedDegSec || ...
        evidence.DerivedTcpTranslationSpeedMmSec > multiplier * ...
            options.MaximumTcpTranslationSpeedMmSec || ...
        evidence.DerivedTcpRotationSpeedDegSec > multiplier * ...
            options.MaximumTcpRotationSpeedDegSec;
end

if evidence.ConfirmedMoving
    state.ConsecutiveMovingFrames = state.ConsecutiveMovingFrames + 1;
else
    state.ConsecutiveMovingFrames = 0;
end
state.LastSequence = sequence;
state.LastHostMonotonicSec = hostTime;
state.LastJointAnglesDeg = joints;
state.LastCartesianPose = cartesian;
state.LastQuaternionWxyz = quaternion;
evidence.ConsecutiveMovingFrames = state.ConsecutiveMovingFrames;
end

function state = normalizeState(state)
defaults = struct('LastSequence', uint64(0), ...
    'LastHostMonotonicSec', NaN, ...
    'LastJointAnglesDeg', zeros(1, 6), ...
    'LastCartesianPose', zeros(1, 6), ...
    'LastQuaternionWxyz', [1 0 0 0], ...
    'ConsecutiveMovingFrames', 0);
names = fieldnames(defaults);
for index = 1:numel(names)
    if ~isfield(state, names{index})
        state.(names{index}) = defaults.(names{index});
    end
end
end

function validateThresholds(options)
names = fieldnames(options);
for index = 1:numel(names)
    value = options.(names{index});
    if ~isscalar(value) || ~isfinite(value) || value <= 0
        error('scopeguide:captureMotion:InvalidThreshold', ...
            '%s must be finite and positive.', names{index});
    end
end
end
