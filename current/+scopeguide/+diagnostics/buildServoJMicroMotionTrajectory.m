function trajectory = buildServoJMicroMotionTrajectory( ...
        initialJointDeg, jointIndex, offsetDeg, rateHz, durationsSec)
%BUILDSERVOJMICROMOTIONTRAJECTORY Deterministic tiny out-and-back target.
% durationsSec = [pre-hold, outbound ramp, peak hold, return ramp,
% final hold].  Both ramps use a cubic smoothstep with zero endpoint speed.

arguments
    initialJointDeg (1, 6) double
    jointIndex (1, 1) double
    offsetDeg (1, 1) double
    rateHz (1, 1) double
    durationsSec (1, 5) double
end

if any(~isfinite(initialJointDeg)) || ...
        ~isfinite(jointIndex) || jointIndex ~= fix(jointIndex) || ...
        jointIndex < 1 || jointIndex > 6 || ...
        ~isfinite(offsetDeg) || offsetDeg == 0 || ...
        ~isfinite(rateHz) || rateHz <= 0 || ...
        any(~isfinite(durationsSec)) || any(durationsSec <= 0)
    error('scopeguide:servoJMicro:InvalidTrajectoryInput', ...
        'Trajectory inputs must be finite and physically meaningful.');
end

periodSec = 1 / rateHz;
totalSec = sum(durationsSec);
timeSec = (0:periodSec:totalSec).';
if timeSec(end) < totalSec - 10 * eps(totalSec)
    timeSec(end + 1, 1) = totalSec;
else
    timeSec(end) = totalSec;
end

edges = [0, cumsum(durationsSec)];
amplitude = zeros(size(timeSec));
phase = strings(size(timeSec));
for index = 1:numel(timeSec)
    t = timeSec(index);
    if t < edges(2)
        phase(index) = "PRE_HOLD";
        amplitude(index) = 0;
    elseif t < edges(3)
        phase(index) = "OUTBOUND_RAMP";
        u = (t - edges(2)) / durationsSec(2);
        amplitude(index) = smoothstep(u);
    elseif t < edges(4)
        phase(index) = "PEAK_HOLD";
        amplitude(index) = 1;
    elseif t < edges(5)
        phase(index) = "RETURN_RAMP";
        u = (t - edges(4)) / durationsSec(4);
        amplitude(index) = 1 - smoothstep(u);
    else
        phase(index) = "FINAL_HOLD";
        amplitude(index) = 0;
    end
end

targetDeg = repmat(initialJointDeg, numel(timeSec), 1);
targetDeg(:, jointIndex) = targetDeg(:, jointIndex) + ...
    offsetDeg * amplitude;
trajectory = table(timeSec, phase, amplitude, targetDeg, ...
    'VariableNames', {'ScheduledTimeSec', 'Phase', ...
    'NormalizedAmplitude', 'TargetJointDeg'});
end

function value = smoothstep(value)
value = min(max(double(value), 0), 1);
value = value .* value .* (3 - 2 .* value);
end
