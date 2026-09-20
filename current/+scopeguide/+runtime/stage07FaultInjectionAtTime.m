function injection = stage07FaultInjectionAtTime(timeSec, profile)
%STAGE07FAULTINJECTIONATTIME Build the configured deterministic injection.

arguments
    timeSec (1, 1) double {mustBeFinite, mustBeNonnegative}
    profile (1, 1) string {mustBeMember(profile, ...
        ["none", "acceptance"])} = "none"
end

injection = scopeguide.types.stage07FaultInjection();
if profile ~= "acceptance"
    return;
end

schedule = scopeguide.runtime.stage07FaultSchedule();
active = find(timeSec >= schedule.TimeSec & ...
    timeSec < schedule.TimeSec + schedule.DurationSec, 1, 'first');
if isempty(active)
    return;
end

fieldName = char(schedule.FieldName(active));
injection.(fieldName) = true;
injection.StatusCode = schedule.StatusCode(active);
end
