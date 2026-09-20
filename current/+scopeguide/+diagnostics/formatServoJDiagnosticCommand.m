function command = formatServoJDiagnosticCommand( ...
        jointsDeg, profile, legacyT, legacyLookaheadTime, legacyGain)
%FORMATSERVOJDIAGNOSTICCOMMAND Reproduce current and legacy ServoJ syntax.

arguments
    jointsDeg
    profile (1, 1) string
    legacyT (1, 1) double = 0.04
    legacyLookaheadTime (1, 1) double = 60
    legacyGain (1, 1) double = 400
end

joints = double(jointsDeg(:).');
if numel(joints) ~= 6 || any(~isfinite(joints))
    error('scopeguide:servoJDiagnostic:InvalidJointTarget', ...
        'jointsDeg must contain six finite values.');
end
if ~isfinite(legacyT) || legacyT <= 0 || ...
        ~isfinite(legacyLookaheadTime) || legacyLookaheadTime <= 0 || ...
        ~isfinite(legacyGain) || legacyGain <= 0
    error('scopeguide:servoJDiagnostic:InvalidLegacyParameters', ...
        'Legacy ServoJ parameters must be finite and positive.');
end

switch profile
    case "scopeguide_minimal"
        command = string(sprintf( ...
            'ServoJ(%.6f,%.6f,%.6f,%.6f,%.6f,%.6f)', joints));
    case "legacy_parameterized"
        command = string(sprintf([ ...
            'ServoJ(%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,' ...
            't=%.6f,lookahead_time=%.6f,gain=%.6f)'], ...
            joints, legacyT, legacyLookaheadTime, legacyGain));
    otherwise
        error('scopeguide:servoJDiagnostic:UnknownCommandProfile', ...
            'Unknown ServoJ command profile: %s.', profile);
end
end
