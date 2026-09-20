function [cfg, record] = applyManualRcmPoint(cfg, pointBase, options)
%APPLYMANUALRCMPOINT Inject a user-supplied RCM point in the robot base.
% This interface only records and converts the coordinate.  It deliberately
% leaves CalibrationValid and BoundsValidated false, so providing a point
% cannot by itself authorize physical robot motion.

arguments
    cfg (1, 1) struct
    pointBase
    options.InputUnit (1, 1) string = "m"
    options.SourceDescription (1, 1) string = ""
    options.FixtureIdentifier (1, 1) string = ""
end

validateRcmAdmittanceConfig(cfg);
if ~isnumeric(pointBase) || ~isreal(pointBase) || ...
        numel(pointBase) ~= 3 || ~all(isfinite(pointBase(:)))
    error('scopeguide:config:InvalidManualRcmPoint', ...
        'pointBase must contain three finite real coordinates.');
end

inputUnit = lower(strtrim(options.InputUnit));
switch inputUnit
    case "m"
        scaleToM = 1;
    case "mm"
        scaleToM = 1e-3;
    otherwise
        error('scopeguide:config:InvalidManualRcmUnit', ...
            'InputUnit must be "m" or "mm".');
end

pointBaseM = double(pointBase(:)) * scaleToM;
if norm(pointBaseM) > 5
    error('scopeguide:config:ManualRcmPointUnitSuspect', ...
        ['RCM point norm exceeds 5 m. Check whether the supplied ' ...
        'coordinate unit is correct.']);
end

recordedAt = string(datetime('now', 'TimeZone', 'Asia/Shanghai', ...
    'Format', 'yyyy-MM-dd HH:mm:ss Z'));
cfg.rcm.PointBaseM = pointBaseM;
cfg.rcm.PointFrame = "robot_base";
cfg.rcm.PointSource = "manual_base_coordinate";
cfg.rcm.PointSourceDescription = options.SourceDescription;
cfg.rcm.FixtureIdentifier = options.FixtureIdentifier;
cfg.rcm.PointRecordedAtLocal = recordedAt;
cfg.rcm.CalibrationValid = false;
cfg.rcm.BoundsValidated = false;
validateRcmAdmittanceConfig(cfg);

record = struct();
record.PointBaseM = pointBaseM;
record.PointBaseMm = 1e3 * pointBaseM;
record.Frame = "robot_base";
record.InputUnit = inputUnit;
record.Source = "manual_base_coordinate";
record.SourceDescription = options.SourceDescription;
record.FixtureIdentifier = options.FixtureIdentifier;
record.RecordedAtLocal = recordedAt;
record.CalibrationValid = false;
record.BoundsValidated = false;
record.PhysicalMotionAuthorizedByThisOperation = false;
end
