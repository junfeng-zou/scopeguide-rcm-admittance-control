function [calibration, provenance] = loadHexHGravityCalibration(jsonFile)
%LOADHEXHGRAVITYCALIBRATION Load and validate a stage-2 calibration JSON.

if ~(ischar(jsonFile) || isstring(jsonFile)) || ~isfile(jsonFile)
    error('stage2monitor:CalibrationFileMissing', ...
        'Calibration JSON does not exist: %s', char(jsonFile));
end
payload = jsondecode(fileread(jsonFile));
if ~isfield(payload, 'calibration') || ~isstruct(payload.calibration)
    error('stage2monitor:InvalidCalibration', ...
        'Calibration JSON must contain a calibration object.');
end
source = payload.calibration;
required = {'biasSensor','massKg','comSensorM','gravityBaseMps2', ...
    'gravityForceSign','rotationToolFromSensor','quaternionConvention','valid'};
for index = 1:numel(required)
    if ~isfield(source, required{index})
        error('stage2monitor:InvalidCalibration', ...
            'Calibration is missing field %s.', required{index});
    end
end
if ~source.valid
    error('stage2monitor:InvalidCalibration', ...
        'Calibration is marked invalid and cannot be used live.');
end
if ~strcmpi(char(source.quaternionConvention), 'base_from_tool')
    error('stage2monitor:QuaternionConvention', ...
        'Only Dobot base_from_tool ActualQuaternion is supported.');
end

calibration = struct();
calibration.biasSensor = finiteVector(source.biasSensor, 6, 'biasSensor');
calibration.massKg = double(source.massKg);
calibration.comSensorM = finiteVector(source.comSensorM, 3, 'comSensorM');
calibration.gravityBaseMps2 = finiteVector( ...
    source.gravityBaseMps2, 3, 'gravityBaseMps2');
calibration.gravityForceSign = double(source.gravityForceSign);
calibration.rotationToolFromSensor = double(source.rotationToolFromSensor);
calibration.quaternionConvention = char(source.quaternionConvention);
calibration.valid = logical(source.valid);
if ~isfinite(calibration.massKg) || calibration.massKg <= 0
    error('stage2monitor:InvalidCalibration', ...
        'massKg must be finite and positive.');
end
if ~ismember(calibration.gravityForceSign, [-1, 1])
    error('stage2monitor:InvalidCalibration', ...
        'gravityForceSign must be +1 or -1.');
end
rotation = calibration.rotationToolFromSensor;
if ~isequal(size(rotation), [3, 3]) || any(~isfinite(rotation), 'all') || ...
        norm(rotation * rotation.' - eye(3), 'fro') > 1e-7 || ...
        abs(det(rotation) - 1) > 1e-7
    error('stage2monitor:InvalidCalibration', ...
        'rotationToolFromSensor must be a right-handed rotation matrix.');
end

provenance = struct();
provenance.jsonFile = char(jsonFile);
provenance.retainedPoseIndices = fieldOrEmpty(payload, 'retainedPoseIndices');
provenance.excludedPoseIndices = fieldOrEmpty(payload, 'excludedPoseIndices');
end

function value = finiteVector(inputValue, count, name)
value = double(inputValue(:));
if numel(value) ~= count || any(~isfinite(value))
    error('stage2monitor:InvalidCalibration', ...
        '%s must contain %d finite values.', name, count);
end
end

function value = fieldOrEmpty(inputStruct, name)
if isfield(inputStruct, name)
    value = double(inputStruct.(name)(:).');
else
    value = [];
end
end
