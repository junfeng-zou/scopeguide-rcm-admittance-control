function result = compensateHexHWrench( ...
        rawWrenchSensor, quaternionWxyz, calibration)
%COMPENSATEHEXHWRENCH Remove fitted bias and rigid tool gravity wrench.
%
% All sensor outputs are expressed at the HEX-H measurement origin.
% externalTool rotates the compensated wrench into the Nova5 tool axes but
% retains the HEX-H origin because the origin translation is not calibrated.

raw = double(rawWrenchSensor(:));
if numel(raw) ~= 6 || any(~isfinite(raw))
    error('stage2monitor:InvalidWrench', ...
        'rawWrenchSensor must contain six finite values.');
end
rotationBaseFromTool = quaternionWxyzToRotation(quaternionWxyz);
rotationToolFromBase = rotationBaseFromTool.';
rotationSensorFromTool = calibration.rotationToolFromSensor.';
rotationSensorFromBase = rotationSensorFromTool * rotationToolFromBase;
gravityForce = calibration.gravityForceSign * calibration.massKg * ...
    rotationSensorFromBase * calibration.gravityBaseMps2;
gravityTorque = cross(calibration.comSensorM, gravityForce);
gravitySensor = [gravityForce; gravityTorque];
unbiasedSensor = raw - calibration.biasSensor;
externalSensor = unbiasedSensor - gravitySensor;
rotation = calibration.rotationToolFromSensor;
externalTool = [rotation * externalSensor(1:3); ...
    rotation * externalSensor(4:6)];

result = struct();
result.rawSensor = raw;
result.unbiasedSensor = unbiasedSensor;
result.gravitySensor = gravitySensor;
result.externalSensor = externalSensor;
result.externalTool = externalTool;
result.rotationSensorFromBase = rotationSensorFromBase;
end
