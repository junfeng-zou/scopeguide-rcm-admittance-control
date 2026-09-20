function sample = forceSample()
%FORCESAMPLE Create an invalid-by-default force sample in SI units.

sample = struct();
sample.RawWrenchSensor = nan(6, 1);
sample.Sequence = uint64(0);
sample.HostMonotonicSec = NaN;
sample.ReadDurationSec = NaN;
sample.SampleAgeSec = Inf;
sample.DeviceStatus = NaN;
sample.IsValid = false;
sample.StatusCode = "UNINITIALIZED";
end
