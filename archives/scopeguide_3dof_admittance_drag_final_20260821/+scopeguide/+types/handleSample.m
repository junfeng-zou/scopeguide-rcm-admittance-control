function sample = handleSample()
%HANDLESAMPLE Create a fail-safe external deadman input sample.

sample = struct();
sample.Enabled = false;
sample.Sequence = uint64(0);
sample.HostMonotonicSec = NaN;
sample.SampleAgeSec = Inf;
sample.Source = "uninitialized";
sample.SupportsPhysicalMotion = false;
sample.IsValid = false;
sample.StatusCode = "UNINITIALIZED";
end
