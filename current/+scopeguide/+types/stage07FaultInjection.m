function injection = stage07FaultInjection()
%STAGE07FAULTINJECTION Disabled-by-default dry-run fault overrides.

injection = struct();
injection.ForceStale = false;
injection.RobotStale = false;
injection.HandleStale = false;
injection.QpFailure = false;
injection.ControlOverrun = false;
injection.StatusCode = "NONE";
end
