function status = enableSafetyStatus()
%ENABLESAFETYSTATUS Fail-safe prerequisites for the deadman FSM.
% NeutralWrench is a PREARM-only condition.  The remaining fields are
% runtime health gates and must stay true while ENABLED.

status = struct();
status.NeutralWrench = false;
status.ForceFresh = false;
status.ForceStatusCode = "FORCE_STALE";
status.ForceControlReady = true;
status.WrenchSafetyHealthy = true;
status.WrenchSafetyStopCode = "";
status.RobotFresh = false;
status.QpHealthy = false;
status.ControlPeriodHealthy = false;
status.MotionGatesSatisfied = false;
status.CommandChannelHealthy = true;
status.TrackingHealthy = true;
status.StatusCode = "UNINITIALIZED";
end
