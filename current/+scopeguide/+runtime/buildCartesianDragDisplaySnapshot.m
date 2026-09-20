function snapshot = buildCartesianDragDisplaySnapshot( ...
        timeSec, processed, decision)
%BUILDCARTESIANDRAGDISPLAYSNAPSHOT Value-only no-RCM GUI telemetry.

arguments
    timeSec (1, 1) double {mustBeFinite, mustBeNonnegative}
    processed (1, 1) struct
    decision (1, 1) struct
end
if ~isfield(processed, 'BaselineDiagnostics') || ...
        ~isfield(decision, 'Admittance') || ...
        ~isfield(decision, 'Qp') || ...
        ~isfield(decision, 'EnableStatus')
    error('scopeguide:cartesianDrag:IncompleteDisplayData', ...
        'Processed data or Cartesian drag decision is incomplete.');
end
wrench = zeros(6, 1);
if isfield(decision.Admittance, 'Mapping') && ...
        ~isempty(decision.Admittance.Mapping) && ...
        isfield(decision.Admittance.Mapping, 'WrenchDragAtDragPoint')
    wrench = double( ...
        decision.Admittance.Mapping.WrenchDragAtDragPoint(:));
elseif isfield(processed, 'ControlWrenchToolAtSensorOrigin')
    wrench = double(processed.ControlWrenchToolAtSensorOrigin(:));
end
desired = double(decision.Admittance.DesiredTwistBase(:));
achieved = double(decision.Qp.AchievedTwistBase(:));
relative = double(decision.RelativeCoordinate(:));
predicted = double(decision.Qp.PredictedRelativeCoordinate(:));
vectors = {wrench, desired, achieved, relative, predicted};
if any(cellfun(@(value) numel(value) ~= 6 || ...
        any(~isfinite(value)), vectors))
    error('scopeguide:cartesianDrag:InvalidDisplayVector', ...
        'Cartesian display vectors must be finite 6-by-1 values.');
end
baselineReady = false;
if isstruct(processed.BaselineDiagnostics) && ...
        isfield(processed.BaselineDiagnostics, 'Ready')
    baselineReady = logical(processed.BaselineDiagnostics.Ready);
end
snapshot = struct();
snapshot.Type = "telemetry";
snapshot.TimeSec = timeSec;
snapshot.BaselineReady = baselineReady;
snapshot.ControlWrenchDrag = wrench;
snapshot.DesiredTwistBase = desired;
snapshot.AchievedTwistBase = achieved;
snapshot.RelativeCoordinate = relative;
snapshot.PredictedRelativeCoordinate = predicted;
snapshot.EnableState = string(decision.EnableStatus.State);
snapshot.CommandScale = double(decision.EnableStatus.CommandScale);
snapshot.QpStatusCode = string(decision.Qp.StatusCode);
snapshot.FaultCode = string(decision.EnableStatus.FaultCode);
snapshot.PredictionValid = logical(decision.PredictionValid);
snapshot.HardwareCommandAuthorized = ...
    logical(decision.HardwareCommandAuthorized);
snapshot.MotionCommandSent = logical(decision.MotionCommandSent);
snapshot.JointVelocityCommandRadSec = ...
    double(decision.QdotCommandRadSec(:));
snapshot.JointVelocityQpCandidateRadSec = ...
    double(decision.QdotQpCandidateRadSec(:));
snapshot.ForceStatusCode = string(decision.Safety.ForceStatusCode);
snapshot.WrenchSafetyStopCode = ...
    string(decision.Safety.WrenchSafetyStopCode);
snapshot.NoRcmConstraint = true;
end
