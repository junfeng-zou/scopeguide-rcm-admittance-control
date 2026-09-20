function snapshot = buildStage07DisplaySnapshot(timeSec, processed, decision)
%BUILDSTAGE07DISPLAYSNAPSHOT Compact value-only Stage 7 telemetry.
% The struct deliberately contains no graphics, network or handle objects,
% so it can cross a PollableDataQueue from a control worker to the client.

arguments
    timeSec (1, 1) double {mustBeFinite, mustBeNonnegative}
    processed (1, 1) struct
    decision (1, 1) struct
end

requiredProcessed = {'ControlForceTool', 'BaselineDiagnostics'};
for index = 1:numel(requiredProcessed)
    if ~isfield(processed, requiredProcessed{index})
        error('scopeguide:stage07:IncompleteDisplayProcessedData', ...
            'Processed data is missing %s.', requiredProcessed{index});
    end
end
requiredDecision = {'Admittance', 'Qp', 'Geometry', ...
    'GeometryValid', 'EnableStatus', 'PredictionValid', ...
    'HardwareCommandAuthorized', 'MotionCommandSent', 'FaultInjection'};
for index = 1:numel(requiredDecision)
    if ~isfield(decision, requiredDecision{index})
        error('scopeguide:stage07:IncompleteDisplayDecision', ...
            'Decision data is missing %s.', requiredDecision{index});
    end
end

force = double(processed.ControlForceTool(:));
desired = double(decision.Admittance.GeneralizedVelocity(:));
achieved = double(decision.Qp.AchievedGeneralizedVelocity(:));
if numel(force) ~= 3 || numel(desired) ~= 3 || ...
        numel(achieved) ~= 3 || any(~isfinite(force)) || ...
        any(~isfinite(desired)) || any(~isfinite(achieved))
    error('scopeguide:stage07:InvalidDisplayVector', ...
        'Force, desired and achieved display vectors must be finite 3x1.');
end

baselineReady = false;
if isstruct(processed.BaselineDiagnostics) && ...
        isfield(processed.BaselineDiagnostics, 'Ready')
    baselineReady = logical(processed.BaselineDiagnostics.Ready);
end
currentRcmErrorM = NaN;
if decision.GeometryValid && isstruct(decision.Geometry) && ...
        isfield(decision.Geometry, 'RcmErrorNormM')
    currentRcmErrorM = double(decision.Geometry.RcmErrorNormM);
end

snapshot = struct();
snapshot.Type = "telemetry";
snapshot.TimeSec = timeSec;
snapshot.BaselineReady = baselineReady;
snapshot.ControlForceTool = force;
snapshot.DesiredGeneralizedVelocity = desired;
snapshot.AchievedGeneralizedVelocity = achieved;
snapshot.CurrentRcmErrorM = currentRcmErrorM;
snapshot.PredictedRcmErrorM = ...
    double(decision.Qp.PredictedRcmErrorNormM);
snapshot.EnableState = string(decision.EnableStatus.State);
snapshot.CommandScale = double(decision.EnableStatus.CommandScale);
snapshot.QpStatusCode = string(decision.Qp.StatusCode);
snapshot.FaultCode = string(decision.EnableStatus.FaultCode);
snapshot.InjectionStatusCode = ...
    string(decision.FaultInjection.StatusCode);
snapshot.PredictionValid = logical(decision.PredictionValid);
snapshot.HardwareCommandAuthorized = ...
    logical(decision.HardwareCommandAuthorized);
snapshot.MotionCommandSent = logical(decision.MotionCommandSent);
if isfield(decision, 'QdotCommandRadSec')
    snapshot.JointVelocityCommandRadSec = ...
        double(decision.QdotCommandRadSec(:));
else
    snapshot.JointVelocityCommandRadSec = zeros(6, 1);
end
if isfield(decision, 'QdotQpCandidateRadSec')
    snapshot.JointVelocityQpCandidateRadSec = ...
        double(decision.QdotQpCandidateRadSec(:));
else
    snapshot.JointVelocityQpCandidateRadSec = ...
        snapshot.JointVelocityCommandRadSec;
end
snapshot.ForceStatusCode = "";
snapshot.WrenchSafetyStopCode = "";
if isfield(decision, 'Safety') && isstruct(decision.Safety)
    if isfield(decision.Safety, 'ForceStatusCode')
        snapshot.ForceStatusCode = ...
            string(decision.Safety.ForceStatusCode);
    end
    if isfield(decision.Safety, 'WrenchSafetyStopCode')
        snapshot.WrenchSafetyStopCode = ...
            string(decision.Safety.WrenchSafetyStopCode);
    end
end
end
