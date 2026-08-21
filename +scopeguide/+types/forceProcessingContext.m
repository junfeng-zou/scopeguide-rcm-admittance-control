function context = forceProcessingContext()
%FORCEPROCESSINGCONTEXT Fail-safe baseline-update eligibility context.
% Baseline learning is allowed only when every field is explicitly made
% true by later state-estimation and deadman logic.

context = struct();
context.HandleEnabled = true;
context.RobotStationary = false;
context.NoContactConfirmed = false;
context.AllowBaselineUpdate = false;
end
