function decision = controlDecision()
%CONTROLDECISION Create a stop-by-default controller decision.

decision = struct();
decision.AllowCommand = false;
decision.MotionAuthorized = false;
decision.QdotCommandRadSec = zeros(6, 1);
decision.QTargetRad = nan(6, 1);
decision.FsmState = "DISABLED";
decision.StopReason = "NOT_EVALUATED";
decision.FaultCode = "";
decision.IsValid = false;
end
