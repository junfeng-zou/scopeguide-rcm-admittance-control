function state = robotState()
%ROBOTSTATE Create an invalid-by-default robot state in SI units.

state = struct();
state.JointPositionRad = zeros(6, 1);
state.JointVelocityRadSec = zeros(6, 1);
state.TBaseFlange = nan(4, 4);
state.QuaternionBaseFlangeWxyz = nan(4, 1);
state.ControllerCartesianPoseRaw = nan(6, 1);
state.ControllerPosePositionM = nan(3, 1);
state.ControllerPoseRpyRad = nan(3, 1);
state.TBaseControllerPose = nan(4, 4);
state.QuaternionBaseControllerWxyz = nan(4, 1);
state.ControllerRpyQuaternionMismatchRad = NaN;
state.ActualTcpTwistBase = nan(6, 1);
state.FeedbackSequence = uint64(0);
state.HostMonotonicSec = NaN;
state.SampleAgeSec = Inf;
state.InvalidFeedbackByteCount = uint64(0);
state.RobotMode = "UNKNOWN";
state.ControllerPoseReference = "unknown";
state.ControllerPoseUnitsVerified = false;
state.IsValid = false;
state.StatusCode = "UNINITIALIZED";
end
