function cfg = defaultHexHToolCalibrationConfig()
%DEFAULTHEXHTOOLCALIBRATIONCONFIG Conservative calibration defaults.

cfg.RobotIPAddress = '192.168.50.105';
cfg.MotionMode = 'operator_guided'; % operator_guided | automatic_joint

% Automatic motion is rejected unless collision-free joint targets are
% explicitly supplied. Never copy targets from another robot/workcell.
cfg.AutomaticJointTargetsDeg = zeros(0, 6);
cfg.OperatorGuidedPoseCount = 10;
cfg.RequireTypedConfirmation = true;
cfg.RobotSpeedRatioPercent = 5;
cfg.MaximumJointStepDeg = 35;
cfg.JointTargetToleranceDeg = 0.5;
cfg.MoveTimeoutSec = 60;
cfg.StableHoldSec = 1.0;
cfg.MaximumStationaryJointSpeedDegSec = 0.10;
cfg.MaximumStationaryTcpTranslationSpeed = 0.20;
cfg.MaximumStationaryTcpRotationSpeed = 0.20;
% Reject sustained motion while tolerating one isolated 30004 velocity spike.
cfg.MinimumConsecutiveMovingFeedbackFrames = 2;

cfg.SampleRateHz = 200;
cfg.SamplesPerPoseSec = 4;
cfg.MaximumMovingSampleRatio = 0.01;
cfg.MaximumFeedbackStaleSec = 0.10;

% User-confirmed physical gravity direction at J=[0,0,0,0,0,0] deg.
cfg.RequireZeroJointGravityReference = true;
cfg.ZeroJointReferenceDeg = zeros(1, 6);
cfg.ZeroJointToleranceDeg = 0.5;
cfg.ExpectedGravityDirectionTool = [0, 1, 0];
cfg.ExpectedGravityDirectionSensor = [0, -1, 0];
cfg.MaximumGravityDirectionErrorDeg = 10;

cfg.RToolFromSensor = diag([-1, -1, 1]);
cfg.SensorOriginInToolM = [NaN, NaN, NaN];
cfg.GravityBaseMps2 = [0, 0, -9.80665];
cfg.GravityForceSign = 'auto';
cfg.QuaternionConvention = 'base_from_tool';
cfg.MaximumConditionNumber = 1e6;

cfg.OutputDirectory = fullfile(fileparts(mfilename('fullpath')), 'data');
cfg.SaveFigure = true;
end
