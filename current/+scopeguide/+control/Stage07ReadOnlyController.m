classdef Stage07ReadOnlyController < handle
    %STAGE07READONLYCONTROLLER Compose Stages 4-6 without robot commands.

    properties (SetAccess = private)
        Config
        EnableStateMachine
        Admittance
        QpController
        TargetIntegrator
        LastStepTimeSec (1, 1) double = NaN
        QpFaultPending (1, 1) logical = false
        StepCount (1, 1) uint64 = uint64(0)
    end

    methods
        function obj = Stage07ReadOnlyController(cfg)
            arguments
                cfg (1, 1) struct
            end
            validateRcmAdmittanceConfig(cfg);
            if string(cfg.runtime.Mode) ~= "live_dry_run" || ...
                    cfg.robot.EnableMotion || ~cfg.robot.DryRun
                error('scopeguide:stage07:ReadOnlyConfigurationRequired', ...
                    ['Stage07ReadOnlyController requires live_dry_run, ' ...
                    'EnableMotion=false and DryRun=true.']);
            end
            if ~all(isfinite(cfg.rcm.PointBaseM)) || ...
                    string(cfg.rcm.PointFrame) ~= "robot_base"
                error('scopeguide:stage07:RcmPointRequired', ...
                    'A finite robot-base pRcmBase is required.');
            end
            obj.Config = cfg;
            obj.EnableStateMachine = ...
                scopeguide.control.HandleEnableStateMachine(cfg);
            obj.Admittance = ...
                scopeguide.control.ForceOnlyRcmAdmittance(cfg);
            obj.QpController = ...
                scopeguide.control.RcmConstrainedQpController(cfg);
            obj.TargetIntegrator = ...
                scopeguide.control.JointCommandIntegrator(cfg);
        end

        function requestReset(obj)
            obj.EnableStateMachine.requestReset();
        end

        function output = step(obj, processed, robotState, ...
                handleInput, nowSec, injection)
            arguments
                obj
                processed (1, 1) struct
                robotState (1, 1) struct
                handleInput (1, 1) struct
                nowSec (1, 1) double
                injection (1, 1) struct = ...
                    scopeguide.types.stage07FaultInjection()
            end
            obj.StepCount = obj.StepCount + 1;
            validateInjection(injection);
            if isfinite(obj.LastStepTimeSec)
                dtSec = nowSec - obj.LastStepTimeSec;
            else
                dtSec = obj.Config.runtime.NominalDtSec;
            end
            obj.LastStepTimeSec = nowSec;

            if injection.HandleStale
                handleInput.Enabled = false;
                handleInput.IsValid = false;
                handleInput.SampleAgeSec = Inf;
                handleInput.StatusCode = "STALE_INPUT";
            end
            safety = buildSafetyStatus(obj, processed, robotState, ...
                dtSec, injection);
            enableStatus = obj.EnableStateMachine.step( ...
                handleInput, safety, nowSec);
            if validControlWrench(processed) && safety.ForceFresh
                controlWrench = double( ...
                    processed.ControlWrenchToolAtSensorOrigin(:));
            else
                controlWrench = zeros(6, 1);
            end
            admittanceOutput = obj.Admittance.step(controlWrench, ...
                robotEndoscopeTransform(robotState, obj.Config), ...
                obj.Config.rcm.PointBaseM, max(dtSec, eps), enableStatus);

            if validRobotJointState(robotState)
                qState = double(robotState.JointPositionRad(:));
            else
                qState = zeros(6, 1);
            end
            qpOutput = obj.QpController.step(qState, ...
                admittanceOutput.DesiredTipTwistBase, ...
                obj.Config.rcm.PointBaseM, max(dtSec, eps), enableStatus);
            obj.QpFaultPending = qpOutput.RequiresFault;

            commandStatus = enableStatus;
            % Soft-start is already applied to the Stage 5 force input.
            commandStatus.CommandScale = 1;
            if ~qpOutput.MotionCommandValid
                commandStatus.MotionPermitted = false;
                commandStatus.ResetDynamicState = true;
                obj.Admittance.reset();
            end
            integrated = obj.TargetIntegrator.step(qState, ...
                qpOutput.QdotCommandRadSec, commandStatus, nowSec);

            [geometry, geometryValid, geometryCode] = ...
                currentGeometry(qState, robotState.IsValid, obj.Config);
            output = struct();
            output.TimestampSec = nowSec;
            output.DtSec = dtSec;
            output.Safety = safety;
            output.EnableStatus = enableStatus;
            output.Admittance = admittanceOutput;
            output.Qp = qpOutput;
            output.Integrator = integrated;
            output.Geometry = geometry;
            output.GeometryValid = geometryValid;
            output.GeometryStatusCode = geometryCode;
            output.QdotPredictedRadSec = ...
                qpOutput.QdotCommandRadSec;
            output.QTargetPredictedRad = integrated.QTargetRad;
            output.PredictionValid = enableStatus.MotionPermitted && ...
                qpOutput.MotionCommandValid && integrated.IsValid;
            output.FaultInjection = injection;
            output.HardwareCommandAuthorized = false;
            output.MotionCommandSent = false;
            output.DoesNotSendHardwareCommands = true;
        end

        function reset(obj)
            obj.Admittance.reset();
            obj.QpController.reset();
            obj.TargetIntegrator.reset();
            obj.QpFaultPending = false;
            obj.LastStepTimeSec = NaN;
        end
    end
end

function safety = buildSafetyStatus(obj, processed, robotState, ...
        dtSec, injection)
safety = scopeguide.types.enableSafetyStatus();
forceValid = isfield(processed, 'Quality') && ...
    isstruct(processed.Quality) && ...
    isfield(processed.Quality, 'Valid') && ...
    logical(processed.Quality.Valid) && ...
    isfield(processed, 'MotionInputValid') && ...
    logical(processed.MotionInputValid) && ...
    isfield(processed, 'Safety') && isstruct(processed.Safety) && ...
    isfield(processed.Safety, 'StopRequested') && ...
    ~logical(processed.Safety.StopRequested);
safety.ForceFresh = forceValid && ~injection.ForceStale;
safety.RobotFresh = validRobotJointState(robotState) && ...
    logical(robotState.IsValid) && ~injection.RobotStale;
safety.NeutralWrench = isfield(processed, 'NeutralCheckPassed') && ...
    logical(processed.NeutralCheckPassed);
safety.QpHealthy = ~obj.QpFaultPending && ~injection.QpFailure;
safety.ControlPeriodHealthy = isfinite(dtSec) && dtSec > 0 && ...
    dtSec <= obj.Config.runtime.MaximumDtSec && ...
    ~injection.ControlOverrun;
safety.MotionGatesSatisfied = ...
    string(obj.Config.runtime.Mode) == "live_dry_run" && ...
    ~obj.Config.robot.EnableMotion && obj.Config.robot.DryRun && ...
    all(isfinite(obj.Config.rcm.PointBaseM));
safety.StatusCode = string(injection.StatusCode);
end

function valid = validControlWrench(processed)
valid = isfield(processed, 'ControlWrenchToolAtSensorOrigin') && ...
    isnumeric(processed.ControlWrenchToolAtSensorOrigin) && ...
    numel(processed.ControlWrenchToolAtSensorOrigin) == 6 && ...
    all(isfinite(processed.ControlWrenchToolAtSensorOrigin(:)));
end

function valid = validRobotJointState(robotState)
valid = isstruct(robotState) && ...
    isfield(robotState, 'JointPositionRad') && ...
    isnumeric(robotState.JointPositionRad) && ...
    numel(robotState.JointPositionRad) == 6 && ...
    all(isfinite(robotState.JointPositionRad(:))) && ...
    isfield(robotState, 'IsValid') && ...
    islogical(robotState.IsValid) && isscalar(robotState.IsValid);
end

function transform = robotEndoscopeTransform(robotState, cfg)
if validRobotJointState(robotState)
    kinematics = scopeguide.geometry.cr5ForwardKinematics( ...
        robotState.JointPositionRad, cfg);
    transform = kinematics.TBaseEndoscope;
else
    transform = nan(4, 4);
end
end

function [geometry, valid, code] = currentGeometry(qState, robotValid, cfg)
if ~robotValid
    geometry = struct([]);
    valid = false;
    code = "ROBOT_STATE_INVALID";
    return;
end
try
    geometry = scopeguide.geometry.computeRcmConstraintKinematics( ...
        qState, cfg.rcm.PointBaseM, cfg);
    valid = true;
    code = "OK";
catch exception
    geometry = struct([]);
    valid = false;
    code = string(exception.identifier);
end
end

function validateInjection(injection)
names = {'ForceStale', 'RobotStale', 'HandleStale', ...
    'QpFailure', 'ControlOverrun'};
for index = 1:numel(names)
    if ~isfield(injection, names{index}) || ...
            ~islogical(injection.(names{index})) || ...
            ~isscalar(injection.(names{index}))
        error('scopeguide:stage07:InvalidFaultInjection', ...
            'Fault injection field %s must be scalar logical.', ...
            names{index});
    end
end
end
