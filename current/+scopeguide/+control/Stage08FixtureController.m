classdef Stage08FixtureController < handle
    %STAGE08FIXTURECONTROLLER Stage 4-6 chain with Stage 8 motion gates.
    % This class computes targets only.  The RobotAdapter remains the sole
    % ServoJ call site and re-checks authorization immediately before send.

    properties (SetAccess = private)
        Config
        Authorization
        DofMask
        EnableStateMachine
        Admittance
        QpController
        TargetIntegrator
        LastStepTimeSec (1, 1) double = NaN
        QpFaultPending (1, 1) logical = false
        StepCount (1, 1) uint64 = uint64(0)
    end

    methods
        function obj = Stage08FixtureController(cfg, authorization)
            arguments
                cfg (1, 1) struct
                authorization (1, 1) struct
            end
            validateRcmAdmittanceConfig(cfg);
            if string(cfg.runtime.Mode) ~= "fixture_motion" || ...
                    ~cfg.robot.EnableMotion || cfg.robot.DryRun
                error('scopeguide:stage08:FixtureMotionConfigRequired', ...
                    'Stage 8 requires enabled, non-dry-run fixture_motion.');
            end
            if ~isfield(authorization, 'CommissioningAllowed') || ...
                    ~logical(authorization.CommissioningAllowed)
                error('scopeguide:stage08:AuthorizationRequired', ...
                    'Stage 8 commissioning authorization is not allowed.');
            end
            obj.Config = cfg;
            obj.Authorization = authorization;
            obj.DofMask = scopeguide.control.stage08DofMask( ...
                cfg.stage08.DofMode);
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

        function output = step(obj, processed, robotState, handleInput, ...
                nowSec, externalHealth)
            arguments
                obj
                processed (1, 1) struct
                robotState (1, 1) struct
                handleInput (1, 1) struct
                nowSec (1, 1) double
                externalHealth (1, 1) struct = defaultExternalHealth()
            end
            validateExternalHealth(externalHealth);
            obj.StepCount = obj.StepCount + uint64(1);
            if isfinite(obj.LastStepTimeSec)
                dtSec = nowSec - obj.LastStepTimeSec;
            else
                dtSec = obj.Config.runtime.NominalDtSec;
            end
            obj.LastStepTimeSec = nowSec;

            safety = buildSafetyStatus(obj, processed, robotState, ...
                dtSec, externalHealth);
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

            % Disabled generalized axes are exactly zero at the QP input.
            % Every Stage 8 run creates a fresh controller, so hidden state
            % from a previous phase cannot be released in a later phase.
            if ~isempty(admittanceOutput.Mapping)
                maskedVelocity = admittanceOutput.GeneralizedVelocity .* ...
                    double(obj.DofMask);
                admittanceOutput.GeneralizedVelocity = maskedVelocity;
                admittanceOutput.GeneralizedVelocityWithRoll = ...
                    [maskedVelocity; 0];
                admittanceOutput.DesiredTipTwistBase = ...
                    admittanceOutput.Mapping.Basis.TwistBasisBase * ...
                    maskedVelocity;
                admittanceOutput.DofMask = obj.DofMask;
            else
                admittanceOutput.DofMask = obj.DofMask;
            end

            if validRobotJointState(robotState)
                qState = double(robotState.JointPositionRad(:));
            else
                qState = zeros(6, 1);
            end
            qpOutput = obj.QpController.step(qState, ...
                admittanceOutput.DesiredTipTwistBase, ...
                obj.Config.rcm.PointBaseM, max(dtSec, eps), enableStatus);
            qdotQpCandidate = qpOutput.QdotCommandRadSec;
            obj.QpFaultPending = qpOutput.RequiresFault;

            commandStatus = enableStatus;
            commandStatus.CommandScale = 1;
            if ~qpOutput.MotionCommandValid || ...
                    ~externalHealth.CommandChannelHealthy || ...
                    ~externalHealth.TrackingHealthy
                commandStatus.MotionPermitted = false;
                commandStatus.ResetDynamicState = true;
                obj.Admittance.reset();
            end
            if string(obj.Config.stage08.DofMode) == "hold"
                commandStatus.MotionPermitted = false;
                commandStatus.ResetDynamicState = true;
                qpOutput.QdotCommandRadSec = zeros(6, 1);
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
            output.DofMode = string(obj.Config.stage08.DofMode);
            output.DofMask = obj.DofMask;
            output.QdotCommandRadSec = qpOutput.QdotCommandRadSec;
            output.QdotQpCandidateRadSec = qdotQpCandidate;
            output.QTargetCommandRad = integrated.QTargetRad;
            output.CommandEligible = enableStatus.MotionPermitted && ...
                qpOutput.MotionCommandValid && integrated.IsValid && ...
                obj.Authorization.CommissioningAllowed && ...
                externalHealth.CommandChannelHealthy && ...
                externalHealth.TrackingHealthy && ...
                string(obj.Config.stage08.DofMode) ~= "hold";
            output.PredictionValid = output.CommandEligible;
            output.FaultInjection = scopeguide.types.stage07FaultInjection();
            output.HardwareCommandAuthorized = ...
                obj.Authorization.CommissioningAllowed;
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
        dtSec, externalHealth)
safety = scopeguide.types.enableSafetyStatus();
[safety.ForceFresh, safety.ForceStatusCode] = ...
    forceFreshStatus(processed);
safety.ForceControlReady = baselineReady(processed);
[safety.WrenchSafetyHealthy, safety.WrenchSafetyStopCode] = ...
    wrenchSafetyStatus(processed);
safety.RobotFresh = validRobotJointState(robotState) && ...
    logical(robotState.IsValid);
safety.NeutralWrench = isfield(processed, 'NeutralCheckPassed') && ...
    logical(processed.NeutralCheckPassed);
safety.QpHealthy = ~obj.QpFaultPending;
safety.ControlPeriodHealthy = isfinite(dtSec) && dtSec > 0 && ...
    dtSec <= obj.Config.runtime.MaximumDtSec;
safety.CommandChannelHealthy = externalHealth.CommandChannelHealthy;
safety.TrackingHealthy = externalHealth.TrackingHealthy;
safety.MotionGatesSatisfied = obj.Authorization.CommissioningAllowed;
safety.StatusCode = string(externalHealth.StatusCode);
end

function [fresh, code] = forceFreshStatus(processed)
fresh = false;
code = "FORCE_QUALITY_UNAVAILABLE";
if ~isfield(processed, 'Quality') || ~isstruct(processed.Quality) || ...
        ~isfield(processed.Quality, 'Valid')
    return;
end
fresh = logical(processed.Quality.Valid);
if fresh
    code = "OK";
elseif isfield(processed.Quality, 'StatusCode')
    qualityCode = string(processed.Quality.StatusCode);
    if strlength(qualityCode) > 0
        code = "FORCE_" + qualityCode;
    end
end
end

function ready = baselineReady(processed)
ready = isfield(processed, 'BaselineDiagnostics') && ...
    isstruct(processed.BaselineDiagnostics) && ...
    isfield(processed.BaselineDiagnostics, 'Ready') && ...
    logical(processed.BaselineDiagnostics.Ready);
end

function [healthy, code] = wrenchSafetyStatus(processed)
healthy = false;
code = "WRENCH_SAFETY_UNAVAILABLE";
if ~isfield(processed, 'Safety') || ~isstruct(processed.Safety) || ...
        ~isfield(processed.Safety, 'StopRequested')
    return;
end
healthy = ~logical(processed.Safety.StopRequested);
if healthy
    code = "";
    return;
end
if isfield(processed.Safety, 'StopReasons')
    reasons = string(processed.Safety.StopReasons(:));
    reasons = reasons(~ismissing(reasons) & strlength(reasons) > 0);
    if ~isempty(reasons)
        code = strjoin(reasons, "+");
        return;
    end
end
code = "WRENCH_SAFETY_STOP";
end

function health = defaultExternalHealth()
health = struct('CommandChannelHealthy', true, ...
    'TrackingHealthy', true, 'StatusCode', "OK");
end

function validateExternalHealth(health)
required = {'CommandChannelHealthy', 'TrackingHealthy', 'StatusCode'};
for index = 1:numel(required)
    if ~isfield(health, required{index})
        error('scopeguide:stage08:InvalidExternalHealth', ...
            'External health is missing %s.', required{index});
    end
end
if ~islogical(health.CommandChannelHealthy) || ...
        ~isscalar(health.CommandChannelHealthy) || ...
        ~islogical(health.TrackingHealthy) || ...
        ~isscalar(health.TrackingHealthy)
    error('scopeguide:stage08:InvalidExternalHealth', ...
        'External health flags must be scalar logicals.');
end
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
    isfield(robotState, 'IsValid') && islogical(robotState.IsValid) && ...
    isscalar(robotState.IsValid);
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
