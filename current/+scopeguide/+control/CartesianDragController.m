classdef CartesianDragController < handle
    %CARTESIANDRAGCONTROLLER Complete target generator without RCM terms.
    % The class computes ServoJ targets but never sends hardware commands.

    properties (SetAccess = private)
        Config
        Authorization
        EnableStateMachine
        Admittance
        QpController
        TargetIntegrator
        AnchorTransformBase (4, 4) double = nan(4, 4)
        LastStepTimeSec (1, 1) double = NaN
        QpFaultPending (1, 1) logical = false
        StepCount (1, 1) uint64 = uint64(0)
    end

    methods
        function obj = CartesianDragController(cfg, authorization)
            arguments
                cfg (1, 1) struct
                authorization (1, 1) struct
            end
            validateCartesianAdmittanceDragConfig(cfg);
            if string(cfg.runtime.Mode) ~= "fixture_motion" || ...
                    ~cfg.robot.EnableMotion || cfg.robot.DryRun
                error('scopeguide:cartesianDrag:MotionConfigRequired', ...
                    ['Cartesian physical drag requires enabled, ' ...
                     'non-dry-run fixture_motion.']);
            end
            if ~isfield(authorization, 'CommissioningAllowed') || ...
                    ~logical(authorization.CommissioningAllowed) || ...
                    ~isfield(authorization, 'ControllerFamily') || ...
                    string(authorization.ControllerFamily) ~= ...
                    "cartesian_drag"
                error('scopeguide:cartesianDrag:AuthorizationRequired', ...
                    'A valid Cartesian drag authorization is required.');
            end
            obj.Config = cfg;
            obj.Authorization = authorization;
            obj.EnableStateMachine = ...
                scopeguide.control.HandleEnableStateMachine(cfg);
            obj.Admittance = scopeguide.control.CartesianAdmittance6D(cfg);
            obj.QpController = ...
                scopeguide.control.CartesianVelocityQpController(cfg);
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
            [qState, geometry, geometryValid, geometryCode] = ...
                currentGeometry(robotState, obj.Config);
            if geometryValid && (~enableStatus.MotionPermitted || ...
                    enableStatus.ResetDynamicState || ...
                    any(~isfinite(obj.AnchorTransformBase(:))))
                obj.AnchorTransformBase = geometry.TBaseDragPoint;
            end
            if geometryValid && all(isfinite(obj.AnchorTransformBase(:)))
                relative = scopeguide.geometry.relativeDragPose( ...
                    obj.AnchorTransformBase, geometry.TBaseDragPoint);
                anchorRotation = obj.AnchorTransformBase(1:3, 1:3);
            else
                relative = zeros(6, 1);
                anchorRotation = eye(3);
            end

            mapping = struct([]);
            if geometryValid && validControlWrench(processed) && ...
                    safety.ForceFresh
                mapping = scopeguide.control.mapWrenchToDragPoint( ...
                    processed.ControlWrenchToolAtSensorOrigin, ...
                    geometry.TBaseFlange, obj.Config);
                wrenchDrag = mapping.WrenchDragAtDragPoint;
                rotationBaseFromDrag = ...
                    geometry.RotationBaseFromDrag;
            else
                wrenchDrag = zeros(6, 1);
                rotationBaseFromDrag = eye(3);
            end
            admittance = obj.Admittance.step(wrenchDrag, ...
                rotationBaseFromDrag, max(dtSec, eps), enableStatus);
            desiredTwist = admittance.DesiredTwistBase;
            travel = defaultTravelDiagnostics(relative);
            if geometryValid && enableStatus.MotionPermitted && ...
                    ~enableStatus.ResetDynamicState && ...
                    obj.Config.cartesianDrag.TravelLimitsEnabled
                [desiredTwist, travel] = ...
                    scopeguide.control.limitCartesianTwistForTravel( ...
                    desiredTwist, relative, anchorRotation, ...
                    max(dtSec, eps), obj.Config);
            end
            admittance.DesiredTwistBeforeTravelLimitBase = ...
                admittance.DesiredTwistBase;
            admittance.DesiredTwistBase = desiredTwist;
            admittance.TravelLimit = travel;
            admittance.Mapping = mapping;

            qp = obj.QpController.step(qState, desiredTwist, relative, ...
                anchorRotation, max(dtSec, eps), enableStatus);
            qdotCandidate = qp.QdotCommandRadSec;
            obj.QpFaultPending = qp.RequiresFault;
            commandStatus = enableStatus;
            commandStatus.CommandScale = 1;
            if ~qp.MotionCommandValid || ...
                    ~externalHealth.CommandChannelHealthy || ...
                    ~externalHealth.TrackingHealthy || ~geometryValid
                commandStatus.MotionPermitted = false;
                commandStatus.ResetDynamicState = true;
                obj.Admittance.reset();
            end
            if string(obj.Config.cartesianDrag.DofMode) == "hold"
                commandStatus.MotionPermitted = false;
                commandStatus.ResetDynamicState = true;
                qp.QdotCommandRadSec = zeros(6, 1);
            end
            integrated = obj.TargetIntegrator.step(qState, ...
                qp.QdotCommandRadSec, commandStatus, nowSec);

            output = struct();
            output.TimestampSec = nowSec;
            output.DtSec = dtSec;
            output.Safety = safety;
            output.EnableStatus = enableStatus;
            output.Admittance = admittance;
            output.Qp = qp;
            output.Integrator = integrated;
            output.Geometry = geometry;
            output.GeometryValid = geometryValid;
            output.GeometryStatusCode = geometryCode;
            output.AnchorTransformBase = obj.AnchorTransformBase;
            output.RelativeCoordinate = relative;
            output.DofMode = string(obj.Config.cartesianDrag.DofMode);
            output.DofMask = scopeguide.control.cartesianDragDofMask( ...
                obj.Config.cartesianDrag.DofMode);
            output.QdotCommandRadSec = qp.QdotCommandRadSec;
            output.QdotQpCandidateRadSec = qdotCandidate;
            output.QTargetCommandRad = integrated.QTargetRad;
            output.CommandEligible = enableStatus.MotionPermitted && ...
                qp.MotionCommandValid && integrated.IsValid && ...
                geometryValid && obj.Authorization.CommissioningAllowed && ...
                externalHealth.CommandChannelHealthy && ...
                externalHealth.TrackingHealthy && ...
                string(obj.Config.cartesianDrag.DofMode) ~= "hold";
            output.PredictionValid = output.CommandEligible;
            output.HardwareCommandAuthorized = ...
                obj.Authorization.CommissioningAllowed;
            output.MotionCommandSent = false;
            output.NoRcmConstraint = true;
            output.DoesNotSendHardwareCommands = true;
        end

        function reset(obj)
            obj.Admittance.reset();
            obj.QpController.reset();
            obj.TargetIntegrator.reset();
            obj.AnchorTransformBase(:) = NaN;
            obj.QpFaultPending = false;
            obj.LastStepTimeSec = NaN;
        end
    end
end

function safety = buildSafetyStatus(obj, processed, robotState, ...
        dtSec, externalHealth)
safety = scopeguide.types.enableSafetyStatus();
[safety.ForceFresh, safety.ForceStatusCode] = forceFreshStatus(processed);
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

function [q, geometry, valid, code] = currentGeometry(robotState, cfg)
q = zeros(6, 1);
geometry = struct([]);
valid = false;
code = "ROBOT_STATE_INVALID";
if ~validRobotJointState(robotState) || ~robotState.IsValid
    return;
end
q = double(robotState.JointPositionRad(:));
try
    geometry = scopeguide.geometry.computeDragPointKinematics(q, cfg);
    valid = true;
    code = "OK";
catch exception
    code = string(exception.identifier);
end
end

function diagnostics = defaultTravelDiagnostics(relative)
diagnostics = struct('RequestedTwistAnchor', zeros(6, 1), ...
    'LimitedTwistAnchor', zeros(6, 1), ...
    'ActiveAxes', false(6, 1), 'AnyBoundaryActive', false, ...
    'TravelLimitsEnabled', false, ...
    'RelativeCoordinate', relative, 'RelativeLimit', nan(6, 1), ...
    'RecoveryTolerance', nan(6, 1), ...
    'HardRelativeLimit', nan(6, 1), ...
    'RecoveryActiveAxes', false(6, 1), ...
    'AnyRecoveryActive', false, ...
    'OutsideHardBoundaryAxes', false(6, 1), ...
    'AnyOutsideHardBoundary', false, ...
    'NoRcmConstraint', true);
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
elseif isfield(processed.Safety, 'StopReasons')
    reasons = string(processed.Safety.StopReasons(:));
    reasons = reasons(~ismissing(reasons) & strlength(reasons) > 0);
    if ~isempty(reasons)
        code = strjoin(reasons, "+");
    end
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

function health = defaultExternalHealth()
health = struct('CommandChannelHealthy', true, ...
    'TrackingHealthy', true, 'StatusCode', "OK");
end

function validateExternalHealth(health)
required = {'CommandChannelHealthy', 'TrackingHealthy', 'StatusCode'};
if ~isstruct(health) || ~all(isfield(health, required)) || ...
        ~islogical(health.CommandChannelHealthy) || ...
        ~isscalar(health.CommandChannelHealthy) || ...
        ~islogical(health.TrackingHealthy) || ...
        ~isscalar(health.TrackingHealthy)
    error('scopeguide:cartesianDrag:InvalidExternalHealth', ...
        'External health is incomplete or invalid.');
end
end
