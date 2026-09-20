classdef CartesianVelocityQpController < handle
    %CARTESIANVELOCITYQPCONTROLLER Stateful no-RCM QP wrapper.

    properties (SetAccess = private)
        Config
        QdotPreviousRadSec (6, 1) double = zeros(6, 1)
        ConsecutiveFailureCount (1, 1) uint32 = uint32(0)
        SolveCount (1, 1) uint64 = uint64(0)
    end

    methods
        function obj = CartesianVelocityQpController(cfg)
            validateCartesianAdmittanceDragConfig(cfg);
            obj.Config = cfg;
        end

        function reset(obj)
            obj.QdotPreviousRadSec(:) = 0;
            obj.ConsecutiveFailureCount = uint32(0);
        end

        function output = step(obj, jointPositionRad, desiredTwistBase, ...
                relativeCoordinate, rotationBaseFromAnchor, dtSec, ...
                enableStatus)
            obj.SolveCount = obj.SolveCount + uint64(1);
            if ~validEnable(enableStatus)
                obj.QdotPreviousRadSec(:) = 0;
                output = failureOutput("INVALID_ENABLE_STATUS");
                output.RequiresFault = true;
                return;
            end
            if ~enableStatus.MotionPermitted || ...
                    enableStatus.ResetDynamicState
                obj.reset();
                output = failureOutput("DYNAMIC_STATE_RESET");
                output.StatusCode = "QP_RESET_ZERO_OUTPUT";
                output.QpHealthy = true;
                output.RequiresFault = false;
                output.DynamicStateReset = true;
                return;
            end
            solution = scopeguide.control.solveCartesianVelocityQp( ...
                jointPositionRad, obj.QdotPreviousRadSec, ...
                desiredTwistBase, relativeCoordinate, ...
                rotationBaseFromAnchor, dtSec, obj.Config);
            if solution.MotionCommandValid
                obj.QdotPreviousRadSec = solution.QdotCommandRadSec;
                obj.ConsecutiveFailureCount = uint32(0);
            else
                obj.QdotPreviousRadSec(:) = 0;
                obj.ConsecutiveFailureCount = ...
                    obj.ConsecutiveFailureCount + uint32(1);
                % Failure paths are displayed and logged while the FSM
                % moves to FAULT.  Keep telemetry finite and explicitly
                % zero-commanded; NaN solver placeholders must never tear
                % down the client dashboard.
                solution.QdotCommandRadSec = zeros(6, 1);
                solution.AchievedTwistBase = zeros(6, 1);
                solution.AchievedTwistAnchor = zeros(6, 1);
                solution.PredictedRelativeCoordinate = ...
                    double(relativeCoordinate(:));
            end
            output = solution;
            output.ConsecutiveFailureCount = obj.ConsecutiveFailureCount;
            output.RequiresFault = obj.ConsecutiveFailureCount >= ...
                obj.Config.qp.MaximumConsecutiveFailures;
            output.QpHealthy = ~output.RequiresFault;
            output.DynamicStateReset = false;
        end
    end
end

function valid = validEnable(status)
valid = isstruct(status) && ...
    all(isfield(status, {'MotionPermitted', 'ResetDynamicState'})) && ...
    islogical(status.MotionPermitted) && isscalar(status.MotionPermitted) && ...
    islogical(status.ResetDynamicState) && ...
    isscalar(status.ResetDynamicState);
end

function output = failureOutput(code)
output = struct();
output.StatusCode = "QP_FAILED_ZERO_OUTPUT";
output.FailureCode = string(code);
output.MotionCommandValid = false;
output.QdotCommandRadSec = zeros(6, 1);
output.AchievedTwistBase = zeros(6, 1);
output.AchievedTwistAnchor = zeros(6, 1);
output.PredictedRelativeCoordinate = zeros(6, 1);
output.SoftRelativeLimit = nan(6, 1);
output.RecoveryTolerance = nan(6, 1);
output.HardRelativeLimit = nan(6, 1);
output.ActiveRelativeLowerBound = nan(6, 1);
output.ActiveRelativeUpperBound = nan(6, 1);
output.RecoveryActiveAxes = false(6, 1);
output.TravelRecoveryActive = false;
output.TravelLimitsEnabled = false;
output.SolveTimeSec = NaN;
output.ExitFlag = NaN;
output.ConsecutiveFailureCount = uint32(0);
output.RequiresFault = false;
output.QpHealthy = false;
output.DynamicStateReset = false;
output.NoRcmConstraint = true;
output.DoesNotSendHardwareCommands = true;
end
