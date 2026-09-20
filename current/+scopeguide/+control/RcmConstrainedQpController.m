classdef RcmConstrainedQpController < handle
    %RcmConstrainedQpController Stateful Stage 4/6 QP integration wrapper.

    properties (SetAccess = private)
        Config
        QdotPreviousRadSec (6, 1) double = zeros(6, 1)
        AppliedRelativeCoordinate (3, 1) double = zeros(3, 1)
        ConsecutiveFailureCount (1, 1) uint32 = uint32(0)
        SolveCount (1, 1) uint64 = uint64(0)
        ResetCount (1, 1) uint64 = uint64(0)
    end

    methods
        function obj = RcmConstrainedQpController(cfg)
            arguments
                cfg (1, 1) struct
            end
            validateRcmAdmittanceConfig(cfg);
            obj.Config = cfg;
        end

        function reset(obj)
            obj.QdotPreviousRadSec(:) = 0;
            obj.AppliedRelativeCoordinate(:) = 0;
            obj.ConsecutiveFailureCount = uint32(0);
            obj.ResetCount = obj.ResetCount + 1;
        end

        function output = step(obj, jointPositionRad, ...
                desiredTipTwistBase, pRcmBaseM, dtSec, enableStatus)
            arguments
                obj
                jointPositionRad
                desiredTipTwistBase
                pRcmBaseM
                dtSec (1, 1) double
                enableStatus (1, 1) struct
            end
            obj.SolveCount = obj.SolveCount + 1;
            if ~validEnableStatus(enableStatus)
                obj.QdotPreviousRadSec(:) = 0;
                output = wrapperFailure("INVALID_ENABLE_STATUS");
                output.RequiresFault = true;
                output.QpHealthy = false;
                return;
            end
            if ~enableStatus.MotionPermitted || ...
                    enableStatus.ResetDynamicState
                obj.reset();
                output = wrapperFailure("DYNAMIC_STATE_RESET");
                output.StatusCode = "QP_RESET_ZERO_OUTPUT";
                output.RequiresFault = false;
                output.QpHealthy = true;
                output.DynamicStateReset = true;
                output.AppliedRelativeCoordinate = ...
                    obj.AppliedRelativeCoordinate;
                return;
            end

            solution = scopeguide.control.solveRcmConstrainedQp( ...
                jointPositionRad, obj.QdotPreviousRadSec, ...
                desiredTipTwistBase, obj.AppliedRelativeCoordinate, ...
                pRcmBaseM, dtSec, obj.Config);
            if solution.MotionCommandValid
                obj.QdotPreviousRadSec = solution.QdotCommandRadSec;
                obj.AppliedRelativeCoordinate = ...
                    solution.PredictedRelativeCoordinate;
                obj.ConsecutiveFailureCount = uint32(0);
            else
                obj.QdotPreviousRadSec(:) = 0;
                obj.ConsecutiveFailureCount = ...
                    obj.ConsecutiveFailureCount + uint32(1);
            end
            output = solution;
            output.ConsecutiveFailureCount = ...
                obj.ConsecutiveFailureCount;
            output.RequiresFault = obj.ConsecutiveFailureCount >= ...
                obj.Config.qp.MaximumConsecutiveFailures;
            output.QpHealthy = ~output.RequiresFault;
            output.DynamicStateReset = false;
            output.AppliedRelativeCoordinate = ...
                obj.AppliedRelativeCoordinate;
            output.DoesNotSendHardwareCommands = true;
        end
    end
end

function valid = validEnableStatus(status)
required = {'MotionPermitted', 'ResetDynamicState'};
valid = isstruct(status) && all(isfield(status, required)) && ...
    islogical(status.MotionPermitted) && isscalar(status.MotionPermitted) && ...
    islogical(status.ResetDynamicState) && isscalar(status.ResetDynamicState);
end

function output = wrapperFailure(code)
output = struct();
output.StatusCode = "QP_FAILED_ZERO_OUTPUT";
output.FailureCode = string(code);
output.MotionCommandValid = false;
output.QdotCommandRadSec = zeros(6, 1);
output.AchievedTipTwistBase = zeros(6, 1);
output.AchievedGeneralizedVelocity = zeros(3, 1);
output.PredictedRcmErrorNormM = NaN;
output.PredictedRelativeCoordinate = zeros(3, 1);
output.SolveTimeSec = NaN;
output.ExitFlag = NaN;
output.ConsecutiveFailureCount = uint32(0);
output.RequiresFault = false;
output.QpHealthy = false;
output.DynamicStateReset = false;
output.AppliedRelativeCoordinate = zeros(3, 1);
output.DoesNotSendHardwareCommands = true;
end
