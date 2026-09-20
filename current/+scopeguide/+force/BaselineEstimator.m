classdef BaselineEstimator < handle
    %BASELINEESTIMATOR Startup software zero plus guarded slow drift tracking.

    properties (SetAccess = private)
        Enabled
        Config
        BaselineWrench = zeros(6, 1)
        StartupBaselineWrench = zeros(6, 1)
        Ready = true
        StartupElapsedSec = 0
        StartupSampleCount = uint64(0)
        EligibleDurationSec = 0
        UpdateCount = uint64(0)
    end

    properties (Access = private)
        StartupWrenchSum = zeros(6, 1)
    end

    methods
        function obj = BaselineEstimator(forceConfig)
            obj.Enabled = logical(forceConfig.AutomaticBaselineEnabled);
            obj.Config = forceConfig.Baseline;
            obj.Ready = ~obj.Enabled || ~obj.Config.StartupZeroEnabled;
        end

        function [baseline, diagnostics] = step(obj, wrench, dt, context)
            wrench = double(wrench(:));
            if numel(wrench) ~= 6 || any(~isfinite(wrench))
                error('scopeguide:force:InvalidBaselineWrench', ...
                    'wrench must be a finite 6-by-1 vector.');
            end
            if ~isscalar(dt) || ~isfinite(dt) || dt <= 0
                error('scopeguide:force:InvalidBaselineDt', ...
                    'dt must be finite and positive.');
            end
            validateContext(context);

            startupJustCompleted = false;
            if ~obj.Enabled
                eligible = false;
                reason = "DISABLED";
            else
                [eligible, reason] = contextGate(context);
            end

            updated = false;
            if obj.Enabled && ~obj.Ready
                if eligible
                    obj.StartupElapsedSec = obj.StartupElapsedSec + dt;
                    obj.StartupSampleCount = ...
                        obj.StartupSampleCount + uint64(1);
                    obj.StartupWrenchSum = obj.StartupWrenchSum + wrench;
                    reason = "STARTUP_COLLECTING";
                    if obj.StartupElapsedSec >= ...
                            obj.Config.StartupDurationSec
                        candidate = obj.StartupWrenchSum / ...
                            double(obj.StartupSampleCount);
                        if norm(candidate(1:3)) > ...
                                obj.Config.StartupMaximumForceNormN
                            reason = "STARTUP_FORCE_LIMIT";
                            obj.resetStartupCollection();
                        elseif norm(candidate(4:6)) > ...
                                obj.Config.StartupMaximumMomentNormNm
                            reason = "STARTUP_MOMENT_LIMIT";
                            obj.resetStartupCollection();
                        else
                            obj.BaselineWrench = candidate;
                            obj.StartupBaselineWrench = candidate;
                            obj.Ready = true;
                            obj.EligibleDurationSec = 0;
                            obj.UpdateCount = obj.UpdateCount + uint64(1);
                            updated = true;
                            startupJustCompleted = true;
                            reason = "STARTUP_COMPLETE";
                        end
                    end
                else
                    obj.resetStartupCollection();
                    reason = "STARTUP_" + reason;
                end
            elseif eligible
                residual = wrench - obj.BaselineWrench;
                if norm(residual(1:3)) > ...
                        obj.Config.EligibilityForceNormN
                    eligible = false;
                    reason = "FORCE_GATE";
                elseif norm(residual(4:6)) > ...
                        obj.Config.EligibilityMomentNormNm
                    eligible = false;
                    reason = "MOMENT_GATE";
                end
            end

            if obj.Ready && eligible && ~startupJustCompleted
                obj.EligibleDurationSec = obj.EligibleDurationSec + dt;
                if obj.EligibleDurationSec >= ...
                        obj.Config.MinimumEligibleDurationSec
                    alpha = 1 - exp(-dt / ...
                        obj.Config.UpdateTimeConstantSec);
                    trackingMask = double( ...
                        obj.Config.TrackingComponentMask(:));
                    requestedDelta = alpha * ...
                        (wrench - obj.BaselineWrench) .* trackingMask;
                    forceDelta = limitNorm(requestedDelta(1:3), ...
                        obj.Config.MaximumForceUpdateRateNSec * dt);
                    momentDelta = limitNorm(requestedDelta(4:6), ...
                        obj.Config.MaximumMomentUpdateRateNmSec * dt);
                    candidate = obj.BaselineWrench + ...
                        [forceDelta; momentDelta];
                    trackingOffset = candidate - ...
                        obj.StartupBaselineWrench;
                    candidate = obj.StartupBaselineWrench + [ ...
                        limitNorm(trackingOffset(1:3), ...
                            obj.Config.MaximumForceOffsetN); ...
                        limitNorm(trackingOffset(4:6), ...
                            obj.Config.MaximumMomentOffsetNm)];
                    updated = norm(candidate - obj.BaselineWrench) > 0;
                    obj.BaselineWrench = candidate;
                    if updated
                        obj.UpdateCount = obj.UpdateCount + uint64(1);
                    end
                else
                    reason = "QUALIFYING";
                end
            elseif obj.Ready && ~eligible
                obj.EligibleDurationSec = 0;
            end

            baseline = obj.BaselineWrench;
            diagnostics = struct();
            diagnostics.Enabled = obj.Enabled;
            diagnostics.Ready = obj.Ready;
            diagnostics.StartupJustCompleted = startupJustCompleted;
            diagnostics.StartupElapsedSec = obj.StartupElapsedSec;
            diagnostics.StartupDurationSec = ...
                obj.Config.StartupDurationSec;
            diagnostics.StartupProgress = min(1, ...
                obj.StartupElapsedSec / obj.Config.StartupDurationSec);
            diagnostics.StartupSampleCount = obj.StartupSampleCount;
            diagnostics.StartupBaselineWrench = ...
                obj.StartupBaselineWrench;
            diagnostics.Eligible = eligible;
            diagnostics.EligibilityReason = reason;
            diagnostics.EligibleDurationSec = obj.EligibleDurationSec;
            diagnostics.Updated = updated;
            diagnostics.UpdateCount = obj.UpdateCount;
            diagnostics.BaselineWrench = baseline;
        end

        function reset(obj)
            obj.BaselineWrench = zeros(6, 1);
            obj.StartupBaselineWrench = zeros(6, 1);
            obj.Ready = ~obj.Enabled || ...
                ~obj.Config.StartupZeroEnabled;
            obj.resetStartupCollection();
            obj.EligibleDurationSec = 0;
            obj.UpdateCount = uint64(0);
        end
    end

    methods (Access = private)
        function resetStartupCollection(obj)
            obj.StartupElapsedSec = 0;
            obj.StartupSampleCount = uint64(0);
            obj.StartupWrenchSum = zeros(6, 1);
        end
    end
end

function [eligible, reason] = contextGate(context)
eligible = false;
if ~context.AllowBaselineUpdate
    reason = "UPDATE_NOT_ALLOWED";
elseif context.HandleEnabled
    reason = "HANDLE_ENABLED";
elseif ~context.RobotStationary
    reason = "ROBOT_MOVING";
elseif ~context.NoContactConfirmed
    reason = "CONTACT_NOT_EXCLUDED";
else
    eligible = true;
    reason = "ELIGIBLE";
end
end

function validateContext(context)
required = {'HandleEnabled', 'RobotStationary', ...
    'NoContactConfirmed', 'AllowBaselineUpdate'};
for index = 1:numel(required)
    name = required{index};
    if ~isstruct(context) || ~isfield(context, name) || ...
            ~islogical(context.(name)) || ~isscalar(context.(name))
        error('scopeguide:force:InvalidBaselineContext', ...
            'context.%s must be a logical scalar.', name);
    end
end
end

function limited = limitNorm(vector, maximumNorm)
vectorNorm = norm(vector);
if vectorNorm > maximumNorm
    limited = vector * (maximumNorm / vectorNorm);
else
    limited = vector;
end
end
