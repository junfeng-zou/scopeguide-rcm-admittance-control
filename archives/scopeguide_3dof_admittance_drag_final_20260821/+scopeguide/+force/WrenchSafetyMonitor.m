classdef WrenchSafetyMonitor < handle
    %WRENCHSAFETYMONITOR Raw and fast-branch offline safety decisions.

    properties (SetAccess = private)
        Config
        HasPreviousFastForce = false
        PreviousFastForce = zeros(3, 1)
    end

    methods
        function obj = WrenchSafetyMonitor(safetyConfig)
            obj.Config = safetyConfig;
        end

        function decision = step(obj, rawWrench, fastWrench, dt)
            raw = finiteWrench(rawWrench, 'rawWrench');
            fast = finiteWrench(fastWrench, 'fastWrench');
            if ~isscalar(dt) || ~isfinite(dt) || dt <= 0
                error('scopeguide:force:InvalidSafetyDt', ...
                    'dt must be finite and positive.');
            end

            if obj.HasPreviousFastForce
                forceRate = norm(fast(1:3) - obj.PreviousFastForce) / dt;
            else
                forceRate = 0;
            end
            obj.PreviousFastForce = fast(1:3);
            obj.HasPreviousFastForce = true;

            rawForceNorm = norm(raw(1:3));
            rawMomentNorm = norm(raw(4:6));
            fastForceNorm = norm(fast(1:3));
            fastMomentNorm = norm(fast(4:6));

            stopReasons = strings(0, 1);
            warningReasons = strings(0, 1);
            if rawForceNorm >= obj.Config.RawForceStopN
                stopReasons(end + 1, 1) = "RAW_FORCE_STOP";
            end
            if rawMomentNorm >= obj.Config.RawMomentStopNm
                stopReasons(end + 1, 1) = "RAW_MOMENT_STOP";
            end
            if fastForceNorm >= obj.Config.FastForceStopN
                stopReasons(end + 1, 1) = "FAST_FORCE_STOP";
            elseif fastForceNorm >= obj.Config.FastForceWarningN
                warningReasons(end + 1, 1) = "FAST_FORCE_WARNING";
            end
            if fastMomentNorm >= obj.Config.FastMomentStopNm
                stopReasons(end + 1, 1) = "FAST_MOMENT_STOP";
            elseif fastMomentNorm >= obj.Config.FastMomentWarningNm
                warningReasons(end + 1, 1) = "FAST_MOMENT_WARNING";
            end
            if forceRate >= obj.Config.FastForceRateStopNSec
                stopReasons(end + 1, 1) = "FAST_FORCE_RATE_STOP";
            end

            decision = makeDecision(rawForceNorm, rawMomentNorm, ...
                fastForceNorm, fastMomentNorm, forceRate, ...
                warningReasons, stopReasons);
        end

        function decision = invalidDecision(~, reason, rawWrench)
            rawForceNorm = NaN;
            rawMomentNorm = NaN;
            raw = double(rawWrench(:));
            if numel(raw) == 6 && all(isfinite(raw))
                rawForceNorm = norm(raw(1:3));
                rawMomentNorm = norm(raw(4:6));
            end
            decision = makeDecision(rawForceNorm, rawMomentNorm, ...
                NaN, NaN, NaN, strings(0, 1), ...
                "INVALID_DATA_" + string(reason));
        end

        function reset(obj)
            obj.HasPreviousFastForce = false;
            obj.PreviousFastForce = zeros(3, 1);
        end
    end
end

function wrench = finiteWrench(value, name)
wrench = double(value(:));
if numel(wrench) ~= 6 || any(~isfinite(wrench))
    error('scopeguide:force:InvalidSafetyWrench', ...
        '%s must be a finite 6-by-1 vector.', name);
end
end

function decision = makeDecision(rawForceNorm, rawMomentNorm, ...
        fastForceNorm, fastMomentNorm, forceRate, ...
        warningReasons, stopReasons)
decision = struct();
decision.RawForceNormN = rawForceNorm;
decision.RawMomentNormNm = rawMomentNorm;
decision.FastForceNormN = fastForceNorm;
decision.FastMomentNormNm = fastMomentNorm;
decision.FastForceRateNSec = forceRate;
decision.WarningReasons = string(warningReasons(:));
decision.StopReasons = string(stopReasons(:));
decision.WarningActive = ~isempty(decision.WarningReasons);
decision.StopRequested = ~isempty(decision.StopReasons);
decision.AllowControlInput = ~decision.StopRequested;
end
