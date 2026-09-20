classdef KeyboardHoldFilter < handle
    %KEYBOARDHOLDFILTER Suppress Linux/X11 auto-repeat release/press pairs.
    % A release remains pending for a short confirmation window. A repeated
    % press inside that window cancels the synthetic release. This class is
    % for keyboard GUI dry-run input only, never a physical deadman.

    properties (SetAccess = private)
        ReleaseConfirmSec (1, 1) double
        RepeatWatchdogSec (1, 1) double
        Held (1, 1) logical = false
        ReleasePending (1, 1) logical = false
        ReleaseDeadlineSec (1, 1) double = Inf
        RepeatObserved (1, 1) logical = false
        LastPressTimeSec (1, 1) double = NaN
        LastUpdateTimeSec (1, 1) double = NaN
        StatusCode (1, 1) string = "RELEASED"
        SuppressedRepeatReleaseCount (1, 1) uint64 = uint64(0)
        ConfirmedReleaseCount (1, 1) uint64 = uint64(0)
        WatchdogReleaseCount (1, 1) uint64 = uint64(0)
    end

    methods
        function obj = KeyboardHoldFilter(releaseConfirmSec, repeatWatchdogSec)
            arguments
                releaseConfirmSec (1, 1) double
                repeatWatchdogSec (1, 1) double = 0.200
            end
            if ~isfinite(releaseConfirmSec) || ...
                    releaseConfirmSec < 0 || releaseConfirmSec > 0.050
                error('scopeguide:handle:InvalidKeyboardReleaseConfirmSec', ...
                    'Release confirmation must lie in [0, 0.050] seconds.');
            end
            if ~isfinite(repeatWatchdogSec) || ...
                    repeatWatchdogSec <= releaseConfirmSec || ...
                    repeatWatchdogSec > 0.500
                error('scopeguide:handle:InvalidKeyboardRepeatWatchdogSec', ...
                    ['Repeat watchdog must exceed the release confirmation ' ...
                    'window and must not exceed 0.500 seconds.']);
            end
            obj.ReleaseConfirmSec = releaseConfirmSec;
            obj.RepeatWatchdogSec = repeatWatchdogSec;
        end

        function press(obj, nowSec)
            if ~obj.acceptTime(nowSec)
                return;
            end
            repeatedPress = obj.Held;
            if obj.ReleasePending
                obj.SuppressedRepeatReleaseCount = ...
                    obj.SuppressedRepeatReleaseCount + uint64(1);
            end
            if repeatedPress
                obj.RepeatObserved = true;
            else
                obj.RepeatObserved = false;
            end
            obj.Held = true;
            obj.ReleasePending = false;
            obj.ReleaseDeadlineSec = Inf;
            obj.LastPressTimeSec = nowSec;
            obj.StatusCode = "HELD";
        end

        function release(obj, nowSec)
            if ~obj.acceptTime(nowSec)
                return;
            end
            if ~obj.Held
                obj.forceRelease("RELEASED");
                return;
            end
            if obj.ReleaseConfirmSec == 0
                obj.ConfirmedReleaseCount = ...
                    obj.ConfirmedReleaseCount + uint64(1);
                obj.forceRelease("RELEASE_CONFIRMED");
                return;
            end
            obj.ReleasePending = true;
            obj.ReleaseDeadlineSec = nowSec + obj.ReleaseConfirmSec;
            obj.StatusCode = "RELEASE_PENDING";
        end

        function tick(obj, nowSec)
            if ~obj.acceptTime(nowSec)
                return;
            end
            if obj.ReleasePending && nowSec >= obj.ReleaseDeadlineSec
                obj.ConfirmedReleaseCount = ...
                    obj.ConfirmedReleaseCount + uint64(1);
                obj.forceRelease("RELEASE_CONFIRMED");
                return;
            end
            if obj.Held && obj.RepeatObserved && ...
                    isfinite(obj.LastPressTimeSec) && ...
                    nowSec - obj.LastPressTimeSec >= obj.RepeatWatchdogSec
                obj.WatchdogReleaseCount = ...
                    obj.WatchdogReleaseCount + uint64(1);
                obj.forceRelease("KEY_REPEAT_WATCHDOG_RELEASE");
            end
        end

        function forceRelease(obj, statusCode)
            arguments
                obj
                statusCode (1, 1) string = "FORCED_RELEASE"
            end
            obj.Held = false;
            obj.ReleasePending = false;
            obj.ReleaseDeadlineSec = Inf;
            obj.RepeatObserved = false;
            obj.LastPressTimeSec = NaN;
            obj.StatusCode = statusCode;
        end
    end

    methods (Access = private)
        function accepted = acceptTime(obj, nowSec)
            accepted = isfinite(nowSec) && ...
                (~isfinite(obj.LastUpdateTimeSec) || ...
                nowSec >= obj.LastUpdateTimeSec);
            if accepted
                obj.LastUpdateTimeSec = nowSec;
            else
                obj.forceRelease("FILTER_TIME_INVALID");
            end
        end
    end
end
