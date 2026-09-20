classdef EnableHandleAdapter < handle
    %ENABLEHANDLEADAPTER Timestamped external boolean deadman input.
    % The first implementation accepts software keyboard/mouse state.  It
    % owns no GUI and no hardware, so later GPIO or pedal sources can call
    % the same submit/read interface.

    properties (SetAccess = private)
        Config
        LatestEnabled (1, 1) logical = false
        LatestTimestampSec (1, 1) double = NaN
        LatestSource (1, 1) string = "uninitialized"
        LatestSupportsPhysicalMotion (1, 1) logical = false
        LatestInputValid (1, 1) logical = false
        LatestStatusCode (1, 1) string = "UNINITIALIZED"
        Sequence (1, 1) uint64 = uint64(0)
    end

    methods
        function obj = EnableHandleAdapter(cfg)
            arguments
                cfg (1, 1) struct
            end
            validateRcmAdmittanceConfig(cfg);
            obj.Config = cfg;
        end

        function submit(obj, enabled, timestampSec, options)
            arguments
                obj
                enabled (1, 1) logical
                timestampSec (1, 1) double
                options.Source (1, 1) string = "external_boolean"
                options.SourceValid (1, 1) logical = true
                options.SupportsPhysicalMotion (1, 1) logical = false
            end

            obj.Sequence = obj.Sequence + uint64(1);
            source = strtrim(options.Source);
            if strlength(source) == 0
                source = "unnamed_source";
            end
            if ~isfinite(timestampSec)
                obj.rejectInput("NONFINITE_TIMESTAMP", source);
                return;
            end
            if isfinite(obj.LatestTimestampSec) && ...
                    timestampSec < obj.LatestTimestampSec
                obj.rejectInput("TIMESTAMP_REGRESSION", source);
                return;
            end
            obj.LatestEnabled = enabled;
            obj.LatestTimestampSec = timestampSec;
            obj.LatestSource = source;
            obj.LatestSupportsPhysicalMotion = ...
                options.SupportsPhysicalMotion;
            obj.LatestInputValid = options.SourceValid;
            if options.SourceValid
                obj.LatestStatusCode = "OK";
            else
                obj.LatestEnabled = false;
                obj.LatestStatusCode = "SOURCE_INVALID";
            end
        end

        function sample = read(obj, nowSec)
            arguments
                obj
                nowSec (1, 1) double
            end
            if ~isfinite(nowSec)
                error('scopeguide:handle:InvalidReadTime', ...
                    'nowSec must be finite.');
            end

            sample = scopeguide.types.handleSample();
            sample.Sequence = obj.Sequence;
            sample.HostMonotonicSec = obj.LatestTimestampSec;
            sample.Source = obj.LatestSource;
            sample.SupportsPhysicalMotion = ...
                obj.LatestSupportsPhysicalMotion;
            if obj.Sequence == 0
                return;
            end
            sample.SampleAgeSec = nowSec - obj.LatestTimestampSec;
            if ~obj.LatestInputValid
                sample.StatusCode = obj.LatestStatusCode;
                return;
            end
            if sample.SampleAgeSec < ...
                    -obj.Config.handle.FutureTimestampToleranceSec
                sample.StatusCode = "FUTURE_TIMESTAMP";
                return;
            end
            if sample.SampleAgeSec > obj.Config.handle.StaleSec
                sample.StatusCode = "STALE_INPUT";
                return;
            end
            sample.Enabled = obj.LatestEnabled;
            sample.IsValid = true;
            sample.StatusCode = "OK";
        end

        function forceDisable(obj, timestampSec, reason)
            arguments
                obj
                timestampSec (1, 1) double
                reason (1, 1) string = "FORCED_DISABLE"
            end
            obj.submit(false, timestampSec, ...
                Source=reason, SourceValid=true, ...
                SupportsPhysicalMotion=false);
        end
    end

    methods (Access = private)
        function rejectInput(obj, statusCode, source)
            obj.LatestEnabled = false;
            obj.LatestSource = source;
            obj.LatestSupportsPhysicalMotion = false;
            obj.LatestInputValid = false;
            obj.LatestStatusCode = statusCode;
        end
    end
end
