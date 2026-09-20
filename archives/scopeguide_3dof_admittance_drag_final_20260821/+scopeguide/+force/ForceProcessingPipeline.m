classdef ForceProcessingPipeline < handle
    %FORCEPROCESSINGPIPELINE Deterministic Stage 1 wrench processing chain.
    %
    % Chain:
    % quality gate -> static gravity/bias compensation -> optional guarded
    % residual baseline -> fast/slow filters -> continuous radial deadzone ->
    % raw/fast safety monitor. The control wrench remains referenced at the
    % HEX-H origin; the admittance layer shifts it to the configured RCM.

    properties (SetAccess = private)
        Config
        Calibration
        CalibrationProvenance
        Filters
        BaselineEstimator
        SafetyMonitor
        Counters
    end

    properties (Access = private)
        HasLastSample = false
        LastSequence = uint64(0)
        LastTimestampSec = NaN
        ForceDeadzoneActive = false
        MomentDeadzoneActive = false
        UseNominalDtOnNextSample = true
    end

    methods
        function obj = ForceProcessingPipeline(cfg)
            validateRcmAdmittanceConfig(cfg);
            [calibration, provenance] = loadHexHGravityCalibration( ...
                cfg.force.CalibrationFile);
            obj.Config = cfg;
            obj.Calibration = calibration;
            obj.CalibrationProvenance = provenance;
            obj.Filters = onrobot.WrenchFilterBank( ...
                cfg.sensor.SampleRateHz, cfg.force.FastCutoffHz, ...
                cfg.force.ControlCutoffHz);
            obj.BaselineEstimator = ...
                scopeguide.force.BaselineEstimator(cfg.force);
            obj.SafetyMonitor = ...
                scopeguide.force.WrenchSafetyMonitor(cfg.safety);
            obj.resetCounters();
        end

        function output = step(obj, sample, quaternionWxyz, ...
                nowMonotonicSec, context)
            if nargin < 5 || isempty(context)
                context = scopeguide.types.forceProcessingContext();
            end

            quality = obj.assessQuality( ...
                sample, quaternionWxyz, nowMonotonicSec);
            if ~quality.Valid
                obj.Counters.Invalid = obj.Counters.Invalid + uint64(1);
                raw = getRawOrNan(sample);
                safety = obj.SafetyMonitor.invalidDecision( ...
                    quality.StatusCode, raw);
                output = obj.blankOutput(sample, quality, safety);
                output.Counters = obj.Counters;
                return;
            end

            sequence = uint64(sample.Sequence);
            timestamp = double(sample.HostMonotonicSec);
            if obj.HasLastSample
                if obj.UseNominalDtOnNextSample
                    dt = 1 / obj.Config.sensor.SampleRateHz;
                else
                    dt = timestamp - obj.LastTimestampSec;
                end
                gap = double(sequence - obj.LastSequence) - 1;
                if gap > 0
                    obj.Counters.Dropped = obj.Counters.Dropped + uint64(gap);
                end
            else
                dt = 1 / obj.Config.sensor.SampleRateHz;
            end

            compensation = compensateHexHWrench( ...
                sample.RawWrenchSensor, quaternionWxyz, obj.Calibration);
            [baseline, baselineDiagnostics] = obj.BaselineEstimator.step( ...
                compensation.externalTool, dt, context);
            corrected = compensation.externalTool - baseline;
            if baselineDiagnostics.StartupJustCompleted
                % Do not retain pre-zero filter history or create a false
                % force-rate stop when the startup baseline is applied.
                obj.Filters.reset();
                obj.SafetyMonitor.reset();
                obj.ForceDeadzoneActive = false;
                obj.MomentDeadzoneActive = false;
            end
            [fastWrench, slowWrench] = obj.Filters.step(corrected);

            releaseRatio = obj.Config.force.DeadzoneReleaseRatio;
            [deadzoneForce, obj.ForceDeadzoneActive, forceDeadzone] = ...
                scopeguide.force.smoothVectorDeadzone( ...
                slowWrench(1:3), obj.Config.force.ForceDeadzoneN, ...
                releaseRatio * obj.Config.force.ForceDeadzoneN, ...
                obj.ForceDeadzoneActive);
            [deadzoneMoment, obj.MomentDeadzoneActive, momentDeadzone] = ...
                scopeguide.force.smoothVectorDeadzone( ...
                slowWrench(4:6), obj.Config.force.MomentDeadzoneNm, ...
                releaseRatio * obj.Config.force.MomentDeadzoneNm, ...
                obj.MomentDeadzoneActive);

            safety = obj.SafetyMonitor.step( ...
                sample.RawWrenchSensor, fastWrench, dt);
            baselineReady = baselineDiagnostics.Ready;
            neutralCheckPassed = baselineReady && ...
                norm(fastWrench(1:3)) <= ...
                    obj.Config.force.NeutralForceLimitN && ...
                norm(slowWrench(1:3)) <= ...
                    obj.Config.force.NeutralForceLimitN && ...
                norm(fastWrench(4:6)) <= ...
                    obj.Config.force.NeutralMomentLimitNm && ...
                norm(slowWrench(4:6)) <= ...
                    obj.Config.force.NeutralMomentLimitNm;
            motionInputValid = baselineReady && ~safety.StopRequested;
            controlEnabled = motionInputValid && context.HandleEnabled;
            if controlEnabled
                controlForce = deadzoneForce;
                controlWrench = [deadzoneForce; deadzoneMoment];
            else
                controlForce = zeros(3, 1);
                controlWrench = zeros(6, 1);
            end
            if safety.StopRequested
                obj.Counters.SafetyStops = ...
                    obj.Counters.SafetyStops + uint64(1);
            end

            obj.HasLastSample = true;
            obj.LastSequence = sequence;
            obj.LastTimestampSec = timestamp;
            obj.UseNominalDtOnNextSample = false;
            obj.Counters.Accepted = obj.Counters.Accepted + uint64(1);

            output = struct();
            output.Sequence = sequence;
            output.TimestampSec = timestamp;
            output.DtSec = dt;
            output.SampleAgeSec = quality.SampleAgeSec;
            output.Quality = quality;
            output.RawWrenchSensor = double(sample.RawWrenchSensor(:));
            output.UnbiasedWrenchSensor = compensation.unbiasedSensor;
            output.GravityWrenchSensor = compensation.gravitySensor;
            output.ExternalWrenchSensor = compensation.externalSensor;
            output.ExternalWrenchToolAtSensorOrigin = ...
                compensation.externalTool;
            output.BaselineWrenchTool = baseline;
            output.BaselineDiagnostics = baselineDiagnostics;
            output.CorrectedWrenchToolAtSensorOrigin = corrected;
            output.FastWrenchToolAtSensorOrigin = fastWrench;
            output.SlowWrenchToolAtSensorOrigin = slowWrench;
            output.DeadzoneWrenchToolAtSensorOrigin = ...
                [deadzoneForce; deadzoneMoment];
            output.ControlForceTool = controlForce;
            output.ControlWrenchToolAtSensorOrigin = controlWrench;
            output.ControlMomentForDiagnostics = deadzoneMoment;
            output.MomentEligibleForControl = true;
            output.NeutralCheckPassed = neutralCheckPassed;
            output.MotionInputValid = motionInputValid;
            output.OperationEnabled = context.HandleEnabled;
            output.ControlEnabled = controlEnabled;
            output.ForceDeadzone = forceDeadzone;
            output.MomentDeadzone = momentDeadzone;
            output.Safety = safety;
            output.Counters = obj.Counters;
            output.FilterDelayMs = obj.Filters.estimatedDelayMs();
        end

        function resetSignalState(obj)
            % Reset states across an offline discontinuity without losing
            % sequence and quality counters.
            obj.Filters.reset();
            obj.BaselineEstimator.reset();
            obj.SafetyMonitor.reset();
            obj.ForceDeadzoneActive = false;
            obj.MomentDeadzoneActive = false;
            obj.UseNominalDtOnNextSample = true;
        end

        function reset(obj)
            obj.startNewSegment();
            obj.resetCounters();
        end

        function startNewSegment(obj)
            % Preserve aggregate counters, but do not interpret an offline
            % recording gap between poses as packet loss or filter history.
            obj.resetSignalState();
            obj.HasLastSample = false;
            obj.LastSequence = uint64(0);
            obj.LastTimestampSec = NaN;
            obj.UseNominalDtOnNextSample = true;
        end
    end

    methods (Access = private)
        function quality = assessQuality(obj, sample, quaternion, nowSec)
            quality = struct();
            quality.Valid = false;
            quality.StatusCode = "UNASSESSED";
            quality.SampleAgeSec = NaN;
            quality.SequenceGap = 0;

            required = {'RawWrenchSensor', 'Sequence', ...
                'HostMonotonicSec', 'ReadDurationSec', ...
                'DeviceStatus', 'IsValid'};
            for index = 1:numel(required)
                if ~isstruct(sample) || ~isfield(sample, required{index})
                    quality.StatusCode = "MISSING_FIELD_" + ...
                        upper(string(required{index}));
                    return;
                end
            end
            if ~islogical(sample.IsValid) || ~isscalar(sample.IsValid) || ...
                    ~sample.IsValid
                quality.StatusCode = "SAMPLE_MARKED_INVALID";
                return;
            end
            raw = double(sample.RawWrenchSensor(:));
            if numel(raw) ~= 6 || any(~isfinite(raw))
                quality.StatusCode = "NONFINITE_WRENCH";
                return;
            end
            sequenceDouble = double(sample.Sequence);
            if ~isscalar(sequenceDouble) || ~isfinite(sequenceDouble) || ...
                    sequenceDouble < 1 || sequenceDouble ~= fix(sequenceDouble)
                quality.StatusCode = "INVALID_SEQUENCE";
                return;
            end
            sequence = uint64(sequenceDouble);
            timestamp = double(sample.HostMonotonicSec);
            if ~isscalar(timestamp) || ~isfinite(timestamp)
                quality.StatusCode = "INVALID_TIMESTAMP";
                return;
            end
            if ~isscalar(nowSec) || ~isfinite(nowSec)
                quality.StatusCode = "INVALID_CURRENT_TIME";
                return;
            end
            quality.SampleAgeSec = double(nowSec) - timestamp;
            if quality.SampleAgeSec < ...
                    -obj.Config.sensor.FutureTimestampToleranceSec
                quality.StatusCode = "FUTURE_TIMESTAMP";
                return;
            end
            if quality.SampleAgeSec > obj.Config.sensor.StaleSec
                quality.StatusCode = "STALE_SAMPLE";
                return;
            end
            readDuration = double(sample.ReadDurationSec);
            if ~isscalar(readDuration) || ~isfinite(readDuration) || ...
                    readDuration < 0 || readDuration > ...
                    obj.Config.sensor.MaximumReadDurationSec
                quality.StatusCode = "READ_DURATION_EXCEEDED";
                return;
            end
            if obj.Config.sensor.StatusCheckEnabled
                status = double(sample.DeviceStatus);
                if ~isscalar(status) || ~isfinite(status)
                    quality.StatusCode = "STATUS_UNAVAILABLE";
                    return;
                end
                if status ~= 0
                    quality.StatusCode = "DEVICE_STATUS_ERROR";
                    return;
                end
            end
            quaternion = double(quaternion(:));
            if numel(quaternion) ~= 4 || any(~isfinite(quaternion)) || ...
                    norm(quaternion) < 1e-9
                quality.StatusCode = "INVALID_QUATERNION";
                return;
            end
            if obj.HasLastSample
                if sequence == obj.LastSequence
                    quality.StatusCode = "DUPLICATE_SEQUENCE";
                    return;
                elseif sequence < obj.LastSequence
                    quality.StatusCode = "OUT_OF_ORDER_SEQUENCE";
                    return;
                end
                if timestamp <= obj.LastTimestampSec
                    quality.StatusCode = "NONINCREASING_TIMESTAMP";
                    return;
                end
                quality.SequenceGap = ...
                    double(sequence - obj.LastSequence) - 1;
            end

            quality.Valid = true;
            quality.StatusCode = "OK";
        end

        function output = blankOutput(obj, sample, quality, safety)
            output = struct();
            output.Sequence = getSequenceOrZero(sample);
            output.TimestampSec = getScalarOrNan(sample, ...
                'HostMonotonicSec');
            output.DtSec = NaN;
            output.SampleAgeSec = quality.SampleAgeSec;
            output.Quality = quality;
            output.RawWrenchSensor = getRawOrNan(sample);
            output.UnbiasedWrenchSensor = nan(6, 1);
            output.GravityWrenchSensor = nan(6, 1);
            output.ExternalWrenchSensor = nan(6, 1);
            output.ExternalWrenchToolAtSensorOrigin = nan(6, 1);
            output.BaselineWrenchTool = ...
                obj.BaselineEstimator.BaselineWrench;
            output.BaselineDiagnostics = struct();
            output.CorrectedWrenchToolAtSensorOrigin = nan(6, 1);
            output.FastWrenchToolAtSensorOrigin = nan(6, 1);
            output.SlowWrenchToolAtSensorOrigin = nan(6, 1);
            output.DeadzoneWrenchToolAtSensorOrigin = zeros(6, 1);
            output.ControlForceTool = zeros(3, 1);
            output.ControlWrenchToolAtSensorOrigin = zeros(6, 1);
            output.ControlMomentForDiagnostics = zeros(3, 1);
            output.MomentEligibleForControl = false;
            output.NeutralCheckPassed = false;
            output.MotionInputValid = false;
            output.OperationEnabled = false;
            output.ControlEnabled = false;
            output.ForceDeadzone = struct();
            output.MomentDeadzone = struct();
            output.Safety = safety;
            output.FilterDelayMs = obj.Filters.estimatedDelayMs();
        end

        function resetCounters(obj)
            obj.Counters = struct();
            obj.Counters.Accepted = uint64(0);
            obj.Counters.Invalid = uint64(0);
            obj.Counters.Dropped = uint64(0);
            obj.Counters.SafetyStops = uint64(0);
        end
    end
end

function raw = getRawOrNan(sample)
if isstruct(sample) && isfield(sample, 'RawWrenchSensor')
    value = double(sample.RawWrenchSensor(:));
    if numel(value) == 6
        raw = value;
        return;
    end
end
raw = nan(6, 1);
end

function sequence = getSequenceOrZero(sample)
sequence = uint64(0);
if isstruct(sample) && isfield(sample, 'Sequence')
    value = double(sample.Sequence);
    if isscalar(value) && isfinite(value) && value >= 0 && ...
            value == fix(value)
        sequence = uint64(value);
    end
end
end

function value = getScalarOrNan(sample, fieldName)
value = NaN;
if isstruct(sample) && isfield(sample, fieldName)
    candidate = double(sample.(fieldName));
    if isscalar(candidate) && isfinite(candidate)
        value = candidate;
    end
end
end
