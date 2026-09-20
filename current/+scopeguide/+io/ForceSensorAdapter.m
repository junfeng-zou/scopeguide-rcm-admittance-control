classdef ForceSensorAdapter < handle
    %FORCESENSORADAPTER Canonical live HEX-H acquisition/processing path.
    % Both the future controller and live visualizer must consume the
    % Processed output returned by readProcessed; neither should duplicate
    % sample conversion, compensation, filtering, deadzone or safety logic.

    properties (SetAccess = private)
        Config
        SensorConfig
        Client
        Pipeline
        StatusRateHz
        IsConnected = false
        DeviceStatus = NaN
        StatusReadCount = uint64(0)
        LastStatusReadSec = NaN
    end

    properties (Access = private)
        ClockStart
    end

    methods
        function obj = ForceSensorAdapter(cfg, sensorCfg, client, options)
            arguments
                cfg (1, 1) struct
                sensorCfg (1, 1) struct
                client = []
                options.StatusRateHz (1, 1) double ...
                    {mustBeFinite, mustBePositive} = 20
            end
            validateRcmAdmittanceConfig(cfg);
            obj.Config = cfg;
            obj.SensorConfig = sensorCfg;
            if isempty(client)
                obj.Client = onrobot.HexHClient(sensorCfg);
            else
                obj.Client = client;
            end
            obj.Pipeline = ...
                scopeguide.force.ForceProcessingPipeline(cfg);
            obj.StatusRateHz = options.StatusRateHz;
        end

        function connect(obj)
            if obj.IsConnected
                return;
            end
            obj.Client.connect();
            try
                obj.DeviceStatus = double(obj.Client.readStatus());
                obj.StatusReadCount = uint64(1);
                obj.ClockStart = tic;
                obj.LastStatusReadSec = 0;
                obj.IsConnected = true;
            catch exception
                obj.disconnect();
                rethrow(exception);
            end
        end

        function frame = readProcessed(obj, robotAdapter, context)
            if ~obj.IsConnected
                error('scopeguide:force:SensorAdapterNotConnected', ...
                    'Call ForceSensorAdapter.connect before reading.');
            end
            if nargin < 3 || isempty(context)
                context = struct([]);
            end

            completeReadClock = tic;
            nowSec = toc(obj.ClockStart);
            if nowSec - obj.LastStatusReadSec >= 1 / obj.StatusRateHz
                obj.DeviceStatus = double(obj.Client.readStatus());
                obj.StatusReadCount = ...
                    obj.StatusReadCount + uint64(1);
                obj.LastStatusReadSec = toc(obj.ClockStart);
            end
            rawSensorSample = obj.Client.readSample();
            robotState = robotAdapter.readState();
            completeReadDurationSec = toc(completeReadClock);

            sample = convertHexHSample( ...
                rawSensorSample, obj.DeviceStatus);
            if robotState.IsValid
                quaternion = ...
                    robotState.QuaternionBaseControllerWxyz;
            else
                quaternion = nan(4, 1);
            end
            if isa(context, 'function_handle')
                context = context(robotState);
            elseif isempty(context)
                context = defaultLiveContext(robotState);
            end
            processed = obj.Pipeline.step(sample, quaternion, ...
                sample.HostMonotonicSec, context);

            frame = struct();
            frame.RawSensorSample = rawSensorSample;
            frame.ForceSample = sample;
            frame.RobotState = robotState;
            frame.Processed = processed;
            frame.DeviceStatus = obj.DeviceStatus;
            frame.DeviceStatusAgeSec = ...
                toc(obj.ClockStart) - obj.LastStatusReadSec;
            frame.CompleteReadDurationSec = completeReadDurationSec;
            frame.StatusReadCount = obj.StatusReadCount;
            frame.MomentEligibleForControl = ...
                processed.MomentEligibleForControl;
        end

        function disconnect(obj)
            client = obj.Client;
            if ~isempty(client)
                try
                    client.disconnect();
                catch
                end
            end
            obj.IsConnected = false;
        end

        function delete(obj)
            obj.disconnect();
        end
    end
end

function sample = convertHexHSample(rawSample, deviceStatus)
required = {'wrench', 'sequence', 'monotonicTime', ...
    'readDuration', 'valid'};
for index = 1:numel(required)
    if ~isstruct(rawSample) || ~isfield(rawSample, required{index})
        error('scopeguide:force:InvalidHexHSample', ...
            'HEX-H sample is missing field %s.', required{index});
    end
end
sample = scopeguide.types.forceSample();
sample.RawWrenchSensor = double(rawSample.wrench(:));
sample.Sequence = uint64(rawSample.sequence);
sample.HostMonotonicSec = double(rawSample.monotonicTime);
sample.ReadDurationSec = double(rawSample.readDuration);
sample.SampleAgeSec = 0;
sample.DeviceStatus = double(deviceStatus);
sample.IsValid = logical(rawSample.valid);
if sample.IsValid
    sample.StatusCode = "LIVE_HEX_H";
else
    sample.StatusCode = "LIVE_HEX_H_INVALID";
end
end

function context = defaultLiveContext(robotState)
context = scopeguide.types.forceProcessingContext();
context.HandleEnabled = false;
context.RobotStationary = robotState.IsValid && ...
    norm(robotState.JointVelocityRadSec) < deg2rad(0.1);
context.NoContactConfirmed = false;
context.AllowBaselineUpdate = false;
end
