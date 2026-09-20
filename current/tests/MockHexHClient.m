classdef MockHexHClient < handle
    %MOCKHEXHCLIENT Deterministic no-network source for adapter tests.

    properties
        IsConnected = false
        Sequence = uint64(0)
        MonotonicTimeSec = 0
        DeviceStatus = 0
        % No-contact wrench predicted by the current 2026-08-07
        % calibration at the Nova5 zero-joint gravity reference.
        Wrench = [-0.537773104711917; -9.95801683561317; ...
            -4.07223668460024; 0.869417301642276; ...
            0.195965742994837; -0.065721371492331]
        ReadDurationSec = 0.001
        ConnectCallCount = uint64(0)
        DisconnectCallCount = uint64(0)
        StatusReadCount = uint64(0)
    end

    methods
        function connect(obj)
            obj.ConnectCallCount = obj.ConnectCallCount + uint64(1);
            obj.IsConnected = true;
        end

        function disconnect(obj)
            obj.DisconnectCallCount = ...
                obj.DisconnectCallCount + uint64(1);
            obj.IsConnected = false;
        end

        function status = readStatus(obj)
            obj.StatusReadCount = obj.StatusReadCount + uint64(1);
            status = obj.DeviceStatus;
        end

        function sample = readSample(obj)
            if ~obj.IsConnected
                error('scopeguide:tests:MockSensorDisconnected', ...
                    'Mock HEX-H is disconnected.');
            end
            obj.Sequence = obj.Sequence + uint64(1);
            obj.MonotonicTimeSec = obj.MonotonicTimeSec + 0.005;
            sample = struct();
            sample.sequence = obj.Sequence;
            sample.monotonicTime = obj.MonotonicTimeSec;
            sample.readDuration = obj.ReadDurationSec;
            sample.wrench = obj.Wrench;
            sample.force = obj.Wrench(1:3);
            sample.torque = obj.Wrench(4:6);
            sample.valid = true;
        end
    end
end
