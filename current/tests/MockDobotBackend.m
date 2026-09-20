classdef MockDobotBackend < handle
    %MOCKDOBOTBACKEND Offline test double; never opens a network socket.

    properties
        IsConnected = false
        ConnectCallCount = uint64(0)
        DisconnectCallCount = uint64(0)
        ServoJCallCount = uint64(0)
        Snapshot
    end

    methods
        function obj = MockDobotBackend(snapshot)
            obj.Snapshot = snapshot;
        end

        function Connect(obj)
            obj.ConnectCallCount = obj.ConnectCallCount + uint64(1);
            obj.IsConnected = true;
        end

        function Disconnect(obj)
            obj.DisconnectCallCount = obj.DisconnectCallCount + uint64(1);
            obj.IsConnected = false;
        end

        function snapshot = GetStateSnapshot(obj)
            if ~obj.IsConnected
                error('ZJFDobotCR5:NoFeedback', 'Mock is disconnected.');
            end
            snapshot = obj.Snapshot;
        end

        function ServoJ(obj, varargin) %#ok<INUSD>
            obj.ServoJCallCount = obj.ServoJCallCount + uint64(1);
            error('scopeguide:tests:UnexpectedServoJ', ...
                'RobotAdapter must never call the backend ServoJ in Stage 2.');
        end
    end
end
