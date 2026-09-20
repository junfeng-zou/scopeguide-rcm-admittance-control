classdef HexHClient < handle
    %HEXHCLIENT Modular Modbus TCP client for OnRobot HEX-E/H QC.
    %
    % Example:
    %   cfg = onrobot.defaultConfig();
    %   sensor = onrobot.HexHClient(cfg);
    %   sensor.connect();
    %   sample = sensor.readSample();
    %   disp(sample.wrench); % [Fx Fy Fz Tx Ty Tz]'

    properties (SetAccess = private)
        Config
        IsConnected = false
        BackendInUse = ''
        LastSample = struct([])
    end

    properties (Access = private)
        Transport = []
        TransactionID = uint16(0)
        Sequence = uint64(0)
        ClockStart
    end

    methods
        function obj = HexHClient(cfg)
            if nargin < 1 || isempty(cfg)
                cfg = onrobot.defaultConfig();
            end
            obj.validateConfig(cfg);
            obj.Config = cfg;
            obj.ClockStart = tic;
        end

        function connect(obj)
            if obj.IsConnected
                return;
            end

            requested = lower(char(obj.Config.Backend));
            if strcmp(requested, 'auto')
                if exist('modbus', 'file') == 2
                    try
                        obj.connectModbusBackend();
                    catch modbusError
                        warning('onrobot:HexHClient:ModbusFallback', ...
                            ['MATLAB modbus backend failed (%s). ' ...
                             'Trying the raw tcpclient backend.'], ...
                            modbusError.message);
                        obj.connectTcpClientBackend();
                    end
                else
                    obj.connectTcpClientBackend();
                end
            elseif strcmp(requested, 'modbus')
                obj.connectModbusBackend();
            elseif strcmp(requested, 'tcpclient')
                obj.connectTcpClientBackend();
            else
                error('onrobot:HexHClient:InvalidBackend', ...
                    'Backend must be auto, modbus, or tcpclient.');
            end

            obj.IsConnected = true;
            obj.Sequence = uint64(0);
            obj.ClockStart = tic;

            % Verify the complete path, not only that TCP port 502 is open.
            try
                obj.readRawWrenchRegisters();
            catch readError
                obj.disconnect();
                error('onrobot:HexHClient:ConnectionVerificationFailed', ...
                    ['Connected to %s:%d but could not read HEX-H registers. ' ...
                     'Check UnitID=64, Compute Box mode, and exclusive Modbus ' ...
                     'connection. Original error: %s'], ...
                    obj.Config.IPAddress, obj.Config.Port, readError.message);
            end
        end

        function disconnect(obj)
            obj.Transport = [];
            obj.IsConnected = false;
            obj.BackendInUse = '';
        end

        function delete(obj)
            obj.disconnect();
        end

        function sample = readSample(obj)
            %READSAMPLE Read one calibrated SI-unit wrench sample.
            obj.requireConnection();
            readTimer = tic;
            rawRegisters = obj.readRawWrenchRegisters();
            [force, torque, wrench, signedCounts] = ...
                onrobot.HexHClient.decodeWrenchRegisters( ...
                    rawRegisters, obj.Config.ForceScale, ...
                    obj.Config.TorqueScale);

            obj.Sequence = obj.Sequence + uint64(1);
            sample = struct();
            sample.sequence = obj.Sequence;
            sample.hostTimeUTC = datetime('now', 'TimeZone', 'UTC');
            sample.monotonicTime = toc(obj.ClockStart);
            sample.readDuration = toc(readTimer);
            sample.force = force;
            sample.torque = torque;
            sample.wrench = wrench;
            sample.forceNorm = norm(force);
            sample.torqueNorm = norm(torque);
            sample.rawCounts = signedCounts;
            sample.valid = all(isfinite(wrench));

            obj.LastSample = sample;
        end

        function values = readRawWrenchRegisters(obj)
            %READRAWWRENCHREGISTERS Return six uint16 Modbus registers.
            obj.requireConnection();
            values = obj.readHoldingRegisters( ...
                obj.Config.WrenchRegister, ...
                obj.Config.WrenchRegisterCount);
        end

        function status = readStatus(obj)
            %READSTATUS Read the HEX status register; zero means no error.
            obj.requireConnection();
            value = obj.readHoldingRegisters(obj.Config.StatusRegister, 1);
            status = double(value(1));
        end

        function zero(obj)
            %ZERO Set the current wrench as the sensor bias.
            % Call only when the sensor is unloaded and mechanically stable.
            obj.requireConnection();
            obj.writeSingleHoldingRegister(obj.Config.BiasRegister, uint16(1));
        end

        function unzero(obj)
            %UNZERO Restore the default sensor bias.
            obj.requireConnection();
            obj.writeSingleHoldingRegister(obj.Config.BiasRegister, uint16(0));
        end
    end

    methods (Static)
        function [force, torque, wrench, signedCounts] = ...
                decodeWrenchRegisters(registers, forceScale, torqueScale)
            %DECODEWRENCHREGISTERS Convert six uint16 registers to SI units.
            if nargin < 2
                forceScale = 0.1;
            end
            if nargin < 3
                torqueScale = 0.01;
            end
            if numel(registers) ~= 6
                error('onrobot:HexHClient:InvalidWrenchLength', ...
                    'Exactly six wrench registers are required.');
            end

            unsignedCounts = uint16(registers(:));
            signedCounts = double(unsignedCounts);
            negative = signedCounts >= 32768;
            signedCounts(negative) = signedCounts(negative) - 65536;
            force = signedCounts(1:3) * forceScale;
            torque = signedCounts(4:6) * torqueScale;
            wrench = [force; torque];
        end
    end

    methods (Access = private)
        function connectModbusBackend(obj)
            obj.Transport = modbus('tcpip', obj.Config.IPAddress, ...
                obj.Config.Port, 'Timeout', obj.Config.Timeout, ...
                'NumRetries', 1);
            obj.BackendInUse = 'modbus';
        end

        function connectTcpClientBackend(obj)
            if exist('tcpclient', 'file') ~= 2
                error('onrobot:HexHClient:MissingTcpClient', ...
                    ['Neither MATLAB modbus nor tcpclient is available. ' ...
                     'Install Industrial Communication Toolbox or a MATLAB ' ...
                     'release that provides tcpclient.']);
            end
            obj.Transport = tcpclient(obj.Config.IPAddress, obj.Config.Port, ...
                'Timeout', obj.Config.Timeout);
            obj.BackendInUse = 'tcpclient';
        end

        function values = readHoldingRegisters(obj, protocolAddress, count)
            if strcmp(obj.BackendInUse, 'modbus')
                % MATLAB uses one-based addresses and subtracts one internally.
                matlabAddress = double(protocolAddress) + 1;
                values = read(obj.Transport, 'holdingregs', matlabAddress, ...
                    double(count), obj.Config.UnitID, 'uint16');
                values = uint16(values(:));
            else
                payload = [obj.u16be(protocolAddress), obj.u16be(count)];
                pdu = obj.rawModbusTransaction(uint8(3), payload);
                expectedBytes = 2 * double(count);
                if numel(pdu) ~= expectedBytes + 2 || ...
                        pdu(2) ~= uint8(expectedBytes)
                    error('onrobot:HexHClient:MalformedReadResponse', ...
                        'Unexpected Modbus read response length.');
                end
                data = pdu(3:end);
                values = zeros(double(count), 1, 'uint16');
                for i = 1:double(count)
                    values(i) = bitor(bitshift(uint16(data(2*i-1)), 8), ...
                                      uint16(data(2*i)));
                end
            end
        end

        function writeSingleHoldingRegister(obj, protocolAddress, value)
            if strcmp(obj.BackendInUse, 'modbus')
                matlabAddress = double(protocolAddress) + 1;
                write(obj.Transport, 'holdingregs', matlabAddress, ...
                    double(value), obj.Config.UnitID, 'uint16');
            else
                payload = [obj.u16be(protocolAddress), obj.u16be(value)];
                pdu = obj.rawModbusTransaction(uint8(6), payload);
                if numel(pdu) ~= 5 || ~isequal(pdu(2:end), payload)
                    error('onrobot:HexHClient:MalformedWriteResponse', ...
                        'The Modbus write response did not echo the request.');
                end
            end
        end

        function pdu = rawModbusTransaction(obj, functionCode, payload)
            obj.TransactionID = obj.TransactionID + uint16(1);
            transactionID = obj.TransactionID;
            protocolID = uint16(0);
            lengthField = uint16(2 + numel(payload)); % Unit ID + function + data

            request = [obj.u16be(transactionID), obj.u16be(protocolID), ...
                obj.u16be(lengthField), uint8(obj.Config.UnitID), ...
                uint8(functionCode), uint8(payload)];
            write(obj.Transport, request, 'uint8');

            header = obj.readExact(7);
            responseTransactionID = obj.be16(header(1:2));
            responseProtocolID = obj.be16(header(3:4));
            responseLength = double(obj.be16(header(5:6)));

            if responseTransactionID ~= transactionID || responseProtocolID ~= 0
                error('onrobot:HexHClient:InvalidMbapHeader', ...
                    'Modbus transaction or protocol ID mismatch.');
            end
            if header(7) ~= uint8(obj.Config.UnitID) || responseLength < 2
                error('onrobot:HexHClient:InvalidUnitID', ...
                    'Unexpected Modbus Unit ID or response length.');
            end

            pdu = obj.readExact(responseLength - 1); % Unit ID already consumed
            if bitand(pdu(1), uint8(128)) ~= 0
                if numel(pdu) >= 2
                    exceptionCode = double(pdu(2));
                else
                    exceptionCode = -1;
                end
                error('onrobot:HexHClient:ModbusException', ...
                    'Modbus exception response, code %d.', exceptionCode);
            end
            if pdu(1) ~= uint8(functionCode)
                error('onrobot:HexHClient:FunctionMismatch', ...
                    'Unexpected Modbus function code in response.');
            end
        end

        function bytes = readExact(obj, count)
            bytes = zeros(1, count, 'uint8');
            received = 0;
            waitTimer = tic;
            while received < count
                available = obj.Transport.NumBytesAvailable;
                if available > 0
                    take = min(double(available), count - received);
                    chunk = read(obj.Transport, take, 'uint8');
                    bytes(received + (1:take)) = uint8(chunk(:));
                    received = received + take;
                elseif toc(waitTimer) > obj.Config.Timeout
                    error('onrobot:HexHClient:TcpTimeout', ...
                        'Timed out while waiting for %d Modbus TCP bytes.', count);
                else
                    pause(0.001);
                end
            end
        end

        function requireConnection(obj)
            if ~obj.IsConnected
                error('onrobot:HexHClient:NotConnected', ...
                    'Call connect() before accessing the sensor.');
            end
        end
    end

    methods (Static, Access = private)
        function bytes = u16be(value)
            value = uint16(value);
            bytes = uint8([bitshift(value, -8), bitand(value, 255)]);
        end

        function value = be16(bytes)
            value = bitor(bitshift(uint16(bytes(1)), 8), uint16(bytes(2)));
        end

        function validateConfig(cfg)
            required = {'IPAddress', 'Port', 'UnitID', 'Timeout', 'Backend', ...
                'WrenchRegister', 'WrenchRegisterCount', 'StatusRegister', ...
                'BiasRegister', 'ForceScale', 'TorqueScale'};
            for i = 1:numel(required)
                if ~isfield(cfg, required{i})
                    error('onrobot:HexHClient:MissingConfig', ...
                        'Configuration field %s is missing.', required{i});
                end
            end
            if cfg.WrenchRegisterCount ~= 6
                error('onrobot:HexHClient:InvalidConfig', ...
                    'WrenchRegisterCount must be 6.');
            end
        end
    end
end
