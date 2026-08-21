function cfg = defaultConfig()
%DEFAULTCONFIG Default configuration for an OnRobot HEX-E/H QC Compute Box.
%
% Protocol addresses below are zero-based Modbus addresses from the OnRobot
% manual. HexHClient converts them when MATLAB's modbus backend is used.

cfg.IPAddress = '192.168.50.201';
cfg.Port = 502;
cfg.UnitID = 64;                 % HEX-E/H QC fixed device address (0x40)
cfg.Timeout = 0.5;               % Seconds
cfg.Backend = 'auto';            % 'auto', 'modbus', or 'tcpclient'

cfg.WrenchRegister = 259;        % 0x0103, Fx starts here
cfg.WrenchRegisterCount = 6;     % Fx, Fy, Fz, Tx, Ty, Tz
cfg.StatusRegister = 257;        % 0x0101, read separately if required
cfg.BiasRegister = 0;            % 0x0000: 0=un-zero, 1=zero

cfg.ForceScale = 0.1;            % N/count
cfg.TorqueScale = 0.01;          % N*m/count

cfg.SampleRateHz = 200;            % 5 ms polling period
cfg.PlotWindowSec = 10;
end
