function sample = read_hex_h_once()
%READ_HEX_H_ONCE Minimal example for integration into a safety loop.

exampleDir = fileparts(mfilename('fullpath'));
addpath(fileparts(exampleDir));

sensor = onrobot.HexHClient(onrobot.defaultConfig());
cleanup = onCleanup(@() sensor.disconnect());
sensor.connect();
sample = sensor.readSample();

disp('Wrench [Fx Fy Fz Tx Ty Tz] in [N, N*m]:');
disp(sample.wrench.');
end
