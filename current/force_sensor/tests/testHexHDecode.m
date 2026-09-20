function tests = testHexHDecode
tests = functiontests(localfunctions);
end

function testSignedScaling(testCase)
% Counts: [-123, 456, -1, 123, -234, 0]
registers = uint16([65413, 456, 65535, 123, 65302, 0]);
[force, torque, wrench, counts] = ...
    onrobot.HexHClient.decodeWrenchRegisters(registers, 0.1, 0.01);

verifyEqual(testCase, counts, [-123; 456; -1; 123; -234; 0]);
verifyEqual(testCase, force, [-12.3; 45.6; -0.1], 'AbsTol', 1e-12);
verifyEqual(testCase, torque, [1.23; -2.34; 0], 'AbsTol', 1e-12);
verifyEqual(testCase, wrench, [force; torque]);
end

function testInvalidRegisterCount(testCase)
verifyError(testCase, ...
    @() onrobot.HexHClient.decodeWrenchRegisters(uint16([1 2 3])), ...
    'onrobot:HexHClient:InvalidWrenchLength');
end

function testDualBranchFilter(testCase)
rng(7);
filterBank = onrobot.WrenchFilterBank(200, 20, 5);
constant = [1; 2; 3; 0.1; 0.2; 0.3];
[fast, slow] = filterBank.step(constant);
verifyEqual(testCase, fast, constant, 'AbsTol', 1e-12);
verifyEqual(testCase, slow, constant, 'AbsTol', 1e-12);

n = 5000;
raw = repmat(constant, 1, n) + 0.1 * randn(6, n);
fastHistory = zeros(6, n);
slowHistory = zeros(6, n);
filterBank.reset(raw(:, 1));
for i = 1:n
    [fastHistory(:, i), slowHistory(:, i)] = filterBank.step(raw(:, i));
end
valid = 401:n;
rawStd = std(raw(:, valid), 0, 2);
fastStd = std(fastHistory(:, valid), 0, 2);
slowStd = std(slowHistory(:, valid), 0, 2);
verifyTrue(testCase, all(fastStd < rawStd));
verifyTrue(testCase, all(slowStd < fastStd));
end
