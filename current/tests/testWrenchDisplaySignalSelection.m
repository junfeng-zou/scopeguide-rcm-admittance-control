function tests = testWrenchDisplaySignalSelection
tests = functiontests(localfunctions);
end

function setupOnce(~)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
end

function testExternalIsCompensatedBeforeFiltersAndDeadzone(testCase)
processed = syntheticProcessed();
[force, moment, description] = ...
    scopeguide.force.selectWrenchDisplaySignal(processed, "external");

verifyEqual(testCase, force, [1; 2; 3]);
verifyEqual(testCase, moment, [4; 5; 6]);
verifyTrue(testCase, contains(description, "未滤波、未死区"));
end

function testControlUsesDeadzoneOutputs(testCase)
processed = syntheticProcessed();
[force, moment] = ...
    scopeguide.force.selectWrenchDisplaySignal(processed, "control");

verifyEqual(testCase, force, [31; 32; 33]);
verifyEqual(testCase, moment, [34; 35; 36]);
end

function processed = syntheticProcessed()
processed = struct();
processed.ExternalWrenchToolAtSensorOrigin = (1:6).';
processed.FastWrenchToolAtSensorOrigin = (11:16).';
processed.SlowWrenchToolAtSensorOrigin = (21:26).';
processed.ControlForceTool = (31:33).';
processed.ControlMomentForDiagnostics = (34:36).';
end
