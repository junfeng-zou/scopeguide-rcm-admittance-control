function tests = testDobotV3CommandResponse
tests = functiontests(localfunctions);
end

function setupOnce(~)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(projectRoot, 'robot'));
end

function testMotionClientUsesOriginalImmediateResponseInterface(testCase)
client = ZJFDobotCR5('192.168.50.105', 29999, 30003, 30004);
verifyEqual(testCase, string(client.IPAddress), "192.168.50.105");
verifyTrue(testCase, ismethod(client, 'ServoJ'));
verifyTrue(testCase, ismethod(client, 'Enable'));
verifyFalse(testCase, isprop(client, 'LateResponseObservationSec'));
verifyTrue(testCase, ismethod(client, 'GetStateSnapshot'));
verifyTrue(testCase, isprop(client, 'FeedbackSequence'));
verifyTrue(testCase, isprop(client, 'ActualQuaternion'));
end

function testSuccessfulResponse(testCase)
reply = parseDobotV3CommandResponse( ...
    '0,{},MovJ(1,2,3,4,5,6);');
verifyTrue(testCase, reply.Success);
verifyEqual(testCase, reply.ErrorID, 0);
end

function testCommandDoesNotExist(testCase)
reply = parseDobotV3CommandResponse( ...
    '-10000,{},ScopeGuideProbe();');
verifyFalse(testCase, reply.Success);
verifyEqual(testCase, reply.Description, "Command does not exist.");
end

function testParameterTypeIndex(testCase)
reply = parseDobotV3CommandResponse( ...
    '-30002,{},MovJ(a,b,c,d,e,f);');
verifyEqual(testCase, reply.Description, ...
    "Parameter 2 has an incorrect type.");
end

function testParameterRangeIndex(testCase)
reply = parseDobotV3CommandResponse( ...
    '-40006,{},JointMovJ(0,0,0,0,0,999);');
verifyEqual(testCase, reply.Description, ...
    "Parameter 6 is outside its valid range.");
end

function testMalformedResponseRejected(testCase)
verifyError(testCase, @() parseDobotV3CommandResponse('garbage'), ...
    'scopeguide:dobotV3:MalformedResponse');
end

function testServoJCommandWithoutDynamicParameters(testCase)
command = formatDobotServoJCommand([1, 2, 3, 4, 5, 6]);
verifyEqual(testCase, command, ...
    'ServoJ(1.000000,2.000000,3.000000,4.000000,5.000000,6.000000)');
end

function testServoJCommandWithStage08DynamicParameters(testCase)
command = formatDobotServoJCommand( ...
    [1, 2, 3, 4, 5, 6], 0.05, 60, 400);
verifyEqual(testCase, command, ...
    ['ServoJ(1.000000,2.000000,3.000000,4.000000,5.000000,6.000000,' ...
     't=0.050000,lookahead_time=60.000000,gain=400.000000)']);
end
