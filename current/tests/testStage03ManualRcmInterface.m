function tests = testStage03ManualRcmInterface
tests = functiontests(localfunctions);
end

function setupOnce(~)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
end

function testMillimetreInputIsConvertedAndRemainsUnvalidated(testCase)
cfg = defaultRcmAdmittanceConfig();
[updated, record] = applyManualRcmPoint(cfg, ...
    [420; -135; 510], InputUnit="mm", ...
    SourceDescription="external spatial measurement", ...
    FixtureIdentifier="fixture_A");

verifyEqual(testCase, updated.rcm.PointBaseM, ...
    [0.420; -0.135; 0.510], 'AbsTol', 1e-12);
verifyEqual(testCase, updated.rcm.PointFrame, "robot_base");
verifyEqual(testCase, updated.rcm.PointSource, ...
    "manual_base_coordinate");
verifyFalse(testCase, updated.rcm.CalibrationValid);
verifyFalse(testCase, updated.rcm.BoundsValidated);
verifyFalse(testCase, record.PhysicalMotionAuthorizedByThisOperation);
verifyWarningFree(testCase, @() validateRcmAdmittanceConfig(updated));
end

function testManualPointDoesNotOpenMotionGate(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg = applyManualRcmPoint(cfg, [0.4; 0; 0.3]);
authorization = scopeguide.safety.evaluateMotionAuthorization(cfg, "");

verifyFalse(testCase, authorization.Allowed);
verifyTrue(testCase, any(authorization.FailedGates == ...
    "RcmCalibrationValid"));
verifyTrue(testCase, any(authorization.FailedGates == ...
    "RcmBoundsValidated"));
end

function testInvalidUnitAndCoordinateAreRejected(testCase)
cfg = defaultRcmAdmittanceConfig();
verifyError(testCase, @() applyManualRcmPoint( ...
    cfg, [1; 2; 3], InputUnit="cm"), ...
    'scopeguide:config:InvalidManualRcmUnit');
verifyError(testCase, @() applyManualRcmPoint( ...
    cfg, [1; NaN; 3]), ...
    'scopeguide:config:InvalidManualRcmPoint');
end
