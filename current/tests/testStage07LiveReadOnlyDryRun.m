function tests = testStage07LiveReadOnlyDryRun
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
cfg = defaultRcmAdmittanceConfig();
testCase.TestData.Warmup = ...
    scopeguide.control.warmupRcmQpSolver(cfg);
end

function testUserAttestedRcmStillCannotAuthorizeDryRunMotion(testCase)
cfg = defaultRcmAdmittanceConfig();
verifyEqual(testCase, cfg.rcm.PointBaseM(:), ...
    [-0.2869; 0.6260; 0.1446], 'AbsTol', 0);
verifyEqual(testCase, cfg.rcm.PointFrame, "robot_base");
verifyEqual(testCase, cfg.rcm.PointSource, ...
    "manual_base_coordinate");
verifyTrue(testCase, cfg.tool.GeometryVerified);
verifyTrue(testCase, cfg.force.ParametersValidated);
verifyTrue(testCase, cfg.rcm.CalibrationValid);
verifyTrue(testCase, cfg.rcm.BoundsValidated);
authorization = scopeguide.safety.evaluateMotionAuthorization(cfg, "");
verifyFalse(testCase, authorization.Allowed);
verifyFalse(testCase, any(authorization.FailedGates == ...
    "ToolGeometryVerified"));
verifyFalse(testCase, any(authorization.FailedGates == ...
    "RcmCalibrationValid"));
verifyFalse(testCase, any(authorization.FailedGates == ...
    "RcmBoundsValidated"));
verifyFalse(testCase, any(authorization.FailedGates == ...
    "ForceParametersValidated"));
verifyTrue(testCase, any(authorization.FailedGates == ...
    "ServoTimingVerified"));
verifyTrue(testCase, any(authorization.FailedGates == ...
    "SafetyThresholdsValidated"));
verifyFalse(testCase, any(authorization.FailedGates == ...
    "JointLimitsValidated"));
end

function testQuadprogWarmupCoversAllRcmModes(testCase)
warmup = testCase.TestData.Warmup;
verifyEqual(testCase, warmup.RcmModes, ["hard", "soft", "hybrid"]);
verifyTrue(testCase, warmup.AllRcmModesReady);
verifyTrue(testCase, warmup.ReadyForTimingAudit);
verifyTrue(testCase, warmup.DoesNotSendHardwareCommands);
end

function testLiveRunnerContainsNoRobotMotionCallSite(testCase)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
source = string(fileread(fullfile(projectRoot, ...
    'run_stage07_live_readonly_dryrun.m')));
verifyFalse(testCase, contains(source, "robot.sendServoTarget("));
verifyFalse(testCase, contains(source, ".ServoJ("));
verifyTrue(testCase, contains(source, "robot.connectReadOnly("));
verifyTrue(testCase, contains(source, ...
    "summary.RobotCommandSentCount"));
end

function testParallelWorkerContainsNoRobotMotionCallSite(testCase)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
workerSource = string(fileread(fullfile(projectRoot, ...
    '+scopeguide', '+runtime', 'runStage07ReadOnlyWorker.m')));
runnerSource = string(fileread(fullfile(projectRoot, ...
    'run_stage07_live_readonly_parallel.m')));

verifyFalse(testCase, contains(workerSource, ...
    "robot.sendServoTarget("));
verifyFalse(testCase, contains(workerSource, ".ServoJ("));
verifyFalse(testCase, contains(runnerSource, ...
    "robot.sendServoTarget("));
verifyFalse(testCase, contains(runnerSource, ".ServoJ("));
verifyTrue(testCase, contains(workerSource, ...
    "robot.connectReadOnly()"));
verifyTrue(testCase, contains(runnerSource, "backgroundPool"));
verifyTrue(testCase, contains(runnerSource, ...
    "parallel.pool.PollableDataQueue"));
end

function testBackgroundPoolBidirectionalQueues(testCase)
pool = backgroundPool;
clientToWorker = parallel.pool.PollableDataQueue( ...
    Destination="any");
send(clientToWorker, struct('Value', 42));
receiveFuture = parfeval(pool, @poll, 2, clientToWorker, 2);
[received, ok] = fetchOutputs(receiveFuture);
verifyTrue(testCase, ok);
verifyEqual(testCase, received.Value, 42);

workerToClient = parallel.pool.PollableDataQueue( ...
    Destination="any");
sendFuture = parfeval(pool, @send, 0, workerToClient, ...
    struct('Value', 84));
wait(sendFuture);
[received, ok] = poll(workerToClient, 2);
verifyTrue(testCase, ok);
verifyEqual(testCase, received.Value, 84);
close(clientToWorker);
close(workerToClient);
end

function testDisplaySnapshotIsValueOnlyAndCannotAuthorizeMotion(testCase)
[cfg, ~, robotState, processed] = stage07Fixture();
processed.BaselineDiagnostics = struct('Ready', true);
controller = scopeguide.control.Stage07ReadOnlyController(cfg);
adapter = scopeguide.io.EnableHandleAdapter(cfg);
adapter.submit(false, 0, Source="unit_test", ...
    SupportsPhysicalMotion=false);
decision = controller.step(processed, robotState, ...
    adapter.read(0), 0);
snapshot = scopeguide.runtime.buildStage07DisplaySnapshot( ...
    0, processed, decision);

verifyEqual(testCase, snapshot.Type, "telemetry");
verifyTrue(testCase, snapshot.BaselineReady);
verifySize(testCase, snapshot.ControlForceTool, [3, 1]);
verifyFalse(testCase, snapshot.HardwareCommandAuthorized);
verifyFalse(testCase, snapshot.MotionCommandSent);
verifyEqual(testCase, snapshot.InjectionStatusCode, "NONE");
verifyFalse(testCase, any(structfun(@(value) isa(value, 'handle'), ...
    snapshot)));
end

function testAcceptanceFaultScheduleDrivesInjectionAndGuidance(testCase)
schedule = scopeguide.runtime.stage07FaultSchedule();
verifyEqual(testCase, schedule.TimeSec, [15; 30; 45; 60; 75], ...
    'AbsTol', 0);
verifyEqual(testCase, schedule.DurationSec, 0.12 * ones(5, 1), ...
    'AbsTol', 0);
verifyEqual(testCase, schedule.StatusCode, [ ...
    "INJECT_HANDLE_STALE"; "INJECT_FORCE_STALE"; ...
    "INJECT_ROBOT_STALE"; "INJECT_QP_FAILURE"; ...
    "INJECT_CONTROL_OVERRUN"]);

fields = cellstr(schedule.FieldName);
for index = 1:numel(schedule.TimeSec)
    before = scopeguide.runtime.stage07FaultInjectionAtTime( ...
        schedule.TimeSec(index) - eps(schedule.TimeSec(index)), ...
        "acceptance");
    active = scopeguide.runtime.stage07FaultInjectionAtTime( ...
        schedule.TimeSec(index) + 0.01, "acceptance");
    after = scopeguide.runtime.stage07FaultInjectionAtTime( ...
        schedule.TimeSec(index) + schedule.DurationSec(index), ...
        "acceptance");
    verifyEqual(testCase, before.StatusCode, "NONE");
    verifyTrue(testCase, active.(fields{index}));
    verifyEqual(testCase, active.StatusCode, ...
        schedule.StatusCode(index));
    verifyEqual(testCase, after.StatusCode, "NONE");
end

wait = scopeguide.runtime.stage07FaultGuidance(0, "acceptance");
prepare = scopeguide.runtime.stage07FaultGuidance(10, "acceptance");
active = scopeguide.runtime.stage07FaultGuidance(15.01, "acceptance");
recover = scopeguide.runtime.stage07FaultGuidance(15.2, "acceptance");
nextPrepare = scopeguide.runtime.stage07FaultGuidance(25, "acceptance");
finalRecovery = scopeguide.runtime.stage07FaultGuidance(76, "acceptance");
disabled = scopeguide.runtime.stage07FaultGuidance(15, "none");

verifyEqual(testCase, wait.Phase, "WAIT");
verifyEqual(testCase, prepare.Phase, "PREPARE");
verifyEqual(testCase, prepare.EventIndex, 1);
verifyEqual(testCase, prepare.CountdownSec, 5, 'AbsTol', 0);
verifyTrue(testCase, contains(prepare.MessageZh, "Space"));
verifyEqual(testCase, active.Phase, "ACTIVE");
verifyEqual(testCase, active.EventCode, "INJECT_HANDLE_STALE");
verifyEqual(testCase, recover.Phase, "RECOVER");
verifyTrue(testCase, contains(recover.MessageZh, "Esc"));
verifyTrue(testCase, contains(recover.MessageZh, "R"));
verifyEqual(testCase, nextPrepare.Phase, "PREPARE");
verifyEqual(testCase, nextPrepare.EventIndex, 2);
verifyEqual(testCase, finalRecovery.Phase, "RECOVER");
verifyTrue(testCase, contains(finalRecovery.MessageZh, "R"));
verifyFalse(testCase, disabled.Enabled);
end

function testAcceptanceDashboardAcceptsGuidedSnapshot(testCase)
originalVisibility = get(groot, 'defaultFigureVisible');
visibilityCleanup = onCleanup(@() set( ...
    groot, 'defaultFigureVisible', originalVisibility));
set(groot, 'defaultFigureVisible', 'off');
[cfg, ~, robotState, processed] = stage07Fixture();
processed.BaselineDiagnostics = struct('Ready', true);
controller = scopeguide.control.Stage07ReadOnlyController(cfg);
adapter = scopeguide.io.EnableHandleAdapter(cfg);
adapter.submit(true, 10, Source="unit_test", ...
    SupportsPhysicalMotion=false);
decision = controller.step(processed, robotState, ...
    adapter.read(10), 10);
snapshot = scopeguide.runtime.buildStage07DisplaySnapshot( ...
    10, processed, decision);
dashboard = scopeguide.ui.Stage07DryRunDashboard( ...
    cfg, 15, EnablePlot=false, ...
    FaultInjectionProfile="acceptance");
dashboardCleanup = onCleanup(@() dashboard.close());

dashboard.updateFromSnapshot(snapshot);
verifyTrue(testCase, dashboard.isOpen());
verifyTrue(testCase, isempty(findall(dashboard.Figure, ...
    'Type', 'animatedline')));

clear dashboardCleanup visibilityCleanup;
end

function testDashboardAcceptsWorkerSnapshot(testCase)
originalVisibility = get(groot, 'defaultFigureVisible');
visibilityCleanup = onCleanup(@() set( ...
    groot, 'defaultFigureVisible', originalVisibility));
set(groot, 'defaultFigureVisible', 'off');
[cfg, ~, robotState, processed] = stage07Fixture();
processed.BaselineDiagnostics = struct('Ready', true);
controller = scopeguide.control.Stage07ReadOnlyController(cfg);
adapter = scopeguide.io.EnableHandleAdapter(cfg);
adapter.submit(false, 0, Source="unit_test", ...
    SupportsPhysicalMotion=false);
decision = controller.step(processed, robotState, ...
    adapter.read(0), 0);
snapshot = scopeguide.runtime.buildStage07DisplaySnapshot( ...
    0, processed, decision);
dashboard = scopeguide.ui.Stage07DryRunDashboard( ...
    cfg, 15, EnablePlot=true);
dashboardCleanup = onCleanup(@() dashboard.close());

dashboard.updateFromSnapshot(snapshot);
verifyTrue(testCase, dashboard.isOpen());
verifyEmpty(testCase, get(dashboard.Figure, 'WindowKeyReleaseFcn'));

keyPress = get(dashboard.Figure, 'WindowKeyPressFcn');
keyPress(dashboard.Figure, struct('Key', 'space'));
input = dashboard.readInput();
keyboard = dashboard.keyboardDiagnostics();
verifyTrue(testCase, input.Requested);
verifyEqual(testCase, input.Source, "keyboard_space_latched");
verifyEqual(testCase, keyboard.Mode, "space_on_escape_off");
verifyTrue(testCase, keyboard.KeyReleaseIgnored);

% Repeated SPACE events are idempotent and cannot toggle the request off.
keyPress(dashboard.Figure, struct('Key', 'space'));
repeated = dashboard.readInput();
verifyTrue(testCase, repeated.Requested);

keyPress(dashboard.Figure, struct('Key', 'escape'));
input = dashboard.readInput();
verifyFalse(testCase, input.Requested);
keyboard = dashboard.keyboardDiagnostics();
verifyFalse(testCase, keyboard.FinalLatched);
verifyGreaterThanOrEqual(testCase, keyboard.SpaceOnCommandCount, 2);
verifyEqual(testCase, keyboard.EscapeOffCommandCount, 1);
verifyTrue(testCase, keyboard.SoftwareDryRunOnly);

clear dashboardCleanup visibilityCleanup;
end

function testInputOnlyDashboardCreatesNoAnimatedCurves(testCase)
originalVisibility = get(groot, 'defaultFigureVisible');
visibilityCleanup = onCleanup(@() set( ...
    groot, 'defaultFigureVisible', originalVisibility));
set(groot, 'defaultFigureVisible', 'off');
cfg = defaultRcmAdmittanceConfig();
dashboard = scopeguide.ui.Stage07DryRunDashboard( ...
    cfg, 15, EnablePlot=false);
dashboardCleanup = onCleanup(@() dashboard.close());

verifyFalse(testCase, dashboard.PlotEnabled);
verifyTrue(testCase, dashboard.isOpen());
verifyEmpty(testCase, findall( ...
    dashboard.Figure, 'Type', 'animatedline'));

clear dashboardCleanup;
verifyFalse(testCase, dashboard.isOpen());
clear visibilityCleanup;
end

function testCompleteReadOnlyChainPredictsAndReleaseZeros(testCase)
[cfg, q, robotState, processed] = stage07Fixture();
controller = scopeguide.control.Stage07ReadOnlyController(cfg);
adapter = scopeguide.io.EnableHandleAdapter(cfg);
[moving, nowSec] = driveToMoving( ...
    controller, adapter, processed, robotState);

verifyEqual(testCase, moving.EnableStatus.State, "ENABLED");
verifyTrue(testCase, moving.PredictionValid);
verifyGreaterThan(testCase, norm(moving.QdotPredictedRadSec), 0);
verifyTrue(testCase, moving.Qp.MotionCommandValid);
verifyFalse(testCase, moving.HardwareCommandAuthorized);
verifyFalse(testCase, moving.MotionCommandSent);
verifyTrue(testCase, moving.DoesNotSendHardwareCommands);

nowSec = nowSec + cfg.runtime.NominalDtSec;
adapter.submit(false, nowSec, Source="unit_test", ...
    SupportsPhysicalMotion=false);
released = controller.step(processed, robotState, ...
    adapter.read(nowSec), nowSec);
verifyEqual(testCase, released.EnableStatus.State, "STOPPING");
verifyFalse(testCase, released.PredictionValid);
verifyEqual(testCase, released.QdotPredictedRadSec, zeros(6, 1), ...
    'AbsTol', 0);
verifyEqual(testCase, released.QTargetPredictedRad, q, 'AbsTol', 0);
verifyFalse(testCase, released.HardwareCommandAuthorized);
verifyFalse(testCase, released.MotionCommandSent);
end

function testEveryInjectedRuntimeFaultLatchesAndOutputsExactZero(testCase)
cases = { ...
    'HandleStale', "HANDLE_STALE_INPUT"; ...
    'ForceStale', "FORCE_STALE"; ...
    'RobotStale', "ROBOT_FEEDBACK_STALE"; ...
    'QpFailure', "QP_FAILURE"; ...
    'ControlOverrun', "CONTROL_PERIOD_OVERRUN"};
for index = 1:size(cases, 1)
    [cfg, q, robotState, processed] = stage07Fixture();
    controller = scopeguide.control.Stage07ReadOnlyController(cfg);
    adapter = scopeguide.io.EnableHandleAdapter(cfg);
    [~, nowSec] = driveToMoving( ...
        controller, adapter, processed, robotState);
    nowSec = nowSec + cfg.runtime.NominalDtSec;
    adapter.submit(true, nowSec, Source="unit_test", ...
        SupportsPhysicalMotion=false);
    injection = scopeguide.types.stage07FaultInjection();
    injection.(cases{index, 1}) = true;
    injection.StatusCode = "INJECT_" + upper(string(cases{index, 1}));
    fault = controller.step(processed, robotState, ...
        adapter.read(nowSec), nowSec, injection);

    verifyEqual(testCase, fault.EnableStatus.State, "FAULT", ...
        cases{index, 1});
    verifyEqual(testCase, fault.EnableStatus.FaultCode, ...
        cases{index, 2}, cases{index, 1});
    verifyEqual(testCase, fault.QdotPredictedRadSec, zeros(6, 1), ...
        'AbsTol', 0);
    verifyEqual(testCase, fault.QTargetPredictedRad, q, 'AbsTol', 0);
    verifyFalse(testCase, fault.PredictionValid);
    verifyFalse(testCase, fault.HardwareCommandAuthorized);
    verifyFalse(testCase, fault.MotionCommandSent);
end
end

function testNaturalQpFailuresStayZeroAndEscalate(testCase)
[cfg, q, robotState, processed] = stage07Fixture();
kinematics = scopeguide.geometry.cr5ForwardKinematics(q, cfg);
basis = scopeguide.geometry.computeRcmMotionBasis( ...
    kinematics.TBaseEndoscope, cfg.rcm.PointBaseM, cfg);
cfg.rcm.PointBaseM = cfg.rcm.PointBaseM + ...
    5e-3 * basis.PivotAxis1Base;
controller = scopeguide.control.Stage07ReadOnlyController(cfg);
adapter = scopeguide.io.EnableHandleAdapter(cfg);

nowSec = 0;
adapter.submit(false, nowSec, Source="unit_test", ...
    SupportsPhysicalMotion=false);
output = controller.step(processed, robotState, ...
    adapter.read(nowSec), nowSec);
% Debounce is specified in seconds, so the number of samples required to
% reach ENABLED changes with ControlRateHz.  Stop at the first enabled QP
% failure instead of assuming that six samples always reach that point.
for index = 1:20
    nowSec = index * cfg.runtime.NominalDtSec;
    adapter.submit(true, nowSec, Source="unit_test", ...
        SupportsPhysicalMotion=false);
    output = controller.step(processed, robotState, ...
        adapter.read(nowSec), nowSec);
    if output.EnableStatus.State == "ENABLED" && ...
            output.Qp.ConsecutiveFailureCount > 0
        break;
    end
end
verifyEqual(testCase, output.EnableStatus.State, "ENABLED");

while ~output.Qp.RequiresFault
    nowSec = nowSec + cfg.runtime.NominalDtSec;
    adapter.submit(true, nowSec, Source="unit_test", ...
        SupportsPhysicalMotion=false);
    output = controller.step(processed, robotState, ...
        adapter.read(nowSec), nowSec);
    verifyFalse(testCase, output.Qp.MotionCommandValid);
    verifyEqual(testCase, output.QdotPredictedRadSec, zeros(6, 1), ...
        'AbsTol', 0);
    verifyEqual(testCase, output.QTargetPredictedRad, q, 'AbsTol', 0);
end
verifyTrue(testCase, output.Qp.RequiresFault);

nowSec = nowSec + cfg.runtime.NominalDtSec;
adapter.submit(true, nowSec, Source="unit_test", ...
    SupportsPhysicalMotion=false);
latched = controller.step(processed, robotState, ...
    adapter.read(nowSec), nowSec);
verifyEqual(testCase, latched.EnableStatus.State, "FAULT");
verifyEqual(testCase, latched.EnableStatus.FaultCode, "QP_FAILURE");
verifyEqual(testCase, latched.QdotPredictedRadSec, zeros(6, 1), ...
    'AbsTol', 0);
end

function testInvalidRobotGeometryCannotCreatePrediction(testCase)
[cfg, q, robotState, processed] = stage07Fixture();
controller = scopeguide.control.Stage07ReadOnlyController(cfg);
adapter = scopeguide.io.EnableHandleAdapter(cfg);
[~, nowSec] = driveToMoving( ...
    controller, adapter, processed, robotState);
robotState.IsValid = false;
nowSec = nowSec + cfg.runtime.NominalDtSec;
adapter.submit(true, nowSec, Source="unit_test", ...
    SupportsPhysicalMotion=false);
invalid = controller.step(processed, robotState, ...
    adapter.read(nowSec), nowSec);

verifyEqual(testCase, invalid.EnableStatus.State, "FAULT");
verifyEqual(testCase, invalid.EnableStatus.FaultCode, ...
    "ROBOT_FEEDBACK_STALE");
verifyFalse(testCase, invalid.GeometryValid);
verifyEqual(testCase, invalid.GeometryStatusCode, ...
    "ROBOT_STATE_INVALID");
verifyEqual(testCase, invalid.QdotPredictedRadSec, zeros(6, 1), ...
    'AbsTol', 0);
verifyEqual(testCase, invalid.QTargetPredictedRad, q, ...
    'AbsTol', 0);
verifyFalse(testCase, invalid.HardwareCommandAuthorized);
verifyFalse(testCase, invalid.MotionCommandSent);
verifyNotEqual(testCase, q, zeros(6, 1));
end

function [moving, nowSec] = driveToMoving( ...
        controller, adapter, processed, robotState)
cfg = controller.Config;
for index = 0:8
    nowSec = index * cfg.runtime.NominalDtSec;
    adapter.submit(index > 0, nowSec, Source="unit_test", ...
        SupportsPhysicalMotion=false);
    moving = controller.step(processed, robotState, ...
        adapter.read(nowSec), nowSec);
end
end

function [cfg, q, robotState, processed] = stage07Fixture()
cfg = defaultRcmAdmittanceConfig();
cfg.runtime.Mode = "live_dry_run";
cfg.robot.EnableMotion = false;
cfg.robot.DryRun = true;
q = [0.2; -0.5; 0.8; -0.4; 0.3; -0.2];
kinematics = scopeguide.geometry.cr5ForwardKinematics(q, cfg);
cfg.rcm.PointBaseM = kinematics.EndoscopeTipPositionBaseM - ...
    0.15 * kinematics.ShaftAxisBase;
validateRcmAdmittanceConfig(cfg);

robotState = scopeguide.types.robotState();
robotState.JointPositionRad = q;
robotState.JointVelocityRadSec = zeros(6, 1);
robotState.SampleAgeSec = 0;
robotState.IsValid = true;
robotState.StatusCode = "OK";

processed = struct();
processed.Quality = struct('Valid', true);
processed.MotionInputValid = true;
processed.Safety = struct('StopRequested', false);
processed.NeutralCheckPassed = true;
processed.ControlForceTool = [0; -5; 0];
processed.ControlWrenchToolAtSensorOrigin = [0; -5; 0; 0; 0; 0];
end
