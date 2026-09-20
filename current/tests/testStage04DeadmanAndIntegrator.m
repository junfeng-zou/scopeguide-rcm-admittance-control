function tests = testStage04DeadmanAndIntegrator
tests = functiontests(localfunctions);
end

function setupOnce(~)
projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'config'));
end

function testSoftwareAdapterIsTimestampedAndFailSafe(testCase)
cfg = defaultRcmAdmittanceConfig();
adapter = scopeguide.io.EnableHandleAdapter(cfg);

uninitialized = adapter.read(0);
verifyFalse(testCase, uninitialized.IsValid);
verifyFalse(testCase, uninitialized.Enabled);

adapter.submit(true, 0.010, Source="keyboard_space_hold");
fresh = adapter.read(0.015);
verifyTrue(testCase, fresh.IsValid);
verifyTrue(testCase, fresh.Enabled);
verifyEqual(testCase, fresh.Source, "keyboard_space_hold");
verifyFalse(testCase, fresh.SupportsPhysicalMotion);

stale = adapter.read(0.010 + cfg.handle.StaleSec + 0.001);
verifyFalse(testCase, stale.IsValid);
verifyFalse(testCase, stale.Enabled);
verifyEqual(testCase, stale.StatusCode, "STALE_INPUT");

adapter.submit(true, 0.005, Source="regressed_clock");
regressed = adapter.read(0.020);
verifyFalse(testCase, regressed.IsValid);
verifyFalse(testCase, regressed.Enabled);
verifyEqual(testCase, regressed.StatusCode, ...
    "TIMESTAMP_REGRESSION");
end

function testKeyboardAutoRepeatReleaseIsSuppressed(testCase)
filter = scopeguide.io.KeyboardHoldFilter(0.050, 0.200);
filter.press(0);
verifyTrue(testCase, filter.Held);

% Linux/X11 auto-repeat may deliver the repeated press roughly one 30 Hz
% control period after its synthetic release.  The filtered request must
% remain asserted across that interval.
filter.release(0.500);
verifyTrue(testCase, filter.Held);
verifyTrue(testCase, filter.ReleasePending);
filter.tick(0.533);
verifyTrue(testCase, filter.Held);
filter.press(0.540);
verifyTrue(testCase, filter.Held);
verifyFalse(testCase, filter.ReleasePending);
verifyEqual(testCase, filter.SuppressedRepeatReleaseCount, uint64(1));

% A real release has no following press and is confirmed after 50 ms.
filter.press(0.900);
filter.release(1.000);
filter.tick(1.049);
verifyTrue(testCase, filter.Held);
filter.tick(1.050);
verifyFalse(testCase, filter.Held);
verifyFalse(testCase, filter.ReleasePending);
verifyEqual(testCase, filter.ConfirmedReleaseCount, uint64(1));

% Explicit escape/close paths never wait for the confirmation window.
filter.press(2.000);
filter.forceRelease("ESCAPE_RELEASE");
verifyFalse(testCase, filter.Held);
end

function testKeyboardWatchdogRecoversMissingFinalRelease(testCase)
filter = scopeguide.io.KeyboardHoldFilter(0.050, 0.200);
filter.press(0);

% Repeated KeyPress events establish that Linux key repeat is active.
filter.press(0.500);
verifyTrue(testCase, filter.RepeatObserved);

% Simulate a synthetic release/press pair followed by a missing final
% KeyRelease.  The stale repeated press must not hold the GUI forever.
filter.release(0.530);
filter.press(0.570);
filter.tick(0.769);
verifyTrue(testCase, filter.Held);
filter.tick(0.770);
verifyFalse(testCase, filter.Held);
verifyEqual(testCase, filter.StatusCode, ...
    "KEY_REPEAT_WATCHDOG_RELEASE");
verifyEqual(testCase, filter.WatchdogReleaseCount, uint64(1));
end

function testKeyboardWatchdogDoesNotReleaseActiveRepeat(testCase)
filter = scopeguide.io.KeyboardHoldFilter(0.050, 0.200);
filter.press(0);
filter.press(0.500);
filter.tick(0.650);
verifyTrue(testCase, filter.Held);
filter.press(0.680);
filter.tick(0.850);
verifyTrue(testCase, filter.Held);
verifyEqual(testCase, filter.WatchdogReleaseCount, uint64(0));
end

function testPressPrearmSoftStartAndReleaseReanchors(testCase)
cfg = defaultRcmAdmittanceConfig();
adapter = scopeguide.io.EnableHandleAdapter(cfg);
fsm = scopeguide.control.HandleEnableStateMachine(cfg);
integrator = scopeguide.control.JointCommandIntegrator(cfg);
safety = healthySafety();
qState = zeros(6, 1);
qdotRequest = 0.02 * ones(6, 1);

[idle, anchor] = cycle(adapter, fsm, integrator, ...
    false, 0, safety, qState, qdotRequest);
verifyEqual(testCase, idle.State, "DISABLED");
verifyTrue(testCase, anchor.Reanchored);

[prearm, ~] = cycle(adapter, fsm, integrator, ...
    true, 0.010, safety, qState, qdotRequest);
verifyEqual(testCase, prearm.State, "PREARM");
verifyFalse(testCase, prearm.MotionPermitted);

[enabled, enabledAnchor] = cycle(adapter, fsm, integrator, ...
    true, 0.051, safety, qState, qdotRequest);
verifyEqual(testCase, enabled.State, "ENABLED");
verifyEqual(testCase, enabled.CommandScale, 0, 'AbsTol', 1e-12);
verifyTrue(testCase, enabledAnchor.Reanchored);

for nowSec = 0.061:0.010:0.141
    [~, moving] = cycle(adapter, fsm, integrator, ...
        true, nowSec, safety, qState, qdotRequest);
    qState = moving.QTargetRad;
end
[ramping, moving] = cycle(adapter, fsm, integrator, ...
    true, 0.151, safety, qState, qdotRequest);
verifyTrue(testCase, ramping.MotionPermitted);
verifyEqual(testCase, ramping.CommandScale, 0.5, 'AbsTol', 1e-12);
verifyFalse(testCase, moving.Reanchored);
verifyGreaterThan(testCase, norm(moving.QdotAppliedRadSec), 0);

measuredAtRelease = [0.1; -0.2; 0.3; -0.1; 0.05; 0.2];
[stopping, stopped] = cycle(adapter, fsm, integrator, ...
    false, 0.161, safety, measuredAtRelease, qdotRequest);
verifyEqual(testCase, stopping.State, "STOPPING");
verifyFalse(testCase, stopping.MotionPermitted);
verifyEqual(testCase, stopped.QdotAppliedRadSec, zeros(6, 1));
verifyEqual(testCase, stopped.QTargetRad, measuredAtRelease, ...
    'AbsTol', 0);
verifyTrue(testCase, stopped.Reanchored);

[disabled, ~] = cycle(adapter, fsm, integrator, ...
    false, 0.171, safety, measuredAtRelease, qdotRequest);
verifyEqual(testCase, disabled.State, "DISABLED");
end

function testPreloadedPressCannotBypassPrearm(testCase)
cfg = defaultRcmAdmittanceConfig();
adapter = scopeguide.io.EnableHandleAdapter(cfg);
fsm = scopeguide.control.HandleEnableStateMachine(cfg);
safety = healthySafety();
safety.NeutralWrench = false;

adapter.submit(true, 0.010, Source="test");
status = fsm.step(adapter.read(0.010), safety, 0.010);
verifyEqual(testCase, status.State, "PREARM");
for nowSec = [0.060, 0.120, 0.250]
    adapter.submit(true, nowSec, Source="test");
    status = fsm.step(adapter.read(nowSec), safety, nowSec);
    verifyEqual(testCase, status.State, "PREARM");
    verifyFalse(testCase, status.MotionPermitted);
    verifyEqual(testCase, status.StopReason, ...
        "PRELOAD_NOT_NEUTRAL");
end
end

function testFrozenHandleTimestampFaultsAndRequiresReset(testCase)
cfg = defaultRcmAdmittanceConfig();
adapter = scopeguide.io.EnableHandleAdapter(cfg);
fsm = scopeguide.control.HandleEnableStateMachine(cfg);
safety = healthySafety();
driveToEnabled(adapter, fsm, safety);

staleTime = 0.051 + cfg.handle.StaleSec + 0.001;
staleInput = adapter.read(staleTime);
status = fsm.step(staleInput, safety, staleTime);
verifyEqual(testCase, status.State, "FAULT");
verifyEqual(testCase, status.FaultCode, "HANDLE_STALE_INPUT");

adapter.submit(false, 0.080, Source="test");
withoutReset = fsm.step(adapter.read(0.080), safety, 0.080);
verifyEqual(testCase, withoutReset.State, "FAULT");

fsm.requestReset();
adapter.submit(false, 0.090, Source="test");
afterReset = fsm.step(adapter.read(0.090), safety, 0.090);
verifyEqual(testCase, afterReset.State, "DISABLED");
verifyFalse(testCase, afterReset.FaultLatched);
end

function testRuntimeFaultPathsLatchAndZeroPermission(testCase)
cases = { ...
    'ForceFresh', "FORCE_STALE"; ...
    'RobotFresh', "ROBOT_FEEDBACK_STALE"; ...
    'QpHealthy', "QP_FAILURE"; ...
    'ControlPeriodHealthy', "CONTROL_PERIOD_OVERRUN"; ...
    'MotionGatesSatisfied', "MOTION_GATE_BLOCKED"};
for index = 1:size(cases, 1)
    cfg = defaultRcmAdmittanceConfig();
    adapter = scopeguide.io.EnableHandleAdapter(cfg);
    fsm = scopeguide.control.HandleEnableStateMachine(cfg);
    safety = healthySafety();
    driveToEnabled(adapter, fsm, safety);
    safety.(cases{index, 1}) = false;
    adapter.submit(true, 0.061, Source="test");
    fault = fsm.step(adapter.read(0.061), safety, 0.061);
    verifyEqual(testCase, fault.State, "FAULT");
    verifyEqual(testCase, fault.FaultCode, cases{index, 2});
    verifyFalse(testCase, fault.MotionPermitted);
    verifyTrue(testCase, fault.ResetDynamicState);
end
end

function testRapidPressReleaseNeverAccumulatesOldTarget(testCase)
cfg = defaultRcmAdmittanceConfig();
adapter = scopeguide.io.EnableHandleAdapter(cfg);
fsm = scopeguide.control.HandleEnableStateMachine(cfg);
integrator = scopeguide.control.JointCommandIntegrator(cfg);
safety = healthySafety();
qState = zeros(6, 1);
qdot = ones(6, 1);

cycle(adapter, fsm, integrator, false, 0, safety, qState, qdot);
times = 0.01:0.01:0.20;
for index = 1:numel(times)
    pressed = mod(index, 2) == 1;
    [status, integrated] = cycle(adapter, fsm, integrator, ...
        pressed, times(index), safety, qState, qdot);
    verifyFalse(testCase, status.MotionPermitted);
    verifyEqual(testCase, integrated.QdotAppliedRadSec, zeros(6, 1));
    verifyEqual(testCase, integrated.QTargetRad, qState, 'AbsTol', 0);
end
end

function testIntegratorLimitsAndLongPeriodFailClosed(testCase)
cfg = defaultRcmAdmittanceConfig();
cfg.control.MaximumJointCommandRadSec = ones(6, 1);
cfg.control.MaximumJointAccelerationRadSec2 = 10 * ones(6, 1);
cfg.control.MaximumTargetStateErrorRad = 0.01 * ones(6, 1);
validateRcmAdmittanceConfig(cfg);
integrator = scopeguide.control.JointCommandIntegrator(cfg);
enabled = enabledStatus();
qState = zeros(6, 1);

initial = integrator.step(qState, 100 * ones(6, 1), enabled, 0);
verifyTrue(testCase, initial.Reanchored);
limited = integrator.step(qState, 100 * ones(6, 1), enabled, 0.010);
verifyTrue(testCase, limited.VelocityClamped);
verifyTrue(testCase, limited.AccelerationClamped);
verifyLessThanOrEqual(testCase, ...
    max(abs(limited.QTargetRad - qState)), 0.01 + eps);

lateState = 0.2 * ones(6, 1);
late = integrator.step(lateState, ones(6, 1), enabled, 0.100);
verifyTrue(testCase, late.RequiresFault);
verifyTrue(testCase, late.Reanchored);
verifyEqual(testCase, late.QdotAppliedRadSec, zeros(6, 1));
verifyEqual(testCase, late.QTargetRad, lateState, 'AbsTol', 0);
end

function testControlClockRegressionFaults(testCase)
cfg = defaultRcmAdmittanceConfig();
adapter = scopeguide.io.EnableHandleAdapter(cfg);
fsm = scopeguide.control.HandleEnableStateMachine(cfg);
safety = healthySafety();
adapter.submit(false, 0.010, Source="test");
fsm.step(adapter.read(0.010), safety, 0.010);
fault = fsm.step(adapter.read(0.010), safety, 0.010);
verifyEqual(testCase, fault.State, "FAULT");
verifyEqual(testCase, fault.FaultCode, "CONTROL_CLOCK_REGRESSION");
end

function testRandomizedSequencesCannotBypassSafety(testCase)
cfg = defaultRcmAdmittanceConfig();
adapter = scopeguide.io.EnableHandleAdapter(cfg);
fsm = scopeguide.control.HandleEnableStateMachine(cfg);
integrator = scopeguide.control.JointCommandIntegrator(cfg);
qState = zeros(6, 1);
qdotRequest = 0.5 * ones(6, 1);
pressed = false;
rng(404);

for index = 1:2000
    nowSec = index * cfg.runtime.NominalDtSec;
    if rand < 0.04
        pressed = ~pressed;
    end
    safety = healthySafety();
    if rand < 0.03
        faultField = randi(5);
        fields = {'ForceFresh', 'RobotFresh', 'QpHealthy', ...
            'ControlPeriodHealthy', 'MotionGatesSatisfied'};
        safety.(fields{faultField}) = false;
    end
    if rand < 0.08
        safety.NeutralWrench = false;
    end
    submitThisCycle = rand >= 0.04;
    if submitThisCycle
        adapter.submit(pressed, nowSec, Source="randomized_test", ...
            SupportsPhysicalMotion=false);
    end
    input = adapter.read(nowSec);
    if fsm.State == "FAULT" && rand < 0.10
        pressed = false;
        safety = healthySafety();
        adapter.submit(false, nowSec, Source="randomized_reset", ...
            SupportsPhysicalMotion=false);
        input = adapter.read(nowSec);
        fsm.requestReset();
    end
    status = fsm.step(input, safety, nowSec);
    integrated = integrator.step(qState, qdotRequest, status, nowSec);

    if status.MotionPermitted
        verifyEqual(testCase, status.State, "ENABLED");
        verifyTrue(testCase, input.IsValid);
        verifyTrue(testCase, input.Enabled);
        verifyTrue(testCase, safety.ForceFresh);
        verifyTrue(testCase, safety.RobotFresh);
        verifyTrue(testCase, safety.QpHealthy);
        verifyTrue(testCase, safety.ControlPeriodHealthy);
        verifyTrue(testCase, safety.MotionGatesSatisfied);
    else
        verifyEqual(testCase, integrated.QdotAppliedRadSec, ...
            zeros(6, 1), 'AbsTol', 0);
        verifyEqual(testCase, integrated.QTargetRad, qState, ...
            'AbsTol', 0);
        verifyTrue(testCase, integrated.Reanchored);
    end
    if all(isfinite(integrated.QTargetRad))
        qState = integrated.QTargetRad;
    end
end
end

function [status, integrated] = cycle(adapter, fsm, integrator, ...
        pressed, nowSec, safety, qState, qdot)
adapter.submit(logical(pressed), nowSec, Source="test_software_input", ...
    SupportsPhysicalMotion=false);
status = fsm.step(adapter.read(nowSec), safety, nowSec);
integrated = integrator.step(qState, qdot, status, nowSec);
end

function status = driveToEnabled(adapter, fsm, safety)
adapter.submit(true, 0.010, Source="test");
fsm.step(adapter.read(0.010), safety, 0.010);
adapter.submit(true, 0.051, Source="test");
status = fsm.step(adapter.read(0.051), safety, 0.051);
assert(status.State == "ENABLED");
end

function safety = healthySafety()
safety = scopeguide.types.enableSafetyStatus();
safety.NeutralWrench = true;
safety.ForceFresh = true;
safety.RobotFresh = true;
safety.QpHealthy = true;
safety.ControlPeriodHealthy = true;
safety.MotionGatesSatisfied = true;
safety.StatusCode = "SYNTHETIC_HEALTHY";
end

function status = enabledStatus()
status = struct();
status.MotionPermitted = true;
status.ResetDynamicState = false;
status.CommandScale = 1;
end
