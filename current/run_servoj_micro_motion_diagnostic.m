function [summary, outputDirectory] = ...
        run_servoj_micro_motion_diagnostic(options)
%RUN_SERVOJ_MICRO_MOTION_DIAGNOSTIC Isolated tiny ServoJ motion test.
% This diagnostic bypasses HEX-H, force processing, admittance, QP, RCM,
% the Stage 8 integrator and software-enable GUI.  It commands one joint
% through a deterministic smooth 0 -> offset -> 0 trajectory and compares
% the 30004 joint feedback with the exact targets sent on port 30003.
%
% The diagnostic intentionally uses the original ZJFDobotCR5 interface from
% dobot_HW_control2: EnableRobot -> SpeedFactor -> pause -> ServoJ.  The old
% class command transport. ScopeGuide feedback extensions may be present,
% but this isolated test intentionally reads the public feedback values
% directly and never depends on asynchronous-response diagnostics.

arguments
    options.RobotIPAddress (1, 1) string = "192.168.50.105"
    options.DashboardPort (1, 1) double = 29999
    options.MovePort (1, 1) double = 30003
    options.FeedbackPort (1, 1) double = 30004
    options.JointIndex (1, 1) double = 4
    options.OffsetDeg (1, 1) double = 0.10
    options.ControlRateHz (1, 1) double = 20
    options.PreHoldSec (1, 1) double = 1.0
    options.OutboundRampSec (1, 1) double = 1.0
    options.PeakHoldSec (1, 1) double = 1.0
    options.ReturnRampSec (1, 1) double = 1.0
    options.FinalHoldSec (1, 1) double = 1.0
    options.LookaheadTime (1, 1) double = 60
    options.Gain (1, 1) double = 400
    options.InitialFeedbackTimeoutSec (1, 1) double = 5
    options.MaximumNonTestJointMotionDeg (1, 1) double = 0.15
    options.ArmPhysicalMotion (1, 1) logical = false
    options.MotionConfirmation (1, 1) string = ""
    options.ProgrammaticEnableConfirmation (1, 1) string = ""
    options.RobotInTcpModeConfirmed (1, 1) logical = false
    options.EnablePayloadKg (1, 1) double = 1.3
    options.SpeedRatioPercent (1, 1) double = 20
    options.PostEnablePauseSec (1, 1) double = 2.0
    options.DisableRobotOnExit (1, 1) logical = true
    options.FixtureAndClearanceConfirmed (1, 1) logical = false
    options.SecondObserverPresent (1, 1) logical = false
    options.KeepFigureOpenAfterRun (1, 1) logical = true
    options.WriteResults (1, 1) logical = true
end

validateOptions(options);
authorizeMotion(options);
projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'robot'));

servoTimeSec = 1 / options.ControlRateHz;
durationsSec = [options.PreHoldSec, options.OutboundRampSec, ...
    options.PeakHoldSec, options.ReturnRampSec, options.FinalHoldSec];
fprintf(['Isolated ServoJ MICRO-MOTION diagnostic: robot=%s, J%d, ' ...
    'offset=%+.3f deg, rate=%.1f Hz.\n'], ...
    options.RobotIPAddress, options.JointIndex, options.OffsetDeg, ...
    options.ControlRateHz);
fprintf(['ServoJ: t=%.4f s, lookahead_time=%.1f, gain=%.1f. ' ...
    'Expected test duration %.1f s.\n'], servoTimeSec, ...
    options.LookaheadTime, options.Gain, sum(durationsSec));
fprintf(['HEX-H, force processing, admittance, QP, RCM and the Stage 8 ' ...
    'integrator are NOT used.\n']);
fprintf(['Original-command compatibility mode: ServoJ and Dashboard use ' ...
    'the dobot_HW_control2 immediate-read transport.\n']);
fprintf(['The robot must be in TCP/IP mode. Its initial enable state is ' ...
    'not restricted. The program will call EnableRobot(%.3f), ' ...
    'SpeedFactor(%d), then wait %.1f s.\n'], ...
    options.EnablePayloadKg, options.SpeedRatioPercent, ...
    options.PostEnablePauseSec);
if options.DisableRobotOnExit
    fprintf('The program will call DisableRobot() during cleanup.\n');
else
    fprintf(['WARNING: DisableRobotOnExit=false; the robot will remain ' ...
        'enabled after this diagnostic.\n']);
end
fprintf('Keep the pendant and emergency stop reachable.\n');

robot = ZJFDobotCR5(char(options.RobotIPAddress), ...
    options.DashboardPort, options.MovePort, options.FeedbackPort);
robotCleanup = onCleanup(@() safeDisconnect(robot));
robot.Connect();
preEnable = waitForLegacyFeedback(robot, ...
    options.InitialFeedbackTimeoutSec);
fprintf('Initial mode is %s. Sending EnableRobot(%.3f).\n', ...
    string(preEnable.robotMode), options.EnablePayloadKg);
enableCleanup = onCleanup(@() safeDisable(robot, ...
    options.DisableRobotOnExit));
enableResponse = string(robot.Enable(options.EnablePayloadKg));
fprintf('Sending SpeedFactor(%d).\n', options.SpeedRatioPercent);
speedFactorResponse = string( ...
    robot.SetSpeedRatio(options.SpeedRatioPercent));
fprintf('Waiting %.1f s after programmatic enable.\n', ...
    options.PostEnablePauseSec);
pause(options.PostEnablePauseSec);
initial = waitForLegacyFeedback(robot, ...
    options.InitialFeedbackTimeoutSec);
requireServoMode(initial);
requireSpeedRatio(initial, options.SpeedRatioPercent);
initialJointDeg = double(initial.jointAnglesDeg(:).');
validateJointLimitMargin(initialJointDeg, options.JointIndex, ...
    options.OffsetDeg);
trajectory = scopeguide.diagnostics.buildServoJMicroMotionTrajectory( ...
    initialJointDeg, options.JointIndex, options.OffsetDeg, ...
    options.ControlRateHz, durationsSec);

fprintf('Initial q = [%s] deg; mode=%s.\n', ...
    sprintf('%+.5f ', initialJointDeg), string(initial.robotMode));
fprintf(['The test starts automatically in 3 s and then returns to the ' ...
    'initial target. Do not touch the robot.\n']);
for remaining = 3:-1:1
    fprintf('  %d...\n', remaining);
    waitWithLegacyFeedback(robot, 1);
end

sampleCount = height(trajectory);
data = initializeData(sampleCount);
testClock = tic;
sentCount = 0;
terminationIdentifier = "";
terminationMessage = "";
transportDiagnostics = struct( ...
    'Mode', "original_immediate_optional_response", ...
    'ResponseExpected', false, ...
    'OptionalResponseCount', 0, ...
    'ErrorCount', 0);
try
    for index = 1:sampleCount
        scheduled = trajectory.ScheduledTimeSec(index);
        waitUntil(testClock, scheduled);
        snapshot = readLegacySnapshot(robot);
        requireHealthyLegacyFeedback(snapshot);
        requireMotionWithinEnvelope(snapshot.jointAnglesDeg, ...
            initialJointDeg, options);

        sendStart = tic;
        response = robot.ServoJ( ...
            trajectory.TargetJointDeg(index, :), servoTimeSec, ...
            options.LookaheadTime, options.Gain);
        sendDuration = toc(sendStart);
        sentCount = sentCount + 1;
        if ~isempty(response)
            transportDiagnostics.OptionalResponseCount = ...
                transportDiagnostics.OptionalResponseCount + 1;
        end
        data = recordSample(data, sentCount, index, trajectory, ...
            snapshot, response, toc(testClock), sendDuration);
    end
catch exception
    terminationIdentifier = string(exception.identifier);
    terminationMessage = string(exception.message);
    fprintf(2, 'Micro-motion diagnostic stopped safely: %s\n', ...
        terminationIdentifier);
    fprintf(2, '  %s\n', terminationMessage);
end

samples = dataToTable(data, sentCount);
initialization = struct( ...
    'PreEnableRobotMode', string(preEnable.robotMode), ...
    'EnableRobotCalled', true, ...
    'EnablePayloadKg', options.EnablePayloadKg, ...
    'EnableResponse', enableResponse, ...
    'SpeedFactorCalled', true, ...
    'SpeedRatioPercent', options.SpeedRatioPercent, ...
    'SpeedFactorResponse', speedFactorResponse, ...
    'PostEnablePauseSec', options.PostEnablePauseSec, ...
    'PostEnableRobotMode', string(initial.robotMode), ...
    'DisableRobotOnExit', options.DisableRobotOnExit);
summary = summarizeRun(samples, trajectory, initialJointDeg, options, ...
    initialization, transportDiagnostics, terminationIdentifier, ...
    terminationMessage);
outputDirectory = "";
if options.WriteResults
    outputDirectory = makeOutputDirectory(projectRoot);
    writetable(samples, fullfile(outputDirectory, 'samples.csv'));
    writetable(trajectory, fullfile(outputDirectory, ...
        'planned_trajectory.csv'));
    save(fullfile(outputDirectory, ...
        'servoj_micro_motion_diagnostic.mat'), ...
        'summary', 'samples', 'trajectory', 'initialJointDeg', ...
        'initialization', 'options');
    writeJson(fullfile(outputDirectory, 'summary.json'), summary);
end

figureHandle = renderFigure(samples, trajectory, summary, options);
if options.WriteResults
    exportgraphics(figureHandle, fullfile(outputDirectory, ...
        'servoj_micro_motion_diagnostic.png'), 'Resolution', 180);
end
if options.KeepFigureOpenAfterRun
    set(figureHandle, 'Visible', 'on');
    fprintf(['Diagnostic figure retained for manual review; close it ' ...
        'manually when finished.\n']);
else
    close(figureHandle);
end

fprintf('\nServoJ micro-motion result: %s.\n', summary.Status);
fprintf(['Target/actual excursion on J%d: %+.4f / %+.4f deg; ' ...
    'tracking ratio %.1f%%; final return error %.4f deg.\n'], ...
    options.JointIndex, summary.TargetSignedExcursionDeg, ...
    summary.ActualSignedExcursionDeg, ...
    100 * summary.TrackingRatio, summary.FinalReturnErrorDeg);
fprintf(['ServoJ sent=%d; actual send rate=%.2f Hz; send P99=%.3f ms; ' ...
    'nonempty immediate response reads=%d.\n'], summary.CommandSentCount, ...
    summary.ActualSendRateHz, summary.SendDurationP99Ms, ...
    summary.OptionalResponseCount);
if options.WriteResults
    fprintf('Results: %s\n', outputDirectory);
end
clear enableCleanup;
clear robotCleanup;
end

function validateOptions(options)
ports = [options.DashboardPort, options.MovePort, options.FeedbackPort];
if any(~isfinite(ports)) || any(ports ~= fix(ports)) || ...
        any(ports < 1) || any(ports > 65535)
    error('scopeguide:servoJMicro:InvalidPort', ...
        'TCP ports must be integer values in [1, 65535].');
end
if options.JointIndex ~= fix(options.JointIndex) || ...
        options.JointIndex < 1 || options.JointIndex > 6
    error('scopeguide:servoJMicro:InvalidJointIndex', ...
        'JointIndex must be an integer from 1 to 6.');
end
if ~isfinite(options.OffsetDeg) || options.OffsetDeg == 0 || ...
        abs(options.OffsetDeg) > 2.0
    error('scopeguide:servoJMicro:OffsetTooLarge', ...
        'OffsetDeg must be nonzero and no larger than 0.20 deg in magnitude.');
end
timing = [options.ControlRateHz, options.PreHoldSec, ...
    options.OutboundRampSec, options.PeakHoldSec, ...
    options.ReturnRampSec, options.FinalHoldSec, ...
    options.InitialFeedbackTimeoutSec, options.MaximumNonTestJointMotionDeg];
if any(~isfinite(timing)) || any(timing <= 0) || ...
        options.ControlRateHz > 25 || sum(timing(2:6)) > 15
    error('scopeguide:servoJMicro:InvalidTiming', ...
        ['Timing values must be positive and finite; rate is limited to ' ...
         '25 Hz and commanded motion duration to 15 seconds.']);
end
if options.LookaheadTime < 20 || options.LookaheadTime > 100 || ...
        options.Gain < 200 || options.Gain > 1000
    error('scopeguide:servoJMicro:InvalidServoParameters', ...
        ['lookahead_time must be in [20,100] and gain must be in ' ...
         '[200,1000].']);
end
if ~isfinite(options.EnablePayloadKg) || options.EnablePayloadKg < 0 || ...
        options.EnablePayloadKg > 5
    error('scopeguide:servoJMicro:InvalidEnablePayload', ...
        'EnablePayloadKg must be finite and in [0,5] kg.');
end
if ~isfinite(options.SpeedRatioPercent) || ...
        options.SpeedRatioPercent ~= fix(options.SpeedRatioPercent) || ...
        options.SpeedRatioPercent < 1 || options.SpeedRatioPercent > 100
    error('scopeguide:servoJMicro:InvalidSpeedRatio', ...
        'SpeedRatioPercent must be an integer in [1,100].');
end
if ~isfinite(options.PostEnablePauseSec) || ...
        options.PostEnablePauseSec < 2 || options.PostEnablePauseSec > 10
    error('scopeguide:servoJMicro:InvalidPostEnablePause', ...
        'PostEnablePauseSec must be in [2,10] seconds.');
end
end

function authorizeMotion(options)
if ~options.ArmPhysicalMotion
    error('scopeguide:servoJMicro:PhysicalMotionNotArmed', ...
        'Set ArmPhysicalMotion=true only after the fixture is ready.');
end
if options.MotionConfirmation ~= ...
        "RUN_SERVOJ_MICRO_MOTION_DIAGNOSTIC"
    error('scopeguide:servoJMicro:ConfirmationMismatch', ...
        ['MotionConfirmation must exactly equal ' ...
         '"RUN_SERVOJ_MICRO_MOTION_DIAGNOSTIC".']);
end
if options.ProgrammaticEnableConfirmation ~= ...
        "ENABLE_ROBOT_FROM_MICRO_DIAGNOSTIC"
    error('scopeguide:servoJMicro:EnableConfirmationMismatch', ...
        ['ProgrammaticEnableConfirmation must exactly equal ' ...
         '"ENABLE_ROBOT_FROM_MICRO_DIAGNOSTIC".']);
end
if ~options.RobotInTcpModeConfirmed || ...
        ~options.FixtureAndClearanceConfirmed || ...
        ~options.SecondObserverPresent
    error('scopeguide:servoJMicro:SafetyAttestationMissing', ...
        ['RobotInTcpModeConfirmed, ' ...
         'FixtureAndClearanceConfirmed and SecondObserverPresent must ' ...
         'all be true.']);
end
end

function snapshot = waitForLegacyFeedback(robot, timeoutSec)
clock = tic;
while toc(clock) < timeoutSec
    snapshot = readLegacySnapshot(robot);
    mode = string(snapshot.robotMode);
    if snapshot.isConnected && ...
            ~any(mode == ["Disconnected", "UNKNOWN"]) && ...
            all(isfinite(snapshot.jointAnglesDeg)) && ...
            all(isfinite(snapshot.actualJointSpeedsDegSec))
        return;
    end
    pause(0.005);
end
error('scopeguide:servoJMicro:InitialFeedbackTimeout', ...
    ['The original ZJFDobotCR5 class did not publish valid 30004 ' ...
     'feedback within %.3f s.'], timeoutSec);
end

function waitWithLegacyFeedback(robot, durationSec)
clock = tic;
while toc(clock) < durationSec
    snapshot = readLegacySnapshot(robot);
    requireHealthyLegacyFeedback(snapshot);
    pause(0.005);
end
end

function snapshot = readLegacySnapshot(robot)
snapshot = struct( ...
    'isConnected', logical(robot.IsConnected), ...
    'robotMode', string(robot.RobotMode), ...
    'currentSpeedRatio', double(robot.CurrentSpeedRatio), ...
    'jointAnglesDeg', double(robot.JointAngles(:).'), ...
    'actualJointSpeedsDegSec', ...
        double(robot.ActualJointSpeeds(:).'));
end

function requireHealthyLegacyFeedback(snapshot)
if ~snapshot.isConnected || ...
        any(~isfinite(snapshot.jointAnglesDeg)) || ...
        any(~isfinite(snapshot.actualJointSpeedsDegSec))
    error('scopeguide:servoJMicro:FeedbackInvalid', ...
        'The original ZJFDobotCR5 feedback properties are invalid.');
end
requireServoMode(snapshot);
end

function requireServoMode(snapshot)
if ~any(string(snapshot.robotMode) == ["ENABLE", "RUNNING"])
    error('scopeguide:servoJMicro:RobotModeRejectsServo', ...
        'Robot mode is %s; expected ENABLE or RUNNING.', ...
        string(snapshot.robotMode));
end
end

function requireSpeedRatio(snapshot, requestedRatio)
if ~isfinite(snapshot.currentSpeedRatio) || ...
        abs(snapshot.currentSpeedRatio - requestedRatio) > 0.5
    error('scopeguide:servoJMicro:SpeedFactorNotConfirmed', ...
        ['SpeedFactor(%d) was sent, but 30004 reports %.3f. Do not start ' ...
         'ServoJ until the requested speed ratio is confirmed.'], ...
        requestedRatio, snapshot.currentSpeedRatio);
end
end

function validateJointLimitMargin(initialDeg, jointIndex, offsetDeg)
lower = [-360, -360, -160, -360, -360, -360] + 5;
upper = [ 360,  360,  160,  360,  360,  360] - 5;
target = initialDeg;
target(jointIndex) = target(jointIndex) + offsetDeg;
if any(initialDeg <= lower) || any(initialDeg >= upper) || ...
        any(target <= lower) || any(target >= upper)
    error('scopeguide:servoJMicro:JointLimitMargin', ...
        'Initial or peak target violates the 5 deg CR5 joint-limit margin.');
end
end

function requireMotionWithinEnvelope(actualDeg, initialDeg, options)
delta = wrappedDifferenceDeg(actualDeg, initialDeg);
other = true(1, 6);
other(options.JointIndex) = false;
if any(abs(delta(other)) > options.MaximumNonTestJointMotionDeg)
    error('scopeguide:servoJMicro:UnexpectedJointMotion', ...
        'A non-test joint moved more than %.3f deg.', ...
        options.MaximumNonTestJointMotionDeg);
end
selectedLimit = abs(options.OffsetDeg) + ...
    options.MaximumNonTestJointMotionDeg;
if abs(delta(options.JointIndex)) > selectedLimit
    error('scopeguide:servoJMicro:TestJointMotionTooLarge', ...
        'J%d moved %.3f deg; safety envelope is %.3f deg.', ...
        options.JointIndex, delta(options.JointIndex), selectedLimit);
end
end

function waitUntil(clock, scheduledSec)
while true
    remaining = scheduledSec - toc(clock);
    if remaining <= 0
        return;
    end
    pause(min(0.002, max(remaining / 2, 0.0002)));
end
end

function data = initializeData(count)
data = struct();
data.TrajectoryIndex = zeros(count, 1);
data.ScheduledTimeSec = nan(count, 1);
data.SendTimeSec = nan(count, 1);
data.SendDurationSec = nan(count, 1);
data.Phase = strings(count, 1);
data.RobotMode = strings(count, 1);
data.CurrentSpeedRatio = nan(count, 1);
data.ImmediateResponseReceived = false(count, 1);
data.ImmediateResponse = strings(count, 1);
data.TargetDeg = nan(count, 6);
data.ActualDeg = nan(count, 6);
data.ActualSpeedDegSec = nan(count, 6);
end

function data = recordSample(data, row, trajectoryIndex, trajectory, ...
        snapshot, response, sendTimeSec, sendDurationSec)
data.TrajectoryIndex(row) = trajectoryIndex;
data.ScheduledTimeSec(row) = ...
    trajectory.ScheduledTimeSec(trajectoryIndex);
data.SendTimeSec(row) = sendTimeSec;
data.SendDurationSec(row) = sendDurationSec;
data.Phase(row) = trajectory.Phase(trajectoryIndex);
data.RobotMode(row) = string(snapshot.robotMode);
data.CurrentSpeedRatio(row) = double(snapshot.currentSpeedRatio);
data.ImmediateResponseReceived(row) = ...
    ~isempty(response);
data.ImmediateResponse(row) = string(char(response(:).'));
data.TargetDeg(row, :) = trajectory.TargetJointDeg(trajectoryIndex, :);
data.ActualDeg(row, :) = double(snapshot.jointAnglesDeg(:).');
data.ActualSpeedDegSec(row, :) = ...
    double(snapshot.actualJointSpeedsDegSec(:).');
end

function samples = dataToTable(data, count)
indices = (1:count).';
if count == 0
    indices = zeros(0, 1);
end
sendIntervalSec = nan(count, 1);
if count >= 2
    sendIntervalSec(2:end) = diff(data.SendTimeSec(indices));
end
samples = table( ...
    data.TrajectoryIndex(indices), data.ScheduledTimeSec(indices), ...
    data.SendTimeSec(indices), ...
    sendIntervalSec, ...
    data.SendTimeSec(indices) - data.ScheduledTimeSec(indices), ...
    data.SendDurationSec(indices), data.Phase(indices), ...
    data.RobotMode(indices), data.CurrentSpeedRatio(indices), ...
    data.ImmediateResponseReceived(indices), ...
    data.ImmediateResponse(indices), ...
    data.TargetDeg(indices, 1), data.TargetDeg(indices, 2), ...
    data.TargetDeg(indices, 3), data.TargetDeg(indices, 4), ...
    data.TargetDeg(indices, 5), data.TargetDeg(indices, 6), ...
    data.ActualDeg(indices, 1), data.ActualDeg(indices, 2), ...
    data.ActualDeg(indices, 3), data.ActualDeg(indices, 4), ...
    data.ActualDeg(indices, 5), data.ActualDeg(indices, 6), ...
    data.ActualSpeedDegSec(indices, 1), ...
    data.ActualSpeedDegSec(indices, 2), ...
    data.ActualSpeedDegSec(indices, 3), ...
    data.ActualSpeedDegSec(indices, 4), ...
    data.ActualSpeedDegSec(indices, 5), ...
    data.ActualSpeedDegSec(indices, 6), ...
    'VariableNames', {'TrajectoryIndex', 'ScheduledTimeSec', ...
    'SendTimeSec', 'SendIntervalSec', 'SendJitterSec', ...
    'SendDurationSec', 'Phase', 'RobotMode', 'CurrentSpeedRatio', ...
    'ImmediateResponseReceived', ...
    'ImmediateResponse', ...
    'TargetJ1Deg', 'TargetJ2Deg', 'TargetJ3Deg', 'TargetJ4Deg', ...
    'TargetJ5Deg', 'TargetJ6Deg', ...
    'ActualJ1Deg', 'ActualJ2Deg', 'ActualJ3Deg', 'ActualJ4Deg', ...
    'ActualJ5Deg', 'ActualJ6Deg', ...
    'ActualSpeedJ1DegSec', 'ActualSpeedJ2DegSec', ...
    'ActualSpeedJ3DegSec', 'ActualSpeedJ4DegSec', ...
    'ActualSpeedJ5DegSec', 'ActualSpeedJ6DegSec'});
end

function summary = summarizeRun(samples, trajectory, initialDeg, options, ...
        initialization, transport, terminationIdentifier, ...
        terminationMessage)
summary = struct();
summary.SchemaVersion = "1.1-servoj-micro-motion-original-class";
summary.Status = "ABORTED";
summary.RobotIPAddress = options.RobotIPAddress;
summary.JointIndex = options.JointIndex;
summary.OffsetDeg = options.OffsetDeg;
summary.ControlRateHz = options.ControlRateHz;
summary.ServoTimeSec = 1 / options.ControlRateHz;
summary.LookaheadTime = options.LookaheadTime;
summary.Gain = options.Gain;
summary.InitialJointDeg = initialDeg;
summary.PlannedCommandCount = height(trajectory);
summary.CommandSentCount = height(samples);
summary.TerminationIdentifier = string(terminationIdentifier);
summary.TerminationMessage = string(terminationMessage);
summary.ProgrammaticInitialization = initialization;
summary.NoForceSensorUsed = true;
summary.NoAdmittanceQpRcmOrIntegratorUsed = true;
summary.ProgramCalledEnableRobot = true;
summary.ProgramCalledSpeedFactor = true;
summary.ProgramCallsDisableRobotOnExit = options.DisableRobotOnExit;
summary.ServoReplyRequired = false;
summary.RobotClassImplementation = ...
    "dobot_HW_control2_command_transport_with_feedback_extensions";
summary.FeedbackAgeMeasurementAvailable = true;
summary.FeedbackAgeRecordedByThisDiagnostic = false;
summary.ControllerQueueFlagsAvailable = true;
summary.ControllerQueueFlagsRecordedByThisDiagnostic = false;
summary.TransportDiagnosticsAvailable = false;
summary.OptionalResponseCount = getFieldOr( ...
    transport, 'OptionalResponseCount', 0);
summary.AsynchronousControllerErrorCount = getFieldOr( ...
    transport, 'ErrorCount', 0);
summary.TargetSignedExcursionDeg = options.OffsetDeg;
summary.ActualSignedExcursionDeg = NaN;
summary.MaximumAbsoluteActualExcursionDeg = NaN;
summary.TrackingRatio = NaN;
summary.FinalReturnErrorDeg = NaN;
summary.MaximumTrackingErrorDeg = NaN;
summary.ActualSendRateHz = NaN;
summary.SendDurationP99Ms = NaN;
summary.SendIntervalP99Ms = NaN;
summary.MaximumFeedbackAgeSec = NaN;
summary.ControllerEnteredRunningMode = false;
summary.RunQueuedCommandObserved = false;
summary.PauseCommandFlagObserved = false;
summary.MinimumReportedSpeedRatio = NaN;
summary.MaximumReportedSpeedRatio = NaN;
if isempty(samples)
    return;
end

j = options.JointIndex;
target = samples.(sprintf('TargetJ%dDeg', j));
actual = samples.(sprintf('ActualJ%dDeg', j));
actualDelta = wrappedDifferenceDeg(actual, initialDeg(j));
direction = sign(options.OffsetDeg);
directionalExcursionDeg = max(direction * actualDelta);
summary.ActualSignedExcursionDeg = direction * directionalExcursionDeg;
summary.MaximumAbsoluteActualExcursionDeg = max(abs(actualDelta));
summary.TrackingRatio = max(0, directionalExcursionDeg / ...
    abs(options.OffsetDeg));
summary.FinalReturnErrorDeg = abs(actualDelta(end));
summary.MaximumTrackingErrorDeg = max(abs( ...
    wrappedDifferenceDeg(target, actual)));
if height(samples) >= 2 && samples.SendTimeSec(end) > samples.SendTimeSec(1)
    summary.ActualSendRateHz = (height(samples) - 1) / ...
        (samples.SendTimeSec(end) - samples.SendTimeSec(1));
end
summary.SendDurationP99Ms = 1000 * percentile( ...
    samples.SendDurationSec, 99);
summary.SendIntervalP99Ms = 1000 * percentile( ...
    samples.SendIntervalSec, 99);
summary.ControllerEnteredRunningMode = any(samples.RobotMode == "RUNNING");
summary.MinimumReportedSpeedRatio = min(samples.CurrentSpeedRatio);
summary.MaximumReportedSpeedRatio = max(samples.CurrentSpeedRatio);

if strlength(terminationIdentifier) > 0 || ...
        height(samples) < height(trajectory)
    summary.Status = "ABORTED";
elseif directionalExcursionDeg < max(0.02, ...
        0.20 * abs(options.OffsetDeg))
    summary.Status = "NO_MEASURABLE_MOTION";
elseif summary.TrackingRatio < 0.50
    summary.Status = "PARTIAL_TRACKING";
elseif summary.FinalReturnErrorDeg > 0.05
    summary.Status = "RETURN_ERROR";
else
    summary.Status = "PASS";
end
end

function figureHandle = renderFigure(samples, trajectory, summary, options)
figureHandle = figure('Name', 'ScopeGuide Isolated ServoJ Micro Motion', ...
    'Tag', 'ScopeGuideServoJMicroMotionDiagnostic', ...
    'NumberTitle', 'off', 'Color', 'w', 'Visible', 'off');
layout = tiledlayout(figureHandle, 4, 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf( ...
    'Isolated ServoJ J%d %+.3f deg | %s | no force/QP/RCM/integrator', ...
    options.JointIndex, options.OffsetDeg, char(summary.Status)));

j = options.JointIndex;
plannedDelta = wrappedDifferenceDeg( ...
    trajectory.TargetJointDeg(:, j), trajectory.TargetJointDeg(1, j));
ax1 = nexttile(layout);
plot(ax1, trajectory.ScheduledTimeSec, plannedDelta, 'k--', ...
    'LineWidth', 1.4, 'DisplayName', 'target');
hold(ax1, 'on');
if ~isempty(samples)
    actual = samples.(sprintf('ActualJ%dDeg', j));
    actualDelta = wrappedDifferenceDeg(actual, summary.InitialJointDeg(j));
    plot(ax1, samples.SendTimeSec, actualDelta, 'b-', ...
        'LineWidth', 1.3, 'DisplayName', '30004 actual');
end
ylabel(ax1, sprintf('J%d Δq [deg]', j));
legend(ax1, 'Location', 'best'); grid(ax1, 'on');

ax2 = nexttile(layout);
if ~isempty(samples)
    target = samples.(sprintf('TargetJ%dDeg', j));
    actual = samples.(sprintf('ActualJ%dDeg', j));
    plot(ax2, samples.SendTimeSec, ...
        wrappedDifferenceDeg(target, actual), 'r-', 'LineWidth', 1.2);
else
    plot(ax2, NaN, NaN);
end
ylabel(ax2, 'target-actual [deg]'); grid(ax2, 'on');

ax3 = nexttile(layout);
if ~isempty(samples)
    speed = samples.(sprintf('ActualSpeedJ%dDegSec', j));
    plot(ax3, samples.SendTimeSec, speed, 'Color', [0.1, 0.6, 0.2], ...
        'LineWidth', 1.2);
else
    plot(ax3, NaN, NaN);
end
ylabel(ax3, sprintf('J%d speed [deg/s]', j)); grid(ax3, 'on');

ax4 = nexttile(layout);
if ~isempty(samples)
    stairs(ax4, samples.SendTimeSec, samples.CurrentSpeedRatio, 'k-', ...
        'LineWidth', 1.1, 'DisplayName', '30004 SpeedRatio');
    hold(ax4, 'on');
    yline(ax4, options.SpeedRatioPercent, 'b--', ...
        'DisplayName', 'requested SpeedFactor');
else
    plot(ax4, NaN, NaN);
end
ylabel(ax4, 'Speed ratio [%]'); xlabel(ax4, 'Time [s]');
legend(ax4, 'Location', 'best'); grid(ax4, 'on');
linkaxes([ax1, ax2, ax3, ax4], 'x');
end

function value = getFieldOr(structValue, fieldName, defaultValue)
value = defaultValue;
if isstruct(structValue) && isfield(structValue, fieldName)
    candidate = double(structValue.(fieldName));
    if isscalar(candidate) && isfinite(candidate)
        value = candidate;
    end
end
end

function difference = wrappedDifferenceDeg(a, b)
difference = mod(double(a) - double(b) + 180, 360) - 180;
end

function value = percentile(values, percentage)
values = sort(double(values(isfinite(values))));
if isempty(values)
    value = NaN;
    return;
end
position = 1 + (numel(values) - 1) * percentage / 100;
lowerIndex = floor(position);
upperIndex = ceil(position);
if lowerIndex == upperIndex
    value = values(lowerIndex);
else
    fraction = position - lowerIndex;
    value = values(lowerIndex) * (1 - fraction) + ...
        values(upperIndex) * fraction;
end
end

function outputDirectory = makeOutputDirectory(projectRoot)
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
outputDirectory = string(fullfile(projectRoot, 'results', ...
    ['servoj_micro_motion_diagnostic_', stamp]));
mkdir(outputDirectory);
end

function writeJson(path, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end
[fileId, message] = fopen(path, 'w');
if fileId < 0
    error('scopeguide:servoJMicro:CannotWriteJson', ...
        'Cannot open %s: %s', path, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end

function safeDisconnect(robot)
try
    if robot.IsConnected
        robot.Disconnect();
    end
catch exception
    warning('scopeguide:servoJMicro:DisconnectFailed', '%s', ...
        exception.message);
end
end

function safeDisable(robot, shouldDisable)
if ~shouldDisable
    return;
end
try
    if robot.IsConnected
        response = robot.Disable();
        fprintf('Programmatic cleanup DisableRobot response: %s\n', ...
            strtrim(char(response)));
    end
catch exception
    warning('scopeguide:servoJMicro:DisableFailed', ...
        ['Automatic DisableRobot failed: %s. Use the pendant or ' ...
         'emergency stop to disable the robot.'], exception.message);
end
end
