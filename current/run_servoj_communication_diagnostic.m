function [summary, outputDirectory] = ...
        run_servoj_communication_diagnostic(options)
%RUN_SERVOJ_COMMUNICATION_DIAGNOSTIC Isolate Dobot 30003 ServoJ timing.
% This diagnostic does not connect to HEX-H, run QP/RCM control, or call
% EnableRobot/DisableRobot. It repeatedly sends the joint position measured
% immediately before each trial, so no intentional motion is requested.
%
% The test compares:
%   1. Previous ScopeGuide: ServoJ(q)
%   2. Parameterized: ServoJ(q,t=1/rate,lookahead_time=60,gain=400)
% and two receive strategies:
%   A. continuously drain and frame every available 30003 reply;
%   B. reproduce the legacy one-shot NumBytesAvailable check after write.

arguments
    options.RobotIPAddress (1, 1) string = "192.168.50.105"
    options.MovePort (1, 1) double = 30003
    options.FeedbackPort (1, 1) double = 30004
    options.CommandProfiles (1, :) string = ...
        ["scopeguide_minimal", "legacy_parameterized"]
    options.ReadStrategies (1, :) string = ...
        ["continuous_framed", "legacy_immediate"]
    options.CommandRatesHz (1, :) double = [20, 25]
    options.DurationPerTrialSec (1, 1) double = 3
    options.ResponseDrainSec (1, 1) double = 3
    options.InterTrialPauseSec (1, 1) double = 0.5
    options.PollPeriodSec (1, 1) double = 0.001
    options.InitialFeedbackTimeoutSec (1, 1) double = 5
    options.MaximumFeedbackAgeSec (1, 1) double = 0.15
    options.MaximumHoldDeviationDeg (1, 1) double = 0.25
    options.LegacyT (1, 1) double = 0.04
    options.LegacyLookaheadTime (1, 1) double = 60
    options.LegacyGain (1, 1) double = 400
    options.ArmPhysicalMotion (1, 1) logical = false
    options.MotionConfirmation (1, 1) string = ""
    options.FixtureAndClearanceConfirmed (1, 1) logical = false
    options.SecondObserverPresent (1, 1) logical = false
    options.ShowFigure (1, 1) logical = true
    options.WriteResults (1, 1) logical = true
end

validateOptions(options);
authorizePhysicalDiagnostic(options);

projectRoot = string(fileparts(mfilename('fullpath')));
addpath(projectRoot);
addpath(fullfile(projectRoot, 'robot'));

fprintf('ServoJ 30003 isolated HOLD diagnostic. Robot=%s:%d.\n', ...
    options.RobotIPAddress, options.MovePort);
fprintf(['No intentional motion is requested; every trial holds the joint ' ...
    'position sampled immediately before it.\n']);
fprintf(['The program does not enable/disable the robot and has no force ' ...
    'sensor safety channel. Keep the pendant and emergency stop reachable.\n']);
fprintf(['Legacy reference: 25 Hz, ServoJ t=%.3f, lookahead_time=%.1f, ' ...
    'gain=%.1f; empty immediate replies were accepted silently.\n'], ...
    options.LegacyT, options.LegacyLookaheadTime, options.LegacyGain);

programClock = tic;
feedbackClient = tcpclient(char(options.RobotIPAddress), ...
    options.FeedbackPort, 'Timeout', 1);
feedbackCleanup = onCleanup(@() releaseTcpClient(feedbackClient));
feedbackState = emptyFeedbackState();
feedbackState = waitForInitialFeedback(feedbackClient, feedbackState, ...
    programClock, options.InitialFeedbackTimeoutSec);
requireServoCapableMode(feedbackState, options.MaximumFeedbackAgeSec, ...
    programClock);
fprintf('Feedback ready: mode=%s; q=[%s] deg.\n', ...
    feedbackState.ModeName, sprintf('%+.4f ', feedbackState.JointDeg));

trialCount = numel(options.CommandRatesHz) * ...
    numel(options.CommandProfiles) * numel(options.ReadStrategies);
fprintf('Planned isolated trials: %d. Starting in 3 seconds.\n', trialCount);
for remaining = 3:-1:1
    fprintf('  %d...\n', remaining);
    waitClock = tic;
    while toc(waitClock) < 1
        feedbackState = pollFeedback( ...
            feedbackClient, feedbackState, programClock);
        pause(min(options.PollPeriodSec, 0.005));
    end
    requireServoCapableMode(feedbackState, ...
        options.MaximumFeedbackAgeSec, programClock);
end

commandTables = cell(trialCount, 1);
chunkTables = cell(trialCount, 1);
trialRecords = repmat(emptyTrialRecord(), trialCount, 1);
trialIndex = 0;
abortRemaining = false;

for rateHz = options.CommandRatesHz
    for profile = options.CommandProfiles
        for readStrategy = options.ReadStrategies
            trialIndex = trialIndex + 1;
            fprintf(['\nTrial %d/%d: profile=%s; read=%s; ' ...
                'rate=%.1f Hz; send=%.1f s; drain=%.1f s.\n'], ...
                trialIndex, trialCount, profile, readStrategy, rateHz, ...
                options.DurationPerTrialSec, options.ResponseDrainSec);
            [commandTables{trialIndex}, chunkTables{trialIndex}, ...
                trialRecords(trialIndex), feedbackState] = runOneTrial( ...
                trialIndex, profile, readStrategy, rateHz, options, ...
                feedbackClient, feedbackState, programClock);
            printTrialResult(trialRecords(trialIndex));
            if strlength(trialRecords(trialIndex).SafetyAbortReason) > 0
                fprintf(['Remaining trials cancelled because this trial ' ...
                    'reported: %s\n'], ...
                    trialRecords(trialIndex).SafetyAbortReason);
                abortRemaining = true;
                break;
            end
            feedbackState = waitWithFeedback(options.InterTrialPauseSec, ...
                feedbackClient, feedbackState, programClock, ...
                options.PollPeriodSec);
        end
        if abortRemaining
            break;
        end
    end
    if abortRemaining
        break;
    end
end

commandTables = commandTables(1:trialIndex);
chunkTables = chunkTables(1:trialIndex);
trialRecords = trialRecords(1:trialIndex);
commands = concatenateTables(commandTables);
readChunks = concatenateTables(chunkTables);
trials = struct2table(trialRecords);

summary = buildSummary(trials, commands, readChunks, options, ...
    feedbackState, trialCount, trialIndex);
outputDirectory = "";
if options.WriteResults
    outputDirectory = makeOutputDirectory(projectRoot);
    writetable(commands, fullfile(outputDirectory, 'commands.csv'));
    writetable(readChunks, fullfile(outputDirectory, 'read_chunks.csv'));
    writetable(trials, fullfile(outputDirectory, 'trials.csv'));
    save(fullfile(outputDirectory, 'servoj_communication_diagnostic.mat'), ...
        'summary', 'commands', 'readChunks', 'trials', 'options');
    writeJson(fullfile(outputDirectory, 'summary.json'), summary);
end

figureHandle = renderDiagnosticFigure(commands, trials);
if options.WriteResults
    exportgraphics(figureHandle, ...
        fullfile(outputDirectory, 'servoj_communication_diagnostic.png'), ...
        'Resolution', 180);
end
if options.ShowFigure
    set(figureHandle, 'Visible', 'on');
else
    close(figureHandle);
end

fprintf('\nServoJ communication diagnostic: %s.\n', summary.Status);
for index = 1:numel(summary.Findings)
    fprintf('  - %s\n', summary.Findings(index));
end
if options.WriteResults
    fprintf('Results: %s\n', outputDirectory);
end
end

function validateOptions(options)
allowedProfiles = ["scopeguide_minimal", "legacy_parameterized"];
allowedReadStrategies = ["continuous_framed", "legacy_immediate"];
if isempty(options.CommandProfiles) || ...
        any(~ismember(options.CommandProfiles, allowedProfiles)) || ...
        numel(unique(options.CommandProfiles)) ~= ...
        numel(options.CommandProfiles)
    error('scopeguide:servoJDiagnostic:InvalidProfiles', ...
        'CommandProfiles must be unique supported profile names.');
end
if isempty(options.ReadStrategies) || ...
        any(~ismember(options.ReadStrategies, allowedReadStrategies)) || ...
        numel(unique(options.ReadStrategies)) ~= ...
        numel(options.ReadStrategies)
    error('scopeguide:servoJDiagnostic:InvalidReadStrategies', ...
        'ReadStrategies must be unique supported strategy names.');
end
if isempty(options.CommandRatesHz) || ...
        any(~isfinite(options.CommandRatesHz)) || ...
        any(options.CommandRatesHz <= 0) || ...
        any(options.CommandRatesHz > 30) || ...
        numel(unique(options.CommandRatesHz)) ~= ...
        numel(options.CommandRatesHz)
    error('scopeguide:servoJDiagnostic:InvalidRates', ...
        'CommandRatesHz must contain unique values in (0, 30] Hz.');
end
positiveValues = [options.DurationPerTrialSec, ...
    options.ResponseDrainSec, options.InterTrialPauseSec, ...
    options.PollPeriodSec, options.InitialFeedbackTimeoutSec, ...
    options.MaximumFeedbackAgeSec, options.MaximumHoldDeviationDeg, ...
    options.LegacyT, options.LegacyLookaheadTime, options.LegacyGain];
if any(~isfinite(positiveValues)) || any(positiveValues <= 0) || ...
        options.DurationPerTrialSec > 10 || ...
        options.ResponseDrainSec > 10
    error('scopeguide:servoJDiagnostic:InvalidTiming', ...
        ['Diagnostic values must be finite and positive; per-trial send ' ...
         'and drain durations may not exceed 10 seconds.']);
end
ports = [options.MovePort, options.FeedbackPort];
if any(~isfinite(ports)) || any(ports < 1) || any(ports > 65535) || ...
        any(ports ~= fix(ports))
    error('scopeguide:servoJDiagnostic:InvalidPort', ...
        'TCP ports must be integers in [1, 65535].');
end
maximumCommands = sum(ceil(options.CommandRatesHz * ...
    options.DurationPerTrialSec)) * numel(options.CommandProfiles) * ...
    numel(options.ReadStrategies);
if maximumCommands > 1000
    error('scopeguide:servoJDiagnostic:TooManyCommands', ...
        'The requested matrix may send %d commands; the limit is 1000.', ...
        maximumCommands);
end
end

function authorizePhysicalDiagnostic(options)
if ~options.ArmPhysicalMotion
    error('scopeguide:servoJDiagnostic:PhysicalMotionNotArmed', ...
        ['This diagnostic sends ServoJ commands. Set ' ...
         'ArmPhysicalMotion=true only with the fixture and emergency stop ready.']);
end
if options.MotionConfirmation ~= "RUN_SERVOJ_HOLD_DIAGNOSTIC"
    error('scopeguide:servoJDiagnostic:ConfirmationMismatch', ...
        ['MotionConfirmation must exactly equal ' ...
         '"RUN_SERVOJ_HOLD_DIAGNOSTIC".']);
end
if ~options.FixtureAndClearanceConfirmed || ...
        ~options.SecondObserverPresent
    error('scopeguide:servoJDiagnostic:FixtureAttestationMissing', ...
        ['FixtureAndClearanceConfirmed and SecondObserverPresent must ' ...
         'both be true.']);
end
end

function state = emptyFeedbackState()
state = struct( ...
    'Buffer', zeros(1, 0, 'uint8'), ...
    'FrameCount', uint64(0), ...
    'LastFrameProgramSec', NaN, ...
    'JointDeg', nan(1, 6), ...
    'ModeCode', NaN, ...
    'ModeName', "UNKNOWN", ...
    'DroppedByteCount', uint64(0));
end

function state = waitForInitialFeedback(client, state, programClock, timeoutSec)
waitClock = tic;
while toc(waitClock) < timeoutSec
    state = pollFeedback(client, state, programClock);
    if state.FrameCount >= 3 && all(isfinite(state.JointDeg))
        return;
    end
    pause(0.002);
end
error('scopeguide:servoJDiagnostic:InitialFeedbackTimeout', ...
    'No validated 30004 feedback was received within %.3f seconds.', ...
    timeoutSec);
end

function state = pollFeedback(client, state, programClock)
available = double(client.NumBytesAvailable);
if available <= 0
    return;
end
incoming = read(client, available, 'uint8');
[frames, state.Buffer, dropped] = decodeDobotFeedbackFrames( ...
    state.Buffer, incoming);
state.DroppedByteCount = state.DroppedByteCount + uint64(dropped);
if isempty(frames)
    return;
end
frame = frames(end, :);
jointBytes = reshape(uint8(frame(433:480)), 1, []);
joints = typecast(jointBytes, 'double');
if numel(joints) ~= 6 || any(~isfinite(joints))
    error('scopeguide:servoJDiagnostic:InvalidJointFeedback', ...
        'The validated 30004 frame contained invalid joint values.');
end
state.JointDeg = double(joints(:).');
state.ModeCode = double(frame(25));
state.ModeName = robotModeName(state.ModeCode);
state.FrameCount = state.FrameCount + uint64(size(frames, 1));
state.LastFrameProgramSec = toc(programClock);
end

function requireServoCapableMode(state, maximumAgeSec, programClock)
ageSec = toc(programClock) - state.LastFrameProgramSec;
if ~isfinite(ageSec) || ageSec > maximumAgeSec
    error('scopeguide:servoJDiagnostic:FeedbackStale', ...
        'Robot feedback age %.3f s exceeds %.3f s.', ageSec, maximumAgeSec);
end
if ~ismember(state.ModeCode, [5, 7])
    error('scopeguide:servoJDiagnostic:RobotNotServoCapable', ...
        'Robot mode is %s (%d), expected ENABLE or RUNNING.', ...
        state.ModeName, state.ModeCode);
end
end

function name = robotModeName(code)
names = ["INIT", "BRAKE_OPEN", "POWER_OFF", "DISABLED", ...
    "ENABLE", "BACKDRIVE", "RUNNING", "RECORDING", "ERROR", ...
    "PAUSE", "JOG"];
if isfinite(code) && code >= 1 && code <= numel(names) && code == fix(code)
    name = names(code);
else
    name = "UNKNOWN";
end
end

function [commandTable, chunkTable, record, feedbackState] = ...
        runOneTrial(trialIndex, profile, readStrategy, rateHz, options, ...
        feedbackClient, feedbackState, programClock)
feedbackState = pollFeedback(feedbackClient, feedbackState, programClock);
requireServoCapableMode(feedbackState, ...
    options.MaximumFeedbackAgeSec, programClock);
targetDeg = feedbackState.JointDeg;
servoTimeSec = options.LegacyT;
if profile == "legacy_parameterized"
    servoTimeSec = 1 / rateHz;
end
command = scopeguide.diagnostics.formatServoJDiagnosticCommand( ...
    targetDeg, profile, servoTimeSec, ...
    options.LegacyLookaheadTime, options.LegacyGain);

moveClient = tcpclient(char(options.RobotIPAddress), ...
    options.MovePort, 'Timeout', 1);
moveCleanup = onCleanup(@() releaseTcpClient(moveClient));
moveRemainder = zeros(1, 0, 'uint8');
periodSec = 1 / rateHz;
scheduleSec = (0:periodSec: ...
    (options.DurationPerTrialSec - 0.5 * periodSec)).';
sampleCount = numel(scheduleSec);
data = initializeTrialData(sampleCount);
data.ScheduleSec = scheduleSec;
data.Command(:) = command;
data.TargetDeg = repmat(targetDeg, sampleCount, 1);
chunkRecords = repmat(emptyChunkRecord(), 0, 1);

trialClock = tic;
nextIndex = 1;
maximumDeviationDeg = 0;
safetyAbortReason = "";
while nextIndex <= sampleCount && strlength(safetyAbortReason) == 0
    feedbackState = pollFeedback( ...
        feedbackClient, feedbackState, programClock);
    [feedbackReason, deviationDeg] = checkTrialFeedback( ...
        feedbackState, targetDeg, options, programClock);
    maximumDeviationDeg = max(maximumDeviationDeg, deviationDeg);
    if strlength(feedbackReason) > 0
        safetyAbortReason = feedbackReason;
        break;
    end

    nowSec = toc(trialClock);
    if readStrategy == "continuous_framed"
        [frames, moveRemainder, chunk] = drainMoveClient( ...
            moveClient, moveRemainder, nowSec, "continuous_poll");
        [data, responseReason] = recordResponseFrames( ...
            data, frames, nowSec);
        chunkRecords = appendChunk(chunkRecords, chunk, trialIndex, ...
            data.SentCount - data.ReceivedCount);
        safetyAbortReason = firstReason( ...
            safetyAbortReason, responseReason);
    end

    if strlength(safetyAbortReason) == 0 && ...
            nowSec >= scheduleSec(nextIndex)
        data.SentCount = data.SentCount + 1;
        commandIndex = data.SentCount;
        data.SendTimeSec(commandIndex) = toc(trialClock);
        writeClock = tic;
        try
            write(moveClient, char(command), 'char');
        catch exception
            data.SendDurationSec(commandIndex) = toc(writeClock);
            safetyAbortReason = "MOVE_WRITE_FAILURE:" + ...
                string(exception.identifier) + ":" + string(exception.message);
            break;
        end
        data.SendDurationSec(commandIndex) = toc(writeClock);
        data.ImmediateBytesAvailable(commandIndex) = ...
            double(moveClient.NumBytesAvailable);

        shouldReadNow = readStrategy == "continuous_framed" || ...
            data.ImmediateBytesAvailable(commandIndex) > 0;
        if shouldReadNow
            [frames, moveRemainder, chunk] = drainMoveClient( ...
                moveClient, moveRemainder, toc(trialClock), "post_write");
            [data, responseReason] = recordResponseFrames( ...
                data, frames, toc(trialClock));
            chunkRecords = appendChunk(chunkRecords, chunk, trialIndex, ...
                data.SentCount - data.ReceivedCount);
            safetyAbortReason = firstReason( ...
                safetyAbortReason, responseReason);
        end
        data.PendingAfterSend(commandIndex) = ...
            data.SentCount - data.ReceivedCount;
        data.MaximumPending = max(data.MaximumPending, ...
            data.PendingAfterSend(commandIndex));
        nextIndex = nextIndex + 1;
    end
    pause(options.PollPeriodSec);
end

drainClock = tic;
while toc(drainClock) < options.ResponseDrainSec
    feedbackState = pollFeedback( ...
        feedbackClient, feedbackState, programClock);
    [feedbackReason, deviationDeg] = checkTrialFeedback( ...
        feedbackState, targetDeg, options, programClock);
    maximumDeviationDeg = max(maximumDeviationDeg, deviationDeg);
    safetyAbortReason = firstReason(safetyAbortReason, feedbackReason);

    [frames, moveRemainder, chunk] = drainMoveClient( ...
        moveClient, moveRemainder, toc(trialClock), "final_drain");
    [data, responseReason] = recordResponseFrames( ...
        data, frames, toc(trialClock));
    chunkRecords = appendChunk(chunkRecords, chunk, trialIndex, ...
        data.SentCount - data.ReceivedCount);
    safetyAbortReason = firstReason( ...
        safetyAbortReason, responseReason);
    if data.ReceivedCount >= data.SentCount && toc(drainClock) >= 0.25
        break;
    end
    pause(options.PollPeriodSec);
end

data.MaximumPending = max(data.MaximumPending, ...
    data.SentCount - data.ReceivedCount);
commandTable = trialDataTable(data, trialIndex, profile, ...
    readStrategy, rateHz);
chunkTable = chunkRecordTable(chunkRecords);
record = summarizeTrial(data, chunkRecords, trialIndex, profile, ...
    readStrategy, rateHz, targetDeg, maximumDeviationDeg, ...
    moveRemainder, safetyAbortReason);
end

function data = initializeTrialData(count)
data = struct();
data.ScheduleSec = nan(count, 1);
data.Command = strings(count, 1);
data.TargetDeg = nan(count, 6);
data.SendTimeSec = nan(count, 1);
data.SendDurationSec = nan(count, 1);
data.ImmediateBytesAvailable = zeros(count, 1);
data.PendingAfterSend = zeros(count, 1);
data.ResponseTimeSec = nan(count, 1);
data.ResponseLatencySec = nan(count, 1);
data.ResponseErrorID = nan(count, 1);
data.ResponseMatchesCommand = false(count, 1);
data.ResponseRaw = strings(count, 1);
data.SentCount = 0;
data.ReceivedCount = 0;
data.UnpairedResponseCount = 0;
data.MaximumPending = 0;
end

function [reason, deviationDeg] = checkTrialFeedback( ...
        state, targetDeg, options, programClock)
reason = "";
ageSec = toc(programClock) - state.LastFrameProgramSec;
deviationDeg = max(abs(wrappedDifferenceDeg(state.JointDeg, targetDeg)));
if ~isfinite(ageSec) || ageSec > options.MaximumFeedbackAgeSec
    reason = sprintf('FEEDBACK_STALE: age %.3f s', ageSec);
elseif ~ismember(state.ModeCode, [5, 7])
    reason = sprintf('ROBOT_MODE_%s_%d', state.ModeName, state.ModeCode);
elseif ~isfinite(deviationDeg) || ...
        deviationDeg > options.MaximumHoldDeviationDeg
    reason = sprintf('HOLD_DEVIATION: %.4f deg > %.4f deg', ...
        deviationDeg, options.MaximumHoldDeviationDeg);
end
reason = string(reason);
end

function difference = wrappedDifferenceDeg(a, b)
difference = mod(double(a) - double(b) + 180, 360) - 180;
end

function [frames, remainder, record] = drainMoveClient( ...
        client, remainder, timeSec, context)
available = double(client.NumBytesAvailable);
record = emptyChunkRecord();
if available <= 0
    frames = strings(0, 1);
    return;
end
incoming = read(client, available, 'uint8');
[frames, remainder] = ...
    scopeguide.diagnostics.extractDobotAsciiFrames(remainder, incoming);
record.Valid = true;
record.TimeSec = timeSec;
record.Context = string(context);
record.BytesRead = numel(incoming);
record.CompleteFrameCount = numel(frames);
record.RemainderBytes = numel(remainder);
end

function [data, reason] = recordResponseFrames(data, frames, readTimeSec)
reason = "";
for frameIndex = 1:numel(frames)
    if data.ReceivedCount >= data.SentCount
        data.UnpairedResponseCount = data.UnpairedResponseCount + 1;
        reason = firstReason(reason, ...
            "UNPAIRED_RESPONSE:" + strtrim(frames(frameIndex)));
        continue;
    end
    data.ReceivedCount = data.ReceivedCount + 1;
    index = data.ReceivedCount;
    response = strtrim(frames(frameIndex));
    data.ResponseTimeSec(index) = readTimeSec;
    data.ResponseLatencySec(index) = max(0, ...
        readTimeSec - data.SendTimeSec(index));
    data.ResponseRaw(index) = response;
    expectedSuffix = data.Command(index) + ";";
    data.ResponseMatchesCommand(index) = ...
        endsWith(response, expectedSuffix);
    try
        parsed = parseDobotV3CommandResponse(response);
        data.ResponseErrorID(index) = parsed.ErrorID;
    catch exception
        reason = firstReason(reason, ...
            "MALFORMED_RESPONSE:" + string(exception.message));
        continue;
    end
    if ~data.ResponseMatchesCommand(index)
        reason = firstReason(reason, ...
            "RESPONSE_COMMAND_MISMATCH:" + response);
    elseif data.ResponseErrorID(index) ~= 0
        reason = firstReason(reason, sprintf( ...
            'CONTROLLER_ERROR_ID_%d:%s', ...
            data.ResponseErrorID(index), response));
    end
end
end

function result = firstReason(existing, candidate)
result = string(existing);
if strlength(result) == 0 && strlength(string(candidate)) > 0
    result = string(candidate);
end
end

function records = appendChunk(records, record, trialIndex, pending)
if ~record.Valid
    return;
end
record.Trial = trialIndex;
record.PendingAfterRead = pending;
records(end + 1, 1) = record;
end

function record = emptyChunkRecord()
record = struct('Valid', false, 'Trial', 0, 'TimeSec', NaN, ...
    'Context', "", 'BytesRead', 0, 'CompleteFrameCount', 0, ...
    'RemainderBytes', 0, 'PendingAfterRead', 0);
end

function tableValue = chunkRecordTable(records)
if isempty(records)
    tableValue = table('Size', [0, 7], ...
        'VariableTypes', {'double', 'double', 'string', 'double', ...
        'double', 'double', 'double'}, ...
        'VariableNames', {'Trial', 'TimeSec', 'Context', 'BytesRead', ...
        'CompleteFrameCount', 'RemainderBytes', 'PendingAfterRead'});
    return;
end
records = rmfield(records, 'Valid');
tableValue = struct2table(records);
end

function tableValue = trialDataTable( ...
        data, trialIndex, profile, readStrategy, rateHz)
indices = (1:data.SentCount).';
sendTimes = data.SendTimeSec(indices);
sendInterval = [NaN; diff(sendTimes)];
tableValue = table( ...
    repmat(trialIndex, numel(indices), 1), ...
    repmat(string(profile), numel(indices), 1), ...
    repmat(string(readStrategy), numel(indices), 1), ...
    repmat(rateHz, numel(indices), 1), indices, ...
    data.ScheduleSec(indices), sendTimes, sendInterval, ...
    sendTimes - data.ScheduleSec(indices), ...
    data.SendDurationSec(indices), ...
    data.ImmediateBytesAvailable(indices), ...
    data.PendingAfterSend(indices), ...
    isfinite(data.ResponseTimeSec(indices)), ...
    data.ResponseTimeSec(indices), data.ResponseLatencySec(indices), ...
    data.ResponseErrorID(indices), ...
    data.ResponseMatchesCommand(indices), data.ResponseRaw(indices), ...
    data.Command(indices), ...
    data.TargetDeg(indices, 1), data.TargetDeg(indices, 2), ...
    data.TargetDeg(indices, 3), data.TargetDeg(indices, 4), ...
    data.TargetDeg(indices, 5), data.TargetDeg(indices, 6), ...
    'VariableNames', {'Trial', 'Profile', 'ReadStrategy', ...
    'RequestedRateHz', 'Sequence', 'ScheduledTimeSec', 'SendTimeSec', ...
    'SendIntervalSec', 'SendJitterSec', 'SendDurationSec', ...
    'ImmediateBytesAvailable', 'PendingAfterSend', ...
    'ResponseReceived', 'ResponseObservedTimeSec', ...
    'ResponseLatencySec', 'ResponseErrorID', ...
    'ResponseMatchesCommand', 'ResponseRaw', 'Command', ...
    'TargetJ1Deg', 'TargetJ2Deg', 'TargetJ3Deg', 'TargetJ4Deg', ...
    'TargetJ5Deg', 'TargetJ6Deg'});
end

function record = emptyTrialRecord()
record = struct('Trial', 0, 'Profile', "", 'ReadStrategy', "", ...
    'RequestedRateHz', NaN, 'Status', "not_run", ...
    'SentCount', 0, 'ReceivedCount', 0, 'MissingResponseCount', 0, ...
    'ControllerErrorCount', 0, 'MismatchedResponseCount', 0, ...
    'UnpairedResponseCount', 0, 'ActualSendRateHz', NaN, ...
    'ObservedResponseRateHz', NaN, 'SendIntervalP99Ms', NaN, ...
    'SendDurationP99Ms', NaN, 'ResponseLatencyP50Ms', NaN, ...
    'ResponseLatencyP99Ms', NaN, 'ResponseLatencyMaximumMs', NaN, ...
    'LatencySlopeMsPerCommand', NaN, 'MaximumPendingResponses', 0, ...
    'PendingResponsesAfterDrain', 0, 'ImmediateReadNonemptyCount', 0, ...
    'ReadChunkCount', 0, 'MultiFrameReadCount', 0, ...
    'PartialFrameReadCount', 0, 'TrailingPartialBytes', 0, ...
    'MaximumHoldDeviationDeg', NaN, 'TargetJointDeg', nan(1, 6), ...
    'SafetyAbortReason', "");
end

function record = summarizeTrial(data, chunkRecords, trialIndex, ...
        profile, readStrategy, rateHz, targetDeg, maximumDeviationDeg, ...
        remainder, safetyAbortReason)
record = emptyTrialRecord();
record.Trial = trialIndex;
record.Profile = string(profile);
record.ReadStrategy = string(readStrategy);
record.RequestedRateHz = rateHz;
record.SentCount = data.SentCount;
record.ReceivedCount = data.ReceivedCount;
record.MissingResponseCount = data.SentCount - data.ReceivedCount;
received = 1:data.ReceivedCount;
record.ControllerErrorCount = nnz( ...
    isfinite(data.ResponseErrorID(received)) & ...
    data.ResponseErrorID(received) ~= 0);
record.MismatchedResponseCount = nnz( ...
    ~data.ResponseMatchesCommand(received));
record.UnpairedResponseCount = data.UnpairedResponseCount;
sentTimes = data.SendTimeSec(1:data.SentCount);
if numel(sentTimes) >= 2
    record.ActualSendRateHz = ...
        (numel(sentTimes) - 1) / (sentTimes(end) - sentTimes(1));
    record.SendIntervalP99Ms = 1000 * percentile(diff(sentTimes), 99);
end
record.SendDurationP99Ms = 1000 * percentile( ...
    data.SendDurationSec(1:data.SentCount), 99);
responseTimes = data.ResponseTimeSec(received);
if numel(responseTimes) >= 2 && responseTimes(end) > responseTimes(1)
    record.ObservedResponseRateHz = ...
        (numel(responseTimes) - 1) / ...
        (responseTimes(end) - responseTimes(1));
end
latencies = data.ResponseLatencySec(received);
record.ResponseLatencyP50Ms = 1000 * percentile(latencies, 50);
record.ResponseLatencyP99Ms = 1000 * percentile(latencies, 99);
record.ResponseLatencyMaximumMs = 1000 * maxFinite(latencies);
if numel(received) >= 2
    coefficients = polyfit(double(received(:)), latencies(:), 1);
    record.LatencySlopeMsPerCommand = 1000 * coefficients(1);
end
record.MaximumPendingResponses = data.MaximumPending;
record.PendingResponsesAfterDrain = ...
    data.SentCount - data.ReceivedCount;
record.ImmediateReadNonemptyCount = nnz( ...
    data.ImmediateBytesAvailable(1:data.SentCount) > 0);
record.ReadChunkCount = numel(chunkRecords);
if ~isempty(chunkRecords)
    record.MultiFrameReadCount = nnz( ...
        [chunkRecords.CompleteFrameCount] > 1);
    record.PartialFrameReadCount = nnz( ...
        [chunkRecords.RemainderBytes] > 0);
end
record.TrailingPartialBytes = numel(remainder);
record.MaximumHoldDeviationDeg = maximumDeviationDeg;
record.TargetJointDeg = targetDeg;
record.SafetyAbortReason = string(safetyAbortReason);
if strlength(record.SafetyAbortReason) > 0
    record.Status = "aborted";
elseif record.MissingResponseCount > 0
    record.Status = "completed_with_missing_replies";
else
    record.Status = "completed_all_replies";
end
end

function value = maxFinite(values)
values = double(values(isfinite(values)));
if isempty(values)
    value = NaN;
else
    value = max(values);
end
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

function printTrialResult(record)
fprintf(['  sent/received=%d/%d; send=%.2f Hz; reply=%.2f Hz; ' ...
    'latency P99/max=%.1f/%.1f ms; slope=%+.2f ms/cmd; ' ...
    'max/end pending=%d/%d; hold deviation=%.4f deg; status=%s.\n'], ...
    record.SentCount, record.ReceivedCount, record.ActualSendRateHz, ...
    record.ObservedResponseRateHz, record.ResponseLatencyP99Ms, ...
    record.ResponseLatencyMaximumMs, record.LatencySlopeMsPerCommand, ...
    record.MaximumPendingResponses, record.PendingResponsesAfterDrain, ...
    record.MaximumHoldDeviationDeg, record.Status);
end

function state = waitWithFeedback(durationSec, client, state, ...
        programClock, pollPeriodSec)
waitClock = tic;
while toc(waitClock) < durationSec
    state = pollFeedback(client, state, programClock);
    pause(pollPeriodSec);
end
end

function tableValue = concatenateTables(values)
nonempty = ~cellfun(@isempty, values);
if ~any(nonempty)
    tableValue = table();
else
    tableValue = vertcat(values{nonempty});
end
end

function summary = buildSummary(trials, commands, readChunks, options, ...
        feedbackState, plannedTrialCount, completedTrialCount)
summary = struct();
summary.Diagnostic = "Dobot 30003 ServoJ communication isolation";
summary.Status = "COMPLETED";
if completedTrialCount < plannedTrialCount || ...
        any(strlength(trials.SafetyAbortReason) > 0)
    summary.Status = "SAFETY_ABORTED";
elseif any(trials.MissingResponseCount > 0)
    summary.Status = "COMPLETED_WITH_REPLY_BACKLOG";
end
summary.LocalTime = string(datetime('now'));
summary.RobotIPAddress = options.RobotIPAddress;
summary.MovePort = options.MovePort;
summary.FeedbackPort = options.FeedbackPort;
summary.PlannedTrialCount = plannedTrialCount;
summary.CompletedTrialCount = completedTrialCount;
summary.TotalCommandCount = height(commands);
summary.TotalReadChunkCount = height(readChunks);
summary.FinalFeedbackMode = feedbackState.ModeName;
summary.FinalJointDeg = feedbackState.JointDeg;
summary.FeedbackFrameCount = double(feedbackState.FrameCount);
summary.FeedbackDroppedByteCount = ...
    double(feedbackState.DroppedByteCount);
summary.LegacyImplementation = struct( ...
    'Project', "dobot_HW_control2", ...
    'ControlPeriodSec', 0.04, ...
    'ControlRateHz', 25, ...
    'ServoJIncludesDynamicParameters', true, ...
    'ServoJT', options.LegacyT, ...
    'ServoJLookaheadTime', options.LegacyLookaheadTime, ...
    'ServoJGain', options.LegacyGain, ...
    'WaitsForEachResponse', false, ...
    'ReadsOnlyImmediatelyAfterWrite', true, ...
    'AcceptsEmptyResponse', true, ...
    'ParsesControllerErrorID', false);
summary.CurrentScopeGuideImplementation = struct( ...
    'ServoJIncludesDynamicParameters', true, ...
    'ServoJTSource', "runtime.NominalDtSec = 1 / ControlRateHz", ...
    'ServoJLookaheadTime', 60, ...
    'ServoJGain', 400, ...
    'WaitsForEachResponse', false, ...
    'ContinuouslyDrainsReplies', true, ...
    'ResponseExpected', false, ...
    'ParsesOptionalRepliesForErrorID', true, ...
    'MatchesRepliesToCommands', false, ...
    'PerCommandReplyTimeoutApplied', false);
summary.Trials = table2struct(trials);
summary.Findings = interpretFindings(trials);
end

function findings = interpretFindings(trials)
findings = strings(0, 1);
findings(end + 1, 1) = [ ...
    "旧项目没有证明每条 ServoJ 都及时收到回复：空回复被直接接受，" ...
    "且 ErrorID 解析被注释掉。"];
if any(trials.MissingResponseCount > 0)
    findings(end + 1, 1) = [ ...
        "至少一组在额外 drain 后仍缺少回复，说明不是 MATLAB 同步等待造成的，" ...
        "而是 30003 回复吞吐或控制器命令处理存在积压。"];
end
if any(trials.LatencySlopeMsPerCommand > 2)
    findings(end + 1, 1) = ...
        "回复延迟随命令序号增长，存在可测的队列积压；单纯增大单条超时只能延后故障。";
end

rates = unique(trials.RequestedRateHz, 'stable');
legacyBetter = false;
minimalBetter = false;
for rate = rates.'
    currentRows = trials.Profile == "scopeguide_minimal" & ...
        trials.ReadStrategy == "continuous_framed" & ...
        trials.RequestedRateHz == rate;
    legacyRows = trials.Profile == "legacy_parameterized" & ...
        trials.ReadStrategy == "continuous_framed" & ...
        trials.RequestedRateHz == rate;
    if nnz(currentRows) == 1 && nnz(legacyRows) == 1
        current = trials(currentRows, :);
        legacy = trials(legacyRows, :);
        legacyBetter = legacyBetter || ...
            legacy.MissingResponseCount + 2 < ...
            current.MissingResponseCount || ...
            legacy.LatencySlopeMsPerCommand + 2 < ...
            current.LatencySlopeMsPerCommand;
        minimalBetter = minimalBetter || ...
            current.MissingResponseCount + 2 < ...
            legacy.MissingResponseCount;
    end
end
if legacyBetter && ~minimalBetter
    findings(end + 1, 1) = [ ...
        "带 t/lookahead_time/gain 的旧命令格式表现明显更好；" ...
        "命令参数差异很可能是 ScopeGuide 当前积压的主要因素。"];
elseif ~legacyBetter && any(trials.Profile == "legacy_parameterized")
    findings(end + 1, 1) = ...
        "旧命令格式没有消除积压；旧项目之所以看似正常，更可能是因为它忽略了空回复和迟到回复。";
end

if any(trials.ControllerErrorCount > 0)
    findings(end + 1, 1) = ...
        "控制器返回了非零 ErrorID；应先根据 trials.csv 的原始回复修正命令。";
end
if all(trials.MissingResponseCount == 0) && ...
        all(trials.ControllerErrorCount == 0) && ...
        all(trials.MismatchedResponseCount == 0)
    findings(end + 1, 1) = [ ...
        "所有回复最终都完整匹配且 ErrorID=0；问题属于回复时延预算，" ...
        "不是命令拒绝或 TCP 拆包解析错误。"];
end
end

function outputDirectory = makeOutputDirectory(projectRoot)
stamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
outputDirectory = fullfile(projectRoot, 'results', ...
    "servoj_communication_diagnostic_" + stamp);
[created, message] = mkdir(outputDirectory);
if ~created
    error('scopeguide:servoJDiagnostic:CannotCreateResults', ...
        'Cannot create %s: %s', outputDirectory, message);
end
end

function figureHandle = renderDiagnosticFigure(commands, trials)
figureHandle = figure('Name', 'ServoJ 30003 Communication Diagnostic', ...
    'Color', 'w', 'Visible', 'off');
layout = tiledlayout(2, 2, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
title(layout, 'Dobot ServoJ 30003 通讯隔离诊断');
colors = lines(max(1, height(trials)));

latencyAxes = nexttile;
hold on;
for index = 1:height(trials)
    rows = commands.Trial == trials.Trial(index) & ...
        commands.ResponseReceived;
    plot(commands.Sequence(rows), ...
        1000 * commands.ResponseLatencySec(rows), '.-', ...
        'Color', colors(index, :), 'DisplayName', trialLabel(trials, index));
end
yline(250, '--r', '原 Stage 8 硬超时 250 ms');
xlabel('Command sequence');
ylabel('Observed reply latency [ms]');
grid on;
title('逐条回复延迟');

nexttile;
hold on;
for index = 1:height(trials)
    rows = commands.Trial == trials.Trial(index);
    plot(commands.Sequence(rows), commands.PendingAfterSend(rows), ...
        '.-', 'Color', colors(index, :), ...
        'DisplayName', trialLabel(trials, index));
end
xlabel('Command sequence');
ylabel('Pending replies');
grid on;
title('回复积压');

nexttile;
hold on;
for index = 1:height(trials)
    rows = commands.Trial == trials.Trial(index) & ...
        isfinite(commands.SendIntervalSec);
    plot(commands.Sequence(rows), ...
        1000 * commands.SendIntervalSec(rows), '.-', ...
        'Color', colors(index, :), 'DisplayName', trialLabel(trials, index));
end
xlabel('Command sequence');
ylabel('Send interval [ms]');
grid on;
title('主机实际发送周期');

nexttile;
requested = trials.RequestedRateHz;
actual = trials.ActualSendRateHz;
reply = trials.ObservedResponseRateHz;
bar([requested, actual, reply]);
xticks(1:height(trials));
xticklabels(compose('T%d', trials.Trial));
xtickangle(30);
ylabel('Rate [Hz]');
grid on;
legend({'Requested', 'Actual send', 'Observed reply'}, ...
    'Location', 'best');
title('发送与回复吞吐率');

legend(latencyAxes, 'Location', 'bestoutside');
end

function label = trialLabel(trials, index)
label = sprintf('T%d %s/%s %.0fHz', trials.Trial(index), ...
    abbreviatedProfile(trials.Profile(index)), ...
    abbreviatedRead(trials.ReadStrategy(index)), ...
    trials.RequestedRateHz(index));
end

function value = abbreviatedProfile(profile)
if profile == "scopeguide_minimal"
    value = "minimal";
else
    value = "legacy-param";
end
end

function value = abbreviatedRead(strategy)
if strategy == "continuous_framed"
    value = "continuous";
else
    value = "legacy-read";
end
end

function writeJson(path, value)
try
    encoded = jsonencode(value, 'PrettyPrint', true);
catch
    encoded = jsonencode(value);
end
[fileId, message] = fopen(path, 'w');
if fileId < 0
    error('scopeguide:servoJDiagnostic:CannotWriteJson', ...
        'Cannot open %s: %s', path, message);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '%s\n', encoded);
clear cleanup;
end

function releaseTcpClient(client)
% Holding the object until this cleanup returns makes Ctrl+C deterministic.
try
    configureCallback(client, 'off');
catch
end
end
