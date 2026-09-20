function report = summarizeRobotStationarity(feedbackSequence, ...
        jointSpeedDegSec, tcpSpeed, varargin)
%SUMMARIZEROBOTSTATIONARITY Classify motion on unique Dobot feedback frames.
%
% HEX-H may be polled faster than Dobot publishes 30004 feedback. Repeated
% snapshots therefore must not be counted as independent robot samples.
% Isolated one-frame velocity spikes are reported but are not treated as
% confirmed physical motion unless the configured consecutive-frame count
% is reached.

parser = inputParser;
addParameter(parser, 'MaximumJointSpeedDegSec', 0.10, @positiveScalar);
addParameter(parser, 'MaximumTcpTranslationSpeed', 0.20, @positiveScalar);
addParameter(parser, 'MaximumTcpRotationSpeed', 0.20, @positiveScalar);
addParameter(parser, 'MinimumConsecutiveMovingFrames', 2, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 1 && x == round(x));
parse(parser, varargin{:});
opts = parser.Results;

sequence = uint64(feedbackSequence(:));
jointSpeed = double(jointSpeedDegSec);
tcp = double(tcpSpeed);
sampleCount = numel(sequence);
if sampleCount == 0 || size(jointSpeed, 1) ~= sampleCount || ...
        size(tcp, 1) ~= sampleCount || size(jointSpeed, 2) ~= 6 || ...
        size(tcp, 2) ~= 6 || any(~isfinite(jointSpeed), 'all') || ...
        any(~isfinite(tcp), 'all')
    error('calibration:InvalidStationarityInput', ...
        'Expected finite matching N-by-6 speed arrays and N feedback sequences.');
end

% Consecutive host polls can contain the same 125 Hz robot feedback packet.
uniqueRows = [true; sequence(2:end) ~= sequence(1:end-1)];
uniqueSequence = sequence(uniqueRows);
uniqueJointSpeed = jointSpeed(uniqueRows, :);
uniqueTcpSpeed = tcp(uniqueRows, :);

jointMaximum = max(abs(uniqueJointSpeed), [], 2);
tcpTranslationMaximum = max(abs(uniqueTcpSpeed(:, 1:3)), [], 2);
tcpRotationMaximum = max(abs(uniqueTcpSpeed(:, 4:6)), [], 2);
jointTrigger = jointMaximum > opts.MaximumJointSpeedDegSec;
translationTrigger = ...
    tcpTranslationMaximum > opts.MaximumTcpTranslationSpeed;
rotationTrigger = tcpRotationMaximum > opts.MaximumTcpRotationSpeed;
rawMoving = jointTrigger | translationTrigger | rotationTrigger;
confirmedMoving = confirmRuns(rawMoving, ...
    opts.MinimumConsecutiveMovingFrames);

report = struct();
report.hostSampleCount = sampleCount;
report.uniqueFeedbackCount = numel(uniqueSequence);
report.firstFeedbackSequence = uniqueSequence(1);
report.lastFeedbackSequence = uniqueSequence(end);
report.rawMovingFrameCount = sum(rawMoving);
report.confirmedMovingFrameCount = sum(confirmedMoving);
report.ignoredIsolatedFrameCount = sum(rawMoving & ~confirmedMoving);
report.rawMovingRatio = mean(rawMoving);
report.confirmedMovingRatio = mean(confirmedMoving);
report.jointTriggerCount = sum(jointTrigger);
report.translationTriggerCount = sum(translationTrigger);
report.rotationTriggerCount = sum(rotationTrigger);
report.maximumJointSpeedDegSec = max(jointMaximum);
report.maximumTcpTranslationSpeed = max(tcpTranslationMaximum);
report.maximumTcpRotationSpeed = max(tcpRotationMaximum);
report.minimumConsecutiveMovingFrames = ...
    opts.MinimumConsecutiveMovingFrames;
end

function confirmed = confirmRuns(rawMoving, minimumLength)
confirmed = false(size(rawMoving));
runStart = [];
for index = 1:numel(rawMoving) + 1
    inRun = index <= numel(rawMoving) && rawMoving(index);
    if inRun && isempty(runStart)
        runStart = index;
    elseif ~inRun && ~isempty(runStart)
        if index - runStart >= minimumLength
            confirmed(runStart:index - 1) = true;
        end
        runStart = [];
    end
end
end

function valid = positiveScalar(value)
valid = isnumeric(value) && isscalar(value) && isfinite(value) && value > 0;
end
