function [frames, remainder, droppedByteCount] = ...
        decodeDobotFeedbackFrames(remainder, incoming)
%DECODEDOBOTFEEDBACKFRAMES Reassemble and validate Dobot 30004 frames.
%
% TCP is a byte stream: one callback may receive a partial frame, one frame,
% or several frames. This function preserves trailing bytes and only returns
% complete 1440-byte packets whose MessageSize and TestValue are valid.

frameSize = 1440;
messageSizeBytes = uint8([160, 5]); % uint16(1440), little-endian
testValueBytes = uint8([239, 205, 171, 137, 103, 69, 35, 1]);

stream = [reshape(uint8(remainder), 1, []), ...
          reshape(uint8(incoming), 1, [])];
frames = zeros(0, frameSize, 'uint8');
droppedByteCount = 0;

while numel(stream) >= frameSize
    if isFrameAt(stream, 1, frameSize, messageSizeBytes, testValueBytes)
        frames(end + 1, :) = stream(1:frameSize); %#ok<AGROW>
        stream(1:frameSize) = [];
        continue;
    end

    maximumStart = numel(stream) - frameSize + 1;
    nextStart = [];
    for candidate = 2:maximumStart
        if isFrameAt(stream, candidate, frameSize, ...
                messageSizeBytes, testValueBytes)
            nextStart = candidate;
            break;
        end
    end
    if isempty(nextStart)
        % Retain enough trailing data to complete a possible partial frame.
        discardCount = numel(stream) - frameSize + 1;
        droppedByteCount = droppedByteCount + discardCount;
        stream(1:discardCount) = [];
        break;
    end
    droppedByteCount = droppedByteCount + nextStart - 1;
    stream(1:nextStart - 1) = [];
end

remainder = stream;
end

function valid = isFrameAt(stream, startIndex, frameSize, ...
        messageSizeBytes, testValueBytes)
if startIndex + frameSize - 1 > numel(stream)
    valid = false;
    return;
end
valid = isequal(stream(startIndex:startIndex + 1), messageSizeBytes) && ...
    isequal(stream(startIndex + 48:startIndex + 55), testValueBytes);
end
