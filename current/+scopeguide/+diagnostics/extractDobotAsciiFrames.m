function [frames, remainder] = extractDobotAsciiFrames(remainder, incoming)
%EXTRACTDOBOTASCIIFRAMES Reassemble semicolon-terminated TCP replies.
% TCP preserves byte order, not application-message boundaries. One read
% may contain a partial reply, one reply, or several concatenated replies.

stream = [reshape(uint8(remainder), 1, []), ...
          reshape(uint8(incoming), 1, [])];
frames = strings(0, 1);
while true
    terminator = find(stream == uint8(';'), 1, 'first');
    if isempty(terminator)
        break;
    end
    frames(end + 1, 1) = string(char(stream(1:terminator))); %#ok<AGROW>
    stream(1:terminator) = [];
end
remainder = stream;
end
