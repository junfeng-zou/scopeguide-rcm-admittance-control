function reply = parseDobotV3CommandResponse(response)
%PARSEDOBOTV3COMMANDRESPONSE Parse a Nova/CR V3 ASCII command reply.
% V3 format: ErrorID,{values},Command(parameters);

raw = char(string(response));
tokens = regexp(raw, '^\s*(-?\d+)\s*,', 'tokens', 'once');
if isempty(tokens) || ~contains(raw, ';')
    error('scopeguide:dobotV3:MalformedResponse', ...
        'Malformed Nova/CR V3 command response: %s', strtrim(raw));
end

errorID = str2double(tokens{1});
if ~isfinite(errorID) || errorID ~= fix(errorID)
    error('scopeguide:dobotV3:MalformedResponse', ...
        'Invalid ErrorID in Nova/CR V3 response: %s', strtrim(raw));
end

description = describeError(errorID);
reply = struct();
reply.ErrorID = errorID;
reply.Description = description;
reply.RawResponse = raw;
reply.Success = errorID == 0;
end

function description = describeError(errorID)
switch errorID
    case 0
        description = "Command accepted successfully.";
    case -1
        description = "Command reception or execution failed.";
    case -10000
        description = "Command does not exist.";
    case -20000
        description = "Incorrect number of parameters.";
    otherwise
        if errorID <= -30001 && errorID > -40000
            parameterIndex = abs(errorID) - 30000;
            description = string(sprintf( ...
                'Parameter %d has an incorrect type.', parameterIndex));
        elseif errorID <= -40001 && errorID > -50000
            parameterIndex = abs(errorID) - 40000;
            description = string(sprintf( ...
                'Parameter %d is outside its valid range.', parameterIndex));
        else
            description = "Unknown Nova/CR V3 command error.";
        end
end
end
