function [signals, renamed] = normalizeDeadzoneSignalColumnNames(signals)
%NORMALIZEDEADZONESIGNALCOLUMNNAMES Canonicalize legacy axis suffix case.
% Early deadzone-identification checkpoints used ExternalFX/SlowFZ style
% names. The canonical schema uses ExternalFx/SlowFz. Values and row order
% are not modified.

arguments
    signals table
end

renamed = strings(0, 2);
prefixes = ["Raw", "External", "Fast", "Slow", "Control"];
quantities = ["F", "T"];
axesLower = ["x", "y", "z"];
axesUpper = ["X", "Y", "Z"];

for prefix = prefixes
    for quantity = quantities
        for axisIndex = 1:3
            legacy = prefix + quantity + axesUpper(axisIndex);
            canonical = prefix + quantity + axesLower(axisIndex);
            [signals, renamed] = renameIfPresent( ...
                signals, renamed, legacy, canonical);
        end
    end
end
for axisIndex = 1:3
    legacy = "DiagnosticT" + axesUpper(axisIndex);
    canonical = "DiagnosticT" + axesLower(axisIndex);
    [signals, renamed] = renameIfPresent( ...
        signals, renamed, legacy, canonical);
end
end

function [signals, renamed] = renameIfPresent( ...
        signals, renamed, legacy, canonical)
names = string(signals.Properties.VariableNames);
hasLegacy = ismember(legacy, names);
hasCanonical = ismember(canonical, names);
if hasLegacy && hasCanonical
    error('scopeguide:force:AmbiguousDeadzoneSignalColumns', ...
        'Both legacy %s and canonical %s columns are present.', ...
        legacy, canonical);
elseif hasLegacy
    legacyIndex = find(names == legacy, 1);
    signals.Properties.VariableNames{legacyIndex} = char(canonical);
    renamed(end + 1, :) = [legacy, canonical];
end
end
