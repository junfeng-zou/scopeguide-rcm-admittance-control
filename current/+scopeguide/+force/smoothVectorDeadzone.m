function [output, active, diagnostics] = smoothVectorDeadzone( ...
    input, enterThreshold, releaseThreshold, wasActive)
%SMOOTHVECTORDEADZONE Direction-preserving continuous radial deadzone.
% The output is continuous at enterThreshold. The hysteresis state is kept
% separately for diagnostics/release logic and never creates an output jump.

if nargin < 4
    wasActive = false;
end
vector = double(input(:));
if isempty(vector) || any(~isfinite(vector))
    error('scopeguide:force:InvalidDeadzoneInput', ...
        'input must be a nonempty finite vector.');
end
if ~isscalar(enterThreshold) || ~isfinite(enterThreshold) || ...
        enterThreshold <= 0
    error('scopeguide:force:InvalidDeadzoneThreshold', ...
        'enterThreshold must be finite and positive.');
end
if ~isscalar(releaseThreshold) || ~isfinite(releaseThreshold) || ...
        releaseThreshold < 0 || releaseThreshold > enterThreshold
    error('scopeguide:force:InvalidDeadzoneThreshold', ...
        'releaseThreshold must lie in [0, enterThreshold].');
end
if ~islogical(wasActive) || ~isscalar(wasActive)
    error('scopeguide:force:InvalidDeadzoneState', ...
        'wasActive must be a logical scalar.');
end

magnitude = norm(vector);
if wasActive
    active = magnitude > releaseThreshold;
else
    active = magnitude > enterThreshold;
end

effectiveMagnitude = max(magnitude - enterThreshold, 0);
if magnitude > 0
    output = vector * (effectiveMagnitude / magnitude);
else
    output = zeros(size(vector));
end

diagnostics = struct();
diagnostics.InputNorm = magnitude;
diagnostics.OutputNorm = norm(output);
diagnostics.EnterThreshold = enterThreshold;
diagnostics.ReleaseThreshold = releaseThreshold;
diagnostics.Active = active;
end
