function [previousSequence, consecutiveMovingFrames, isNewFeedback] = ...
        updateUniqueMotionCounter(previousSequence, ...
        consecutiveMovingFrames, currentSequence, moving)
%UPDATEUNIQUEMOTIONCOUNTER Count motion only on new robot feedback frames.

previousSequence = uint64(previousSequence);
currentSequence = uint64(currentSequence);
consecutiveMovingFrames = double(consecutiveMovingFrames);
if ~isscalar(previousSequence) || ~isscalar(currentSequence) || ...
        ~isscalar(consecutiveMovingFrames) || ...
        ~isfinite(consecutiveMovingFrames) || consecutiveMovingFrames < 0 || ...
        ~isscalar(moving)
    error('scopeguide:diagnostics:InvalidMotionCounterInput', ...
        'Motion counter inputs must be finite scalars.');
end

isNewFeedback = currentSequence ~= previousSequence;
if ~isNewFeedback
    return;
end
previousSequence = currentSequence;
if logical(moving)
    consecutiveMovingFrames = consecutiveMovingFrames + 1;
else
    consecutiveMovingFrames = 0;
end
end
