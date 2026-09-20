function mapped = mapForceToRcmGeneralizedEffort( ...
        forceToolN, TBaseEndoscope, pRcmBaseM, cfg)
%MAPFORCETORCMGENERALIZEDEFFORT Map tool force to pivot/pivot/insertion.
% Sensor moments are deliberately not accepted by this Stage 5 interface.

forceTool = validateForce(forceToolN);
basis = scopeguide.geometry.computeRcmMotionBasis( ...
    TBaseEndoscope, pRcmBaseM, cfg);
rotation = double(TBaseEndoscope(1:3, 1:3));
forceBase = rotation * forceTool;
generalizedEffort = basis.LinearVelocityBasisBase' * forceBase;

mapped = struct();
mapped.ForceToolN = forceTool;
mapped.ForceBaseN = forceBase;
mapped.GeneralizedEffort = generalizedEffort;
mapped.GeneralizedEffortWithRoll = [generalizedEffort; 0];
mapped.Basis = basis;
mapped.MomentUsedForControl = false;
mapped.RollEffort = 0;
end

function force = validateForce(value)
if ~isnumeric(value) || numel(value) ~= 3 || ...
        any(~isfinite(value(:)))
    error('scopeguide:control:InvalidToolForce', ...
        'forceToolN must contain three finite force values.');
end
force = double(value(:));
end
