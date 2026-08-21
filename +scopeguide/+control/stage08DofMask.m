function mask = stage08DofMask(mode)
%STAGE08DOFMASK Generalized order: pivot1, pivot2, insertion.

switch string(mode)
    case "hold"
        mask = logical([0; 0; 0]);
    case "insertion_only"
        mask = logical([0; 0; 1]);
    case "pivot1_only"
        mask = logical([1; 0; 0]);
    case "pivot2_only"
        mask = logical([0; 1; 0]);
    case "dual_pivot"
        mask = logical([1; 1; 0]);
    case "full_3dof"
        mask = logical([1; 1; 1]);
    otherwise
        error('scopeguide:stage08:InvalidDofMode', ...
            'Unsupported Stage 8 DOF mode: %s.', string(mode));
end
end
