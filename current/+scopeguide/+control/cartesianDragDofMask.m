function mask = cartesianDragDofMask(mode)
%CARTESIANDRAGDOFMASK Active local twist axes [vx vy vz wx wy wz].

switch string(mode)
    case "hold"
        mask = false(6, 1);
    case "translation_z"
        mask = logical([0; 0; 1; 0; 0; 0]);
    case "translation_xyz"
        mask = logical([1; 1; 1; 0; 0; 0]);
    case "rotation_x"
        mask = logical([0; 0; 0; 1; 0; 0]);
    case "rotation_y"
        mask = logical([0; 0; 0; 0; 1; 0]);
    case "rotation_z"
        mask = logical([0; 0; 0; 0; 0; 1]);
    case "rotation_xyz"
        mask = logical([0; 0; 0; 1; 1; 1]);
    case "full_6dof"
        mask = true(6, 1);
    otherwise
        error('scopeguide:cartesianDrag:UnknownDofMode', ...
            'Unknown Cartesian drag DOF mode: %s.', string(mode));
end
end
