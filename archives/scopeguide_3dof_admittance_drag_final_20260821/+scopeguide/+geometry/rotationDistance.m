function angle = rotationDistance(rotationA, rotationB)
%ROTATIONDISTANCE Geodesic SO(3) distance in radians.

angle = norm(scopeguide.geometry.rotationLogVector( ...
    rotationA * rotationB'));
end
