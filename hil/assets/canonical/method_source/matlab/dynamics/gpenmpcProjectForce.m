function [force, mismatch] = gpenmpcProjectForce(rawForce, totalThrustUpperN, maximumTiltRad)
%GPENMPCPROJECTFORCE Apply the common force-norm and tilt authority limits.

raw = double(rawForce(:));
force = raw;
if norm(force) > totalThrustUpperN
    force = force .* (totalThrustUpperN ./ norm(force));
end
horizontal = norm(force(1:2));
vertical = max(force(3), 1e-9);
maximumHorizontal = tan(maximumTiltRad) .* vertical;
if horizontal > maximumHorizontal && horizontal > 0 %#ok<BDSCI>
    force(1:2) = force(1:2) .* (maximumHorizontal ./ horizontal);
end
mismatch = norm(raw - force);
end
