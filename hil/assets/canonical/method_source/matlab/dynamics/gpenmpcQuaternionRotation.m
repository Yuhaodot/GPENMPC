function rotation = gpenmpcQuaternionRotation(quaternionWxyz)
%GPENMPCQUATERNIONROTATION Convert a scalar-first unit quaternion to SO(3).

q = double(quaternionWxyz(:));
q = q ./ max(norm(q), 1e-15);
w = q(1); x = q(2); y = q(3); z = q(4);
rotation = [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w); ...
    2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w); ...
    2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)];
end
