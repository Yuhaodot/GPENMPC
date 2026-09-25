function matrix = gpenmpcQuaternionDerivativeMatrix(bodyRateRadS)
%GPENMPCQUATERNIONDERIVATIVEMATRIX Scalar-first quaternion rate matrix.

omega = double(bodyRateRadS(:));
p = omega(1); q = omega(2); r = omega(3);
matrix = [0, -p, -q, -r; ...
    p, 0, r, -q; ...
    q, -r, 0, p; ...
    r, q, -p, 0];
end
