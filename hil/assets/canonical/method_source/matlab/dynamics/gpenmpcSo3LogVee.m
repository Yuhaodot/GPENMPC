function vector = gpenmpcSo3LogVee(rotation)
%GPENMPCSO3LOGVEE Principal SO(3) logarithm as a three-vector.

r = double(rotation);
if ~isequal(size(r), [3, 3]) || any(~isfinite(r), "all")
    error("gpenmpcSo3LogVee:Input", ...
        "Rotation must be a finite 3 by 3 matrix.");
end
cosine = min(max((trace(r) - 1.0) ./ 2.0, -1.0), 1.0);
angle = acos(cosine);
skewVector = 0.5 .* gpenmpcVee(r - r.');
if angle < 1.0e-8
    vector = skewVector;
elseif pi - angle < 1.0e-6
    [eigenvectors, eigenvalues] = eig(0.5 .* (r + eye(3)));
    [~, index] = max(real(diag(eigenvalues)));
    axis = real(eigenvectors(:, index));
    axis = axis ./ max(norm(axis), 1.0e-15);
    if dot(axis, skewVector) < 0.0
        axis = -axis;
    end
    vector = angle .* axis;
else
    vector = angle ./ sin(angle) .* skewVector;
end
end
