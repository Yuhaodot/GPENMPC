function clipped = gpenmpcClipNorm(value, maximumNorm)
%GPENMPCCLIPNORM Radially clip a vector without changing its direction.

clipped = double(value(:));
valueNorm = norm(clipped);
if valueNorm > maximumNorm && valueNorm > 0
    clipped = clipped .* (maximumNorm ./ valueNorm);
end
end
