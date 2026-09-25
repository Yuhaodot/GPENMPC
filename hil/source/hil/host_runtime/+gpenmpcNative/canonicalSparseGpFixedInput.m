function output18 = canonicalSparseGpFixedInput(model, input17)
%#codegen
% Fixed-size entry for the canonical sparse-GP predictor.
assert(isequal(size(input17),[1 17]));
p = gpenmpcSparseGpPredict(model,input17);
output18 = [p.mean_mps2,p.raw_std_mps2,p.calibrated_half_width_mps2, ...
    p.latent_variance_standardized,p.support_distance,p.trust, ...
    double(p.hard_invalid),p.applied_mean_f_mps2];
end
