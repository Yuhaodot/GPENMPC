function prepared = gpenmpcPrepareSparseGpModel(model)
%GPENMPCPREPARESPARSEGPMODEL Cache immutable online GP kernel quantities.
%
% Appends derived factors while preserving learned normalization, kernel,
% posterior and calibration values.

arguments
    model (1,1) struct
end
prepared = model;
prepared.prepared_scaled_inducing = ...
    model.inducing_standardized ./ model.lengthscale;
prepared.prepared_scaled_inducing_squared_norm = ...
    sum(prepared.prepared_scaled_inducing.^2, 2);
prepared.prepared_inducing_standardized_squared_norm = ...
    sum(model.inducing_standardized.^2, 2);
prepared.prepared_posterior_covariance_white_stack = [ ...
    model.posterior_cov_white_axis1; ...
    model.posterior_cov_white_axis2; ...
    model.posterior_cov_white_axis3];
prepared.prepared_for_online_inference = true;
end
