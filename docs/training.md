# Sparse GP training

Training uses a 17-input, three-output ARD-RBF sparse Gaussian process with
256 inducing points and a collapsed variational free-energy objective.
MATLAB Deep Learning Toolbox is required for the gradient calculation.
The default optimizer runs two initializations with 48 Adam updates each.

## Inputs

The raw training logs are separate from the pre-trained model distributed in
`data/gp/model.mat`. Supply a raw-log directory containing:

```text
training/SLRGP_TR_001/ ... training/SLRGP_TR_012/
calibration/SLRGP_CA_001/ ... calibration/SLRGP_CA_006/
```

Each mission directory contains `raw_trace.npz` and one
`MISSION_DATA_SUMMARY_*.json` with matching `mission_id` and `partition` values
and a `family` field. The NPZ observation channels are consumed by
`gpenmpcExtractCausalAeroF17`. The builder selects 390 rows per mission, giving
4,680 training and 2,340 calibration rows. Labels use the next estimated
velocity sample; the F17 inputs use information available at the current step.

## Build and train

Choose a new output directory and set `sourceRoot` to the raw-log directory:

```matlab
setup_project
sourceRoot = "path/to/raw_missions";
outputRoot = "path/to/new_training_run";
dataPath = fullfile(outputRoot, "training_data.mat");
inducingPath = fullfile(outputRoot, "inducing.mat");
buildGPENMPCNativeTrainingData(sourceRoot, dataPath);
buildGPENMPCNativeInducingData(dataPath, inducingPath);
result = trainGPENMPCSparseGpNative(dataPath, inducingPath, ...
    fullfile(outputRoot, "model"));
```

The inducing builder applies the fixed PCG64 seed `1323034700`, k-means++
initialization and up to 12 Lloyd iterations. Its input domain is exactly
4,680 rows with 17 features. It checks deterministic repetition and binds the
output to the training MAT checksum.

Both builders accept an optional third argument for a reference MAT. The data
builder compares train/calibration matrices and selected row indices at
`1e-12` absolute tolerance. The inducing builder compares the reference's
`inducing_standardized_matlab` or `inducing_standardized_python` at `1e-10`.

Training writes `MATLAB_NATIVE_SPARSE_GP_MODEL.mat` and a JSON result to the
chosen model directory. The saved `nativeModel` contains the fitted kernel,
posterior, calibration quantiles and support thresholds. Each restart's final
loss is evaluated after its last Adam update, and that loss selects the saved
parameters. `loss_trace` records the pre-update objectives; its update counts
identify the corresponding parameter states.

## Short regression

The regression requires a data MAT built without an optional reference:

```matlab
addpath("tests")
report = test_training(dataPath, fullfile(outputRoot, "regression"));
```

This runs two Adam updates per initialization on the full input domain and
recomputes the saved model's objective independently. It also checks input
rejections, inducing-point repeatability, output overwrite protection and
finite-difference gradients. An optional third argument compares inducing
points with a supplied reference MAT. All regression artifacts go to the
specified new directory.

For other bounded training runs, use `OptimizationSteps` and `Restarts`:

```matlab
result = trainGPENMPCSparseGpNative(dataPath, inducingPath, ...
    fullfile(outputRoot, "short_fit"), OptimizationSteps=2, Restarts=2);
```
