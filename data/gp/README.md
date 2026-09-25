# Sparse Gaussian Process Model

`model.mat` supplies the pretrained sparse Gaussian process (GP) model for
acceleration residuals used by the MATLAB controllers. Its `nativeModel` structure contains normalization values,
inducing inputs, kernel parameters, posterior quantities and uncertainty
calibration values. The asset loader prepares these for inference during
simulation.

The model uses 17 causal input features, three acceleration-residual outputs,
256 inducing points and an ARD-RBF kernel. Training minimizes a collapsed
variational free-energy objective with Adam. GP prediction and feature
construction are implemented in `matlab/core/inference`.

## Training source

The repository retains the MATLAB training and data-processing algorithms:

- `buildGPENMPCNativeTrainingData`: constructs training and calibration matrices
  from raw NPZ trajectory logs.
- `buildGPENMPCNativeInducingData`: constructs the inducing-point MAT with
  deterministic k-means++ selection.
- `trainGPENMPCSparseGpNative`: fits the sparse GP from validated training and
  inducing-point MAT inputs.
- `testGPENMPCSparseVfeAutodiff`: checks objective gradients.

Training requires Deep Learning Toolbox and raw training and calibration NPZ
logs. See [Sparse GP training](../../docs/training.md) for the build and
training commands.
