/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_gpenmpcNative_canonicalLocalPhaseAdvance_mex.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-06 21:28:59
 */

/* Include Files */
#include "_coder_gpenmpcNative_canonicalLocalPhaseAdvance_mex.h"
#include "_coder_gpenmpcNative_canonicalLocalPhaseAdvance_api.h"

/* Function Definitions */
/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[]
 *                int32_T nrhs
 *                const mxArray *prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&gpenmpcNative_canonicalLocalPhaseAdvance_atexit);
  gpenmpcNative_canonicalLocalPhaseAdvance_initialize();
  unsafe_gpenmpcNative_canonicalLocalPhaseAdvance_mexFunction(nlhs, plhs, nrhs,
                                                             prhs);
  gpenmpcNative_canonicalLocalPhaseAdvance_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "GBK", true);
  return emlrtRootTLSGlobal;
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[1]
 * Return Type  : void
 */
void unsafe_gpenmpcNative_canonicalLocalPhaseAdvance_mexFunction(
    int32_T nlhs, mxArray *plhs[1], int32_T nrhs, const mxArray *prhs[1])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4,
                        39, "gpenmpcNative.canonicalLocalPhaseAdvance");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 39,
                        "gpenmpcNative.canonicalLocalPhaseAdvance");
  }
  /* Call the function. */
  c_gpenmpcNative_canonicalLocalPh(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/*
 * File trailer for _coder_gpenmpcNative_canonicalLocalPhaseAdvance_mex.c
 *
 * [EOF]
 */
