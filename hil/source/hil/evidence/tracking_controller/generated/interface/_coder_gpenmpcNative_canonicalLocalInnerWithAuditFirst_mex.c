/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_gpenmpcNative_canonicalLocalInnerWithAuditFirst_mex.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "_coder_gpenmpcNative_canonicalLocalInnerWithAuditFirst_mex.h"
#include "_coder_gpenmpcNative_canonicalLocalInnerWithAuditFirst_api.h"

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
  e_gpenmpcNative_canonicalLocalIn *f_gpenmpcNative_canonicalLocalIn = NULL;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const char_T *entryPointTemplateNames[4] = {
      "gpenmpcNative.canonicalLocalInnerWithAuditFirst",
      "gpenmpcNative.canonicalLocalInnerWithAuditStep",
      "gpenmpcNative.queryCanonicalReferenceWindow",
      "gpenmpcNative.canonicalReferenceTransitionFromJet"};
  f_gpenmpcNative_canonicalLocalIn =
      (e_gpenmpcNative_canonicalLocalIn *)emlrtMxCalloc(
          (size_t)1, (size_t)1U * sizeof(e_gpenmpcNative_canonicalLocalIn));
  mexAtExit(&gpenmpcNative_canonicalLocalInnerWithAuditFirst_atexit);
  gpenmpcNative_canonicalLocalInnerWithAuditFirst_initialize();
  st.tls = emlrtRootTLSGlobal;
  switch (emlrtGetEntryPointIndexR2016a(
      &st, nrhs, &prhs[0], (const char_T **)&entryPointTemplateNames[0], 4)) {
  case 0:
    unsafe_gpenmpcNative_canonicalLocalInnerWithAuditFirst_mexFunction(
        nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  case 1:
    unsafe_gpenmpcNative_canonicalLocalInnerWithAuditStep_mexFunction(
        nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  case 2:
    unsafe_gpenmpcNative_queryCanonicalReferenceWindow_mexFunction(
        f_gpenmpcNative_canonicalLocalIn, nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  case 3:
    unsafe_gpenmpcNative_canonicalReferenceTransitionFromJet_mexFunction(
        nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  }
  gpenmpcNative_canonicalLocalInnerWithAuditFirst_terminate();
  emlrtMxFree(f_gpenmpcNative_canonicalLocalIn);
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
 *                mxArray *plhs[6]
 *                int32_T nrhs
 *                const mxArray *prhs[3]
 * Return Type  : void
 */
void unsafe_gpenmpcNative_canonicalLocalInnerWithAuditFirst_mexFunction(
    int32_T nlhs, mxArray *plhs[6], int32_T nrhs, const mxArray *prhs[3])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[6];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs < 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooFewInputsConstants", 9, 4, 46,
                        "gpenmpcNative.canonicalLocalInnerWithAuditFirst", 4, 46,
                        "gpenmpcNative.canonicalLocalInnerWithAuditFirst", 4, 46,
                        "gpenmpcNative.canonicalLocalInnerWithAuditFirst");
  }
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        46, "gpenmpcNative.canonicalLocalInnerWithAuditFirst");
  }
  if (nlhs > 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 46,
                        "gpenmpcNative.canonicalLocalInnerWithAuditFirst");
  }
  /* Call the function. */
  c_gpenmpcNative_canonicalLocalIn(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[6]
 *                int32_T nrhs
 *                const mxArray *prhs[7]
 * Return Type  : void
 */
void unsafe_gpenmpcNative_canonicalLocalInnerWithAuditStep_mexFunction(
    int32_T nlhs, mxArray *plhs[6], int32_T nrhs, const mxArray *prhs[7])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[6];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs < 7) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooFewInputsConstants", 9, 4, 45,
                        "gpenmpcNative.canonicalLocalInnerWithAuditStep", 4, 45,
                        "gpenmpcNative.canonicalLocalInnerWithAuditStep", 4, 45,
                        "gpenmpcNative.canonicalLocalInnerWithAuditStep");
  }
  if (nrhs != 7) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 7, 4,
                        45, "gpenmpcNative.canonicalLocalInnerWithAuditStep");
  }
  if (nlhs > 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 45,
                        "gpenmpcNative.canonicalLocalInnerWithAuditStep");
  }
  /* Call the function. */
  d_gpenmpcNative_canonicalLocalIn(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[8]
 * Return Type  : void
 */
void unsafe_gpenmpcNative_canonicalReferenceTransitionFromJet_mexFunction(
    int32_T nlhs, mxArray *plhs[1], int32_T nrhs, const mxArray *prhs[8])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 8) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 8, 4,
                        48, "gpenmpcNative.canonicalReferenceTransitionFromJet");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 48,
                        "gpenmpcNative.canonicalReferenceTransitionFromJet");
  }
  /* Call the function. */
  c_gpenmpcNative_canonicalReferen(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

/*
 * Arguments    : e_gpenmpcNative_canonicalLocalIn *SD
 *                int32_T nlhs
 *                mxArray *plhs[3]
 *                int32_T nrhs
 *                const mxArray *prhs[3]
 * Return Type  : void
 */
void unsafe_gpenmpcNative_queryCanonicalReferenceWindow_mexFunction(
    e_gpenmpcNative_canonicalLocalIn *SD, int32_T nlhs, mxArray *plhs[3],
    int32_T nrhs, const mxArray *prhs[3])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[3];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        42, "gpenmpcNative.queryCanonicalReferenceWindow");
  }
  if (nlhs > 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 42,
                        "gpenmpcNative.queryCanonicalReferenceWindow");
  }
  /* Call the function. */
  c_gpenmpcNative_queryCanonicalRe(SD, prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

/*
 * File trailer for _coder_gpenmpcNative_canonicalLocalInnerWithAuditFirst_mex.c
 *
 * [EOF]
 */
