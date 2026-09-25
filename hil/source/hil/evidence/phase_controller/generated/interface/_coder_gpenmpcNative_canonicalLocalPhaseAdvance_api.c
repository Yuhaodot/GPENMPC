/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_gpenmpcNative_canonicalLocalPhaseAdvance_api.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-06 21:28:59
 */

/* Include Files */
#include "_coder_gpenmpcNative_canonicalLocalPhaseAdvance_api.h"
#include "_coder_gpenmpcNative_canonicalLocalPhaseAdvance_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131690U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "gpenmpcNative_canonicalLocalPhaseAdvance",            /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[7];

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[7];

static void emlrtExitTimeCleanupDtorFcn(const void *r);

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[7];

static const mxArray *emlrt_marshallOut(real_T u[2]);

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[7]
 */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[7]
{
  real_T(*y)[7];
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[7]
 */
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[7]
{
  static const int32_T dims = 7;
  real_T(*ret)[7];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[7])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const void *r
 * Return Type  : void
 */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : real_T (*)[7]
 */
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[7]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[7];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : real_T u[2]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(real_T u[2])
{
  static const int32_T i = 0;
  static const int32_T i1 = 2;
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  existingData = emlrtMxGetData((mxArray *)m);
  if (existingData != (void *)&u[0]) {
    emlrtFreeMex(existingData);
  }
  emlrtMxSetData((mxArray *)m, &u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[1]
 *                const mxArray **plhs
 * Return Type  : void
 */
void c_gpenmpcNative_canonicalLocalPh(const mxArray *const prhs[1],
                                     const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *prhs_copy_idx_0;
  real_T(*input7)[7];
  real_T(*next)[2];
  st.tls = emlrtRootTLSGlobal;
  next = (real_T(*)[2])mxMalloc(sizeof(real_T[2]));
  prhs_copy_idx_0 = emlrtProtectR2012b(prhs[0], 0, false, -1);
  /* Marshall function inputs */
  input7 = emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_0), "input7");
  /* Invoke the target function */
  gpenmpcNative_canonicalLocalPhaseAdvance(*input7, *next);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(*next);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void gpenmpcNative_canonicalLocalPhaseAdvance_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtPushHeapReferenceStackR2021a(
      &st, false, NULL, (void *)&emlrtExitTimeCleanupDtorFcn, NULL, NULL, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  gpenmpcNative_canonicalLocalPhaseAdvance_xil_terminate();
  gpenmpcNative_canonicalLocalPhaseAdvance_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void gpenmpcNative_canonicalLocalPhaseAdvance_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void gpenmpcNative_canonicalLocalPhaseAdvance_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_gpenmpcNative_canonicalLocalPhaseAdvance_api.c
 *
 * [EOF]
 */
