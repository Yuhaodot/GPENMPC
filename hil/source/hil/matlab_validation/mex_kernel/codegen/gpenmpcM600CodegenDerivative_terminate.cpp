//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// gpenmpcM600CodegenDerivative_terminate.cpp
//
// Code generation for function 'gpenmpcM600CodegenDerivative_terminate'
//

// Include files
#include "gpenmpcM600CodegenDerivative_terminate.h"
#include "_coder_gpenmpcM600CodegenDerivative_mex.h"
#include "gpenmpcM600CodegenDerivative_data.h"
#include "rt_nonfinite.h"

// Function Declarations
static void emlrtExitTimeCleanupDtorFcn(const void *r);

// Function Definitions
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void gpenmpcM600CodegenDerivative_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  try {
    emlrtPushHeapReferenceStackR2021a(&st, false, nullptr,
                                      (void *)&emlrtExitTimeCleanupDtorFcn,
                                      nullptr, nullptr, nullptr);
    emlrtEnterRtStackR2012b(&st);
    emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
    emlrtExitTimeCleanup(&emlrtContextGlobal);
  } catch (...) {
    emlrtCleanupOnException((emlrtCTX *)emlrtRootTLSGlobal);
    throw;
  }
}

void gpenmpcM600CodegenDerivative_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

// End of code generation (gpenmpcM600CodegenDerivative_terminate.cpp)
