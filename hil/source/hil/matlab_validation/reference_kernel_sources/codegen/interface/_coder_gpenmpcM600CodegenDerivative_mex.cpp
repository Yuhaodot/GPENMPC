//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_gpenmpcM600CodegenDerivative_mex.cpp
//
// Code generation for function '_coder_gpenmpcM600CodegenDerivative_mex'
//

// Include files
#include "_coder_gpenmpcM600CodegenDerivative_mex.h"
#include "_coder_gpenmpcM600CodegenDerivative_api.h"
#include "gpenmpcM600CodegenDerivative_data.h"
#include "gpenmpcM600CodegenDerivative_initialize.h"
#include "gpenmpcM600CodegenDerivative_terminate.h"
#include "rt_nonfinite.h"
#include <stdexcept>

void emlrtExceptionBridge();
void emlrtExceptionBridge()
{
  throw std::runtime_error("");
}
// Function Definitions
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&gpenmpcM600CodegenDerivative_atexit);
  gpenmpcM600CodegenDerivative_initialize();
  try {
    gpenmpcM600CodegenDerivative_mexFunction(nlhs, plhs, nrhs, prhs);
    gpenmpcM600CodegenDerivative_terminate();
  } catch (...) {
    emlrtCleanupOnException((emlrtCTX *)emlrtRootTLSGlobal);
    throw;
  }
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           (void *)&emlrtExceptionBridge, "GBK", true);
  return emlrtRootTLSGlobal;
}

void gpenmpcM600CodegenDerivative_mexFunction(int32_T nlhs, mxArray *plhs[4],
                                             int32_T nrhs,
                                             const mxArray *prhs[7])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *outputs[4];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 7) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 7, 4,
                        28, "gpenmpcM600CodegenDerivative");
  }
  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 28,
                        "gpenmpcM600CodegenDerivative");
  }
  // Call the function.
  gpenmpcM600CodegenDerivative_api(prhs, nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

// End of code generation (_coder_gpenmpcM600CodegenDerivative_mex.cpp)
