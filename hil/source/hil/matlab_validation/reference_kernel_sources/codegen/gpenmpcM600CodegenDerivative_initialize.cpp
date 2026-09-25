//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// gpenmpcM600CodegenDerivative_initialize.cpp
//
// Code generation for function 'gpenmpcM600CodegenDerivative_initialize'
//

// Include files
#include "gpenmpcM600CodegenDerivative_initialize.h"
#include "_coder_gpenmpcM600CodegenDerivative_mex.h"
#include "gpenmpcM600CodegenDerivative_data.h"
#include "rt_nonfinite.h"

// Function Declarations
static void gpenmpcM600CodegenDerivative_once();

// Function Definitions
static void gpenmpcM600CodegenDerivative_once()
{
  mex_InitInfAndNan();
}

void gpenmpcM600CodegenDerivative_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    gpenmpcM600CodegenDerivative_once();
  }
}

// End of code generation (gpenmpcM600CodegenDerivative_initialize.cpp)
