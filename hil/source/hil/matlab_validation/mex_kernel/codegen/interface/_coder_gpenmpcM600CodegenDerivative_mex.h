//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_gpenmpcM600CodegenDerivative_mex.h
//
// Code generation for function '_coder_gpenmpcM600CodegenDerivative_mex'
//

#pragma once

// Include files
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void gpenmpcM600CodegenDerivative_mexFunction(int32_T nlhs, mxArray *plhs[4],
                                             int32_T nrhs,
                                             const mxArray *prhs[7]);

// End of code generation (_coder_gpenmpcM600CodegenDerivative_mex.h)
