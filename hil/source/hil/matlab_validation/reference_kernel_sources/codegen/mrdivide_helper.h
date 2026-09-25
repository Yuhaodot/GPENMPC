//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mrdivide_helper.h
//
// Code generation for function 'mrdivide_helper'
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
namespace coder {
namespace internal {
void mrdiv(const emlrtStack &sp, real_T A[24], const real_T B[16]);

}
} // namespace coder

// End of code generation (mrdivide_helper.h)
