//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// svd.h
//
// Code generation for function 'svd'
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
void svd(const emlrtStack &sp, const real_T A[16], real_T U[4]);

}
} // namespace coder

// End of code generation (svd.h)
