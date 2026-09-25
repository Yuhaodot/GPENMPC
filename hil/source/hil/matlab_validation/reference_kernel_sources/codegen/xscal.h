//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xscal.h
//
// Code generation for function 'xscal'
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
namespace blas {
void b_xscal(int32_T n, real_T a, real_T x[4], int32_T ix0);

void xscal(int32_T n, real_T a, real_T x[16], int32_T ix0);

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xscal.h)
