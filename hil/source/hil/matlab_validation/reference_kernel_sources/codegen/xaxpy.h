//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xaxpy.h
//
// Code generation for function 'xaxpy'
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
void b_xaxpy(int32_T n, real_T a, const real_T x[4], int32_T ix0, real_T y[16],
             int32_T iy0);

void xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[16], int32_T iy0);

void xaxpy(int32_T n, real_T a, const real_T x[16], int32_T ix0, real_T y[4],
           int32_T iy0);

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xaxpy.h)
