//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xnrm2.cpp
//
// Code generation for function 'xnrm2'
//

// Include files
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
real_T b_xnrm2(int32_T n, const real_T x[4], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  boolean_T b;
  y = 0.0;
  scale = 3.312168642111238E-170;
  kend = ix0 + n;
  for (int32_T k{ix0}; k < kend; k++) {
    real_T absxk;
    absxk = muDoubleScalarAbs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }
  y = scale * muDoubleScalarSqrt(y);
  b = muDoubleScalarIsNaN(y);
  if (b) {
    int32_T b_k;
    b_k = ix0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_k <= kend - 1) {
        if (muDoubleScalarIsNaN(x[b_k - 1])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

real_T xnrm2(int32_T n, const real_T x[16], int32_T ix0)
{
  real_T scale;
  real_T y;
  int32_T kend;
  boolean_T b;
  y = 0.0;
  scale = 3.312168642111238E-170;
  kend = ix0 + n;
  for (int32_T k{ix0}; k < kend; k++) {
    real_T absxk;
    absxk = muDoubleScalarAbs(x[k - 1]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }
  y = scale * muDoubleScalarSqrt(y);
  b = muDoubleScalarIsNaN(y);
  if (b) {
    int32_T b_k;
    b_k = ix0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_k <= kend - 1) {
        if (muDoubleScalarIsNaN(x[b_k - 1])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xnrm2.cpp)
