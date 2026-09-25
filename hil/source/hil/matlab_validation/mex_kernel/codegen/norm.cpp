//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// norm.cpp
//
// Code generation for function 'norm'
//

// Include files
#include "norm.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

// Function Definitions
namespace coder {
real_T b_norm(const real_T x[4])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  boolean_T b;
  scale = 3.312168642111238E-170;
  absxk = muDoubleScalarAbs(x[0]);
  if (absxk > 3.312168642111238E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.312168642111238E-170;
    y = t * t;
  }
  absxk = muDoubleScalarAbs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(x[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * muDoubleScalarSqrt(y);
  b = muDoubleScalarIsNaN(y);
  if (b) {
    int32_T k;
    k = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (k < 4) {
        if (muDoubleScalarIsNaN(x[k])) {
          exitg1 = 1;
        } else {
          k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

real_T c_norm(const real_T x[2])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  boolean_T b;
  scale = 3.312168642111238E-170;
  absxk = muDoubleScalarAbs(x[0]);
  if (absxk > 3.312168642111238E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.312168642111238E-170;
    y = t * t;
  }
  absxk = muDoubleScalarAbs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * muDoubleScalarSqrt(y);
  b = muDoubleScalarIsNaN(y);
  if (b) {
    int32_T k;
    k = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (k < 2) {
        if (muDoubleScalarIsNaN(x[k])) {
          exitg1 = 1;
        } else {
          k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

} // namespace coder

// End of code generation (norm.cpp)
