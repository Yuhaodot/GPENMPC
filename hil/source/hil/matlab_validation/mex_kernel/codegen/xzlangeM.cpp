//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzlangeM.cpp
//
// Code generation for function 'xzlangeM'
//

// Include files
#include "xzlangeM.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
real_T xzlangeM(const real_T x[16])
{
  real_T y;
  int32_T k;
  boolean_T exitg1;
  y = 0.0;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k < 16)) {
    real_T absxk;
    absxk = muDoubleScalarAbs(x[k]);
    if (muDoubleScalarIsNaN(absxk)) {
      y = rtNaN;
      exitg1 = true;
    } else {
      if (absxk > y) {
        y = absxk;
      }
      k++;
    }
  }
  return y;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzlangeM.cpp)
