//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xdotc.cpp
//
// Code generation for function 'xdotc'
//

// Include files
#include "xdotc.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
real_T xdotc(int32_T n, const real_T x[16], int32_T ix0, const real_T y[16],
             int32_T iy0)
{
  real_T d;
  d = 0.0;
  for (int32_T k{0}; k < n; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }
  return d;
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xdotc.cpp)
