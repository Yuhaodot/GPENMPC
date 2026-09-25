//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sumMatrixIncludeNaN.cpp
//
// Code generation for function 'sumMatrixIncludeNaN'
//

// Include files
#include "sumMatrixIncludeNaN.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
real_T sumColumnB(const real_T x[6])
{
  real_T y;
  y = x[0];
  for (int32_T k{0}; k < 5; k++) {
    y += x[k + 1];
  }
  return y;
}

} // namespace coder

// End of code generation (sumMatrixIncludeNaN.cpp)
