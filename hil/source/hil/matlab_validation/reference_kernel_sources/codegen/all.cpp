//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// all.cpp
//
// Code generation for function 'all'
//

// Include files
#include "all.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
boolean_T all(const boolean_T x[6])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k <= 5)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

} // namespace coder

// End of code generation (all.cpp)
