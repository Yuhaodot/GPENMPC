//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eml_int_forloop_overflow_error.cpp
//
// Code generation for function 'eml_int_forloop_overflow_error'
//

// Include files
#include "eml_int_forloop_overflow_error.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRTEInfo q_emlrtRTEI{
    11,                               // lineNo
    1,                                // colNo
    "eml_int_forloop_overflow_error", // fName
    "matlab\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_"
    "forloop_overflow_error.m" // pName
};

// Function Definitions
namespace coder {
void eml_int_forloop_overflow_error(const emlrtStack &sp)
{
  emlrtErrorWithMessageIdR2018a(
      &sp, &q_emlrtRTEI, "Coder:toolbox:int_forloop_overflow",
      "Coder:toolbox:int_forloop_overflow", 3, 4, 5, "int32");
}

} // namespace coder

// End of code generation (eml_int_forloop_overflow_error.cpp)
