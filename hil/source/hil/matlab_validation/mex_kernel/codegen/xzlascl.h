//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzlascl.h
//
// Code generation for function 'xzlascl'
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
namespace reflapack {
void b_xzlascl(real_T cfrom, real_T cto, real_T A[16]);

void xzlascl(real_T cfrom, real_T cto, real_T A[4]);

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzlascl.h)
