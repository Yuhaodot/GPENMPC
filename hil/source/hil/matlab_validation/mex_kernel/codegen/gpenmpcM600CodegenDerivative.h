//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// gpenmpcM600CodegenDerivative.h
//
// Code generation for function 'gpenmpcM600CodegenDerivative'
//

#pragma once

// Include files
#include "gpenmpcM600CodegenDerivative_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Function Declarations
void gpenmpcM600CodegenDerivative(const emlrtStack *sp, const real_T x[19],
                                 const real_T u[16], const real_T jet[12],
                                 real_T payload, const real_T wind[2],
                                 real_T b_time, const struct0_T *p,
                                 real_T dx[19], struct12_T *diagnostic,
                                 struct13_T *contact, real_T rotorCommandN[6]);

// End of code generation (gpenmpcM600CodegenDerivative.h)
