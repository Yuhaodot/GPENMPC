/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcComputeGpResponsibility.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCCOMPUTEGPRESPONSIBILITY_H
#define GPENMPCCOMPUTEGPRESPONSIBILITY_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
boolean_T c_gpenmpcComputeGpResponsibility(
    double responsibilityBlend, double trust, const double agreementF[3],
    const double meanF[3], boolean_T *c_responsibility_exact_b1_requi,
    char c_responsibility_reason_Value_d[],
    int c_responsibility_reason_Value_s[2], double *responsibility_blend,
    double *responsibility_trust, double responsibility_agreement_f[3],
    double c_responsibility_alpha_effectiv[3],
    double *d_responsibility_alpha_effectiv,
    double responsibility_mean_f_mps2[3],
    double c_responsibility_weighted_mean_[3],
    double *c_responsibility_future_samples);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcComputeGpResponsibility.h
 *
 * [EOF]
 */
