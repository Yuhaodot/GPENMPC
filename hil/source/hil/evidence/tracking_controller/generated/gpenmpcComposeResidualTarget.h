/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcComposeResidualTarget.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCCOMPOSERESIDUALTARGET_H
#define GPENMPCCOMPOSERESIDUALTARGET_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_gpenmpcComposeResidualTarget(
    const double slidingF[3], const double c_responsibility_alpha_effectiv[3],
    const double c_responsibility_weighted_mean_[3],
    const double gpFrameIFromF[9], const double robustFrameIFromF[9],
    const double observerShadowI[3], h_struct_T *composition);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcComposeResidualTarget.h
 *
 * [EOF]
 */
