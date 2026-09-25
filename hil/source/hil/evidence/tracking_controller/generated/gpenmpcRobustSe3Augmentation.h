/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcRobustSe3Augmentation.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCROBUSTSE3AUGMENTATION_H
#define GPENMPCROBUSTSE3AUGMENTATION_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_internal_types.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void gpenmpcRobustSe3Augmentation(
    const double plantState[19], const double reference_position_m[3],
    const double reference_velocity_mps[3],
    const double c_robustConfig_residual_tail_ma[3],
    const double c_robustConfig_robust_radius_f_[3], i_struct_T *state,
    double dt, double augmentation[3], t_struct_T *diagnostic);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcRobustSe3Augmentation.h
 *
 * [EOF]
 */
