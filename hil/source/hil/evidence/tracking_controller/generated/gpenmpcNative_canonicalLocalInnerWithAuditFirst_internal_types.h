/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcNative_canonicalLocalInnerWithAuditFirst_internal_types.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCNATIVE_CANONICALLOCALINNERWITHAUDITFIRST_INTERNAL_TYPES_H
#define GPENMPCNATIVE_CANONICALLOCALINNERWITHAUDITFIRST_INTERNAL_TYPES_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_h_struct_T
#define typedef_h_struct_T
typedef struct {
  double alpha_physical_f[3];
  double prediction_residual_i_mps2[3];
  double gp_control_i_mps2[3];
  double observer_i_mps2[3];
  double rho_applied_f_mps2[3];
  double robust_i_mps2[3];
  double raw_target_i_mps2[3];
} h_struct_T;
#endif /* typedef_h_struct_T */

#ifndef typedef_t_struct_T
#define typedef_t_struct_T
typedef struct {
  double sliding_i_mps[3];
  double sliding_f_mps[3];
  double radius_f_mps2[3];
  double target_i_mps2[3];
  double raw_target_i_mps2[3];
  double c_vertical_disturbance_estimate;
  double c_vertical_observer_compensatio[3];
  double filter_gain;
} t_struct_T;
#endif /* typedef_t_struct_T */

#endif
/*
 * File trailer for
 * gpenmpcNative_canonicalLocalInnerWithAuditFirst_internal_types.h
 *
 * [EOF]
 */
