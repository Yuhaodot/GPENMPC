/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: canonicalLocalInnerFixedAbi.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef CANONICALLOCALINNERFIXEDABI_H
#define CANONICALLOCALINNERFIXEDABI_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void canonicalLocalInnerFixedAbi(e_gpenmpcNative_canonicalLocalIn *SD,
                                 const double input36[36],
                                 const unsigned long long inputTags2[2],
                                 double next64[64], double kernel61[61],
                                 double scaffold70[70], double request19[19]);

double unpackPending(const double v[70], const unsigned long long t[2],
                     m_struct_T *p_prediction, double p_velocity_up_mps[3],
                     double p_nominal_acceleration_up_mps2[3],
                     double p_frame_i_from_f[9],
                     unsigned long long *p_timestamp_ns,
                     unsigned long long *p_generation);

boolean_T unpackState(const double v[64], const unsigned long long t[2],
                      i_struct_T *s_robust_state,
                      o_struct_T *s_attitude_continuity_state,
                      double s_residual_history_f_mps2[3],
                      double s_gp_agreement_weight_f[3],
                      double s_previous_rotor_command_n[6],
                      double c_s_previous_desired_force_proj[3],
                      double *s_leg_index,
                      unsigned long long *s_last_sample_timestamp_ns,
                      unsigned long long *s_source_generation);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for canonicalLocalInnerFixedAbi.h
 *
 * [EOF]
 */
