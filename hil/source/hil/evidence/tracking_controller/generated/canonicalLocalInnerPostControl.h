/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: canonicalLocalInnerPostControl.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef CANONICALLOCALINNERPOSTCONTROL_H
#define CANONICALLOCALINNERPOSTCONTROL_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
boolean_T b_canonicalLocalInnerPostContro(
    const s_struct_T *candidate, const q_struct_T *precontrol,
    const double c_numerics_calibration_rotor_al[6],
    const double d_numerics_calibration_rotor_al[6],
    i_struct_T *nextState_robust_state,
    o_struct_T *c_nextState_attitude_continuity,
    double c_nextState_residual_history_f_[3],
    double nextState_gp_agreement_weight_f[3],
    double c_nextState_previous_rotor_comm[6],
    double c_nextState_previous_desired_fo[3], double *nextState_leg_index,
    unsigned long long *c_nextState_last_sample_timesta,
    unsigned long long *nextState_source_generation,
    d_struct_T *pendingScaffold, f_struct_T *gpRequest);

boolean_T canonicalLocalInnerPostControl(
    const p_struct_T *candidate, const l_struct_T *precontrol,
    const double c_numerics_calibration_rotor_al[6],
    const double d_numerics_calibration_rotor_al[6],
    i_struct_T *nextState_robust_state,
    double c_nextState_attitude_continuity[9],
    double d_nextState_attitude_continuity[3],
    double e_nextState_attitude_continuity[3],
    double *f_nextState_attitude_continuity,
    double c_nextState_residual_history_f_[3],
    double nextState_gp_agreement_weight_f[3],
    double c_nextState_previous_rotor_comm[6],
    double c_nextState_previous_desired_fo[3], double *nextState_leg_index,
    d_struct_T *pendingScaffold, f_struct_T *gpRequest);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for canonicalLocalInnerPostControl.h
 *
 * [EOF]
 */
