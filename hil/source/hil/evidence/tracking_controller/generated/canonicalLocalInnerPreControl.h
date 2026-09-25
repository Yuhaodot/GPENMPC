/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: canonicalLocalInnerPreControl.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef CANONICALLOCALINNERPRECONTROL_H
#define CANONICALLOCALINNERPRECONTROL_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_canonicalLocalInnerPreControl(
    e_gpenmpcNative_canonicalLocalIn *SD,
    const i_struct_T *committed_robust_state,
    const o_struct_T *c_committed_attitude_continuity,
    const double c_committed_residual_history_f_[3],
    const double committed_gp_agreement_weight_f[3],
    const double c_committed_previous_rotor_comm[6],
    unsigned long long c_committed_last_sample_timesta, const struct_T *input,
    const m_struct_T *previousPending_prediction,
    const double previousPending_velocity_up_mps[3],
    const double c_previousPending_nominal_accel[3],
    const double previousPending_frame_i_from_f[9], s_struct_T *candidate,
    q_struct_T *out);

void canonicalLocalInnerPreControl(
    e_gpenmpcNative_canonicalLocalIn *SD, const double input_x13[13],
    const double input_rotor_thrust_state_n[6],
    const double input_reference_up_position_m[3],
    const double input_reference_up_velocity_mps[3],
    const double c_input_reference_up_accelerati[3],
    const double input_reference_up_jerk_mps3[3], double input_payload_kg,
    const double input_wind_estimate_xy_mps[2], double input_dt_s,
    double input_leg_index, unsigned long long input_source_timestamp_ns,
    unsigned long long input_source_generation, p_struct_T *candidate,
    l_struct_T *out);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for canonicalLocalInnerPreControl.h
 *
 * [EOF]
 */
