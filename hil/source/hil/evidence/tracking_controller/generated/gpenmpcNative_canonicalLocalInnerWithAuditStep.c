/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcNative_canonicalLocalInnerWithAuditStep.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditStep.h"
#include "canonicalLocalInnerFixedAbi.h"
#include "canonicalLocalInnerPostControl.h"
#include "canonicalLocalInnerPreControl.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_data.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Tracking-controller step with learning-state outputs.
 *
 * Arguments    : e_gpenmpcNative_canonicalLocalIn *SD
 *                const double state64[64]
 *                const unsigned long long stateTags2[2]
 *                const double input36[36]
 *                const unsigned long long inputTags2[2]
 *                const double pending70[70]
 *                const unsigned long long pendingTags2[2]
 *                double next64[64]
 *                double kernel61[61]
 *                double scaffold70[70]
 *                double request19[19]
 *                double closed5[5]
 *                double learning12[12]
 * Return Type  : void
 */
void gpenmpcNative_canonicalLocalInnerWithAuditStep(
    e_gpenmpcNative_canonicalLocalIn *SD, const double state64[64],
    const unsigned long long stateTags2[2], const double input36[36],
    const unsigned long long inputTags2[2], const double pending70[70],
    const unsigned long long pendingTags2[2], double next64[64],
    double kernel61[61], double scaffold70[70], double request19[19],
    double closed5[5], double learning12[12])
{
  double c_t11_calibration_rotor_allocat[6];
  double next_previous_rotor_command_n[6];
  double c_next_previous_desired_force_p[3];
  double next_gp_agreement_weight_f[3];
  double next_residual_history_f_mps2[3];
  double next_leg_index;
  unsigned long long expl_temp;
  unsigned long long next_last_sample_timestamp_ns;
  unsigned long long next_source_generation;
  int i;
  /* Fixed-shape adapter for precontrol and postcontrol calculations.
   * newLeg is a compile-time constant in the two generated entry points.
   * Tags are uint64 [timestamp_ns; source_generation]; numerical arrays are double.
   * The caller validates and commits the returned candidate state.
   * Complete the GP request in scaffold70 before the next sample closes innovation.
   *
   * input36: x13(1:13), rotor lag6(14:19), reference p/v/a/j(20:31),
   * payload(32), wind2(33:34), dt(35), leg(36).
   * state64: robust27, attitude20, residual3, agreement3, causal1,
   * previous rotor6, previous projected force3, leg1 (details in packState).
   * pending70: prediction54 then velocity3/nominal3/frame9/leg1.
   * request19: required1, features17, mean_scale1.
   * Optional closed5 contains precontrol evidence for gpenmpcCoordinatedOuterStep:
   * available/closed/innovation/hard_invalid/trust. The caller installs it
   * separately from the pending prediction in scaffold70.
   */
  memcpy(&SD->u4.f7.b_expl_temp.x13[0], &input36[0], 13U * sizeof(double));
  for (i = 0; i < 6; i++) {
    SD->u4.f7.b_expl_temp.rotor_thrust_state_n[i] = input36[i + 13];
  }
  SD->u4.f7.b_expl_temp.reference_up.position_m[0] = input36[19];
  SD->u4.f7.b_expl_temp.reference_up.velocity_mps[0] = input36[22];
  SD->u4.f7.b_expl_temp.reference_up.acceleration_mps2[0] = input36[25];
  SD->u4.f7.b_expl_temp.reference_up.jerk_mps3[0] = input36[28];
  SD->u4.f7.b_expl_temp.reference_up.position_m[1] = input36[20];
  SD->u4.f7.b_expl_temp.reference_up.velocity_mps[1] = input36[23];
  SD->u4.f7.b_expl_temp.reference_up.acceleration_mps2[1] = input36[26];
  SD->u4.f7.b_expl_temp.reference_up.jerk_mps3[1] = input36[29];
  SD->u4.f7.b_expl_temp.reference_up.position_m[2] = input36[21];
  SD->u4.f7.b_expl_temp.reference_up.velocity_mps[2] = input36[24];
  SD->u4.f7.b_expl_temp.reference_up.acceleration_mps2[2] = input36[27];
  SD->u4.f7.b_expl_temp.reference_up.jerk_mps3[2] = input36[30];
  SD->u4.f7.b_expl_temp.wind_estimate_xy_mps[0] = input36[32];
  SD->u4.f7.b_expl_temp.wind_estimate_xy_mps[1] = input36[33];
  unpackState(state64, stateTags2, &SD->u4.f7.next_robust_state,
              &SD->u4.f7.next_attitude_continuity_state,
              next_residual_history_f_mps2, next_gp_agreement_weight_f,
              next_previous_rotor_command_n, c_next_previous_desired_force_p,
              &next_leg_index, &next_last_sample_timestamp_ns,
              &next_source_generation);
  unpackPending(pending70, pendingTags2, &SD->u4.f7.expl_temp.prediction,
                SD->u4.f7.expl_temp.velocity_up_mps,
                SD->u4.f7.expl_temp.nominal_acceleration_up_mps2,
                SD->u4.f7.expl_temp.frame_i_from_f, &next_source_generation,
                &expl_temp);
  SD->u4.f7.b_expl_temp.source_generation = inputTags2[1];
  SD->u4.f7.b_expl_temp.source_timestamp_ns = inputTags2[0];
  SD->u4.f7.b_expl_temp.leg_index = input36[35];
  SD->u4.f7.b_expl_temp.dt_s = input36[34];
  SD->u4.f7.b_expl_temp.payload_kg = input36[31];
  b_canonicalLocalInnerPreControl(
      SD, &SD->u4.f7.next_robust_state,
      &SD->u4.f7.next_attitude_continuity_state, next_residual_history_f_mps2,
      next_gp_agreement_weight_f, next_previous_rotor_command_n,
      next_last_sample_timestamp_ns, &SD->u4.f7.b_expl_temp,
      &SD->u4.f7.expl_temp.prediction, SD->u4.f7.expl_temp.velocity_up_mps,
      SD->u4.f7.expl_temp.nominal_acceleration_up_mps2,
      SD->u4.f7.expl_temp.frame_i_from_f, &SD->u4.f7.candidate, &SD->u4.f7.out);
  for (i = 0; i < 6; i++) {
    c_t11_calibration_rotor_allocat[i] = 60.0 * (double)i;
  }
  boolean_T next_causal_valid;
  next_causal_valid = b_canonicalLocalInnerPostContro(
      &SD->u4.f7.candidate, &SD->u4.f7.out, c_t11_calibration_rotor_allocat, dv,
      &SD->u4.f7.next_robust_state, &SD->u4.f7.next_attitude_continuity_state,
      next_residual_history_f_mps2, next_gp_agreement_weight_f,
      next_previous_rotor_command_n, c_next_previous_desired_force_p,
      &next_leg_index, &next_last_sample_timestamp_ns, &next_source_generation,
      &SD->u4.f7.scaffold, &SD->u4.f7.query);
  next64[0] = SD->u4.f7.next_robust_state.filtered_compensation_i_mps2[0];
  next64[1] = SD->u4.f7.next_robust_state.filtered_compensation_i_mps2[1];
  next64[2] = SD->u4.f7.next_robust_state.filtered_compensation_i_mps2[2];
  next64[3] = SD->u4.f7.next_robust_state.authority_scale;
  next64[4] = SD->u4.f7.next_robust_state.last_tangent_xy[0];
  next64[5] = SD->u4.f7.next_robust_state.last_tangent_xy[1];
  next64[6] = SD->u4.f7.next_robust_state.vertical_disturbance_ewma_mps2;
  next64[7] = SD->u4.f7.next_robust_state.c_vertical_observer_previous_ve;
  next64[8] = SD->u4.f7.next_robust_state.c_vertical_observer_previous_no;
  next64[9] = SD->u4.f7.next_robust_state.c_vertical_observer_previous_ti;
  next64[10] = SD->u4.f7.next_robust_state.c_vertical_observer_previous_le;
  next64[11] = SD->u4.f7.next_robust_state.c_vertical_observer_previous_pa;
  next64[12] = 1.0;
  next64[13] = SD->u4.f7.next_robust_state.c_vertical_observer_update_enab;
  next64[14] = SD->u4.f7.next_robust_state.vertical_observer_reset_count;
  next64[15] = SD->u4.f7.next_robust_state.c_vertical_observer_antiwindup_;
  next64[19] = SD->u4.f7.next_robust_state.gp_responsibility_mode_active;
  next64[20] = SD->u4.f7.next_robust_state.c_gp_responsibility_enter_elapsed;
  next64[21] = SD->u4.f7.next_robust_state.c_gp_responsibility_exit_elapsed;
  next64[22] = SD->u4.f7.next_robust_state.gp_responsibility_blend;
  next64[16] = SD->u4.f7.next_robust_state.responsibility_innovation_ratio_ewma_f[0];
  next64[23] = SD->u4.f7.next_robust_state.responsibility_filtered_gp_mean_f_mps2[0];
  next64[17] = SD->u4.f7.next_robust_state.responsibility_innovation_ratio_ewma_f[1];
  next64[24] = SD->u4.f7.next_robust_state.responsibility_filtered_gp_mean_f_mps2[1];
  next64[18] = SD->u4.f7.next_robust_state.responsibility_innovation_ratio_ewma_f[2];
  next64[25] = SD->u4.f7.next_robust_state.responsibility_filtered_gp_mean_f_mps2[2];
  next64[26] = SD->u4.f7.next_robust_state.gp_responsibility_mode_transition_count;
  next64[27] = 1.0;
  next64[28] = SD->u4.f7.next_attitude_continuity_state.angular_velocity_valid;
  next64[29] =
      SD->u4.f7.next_attitude_continuity_state.angular_acceleration_valid;
  memcpy(&next64[30],
         &SD->u4.f7.next_attitude_continuity_state.filtered_rotation[0],
         9U * sizeof(double));
  next64[45] = SD->u4.f7.next_attitude_continuity_state.update_count;
  next64[46] = SD->u4.f7.next_attitude_continuity_state.reset_count;
  next64[39] = SD->u4.f7.next_attitude_continuity_state
                   .c_desired_angular_velocity_body[0];
  next64[42] = SD->u4.f7.next_attitude_continuity_state
                   .c_desired_angular_acceleration_[0];
  next64[47] = next_residual_history_f_mps2[0];
  next64[50] = next_gp_agreement_weight_f[0];
  next64[40] = SD->u4.f7.next_attitude_continuity_state
                   .c_desired_angular_velocity_body[1];
  next64[43] = SD->u4.f7.next_attitude_continuity_state
                   .c_desired_angular_acceleration_[1];
  next64[48] = next_residual_history_f_mps2[1];
  next64[51] = next_gp_agreement_weight_f[1];
  next64[41] = SD->u4.f7.next_attitude_continuity_state
                   .c_desired_angular_velocity_body[2];
  next64[44] = SD->u4.f7.next_attitude_continuity_state
                   .c_desired_angular_acceleration_[2];
  next64[49] = next_residual_history_f_mps2[2];
  next64[52] = next_gp_agreement_weight_f[2];
  next64[53] = next_causal_valid;
  for (i = 0; i < 6; i++) {
    next64[i + 54] = next_previous_rotor_command_n[i];
  }
  next64[60] = c_next_previous_desired_force_p[0];
  next64[61] = c_next_previous_desired_force_p[1];
  next64[62] = c_next_previous_desired_force_p[2];
  next64[63] = next_leg_index;
  memcpy(&kernel61[0], &SD->u4.f7.out.kernel_output61[0], 61U * sizeof(double));
  scaffold70[0] = 1.0;
  scaffold70[1] = 0.0;
  scaffold70[2] = SD->u4.f7.query.pending.causal_valid;
  scaffold70[3] = 1.0;
  scaffold70[4] = 0.0;
  scaffold70[5] = 0.0;
  scaffold70[6] = 0.25;
  scaffold70[7] = rtNaN;
  scaffold70[8] = rtNaN;
  for (i = 0; i < 17; i++) {
    scaffold70[i + 9] = rtNaN;
  }
  scaffold70[44] = 0.0;
  scaffold70[45] = 0.0;
  scaffold70[52] = 0.0;
  scaffold70[53] = 0.0;
  scaffold70[35] = rtNaN;
  scaffold70[38] = rtNaN;
  scaffold70[41] = rtNaN;
  scaffold70[46] = rtNaN;
  scaffold70[49] = rtNaN;
  scaffold70[54] = SD->u4.f7.scaffold.velocity_up_mps[0];
  scaffold70[57] = SD->u4.f7.scaffold.nominal_acceleration_up_mps2[0];
  scaffold70[36] = rtNaN;
  scaffold70[39] = rtNaN;
  scaffold70[42] = rtNaN;
  scaffold70[47] = rtNaN;
  scaffold70[50] = rtNaN;
  scaffold70[55] = SD->u4.f7.scaffold.velocity_up_mps[1];
  scaffold70[58] = SD->u4.f7.scaffold.nominal_acceleration_up_mps2[1];
  scaffold70[37] = rtNaN;
  scaffold70[40] = rtNaN;
  scaffold70[43] = rtNaN;
  scaffold70[48] = rtNaN;
  scaffold70[51] = rtNaN;
  scaffold70[56] = SD->u4.f7.scaffold.velocity_up_mps[2];
  scaffold70[59] = SD->u4.f7.scaffold.nominal_acceleration_up_mps2[2];
  memcpy(&scaffold70[26], &SD->u4.f7.query.pending.gp_frame_i_from_f[0],
         9U * sizeof(double));
  memcpy(&scaffold70[60], &SD->u4.f7.scaffold.frame_i_from_f[0],
         9U * sizeof(double));
  scaffold70[69] = SD->u4.f7.scaffold.leg_index;
  request19[0] = SD->u4.f7.query.prediction_required;
  memcpy(&request19[1], &SD->u4.f7.query.features_f17[0], 17U * sizeof(double));
  request19[18] = SD->u4.f7.query.gp_mean_scale;
  closed5[0] = SD->u4.f7.out.closed_gp_evidence.available;
  closed5[1] = 1.0;
  closed5[2] = SD->u4.f7.out.closed_gp_evidence.observed_innovation_available;
  closed5[3] = SD->u4.f7.out.closed_gp_evidence.hard_invalid;
  closed5[4] = SD->u4.f7.out.closed_gp_evidence.trust;
  /*  Read-only CURRENT closed/execution diagnostics. Do not infer physical */
  /*  authority from the next open GP request, aggregate trust or labels. */
  learning12[0] =
      SD->u4.f7.out.closed_gp_evidence.observed_innovation_f_mps2[0];
  learning12[3] =
      SD->u4.f7.out.physical_diagnostic.gp_prediction_axis_authority_f[0];
  learning12[6] =
      SD->u4.f7.out.physical_diagnostic.gp_physical_axis_authority_f[0];
  learning12[1] =
      SD->u4.f7.out.closed_gp_evidence.observed_innovation_f_mps2[1];
  learning12[4] =
      SD->u4.f7.out.physical_diagnostic.gp_prediction_axis_authority_f[1];
  learning12[7] =
      SD->u4.f7.out.physical_diagnostic.gp_physical_axis_authority_f[1];
  learning12[2] =
      SD->u4.f7.out.closed_gp_evidence.observed_innovation_f_mps2[2];
  learning12[5] =
      SD->u4.f7.out.physical_diagnostic.gp_prediction_axis_authority_f[2];
  learning12[8] =
      SD->u4.f7.out.physical_diagnostic.gp_physical_axis_authority_f[2];
  learning12[9] = SD->u4.f7.out.physical_diagnostic.exact_b1_fallback;
  learning12[10] = SD->u4.f7.out.physical_diagnostic.gp_responsibility_blend;
  learning12[11] = SD->u4.f7.out.physical_diagnostic.gp_trust;
}

/*
 * File trailer for gpenmpcNative_canonicalLocalInnerWithAuditStep.c
 *
 * [EOF]
 */
