/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: canonicalLocalInnerFixedAbi.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "canonicalLocalInnerFixedAbi.h"
#include "canonicalLocalInnerPostControl.h"
#include "canonicalLocalInnerPreControl.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_data.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Fixed-shape adapter for precontrol and postcontrol calculations.
 *  newLeg is a compile-time constant in the two generated entry points.
 *  Tags are uint64 [timestamp_ns; source_generation]; numerical arrays are double.
 *  The caller validates and commits the returned candidate state.
 *  Complete the GP request in scaffold70 before the next sample closes innovation.
 *
 *  input36: x13(1:13), rotor lag6(14:19), reference p/v/a/j(20:31),
 *  payload(32), wind2(33:34), dt(35), leg(36).
 *  state64: robust27, attitude20, residual3, agreement3, causal1,
 *  previous rotor6, previous projected force3, leg1 (details in packState).
 *  pending70: prediction54 then velocity3/nominal3/frame9/leg1.
 *  request19: required1, features17, mean_scale1.
 *  Optional closed5 contains precontrol evidence for gpenmpcCoordinatedOuterStep:
 *  available/closed/innovation/hard_invalid/trust. The caller installs it
 *  separately from the pending prediction in scaffold70.
 *
 * Arguments    : e_gpenmpcNative_canonicalLocalIn *SD
 *                const double input36[36]
 *                const unsigned long long inputTags2[2]
 *                double next64[64]
 *                double kernel61[61]
 *                double scaffold70[70]
 *                double request19[19]
 * Return Type  : void
 */
void canonicalLocalInnerFixedAbi(e_gpenmpcNative_canonicalLocalIn *SD,
                                 const double input36[36],
                                 const unsigned long long inputTags2[2],
                                 double next64[64], double kernel61[61],
                                 double scaffold70[70], double request19[19])
{
  double input_rotor_thrust_state_n[6];
  double next_previous_rotor_command_n[6];
  double c_input_reference_up_accelerati[3];
  double c_next_previous_desired_force_p[3];
  double input_reference_up_jerk_mps3[3];
  double input_reference_up_position_m[3];
  double input_reference_up_velocity_mps[3];
  double c_next_attitude_continuity_stat;
  double next_leg_index;
  int i;
  memcpy(&SD->u4.f6.input_x13[0], &input36[0], 13U * sizeof(double));
  for (i = 0; i < 6; i++) {
    input_rotor_thrust_state_n[i] = input36[i + 13];
  }
  double input_wind_estimate_xy_mps[2];
  input_reference_up_position_m[0] = input36[19];
  input_reference_up_velocity_mps[0] = input36[22];
  c_input_reference_up_accelerati[0] = input36[25];
  input_reference_up_jerk_mps3[0] = input36[28];
  input_reference_up_position_m[1] = input36[20];
  input_reference_up_velocity_mps[1] = input36[23];
  c_input_reference_up_accelerati[1] = input36[26];
  input_reference_up_jerk_mps3[1] = input36[29];
  input_reference_up_position_m[2] = input36[21];
  input_reference_up_velocity_mps[2] = input36[24];
  c_input_reference_up_accelerati[2] = input36[27];
  input_reference_up_jerk_mps3[2] = input36[30];
  input_wind_estimate_xy_mps[0] = input36[32];
  input_wind_estimate_xy_mps[1] = input36[33];
  /*  Same original fresh-owner initializer, typed before entering precontrol.
   */
  /*  The two fixed zero buffers replace only empty placeholders: precontrol's
   */
  /*  explicit new-leg branch overwrites both before any numerical read. */
  canonicalLocalInnerPreControl(
      SD, SD->u4.f6.input_x13, input_rotor_thrust_state_n,
      input_reference_up_position_m, input_reference_up_velocity_mps,
      c_input_reference_up_accelerati, input_reference_up_jerk_mps3,
      input36[31], input_wind_estimate_xy_mps, input36[34], input36[35],
      inputTags2[0], inputTags2[1], &SD->u4.f6.candidate, &SD->u4.f6.out);
  for (i = 0; i < 6; i++) {
    input_rotor_thrust_state_n[i] = 60.0 * (double)i;
  }
  boolean_T d_next_attitude_continuity_stat;
  d_next_attitude_continuity_stat = canonicalLocalInnerPostControl(
      &SD->u4.f6.candidate, &SD->u4.f6.out, input_rotor_thrust_state_n, dv,
      &SD->u4.f6.next_robust_state, SD->u4.f6.c_next_attitude_continuity_stat,
      input_reference_up_position_m, input_reference_up_velocity_mps,
      &c_next_attitude_continuity_stat, c_input_reference_up_accelerati,
      input_reference_up_jerk_mps3, next_previous_rotor_command_n,
      c_next_previous_desired_force_p, &next_leg_index, &SD->u4.f6.scaffold,
      &SD->u4.f6.query);
  next64[0] = SD->u4.f6.next_robust_state.filtered_compensation_i_mps2[0];
  next64[1] = SD->u4.f6.next_robust_state.filtered_compensation_i_mps2[1];
  next64[2] = SD->u4.f6.next_robust_state.filtered_compensation_i_mps2[2];
  next64[3] = SD->u4.f6.next_robust_state.authority_scale;
  next64[4] = SD->u4.f6.next_robust_state.last_tangent_xy[0];
  next64[5] = SD->u4.f6.next_robust_state.last_tangent_xy[1];
  next64[6] = SD->u4.f6.next_robust_state.vertical_disturbance_ewma_mps2;
  next64[7] = SD->u4.f6.next_robust_state.c_vertical_observer_previous_ve;
  next64[8] = SD->u4.f6.next_robust_state.c_vertical_observer_previous_no;
  next64[9] = SD->u4.f6.next_robust_state.c_vertical_observer_previous_ti;
  next64[10] = SD->u4.f6.next_robust_state.c_vertical_observer_previous_le;
  next64[11] = SD->u4.f6.next_robust_state.c_vertical_observer_previous_pa;
  next64[12] = 1.0;
  next64[13] = SD->u4.f6.next_robust_state.c_vertical_observer_update_enab;
  next64[14] = 1.0;
  next64[15] = SD->u4.f6.next_robust_state.c_vertical_observer_antiwindup_;
  next64[19] = 0.0;
  next64[20] = 0.0;
  next64[21] = 0.0;
  next64[22] = 0.0;
  next64[16] = SD->u4.f6.next_robust_state.responsibility_innovation_ratio_ewma_f[0];
  next64[23] = SD->u4.f6.next_robust_state.responsibility_filtered_gp_mean_f_mps2[0];
  next64[17] = SD->u4.f6.next_robust_state.responsibility_innovation_ratio_ewma_f[1];
  next64[24] = SD->u4.f6.next_robust_state.responsibility_filtered_gp_mean_f_mps2[1];
  next64[18] = SD->u4.f6.next_robust_state.responsibility_innovation_ratio_ewma_f[2];
  next64[25] = SD->u4.f6.next_robust_state.responsibility_filtered_gp_mean_f_mps2[2];
  next64[26] = 0.0;
  next64[27] = 1.0;
  next64[28] = d_next_attitude_continuity_stat;
  next64[29] = 0.0;
  memcpy(&next64[30], &SD->u4.f6.c_next_attitude_continuity_stat[0],
         9U * sizeof(double));
  next64[45] = 1.0;
  next64[46] = c_next_attitude_continuity_stat;
  next64[39] = input_reference_up_position_m[0];
  next64[42] = 0.0;
  next64[47] = c_input_reference_up_accelerati[0];
  next64[50] = input_reference_up_jerk_mps3[0];
  next64[40] = input_reference_up_position_m[1];
  next64[43] = 0.0;
  next64[48] = c_input_reference_up_accelerati[1];
  next64[51] = input_reference_up_jerk_mps3[1];
  next64[41] = input_reference_up_position_m[2];
  next64[44] = 0.0;
  next64[49] = c_input_reference_up_accelerati[2];
  next64[52] = input_reference_up_jerk_mps3[2];
  next64[53] = 0.0;
  for (i = 0; i < 6; i++) {
    next64[i + 54] = next_previous_rotor_command_n[i];
  }
  next64[60] = c_next_previous_desired_force_p[0];
  next64[61] = c_next_previous_desired_force_p[1];
  next64[62] = c_next_previous_desired_force_p[2];
  next64[63] = next_leg_index;
  memcpy(&kernel61[0], &SD->u4.f6.out.kernel_output61[0], 61U * sizeof(double));
  scaffold70[0] = 1.0;
  scaffold70[1] = 0.0;
  scaffold70[2] = 0.0;
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
  scaffold70[54] = SD->u4.f6.scaffold.velocity_up_mps[0];
  scaffold70[57] = SD->u4.f6.scaffold.nominal_acceleration_up_mps2[0];
  scaffold70[36] = rtNaN;
  scaffold70[39] = rtNaN;
  scaffold70[42] = rtNaN;
  scaffold70[47] = rtNaN;
  scaffold70[50] = rtNaN;
  scaffold70[55] = SD->u4.f6.scaffold.velocity_up_mps[1];
  scaffold70[58] = SD->u4.f6.scaffold.nominal_acceleration_up_mps2[1];
  scaffold70[37] = rtNaN;
  scaffold70[40] = rtNaN;
  scaffold70[43] = rtNaN;
  scaffold70[48] = rtNaN;
  scaffold70[51] = rtNaN;
  scaffold70[56] = SD->u4.f6.scaffold.velocity_up_mps[2];
  scaffold70[59] = SD->u4.f6.scaffold.nominal_acceleration_up_mps2[2];
  memcpy(&scaffold70[26], &SD->u4.f6.query.pending.gp_frame_i_from_f[0],
         9U * sizeof(double));
  memcpy(&scaffold70[60], &SD->u4.f6.scaffold.frame_i_from_f[0],
         9U * sizeof(double));
  scaffold70[69] = SD->u4.f6.scaffold.leg_index;
  request19[0] = 0.0;
  for (i = 0; i < 17; i++) {
    request19[i + 1] = rtNaN;
  }
  request19[18] = rtNaN;
  /*  Read-only CURRENT closed/execution diagnostics. Do not infer physical */
  /*  authority from the next open GP request, aggregate trust or labels. */
}

/*
 * Arguments    : const double v[70]
 *                const unsigned long long t[2]
 *                m_struct_T *p_prediction
 *                double p_velocity_up_mps[3]
 *                double p_nominal_acceleration_up_mps2[3]
 *                double p_frame_i_from_f[9]
 *                unsigned long long *p_timestamp_ns
 *                unsigned long long *p_generation
 * Return Type  : double
 */
double unpackPending(const double v[70], const unsigned long long t[2],
                     m_struct_T *p_prediction, double p_velocity_up_mps[3],
                     double p_nominal_acceleration_up_mps2[3],
                     double p_frame_i_from_f[9],
                     unsigned long long *p_timestamp_ns,
                     unsigned long long *p_generation)
{
  static const char b_cv[42] = {
      'G', 'P', 'E', 'N', 'M', 'P', 'C', '_', 'C', 'U', 'R', 'R', 'E', 'N', 'T',
      '_', 'S', 'A', 'M', 'P', 'L', 'E', '_', 'G', 'P', '_', 'P', 'R', 'E', 'D',
      'I', 'C', 'T', 'I', 'O', 'N', '_', 'V', '1', '\0', '\0', '\0'};
  double p_leg_index;
  int i;
  for (i = 0; i < 42; i++) {
    p_prediction->schema.Value.data[i] = b_cv[i];
  }
  p_prediction->read_only = (v[0] != 0.0);
  p_prediction->available = (v[1] != 0.0);
  p_prediction->causal_valid = (v[2] != 0.0);
  p_prediction->gp_model_available = (v[3] != 0.0);
  p_prediction->hard_invalid = (v[4] != 0.0);
  p_prediction->trust = v[5];
  p_prediction->minimum_soft_trust = v[6];
  p_prediction->support_distance = v[7];
  p_prediction->latent_variance_max = v[8];
  memcpy(&p_prediction->features_f17[0], &v[9], 17U * sizeof(double));
  p_prediction->observed_innovation_available = (v[45] != 0.0);
  p_prediction->observed_innovation_consistent = (v[52] != 0.0);
  /*  The close function writes these fixed-shape scratch fields when */
  /*  observation evidence is available. The availability flag selects */
  /*  the update or fallback branch. */
  p_prediction->predicted_mean_f_mps2[0] = v[35];
  p_prediction->runtime_weighted_mean_f_mps2[0] = v[38];
  p_prediction->calibrated_half_width_f_mps2[0] = v[41];
  p_prediction->observed_innovation_f_mps2[0] = v[46];
  p_prediction->innovation_error_f_mps2[0] = v[49];
  p_prediction->innovation_consistency_score_f[0] = rtNaN;
  p_prediction->c_observed_innovation_consisten[0] = false;
  p_velocity_up_mps[0] = v[54];
  p_nominal_acceleration_up_mps2[0] = v[57];
  p_prediction->predicted_mean_f_mps2[1] = v[36];
  p_prediction->runtime_weighted_mean_f_mps2[1] = v[39];
  p_prediction->calibrated_half_width_f_mps2[1] = v[42];
  p_prediction->observed_innovation_f_mps2[1] = v[47];
  p_prediction->innovation_error_f_mps2[1] = v[50];
  p_prediction->innovation_consistency_score_f[1] = rtNaN;
  p_prediction->c_observed_innovation_consisten[1] = false;
  p_velocity_up_mps[1] = v[55];
  p_nominal_acceleration_up_mps2[1] = v[58];
  p_prediction->predicted_mean_f_mps2[2] = v[37];
  p_prediction->runtime_weighted_mean_f_mps2[2] = v[40];
  p_prediction->calibrated_half_width_f_mps2[2] = v[43];
  p_prediction->observed_innovation_f_mps2[2] = v[48];
  p_prediction->innovation_error_f_mps2[2] = v[51];
  p_prediction->innovation_consistency_score_f[2] = rtNaN;
  p_prediction->c_observed_innovation_consisten[2] = false;
  p_velocity_up_mps[2] = v[56];
  p_nominal_acceleration_up_mps2[2] = v[59];
  memcpy(&p_prediction->gp_frame_i_from_f[0], &v[26], 9U * sizeof(double));
  memcpy(&p_frame_i_from_f[0], &v[60], 9U * sizeof(double));
  p_leg_index = v[69];
  *p_timestamp_ns = t[0];
  *p_generation = t[1];
  return p_leg_index;
}

/*
 * Arguments    : const double v[64]
 *                const unsigned long long t[2]
 *                i_struct_T *s_robust_state
 *                o_struct_T *s_attitude_continuity_state
 *                double s_residual_history_f_mps2[3]
 *                double s_gp_agreement_weight_f[3]
 *                double s_previous_rotor_command_n[6]
 *                double c_s_previous_desired_force_proj[3]
 *                double *s_leg_index
 *                unsigned long long *s_last_sample_timestamp_ns
 *                unsigned long long *s_source_generation
 * Return Type  : boolean_T
 */
boolean_T unpackState(const double v[64], const unsigned long long t[2],
                      i_struct_T *s_robust_state,
                      o_struct_T *s_attitude_continuity_state,
                      double s_residual_history_f_mps2[3],
                      double s_gp_agreement_weight_f[3],
                      double s_previous_rotor_command_n[6],
                      double c_s_previous_desired_force_proj[3],
                      double *s_leg_index,
                      unsigned long long *s_last_sample_timestamp_ns,
                      unsigned long long *s_source_generation)
{
  int i;
  boolean_T s_causal_valid;
  s_robust_state->filtered_compensation_i_mps2[0] = v[0];
  s_robust_state->filtered_compensation_i_mps2[1] = v[1];
  s_robust_state->filtered_compensation_i_mps2[2] = v[2];
  s_robust_state->authority_scale = v[3];
  s_robust_state->last_tangent_xy[0] = v[4];
  s_robust_state->last_tangent_xy[1] = v[5];
  s_robust_state->vertical_disturbance_ewma_mps2 = v[6];
  s_robust_state->c_vertical_observer_previous_ve = v[7];
  s_robust_state->c_vertical_observer_previous_no = v[8];
  s_robust_state->c_vertical_observer_previous_ti = v[9];
  s_robust_state->c_vertical_observer_previous_le = v[10];
  s_robust_state->c_vertical_observer_previous_pa = v[11];
  s_robust_state->c_vertical_observer_observation = (v[12] != 0.0);
  s_robust_state->c_vertical_observer_update_enab = (v[13] != 0.0);
  s_robust_state->vertical_observer_reset_count = v[14];
  s_robust_state->c_vertical_observer_antiwindup_ = v[15];
  s_robust_state->gp_responsibility_mode_active = (v[19] != 0.0);
  s_robust_state->c_gp_responsibility_enter_elapsed = v[20];
  s_robust_state->c_gp_responsibility_exit_elapsed = v[21];
  s_robust_state->gp_responsibility_blend = v[22];
  s_robust_state->responsibility_innovation_ratio_ewma_f[0] = v[16];
  s_robust_state->responsibility_filtered_gp_mean_f_mps2[0] = v[23];
  s_robust_state->responsibility_innovation_ratio_ewma_f[1] = v[17];
  s_robust_state->responsibility_filtered_gp_mean_f_mps2[1] = v[24];
  s_robust_state->responsibility_innovation_ratio_ewma_f[2] = v[18];
  s_robust_state->responsibility_filtered_gp_mean_f_mps2[2] = v[25];
  s_robust_state->gp_responsibility_mode_transition_count = v[26];
  s_attitude_continuity_state->initialized = (v[27] != 0.0);
  s_attitude_continuity_state->angular_velocity_valid = (v[28] != 0.0);
  s_attitude_continuity_state->angular_acceleration_valid = (v[29] != 0.0);
  memcpy(&s_attitude_continuity_state->filtered_rotation[0], &v[30],
         9U * sizeof(double));
  s_attitude_continuity_state->update_count = v[45];
  s_attitude_continuity_state->reset_count = v[46];
  s_attitude_continuity_state->c_desired_angular_velocity_body[0] = v[39];
  s_attitude_continuity_state->c_desired_angular_acceleration_[0] = v[42];
  s_residual_history_f_mps2[0] = v[47];
  s_gp_agreement_weight_f[0] = v[50];
  s_attitude_continuity_state->c_desired_angular_velocity_body[1] = v[40];
  s_attitude_continuity_state->c_desired_angular_acceleration_[1] = v[43];
  s_residual_history_f_mps2[1] = v[48];
  s_gp_agreement_weight_f[1] = v[51];
  s_attitude_continuity_state->c_desired_angular_velocity_body[2] = v[41];
  s_attitude_continuity_state->c_desired_angular_acceleration_[2] = v[44];
  s_residual_history_f_mps2[2] = v[49];
  s_gp_agreement_weight_f[2] = v[52];
  s_causal_valid = (v[53] != 0.0);
  for (i = 0; i < 6; i++) {
    s_previous_rotor_command_n[i] = v[i + 54];
  }
  c_s_previous_desired_force_proj[0] = v[60];
  c_s_previous_desired_force_proj[1] = v[61];
  c_s_previous_desired_force_proj[2] = v[62];
  *s_leg_index = v[63];
  *s_last_sample_timestamp_ns = t[0];
  *s_source_generation = t[1];
  return s_causal_valid;
}

/*
 * File trailer for canonicalLocalInnerFixedAbi.c
 *
 * [EOF]
 */
