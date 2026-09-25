/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: canonicalLocalInnerPostControl.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "canonicalLocalInnerPostControl.h"
#include "norm.h"
#include "prepareCanonicalCurrentGpPrediction.h"
#include "gpenmpcM600Allocation.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
static const double dv1[3] = {0.0, 0.0, 9.80665};

/* Function Definitions */
/*
 * CANONICALLOCALINNERPOSTCONTROL Numerical updates after a successful output commit.
 *  Pure math only: this function does NOT observe or confirm any publication.
 *  The real owner must call/install it only after successful local output
 *  commit. No HOST acknowledgment is substituted for a board output receipt.
 *
 *  The GP request uses the PREVIOUS rotor command and the CURRENT projected
 *  force, exactly as CurrentPhysicalCausalRuntime.commitControl. GP inference
 *  stays separate. pendingScaffold intentionally lacks a prediction field;
 *  next-sample closure cannot consume it before completeCanonical... returns.
 *
 * Arguments    : const s_struct_T *candidate
 *                const q_struct_T *precontrol
 *                const double c_numerics_calibration_rotor_al[6]
 *                const double d_numerics_calibration_rotor_al[6]
 *                i_struct_T *nextState_robust_state
 *                o_struct_T *c_nextState_attitude_continuity
 *                double c_nextState_residual_history_f_[3]
 *                double nextState_gp_agreement_weight_f[3]
 *                double c_nextState_previous_rotor_comm[6]
 *                double c_nextState_previous_desired_fo[3]
 *                double *nextState_leg_index
 *                unsigned long long *c_nextState_last_sample_timesta
 *                unsigned long long *nextState_source_generation
 *                d_struct_T *pendingScaffold
 *                f_struct_T *gpRequest
 * Return Type  : boolean_T
 */
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
    d_struct_T *pendingScaffold, f_struct_T *gpRequest)
{
  double allocation_matrix[24];
  double b_expl_temp[24];
  double plantState[19];
  double d_expl_temp[9];
  double b_allocation_matrix[4];
  double y[3];
  double c_expl_temp;
  double d;
  double expl_temp;
  double q_idx_3;
  int i;
  boolean_T nextState_causal_valid;
  *c_nextState_attitude_continuity = candidate->attitude_continuity_state;
  c_nextState_residual_history_f_[0] = candidate->residual_history_f_mps2[0];
  nextState_gp_agreement_weight_f[0] = candidate->gp_agreement_weight_f[0];
  c_nextState_residual_history_f_[1] = candidate->residual_history_f_mps2[1];
  nextState_gp_agreement_weight_f[1] = candidate->gp_agreement_weight_f[1];
  c_nextState_residual_history_f_[2] = candidate->residual_history_f_mps2[2];
  nextState_gp_agreement_weight_f[2] = candidate->gp_agreement_weight_f[2];
  nextState_causal_valid = candidate->causal_valid;
  *nextState_leg_index = candidate->leg_index;
  *c_nextState_last_sample_timesta = candidate->last_sample_timestamp_ns;
  *nextState_source_generation = candidate->source_generation;
  *nextState_robust_state = candidate->robust_state;
  /* GPENMPCROBUSTSE3OBSERVECONTROL Update shared fail-soft authority memory. */
  if (precontrol->kernel_control_candidate.rotor_saturated) {
    expl_temp = 0.35;
  } else if (precontrol->kernel_control_candidate
                 .c_force_projection_norm_mismatc > 1.0E-6) {
    expl_temp = 0.65;
  } else {
    expl_temp = 1.0;
  }
  nextState_robust_state->authority_scale =
      fmin(fmax(candidate->robust_state.authority_scale +
                    (1.0 - exp(-fmax(precontrol->input.dt_s, 1.0E-6) / 0.2)) *
                        (expl_temp - candidate->robust_state.authority_scale),
                0.25),
           1.0);
  if (!precontrol->kernel_control_candidate.rotor_saturated &&
      (precontrol->kernel_control_candidate.c_force_projection_norm_mismatc <=
       1.0E-6)) {
    nextState_robust_state->c_vertical_observer_update_enab = true;
  } else {
    nextState_robust_state->c_vertical_observer_update_enab = false;
  }
  if (precontrol->kernel_control_candidate.rotor_saturated) {
    nextState_robust_state->filtered_compensation_i_mps2[0] =
        0.8 * candidate->robust_state.filtered_compensation_i_mps2[0];
    nextState_robust_state->filtered_compensation_i_mps2[1] =
        0.8 * candidate->robust_state.filtered_compensation_i_mps2[1];
    nextState_robust_state->filtered_compensation_i_mps2[2] =
        0.8 * candidate->robust_state.filtered_compensation_i_mps2[2];
  }
  memcpy(&plantState[0], &precontrol->input.x13[0], 13U * sizeof(double));
  /* GPENMPCKNOWNNOMINALACCELERATIONFROMOBSERVATION Causal nominal-model output.
   */
  /*  */
  /*  This helper reads only the current estimated/observed state, retained
   * rotor */
  /*  thrust states, the available wind estimate and known payload.  It does not
   */
  /*  read software-plant private mismatch, disturbance or actual acceleration.
   */
  gpenmpcM600Allocation(c_numerics_calibration_rotor_al,
                       d_numerics_calibration_rotor_al, allocation_matrix,
                       b_expl_temp, &expl_temp, &c_expl_temp, d_expl_temp,
                       c_nextState_previous_desired_fo, &q_idx_3);
  /* GPENMPCQUATERNIONROTATION Convert a scalar-first unit quaternion to SO(3).
   */
  c_nextState_previous_desired_fo[0] =
      plantState[3] - precontrol->input.wind_estimate_xy_mps[0];
  c_nextState_previous_desired_fo[1] =
      plantState[4] - precontrol->input.wind_estimate_xy_mps[1];
  c_nextState_previous_desired_fo[2] = plantState[5];
  y[0] = fabs(c_nextState_previous_desired_fo[0]);
  y[1] = fabs(c_nextState_previous_desired_fo[1]);
  y[2] = fabs(plantState[5]);
  memset(&b_allocation_matrix[0], 0, sizeof(double) << 2);
  d = b_allocation_matrix[0];
  for (i = 0; i < 6; i++) {
    expl_temp = precontrol->input.rotor_thrust_state_n[i];
    plantState[i + 13] = expl_temp;
    d += allocation_matrix[i << 2] * expl_temp;
  }
  double b_expl_temp_tmp;
  double c_expl_temp_tmp;
  double d_expl_temp_tmp;
  double e_expl_temp_tmp;
  double expl_temp_tmp;
  double q_idx_0;
  double q_idx_1;
  double q_idx_2;
  expl_temp = fmax(d_norm(&plantState[6]), 1.0E-15);
  q_idx_0 = plantState[6] / expl_temp;
  q_idx_1 = plantState[7] / expl_temp;
  q_idx_2 = plantState[8] / expl_temp;
  q_idx_3 = plantState[9] / expl_temp;
  expl_temp_tmp = q_idx_3 * q_idx_3;
  b_expl_temp_tmp = q_idx_2 * q_idx_2;
  d_expl_temp[0] = 1.0 - 2.0 * (b_expl_temp_tmp + expl_temp_tmp);
  expl_temp = q_idx_1 * q_idx_2;
  c_expl_temp = q_idx_0 * q_idx_3;
  d_expl_temp[3] = 2.0 * (expl_temp - c_expl_temp);
  c_expl_temp_tmp = q_idx_1 * q_idx_3;
  d_expl_temp_tmp = q_idx_0 * q_idx_2;
  d_expl_temp[6] = 2.0 * (c_expl_temp_tmp + d_expl_temp_tmp);
  d_expl_temp[1] = 2.0 * (expl_temp + c_expl_temp);
  e_expl_temp_tmp = q_idx_1 * q_idx_1;
  d_expl_temp[4] = 1.0 - 2.0 * (e_expl_temp_tmp + expl_temp_tmp);
  c_expl_temp = q_idx_2 * q_idx_3;
  expl_temp = q_idx_0 * q_idx_1;
  d_expl_temp[7] = 2.0 * (c_expl_temp - expl_temp);
  d_expl_temp[2] = 2.0 * (c_expl_temp_tmp - d_expl_temp_tmp);
  d_expl_temp[5] = 2.0 * (c_expl_temp + expl_temp);
  d_expl_temp[8] = 1.0 - 2.0 * (e_expl_temp_tmp + b_expl_temp_tmp);
  nextState_robust_state->c_vertical_observer_previous_ti =
      (double)precontrol->input.source_timestamp_ns * 1.0E-9;
  nextState_robust_state->c_vertical_observer_previous_le =
      precontrol->input.leg_index;
  nextState_robust_state->c_vertical_observer_previous_pa =
      precontrol->input.payload_kg;
  /* GPENMPCCOMMITCAUSALVERTICALDISTURBANCEOBSERVER Retain sample k for k+1. */
  nextState_robust_state->c_vertical_observer_previous_ve =
      precontrol->input.x13[5];
  nextState_robust_state->c_vertical_observer_observation = true;
  gpRequest->prediction_required = c_prepareCanonicalCurrentGpPred(
      &precontrol->input.x13[3], precontrol->input.reference_up.velocity_mps,
      precontrol->input.reference_up.acceleration_mps2,
      precontrol->input.wind_estimate_xy_mps, precontrol->input.payload_kg,
      precontrol->kernel_control_candidate.desired_force_projected_up_n,
      candidate->previous_rotor_command_n, candidate->residual_history_f_mps2,
      candidate->causal_valid, gpRequest->schema.Value, &gpRequest->pending,
      gpRequest->features_f17, &gpRequest->gp_mean_scale);
  for (i = 0; i < 3; i++) {
    pendingScaffold->nominal_acceleration_up_mps2[i] =
        (((d_expl_temp[i] * 0.0 + d_expl_temp[i + 3] * 0.0) +
          d_expl_temp[i + 6] * d) -
         0.0634905529323215 * c_nextState_previous_desired_fo[i] * y[i]) /
            (precontrol->input.payload_kg + 9.5) -
        dv1[i];
    pendingScaffold->velocity_up_mps[i] = precontrol->input.x13[i + 3];
  }
  nextState_robust_state->c_vertical_observer_previous_no =
      pendingScaffold->nominal_acceleration_up_mps2[2];
  memcpy(&pendingScaffold->frame_i_from_f[0],
         &gpRequest->pending.gp_frame_i_from_f[0], 9U * sizeof(double));
  pendingScaffold->leg_index = precontrol->input.leg_index;
  for (i = 0; i < 6; i++) {
    c_nextState_previous_rotor_comm[i] =
        precontrol->kernel_control_candidate.rotor_command_n[i];
  }
  c_nextState_previous_desired_fo[0] =
      precontrol->kernel_control_candidate.desired_force_projected_up_n[0];
  c_nextState_previous_desired_fo[1] =
      precontrol->kernel_control_candidate.desired_force_projected_up_n[1];
  c_nextState_previous_desired_fo[2] =
      precontrol->kernel_control_candidate.desired_force_projected_up_n[2];
  return nextState_causal_valid;
}

/*
 * CANONICALLOCALINNERPOSTCONTROL Numerical updates after a successful output commit.
 *  Pure math only: this function does NOT observe or confirm any publication.
 *  The real owner must call/install it only after successful local output
 *  commit. No HOST acknowledgment is substituted for a board output receipt.
 *
 *  The GP request uses the PREVIOUS rotor command and the CURRENT projected
 *  force, exactly as CurrentPhysicalCausalRuntime.commitControl. GP inference
 *  stays separate. pendingScaffold intentionally lacks a prediction field;
 *  next-sample closure cannot consume it before completeCanonical... returns.
 *
 * Arguments    : const p_struct_T *candidate
 *                const l_struct_T *precontrol
 *                const double c_numerics_calibration_rotor_al[6]
 *                const double d_numerics_calibration_rotor_al[6]
 *                i_struct_T *nextState_robust_state
 *                double c_nextState_attitude_continuity[9]
 *                double d_nextState_attitude_continuity[3]
 *                double e_nextState_attitude_continuity[3]
 *                double *f_nextState_attitude_continuity
 *                double c_nextState_residual_history_f_[3]
 *                double nextState_gp_agreement_weight_f[3]
 *                double c_nextState_previous_rotor_comm[6]
 *                double c_nextState_previous_desired_fo[3]
 *                double *nextState_leg_index
 *                d_struct_T *pendingScaffold
 *                f_struct_T *gpRequest
 * Return Type  : boolean_T
 */
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
    d_struct_T *pendingScaffold, f_struct_T *gpRequest)
{
  double allocation_matrix[24];
  double b_expl_temp[24];
  double plantState[19];
  double d_expl_temp[9];
  double b_allocation_matrix[4];
  double y[3];
  double c_expl_temp;
  double d;
  double expl_temp;
  double q_idx_3;
  int i;
  boolean_T g_nextState_attitude_continuity;
  g_nextState_attitude_continuity =
      candidate->attitude_continuity_state.angular_velocity_valid;
  memcpy(&c_nextState_attitude_continuity[0],
         &candidate->attitude_continuity_state.filtered_rotation[0],
         9U * sizeof(double));
  *f_nextState_attitude_continuity =
      candidate->attitude_continuity_state.reset_count;
  d_nextState_attitude_continuity[0] =
      candidate->attitude_continuity_state.c_desired_angular_velocity_body[0];
  e_nextState_attitude_continuity[0] = 0.0;
  c_nextState_residual_history_f_[0] = candidate->residual_history_f_mps2[0];
  nextState_gp_agreement_weight_f[0] = candidate->gp_agreement_weight_f[0];
  d_nextState_attitude_continuity[1] =
      candidate->attitude_continuity_state.c_desired_angular_velocity_body[1];
  e_nextState_attitude_continuity[1] = 0.0;
  c_nextState_residual_history_f_[1] = candidate->residual_history_f_mps2[1];
  nextState_gp_agreement_weight_f[1] = candidate->gp_agreement_weight_f[1];
  d_nextState_attitude_continuity[2] =
      candidate->attitude_continuity_state.c_desired_angular_velocity_body[2];
  e_nextState_attitude_continuity[2] = 0.0;
  c_nextState_residual_history_f_[2] = candidate->residual_history_f_mps2[2];
  nextState_gp_agreement_weight_f[2] = candidate->gp_agreement_weight_f[2];
  *nextState_leg_index = candidate->leg_index;
  *nextState_robust_state = candidate->robust_state;
  /* GPENMPCROBUSTSE3OBSERVECONTROL Update shared fail-soft authority memory. */
  if (precontrol->kernel_control_candidate.rotor_saturated) {
    expl_temp = 0.35;
  } else if (precontrol->kernel_control_candidate
                 .c_force_projection_norm_mismatc > 1.0E-6) {
    expl_temp = 0.65;
  } else {
    expl_temp = 1.0;
  }
  nextState_robust_state->authority_scale =
      fmin(fmax(candidate->robust_state.authority_scale +
                    (1.0 - exp(-fmax(precontrol->input.dt_s, 1.0E-6) / 0.2)) *
                        (expl_temp - candidate->robust_state.authority_scale),
                0.25),
           1.0);
  if (!precontrol->kernel_control_candidate.rotor_saturated &&
      (precontrol->kernel_control_candidate.c_force_projection_norm_mismatc <=
       1.0E-6)) {
    nextState_robust_state->c_vertical_observer_update_enab = true;
  } else {
    nextState_robust_state->c_vertical_observer_update_enab = false;
  }
  if (precontrol->kernel_control_candidate.rotor_saturated) {
    nextState_robust_state->filtered_compensation_i_mps2[0] =
        0.8 * candidate->robust_state.filtered_compensation_i_mps2[0];
    nextState_robust_state->filtered_compensation_i_mps2[1] =
        0.8 * candidate->robust_state.filtered_compensation_i_mps2[1];
    nextState_robust_state->filtered_compensation_i_mps2[2] =
        0.8 * candidate->robust_state.filtered_compensation_i_mps2[2];
  }
  memcpy(&plantState[0], &precontrol->input.x13[0], 13U * sizeof(double));
  /* GPENMPCKNOWNNOMINALACCELERATIONFROMOBSERVATION Causal nominal-model output.
   */
  /*  */
  /*  This helper reads only the current estimated/observed state, retained
   * rotor */
  /*  thrust states, the available wind estimate and known payload.  It does not
   */
  /*  read software-plant private mismatch, disturbance or actual acceleration.
   */
  gpenmpcM600Allocation(c_numerics_calibration_rotor_al,
                       d_numerics_calibration_rotor_al, allocation_matrix,
                       b_expl_temp, &expl_temp, &c_expl_temp, d_expl_temp,
                       c_nextState_previous_desired_fo, &q_idx_3);
  /* GPENMPCQUATERNIONROTATION Convert a scalar-first unit quaternion to SO(3).
   */
  c_nextState_previous_desired_fo[0] =
      plantState[3] - precontrol->input.wind_estimate_xy_mps[0];
  c_nextState_previous_desired_fo[1] =
      plantState[4] - precontrol->input.wind_estimate_xy_mps[1];
  c_nextState_previous_desired_fo[2] = plantState[5];
  y[0] = fabs(c_nextState_previous_desired_fo[0]);
  y[1] = fabs(c_nextState_previous_desired_fo[1]);
  y[2] = fabs(plantState[5]);
  memset(&b_allocation_matrix[0], 0, sizeof(double) << 2);
  d = b_allocation_matrix[0];
  for (i = 0; i < 6; i++) {
    expl_temp = precontrol->input.rotor_thrust_state_n[i];
    plantState[i + 13] = expl_temp;
    d += allocation_matrix[i << 2] * expl_temp;
  }
  double b_expl_temp_tmp;
  double c_expl_temp_tmp;
  double d_expl_temp_tmp;
  double e_expl_temp_tmp;
  double expl_temp_tmp;
  double q_idx_0;
  double q_idx_1;
  double q_idx_2;
  expl_temp = fmax(d_norm(&plantState[6]), 1.0E-15);
  q_idx_0 = plantState[6] / expl_temp;
  q_idx_1 = plantState[7] / expl_temp;
  q_idx_2 = plantState[8] / expl_temp;
  q_idx_3 = plantState[9] / expl_temp;
  expl_temp_tmp = q_idx_3 * q_idx_3;
  b_expl_temp_tmp = q_idx_2 * q_idx_2;
  d_expl_temp[0] = 1.0 - 2.0 * (b_expl_temp_tmp + expl_temp_tmp);
  expl_temp = q_idx_1 * q_idx_2;
  c_expl_temp = q_idx_0 * q_idx_3;
  d_expl_temp[3] = 2.0 * (expl_temp - c_expl_temp);
  c_expl_temp_tmp = q_idx_1 * q_idx_3;
  d_expl_temp_tmp = q_idx_0 * q_idx_2;
  d_expl_temp[6] = 2.0 * (c_expl_temp_tmp + d_expl_temp_tmp);
  d_expl_temp[1] = 2.0 * (expl_temp + c_expl_temp);
  e_expl_temp_tmp = q_idx_1 * q_idx_1;
  d_expl_temp[4] = 1.0 - 2.0 * (e_expl_temp_tmp + expl_temp_tmp);
  c_expl_temp = q_idx_2 * q_idx_3;
  expl_temp = q_idx_0 * q_idx_1;
  d_expl_temp[7] = 2.0 * (c_expl_temp - expl_temp);
  d_expl_temp[2] = 2.0 * (c_expl_temp_tmp - d_expl_temp_tmp);
  d_expl_temp[5] = 2.0 * (c_expl_temp + expl_temp);
  d_expl_temp[8] = 1.0 - 2.0 * (e_expl_temp_tmp + b_expl_temp_tmp);
  nextState_robust_state->c_vertical_observer_previous_ti =
      (double)precontrol->input.source_timestamp_ns * 1.0E-9;
  nextState_robust_state->c_vertical_observer_previous_le =
      precontrol->input.leg_index;
  nextState_robust_state->c_vertical_observer_previous_pa =
      precontrol->input.payload_kg;
  /* GPENMPCCOMMITCAUSALVERTICALDISTURBANCEOBSERVER Retain sample k for k+1. */
  nextState_robust_state->c_vertical_observer_previous_ve =
      precontrol->input.x13[5];
  nextState_robust_state->c_vertical_observer_observation = true;
  gpRequest->prediction_required = c_prepareCanonicalCurrentGpPred(
      &precontrol->input.x13[3], precontrol->input.reference_up.velocity_mps,
      precontrol->input.reference_up.acceleration_mps2,
      precontrol->input.wind_estimate_xy_mps, precontrol->input.payload_kg,
      precontrol->kernel_control_candidate.desired_force_projected_up_n,
      candidate->previous_rotor_command_n, candidate->residual_history_f_mps2,
      false, gpRequest->schema.Value, &gpRequest->pending,
      gpRequest->features_f17, &gpRequest->gp_mean_scale);
  for (i = 0; i < 3; i++) {
    pendingScaffold->nominal_acceleration_up_mps2[i] =
        (((d_expl_temp[i] * 0.0 + d_expl_temp[i + 3] * 0.0) +
          d_expl_temp[i + 6] * d) -
         0.0634905529323215 * c_nextState_previous_desired_fo[i] * y[i]) /
            (precontrol->input.payload_kg + 9.5) -
        dv1[i];
    pendingScaffold->velocity_up_mps[i] = precontrol->input.x13[i + 3];
  }
  nextState_robust_state->c_vertical_observer_previous_no =
      pendingScaffold->nominal_acceleration_up_mps2[2];
  memcpy(&pendingScaffold->frame_i_from_f[0],
         &gpRequest->pending.gp_frame_i_from_f[0], 9U * sizeof(double));
  pendingScaffold->leg_index = precontrol->input.leg_index;
  for (i = 0; i < 6; i++) {
    c_nextState_previous_rotor_comm[i] =
        precontrol->kernel_control_candidate.rotor_command_n[i];
  }
  c_nextState_previous_desired_fo[0] =
      precontrol->kernel_control_candidate.desired_force_projected_up_n[0];
  c_nextState_previous_desired_fo[1] =
      precontrol->kernel_control_candidate.desired_force_projected_up_n[1];
  c_nextState_previous_desired_fo[2] =
      precontrol->kernel_control_candidate.desired_force_projected_up_n[2];
  return g_nextState_attitude_continuity;
}

/*
 * File trailer for canonicalLocalInnerPostControl.c
 *
 * [EOF]
 */
