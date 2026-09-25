/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: canonicalLocalInnerPreControl.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "canonicalLocalInnerPreControl.h"
#include "all.h"
#include "gpenmpcAdvanceCausalResidualHistory.h"
#include "gpenmpcCloseGpInnovationEvidence.h"
#include "gpenmpcCoordinatedGpRobustSe3Augmentation.h"
#include "gpenmpcDesiredSe3Command.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "gpenmpcObserveCausalVerticalDisturbance.h"
#include "gpenmpcResetCausalVerticalDisturbanceObserver.h"
#include "gpenmpcUpdateDesiredAttitudeContinuity.h"
#include "gpenmpcUpdateGpAgreementWeight.h"
#include "rt_nonfinite.h"
#include "se3WrenchKernel.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * CANONICALLOCALINNERPRECONTROL Compute a fast-loop numerical candidate.
 *  Inputs combine committed numerical state with the current sample and reference.
 *
 *  INPUT: x13, rotor_thrust_state_n, reference_up (p/v/a/j), payload_kg,
 *  wind_estimate_xy_mps, dt_s, source_timestamp_ns (original uint64),
 *  source_generation (original uint64), leg_index and explicit new_leg.
 *  NUMERICS: robust, enmpc, calibration, profile, attitudeContinuity and
 *  kernelParameters from the current canonical assets; no gp_model is needed.
 *  COMMITTED=[] initializes the original numerical states on a new leg only.
 *  Otherwise it is the prior candidate AFTER the caller's actual control
 *  commit has applied gpenmpcRobustSe3ObserveControl and the original vertical
 *  observer commit. PREVIOUSPENDING is that commit's original Runtime.Pending:
 *  prediction, velocity_up_mps, nominal_acceleration_up_mps2, frame_i_from_f,
 *  generation, timestamp_ns and leg_index. Prediction remains HOST-owned.
 *
 *  The caller must not install CANDIDATE on unsuccessful publication/commit.
 *  kernel_output61 is a numerical candidate. The caller validates commit status
 *  and source identity before use; source tags are copied from the inputs.
 *
 * Arguments    : e_gpenmpcNative_canonicalLocalIn *SD
 *                const i_struct_T *committed_robust_state
 *                const o_struct_T *c_committed_attitude_continuity
 *                const double c_committed_residual_history_f_[3]
 *                const double committed_gp_agreement_weight_f[3]
 *                const double c_committed_previous_rotor_comm[6]
 *                unsigned long long c_committed_last_sample_timesta
 *                const struct_T *input
 *                const m_struct_T *previousPending_prediction
 *                const double previousPending_velocity_up_mps[3]
 *                const double c_previousPending_nominal_accel[3]
 *                const double previousPending_frame_i_from_f[9]
 *                s_struct_T *candidate
 *                q_struct_T *out
 * Return Type  : void
 */
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
    q_struct_T *out)
{
  double w[4];
  double c_executionEvidence_calibrated_[3];
  double c_executionEvidence_predicted_m[3];
  double observedI[3];
  double d;
  double d1;
  double d2;
  double interval;
  int i;
  boolean_T bv[3];
  boolean_T b;
  boolean_T labelValid;
  /*  The generated first/ordinary entries deliberately specialize this branch.
   */
  /*  Interpreted MATLAB retains the exact original value and acceptance set. */
  /*  .002 is the existing physical adapter minimum, .0100001 the existing */
  /*  CurrentPhysicalCausalRuntime maximum; their accepted intersection is kept.
   */
  for (i = 0; i < 6; i++) {
    candidate->previous_rotor_command_n[i] = c_committed_previous_rotor_comm[i];
  }
  memcpy(&out->x19[0], &input->x13[0], 13U * sizeof(double));
  for (i = 0; i < 6; i++) {
    out->x19[i + 13] = input->rotor_thrust_state_n[i];
  }
  interval = ((double)input->source_timestamp_ns -
              (double)c_committed_last_sample_timesta) *
             1.0E-9;
  labelValid = c_gpenmpcAdvanceCausalResidualHi(
      c_committed_residual_history_f_, previousPending_velocity_up_mps,
      &input->x13[3], c_previousPending_nominal_accel,
      previousPending_frame_i_from_f, interval,
      candidate->residual_history_f_mps2, out->augmentation_up_mps2);
  memset(&observedI[0], 0, 3U * sizeof(double));
  d = observedI[0];
  d1 = observedI[1];
  d2 = observedI[2];
  for (i = 0; i < 3; i++) {
    d += previousPending_frame_i_from_f[3 * i] * out->augmentation_up_mps2[i];
    d1 += previousPending_frame_i_from_f[3 * i + 1] *
          out->augmentation_up_mps2[i];
    d2 += previousPending_frame_i_from_f[3 * i + 2] *
          out->augmentation_up_mps2[i];
  }
  observedI[2] = d2;
  observedI[1] = d1;
  observedI[0] = d;
  if (labelValid) {
    bv[0] = (!rtIsInf(d) && !rtIsNaN(d));
    bv[1] = (!rtIsInf(d1) && !rtIsNaN(d1));
    bv[2] = (!rtIsInf(d2) && !rtIsNaN(d2));
    if (all(bv)) {
      b = true;
    } else {
      b = false;
    }
  } else {
    b = false;
  }
  c_gpenmpcCloseGpInnovationEvidence(previousPending_prediction, observedI, b,
                                  &SD->u3.f5.closed);
  gpenmpcUpdateGpAgreementWeight(
      committed_gp_agreement_weight_f, SD->u3.f5.closed.available,
      SD->u3.f5.closed.hard_invalid, SD->u3.f5.closed.trust,
      SD->u3.f5.closed.predicted_mean_f_mps2,
      SD->u3.f5.closed.calibrated_half_width_f_mps2,
      SD->u3.f5.closed.observed_innovation_available,
      SD->u3.f5.closed.observed_innovation_f_mps2, interval,
      candidate->gp_agreement_weight_f, &SD->u3.f5.consistency);
  out->closed_gp_evidence.available = SD->u3.f5.closed.available;
  out->closed_gp_evidence.hard_invalid = SD->u3.f5.closed.hard_invalid;
  out->closed_gp_evidence.trust = SD->u3.f5.closed.trust;
  memcpy(&out->closed_gp_evidence.features_f17[0],
         &SD->u3.f5.closed.features_f17[0], 17U * sizeof(double));
  memcpy(&out->closed_gp_evidence.gp_frame_i_from_f[0],
         &SD->u3.f5.closed.gp_frame_i_from_f[0], 9U * sizeof(double));
  out->closed_gp_evidence.observed_innovation_available =
      SD->u3.f5.closed.observed_innovation_available;
  out->closed_gp_evidence.predicted_mean_f_mps2[0] =
      SD->u3.f5.closed.predicted_mean_f_mps2[0];
  out->closed_gp_evidence.calibrated_half_width_f_mps2[0] =
      SD->u3.f5.closed.calibrated_half_width_f_mps2[0];
  out->closed_gp_evidence.observed_innovation_f_mps2[0] =
      SD->u3.f5.closed.observed_innovation_f_mps2[0];
  out->closed_gp_evidence.predicted_mean_f_mps2[1] =
      SD->u3.f5.closed.predicted_mean_f_mps2[1];
  out->closed_gp_evidence.calibrated_half_width_f_mps2[1] =
      SD->u3.f5.closed.calibrated_half_width_f_mps2[1];
  out->closed_gp_evidence.observed_innovation_f_mps2[1] =
      SD->u3.f5.closed.observed_innovation_f_mps2[1];
  out->closed_gp_evidence.predicted_mean_f_mps2[2] =
      SD->u3.f5.closed.predicted_mean_f_mps2[2];
  out->closed_gp_evidence.calibrated_half_width_f_mps2[2] =
      SD->u3.f5.closed.calibrated_half_width_f_mps2[2];
  out->closed_gp_evidence.observed_innovation_f_mps2[2] =
      SD->u3.f5.closed.observed_innovation_f_mps2[2];
  if (labelValid) {
    bv[0] = (!rtIsInf(candidate->residual_history_f_mps2[0]) &&
             !rtIsNaN(candidate->residual_history_f_mps2[0]));
    bv[1] = (!rtIsInf(candidate->residual_history_f_mps2[1]) &&
             !rtIsNaN(candidate->residual_history_f_mps2[1]));
    bv[2] = (!rtIsInf(candidate->residual_history_f_mps2[2]) &&
             !rtIsNaN(candidate->residual_history_f_mps2[2]));
    if (all(bv)) {
      labelValid = true;
    } else {
      labelValid = false;
    }
  } else {
    labelValid = false;
  }
  candidate->causal_valid = labelValid;
  candidate->robust_state = *committed_robust_state;
  c_gpenmpcObserveCausalVerticalDi(
      &candidate->robust_state, &input->x13[3],
      (double)input->source_timestamp_ns * 1.0E-9, input->leg_index,
      input->payload_kg, &out->vertical_observer.updated,
      &out->vertical_observer.reset, &out->vertical_observer.residual_z_mps2,
      &out->vertical_observer.estimate_z_mps2);
  c_executionEvidence_predicted_m[0] =
      out->closed_gp_evidence.predicted_mean_f_mps2[0];
  c_executionEvidence_calibrated_[0] =
      out->closed_gp_evidence.calibrated_half_width_f_mps2[0];
  observedI[0] = out->closed_gp_evidence.observed_innovation_f_mps2[0];
  c_executionEvidence_predicted_m[1] =
      out->closed_gp_evidence.predicted_mean_f_mps2[1];
  c_executionEvidence_calibrated_[1] =
      out->closed_gp_evidence.calibrated_half_width_f_mps2[1];
  observedI[1] = out->closed_gp_evidence.observed_innovation_f_mps2[1];
  c_executionEvidence_predicted_m[2] =
      out->closed_gp_evidence.predicted_mean_f_mps2[2];
  c_executionEvidence_calibrated_[2] =
      out->closed_gp_evidence.calibrated_half_width_f_mps2[2];
  observedI[2] = out->closed_gp_evidence.observed_innovation_f_mps2[2];
  if (!out->closed_gp_evidence.available) {
    c_executionEvidence_predicted_m[0] = 0.0;
    observedI[0] = 0.0;
    c_executionEvidence_calibrated_[0] = 1.0;
    c_executionEvidence_predicted_m[1] = 0.0;
    observedI[1] = 0.0;
    c_executionEvidence_calibrated_[1] = 1.0;
    c_executionEvidence_predicted_m[2] = 0.0;
    observedI[2] = 0.0;
    c_executionEvidence_calibrated_[2] = 1.0;
  }
  c_gpenmpcCoordinatedGpRobustSe3A(
      out->x19, input->reference_up.position_m,
      input->reference_up.velocity_mps, &candidate->robust_state, input->dt_s,
      out->closed_gp_evidence.available, out->closed_gp_evidence.hard_invalid,
      out->closed_gp_evidence.trust, out->closed_gp_evidence.gp_frame_i_from_f,
      c_executionEvidence_predicted_m, c_executionEvidence_calibrated_,
      out->closed_gp_evidence.observed_innovation_available, observedI,
      candidate->gp_agreement_weight_f, out->augmentation_up_mps2,
      &out->physical_diagnostic);
  gpenmpcDesiredSe3Command(
      out->x19, input->reference_up.position_m,
      input->reference_up.velocity_mps, input->reference_up.acceleration_mps2,
      input->payload_kg, input->wind_estimate_xy_mps, out->augmentation_up_mps2,
      observedI, c_executionEvidence_predicted_m,
      SD->u3.f5.t10_desired_rotation);
  c_gpenmpcUpdateDesiredAttitudeCo(
      SD, c_committed_attitude_continuity->initialized,
      c_committed_attitude_continuity->angular_velocity_valid,
      c_committed_attitude_continuity->filtered_rotation,
      c_committed_attitude_continuity->c_desired_angular_velocity_body,
      c_committed_attitude_continuity->c_desired_angular_acceleration_,
      c_committed_attitude_continuity->update_count,
      c_committed_attitude_continuity->reset_count,
      SD->u3.f5.t10_desired_rotation, input->dt_s, false,
      &out->attitude_command, &candidate->attitude_continuity_state);
  se3WrenchKernel(out->x19, input->reference_up.position_m,
                  input->reference_up.velocity_mps,
                  input->reference_up.acceleration_mps2, input->payload_kg,
                  input->wind_estimate_xy_mps, out->augmentation_up_mps2,
                  out->attitude_command.desired_rotation,
                  out->attitude_command.c_desired_angular_velocity_body,
                  out->attitude_command.c_desired_angular_acceleration_, w,
                  out->kernel_control_candidate.rotor_command_n, SD->u3.f5.d);
  candidate->last_sample_timestamp_ns = input->source_timestamp_ns;
  candidate->source_generation = input->source_generation;
  candidate->leg_index = input->leg_index;
  out->kernel_control_candidate.rotor_saturated = (SD->u3.f5.d[50] != 0.0);
  out->kernel_control_candidate.c_force_projection_norm_mismatc =
      SD->u3.f5.d[48];
  out->kernel_control_candidate.desired_force_projected_up_n[0] =
      SD->u3.f5.d[3];
  out->kernel_control_candidate.desired_force_projected_up_n[1] =
      SD->u3.f5.d[4];
  out->kernel_control_candidate.desired_force_projected_up_n[2] =
      SD->u3.f5.d[5];
  out->input = *input;
  out->kernel_output61[0] = w[0];
  out->kernel_output61[1] = w[1];
  out->kernel_output61[2] = w[2];
  out->kernel_output61[3] = w[3];
  for (i = 0; i < 6; i++) {
    out->kernel_output61[i + 4] =
        out->kernel_control_candidate.rotor_command_n[i];
  }
  memcpy(&out->kernel_output61[10], &SD->u3.f5.d[0], 51U * sizeof(double));
}

/*
 * CANONICALLOCALINNERPRECONTROL Compute a fast-loop numerical candidate.
 *  Inputs combine committed numerical state with the current sample and reference.
 *
 *  INPUT: x13, rotor_thrust_state_n, reference_up (p/v/a/j), payload_kg,
 *  wind_estimate_xy_mps, dt_s, source_timestamp_ns (original uint64),
 *  source_generation (original uint64), leg_index and explicit new_leg.
 *  NUMERICS: robust, enmpc, calibration, profile, attitudeContinuity and
 *  kernelParameters from the current canonical assets; no gp_model is needed.
 *  COMMITTED=[] initializes the original numerical states on a new leg only.
 *  Otherwise it is the prior candidate AFTER the caller's actual control
 *  commit has applied gpenmpcRobustSe3ObserveControl and the original vertical
 *  observer commit. PREVIOUSPENDING is that commit's original Runtime.Pending:
 *  prediction, velocity_up_mps, nominal_acceleration_up_mps2, frame_i_from_f,
 *  generation, timestamp_ns and leg_index. Prediction remains HOST-owned.
 *
 *  The caller must not install CANDIDATE on unsuccessful publication/commit.
 *  kernel_output61 is a numerical candidate. The caller validates commit status
 *  and source identity before use; source tags are copied from the inputs.
 *
 * Arguments    : e_gpenmpcNative_canonicalLocalIn *SD
 *                const double input_x13[13]
 *                const double input_rotor_thrust_state_n[6]
 *                const double input_reference_up_position_m[3]
 *                const double input_reference_up_velocity_mps[3]
 *                const double c_input_reference_up_accelerati[3]
 *                const double input_reference_up_jerk_mps3[3]
 *                double input_payload_kg
 *                const double input_wind_estimate_xy_mps[2]
 *                double input_dt_s
 *                double input_leg_index
 *                unsigned long long input_source_timestamp_ns
 *                unsigned long long input_source_generation
 *                p_struct_T *candidate
 *                l_struct_T *out
 * Return Type  : void
 */
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
    l_struct_T *out)
{
  static const double t16_filtered_rotation[9] = {1.0, 0.0, 0.0, 0.0, 1.0,
                                                  0.0, 0.0, 0.0, 1.0};
  static const double c_t16_desired_angular_accelerat[3] = {0.0, 0.0, 0.0};
  static const double c_t16_desired_angular_velocity_[3] = {0.0, 0.0, 0.0};
  double w[4];
  int i;
  /*  The generated first/ordinary entries deliberately specialize this branch.
   */
  /*  Interpreted MATLAB retains the exact original value and acceptance set. */
  /*  .002 is the existing physical adapter minimum, .0100001 the existing */
  /*  CurrentPhysicalCausalRuntime maximum; their accepted intersection is kept.
   */
  memcpy(&out->x19[0], &input_x13[0], 13U * sizeof(double));
  candidate->residual_history_f_mps2[0] = 0.0;
  candidate->gp_agreement_weight_f[0] = 0.0;
  candidate->residual_history_f_mps2[1] = 0.0;
  candidate->gp_agreement_weight_f[1] = 0.0;
  candidate->residual_history_f_mps2[2] = 0.0;
  candidate->gp_agreement_weight_f[2] = 0.0;
  for (i = 0; i < 6; i++) {
    out->x19[i + 13] = input_rotor_thrust_state_n[i];
    candidate->previous_rotor_command_n[i] = input_rotor_thrust_state_n[i];
  }
  double b_expl_temp[3];
  double c_expl_temp[3];
  double expl_temp;
  candidate->robust_state.filtered_compensation_i_mps2[0] = 0.0;
  candidate->robust_state.filtered_compensation_i_mps2[1] = 0.0;
  candidate->robust_state.filtered_compensation_i_mps2[2] = 0.0;
  candidate->robust_state.authority_scale = 1.0;
  candidate->robust_state.last_tangent_xy[0] = 1.0;
  candidate->robust_state.last_tangent_xy[1] = 0.0;
  candidate->robust_state.vertical_disturbance_ewma_mps2 = 0.0;
  candidate->robust_state.c_vertical_observer_previous_ve = 0.0;
  candidate->robust_state.c_vertical_observer_previous_no = 0.0;
  candidate->robust_state.c_vertical_observer_previous_ti = rtNaN;
  candidate->robust_state.c_vertical_observer_previous_le = rtNaN;
  candidate->robust_state.c_vertical_observer_previous_pa = rtNaN;
  candidate->robust_state.c_vertical_observer_observation = false;
  candidate->robust_state.c_vertical_observer_update_enab = true;
  candidate->robust_state.vertical_observer_reset_count = 0.0;
  candidate->robust_state.c_vertical_observer_antiwindup_ = 0.0;
  candidate->robust_state.gp_responsibility_mode_transition_count = 0.0;
  expl_temp = (double)input_source_timestamp_ns * 1.0E-9;
  c_gpenmpcResetCausalVerticalDist(&candidate->robust_state, &input_x13[3],
                                  expl_temp, input_leg_index, input_payload_kg);
  c_gpenmpcObserveCausalVerticalDi(
      &candidate->robust_state, &input_x13[3], expl_temp, input_leg_index,
      input_payload_kg, &out->vertical_observer.updated,
      &out->vertical_observer.reset, &out->vertical_observer.residual_z_mps2,
      &out->vertical_observer.estimate_z_mps2);
  /* GPENMPCCOORDINATEDGPROBUSTSE3AUGMENTATION One coordinated GP/robust action.
   */
  /*  */
  /*  The learned acceleration residual enters the prediction model with a */
  /*  positive sign.  This execution function uses its negative, in the exact */
  /*  prediction frame, as one bounded cancellation term inside the existing */
  /*  robust compensation authority.  It does not append a second controller. */
  /*  When the current closed evidence is unavailable, hard invalid, below the
   */
  /*  inherited trust floor, or has zero causal responsibility, this function */
  /*  delegates directly to gpenmpcRobustSe3Augmentation for exact B1 behavior.
   */
  candidate->robust_state.gp_responsibility_mode_active = false;
  candidate->robust_state.c_gp_responsibility_enter_elapsed = 0.0;
  candidate->robust_state.c_gp_responsibility_exit_elapsed = 0.0;
  candidate->robust_state.gp_responsibility_blend = 0.0;
  candidate->robust_state.responsibility_innovation_ratio_ewma_f[0] = 0.0;
  candidate->robust_state.responsibility_filtered_gp_mean_f_mps2[0] = 0.0;
  candidate->robust_state.responsibility_innovation_ratio_ewma_f[1] = 0.0;
  candidate->robust_state.responsibility_filtered_gp_mean_f_mps2[1] = 0.0;
  candidate->robust_state.responsibility_innovation_ratio_ewma_f[2] = 0.0;
  candidate->robust_state.responsibility_filtered_gp_mean_f_mps2[2] = 0.0;
  exactB1Fallback(out->x19, input_reference_up_position_m,
                  input_reference_up_velocity_mps, &candidate->robust_state,
                  input_dt_s, &out->physical_diagnostic,
                  out->augmentation_up_mps2);
  gpenmpcDesiredSe3Command(
      out->x19, input_reference_up_position_m, input_reference_up_velocity_mps,
      c_input_reference_up_accelerati, input_payload_kg,
      input_wind_estimate_xy_mps, out->augmentation_up_mps2, b_expl_temp,
      c_expl_temp, SD->u3.f4.t18_desired_rotation);
  c_gpenmpcUpdateDesiredAttitudeCo(
      SD, false, false, t16_filtered_rotation, c_t16_desired_angular_velocity_,
      c_t16_desired_angular_accelerat, 0.0, 0.0, SD->u3.f4.t18_desired_rotation,
      input_dt_s, true, &out->attitude_command,
      &candidate->attitude_continuity_state);
  se3WrenchKernel(out->x19, input_reference_up_position_m,
                  input_reference_up_velocity_mps,
                  c_input_reference_up_accelerati, input_payload_kg,
                  input_wind_estimate_xy_mps, out->augmentation_up_mps2,
                  out->attitude_command.desired_rotation,
                  out->attitude_command.c_desired_angular_velocity_body,
                  out->attitude_command.c_desired_angular_acceleration_, w,
                  out->kernel_control_candidate.rotor_command_n, SD->u3.f4.d);
  candidate->leg_index = input_leg_index;
  out->kernel_control_candidate.rotor_saturated = (SD->u3.f4.d[50] != 0.0);
  out->kernel_control_candidate.c_force_projection_norm_mismatc =
      SD->u3.f4.d[48];
  out->kernel_control_candidate.desired_force_projected_up_n[0] =
      SD->u3.f4.d[3];
  out->kernel_control_candidate.desired_force_projected_up_n[1] =
      SD->u3.f4.d[4];
  out->kernel_control_candidate.desired_force_projected_up_n[2] =
      SD->u3.f4.d[5];
  memcpy(&out->input.x13[0], &input_x13[0], 13U * sizeof(double));
  for (i = 0; i < 6; i++) {
    out->input.rotor_thrust_state_n[i] = input_rotor_thrust_state_n[i];
  }
  out->input.reference_up.position_m[0] = input_reference_up_position_m[0];
  out->input.reference_up.velocity_mps[0] = input_reference_up_velocity_mps[0];
  out->input.reference_up.acceleration_mps2[0] =
      c_input_reference_up_accelerati[0];
  out->input.reference_up.jerk_mps3[0] = input_reference_up_jerk_mps3[0];
  out->input.reference_up.position_m[1] = input_reference_up_position_m[1];
  out->input.reference_up.velocity_mps[1] = input_reference_up_velocity_mps[1];
  out->input.reference_up.acceleration_mps2[1] =
      c_input_reference_up_accelerati[1];
  out->input.reference_up.jerk_mps3[1] = input_reference_up_jerk_mps3[1];
  out->input.reference_up.position_m[2] = input_reference_up_position_m[2];
  out->input.reference_up.velocity_mps[2] = input_reference_up_velocity_mps[2];
  out->input.reference_up.acceleration_mps2[2] =
      c_input_reference_up_accelerati[2];
  out->input.reference_up.jerk_mps3[2] = input_reference_up_jerk_mps3[2];
  out->input.payload_kg = input_payload_kg;
  out->input.wind_estimate_xy_mps[0] = input_wind_estimate_xy_mps[0];
  out->input.wind_estimate_xy_mps[1] = input_wind_estimate_xy_mps[1];
  out->input.dt_s = input_dt_s;
  out->input.leg_index = input_leg_index;
  out->input.source_timestamp_ns = input_source_timestamp_ns;
  out->input.source_generation = input_source_generation;
  out->kernel_output61[0] = w[0];
  out->kernel_output61[1] = w[1];
  out->kernel_output61[2] = w[2];
  out->kernel_output61[3] = w[3];
  for (i = 0; i < 6; i++) {
    out->kernel_output61[i + 4] =
        out->kernel_control_candidate.rotor_command_n[i];
  }
  memcpy(&out->kernel_output61[10], &SD->u3.f4.d[0], 51U * sizeof(double));
}

/*
 * File trailer for canonicalLocalInnerPreControl.c
 *
 * [EOF]
 */
