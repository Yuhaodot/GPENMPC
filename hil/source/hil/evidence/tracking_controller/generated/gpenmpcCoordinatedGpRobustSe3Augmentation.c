/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcCoordinatedGpRobustSe3Augmentation.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcCoordinatedGpRobustSe3Augmentation.h"
#include "allOrAny.h"
#include "minOrMax.h"
#include "norm.h"
#include "gpenmpcAdvanceCompensationState.h"
#include "gpenmpcClipNorm.h"
#include "gpenmpcComposeResidualTarget.h"
#include "gpenmpcComputeGpResponsibility.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_data.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_internal_types.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "gpenmpcRobustSe3Augmentation.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
static const double dv4[3] = {0.07196314079429497, 0.05952671236830262,
                              0.07232596728766205};

static const double dv5[3] = {0.125021561465691, 0.10321513341156671,
                              0.39568699178777844};

/* Function Declarations */
static void b_exactB1Fallback(const double plantState[19],
                              const double reference_position_m[3],
                              const double reference_velocity_mps[3],
                              const double c_robustConfig_residual_tail_ma[3],
                              const double c_robustConfig_robust_radius_f_[3],
                              i_struct_T *state, double dt,
                              c_struct_T *diagnostic, double augmentation[3]);

static void c_exactB1Fallback(const double plantState[19],
                              const double reference_position_m[3],
                              const double reference_velocity_mps[3],
                              const double c_robustConfig_residual_tail_ma[3],
                              const double c_robustConfig_robust_radius_f_[3],
                              i_struct_T *state, double dt,
                              const char reason_Value_data[],
                              const int reason_Value_size[2],
                              c_struct_T *diagnostic, double augmentation[3]);

static void resetResponsibilityState(i_struct_T *state);

static double updateResponsibilityGate(
    i_struct_T *state, const double observedF[3], double dt,
    double c_diagnostic_residual_floor_f_m[3],
    double c_diagnostic_observed_innovatio[3],
    double c_diagnostic_instantaneous_rati[3],
    double diagnostic_ewma_ratio_f[3], boolean_T *diagnostic_mode_active,
    double *diagnostic_blend, double *c_diagnostic_mode_transition_co,
    double *diagnostic_future_samples_read);

/* Function Definitions */
/*
 * Arguments    : const double plantState[19]
 *                const double reference_position_m[3]
 *                const double reference_velocity_mps[3]
 *                const double c_robustConfig_residual_tail_ma[3]
 *                const double c_robustConfig_robust_radius_f_[3]
 *                i_struct_T *state
 *                double dt
 *                c_struct_T *diagnostic
 *                double augmentation[3]
 * Return Type  : void
 */
static void b_exactB1Fallback(const double plantState[19],
                              const double reference_position_m[3],
                              const double reference_velocity_mps[3],
                              const double c_robustConfig_residual_tail_ma[3],
                              const double c_robustConfig_robust_radius_f_[3],
                              i_struct_T *state, double dt,
                              c_struct_T *diagnostic, double augmentation[3])
{
  static const char b_cv[25] = {'I', 'N', 'V', 'A', 'L', 'I', 'D', '_', 'C',
                                'L', 'O', 'S', 'E', 'D', '_', 'I', 'N', 'N',
                                'O', 'V', 'A', 'T', 'I', 'O', 'N'};
  t_struct_T inherited;
  int i;
  gpenmpcRobustSe3Augmentation(
      plantState, reference_position_m, reference_velocity_mps,
      c_robustConfig_residual_tail_ma, c_robustConfig_robust_radius_f_, state,
      dt, augmentation, &inherited);
  diagnostic->gp_trust = 0.0;
  diagnostic->gp_prediction_axis_authority_f[0] = 0.0;
  diagnostic->gp_physical_axis_authority_f[0] = 0.0;
  diagnostic->gp_mean_f_mps2[0] = 0.0;
  diagnostic->gp_prediction_axis_authority_f[1] = 0.0;
  diagnostic->gp_physical_axis_authority_f[1] = 0.0;
  diagnostic->gp_mean_f_mps2[1] = 0.0;
  diagnostic->gp_prediction_axis_authority_f[2] = 0.0;
  diagnostic->gp_physical_axis_authority_f[2] = 0.0;
  diagnostic->gp_mean_f_mps2[2] = 0.0;
  for (i = 0; i < 9; i++) {
    int i1;
    i1 = iv[i];
    diagnostic->gp_frame_i_from_f[i] = i1;
    diagnostic->robust_frame_i_from_f[i] = i1;
  }
  diagnostic->vertical_observer_shadow_i_mps2[0] = 0.0;
  diagnostic->vertical_observer_shadow_i_mps2[1] = 0.0;
  diagnostic->vertical_observer_shadow_i_mps2[2] = 0.0;
  diagnostic->gp_responsibility_blend = 0.0;
  for (i = 0; i < 25; i++) {
    diagnostic->fallback_reason.Value.data[i] = b_cv[i];
  }
  diagnostic->exact_b1_fallback = true;
  diagnostic->sliding_i_mps[0] = inherited.sliding_i_mps[0];
  diagnostic->sliding_f_mps[0] = inherited.sliding_f_mps[0];
  diagnostic->target_i_mps2[0] = inherited.target_i_mps2[0];
  diagnostic->sliding_i_mps[1] = inherited.sliding_i_mps[1];
  diagnostic->sliding_f_mps[1] = inherited.sliding_f_mps[1];
  diagnostic->target_i_mps2[1] = inherited.target_i_mps2[1];
  diagnostic->sliding_i_mps[2] = inherited.sliding_i_mps[2];
  diagnostic->sliding_f_mps[2] = inherited.sliding_f_mps[2];
  diagnostic->target_i_mps2[2] = inherited.target_i_mps2[2];
}

/*
 * Arguments    : const double plantState[19]
 *                const double reference_position_m[3]
 *                const double reference_velocity_mps[3]
 *                const double c_robustConfig_residual_tail_ma[3]
 *                const double c_robustConfig_robust_radius_f_[3]
 *                i_struct_T *state
 *                double dt
 *                const char reason_Value_data[]
 *                const int reason_Value_size[2]
 *                c_struct_T *diagnostic
 *                double augmentation[3]
 * Return Type  : void
 */
static void c_exactB1Fallback(const double plantState[19],
                              const double reference_position_m[3],
                              const double reference_velocity_mps[3],
                              const double c_robustConfig_residual_tail_ma[3],
                              const double c_robustConfig_robust_radius_f_[3],
                              i_struct_T *state, double dt,
                              const char reason_Value_data[],
                              const int reason_Value_size[2],
                              c_struct_T *diagnostic, double augmentation[3])
{
  t_struct_T inherited;
  int i;
  int loop_ub;
  gpenmpcRobustSe3Augmentation(
      plantState, reference_position_m, reference_velocity_mps,
      c_robustConfig_residual_tail_ma, c_robustConfig_robust_radius_f_, state,
      dt, augmentation, &inherited);
  diagnostic->gp_trust = 0.0;
  diagnostic->gp_prediction_axis_authority_f[0] = 0.0;
  diagnostic->gp_physical_axis_authority_f[0] = 0.0;
  diagnostic->gp_mean_f_mps2[0] = 0.0;
  diagnostic->gp_prediction_axis_authority_f[1] = 0.0;
  diagnostic->gp_physical_axis_authority_f[1] = 0.0;
  diagnostic->gp_mean_f_mps2[1] = 0.0;
  diagnostic->gp_prediction_axis_authority_f[2] = 0.0;
  diagnostic->gp_physical_axis_authority_f[2] = 0.0;
  diagnostic->gp_mean_f_mps2[2] = 0.0;
  for (i = 0; i < 9; i++) {
    loop_ub = iv[i];
    diagnostic->gp_frame_i_from_f[i] = loop_ub;
    diagnostic->robust_frame_i_from_f[i] = loop_ub;
  }
  diagnostic->vertical_observer_shadow_i_mps2[0] = 0.0;
  diagnostic->vertical_observer_shadow_i_mps2[1] = 0.0;
  diagnostic->vertical_observer_shadow_i_mps2[2] = 0.0;
  diagnostic->gp_responsibility_blend = 0.0;
  loop_ub = reason_Value_size[1];
  memcpy(&diagnostic->fallback_reason.Value.data[0], &reason_Value_data[0],
         (unsigned int)loop_ub * sizeof(char));
  diagnostic->exact_b1_fallback = true;
  diagnostic->sliding_i_mps[0] = inherited.sliding_i_mps[0];
  diagnostic->sliding_f_mps[0] = inherited.sliding_f_mps[0];
  diagnostic->target_i_mps2[0] = inherited.target_i_mps2[0];
  diagnostic->sliding_i_mps[1] = inherited.sliding_i_mps[1];
  diagnostic->sliding_f_mps[1] = inherited.sliding_f_mps[1];
  diagnostic->target_i_mps2[1] = inherited.target_i_mps2[1];
  diagnostic->sliding_i_mps[2] = inherited.sliding_i_mps[2];
  diagnostic->sliding_f_mps[2] = inherited.sliding_f_mps[2];
  diagnostic->target_i_mps2[2] = inherited.target_i_mps2[2];
}

/*
 * Arguments    : i_struct_T *state
 * Return Type  : void
 */
static void resetResponsibilityState(i_struct_T *state)
{
  state->gp_responsibility_mode_active = false;
  state->c_gp_responsibility_enter_elapsed = 0.0;
  state->c_gp_responsibility_exit_elapsed = 0.0;
  state->gp_responsibility_blend = 0.0;
  state->responsibility_innovation_ratio_ewma_f[0] = 0.0;
  state->responsibility_filtered_gp_mean_f_mps2[0] = 0.0;
  state->responsibility_innovation_ratio_ewma_f[1] = 0.0;
  state->responsibility_filtered_gp_mean_f_mps2[1] = 0.0;
  state->responsibility_innovation_ratio_ewma_f[2] = 0.0;
  state->responsibility_filtered_gp_mean_f_mps2[2] = 0.0;
}

/*
 * Arguments    : i_struct_T *state
 *                const double observedF[3]
 *                double dt
 *                double c_diagnostic_residual_floor_f_m[3]
 *                double c_diagnostic_observed_innovatio[3]
 *                double c_diagnostic_instantaneous_rati[3]
 *                double diagnostic_ewma_ratio_f[3]
 *                boolean_T *diagnostic_mode_active
 *                double *diagnostic_blend
 *                double *c_diagnostic_mode_transition_co
 *                double *diagnostic_future_samples_read
 * Return Type  : double
 */
static double updateResponsibilityGate(
    i_struct_T *state, const double observedF[3], double dt,
    double c_diagnostic_residual_floor_f_m[3],
    double c_diagnostic_observed_innovatio[3],
    double c_diagnostic_instantaneous_rati[3],
    double diagnostic_ewma_ratio_f[3], boolean_T *diagnostic_mode_active,
    double *diagnostic_blend, double *c_diagnostic_mode_transition_co,
    double *diagnostic_future_samples_read)
{
  double d;
  double diagnostic_aggregate_ratio;
  double step;
  step = exp(-dt / 0.3);
  diagnostic_aggregate_ratio = fabs(observedF[0]) / 0.03219254730192427;
  c_diagnostic_instantaneous_rati[0] = diagnostic_aggregate_ratio;
  d = state->responsibility_innovation_ratio_ewma_f[0];
  d += (1.0 - step) * (diagnostic_aggregate_ratio - d);
  state->responsibility_innovation_ratio_ewma_f[0] = d;
  diagnostic_aggregate_ratio = fabs(observedF[1]) / 0.029209190368417185;
  c_diagnostic_instantaneous_rati[1] = diagnostic_aggregate_ratio;
  d = state->responsibility_innovation_ratio_ewma_f[1];
  d += (1.0 - step) * (diagnostic_aggregate_ratio - d);
  state->responsibility_innovation_ratio_ewma_f[1] = d;
  diagnostic_aggregate_ratio = fabs(observedF[2]) / 0.037378411972285294;
  c_diagnostic_instantaneous_rati[2] = diagnostic_aggregate_ratio;
  d = state->responsibility_innovation_ratio_ewma_f[2];
  d += (1.0 - step) * (diagnostic_aggregate_ratio - d);
  state->responsibility_innovation_ratio_ewma_f[2] = d;
  diagnostic_aggregate_ratio = b_maximum(state->responsibility_innovation_ratio_ewma_f);
  if (state->gp_responsibility_mode_active) {
    state->c_gp_responsibility_enter_elapsed = 0.0;
    if (diagnostic_aggregate_ratio < 0.75) {
      state->c_gp_responsibility_exit_elapsed += dt;
    } else {
      state->c_gp_responsibility_exit_elapsed = 0.0;
    }
    if (state->c_gp_responsibility_exit_elapsed >= 0.6) {
      state->gp_responsibility_mode_active = false;
      state->c_gp_responsibility_exit_elapsed = 0.0;
      state->gp_responsibility_mode_transition_count++;
    }
  } else {
    state->c_gp_responsibility_exit_elapsed = 0.0;
    if (diagnostic_aggregate_ratio > 1.25) {
      state->c_gp_responsibility_enter_elapsed += dt;
    } else {
      state->c_gp_responsibility_enter_elapsed = 0.0;
    }
    if (state->c_gp_responsibility_enter_elapsed >= 0.3) {
      state->gp_responsibility_mode_active = true;
      state->c_gp_responsibility_enter_elapsed = 0.0;
      state->gp_responsibility_mode_transition_count++;
    }
  }
  step = dt / 0.2;
  if (state->gp_responsibility_mode_active) {
    state->gp_responsibility_blend =
        fmin(1.0, state->gp_responsibility_blend + step);
  } else {
    state->gp_responsibility_blend =
        fmax(0.0, state->gp_responsibility_blend - step);
  }
  c_diagnostic_residual_floor_f_m[0] = 0.03219254730192427;
  c_diagnostic_observed_innovatio[0] = observedF[0];
  diagnostic_ewma_ratio_f[0] = state->responsibility_innovation_ratio_ewma_f[0];
  c_diagnostic_residual_floor_f_m[1] = 0.029209190368417185;
  c_diagnostic_observed_innovatio[1] = observedF[1];
  diagnostic_ewma_ratio_f[1] = state->responsibility_innovation_ratio_ewma_f[1];
  c_diagnostic_residual_floor_f_m[2] = 0.037378411972285294;
  c_diagnostic_observed_innovatio[2] = observedF[2];
  diagnostic_ewma_ratio_f[2] = state->responsibility_innovation_ratio_ewma_f[2];
  *diagnostic_mode_active = state->gp_responsibility_mode_active;
  *diagnostic_blend = state->gp_responsibility_blend;
  *c_diagnostic_mode_transition_co = state->gp_responsibility_mode_transition_count;
  *diagnostic_future_samples_read = 0.0;
  return diagnostic_aggregate_ratio;
}

/*
 * GPENMPCCOORDINATEDGPROBUSTSE3AUGMENTATION One coordinated GP/robust action.
 *
 *  The learned acceleration residual enters the prediction model with a
 *  positive sign.  This execution function uses its negative, in the exact
 *  prediction frame, as one bounded cancellation term inside the existing
 *  robust compensation authority.  It does not append a second controller.
 *  When the current closed evidence is unavailable, hard invalid, below the
 *  inherited trust floor, or has zero causal responsibility, this function
 *  delegates directly to gpenmpcRobustSe3Augmentation for exact B1 behavior.
 *
 * Arguments    : const double plantState[19]
 *                const double reference_position_m[3]
 *                const double reference_velocity_mps[3]
 *                i_struct_T *state
 *                double dt
 *                boolean_T gpEvidence_available
 *                boolean_T gpEvidence_hard_invalid
 *                double gpEvidence_trust
 *                const double gpEvidence_gp_frame_i_from_f[9]
 *                const double c_gpEvidence_predicted_mean_f_m[3]
 *                const double c_gpEvidence_calibrated_half_wi[3]
 *                boolean_T c_gpEvidence_observed_innovatio
 *                const double d_gpEvidence_observed_innovatio[3]
 *                const double gpAgreementWeightF[3]
 *                double augmentation[3]
 *                c_struct_T *diagnostic
 * Return Type  : void
 */
void c_gpenmpcCoordinatedGpRobustSe3A(
    const double plantState[19], const double reference_position_m[3],
    const double reference_velocity_mps[3], i_struct_T *state, double dt,
    boolean_T gpEvidence_available, boolean_T gpEvidence_hard_invalid,
    double gpEvidence_trust, const double gpEvidence_gp_frame_i_from_f[9],
    const double c_gpEvidence_predicted_mean_f_m[3],
    const double c_gpEvidence_calibrated_half_wi[3],
    boolean_T c_gpEvidence_observed_innovatio,
    const double d_gpEvidence_observed_innovatio[3],
    const double gpAgreementWeightF[3], double augmentation[3],
    c_struct_T *diagnostic)
{
  h_struct_T e_expl_temp;
  double b_expl_temp;
  double expl_temp;
  double gate_blend;
  double gate_mode_transition_count;
  double speed;
  double x;
  int i;
  char c_responsibility_reason_Value_d[44];
  boolean_T c_responsibility_exact_b1_requi;
  boolean_T gate_mode_active;
  boolean_T guard1;
  boolean_T guard2;
  diagnostic->sliding_f_mps[0] = gpAgreementWeightF[0];
  diagnostic->sliding_f_mps[1] = gpAgreementWeightF[1];
  diagnostic->sliding_f_mps[2] = gpAgreementWeightF[2];
  guard1 = false;
  guard2 = false;
  if (gpEvidence_available && c_gpEvidence_observed_innovatio &&
      !gpEvidence_hard_invalid &&
      (!rtIsInf(gpEvidence_trust) && !rtIsNaN(gpEvidence_trust)) &&
      (gpEvidence_trust >= 0.25)) {
    boolean_T d_gpEvidence_calibrated_half_wi[17];
    d_gpEvidence_calibrated_half_wi[0] =
        (rtIsInf(c_gpEvidence_predicted_mean_f_m[0]) ||
         rtIsNaN(c_gpEvidence_predicted_mean_f_m[0]));
    d_gpEvidence_calibrated_half_wi[1] =
        (rtIsInf(c_gpEvidence_predicted_mean_f_m[1]) ||
         rtIsNaN(c_gpEvidence_predicted_mean_f_m[1]));
    d_gpEvidence_calibrated_half_wi[2] =
        (rtIsInf(c_gpEvidence_predicted_mean_f_m[2]) ||
         rtIsNaN(c_gpEvidence_predicted_mean_f_m[2]));
    if (vectorAny(d_gpEvidence_calibrated_half_wi, 3)) {
      guard1 = true;
    } else {
      boolean_T bv[9];
      for (i = 0; i < 9; i++) {
        speed = gpEvidence_gp_frame_i_from_f[i];
        bv[i] = (rtIsInf(speed) || rtIsNaN(speed));
      }
      if (vectorAny(bv, 9)) {
        guard1 = true;
      } else {
        diagnostic->gp_mean_f_mps2[0] = c_gpEvidence_predicted_mean_f_m[0];
        diagnostic->gp_mean_f_mps2[1] = c_gpEvidence_predicted_mean_f_m[1];
        diagnostic->gp_mean_f_mps2[2] = c_gpEvidence_predicted_mean_f_m[2];
        gpenmpcClipNorm(diagnostic->gp_mean_f_mps2);
        for (i = 0; i < 3; i++) {
          b_expl_temp = d_gpEvidence_observed_innovatio[i];
          d_gpEvidence_calibrated_half_wi[i] =
              (rtIsInf(b_expl_temp) || rtIsNaN(b_expl_temp));
        }
        if (vectorAny(d_gpEvidence_calibrated_half_wi, 3)) {
          guard2 = true;
        } else {
          d_gpEvidence_calibrated_half_wi[0] =
              (rtIsInf(c_gpEvidence_calibrated_half_wi[0]) ||
               rtIsNaN(c_gpEvidence_calibrated_half_wi[0]));
          d_gpEvidence_calibrated_half_wi[1] =
              (rtIsInf(c_gpEvidence_calibrated_half_wi[1]) ||
               rtIsNaN(c_gpEvidence_calibrated_half_wi[1]));
          d_gpEvidence_calibrated_half_wi[2] =
              (rtIsInf(c_gpEvidence_calibrated_half_wi[2]) ||
               rtIsNaN(c_gpEvidence_calibrated_half_wi[2]));
          if (vectorAny(d_gpEvidence_calibrated_half_wi, 3)) {
            guard2 = true;
          } else {
            d_gpEvidence_calibrated_half_wi[0] =
                (c_gpEvidence_calibrated_half_wi[0] <= 0.0);
            d_gpEvidence_calibrated_half_wi[1] =
                (c_gpEvidence_calibrated_half_wi[1] <= 0.0);
            d_gpEvidence_calibrated_half_wi[2] =
                (c_gpEvidence_calibrated_half_wi[2] <= 0.0);
            if (vectorAny(d_gpEvidence_calibrated_half_wi, 3)) {
              guard2 = true;
            } else {
              double c_expl_temp[3];
              double c_responsibility_alpha_effectiv[3];
              double c_responsibility_weighted_mean_[3];
              double d_expl_temp[3];
              double gate_ewma_ratio_f[3];
              double gate_instantaneous_ratio_f[3];
              double gate_residual_floor_f_mps2[3];
              int c_responsibility_reason_Value_s[2];
              updateResponsibilityGate(state, d_gpEvidence_observed_innovatio,
                                       dt, gate_residual_floor_f_mps2,
                                       c_expl_temp, gate_instantaneous_ratio_f,
                                       gate_ewma_ratio_f, &gate_mode_active,
                                       &gate_blend, &gate_mode_transition_count,
                                       &speed);
              if (state->gp_responsibility_mode_active ||
                  (state->gp_responsibility_blend > 0.0)) {
                x = exp(-dt / 0.12);
                b_expl_temp = state->responsibility_filtered_gp_mean_f_mps2[0];
                b_expl_temp +=
                    (1.0 - x) * (diagnostic->gp_mean_f_mps2[0] - b_expl_temp);
                state->responsibility_filtered_gp_mean_f_mps2[0] = b_expl_temp;
                b_expl_temp = state->responsibility_filtered_gp_mean_f_mps2[1];
                b_expl_temp +=
                    (1.0 - x) * (diagnostic->gp_mean_f_mps2[1] - b_expl_temp);
                state->responsibility_filtered_gp_mean_f_mps2[1] = b_expl_temp;
                b_expl_temp = state->responsibility_filtered_gp_mean_f_mps2[2];
                b_expl_temp +=
                    (1.0 - x) * (diagnostic->gp_mean_f_mps2[2] - b_expl_temp);
                state->responsibility_filtered_gp_mean_f_mps2[2] = b_expl_temp;
              } else {
                state->responsibility_filtered_gp_mean_f_mps2[0] = 0.0;
                state->responsibility_filtered_gp_mean_f_mps2[1] = 0.0;
                state->responsibility_filtered_gp_mean_f_mps2[2] = 0.0;
              }
              c_gpenmpcComputeGpResponsibility(
                  state->gp_responsibility_blend, gpEvidence_trust,
                  diagnostic->sliding_f_mps, state->responsibility_filtered_gp_mean_f_mps2,
                  &c_responsibility_exact_b1_requi,
                  c_responsibility_reason_Value_d,
                  c_responsibility_reason_Value_s, &b_expl_temp, &speed,
                  c_expl_temp, c_responsibility_alpha_effectiv, &expl_temp,
                  d_expl_temp, c_responsibility_weighted_mean_, &x);
              if (c_responsibility_exact_b1_requi) {
                c_exactB1Fallback(
                    plantState, reference_position_m, reference_velocity_mps,
                    dv4, dv5, state, dt, c_responsibility_reason_Value_d,
                    c_responsibility_reason_Value_s, diagnostic, augmentation);
                diagnostic->gp_responsibility_blend = gate_blend;
                diagnostic->gp_prediction_axis_authority_f[0] =
                    c_responsibility_alpha_effectiv[0];
                diagnostic->gp_physical_axis_authority_f[0] = 0.0;
                diagnostic->gp_prediction_axis_authority_f[1] =
                    c_responsibility_alpha_effectiv[1];
                diagnostic->gp_physical_axis_authority_f[1] = 0.0;
                diagnostic->gp_prediction_axis_authority_f[2] =
                    c_responsibility_alpha_effectiv[2];
                diagnostic->gp_physical_axis_authority_f[2] = 0.0;
              } else {
                speed = b_norm(&reference_velocity_mps[0]);
                if (speed >= 0.75) {
                  state->last_tangent_xy[0] = reference_velocity_mps[0] / speed;
                  state->last_tangent_xy[1] = reference_velocity_mps[1] / speed;
                }
                diagnostic->robust_frame_i_from_f[0] =
                    state->last_tangent_xy[0];
                diagnostic->robust_frame_i_from_f[3] =
                    -state->last_tangent_xy[1];
                diagnostic->robust_frame_i_from_f[6] = 0.0;
                diagnostic->robust_frame_i_from_f[1] =
                    state->last_tangent_xy[1];
                diagnostic->robust_frame_i_from_f[4] =
                    state->last_tangent_xy[0];
                diagnostic->robust_frame_i_from_f[7] = 0.0;
                diagnostic->robust_frame_i_from_f[2] = 0.0;
                diagnostic->sliding_i_mps[0] =
                    (plantState[3] - reference_velocity_mps[0]) +
                    0.7 * (plantState[0] - reference_position_m[0]);
                diagnostic->robust_frame_i_from_f[5] = 0.0;
                diagnostic->sliding_i_mps[1] =
                    (plantState[4] - reference_velocity_mps[1]) +
                    0.7 * (plantState[1] - reference_position_m[1]);
                diagnostic->robust_frame_i_from_f[8] = 1.0;
                diagnostic->sliding_i_mps[2] =
                    (plantState[5] - reference_velocity_mps[2]) +
                    0.8 * (plantState[2] - reference_position_m[2]);
                for (i = 0; i < 3; i++) {
                  diagnostic->sliding_f_mps[i] =
                      (diagnostic->robust_frame_i_from_f[3 * i] *
                           diagnostic->sliding_i_mps[0] +
                       diagnostic->robust_frame_i_from_f[3 * i + 1] *
                           diagnostic->sliding_i_mps[1]) +
                      diagnostic->robust_frame_i_from_f[3 * i + 2] *
                          diagnostic->sliding_i_mps[2];
                }
                diagnostic->vertical_observer_shadow_i_mps2[0] = 0.0;
                diagnostic->vertical_observer_shadow_i_mps2[1] = 0.0;
                diagnostic->vertical_observer_shadow_i_mps2[2] =
                    -state->vertical_disturbance_ewma_mps2;
                c_gpenmpcComposeResidualTarget(
                    diagnostic->sliding_f_mps, c_responsibility_alpha_effectiv,
                    c_responsibility_weighted_mean_,
                    gpEvidence_gp_frame_i_from_f,
                    diagnostic->robust_frame_i_from_f,
                    diagnostic->vertical_observer_shadow_i_mps2, &e_expl_temp);
                c_gpenmpcAdvanceCompensationState(
                    state->filtered_compensation_i_mps2,
                    e_expl_temp.raw_target_i_mps2, dt, state->authority_scale,
                    c_expl_temp, diagnostic->target_i_mps2, d_expl_temp,
                    c_responsibility_weighted_mean_, augmentation);
                state->filtered_compensation_i_mps2[0] = augmentation[0];
                state->filtered_compensation_i_mps2[1] = augmentation[1];
                state->filtered_compensation_i_mps2[2] = augmentation[2];
                diagnostic->fallback_reason.Value.data[0] = 'N';
                diagnostic->fallback_reason.Value.data[1] = 'O';
                diagnostic->fallback_reason.Value.data[2] = 'N';
                diagnostic->fallback_reason.Value.data[3] = 'E';
                diagnostic->exact_b1_fallback = false;
                diagnostic->gp_trust = gpEvidence_trust;
                diagnostic->gp_prediction_axis_authority_f[0] =
                    c_responsibility_alpha_effectiv[0];
                diagnostic->gp_physical_axis_authority_f[0] =
                    e_expl_temp.alpha_physical_f[0];
                diagnostic->gp_prediction_axis_authority_f[1] =
                    c_responsibility_alpha_effectiv[1];
                diagnostic->gp_physical_axis_authority_f[1] =
                    e_expl_temp.alpha_physical_f[1];
                diagnostic->gp_prediction_axis_authority_f[2] =
                    c_responsibility_alpha_effectiv[2];
                diagnostic->gp_physical_axis_authority_f[2] =
                    e_expl_temp.alpha_physical_f[2];
                memcpy(&diagnostic->gp_frame_i_from_f[0],
                       &gpEvidence_gp_frame_i_from_f[0], 9U * sizeof(double));
                diagnostic->gp_responsibility_blend = gate_blend;
              }
            }
          }
        }
      }
    }
  } else {
    guard1 = true;
  }
  if (guard2) {
    resetResponsibilityState(state);
    b_exactB1Fallback(plantState, reference_position_m, reference_velocity_mps,
                      dv4, dv5, state, dt, diagnostic, augmentation);
  }
  if (guard1) {
    resetResponsibilityState(state);
    exactB1Fallback(plantState, reference_position_m, reference_velocity_mps,
                    state, dt, diagnostic, augmentation);
  }
}

/*
 * Arguments    : const double plantState[19]
 *                const double reference_position_m[3]
 *                const double reference_velocity_mps[3]
 *                i_struct_T *state
 *                double dt
 *                c_struct_T *diagnostic
 *                double augmentation[3]
 * Return Type  : void
 */
void exactB1Fallback(const double plantState[19],
                     const double reference_position_m[3],
                     const double reference_velocity_mps[3], i_struct_T *state,
                     double dt, c_struct_T *diagnostic, double augmentation[3])
{
  static const char b_cv[19] = {'I', 'N', 'V', 'A', 'L', 'I', 'D',
                                '_', 'O', 'R', '_', 'S', 'T', 'A',
                                'L', 'E', '_', 'G', 'P'};
  t_struct_T inherited;
  int i;
  diagnostic->gp_trust = 0.0;
  diagnostic->gp_prediction_axis_authority_f[0] = 0.0;
  diagnostic->gp_physical_axis_authority_f[0] = 0.0;
  diagnostic->gp_mean_f_mps2[0] = 0.0;
  diagnostic->gp_prediction_axis_authority_f[1] = 0.0;
  diagnostic->gp_physical_axis_authority_f[1] = 0.0;
  diagnostic->gp_mean_f_mps2[1] = 0.0;
  diagnostic->gp_prediction_axis_authority_f[2] = 0.0;
  diagnostic->gp_physical_axis_authority_f[2] = 0.0;
  diagnostic->gp_mean_f_mps2[2] = 0.0;
  gpenmpcRobustSe3Augmentation(plantState, reference_position_m,
                              reference_velocity_mps, dv4, dv5, state, dt,
                              augmentation, &inherited);
  for (i = 0; i < 9; i++) {
    int i1;
    i1 = iv[i];
    diagnostic->gp_frame_i_from_f[i] = i1;
    diagnostic->robust_frame_i_from_f[i] = i1;
  }
  diagnostic->vertical_observer_shadow_i_mps2[0] = 0.0;
  diagnostic->vertical_observer_shadow_i_mps2[1] = 0.0;
  diagnostic->vertical_observer_shadow_i_mps2[2] = 0.0;
  diagnostic->gp_responsibility_blend = 0.0;
  for (i = 0; i < 19; i++) {
    diagnostic->fallback_reason.Value.data[i] = b_cv[i];
  }
  diagnostic->exact_b1_fallback = true;
  diagnostic->sliding_i_mps[0] = inherited.sliding_i_mps[0];
  diagnostic->sliding_f_mps[0] = inherited.sliding_f_mps[0];
  diagnostic->target_i_mps2[0] = inherited.target_i_mps2[0];
  diagnostic->sliding_i_mps[1] = inherited.sliding_i_mps[1];
  diagnostic->sliding_f_mps[1] = inherited.sliding_f_mps[1];
  diagnostic->target_i_mps2[1] = inherited.target_i_mps2[1];
  diagnostic->sliding_i_mps[2] = inherited.sliding_i_mps[2];
  diagnostic->sliding_f_mps[2] = inherited.sliding_f_mps[2];
  diagnostic->target_i_mps2[2] = inherited.target_i_mps2[2];
}

/*
 * File trailer for gpenmpcCoordinatedGpRobustSe3Augmentation.c
 *
 * [EOF]
 */
