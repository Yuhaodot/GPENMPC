/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcRobustSe3Augmentation.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcRobustSe3Augmentation.h"
#include "norm.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_internal_types.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCROBUSTSE3AUGMENTATION Shared causal robust acceleration augmentation.
 *
 *  Inputs use only current state, current reference and retained controller
 *  memory.  The same function is used by B1 and ordinary B2.
 *
 * Arguments    : const double plantState[19]
 *                const double reference_position_m[3]
 *                const double reference_velocity_mps[3]
 *                const double c_robustConfig_residual_tail_ma[3]
 *                const double c_robustConfig_robust_radius_f_[3]
 *                i_struct_T *state
 *                double dt
 *                double augmentation[3]
 *                t_struct_T *diagnostic
 * Return Type  : void
 */
void gpenmpcRobustSe3Augmentation(
    const double plantState[19], const double reference_position_m[3],
    const double reference_velocity_mps[3],
    const double c_robustConfig_residual_tail_ma[3],
    const double c_robustConfig_robust_radius_f_[3], i_struct_T *state,
    double dt, double augmentation[3], t_struct_T *diagnostic)
{
  double frame[9];
  double speed;
  double valueNorm;
  double x;
  int k;
  speed = b_norm(&reference_velocity_mps[0]);
  if (speed >= 0.75) {
    state->last_tangent_xy[0] = reference_velocity_mps[0] / speed;
    state->last_tangent_xy[1] = reference_velocity_mps[1] / speed;
  }
  frame[0] = state->last_tangent_xy[0];
  frame[3] = -state->last_tangent_xy[1];
  frame[6] = 0.0;
  frame[1] = state->last_tangent_xy[1];
  frame[4] = state->last_tangent_xy[0];
  frame[7] = 0.0;
  frame[2] = 0.0;
  diagnostic->sliding_i_mps[0] =
      (plantState[3] - reference_velocity_mps[0]) +
      0.7 * (plantState[0] - reference_position_m[0]);
  frame[5] = 0.0;
  diagnostic->sliding_i_mps[1] =
      (plantState[4] - reference_velocity_mps[1]) +
      0.7 * (plantState[1] - reference_position_m[1]);
  frame[8] = 1.0;
  diagnostic->sliding_i_mps[2] =
      (plantState[5] - reference_velocity_mps[2]) +
      0.8 * (plantState[2] - reference_position_m[2]);
  /*  A causal vertical disturbance estimate is shared by B1 and B2.  It is */
  /*  composed with the existing robust target before the existing total cap, */
  /*  filter and slew limit, so it does not introduce additional authority. */
  diagnostic->c_vertical_observer_compensatio[0] = 0.0;
  diagnostic->c_vertical_observer_compensatio[1] = 0.0;
  diagnostic->c_vertical_observer_compensatio[2] =
      -state->vertical_disturbance_ewma_mps2;
  for (k = 0; k < 3; k++) {
    speed = (frame[3 * k] * diagnostic->sliding_i_mps[0] +
             frame[3 * k + 1] * diagnostic->sliding_i_mps[1]) +
            frame[3 * k + 2] * diagnostic->sliding_i_mps[2];
    diagnostic->sliding_f_mps[k] = speed;
    valueNorm = (c_robustConfig_robust_radius_f_[k] +
                 c_robustConfig_residual_tail_ma[k]) +
                0.02;
    diagnostic->radius_f_mps2[k] = valueNorm;
    augmentation[k] = -valueNorm * tanh(speed / 0.2);
  }
  /* GPENMPCCLIPNORM Radially clip a vector without changing its direction. */
  speed = augmentation[0];
  valueNorm = augmentation[1];
  x = augmentation[2];
  for (k = 0; k < 3; k++) {
    double d;
    d = ((frame[k] * speed + frame[k + 3] * valueNorm) + frame[k + 6] * x) +
        diagnostic->c_vertical_observer_compensatio[k];
    diagnostic->raw_target_i_mps2[k] = d;
    diagnostic->target_i_mps2[k] = d;
  }
  speed = c_norm(diagnostic->raw_target_i_mps2);
  if (speed > 0.75) {
    speed = 0.75 / speed;
    diagnostic->target_i_mps2[0] = diagnostic->raw_target_i_mps2[0] * speed;
    diagnostic->target_i_mps2[1] = diagnostic->raw_target_i_mps2[1] * speed;
    diagnostic->target_i_mps2[2] = diagnostic->raw_target_i_mps2[2] * speed;
  }
  x = exp(-dt / 0.2);
  speed = diagnostic->target_i_mps2[0] * state->authority_scale;
  diagnostic->target_i_mps2[0] = speed;
  valueNorm = state->filtered_compensation_i_mps2[0];
  augmentation[0] = (valueNorm + (1.0 - x) * (speed - valueNorm)) - valueNorm;
  speed = diagnostic->target_i_mps2[1] * state->authority_scale;
  diagnostic->target_i_mps2[1] = speed;
  valueNorm = state->filtered_compensation_i_mps2[1];
  augmentation[1] = (valueNorm + (1.0 - x) * (speed - valueNorm)) - valueNorm;
  speed = diagnostic->target_i_mps2[2] * state->authority_scale;
  diagnostic->target_i_mps2[2] = speed;
  valueNorm = state->filtered_compensation_i_mps2[2];
  augmentation[2] = (valueNorm + (1.0 - x) * (speed - valueNorm)) - valueNorm;
  speed = 3.0 * dt;
  /* GPENMPCCLIPNORM Radially clip a vector without changing its direction. */
  valueNorm = c_norm(augmentation);
  if (valueNorm > speed) {
    speed /= valueNorm;
    augmentation[0] *= speed;
    augmentation[1] *= speed;
    augmentation[2] *= speed;
  }
  /* GPENMPCCLIPNORM Radially clip a vector without changing its direction. */
  valueNorm = state->filtered_compensation_i_mps2[0] + augmentation[0];
  augmentation[0] = valueNorm;
  state->filtered_compensation_i_mps2[0] = valueNorm;
  valueNorm = state->filtered_compensation_i_mps2[1] + augmentation[1];
  augmentation[1] = valueNorm;
  state->filtered_compensation_i_mps2[1] = valueNorm;
  valueNorm = state->filtered_compensation_i_mps2[2] + augmentation[2];
  augmentation[2] = valueNorm;
  state->filtered_compensation_i_mps2[2] = valueNorm;
  speed = c_norm(augmentation);
  if (speed > 0.75) {
    speed = 0.75 / speed;
    state->filtered_compensation_i_mps2[0] = augmentation[0] * speed;
    state->filtered_compensation_i_mps2[1] = augmentation[1] * speed;
    state->filtered_compensation_i_mps2[2] = valueNorm * speed;
  }
  augmentation[0] = state->filtered_compensation_i_mps2[0];
  augmentation[1] = state->filtered_compensation_i_mps2[1];
  augmentation[2] = state->filtered_compensation_i_mps2[2];
  diagnostic->c_vertical_disturbance_estimate =
      state->vertical_disturbance_ewma_mps2;
  diagnostic->filter_gain = 1.0 - x;
}

/*
 * File trailer for gpenmpcRobustSe3Augmentation.c
 *
 * [EOF]
 */
