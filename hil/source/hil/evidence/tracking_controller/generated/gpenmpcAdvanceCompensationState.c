/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcAdvanceCompensationState.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcAdvanceCompensationState.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCADVANCECOMPENSATIONSTATE Shared filter, slew and cap transition.
 *
 * Arguments    : const double previousCompensationI[3]
 *                const double rawTargetI[3]
 *                double dt
 *                double authorityScale
 *                double transition_raw_target_i_mps2[3]
 *                double transition_target_i_mps2[3]
 *                double transition_low_pass_i_mps2[3]
 *                double transition_delta_i_mps2[3]
 *                double transition_applied_i_mps2[3]
 * Return Type  : double
 */
double c_gpenmpcAdvanceCompensationState(
    const double previousCompensationI[3], const double rawTargetI[3],
    double dt, double authorityScale, double transition_raw_target_i_mps2[3],
    double transition_target_i_mps2[3], double transition_low_pass_i_mps2[3],
    double transition_delta_i_mps2[3], double transition_applied_i_mps2[3])
{
  double transition_filter_gain;
  double valueNorm;
  double x;
  /* GPENMPCCLIPNORM Radially clip a vector without changing its direction. */
  transition_target_i_mps2[0] = rawTargetI[0];
  transition_target_i_mps2[1] = rawTargetI[1];
  transition_target_i_mps2[2] = rawTargetI[2];
  transition_filter_gain = c_norm(rawTargetI);
  if (transition_filter_gain > 0.75) {
    transition_filter_gain = 0.75 / transition_filter_gain;
    transition_target_i_mps2[0] = rawTargetI[0] * transition_filter_gain;
    transition_target_i_mps2[1] = rawTargetI[1] * transition_filter_gain;
    transition_target_i_mps2[2] = rawTargetI[2] * transition_filter_gain;
  }
  x = exp(-dt / 0.2);
  transition_filter_gain = transition_target_i_mps2[0] * authorityScale;
  transition_target_i_mps2[0] = transition_filter_gain;
  transition_filter_gain =
      previousCompensationI[0] +
      (1.0 - x) * (transition_filter_gain - previousCompensationI[0]);
  transition_low_pass_i_mps2[0] = transition_filter_gain;
  transition_delta_i_mps2[0] =
      transition_filter_gain - previousCompensationI[0];
  transition_filter_gain = transition_target_i_mps2[1] * authorityScale;
  transition_target_i_mps2[1] = transition_filter_gain;
  transition_filter_gain =
      previousCompensationI[1] +
      (1.0 - x) * (transition_filter_gain - previousCompensationI[1]);
  transition_low_pass_i_mps2[1] = transition_filter_gain;
  transition_delta_i_mps2[1] =
      transition_filter_gain - previousCompensationI[1];
  transition_filter_gain = transition_target_i_mps2[2] * authorityScale;
  transition_target_i_mps2[2] = transition_filter_gain;
  transition_filter_gain =
      previousCompensationI[2] +
      (1.0 - x) * (transition_filter_gain - previousCompensationI[2]);
  transition_low_pass_i_mps2[2] = transition_filter_gain;
  transition_delta_i_mps2[2] =
      transition_filter_gain - previousCompensationI[2];
  transition_filter_gain = 3.0 * dt;
  /* GPENMPCCLIPNORM Radially clip a vector without changing its direction. */
  valueNorm = c_norm(transition_delta_i_mps2);
  if (valueNorm > transition_filter_gain) {
    transition_filter_gain /= valueNorm;
    transition_delta_i_mps2[0] *= transition_filter_gain;
    transition_delta_i_mps2[1] *= transition_filter_gain;
    transition_delta_i_mps2[2] *= transition_filter_gain;
  }
  transition_applied_i_mps2[0] =
      previousCompensationI[0] + transition_delta_i_mps2[0];
  transition_applied_i_mps2[1] =
      previousCompensationI[1] + transition_delta_i_mps2[1];
  transition_applied_i_mps2[2] =
      previousCompensationI[2] + transition_delta_i_mps2[2];
  /* GPENMPCCLIPNORM Radially clip a vector without changing its direction. */
  transition_filter_gain = c_norm(transition_applied_i_mps2);
  if (transition_filter_gain > 0.75) {
    transition_filter_gain = 0.75 / transition_filter_gain;
    transition_applied_i_mps2[0] *= transition_filter_gain;
    transition_applied_i_mps2[1] *= transition_filter_gain;
    transition_applied_i_mps2[2] *= transition_filter_gain;
  }
  transition_raw_target_i_mps2[0] = rawTargetI[0];
  transition_raw_target_i_mps2[1] = rawTargetI[1];
  transition_raw_target_i_mps2[2] = rawTargetI[2];
  return 1.0 - x;
}

/*
 * File trailer for gpenmpcAdvanceCompensationState.c
 *
 * [EOF]
 */
