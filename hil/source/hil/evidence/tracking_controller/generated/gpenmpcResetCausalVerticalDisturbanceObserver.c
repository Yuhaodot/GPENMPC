/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcResetCausalVerticalDisturbanceObserver.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcResetCausalVerticalDisturbanceObserver.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCRESETCAUSALVERTICALDISTURBANCEOBSERVER Reset at a context boundary.
 *
 * Arguments    : i_struct_T *state
 *                const double velocityMps[3]
 *                double timeS
 *                double legIndex
 *                double payloadKg
 * Return Type  : void
 */
void c_gpenmpcResetCausalVerticalDist(i_struct_T *state,
                                     const double velocityMps[3], double timeS,
                                     double legIndex, double payloadKg)
{
  state->vertical_disturbance_ewma_mps2 = 0.0;
  state->c_vertical_observer_previous_ve = velocityMps[2];
  state->c_vertical_observer_previous_no = 0.0;
  state->c_vertical_observer_previous_ti = timeS;
  state->c_vertical_observer_previous_le = legIndex;
  state->c_vertical_observer_previous_pa = payloadKg;
  state->c_vertical_observer_observation = false;
  state->c_vertical_observer_update_enab = true;
  state->vertical_observer_reset_count++;
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
 * File trailer for gpenmpcResetCausalVerticalDisturbanceObserver.c
 *
 * [EOF]
 */
