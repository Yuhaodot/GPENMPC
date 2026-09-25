/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcObserveCausalVerticalDisturbance.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcObserveCausalVerticalDisturbance.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "gpenmpcResetCausalVerticalDisturbanceObserver.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCOBSERVECAUSALVERTICALDISTURBANCE Finish interval k-1 at sample k.
 *
 *  The observer is common to B1 and B2.  It estimates only the vertical
 *  residual that remains after the known nominal model.  Its estimate is
 *  bounded by the existing total compensation cap and is frozen whenever the
 *  previous control sample saturated or required force projection.
 *
 * Arguments    : i_struct_T *state
 *                const double velocityMps[3]
 *                double timeS
 *                double legIndex
 *                double payloadKg
 *                boolean_T *diagnostic_updated
 *                boolean_T *diagnostic_reset
 *                double *diagnostic_residual_z_mps2
 *                double *diagnostic_estimate_z_mps2
 * Return Type  : boolean_T
 */
boolean_T c_gpenmpcObserveCausalVerticalDi(
    i_struct_T *state, const double velocityMps[3], double timeS,
    double legIndex, double payloadKg, boolean_T *diagnostic_updated,
    boolean_T *diagnostic_reset, double *diagnostic_residual_z_mps2,
    double *diagnostic_estimate_z_mps2)
{
  double dt;
  double residualZ;
  boolean_T diagnostic_interval_valid;
  if (state->c_vertical_observer_observation &&
      (legIndex == state->c_vertical_observer_previous_le) &&
      (fabs(payloadKg - state->c_vertical_observer_previous_pa) <= 1.0E-12)) {
    diagnostic_interval_valid = true;
  } else {
    diagnostic_interval_valid = false;
  }
  dt = timeS - state->c_vertical_observer_previous_ti;
  if (diagnostic_interval_valid && (!rtIsInf(dt) && !rtIsNaN(dt)) &&
      /* Use the measured sample interval for differentiation and filtering.
       * Intervals above 20.0001 ms reset the observer and GP responsibility
       * memory. */
      (dt > 1.0E-9) && (dt <= 0.0200001) &&
      (!rtIsInf(velocityMps[2]) && !rtIsNaN(velocityMps[2]))) {
    diagnostic_interval_valid = true;
  } else {
    diagnostic_interval_valid = false;
  }
  residualZ = 0.0;
  *diagnostic_updated = false;
  *diagnostic_reset = false;
  if (diagnostic_interval_valid) {
    residualZ = (velocityMps[2] - state->c_vertical_observer_previous_ve) / dt -
                state->c_vertical_observer_previous_no;
    if (state->c_vertical_observer_update_enab &&
        (!rtIsInf(residualZ) && !rtIsNaN(residualZ))) {
      /*  Use the F17 causal-history time constant for vertical residual filtering. */
      state->vertical_disturbance_ewma_mps2 =
          fmin(fmax(state->vertical_disturbance_ewma_mps2 +
                        (1.0 - exp(-dt / 0.5)) *
                            (residualZ - state->vertical_disturbance_ewma_mps2),
                    -0.75),
               0.75);
      *diagnostic_updated = true;
    } else {
      state->c_vertical_observer_antiwindup_++;
    }
  } else if (state->c_vertical_observer_observation) {
    c_gpenmpcResetCausalVerticalDist(state, velocityMps, timeS, legIndex,
                                    payloadKg);
    *diagnostic_reset = true;
  }
  /*  Consume the retained interval.  A new one becomes valid only after commit.
   */
  state->c_vertical_observer_observation = false;
  *diagnostic_residual_z_mps2 = residualZ;
  *diagnostic_estimate_z_mps2 = state->vertical_disturbance_ewma_mps2;
  return diagnostic_interval_valid;
}

/*
 * File trailer for gpenmpcObserveCausalVerticalDisturbance.c
 *
 * [EOF]
 */
