/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcAdvanceCausalResidualHistory.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcAdvanceCausalResidualHistory.h"
#include "all.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCADVANCECAUSALRESIDUALHISTORY Advance the strictly causal F17 history.
 *
 *  The interval-k label becomes available only after velocity k+1 arrives:
 *    r(k) = (v_hat(k+1)-v_hat(k))/dt(k) - a_nominal_known(k).
 *  The stored frame and nominal acceleration must therefore both belong to k.
 *
 * Arguments    : const double historyF[3]
 *                const double velocityAtK[3]
 *                const double velocityAtKp1[3]
 *                const double nominalAccelerationAtK[3]
 *                const double frameIFromFAtK[9]
 *                double dt
 *                double nextHistoryF[3]
 *                double residualF[3]
 * Return Type  : boolean_T
 */
boolean_T c_gpenmpcAdvanceCausalResidualHi(
    const double historyF[3], const double velocityAtK[3],
    const double velocityAtKp1[3], const double nominalAccelerationAtK[3],
    const double frameIFromFAtK[9], double dt, double nextHistoryF[3],
    double residualF[3])
{
  double velocityAtKp1_idx_0;
  int i;
  boolean_T bv[3];
  boolean_T labelValid;
  bv[0] = (!rtIsInf(velocityAtK[0]) && !rtIsNaN(velocityAtK[0]));
  bv[1] = (!rtIsInf(velocityAtK[1]) && !rtIsNaN(velocityAtK[1]));
  bv[2] = (!rtIsInf(velocityAtK[2]) && !rtIsNaN(velocityAtK[2]));
  if (all(bv)) {
    bv[0] = (!rtIsInf(velocityAtKp1[0]) && !rtIsNaN(velocityAtKp1[0]));
    bv[1] = (!rtIsInf(velocityAtKp1[1]) && !rtIsNaN(velocityAtKp1[1]));
    bv[2] = (!rtIsInf(velocityAtKp1[2]) && !rtIsNaN(velocityAtKp1[2]));
    if (all(bv)) {
      bv[0] = (!rtIsInf(nominalAccelerationAtK[0]) &&
               !rtIsNaN(nominalAccelerationAtK[0]));
      bv[1] = (!rtIsInf(nominalAccelerationAtK[1]) &&
               !rtIsNaN(nominalAccelerationAtK[1]));
      bv[2] = (!rtIsInf(nominalAccelerationAtK[2]) &&
               !rtIsNaN(nominalAccelerationAtK[2]));
      if (all(bv)) {
        boolean_T bv1[9];
        for (i = 0; i < 9; i++) {
          velocityAtKp1_idx_0 = frameIFromFAtK[i];
          bv1[i] =
              (!rtIsInf(velocityAtKp1_idx_0) && !rtIsNaN(velocityAtKp1_idx_0));
        }
        if (b_all(bv1)) {
          labelValid = true;
        } else {
          labelValid = false;
        }
      } else {
        labelValid = false;
      }
    } else {
      labelValid = false;
    }
  } else {
    labelValid = false;
  }
  if (labelValid) {
    double velocityAtKp1_idx_1;
    double velocityAtKp1_idx_2;
    velocityAtKp1_idx_0 =
        (velocityAtKp1[0] - velocityAtK[0]) / dt - nominalAccelerationAtK[0];
    velocityAtKp1_idx_1 =
        (velocityAtKp1[1] - velocityAtK[1]) / dt - nominalAccelerationAtK[1];
    velocityAtKp1_idx_2 =
        (velocityAtKp1[2] - velocityAtK[2]) / dt - nominalAccelerationAtK[2];
    for (i = 0; i < 3; i++) {
      residualF[i] = (frameIFromFAtK[3 * i] * velocityAtKp1_idx_0 +
                      frameIFromFAtK[3 * i + 1] * velocityAtKp1_idx_1) +
                     frameIFromFAtK[3 * i + 2] * velocityAtKp1_idx_2;
    }
  } else {
    /*  Match the frozen offline history convention at context boundaries: an */
    /*  invalid label is not used, and the retained history decays toward zero.
     */
    residualF[0] = 0.0;
    residualF[1] = 0.0;
    residualF[2] = 0.0;
  }
  velocityAtKp1_idx_0 = exp(-fmax(dt, 0.0) / 0.5);
  nextHistoryF[0] =
      historyF[0] + (1.0 - velocityAtKp1_idx_0) * (residualF[0] - historyF[0]);
  nextHistoryF[1] =
      historyF[1] + (1.0 - velocityAtKp1_idx_0) * (residualF[1] - historyF[1]);
  nextHistoryF[2] =
      historyF[2] + (1.0 - velocityAtKp1_idx_0) * (residualF[2] - historyF[2]);
  return labelValid;
}

/*
 * File trailer for gpenmpcAdvanceCausalResidualHistory.c
 *
 * [EOF]
 */
