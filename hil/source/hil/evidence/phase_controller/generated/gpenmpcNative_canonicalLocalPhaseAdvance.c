/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcNative_canonicalLocalPhaseAdvance.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-06 21:28:59
 */

/* Include Files */
#include "gpenmpcNative_canonicalLocalPhaseAdvance.h"
#include <math.h>

/* Function Definitions */
/*
 * Advance candidate phase and rate after the reference transition.
 *  input = [phase;rate;acceptedAcceleration;dt;duration;rateMin;rateMax].
 *  The execution owner commits the result with the matching
 *  numerical/reference publication receipt.
 *
 * Arguments    : const double input7[7]
 *                double next[2]
 * Return Type  : void
 */
void gpenmpcNative_canonicalLocalPhaseAdvance(const double input7[7],
                                             double next[2])
{
  next[0] =
      fmin(input7[4], fmax(0.0, (input7[0] + input7[1] * input7[3]) +
                                    0.5 * (input7[3] * input7[3]) * input7[2]));
  next[1] = fmin(fmax(input7[1] + input7[2] * input7[3], input7[5]), input7[6]);
}

/*
 * File trailer for gpenmpcNative_canonicalLocalPhaseAdvance.c
 *
 * [EOF]
 */
