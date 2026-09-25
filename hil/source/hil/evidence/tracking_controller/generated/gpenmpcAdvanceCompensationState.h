/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcAdvanceCompensationState.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCADVANCECOMPENSATIONSTATE_H
#define GPENMPCADVANCECOMPENSATIONSTATE_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double c_gpenmpcAdvanceCompensationState(
    const double previousCompensationI[3], const double rawTargetI[3],
    double dt, double authorityScale, double transition_raw_target_i_mps2[3],
    double transition_target_i_mps2[3], double transition_low_pass_i_mps2[3],
    double transition_delta_i_mps2[3], double transition_applied_i_mps2[3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcAdvanceCompensationState.h
 *
 * [EOF]
 */
