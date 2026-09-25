/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcUpdateDesiredAttitudeContinuity.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCUPDATEDESIREDATTITUDECONTINUITY_H
#define GPENMPCUPDATEDESIREDATTITUDECONTINUITY_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_gpenmpcUpdateDesiredAttitudeCo(
    e_gpenmpcNative_canonicalLocalIn *SD, boolean_T state_initialized,
    boolean_T state_angular_velocity_valid,
    const double state_filtered_rotation[9],
    const double c_state_desired_angular_velocit[3],
    const double c_state_desired_angular_acceler[3], double state_update_count,
    double state_reset_count, const double rawDesiredRotation[9],
    double actualDtS, boolean_T resetRequested, k_struct_T *command,
    o_struct_T *nextState);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcUpdateDesiredAttitudeContinuity.h
 *
 * [EOF]
 */
