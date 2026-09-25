/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcObserveCausalVerticalDisturbance.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCOBSERVECAUSALVERTICALDISTURBANCE_H
#define GPENMPCOBSERVECAUSALVERTICALDISTURBANCE_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
boolean_T c_gpenmpcObserveCausalVerticalDi(
    i_struct_T *state, const double velocityMps[3], double timeS,
    double legIndex, double payloadKg, boolean_T *diagnostic_updated,
    boolean_T *diagnostic_reset, double *diagnostic_residual_z_mps2,
    double *diagnostic_estimate_z_mps2);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcObserveCausalVerticalDisturbance.h
 *
 * [EOF]
 */
