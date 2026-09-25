/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcResetCausalVerticalDisturbanceObserver.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCRESETCAUSALVERTICALDISTURBANCEOBSERVER_H
#define GPENMPCRESETCAUSALVERTICALDISTURBANCEOBSERVER_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_gpenmpcResetCausalVerticalDist(i_struct_T *state,
                                     const double velocityMps[3], double timeS,
                                     double legIndex, double payloadKg);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcResetCausalVerticalDisturbanceObserver.h
 *
 * [EOF]
 */
