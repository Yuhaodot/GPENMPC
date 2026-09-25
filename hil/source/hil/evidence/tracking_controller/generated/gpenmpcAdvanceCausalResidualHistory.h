/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcAdvanceCausalResidualHistory.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCADVANCECAUSALRESIDUALHISTORY_H
#define GPENMPCADVANCECAUSALRESIDUALHISTORY_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
boolean_T c_gpenmpcAdvanceCausalResidualHi(
    const double historyF[3], const double velocityAtK[3],
    const double velocityAtKp1[3], const double nominalAccelerationAtK[3],
    const double frameIFromFAtK[9], double dt, double nextHistoryF[3],
    double residualF[3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcAdvanceCausalResidualHistory.h
 *
 * [EOF]
 */
