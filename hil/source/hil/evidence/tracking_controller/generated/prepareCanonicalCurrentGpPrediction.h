/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: prepareCanonicalCurrentGpPrediction.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef PREPARECANONICALCURRENTGPPREDICTION_H
#define PREPARECANONICALCURRENTGPPREDICTION_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
boolean_T c_prepareCanonicalCurrentGpPred(
    const double velocityI[3], const double reference_velocity_mps[3],
    const double reference_acceleration_mps2[3], const double windEstimateXY[2],
    double payloadKg, const double desiredForceI[3],
    const double previousRotorCommandN[6], const double residualHistoryF[3],
    boolean_T causalValid, char prepared_schema_Value[39],
    e_struct_T *prepared_pending, double prepared_features_f17[17],
    double *prepared_gp_mean_scale);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for prepareCanonicalCurrentGpPrediction.h
 *
 * [EOF]
 */
