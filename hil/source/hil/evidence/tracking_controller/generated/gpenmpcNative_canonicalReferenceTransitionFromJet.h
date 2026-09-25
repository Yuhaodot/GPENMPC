/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcNative_canonicalReferenceTransitionFromJet.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCNATIVE_CANONICALREFERENCETRANSITIONFROMJET_H
#define GPENMPCNATIVE_CANONICALREFERENCETRANSITIONFROMJET_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void gpenmpcNative_canonicalReferenceTransitionFromJet(
    const double trajectoryJet[12], double progressRate,
    double previousPhaseAcceleration, double targetPhaseAcceleration,
    const double previousOuterCorrectionI[3],
    const double targetOuterCorrectionF[3], double dtS, double jerkLimitMps3,
    struct55_T *transition);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcNative_canonicalReferenceTransitionFromJet.h
 *
 * [EOF]
 */
