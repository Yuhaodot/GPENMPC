/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcNative_canonicalLocalInnerWithAuditStep.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCNATIVE_CANONICALLOCALINNERWITHAUDITSTEP_H
#define GPENMPCNATIVE_CANONICALLOCALINNERWITHAUDITSTEP_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void gpenmpcNative_canonicalLocalInnerWithAuditStep(
    e_gpenmpcNative_canonicalLocalIn *SD, const double state64[64],
    const unsigned long long stateTags2[2], const double input36[36],
    const unsigned long long inputTags2[2], const double pending70[70],
    const unsigned long long pendingTags2[2], double next64[64],
    double kernel61[61], double scaffold70[70], double request19[19],
    double closed5[5], double learning12[12]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcNative_canonicalLocalInnerWithAuditStep.h
 *
 * [EOF]
 */
