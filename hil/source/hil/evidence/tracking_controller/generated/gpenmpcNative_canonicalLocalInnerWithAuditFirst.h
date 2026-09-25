/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcNative_canonicalLocalInnerWithAuditFirst.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCNATIVE_CANONICALLOCALINNERWITHAUDITFIRST_H
#define GPENMPCNATIVE_CANONICALLOCALINNERWITHAUDITFIRST_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void gpenmpcNative_canonicalLocalInnerWithAuditFirst(
    e_gpenmpcNative_canonicalLocalIn *SD, const double input36[36],
    const unsigned long long inputTags2[2], double next64[64],
    double kernel61[61], double scaffold70[70], double request19[19],
    double closed5[5], double learning12[12]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcNative_canonicalLocalInnerWithAuditFirst.h
 *
 * [EOF]
 */
