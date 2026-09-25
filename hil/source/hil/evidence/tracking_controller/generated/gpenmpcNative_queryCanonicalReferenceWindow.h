/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcNative_queryCanonicalReferenceWindow.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCNATIVE_QUERYCANONICALREFERENCEWINDOW_H
#define GPENMPCNATIVE_QUERYCANONICALREFERENCEWINDOW_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void gpenmpcNative_queryCanonicalReferenceWindow(
    e_gpenmpcNative_canonicalLocalIn *SD, const struct51_T *window,
    const struct52_T *state, const struct53_T *request, struct52_T *next,
    double jet[12], struct54_T *receipt);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcNative_queryCanonicalReferenceWindow.h
 *
 * [EOF]
 */
