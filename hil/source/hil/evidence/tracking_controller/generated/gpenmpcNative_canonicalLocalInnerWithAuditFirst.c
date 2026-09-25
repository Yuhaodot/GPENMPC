/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcNative_canonicalLocalInnerWithAuditFirst.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst.h"
#include "canonicalLocalInnerFixedAbi.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Same first kernel, optional read-only CURRENT physical/closed diagnostics.
 *
 * Arguments    : e_gpenmpcNative_canonicalLocalIn *SD
 *                const double input36[36]
 *                const unsigned long long inputTags2[2]
 *                double next64[64]
 *                double kernel61[61]
 *                double scaffold70[70]
 *                double request19[19]
 *                double closed5[5]
 *                double learning12[12]
 * Return Type  : void
 */
void gpenmpcNative_canonicalLocalInnerWithAuditFirst(
    e_gpenmpcNative_canonicalLocalIn *SD, const double input36[36],
    const unsigned long long inputTags2[2], double next64[64],
    double kernel61[61], double scaffold70[70], double request19[19],
    double closed5[5], double learning12[12])
{
  static const signed char b_iv[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0};
  int i;
  for (i = 0; i < 12; i++) {
    learning12[i] = b_iv[i];
  }
  canonicalLocalInnerFixedAbi(SD, input36, inputTags2, next64, kernel61,
                              scaffold70, request19);
  for (i = 0; i < 5; i++) {
    closed5[i] = 0.0;
  }
}

/*
 * File trailer for gpenmpcNative_canonicalLocalInnerWithAuditFirst.c
 *
 * [EOF]
 */
