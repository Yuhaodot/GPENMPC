/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgemv.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef XGEMV_H
#define XGEMV_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_xgemv(int n, const double x[9], double beta1, double y[9], int iy0);

void xgemv(int m, int n, const double A[9], int ia0, const double x[9], int ix0,
           double y[3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for xgemv.h
 *
 * [EOF]
 */
