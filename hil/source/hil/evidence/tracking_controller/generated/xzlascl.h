/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzlascl.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef XZLASCL_H
#define XZLASCL_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_xzlascl(double cfrom, double cto, int m, double A[3], int iA0);

void c_xzlascl(double cfrom, double cto, int m, double A[2], int iA0);

void xzlascl(double cfrom, double cto, double A[9]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for xzlascl.h
 *
 * [EOF]
 */
