/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xdotc.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xdotc.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int n
 *                const double x[9]
 *                int ix0
 *                const double y[9]
 *                int iy0
 * Return Type  : double
 */
double xdotc(int n, const double x[9], int ix0, const double y[9], int iy0)
{
  double d;
  int k;
  d = 0.0;
  for (k = 0; k < n; k++) {
    d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
  }
  return d;
}

/*
 * File trailer for xdotc.c
 *
 * [EOF]
 */
