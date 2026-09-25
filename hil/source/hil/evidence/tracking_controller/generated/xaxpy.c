/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xaxpy.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xaxpy.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double a
 *                const double x[9]
 *                int ix0
 *                double y[3]
 * Return Type  : void
 */
void b_xaxpy(double a, const double x[9], int ix0, double y[3])
{
  int k;
  if (!(a == 0.0)) {
    for (k = 0; k < 2; k++) {
      y[k + 1] += a * x[(ix0 + k) - 1];
    }
  }
}

/*
 * Arguments    : double a
 *                const double x[3]
 *                double y[9]
 *                int iy0
 * Return Type  : void
 */
void c_xaxpy(double a, const double x[3], double y[9], int iy0)
{
  int k;
  if (!(a == 0.0)) {
    for (k = 0; k < 2; k++) {
      int i;
      i = (iy0 + k) - 1;
      y[i] += a * x[k + 1];
    }
  }
}

/*
 * Arguments    : int n
 *                double a
 *                int ix0
 *                double y[9]
 *                int iy0
 * Return Type  : void
 */
void xaxpy(int n, double a, int ix0, double y[9], int iy0)
{
  int k;
  if (!(a == 0.0)) {
    for (k = 0; k < n; k++) {
      int i;
      i = (iy0 + k) - 1;
      y[i] += a * y[(ix0 + k) - 1];
    }
  }
}

/*
 * File trailer for xaxpy.c
 *
 * [EOF]
 */
