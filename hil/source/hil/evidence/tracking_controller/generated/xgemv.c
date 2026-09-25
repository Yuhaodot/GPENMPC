/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgemv.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xgemv.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int n
 *                const double x[9]
 *                double beta1
 *                double y[9]
 *                int iy0
 * Return Type  : void
 */
void b_xgemv(int n, const double x[9], double beta1, double y[9], int iy0)
{
  int ia;
  int iac;
  int iyend;
  iyend = iy0 + 2;
  if (beta1 != 1.0) {
    if (beta1 == 0.0) {
      if (iy0 <= iyend) {
        memset(&y[iy0 + -1], 0,
               (unsigned int)((iyend - iy0) + 1) * sizeof(double));
      }
    } else {
      for (iac = iy0; iac <= iyend; iac++) {
        y[iac - 1] *= beta1;
      }
    }
  }
  iyend = 3 * (n - 1) + 1;
  for (iac = 1; iac <= iyend; iac += 3) {
    for (ia = iac; ia <= iac + 2; ia++) {
      int i;
      i = ((iy0 + ia) - iac) - 1;
      y[i] += y[ia - 1] * x[(iac - 1) / 3 + 6];
    }
  }
}

/*
 * Arguments    : int m
 *                int n
 *                const double A[9]
 *                int ia0
 *                const double x[9]
 *                int ix0
 *                double y[3]
 * Return Type  : void
 */
void xgemv(int m, int n, const double A[9], int ia0, const double x[9], int ix0,
           double y[3])
{
  int ia;
  int iac;
  if (n != 0) {
    int i;
    int i1;
    i = (unsigned char)n;
    memset(&y[0], 0, (unsigned int)i * sizeof(double));
    i1 = ia0 + 3 * (n - 1);
    for (iac = ia0; iac <= i1; iac += 3) {
      double c;
      c = 0.0;
      i = iac + m;
      for (ia = iac; ia < i; ia++) {
        c += A[ia - 1] * x[((ix0 + ia) - iac) - 1];
      }
      i = (iac - ia0) / 3;
      y[i] += c;
    }
  }
}

/*
 * File trailer for xgemv.c
 *
 * [EOF]
 */
