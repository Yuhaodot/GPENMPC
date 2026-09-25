/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: svd.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "svd.h"
#include "rt_nonfinite.h"
#include "svd1.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double A[9]
 *                double U[9]
 *                double S[9]
 *                double V[9]
 * Return Type  : void
 */
void svd(const double A[9], double U[9], double S[9], double V[9])
{
  double s[3];
  int k;
  boolean_T p;
  p = true;
  for (k = 0; k < 9; k++) {
    if (p) {
      double d;
      d = A[k];
      if (rtIsInf(d) || rtIsNaN(d)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (p) {
    b_svd(A, U, s, V);
  } else {
    s[0] = rtNaN;
    s[1] = rtNaN;
    s[2] = rtNaN;
    for (k = 0; k < 9; k++) {
      U[k] = rtNaN;
      V[k] = rtNaN;
    }
  }
  memset(&S[0], 0, 9U * sizeof(double));
  S[0] = s[0];
  S[4] = s[1];
  S[8] = s[2];
}

/*
 * File trailer for svd.c
 *
 * [EOF]
 */
