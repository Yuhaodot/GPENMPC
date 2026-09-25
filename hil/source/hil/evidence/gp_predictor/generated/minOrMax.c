/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: minOrMax.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-06 17:53:25
 */

/* Include Files */
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double x[3]
 * Return Type  : double
 */
double maximum(const double x[3])
{
  double ex;
  ex = x[0];
  if (x[0] < x[1]) {
    ex = x[1];
  }
  if (ex < x[2]) {
    ex = x[2];
  }
  return ex;
}

/*
 * Arguments    : const double x[256]
 * Return Type  : double
 */
double minimum(const double x[256])
{
  double ex;
  int b_k;
  int idx;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    int k;
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while (!exitg1 && (k <= 256)) {
      if (!rtIsNaN(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (b_k = idx + 1; b_k < 257; b_k++) {
      double d;
      d = x[b_k - 1];
      if (ex > d) {
        ex = d;
      }
    }
  }
  return ex;
}

/*
 * File trailer for minOrMax.c
 *
 * [EOF]
 */
