/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: minOrMax.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
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
double b_maximum(const double x[3])
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
    while (!exitg1 && (k < 4)) {
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
    for (b_k = idx + 1; b_k < 4; b_k++) {
      double d;
      d = x[b_k - 1];
      if (ex < d) {
        ex = d;
      }
    }
  }
  return ex;
}

/*
 * Arguments    : const double x[3]
 *                int *idx
 * Return Type  : double
 */
double maximum(const double x[3], int *idx)
{
  double ex;
  int b_idx;
  int b_k;
  if (!rtIsNaN(x[0])) {
    b_idx = 1;
  } else {
    int k;
    boolean_T exitg1;
    b_idx = 0;
    k = 2;
    exitg1 = false;
    while (!exitg1 && (k < 4)) {
      if (!rtIsNaN(x[k - 1])) {
        b_idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (b_idx == 0) {
    ex = x[0];
    *idx = 1;
  } else {
    ex = x[b_idx - 1];
    *idx = b_idx;
    for (b_k = b_idx + 1; b_k < 4; b_k++) {
      double d;
      d = x[b_k - 1];
      if (ex < d) {
        ex = d;
        *idx = b_k;
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
