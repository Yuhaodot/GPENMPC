/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: all.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "all.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const boolean_T x[3]
 * Return Type  : boolean_T
 */
boolean_T all(const boolean_T x[3])
{
  int k;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k <= 2)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

/*
 * Arguments    : const boolean_T x[9]
 * Return Type  : boolean_T
 */
boolean_T b_all(const boolean_T x[9])
{
  int k;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k <= 8)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

/*
 * File trailer for all.c
 *
 * [EOF]
 */
