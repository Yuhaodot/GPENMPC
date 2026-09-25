/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: any.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "any.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const unsigned char x[32]
 * Return Type  : boolean_T
 */
boolean_T any(const unsigned char x[32])
{
  int k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k < 32)) {
    if (x[k] != 0) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

/*
 * Arguments    : const boolean_T x_data[]
 *                int x_size
 * Return Type  : boolean_T
 */
boolean_T b_any(const boolean_T x_data[], int x_size)
{
  int ix;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  ix = 1;
  exitg1 = false;
  while (!exitg1 && (ix <= x_size)) {
    if (x_data[ix - 1]) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  return y;
}

/*
 * File trailer for any.c
 *
 * [EOF]
 */
