/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: any.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-06 17:53:25
 */

/* Include Files */
#include "any.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const boolean_T x[3]
 * Return Type  : boolean_T
 */
boolean_T any(const boolean_T x[3])
{
  int k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k < 3)) {
    if (x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

/*
 * File trailer for any.c
 *
 * [EOF]
 */
