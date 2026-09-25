/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: allOrAny.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "allOrAny.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const boolean_T x_data[]
 *                int x_size
 * Return Type  : boolean_T
 */
boolean_T vectorAny(const boolean_T x_data[], int x_size)
{
  int k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k <= x_size - 1)) {
    if (x_data[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

/*
 * File trailer for allOrAny.c
 *
 * [EOF]
 */
