/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: atan2.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "atan2.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double y
 *                double x
 * Return Type  : double
 */
double b_atan2(double y, double x)
{
  double r;
  if (rtIsNaN(y)) {
    r = rtNaN;
  } else if (rtIsInf(y) && rtIsInf(x)) {
    int i;
    if (y > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    r = atan2(i, 1.0);
  } else {
    r = atan2(y, x);
  }
  return r;
}

/*
 * File trailer for atan2.c
 *
 * [EOF]
 */
