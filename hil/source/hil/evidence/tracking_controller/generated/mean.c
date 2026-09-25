/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mean.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "mean.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double x[3]
 * Return Type  : double
 */
double mean(const double x[3])
{
  return ((x[0] + x[1]) + x[2]) / 3.0;
}

/*
 * File trailer for mean.c
 *
 * [EOF]
 */
