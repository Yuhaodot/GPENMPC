/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: diag.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "diag.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double d[9]
 * Return Type  : void
 */
void diag(double d[9])
{
  memset(&d[0], 0, 9U * sizeof(double));
  d[0] = 1.6;
  d[4] = 1.6;
  d[8] = 3.0;
}

/*
 * File trailer for diag.c
 *
 * [EOF]
 */
