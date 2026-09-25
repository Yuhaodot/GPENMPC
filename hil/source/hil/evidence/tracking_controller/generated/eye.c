/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: eye.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "eye.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double b_I[9]
 * Return Type  : void
 */
void eye(double b_I[9])
{
  memset(&b_I[0], 0, 9U * sizeof(double));
  b_I[0] = 1.0;
  b_I[4] = 1.0;
  b_I[8] = 1.0;
}

/*
 * File trailer for eye.c
 *
 * [EOF]
 */
