/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcClipNorm.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcClipNorm.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCCLIPNORM Radially clip a vector without changing its direction.
 *
 * Arguments    : double b_value[3]
 * Return Type  : void
 */
void gpenmpcClipNorm(double b_value[3])
{
  double valueNorm;
  valueNorm = c_norm(b_value);
  if (valueNorm > 0.4) {
    valueNorm = 0.4 / valueNorm;
    b_value[0] *= valueNorm;
    b_value[1] *= valueNorm;
    b_value[2] *= valueNorm;
  }
}

/*
 * File trailer for gpenmpcClipNorm.c
 *
 * [EOF]
 */
