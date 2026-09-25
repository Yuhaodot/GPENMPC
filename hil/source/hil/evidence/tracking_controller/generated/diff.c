/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: diff.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "diff.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double x_data[]
 *                int x_size
 *                double y_data[]
 * Return Type  : int
 */
int diff(const double x_data[], int x_size, double y_data[])
{
  double work_data;
  int m;
  int y_size;
  y_size = x_size - 1;
  work_data = x_data[0];
  for (m = 2; m <= x_size; m++) {
    double tmp2;
    tmp2 = work_data;
    work_data = x_data[m - 1];
    y_data[m - 2] = work_data - tmp2;
  }
  return y_size;
}

/*
 * File trailer for diff.c
 *
 * [EOF]
 */
