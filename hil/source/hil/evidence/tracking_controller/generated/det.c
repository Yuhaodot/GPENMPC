/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: det.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "det.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double x[9]
 * Return Type  : double
 */
double det(const double x[9])
{
  double b_x[9];
  double y;
  int ijA;
  int j;
  int k;
  signed char ipiv[3];
  boolean_T isodd;
  memcpy(&b_x[0], &x[0], 9U * sizeof(double));
  ipiv[0] = 1;
  ipiv[1] = 2;
  for (j = 0; j < 2; j++) {
    int a;
    int b;
    int jA;
    int mmj;
    mmj = 1 - j;
    b = j << 2;
    jA = 4 - j;
    a = 0;
    y = fabs(b_x[b]);
    for (k = 2; k < jA; k++) {
      double s;
      s = fabs(b_x[(b + k) - 1]);
      if (s > y) {
        a = k - 1;
        y = s;
      }
    }
    if (b_x[b + a] != 0.0) {
      if (a != 0) {
        jA = j + a;
        ipiv[j] = (signed char)(jA + 1);
        y = b_x[j];
        b_x[j] = b_x[jA];
        b_x[jA] = y;
        y = b_x[j + 3];
        b_x[j + 3] = b_x[jA + 3];
        b_x[jA + 3] = y;
        y = b_x[j + 6];
        b_x[j + 6] = b_x[jA + 6];
        b_x[jA + 6] = y;
      }
      jA = (b - j) + 3;
      for (k = b + 2; k <= jA; k++) {
        b_x[k - 1] /= b_x[b];
      }
    }
    jA = b;
    for (k = 0; k <= mmj; k++) {
      y = b_x[(b + k * 3) + 3];
      if (y != 0.0) {
        a = (jA - j) + 6;
        for (ijA = jA + 5; ijA <= a; ijA++) {
          b_x[ijA - 1] += b_x[((b + ijA) - jA) - 4] * -y;
        }
      }
      jA += 3;
    }
  }
  isodd = (ipiv[0] > 1);
  y = b_x[0] * b_x[4] * b_x[8];
  if (ipiv[1] > 2) {
    isodd = !isodd;
  }
  if (isodd) {
    y = -y;
  }
  return y;
}

/*
 * File trailer for det.c
 *
 * [EOF]
 */
