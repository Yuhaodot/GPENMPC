/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mrdivide_helper.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double A[24]
 *                const double B[16]
 * Return Type  : void
 */
void mrdiv(double A[24], const double B[16])
{
  double b_A[16];
  double smax;
  int i;
  int j;
  int jA;
  int jBcol;
  int k;
  int kBcol;
  int mmj;
  signed char ipiv[4];
  memcpy(&b_A[0], &B[0], 16U * sizeof(double));
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (j = 0; j < 3; j++) {
    int jj;
    mmj = 2 - j;
    jBcol = j * 5;
    jj = j * 5;
    jA = 5 - j;
    kBcol = 0;
    smax = fabs(b_A[jj]);
    for (k = 2; k < jA; k++) {
      double s;
      s = fabs(b_A[(jBcol + k) - 1]);
      if (s > smax) {
        kBcol = k - 1;
        smax = s;
      }
    }
    if (b_A[jj + kBcol] != 0.0) {
      if (kBcol != 0) {
        jA = j + kBcol;
        ipiv[j] = (signed char)(jA + 1);
        smax = b_A[j];
        b_A[j] = b_A[jA];
        b_A[jA] = smax;
        smax = b_A[j + 4];
        b_A[j + 4] = b_A[jA + 4];
        b_A[jA + 4] = smax;
        smax = b_A[j + 8];
        b_A[j + 8] = b_A[jA + 8];
        b_A[jA + 8] = smax;
        smax = b_A[j + 12];
        b_A[j + 12] = b_A[jA + 12];
        b_A[jA + 12] = smax;
      }
      kBcol = (jj - j) + 4;
      for (i = jBcol + 2; i <= kBcol; i++) {
        b_A[i - 1] /= b_A[jj];
      }
    }
    jA = jj;
    for (i = 0; i <= mmj; i++) {
      smax = b_A[(jBcol + (i << 2)) + 4];
      if (smax != 0.0) {
        kBcol = (jA - j) + 8;
        for (k = jA + 6; k <= kBcol; k++) {
          b_A[k - 1] += b_A[((jj + k) - jA) - 5] * -smax;
        }
      }
      jA += 4;
    }
  }
  for (j = 0; j < 4; j++) {
    jBcol = 6 * j - 1;
    jA = j << 2;
    for (i = 0; i < j; i++) {
      kBcol = 6 * i;
      smax = b_A[i + jA];
      if (smax != 0.0) {
        for (k = 0; k < 6; k++) {
          mmj = (k + jBcol) + 1;
          A[mmj] -= smax * A[k + kBcol];
        }
      }
    }
    smax = 1.0 / b_A[j + jA];
    for (i = 0; i < 6; i++) {
      jA = (i + jBcol) + 1;
      A[jA] *= smax;
    }
  }
  for (i = 3; i >= 0; i--) {
    jA = 6 * i - 1;
    kBcol = (i << 2) - 1;
    for (k = i + 2; k < 5; k++) {
      mmj = 6 * (k - 1);
      smax = b_A[k + kBcol];
      if (smax != 0.0) {
        for (j = 0; j < 6; j++) {
          jBcol = (j + jA) + 1;
          A[jBcol] -= smax * A[j + mmj];
        }
      }
    }
  }
  for (i = 2; i >= 0; i--) {
    signed char b_i;
    b_i = ipiv[i];
    if (b_i != i + 1) {
      for (k = 0; k < 6; k++) {
        jA = k + 6 * i;
        smax = A[jA];
        kBcol = k + 6 * (b_i - 1);
        A[jA] = A[kBcol];
        A[kBcol] = smax;
      }
    }
  }
}

/*
 * File trailer for mrdivide_helper.c
 *
 * [EOF]
 */
