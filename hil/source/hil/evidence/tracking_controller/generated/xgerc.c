/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xgerc.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xgerc.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int m
 *                int n
 *                double alpha1
 *                int ix0
 *                const double y[3]
 *                double A[9]
 *                int ia0
 * Return Type  : void
 */
void xgerc(int m, int n, double alpha1, int ix0, const double y[3], double A[9],
           int ia0)
{
  int ijA;
  int j;
  if (!(alpha1 == 0.0)) {
    int i;
    int jA;
    jA = ia0;
    i = (unsigned char)n;
    for (j = 0; j < i; j++) {
      double temp;
      temp = y[j];
      if (temp != 0.0) {
        int i1;
        temp *= alpha1;
        i1 = m + jA;
        for (ijA = jA; ijA < i1; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }
      jA += 3;
    }
  }
}

/*
 * File trailer for xgerc.c
 *
 * [EOF]
 */
