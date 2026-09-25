/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzunghr.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xzunghr.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xgerc.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int ilo
 *                int ihi
 *                double A[9]
 *                const double tau[2]
 * Return Type  : void
 */
void xzunghr(int ilo, int ihi, double A[9], const double tau[2])
{
  int b_i;
  int b_ia;
  int i;
  int ia;
  int ia0;
  int j;
  int nh;
  nh = ihi - ilo;
  for (j = ihi; j >= ilo + 1; j--) {
    b_ia = (j - 1) * 3;
    ia = (unsigned char)(j - 1);
    memset(&A[b_ia], 0, (unsigned int)ia * sizeof(double));
    for (i = j + 1; i <= ihi; i++) {
      A[b_ia + 2] = A[b_ia - 1];
    }
    if (ihi + 1 <= 3) {
      memset(&A[ihi + b_ia], 0,
             (unsigned int)(((b_ia - ihi) - b_ia) + 3) * sizeof(double));
    }
  }
  ia = (unsigned char)ilo;
  for (i = 0; i < ia; i++) {
    b_ia = i * 3;
    A[b_ia] = 0.0;
    A[b_ia + 1] = 0.0;
    A[b_ia + 2] = 0.0;
    A[b_ia + i] = 1.0;
  }
  for (i = ihi + 1; i < 4; i++) {
    b_ia = (i - 1) * 3;
    A[b_ia] = 0.0;
    A[b_ia + 1] = 0.0;
    A[b_ia + 2] = 0.0;
    A[(b_ia + i) - 1] = 1.0;
  }
  ia0 = ilo + ilo * 3;
  if (nh >= 1) {
    double work[3];
    int itau;
    for (i = nh; i < nh; i++) {
      ia = ia0 + i * 3;
      memset(&A[ia], 0, (unsigned int)nh * sizeof(double));
      A[ia + i] = 1.0;
    }
    itau = (ilo + nh) - 2;
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    for (b_i = nh; b_i >= 1; b_i--) {
      int iaii;
      iaii = (ia0 + b_i) + (b_i - 1) * 3;
      if (b_i < nh) {
        int lastv;
        A[iaii - 1] = 1.0;
        if (tau[itau] != 0.0) {
          lastv = nh;
          ia = iaii + nh;
          while ((lastv > 0) && (A[ia - 2] == 0.0)) {
            lastv--;
            ia--;
          }
          ia = 1;
          b_ia = iaii + 2;
          int exitg1;
          do {
            exitg1 = 0;
            if (b_ia + 1 <= (iaii + lastv) + 2) {
              if (A[b_ia] != 0.0) {
                exitg1 = 1;
              } else {
                b_ia++;
              }
            } else {
              ia = 0;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        } else {
          lastv = 0;
          ia = 0;
        }
        if (lastv > 0) {
          xgemv(lastv, ia, A, iaii + 3, A, iaii, work);
          xgerc(lastv, ia, -tau[itau], iaii, work, A, iaii + 3);
        }
        ia = iaii + 1;
        for (i = ia; i <= ia; i++) {
          A[i - 1] *= -tau[itau];
        }
      }
      A[iaii - 1] = 1.0 - tau[itau];
      ia = (unsigned char)(b_i - 1);
      for (i = 0; i < ia; i++) {
        A[iaii - 2] = 0.0;
      }
      itau--;
    }
  }
}

/*
 * File trailer for xzunghr.c
 *
 * [EOF]
 */
