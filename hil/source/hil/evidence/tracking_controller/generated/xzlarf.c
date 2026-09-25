/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzlarf.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xzlarf.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xgerc.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int m
 *                int n
 *                int iv0
 *                double tau
 *                double C[9]
 *                int ic0
 *                double work[3]
 * Return Type  : void
 */
void xzlarf(int m, int n, int iv0, double tau, double C[9], int ic0,
            double work[3])
{
  int i;
  int lastv;
  if (tau != 0.0) {
    boolean_T exitg2;
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C[i - 2] == 0.0)) {
      lastv--;
      i--;
    }
    i = n;
    exitg2 = false;
    while (!exitg2 && (i > 0)) {
      int coltop;
      int exitg1;
      int ia;
      coltop = ic0 + (i - 1) * 3;
      ia = coltop;
      do {
        exitg1 = 0;
        if (ia <= (coltop + lastv) - 1) {
          if (C[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          i--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    i = 0;
  }
  if (lastv > 0) {
    xgemv(lastv, i, C, ic0, C, iv0, work);
    xgerc(lastv, i, -tau, iv0, work, C, ic0);
  }
}

/*
 * File trailer for xzlarf.c
 *
 * [EOF]
 */
