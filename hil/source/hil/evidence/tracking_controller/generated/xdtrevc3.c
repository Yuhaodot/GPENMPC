/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xdtrevc3.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xdtrevc3.h"
#include "rt_nonfinite.h"
#include "xdlaln2.h"
#include "xgemv.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double T[9]
 *                double vr[9]
 * Return Type  : void
 */
void xdtrevc3(const double T[9], double vr[9])
{
  double work[9];
  double x[4];
  double smax;
  int i;
  int ip;
  int j;
  memset(&work[0], 0, 9U * sizeof(double));
  x[0] = 0.0;
  x[1] = 0.0;
  x[2] = 0.0;
  x[3] = 0.0;
  work[0] = 0.0;
  for (j = 0; j < 2; j++) {
    work[j + 1] = 0.0;
    for (i = 0; i <= j; i++) {
      work[j + 1] += fabs(T[i + 3 * (j + 1)]);
    }
  }
  ip = 0;
  for (i = 2; i >= 0; i--) {
    if (ip == -1) {
      ip = 1;
    } else {
      double smin;
      double wi;
      double wr;
      int b_j;
      if ((i == 0) || (T[i + 3 * (i - 1)] == 0.0)) {
        ip = 0;
      } else {
        ip = -1;
      }
      b_j = i + 3 * i;
      wr = T[b_j];
      wi = 0.0;
      if (ip != 0) {
        wi = sqrt(fabs(T[i + 3 * (i - 1)])) * sqrt(fabs(T[b_j - 1]));
      }
      smin =
          fmax(2.220446049250313E-16 * (fabs(wr) + wi), 3.006252540013459E-292);
      if (ip == 0) {
        double scale;
        int ii;
        work[i + 6] = 1.0;
        for (j = 0; j < i; j++) {
          work[j + 6] = -T[j + 3 * i];
        }
        b_j = i - 1;
        while (b_j + 1 >= 1) {
          if ((b_j + 1 == 1) || (T[1] == 0.0)) {
            scale = xdlaln2(1, 1, smin, T, (b_j * 3 + b_j) + 1, work, b_j + 7,
                            wr, 0.0, x, &smax);
            if ((smax > 1.0) && (work[b_j] > 3.326400515891199E+291 / smax)) {
              x[0] /= smax;
              scale /= smax;
            }
            if (scale != 1.0) {
              for (j = 7; j <= i + 7; j++) {
                work[j - 1] *= scale;
              }
            }
            work[b_j + 6] = x[0];
            if ((b_j >= 1) && !(-x[0] == 0.0)) {
              work[6] += -x[0] * T[b_j * 3];
            }
            b_j--;
          } else {
            scale = xdlaln2(2, 1, smin, T, 1, work, 7, wr, 0.0, x, &smax);
            if ((smax > 1.0) &&
                (fmax(0.0, work[1]) > 3.326400515891199E+291 / smax)) {
              x[0] /= smax;
              x[1] /= smax;
              scale /= smax;
            }
            if (scale != 1.0) {
              for (j = 7; j <= i + 7; j++) {
                work[j - 1] *= scale;
              }
            }
            work[6] = x[0];
            work[7] = x[1];
            b_j = -1;
          }
        }
        if (i > 0) {
          b_xgemv(i, work, work[i + 6], vr, i * 3 + 1);
        }
        b_j = i * 3;
        ii = 0;
        smax = fabs(vr[b_j]);
        scale = fabs(vr[b_j + 1]);
        if (scale > smax) {
          ii = 1;
          smax = scale;
        }
        if (fabs(vr[b_j + 2]) > smax) {
          ii = 2;
        }
        smax = 1.0 / fabs(vr[ii + 3 * i]);
        b_j = i * 3 + 1;
        for (j = b_j; j <= b_j + 2; j++) {
          vr[j - 1] *= smax;
        }
      } else {
        double scale;
        int b_i;
        int ii;
        b_i = 3 * (i - 1);
        smax = T[i + b_i];
        scale = T[b_j - 1];
        if (fabs(scale) >= fabs(smax)) {
          work[i + 2] = 1.0;
          work[i + 6] = wi / scale;
        } else {
          work[i + 2] = -wi / smax;
          work[i + 6] = 1.0;
        }
        work[i + 3] = 0.0;
        work[i + 5] = 0.0;
        for (j = 0; j <= i - 2; j++) {
          work[3] = -work[i + 2] * T[b_i];
          work[6] = -work[i + 6] * T[3 * i];
        }
        b_j = i - 1;
        while (b_j >= 1) {
          smax = xdlaln2(1, 2, smin, T, 1, work, 4, wr, wi, x, &smax);
          if (smax != 1.0) {
            for (j = 4; j <= i + 4; j++) {
              work[j - 1] *= smax;
            }
            for (j = 7; j <= i + 7; j++) {
              work[j - 1] *= smax;
            }
          }
          work[3] = x[0];
          work[6] = x[2];
          b_j = 0;
        }
        if (i + 1 > 2) {
          if (work[4] != 1.0) {
            if (work[4] == 0.0) {
              vr[3] = 0.0;
              vr[4] = 0.0;
              vr[5] = 0.0;
            } else {
              vr[3] *= work[4];
              vr[4] *= work[4];
              vr[5] *= work[4];
            }
          }
          vr[3] += vr[0] * work[3];
          vr[4] += vr[1] * work[3];
          vr[5] += vr[2] * work[3];
          b_xgemv(1, work, work[8], vr, 7);
        } else {
          ii = b_i + 1;
          for (j = ii; j <= ii + 2; j++) {
            vr[j - 1] *= work[3];
          }
          ii = i * 3 + 1;
          for (j = ii; j <= ii + 2; j++) {
            vr[j - 1] *= work[i + 6];
          }
        }
        ii = b_i + 1;
        b_j = 3 * i + 1;
        smax = 1.0 / fmax(fmax(fmax(0.0, fabs(vr[b_i]) + fabs(vr[3 * i])),
                               fabs(vr[b_i + 1]) + fabs(vr[b_j])),
                          fabs(vr[b_i + 2]) + fabs(vr[3 * i + 2]));
        for (j = ii; j <= ii + 2; j++) {
          vr[j - 1] *= smax;
        }
        for (j = b_j; j <= b_j + 2; j++) {
          vr[j - 1] *= smax;
        }
      }
    }
  }
}

/*
 * File trailer for xdtrevc3.c
 *
 * [EOF]
 */
