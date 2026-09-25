/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xdlahqr.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xdlahqr.h"
#include "rt_nonfinite.h"
#include "xdlanv2.h"
#include "xzlarfg.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int ilo
 *                int ihi
 *                double h[9]
 *                int iloz
 *                int ihiz
 *                double z[9]
 *                double wr[3]
 *                double wi[3]
 * Return Type  : int
 */
int xdlahqr(int ilo, int ihi, double h[9], int iloz, int ihiz, double z[9],
            double wr[3], double wi[3])
{
  double h11;
  double h12;
  double h21;
  double h22;
  double rt1r;
  double rt2r;
  double s;
  int b_k;
  int i;
  int info;
  int k;
  info = 0;
  k = (unsigned char)(ilo - 1);
  for (i = 0; i < k; i++) {
    wr[i] = h[i + 3 * i];
    wi[i] = 0.0;
  }
  for (i = ihi + 1; i < 4; i++) {
    wr[i - 1] = h[(i + 3 * (i - 1)) - 1];
    wi[i - 1] = 0.0;
  }
  if (ilo == ihi) {
    wr[ilo - 1] = h[(ilo + 3 * (ilo - 1)) - 1];
    wi[ilo - 1] = 0.0;
  } else {
    double smlnum;
    int b_i;
    int kdefl;
    int nz;
    boolean_T exitg1;
    if (ilo <= ihi - 2) {
      h[ihi - 1] = 0.0;
    }
    nz = (ihiz - iloz) + 1;
    smlnum = 2.2250738585072014E-308 *
             ((double)((ihi - ilo) + 1) / 2.220446049250313E-16);
    kdefl = 0;
    b_i = ihi - 1;
    exitg1 = false;
    while (!exitg1 && (b_i + 1 >= ilo)) {
      int ix;
      int ix_tmp;
      int l;
      int temp_tmp_tmp;
      boolean_T converged;
      boolean_T exitg2;
      l = ilo;
      converged = false;
      ix = 0;
      exitg2 = false;
      while (!exitg2 && (ix < 301)) {
        boolean_T exitg3;
        k = b_i;
        exitg3 = false;
        while (!exitg3 && (k + 1 > l)) {
          ix_tmp = k + 3 * (k - 1);
          rt1r = fabs(h[ix_tmp]);
          if (rt1r <= smlnum) {
            exitg3 = true;
          } else {
            temp_tmp_tmp = k + 3 * k;
            h21 = fabs(h[temp_tmp_tmp]);
            h11 = fabs(h[ix_tmp - 1]) + h21;
            if (h11 == 0.0) {
              if (k - 1 >= ilo) {
                h11 = fabs(h[k - 1]);
              }
              if (k + 2 <= ihi) {
                h11 += fabs(h[3 * k + 2]);
              }
            }
            if (rt1r <= 2.220446049250313E-16 * h11) {
              h11 = fabs(h[temp_tmp_tmp - 1]);
              h12 = fabs(h[ix_tmp - 1] - h[temp_tmp_tmp]);
              rt2r = fmax(h21, h12);
              h12 = fmin(h21, h12);
              s = rt2r + h12;
              if (fmin(rt1r, h11) * (fmax(rt1r, h11) / s) <=
                  fmax(smlnum, 2.220446049250313E-16 * (h12 * (rt2r / s)))) {
                exitg3 = true;
              } else {
                k--;
              }
            } else {
              k--;
            }
          }
        }
        l = k + 1;
        if (k + 1 > ilo) {
          h[k + 3 * (k - 1)] = 0.0;
        }
        if (k + 1 >= b_i) {
          converged = true;
          exitg2 = true;
        } else {
          double v[3];
          kdefl++;
          if (kdefl - kdefl / 20 * 20 == 0) {
            s = fabs(h[b_i + 3 * (b_i - 1)]) + fabs(h[b_i - 1]);
            h11 = 0.75 * s + h[b_i + 3 * b_i];
            h12 = -0.4375 * s;
            h21 = s;
            h22 = h11;
          } else if (kdefl - kdefl / 10 * 10 == 0) {
            s = fabs(h[1]) + fabs(h[5]);
            h11 = 0.75 * s + h[0];
            h12 = -0.4375 * s;
            h21 = s;
            h22 = h11;
          } else {
            k = b_i + 3 * (b_i - 1);
            h11 = h[k - 1];
            h21 = h[k];
            ix_tmp = b_i + 3 * b_i;
            h12 = h[ix_tmp - 1];
            h22 = h[ix_tmp];
          }
          s = ((fabs(h11) + fabs(h12)) + fabs(h21)) + fabs(h22);
          if (s == 0.0) {
            rt1r = 0.0;
            h11 = 0.0;
            rt2r = 0.0;
            h12 = 0.0;
          } else {
            h11 /= s;
            h21 /= s;
            h12 /= s;
            h22 /= s;
            rt2r = (h11 + h22) / 2.0;
            h11 = (h11 - rt2r) * (h22 - rt2r) - h12 * h21;
            h12 = sqrt(fabs(h11));
            if (h11 >= 0.0) {
              rt1r = rt2r * s;
              rt2r = rt1r;
              h11 = h12 * s;
              h12 = -h11;
            } else {
              rt1r = rt2r + h12;
              rt2r -= h12;
              if (fabs(rt1r - h22) <= fabs(rt2r - h22)) {
                rt1r *= s;
                rt2r = rt1r;
              } else {
                rt2r *= s;
                rt1r = rt2r;
              }
              h11 = 0.0;
              h12 = 0.0;
            }
          }
          if (b_i - 1 >= 1) {
            s = (fabs(h[0] - rt2r) + fabs(h12)) + fabs(h[1]);
            h21 = h[1] / s;
            v[0] = (h21 * h[3] + (h[0] - rt1r) * ((h[0] - rt2r) / s)) -
                   h11 * (h12 / s);
            v[1] = h21 * (((h[0] + h[4]) - rt1r) - rt2r);
            v[2] = h21 * h[5];
            s = (fabs(v[0]) + fabs(v[1])) + fabs(v[2]);
            v[0] /= s;
            v[1] /= s;
            v[2] /= s;
          }
          for (b_k = b_i - 1; b_k <= b_i; b_k++) {
            k = (b_i - b_k) + 2;
            if (k >= 3) {
              k = 3;
            }
            if (b_k > b_i - 1) {
              ix_tmp = ((b_k - 2) * 3 + b_k) - 1;
              temp_tmp_tmp = (unsigned char)k;
              for (i = 0; i < temp_tmp_tmp; i++) {
                v[i] = h[ix_tmp + i];
              }
            }
            h11 = v[0];
            s = b_xzlarfg(k, &h11, v);
            if (b_k > b_i - 1) {
              h[b_k - 1] = h11;
              h[b_k] = 0.0;
              if (b_k < b_i) {
                /* Check node always fails. would cause program termination and
                 * was eliminated */
              }
            }
            rt2r = v[1];
            rt1r = s * v[1];
            if (k == 3) {
              h12 = v[2];
              h21 = s * v[2];
              for (i = b_k; i < 4; i++) {
                k = 3 * (i - 1);
                ix_tmp = b_k + k;
                h11 = h[ix_tmp - 1];
                h22 = (h11 + rt2r * h[ix_tmp]) + h12 * h[k + 2];
                h[ix_tmp - 1] = h11 - h22 * s;
                h[ix_tmp] -= h22 * rt1r;
                h[k + 2] -= h22 * h21;
              }
              if (b_k + 3 <= b_i + 1) {
                k = b_k;
              } else {
                k = b_i - 2;
              }
              k = (unsigned char)(k + 3);
              for (i = 0; i < k; i++) {
                ix_tmp = i + 3 * (b_k - 1);
                h11 = h[ix_tmp];
                temp_tmp_tmp = i + 3 * b_k;
                h22 = (h11 + rt2r * h[temp_tmp_tmp]) + h12 * h[i + 6];
                h[ix_tmp] = h11 - h22 * s;
                h[temp_tmp_tmp] -= h22 * rt1r;
                h[i + 6] -= h22 * h21;
              }
              for (i = iloz; i <= ihiz; i++) {
                k = (i + 3 * (b_k - 1)) - 1;
                h11 = z[k];
                ix_tmp = (i + 3 * b_k) - 1;
                h22 = (h11 + rt2r * z[ix_tmp]) + h12 * z[i + 5];
                z[k] = h11 - h22 * s;
                z[ix_tmp] -= h22 * rt1r;
                z[i + 5] -= h22 * h21;
              }
            } else if (k == 2) {
              for (i = b_k; i < 4; i++) {
                k = b_k + 3 * (i - 1);
                h11 = h[k - 1];
                h12 = h[k];
                h22 = h11 + rt2r * h12;
                h11 -= h22 * s;
                h[k - 1] = h11;
                h12 -= h22 * rt1r;
                h[k] = h12;
              }
              k = (unsigned char)(b_i + 1);
              for (i = 0; i < k; i++) {
                ix_tmp = i + 3 * (b_k - 1);
                h11 = h[ix_tmp];
                temp_tmp_tmp = i + 3 * b_k;
                h12 = h[temp_tmp_tmp];
                h22 = h11 + rt2r * h12;
                h11 -= h22 * s;
                h[ix_tmp] = h11;
                h12 -= h22 * rt1r;
                h[temp_tmp_tmp] = h12;
              }
              for (i = iloz; i <= ihiz; i++) {
                k = (i + 3 * (b_k - 1)) - 1;
                h11 = z[k];
                ix_tmp = (i + 3 * b_k) - 1;
                h12 = z[ix_tmp];
                h22 = h11 + rt2r * h12;
                h11 -= h22 * s;
                z[k] = h11;
                h12 -= h22 * rt1r;
                z[ix_tmp] = h12;
              }
            }
          }
          ix++;
        }
      }
      if (!converged) {
        info = b_i + 1;
        exitg1 = true;
      } else {
        if (l == b_i + 1) {
          wr[b_i] = h[b_i + 3 * b_i];
          wi[b_i] = 0.0;
        } else if (l == b_i) {
          k = b_i + 3 * b_i;
          h11 = h[k - 1];
          ix = 3 * (b_i - 1);
          ix_tmp = b_i + ix;
          h12 = h[ix_tmp];
          h21 = h[k];
          wr[b_i - 1] = xdlanv2(&h[ix_tmp - 1], &h11, &h12, &h21, &wi[b_i - 1],
                                &rt2r, &rt1r, &h22, &s);
          wr[b_i] = rt2r;
          wi[b_i] = rt1r;
          h[k - 1] = h11;
          h[ix_tmp] = h12;
          h[k] = h21;
          if (b_i + 1 < 3) {
            ix_tmp = (b_i + 1) * 3 + b_i;
            k = (unsigned char)(2 - b_i);
            for (i = 0; i < k; i++) {
              temp_tmp_tmp = ix_tmp + i * 3;
              h11 = h[temp_tmp_tmp];
              h12 = h[temp_tmp_tmp - 1];
              h[temp_tmp_tmp] = h22 * h11 - s * h12;
              h[temp_tmp_tmp - 1] = h22 * h12 + s * h11;
            }
          }
          if (b_i - 1 >= 1) {
            k = b_i * 3;
            ix_tmp = (unsigned char)(b_i - 1);
            for (i = 0; i < ix_tmp; i++) {
              temp_tmp_tmp = k + i;
              h11 = h[temp_tmp_tmp];
              kdefl = ix + i;
              h12 = h[kdefl];
              h[temp_tmp_tmp] = h22 * h11 - s * h12;
              h[kdefl] = h22 * h12 + s * h11;
            }
          }
          if (nz >= 1) {
            ix = (ix + iloz) - 1;
            k = (b_i * 3 + iloz) - 1;
            ix_tmp = (unsigned char)nz;
            for (i = 0; i < ix_tmp; i++) {
              temp_tmp_tmp = k + i;
              h11 = z[temp_tmp_tmp];
              kdefl = ix + i;
              h12 = z[kdefl];
              z[temp_tmp_tmp] = h22 * h11 - s * h12;
              z[kdefl] = h22 * h12 + s * h11;
            }
          }
        }
        kdefl = 0;
        b_i = l - 2;
      }
    }
    h[2] = 0.0;
  }
  return info;
}

/*
 * File trailer for xdlahqr.c
 *
 * [EOF]
 */
