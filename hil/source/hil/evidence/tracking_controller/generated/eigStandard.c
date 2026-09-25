/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: eigStandard.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "eigStandard.h"
#include "rt_nonfinite.h"
#include "xdlahqr.h"
#include "xdtrevc3.h"
#include "xnrm2.h"
#include "xzgebal.h"
#include "xzgehrd.h"
#include "xzlascl.h"
#include "xzunghr.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double A[9]
 *                creal_T V[9]
 *                creal_T D[9]
 * Return Type  : void
 */
void eigStandard(const double A[9], creal_T V[9], creal_T D[9])
{
  creal_T W[3];
  double b_A[9];
  double vr[9];
  double absxk;
  double anrm;
  int b_k;
  int i;
  int k;
  boolean_T exitg1;
  memcpy(&b_A[0], &A[0], 9U * sizeof(double));
  anrm = 0.0;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k < 9)) {
    absxk = fabs(A[k]);
    if (rtIsNaN(absxk)) {
      anrm = rtNaN;
      exitg1 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }
      k++;
    }
  }
  if (rtIsInf(anrm) || rtIsNaN(anrm)) {
    W[0].re = rtNaN;
    W[0].im = 0.0;
    W[1].re = rtNaN;
    W[1].im = 0.0;
    W[2].re = rtNaN;
    W[2].im = 0.0;
    for (i = 0; i < 9; i++) {
      V[i].re = rtNaN;
      V[i].im = 0.0;
    }
  } else {
    double scale[3];
    double wi[3];
    double wr[3];
    double tau[2];
    double cscale;
    int ilo;
    int info;
    boolean_T scalea;
    cscale = anrm;
    scalea = false;
    if ((anrm > 0.0) && (anrm < 6.717876107567089E-139)) {
      scalea = true;
      cscale = 6.717876107567089E-139;
      xzlascl(anrm, cscale, b_A);
    } else if (anrm > 1.488565707357403E+138) {
      scalea = true;
      cscale = 1.488565707357403E+138;
      xzlascl(anrm, cscale, b_A);
    }
    ilo = xzgebal(b_A, &k, scale);
    xzgehrd(b_A, ilo, k, tau);
    memcpy(&vr[0], &b_A[0], 9U * sizeof(double));
    xzunghr(ilo, k, vr, tau);
    info = xdlahqr(ilo, k, b_A, ilo, k, vr, wr, wi);
    if (info == 0) {
      double temp;
      int ix;
      xdtrevc3(b_A, vr);
      if (ilo != k) {
        for (i = ilo; i <= k; i++) {
          for (b_k = i; b_k <= i + 6; b_k += 3) {
            vr[b_k - 1] *= scale[i - 1];
          }
        }
      }
      for (i = ilo - 1; i >= 1; i--) {
        absxk = scale[i - 1];
        if ((int)absxk != i) {
          temp = vr[i - 1];
          vr[i - 1] = vr[(int)absxk - 1];
          vr[(int)absxk - 1] = temp;
          temp = vr[i + 2];
          vr[i + 2] = vr[(int)absxk + 2];
          vr[(int)absxk + 2] = temp;
          temp = vr[i + 5];
          vr[i + 5] = vr[(int)absxk + 5];
          vr[(int)absxk + 5] = temp;
        }
      }
      for (i = k + 1; i < 4; i++) {
        absxk = scale[i - 1];
        if ((int)absxk != i) {
          temp = vr[i - 1];
          vr[i - 1] = vr[(int)absxk - 1];
          vr[(int)absxk - 1] = temp;
          temp = vr[i + 2];
          vr[i + 2] = vr[(int)absxk + 2];
          vr[(int)absxk + 2] = temp;
          temp = vr[i + 5];
          vr[i + 5] = vr[(int)absxk + 5];
          vr[(int)absxk + 5] = temp;
        }
      }
      for (b_k = 0; b_k < 3; b_k++) {
        absxk = wi[b_k];
        if (!(absxk < 0.0)) {
          if ((b_k + 1 != 3) && (absxk > 0.0)) {
            double c;
            double f1_tmp;
            double g1_tmp;
            double smax;
            int b_tmp_tmp;
            k = b_k * 3 + 1;
            absxk = xnrm2(3, vr, k);
            b_tmp_tmp = (b_k + 1) * 3;
            ix = b_tmp_tmp + 1;
            temp = xnrm2(3, vr, b_tmp_tmp + 1);
            if (absxk < temp) {
              absxk /= temp;
              absxk = temp * sqrt(absxk * absxk + 1.0);
            } else if (absxk > temp) {
              temp /= absxk;
              absxk *= sqrt(temp * temp + 1.0);
            } else if (rtIsNaN(temp)) {
              absxk = rtNaN;
            } else {
              absxk *= 1.4142135623730951;
            }
            absxk = 1.0 / absxk;
            for (i = k; i <= k + 2; i++) {
              vr[i - 1] *= absxk;
            }
            for (i = ix; i <= ix + 2; i++) {
              vr[i - 1] *= absxk;
            }
            absxk = vr[3 * b_k];
            temp = vr[b_tmp_tmp];
            scale[0] = absxk * absxk + temp * temp;
            absxk = vr[k];
            temp = vr[b_tmp_tmp + 1];
            scale[1] = absxk * absxk + temp * temp;
            absxk = vr[3 * b_k + 2];
            temp = vr[b_tmp_tmp + 2];
            k = 0;
            smax = scale[0];
            if (scale[1] > scale[0]) {
              k = 1;
              smax = scale[1];
            }
            if (absxk * absxk + temp * temp > smax) {
              k = 2;
            }
            f1_tmp = vr[k + 3 * b_k];
            temp = fabs(f1_tmp);
            k += b_tmp_tmp;
            g1_tmp = vr[k];
            absxk = fabs(g1_tmp);
            if (g1_tmp == 0.0) {
              c = 1.0;
              smax = 0.0;
            } else if (f1_tmp == 0.0) {
              c = 0.0;
              if (g1_tmp >= 0.0) {
                smax = 1.0;
              } else {
                smax = -1.0;
              }
            } else if ((temp > 1.4916681462400413E-154) &&
                       (temp < 4.740375954054589E+153) &&
                       (absxk > 1.4916681462400413E-154) &&
                       (absxk < 4.740375954054589E+153)) {
              smax = sqrt(f1_tmp * f1_tmp + g1_tmp * g1_tmp);
              c = temp / smax;
              if (!(f1_tmp >= 0.0)) {
                smax = -smax;
              }
              smax = g1_tmp / smax;
            } else {
              absxk = fmin(4.49423283715579E+307,
                           fmax(2.2250738585072014E-308, fmax(temp, absxk)));
              temp = f1_tmp / absxk;
              absxk = g1_tmp / absxk;
              smax = sqrt(temp * temp + absxk * absxk);
              c = fabs(temp) / smax;
              if (!(f1_tmp >= 0.0)) {
                smax = -smax;
              }
              smax = absxk / smax;
            }
            ix = b_k * 3;
            absxk = c * vr[ix] + smax * vr[b_tmp_tmp];
            vr[b_tmp_tmp] = c * vr[b_tmp_tmp] - smax * vr[ix];
            vr[ix] = absxk;
            absxk = vr[b_tmp_tmp + 1];
            temp = vr[ix + 1];
            vr[b_tmp_tmp + 1] = c * absxk - smax * temp;
            vr[ix + 1] = c * temp + smax * absxk;
            absxk = vr[b_tmp_tmp + 2];
            temp = vr[ix + 2];
            vr[b_tmp_tmp + 2] = c * absxk - smax * temp;
            vr[ix + 2] = c * temp + smax * absxk;
            vr[k] = 0.0;
          } else {
            k = b_k * 3 + 1;
            absxk = 1.0 / xnrm2(3, vr, k);
            for (i = k; i <= k + 2; i++) {
              vr[i - 1] *= absxk;
            }
          }
        }
      }
      for (i = 0; i < 9; i++) {
        V[i].re = vr[i];
        V[i].im = 0.0;
      }
      for (i = 0; i < 2; i++) {
        if ((wi[i] > 0.0) && (wi[i + 1] < 0.0)) {
          ix = 3 * (i + 1);
          absxk = V[ix].re;
          V[3 * i].im = absxk;
          V[ix].re = V[3 * i].re;
          V[ix].im = -absxk;
          k = 3 * i + 1;
          absxk = V[ix + 1].re;
          V[k].im = absxk;
          V[ix + 1].re = V[k].re;
          V[ix + 1].im = -absxk;
          k = 3 * i + 2;
          absxk = V[ix + 2].re;
          V[k].im = absxk;
          V[ix + 2].re = V[k].re;
          V[ix + 2].im = -absxk;
        }
      }
    } else {
      for (i = 0; i < 9; i++) {
        V[i].re = rtNaN;
        V[i].im = 0.0;
      }
    }
    if (scalea) {
      b_xzlascl(cscale, anrm, 3 - info, wr, info + 1);
      b_xzlascl(cscale, anrm, 3 - info, wi, info + 1);
      if (info != 0) {
        b_xzlascl(cscale, anrm, ilo - 1, wr, 1);
        b_xzlascl(cscale, anrm, ilo - 1, wi, 1);
      }
    }
    if (info != 0) {
      for (i = ilo; i <= info; i++) {
        wr[i - 1] = rtNaN;
        wi[i - 1] = 0.0;
      }
    }
    W[0].re = wr[0];
    W[0].im = wi[0];
    W[1].re = wr[1];
    W[1].im = wi[1];
    W[2].re = wr[2];
    W[2].im = wi[2];
  }
  memset(&D[0], 0, 9U * sizeof(creal_T));
  D[0] = W[0];
  D[4] = W[1];
  D[8] = W[2];
}

/*
 * File trailer for eigStandard.c
 *
 * [EOF]
 */
