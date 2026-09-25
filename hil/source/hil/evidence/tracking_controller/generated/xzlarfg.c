/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzlarfg.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xzlarfg.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : int n
 *                double *alpha1
 *                double x[3]
 * Return Type  : double
 */
double b_xzlarfg(int n, double *alpha1, double x[3])
{
  double tau;
  int k;
  tau = 0.0;
  if (n > 0) {
    double xnorm;
    xnorm = b_xnrm2(n - 1, x);
    if (xnorm != 0.0) {
      double beta1;
      beta1 = fabs(*alpha1);
      if (beta1 < xnorm) {
        beta1 /= xnorm;
        beta1 = xnorm * sqrt(beta1 * beta1 + 1.0);
      } else if (beta1 > xnorm) {
        xnorm /= beta1;
        beta1 *= sqrt(xnorm * xnorm + 1.0);
      } else if (rtIsNaN(xnorm)) {
        beta1 = rtNaN;
      } else {
        beta1 *= 1.4142135623730951;
      }
      if (*alpha1 >= 0.0) {
        beta1 = -beta1;
      }
      if (fabs(beta1) < 1.0020841800044864E-292) {
        int knt;
        knt = 0;
        do {
          knt++;
          for (k = 2; k <= n; k++) {
            x[k - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));
        xnorm = b_xnrm2(n - 1, x);
        beta1 = fabs(*alpha1);
        if (beta1 < xnorm) {
          beta1 /= xnorm;
          beta1 = xnorm * sqrt(beta1 * beta1 + 1.0);
        } else if (beta1 > xnorm) {
          xnorm /= beta1;
          beta1 *= sqrt(xnorm * xnorm + 1.0);
        } else if (rtIsNaN(xnorm)) {
          beta1 = rtNaN;
        } else {
          beta1 *= 1.4142135623730951;
        }
        if (*alpha1 >= 0.0) {
          beta1 = -beta1;
        }
        tau = (beta1 - *alpha1) / beta1;
        xnorm = 1.0 / (*alpha1 - beta1);
        for (k = 2; k <= n; k++) {
          x[k - 1] *= xnorm;
        }
        for (k = 0; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }
        *alpha1 = beta1;
      } else {
        tau = (beta1 - *alpha1) / beta1;
        xnorm = 1.0 / (*alpha1 - beta1);
        for (k = 2; k <= n; k++) {
          x[k - 1] *= xnorm;
        }
        *alpha1 = beta1;
      }
    }
  }
  return tau;
}

/*
 * Arguments    : int n
 *                double *alpha1
 *                double x[9]
 *                int ix0
 * Return Type  : double
 */
double xzlarfg(int n, double *alpha1, double x[9], int ix0)
{
  double tau;
  int k;
  tau = 0.0;
  if (n > 0) {
    double xnorm;
    xnorm = xnrm2(n - 1, x, ix0);
    if (xnorm != 0.0) {
      double beta1;
      beta1 = fabs(*alpha1);
      if (beta1 < xnorm) {
        beta1 /= xnorm;
        beta1 = xnorm * sqrt(beta1 * beta1 + 1.0);
      } else if (beta1 > xnorm) {
        xnorm /= beta1;
        beta1 *= sqrt(xnorm * xnorm + 1.0);
      } else if (rtIsNaN(xnorm)) {
        beta1 = rtNaN;
      } else {
        beta1 *= 1.4142135623730951;
      }
      if (*alpha1 >= 0.0) {
        beta1 = -beta1;
      }
      if (fabs(beta1) < 1.0020841800044864E-292) {
        int i;
        int knt;
        knt = 0;
        i = (ix0 + n) - 2;
        do {
          knt++;
          for (k = ix0; k <= i; k++) {
            x[k - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));
        xnorm = xnrm2(n - 1, x, ix0);
        beta1 = fabs(*alpha1);
        if (beta1 < xnorm) {
          beta1 /= xnorm;
          beta1 = xnorm * sqrt(beta1 * beta1 + 1.0);
        } else if (beta1 > xnorm) {
          xnorm /= beta1;
          beta1 *= sqrt(xnorm * xnorm + 1.0);
        } else if (rtIsNaN(xnorm)) {
          beta1 = rtNaN;
        } else {
          beta1 *= 1.4142135623730951;
        }
        if (*alpha1 >= 0.0) {
          beta1 = -beta1;
        }
        tau = (beta1 - *alpha1) / beta1;
        xnorm = 1.0 / (*alpha1 - beta1);
        for (k = ix0; k <= i; k++) {
          x[k - 1] *= xnorm;
        }
        for (k = 0; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }
        *alpha1 = beta1;
      } else {
        int i;
        tau = (beta1 - *alpha1) / beta1;
        xnorm = 1.0 / (*alpha1 - beta1);
        i = (ix0 + n) - 2;
        for (k = ix0; k <= i; k++) {
          x[k - 1] *= xnorm;
        }
        *alpha1 = beta1;
      }
    }
  }
  return tau;
}

/*
 * File trailer for xzlarfg.c
 *
 * [EOF]
 */
