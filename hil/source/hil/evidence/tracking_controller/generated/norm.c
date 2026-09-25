/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: norm.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "norm.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double x[2]
 * Return Type  : double
 */
double b_norm(const double x[2])
{
  double absxk;
  double scale;
  double t;
  double y;
  boolean_T b;
  scale = 3.312168642111238E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.312168642111238E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.312168642111238E-170;
    y = t * t;
  }
  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * sqrt(y);
  b = rtIsNaN(y);
  if (b) {
    int k;
    k = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (k < 2) {
        if (rtIsNaN(x[k])) {
          exitg1 = 1;
        } else {
          k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

/*
 * Arguments    : const double x[3]
 * Return Type  : double
 */
double c_norm(const double x[3])
{
  double absxk;
  double scale;
  double t;
  double y;
  boolean_T b;
  scale = 3.312168642111238E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.312168642111238E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.312168642111238E-170;
    y = t * t;
  }
  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * sqrt(y);
  b = rtIsNaN(y);
  if (b) {
    int k;
    k = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (k < 3) {
        if (rtIsNaN(x[k])) {
          exitg1 = 1;
        } else {
          k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

/*
 * Arguments    : const double x[4]
 * Return Type  : double
 */
double d_norm(const double x[4])
{
  double absxk;
  double scale;
  double t;
  double y;
  boolean_T b;
  scale = 3.312168642111238E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.312168642111238E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.312168642111238E-170;
    y = t * t;
  }
  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = fabs(x[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * sqrt(y);
  b = rtIsNaN(y);
  if (b) {
    int k;
    k = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (k < 4) {
        if (rtIsNaN(x[k])) {
          exitg1 = 1;
        } else {
          k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

/*
 * Arguments    : const double x[9]
 * Return Type  : double
 */
double e_norm(const double x[9])
{
  double scale;
  double y;
  int k;
  boolean_T b;
  y = 0.0;
  scale = 3.312168642111238E-170;
  for (k = 0; k < 9; k++) {
    double absxk;
    absxk = fabs(x[k]);
    if (absxk > scale) {
      double t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      double t;
      t = absxk / scale;
      y += t * t;
    }
  }
  y = scale * sqrt(y);
  b = rtIsNaN(y);
  if (b) {
    int b_k;
    b_k = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (b_k < 9) {
        if (rtIsNaN(x[b_k])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return y;
}

/*
 * File trailer for norm.c
 *
 * [EOF]
 */
