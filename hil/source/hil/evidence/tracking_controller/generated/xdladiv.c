/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xdladiv.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xdladiv.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double a
 *                double b
 *                double c
 *                double d
 *                double *q
 * Return Type  : double
 */
double xdladiv(double a, double b, double c, double d, double *q)
{
  double aa;
  double ab;
  double bb;
  double cc;
  double cd;
  double dd;
  double p;
  double r;
  double s;
  double t;
  aa = a;
  bb = b;
  cc = c;
  dd = d;
  ab = fmax(fabs(a), fabs(b));
  r = fabs(d);
  t = fabs(c);
  cd = fmax(t, r);
  s = 1.0;
  if (ab >= 8.988465674311579E+307) {
    aa = 0.5 * a;
    bb = 0.5 * b;
    s = 2.0;
  }
  if (cd >= 8.988465674311579E+307) {
    cc = 0.5 * c;
    dd = 0.5 * d;
    s *= 0.5;
  }
  if (ab <= 2.004168360008973E-292) {
    aa *= 4.056481920730334E+31;
    bb *= 4.056481920730334E+31;
    s /= 4.056481920730334E+31;
  }
  if (cd <= 2.004168360008973E-292) {
    cc *= 4.056481920730334E+31;
    dd *= 4.056481920730334E+31;
    s *= 4.056481920730334E+31;
  }
  if (r <= t) {
    r = dd / cc;
    t = 1.0 / (cc + dd * r);
    if (r != 0.0) {
      ab = bb * r;
      if (ab != 0.0) {
        p = (aa + ab) * t;
      } else {
        p = aa * t + bb * t * r;
      }
      ab = -aa * r;
      if (ab != 0.0) {
        *q = (bb + ab) * t;
      } else {
        *q = bb * t + -aa * t * r;
      }
    } else {
      p = (aa + dd * (bb / cc)) * t;
      *q = (bb + dd * (-aa / cc)) * t;
    }
  } else {
    cd = cc / dd;
    r = 1.0 / (dd + cc * cd);
    if (cd != 0.0) {
      ab = aa * cd;
      if (ab != 0.0) {
        p = (bb + ab) * r;
      } else {
        p = bb * r + aa * r * cd;
      }
      ab = -bb * cd;
      if (ab != 0.0) {
        ab = (aa + ab) * r;
      } else {
        ab = aa * r + -bb * r * cd;
      }
    } else {
      p = (bb + cc * (aa / dd)) * r;
      ab = (aa + cc * (-bb / dd)) * r;
    }
    *q = -ab;
  }
  p *= s;
  *q *= s;
  return p;
}

/*
 * File trailer for xdladiv.c
 *
 * [EOF]
 */
