/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzlartg.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xzlartg.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double f
 *                double g
 *                double *sn
 *                double *r
 * Return Type  : double
 */
double xzlartg(double f, double g, double *sn, double *r)
{
  double cs;
  double g1;
  cs = fabs(f);
  g1 = fabs(g);
  if (g == 0.0) {
    cs = 1.0;
    *sn = 0.0;
    *r = f;
  } else if (f == 0.0) {
    cs = 0.0;
    if (g >= 0.0) {
      *sn = 1.0;
    } else {
      *sn = -1.0;
    }
    *r = g1;
  } else if ((cs > 1.4916681462400413E-154) && (cs < 4.740375954054589E+153) &&
             (g1 > 1.4916681462400413E-154) && (g1 < 4.740375954054589E+153)) {
    g1 = sqrt(f * f + g * g);
    cs /= g1;
    *r = g1;
    if (!(f >= 0.0)) {
      *r = -g1;
    }
    *sn = g / *r;
  } else {
    double gs;
    double u;
    u = fmin(4.49423283715579E+307,
             fmax(2.2250738585072014E-308, fmax(cs, g1)));
    cs = f / u;
    gs = g / u;
    g1 = sqrt(cs * cs + gs * gs);
    cs = fabs(cs) / g1;
    *r = g1;
    if (!(f >= 0.0)) {
      *r = -g1;
    }
    *sn = gs / *r;
    *r *= u;
  }
  return cs;
}

/*
 * File trailer for xzlartg.c
 *
 * [EOF]
 */
