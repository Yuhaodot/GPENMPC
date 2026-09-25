/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xdlaev2.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xdlaev2.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double a
 *                double b
 *                double c
 *                double *rt2
 *                double *cs1
 *                double *sn1
 * Return Type  : double
 */
double xdlaev2(double a, double b, double c, double *rt2, double *cs1,
               double *sn1)
{
  double ab;
  double acmn;
  double acmx;
  double adf;
  double df;
  double rt1;
  double sm;
  double tb;
  int sgn1;
  int sgn2;
  sm = a + c;
  df = a - c;
  adf = fabs(df);
  tb = b + b;
  ab = fabs(tb);
  if (fabs(a) > fabs(c)) {
    acmx = a;
    acmn = c;
  } else {
    acmx = c;
    acmn = a;
  }
  if (adf > ab) {
    rt1 = ab / adf;
    adf *= sqrt(rt1 * rt1 + 1.0);
  } else if (adf < ab) {
    adf /= ab;
    adf = ab * sqrt(adf * adf + 1.0);
  } else {
    adf = ab * 1.4142135623730951;
  }
  if (sm < 0.0) {
    rt1 = 0.5 * (sm - adf);
    sgn1 = -1;
    *rt2 = acmx / rt1 * acmn - b / rt1 * b;
  } else if (sm > 0.0) {
    rt1 = 0.5 * (sm + adf);
    sgn1 = 1;
    *rt2 = acmx / rt1 * acmn - b / rt1 * b;
  } else {
    rt1 = 0.5 * adf;
    *rt2 = -0.5 * adf;
    sgn1 = 1;
  }
  if (df >= 0.0) {
    adf += df;
    sgn2 = 1;
  } else {
    adf = df - adf;
    sgn2 = -1;
  }
  if (fabs(adf) > ab) {
    adf = -tb / adf;
    *sn1 = 1.0 / sqrt(adf * adf + 1.0);
    *cs1 = adf * *sn1;
  } else if (ab == 0.0) {
    *cs1 = 1.0;
    *sn1 = 0.0;
  } else {
    adf = -adf / tb;
    *cs1 = 1.0 / sqrt(adf * adf + 1.0);
    *sn1 = adf * *cs1;
  }
  if (sgn1 == sgn2) {
    adf = *cs1;
    *cs1 = -*sn1;
    *sn1 = adf;
  }
  return rt1;
}

/*
 * File trailer for xdlaev2.c
 *
 * [EOF]
 */
