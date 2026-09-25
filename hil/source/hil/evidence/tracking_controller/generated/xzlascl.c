/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzlascl.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xzlascl.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double cfrom
 *                double cto
 *                int m
 *                double A[3]
 *                int iA0
 * Return Type  : void
 */
void b_xzlascl(double cfrom, double cto, int m, double A[3], int iA0)
{
  double cfromc;
  double ctoc;
  int i;
  boolean_T notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    double cfrom1;
    double cto1;
    double mul;
    cfrom1 = cfromc * 2.004168360008973E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > fabs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.004168360008973E-292;
      cfromc = cfrom1;
    } else if (fabs(cto1) > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }
    for (i = 0; i < m; i++) {
      int b_i;
      b_i = (iA0 + i) - 1;
      A[b_i] *= mul;
    }
  }
}

/*
 * Arguments    : double cfrom
 *                double cto
 *                int m
 *                double A[2]
 *                int iA0
 * Return Type  : void
 */
void c_xzlascl(double cfrom, double cto, int m, double A[2], int iA0)
{
  double cfromc;
  double ctoc;
  int i;
  boolean_T notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    double cfrom1;
    double cto1;
    double mul;
    cfrom1 = cfromc * 2.004168360008973E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > fabs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.004168360008973E-292;
      cfromc = cfrom1;
    } else if (fabs(cto1) > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }
    for (i = 0; i < m; i++) {
      int b_i;
      b_i = (iA0 + i) - 1;
      A[b_i] *= mul;
    }
  }
}

/*
 * Arguments    : double cfrom
 *                double cto
 *                double A[9]
 * Return Type  : void
 */
void xzlascl(double cfrom, double cto, double A[9])
{
  double cfromc;
  double ctoc;
  int j;
  boolean_T notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    double cfrom1;
    double cto1;
    double mul;
    cfrom1 = cfromc * 2.004168360008973E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((fabs(cfrom1) > ctoc) && (ctoc != 0.0)) {
      mul = 2.004168360008973E-292;
      cfromc = cfrom1;
    } else if (cto1 > fabs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }
    for (j = 0; j < 3; j++) {
      int offset;
      offset = j * 3 - 1;
      A[offset + 1] *= mul;
      A[offset + 2] *= mul;
      A[offset + 3] *= mul;
    }
  }
}

/*
 * File trailer for xzlascl.c
 *
 * [EOF]
 */
