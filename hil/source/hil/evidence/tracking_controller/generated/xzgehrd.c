/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzgehrd.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xzgehrd.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double a[9]
 *                int ilo
 *                int ihi
 *                double tau[2]
 * Return Type  : void
 */
void xzgehrd(double a[9], int ilo, int ihi, double tau[2])
{
  double work[3];
  double alpha1;
  int b_i;
  int b_ia;
  int i;
  if ((ihi - ilo) + 1 > 1) {
    int im1n;
    im1n = (unsigned char)(ilo - 1);
    if (im1n - 1 >= 0) {
      memset(&tau[0], 0, (unsigned int)im1n * sizeof(double));
    }
    for (i = ihi; i < 3; i++) {
      tau[i - 1] = 0.0;
    }
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    for (b_i = ilo; b_i < ihi; b_i++) {
      double temp;
      int alpha1_tmp;
      int ia;
      int in;
      int lastc;
      int lastv;
      int n;
      im1n = (b_i - 1) * 3;
      in = b_i * 3;
      n = ihi - b_i;
      alpha1_tmp = b_i + im1n;
      alpha1 = a[alpha1_tmp];
      temp = xzlarfg(n, &alpha1, a, im1n + 3);
      tau[b_i - 1] = temp;
      a[alpha1_tmp] = 1.0;
      if (temp != 0.0) {
        boolean_T exitg2;
        lastv = n;
        im1n = alpha1_tmp + n;
        while ((lastv > 0) && (a[im1n - 1] == 0.0)) {
          lastv--;
          im1n--;
        }
        lastc = ihi;
        exitg2 = false;
        while (!exitg2 && (lastc > 0)) {
          int exitg1;
          im1n = in + lastc;
          ia = im1n;
          do {
            exitg1 = 0;
            if (ia <= im1n + (lastv - 1) * 3) {
              if (a[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia += 3;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        double d;
        int jA;
        if (lastc != 0) {
          memset(&work[0], 0, (unsigned int)lastc * sizeof(double));
          im1n = (in + 3 * (lastv - 1)) + 1;
          for (i = in + 1; i <= im1n; i += 3) {
            ia = i + lastc;
            for (b_ia = i; b_ia < ia; b_ia++) {
              jA = b_ia - i;
              work[jA] += a[b_ia - 1] * a[alpha1_tmp + ((i - in) - 1) / 3];
            }
          }
        }
        d = -tau[b_i - 1];
        if (!(d == 0.0)) {
          jA = in;
          im1n = (unsigned char)lastv;
          for (i = 0; i < im1n; i++) {
            temp = a[alpha1_tmp + i];
            if (temp != 0.0) {
              temp *= d;
              ia = lastc + jA;
              for (b_ia = jA + 1; b_ia <= ia; b_ia++) {
                a[b_ia - 1] += work[(b_ia - jA) - 1] * temp;
              }
            }
            jA += 3;
          }
        }
      }
      xzlarf(n, 3 - b_i, alpha1_tmp + 1, tau[b_i - 1], a, (b_i + in) + 1, work);
      a[alpha1_tmp] = alpha1;
    }
  }
}

/*
 * File trailer for xzgehrd.c
 *
 * [EOF]
 */
