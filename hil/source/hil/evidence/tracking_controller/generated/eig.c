/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: eig.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "eig.h"
#include "eigStandard.h"
#include "rt_nonfinite.h"
#include "xdlahqr.h"
#include "xzgehrd.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "xzlascl.h"
#include "xzsteqr.h"
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
void eig(const double A[9], creal_T V[9], creal_T D[9])
{
  double Q[9];
  double b_A[9];
  double e[2];
  double absx;
  int b_i;
  int i;
  int jj;
  boolean_T iscale;
  iscale = true;
  for (i = 0; i < 9; i++) {
    if (iscale) {
      absx = A[i];
      if (rtIsInf(absx) || rtIsNaN(absx)) {
        iscale = false;
      }
    } else {
      iscale = false;
    }
  }
  if (!iscale) {
    for (i = 0; i < 9; i++) {
      V[i].re = rtNaN;
      V[i].im = 0.0;
      D[i].re = 0.0;
      D[i].im = 0.0;
    }
    D[0].re = rtNaN;
    D[0].im = 0.0;
    D[4].re = rtNaN;
    D[4].im = 0.0;
    D[8].re = rtNaN;
    D[8].im = 0.0;
  } else {
    int exitg1;
    int iaii;
    int itau;
    boolean_T exitg2;
    iscale = true;
    itau = 0;
    exitg2 = false;
    while (!exitg2 && (itau < 3)) {
      iaii = 0;
      do {
        exitg1 = 0;
        if (iaii <= itau) {
          if (!(A[iaii + 3 * itau] == A[itau + 3 * iaii])) {
            iscale = false;
            exitg1 = 1;
          } else {
            iaii++;
          }
        } else {
          itau++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
    if (iscale) {
      double a__4[3];
      double anrm;
      memcpy(&b_A[0], &A[0], 9U * sizeof(double));
      anrm = 0.0;
      itau = 0;
      exitg2 = false;
      while (!exitg2 && (itau < 3)) {
        iaii = 0;
        do {
          exitg1 = 0;
          if (iaii <= itau) {
            absx = fabs(A[iaii + 3 * itau]);
            if (rtIsNaN(absx)) {
              anrm = rtNaN;
              exitg1 = 1;
            } else {
              if (absx > anrm) {
                anrm = absx;
              }
              iaii++;
            }
          } else {
            itau++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (rtIsInf(anrm) || rtIsNaN(anrm)) {
        a__4[0] = rtNaN;
        a__4[1] = rtNaN;
        a__4[2] = rtNaN;
        for (i = 0; i < 9; i++) {
          b_A[i] = rtNaN;
        }
      } else {
        double work[3];
        double tau[2];
        int temp1_tmp;
        iscale = false;
        if ((anrm > 0.0) && (anrm < 1.0010415475915505E-146)) {
          iscale = true;
          anrm = 1.0010415475915505E-146 / anrm;
          xzlascl(1.0, anrm, b_A);
        } else if (anrm > 9.989595361011175E+145) {
          iscale = true;
          anrm = 9.989595361011175E+145 / anrm;
          xzlascl(1.0, anrm, b_A);
        }
        for (b_i = 0; b_i < 2; b_i++) {
          double taui;
          int e_tmp_tmp;
          e_tmp_tmp = b_i + 3 * b_i;
          e[b_i] = b_A[e_tmp_tmp + 1];
          taui = xzlarfg(2 - b_i, &e[b_i], b_A, b_i * 3 + 3);
          if (taui != 0.0) {
            double temp1;
            double temp2;
            int sgn;
            b_A[e_tmp_tmp + 1] = 1.0;
            for (i = b_i + 1; i < 3; i++) {
              tau[i - 1] = 0.0;
            }
            itau = 1 - b_i;
            iaii = 3 - b_i;
            for (jj = 0; jj <= itau; jj++) {
              temp1_tmp = b_i + jj;
              temp1 = taui * b_A[(temp1_tmp + 3 * b_i) + 1];
              temp2 = 0.0;
              sgn = 3 * (temp1_tmp + 1);
              tau[temp1_tmp] += temp1 * b_A[(temp1_tmp + sgn) + 1];
              for (i = jj + 2; i < iaii; i++) {
                absx = b_A[sgn + 2];
                tau[1] += temp1 * absx;
                temp2 += absx * b_A[3 * b_i + 2];
              }
              tau[temp1_tmp] += taui * temp2;
            }
            itau = 1 - b_i;
            absx = 0.0;
            for (i = 0; i <= itau; i++) {
              absx += tau[b_i + i] * b_A[(e_tmp_tmp + i) + 1];
            }
            absx *= -0.5 * taui;
            if (!(absx == 0.0)) {
              itau = 2 - b_i;
              for (i = 0; i < itau; i++) {
                iaii = b_i + i;
                tau[iaii] += absx * b_A[(e_tmp_tmp + i) + 1];
              }
            }
            iaii = 1 - b_i;
            sgn = 3 - b_i;
            for (jj = 0; jj <= iaii; jj++) {
              int A_tmp_tmp;
              temp1_tmp = jj + 1;
              itau = b_i + jj;
              temp1 = b_A[(itau + 3 * b_i) + 1];
              temp2 = tau[itau];
              absx = temp2 * temp1;
              A_tmp_tmp = 3 * (itau + 1);
              itau = (itau + A_tmp_tmp) + 1;
              b_A[itau] = (b_A[itau] - absx) - absx;
              for (i = temp1_tmp + 1; i < sgn; i++) {
                b_A[A_tmp_tmp + 2] = (b_A[A_tmp_tmp + 2] - tau[1] * temp1) -
                                     b_A[3 * b_i + 2] * temp2;
              }
            }
          }
          b_A[e_tmp_tmp + 1] = e[b_i];
          a__4[b_i] = b_A[e_tmp_tmp];
          tau[b_i] = taui;
        }
        a__4[2] = b_A[8];
        for (i = 1; i >= 0; i--) {
          itau = 3 * (i + 1);
          b_A[itau] = 0.0;
          for (jj = i + 3; jj < 4; jj++) {
            b_A[itau + 2] = b_A[3 * i + 2];
          }
        }
        b_A[0] = 1.0;
        b_A[1] = 0.0;
        b_A[2] = 0.0;
        itau = 1;
        work[0] = 0.0;
        work[1] = 0.0;
        work[2] = 0.0;
        for (i = 1; i >= 0; i--) {
          iaii = (i + i * 3) + 4;
          if (i + 1 < 2) {
            b_A[iaii] = 1.0;
            xzlarf(2, 1, iaii + 1, tau[itau], b_A, iaii + 4, work);
            temp1_tmp = iaii + 2;
            for (jj = temp1_tmp; jj <= temp1_tmp; jj++) {
              b_A[jj - 1] *= -tau[itau];
            }
          }
          b_A[iaii] = 1.0 - tau[itau];
          if (i - 1 >= 0) {
            b_A[iaii - 1] = 0.0;
          }
          itau = i - 1;
        }
        itau = xzsteqr(a__4, e, b_A);
        if (itau != 0) {
          a__4[0] = rtNaN;
          a__4[1] = rtNaN;
          a__4[2] = rtNaN;
          for (i = 0; i < 9; i++) {
            b_A[i] = rtNaN;
          }
        } else if (iscale) {
          absx = 1.0 / anrm;
          a__4[0] *= absx;
          a__4[1] *= absx;
          a__4[2] *= absx;
        }
      }
      memset(&D[0], 0, 9U * sizeof(creal_T));
      D[0].re = a__4[0];
      D[0].im = 0.0;
      D[4].re = a__4[1];
      D[4].im = 0.0;
      D[8].re = a__4[2];
      D[8].im = 0.0;
      for (i = 0; i < 9; i++) {
        V[i].re = b_A[i];
        V[i].im = 0.0;
      }
    } else {
      iscale = true;
      itau = 0;
      exitg2 = false;
      while (!exitg2 && (itau < 3)) {
        iaii = 0;
        do {
          exitg1 = 0;
          if (iaii <= itau) {
            if (!(A[iaii + 3 * itau] == -A[itau + 3 * iaii])) {
              iscale = false;
              exitg1 = 1;
            } else {
              iaii++;
            }
          } else {
            itau++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (iscale) {
        double a__4[3];
        double work[3];
        double tau[2];
        memcpy(&b_A[0], &A[0], 9U * sizeof(double));
        xzgehrd(b_A, 1, 3, tau);
        memcpy(&Q[0], &b_A[0], 9U * sizeof(double));
        xzunghr(1, 3, Q, tau);
        iaii = xdlahqr(1, 3, b_A, 1, 3, Q, a__4, work);
        memset(&D[0], 0, 9U * sizeof(creal_T));
        for (i = 0; i < iaii; i++) {
          itau = i + 3 * i;
          D[itau].re = rtNaN;
          D[itau].im = 0.0;
        }
        for (i = iaii + 1; i < 4; i++) {
          itau = (i + 3 * (i - 1)) - 1;
          D[itau].re = 0.0;
          D[itau].im = work[i - 1];
        }
        if (iaii == 0) {
          for (i = 0; i < 9; i++) {
            V[i].re = Q[i];
            V[i].im = 0.0;
          }
          iaii = 1;
          do {
            exitg1 = 0;
            if (iaii <= 3) {
              if (iaii != 3) {
                int temp1_tmp;
                temp1_tmp = 3 * (iaii - 1);
                absx = b_A[iaii + temp1_tmp];
                if (absx != 0.0) {
                  double temp1;
                  int sgn;
                  if (absx < 0.0) {
                    sgn = 1;
                  } else {
                    sgn = -1;
                  }
                  absx = V[temp1_tmp].re;
                  temp1 = (double)sgn * V[3 * iaii].re;
                  if (temp1 == 0.0) {
                    V[temp1_tmp].re = absx / 1.4142135623730951;
                    V[temp1_tmp].im = 0.0;
                  } else if (absx == 0.0) {
                    V[temp1_tmp].re = 0.0;
                    V[temp1_tmp].im = temp1 / 1.4142135623730951;
                  } else {
                    V[temp1_tmp].re = absx / 1.4142135623730951;
                    V[temp1_tmp].im = temp1 / 1.4142135623730951;
                  }
                  V[3 * iaii].re = V[temp1_tmp].re;
                  V[3 * iaii].im = -V[temp1_tmp].im;
                  absx = V[temp1_tmp + 1].re;
                  itau = 3 * iaii + 1;
                  temp1 = (double)sgn * V[itau].re;
                  if (temp1 == 0.0) {
                    V[temp1_tmp + 1].re = absx / 1.4142135623730951;
                    V[temp1_tmp + 1].im = 0.0;
                  } else if (absx == 0.0) {
                    V[temp1_tmp + 1].re = 0.0;
                    V[temp1_tmp + 1].im = temp1 / 1.4142135623730951;
                  } else {
                    V[temp1_tmp + 1].re = absx / 1.4142135623730951;
                    V[temp1_tmp + 1].im = temp1 / 1.4142135623730951;
                  }
                  V[itau].re = V[temp1_tmp + 1].re;
                  V[itau].im = -V[temp1_tmp + 1].im;
                  absx = V[temp1_tmp + 2].re;
                  itau = 3 * iaii + 2;
                  temp1 = (double)sgn * V[itau].re;
                  if (temp1 == 0.0) {
                    V[temp1_tmp + 2].re = absx / 1.4142135623730951;
                    V[temp1_tmp + 2].im = 0.0;
                  } else if (absx == 0.0) {
                    V[temp1_tmp + 2].re = 0.0;
                    V[temp1_tmp + 2].im = temp1 / 1.4142135623730951;
                  } else {
                    V[temp1_tmp + 2].re = absx / 1.4142135623730951;
                    V[temp1_tmp + 2].im = temp1 / 1.4142135623730951;
                  }
                  V[itau].re = V[temp1_tmp + 2].re;
                  V[itau].im = -V[temp1_tmp + 2].im;
                  iaii += 2;
                } else {
                  iaii++;
                }
              } else {
                iaii++;
              }
            } else {
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        } else {
          for (i = 0; i < 9; i++) {
            V[i].re = rtNaN;
            V[i].im = 0.0;
          }
        }
      } else {
        eigStandard(A, V, D);
      }
    }
  }
}

/*
 * File trailer for eig.c
 *
 * [EOF]
 */
