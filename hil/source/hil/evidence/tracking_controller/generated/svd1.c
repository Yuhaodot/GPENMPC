/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: svd1.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "svd1.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xrot.h"
#include "xrotg.h"
#include "xswap.h"
#include "xzlangeM.h"
#include "xzlascl.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double A[9]
 *                double U[9]
 *                double s[3]
 *                double V[9]
 * Return Type  : void
 */
void b_svd(const double A[9], double U[9], double s[3], double V[9])
{
  double b_A[9];
  double e[3];
  double work[3];
  double anrm;
  double b;
  double cscale;
  double f;
  double nrm;
  double rt;
  double sm;
  double snorm;
  double sqds;
  int k;
  int m;
  int q;
  int qjj;
  int qp1;
  int qq;
  int qs;
  boolean_T doscale;
  s[0] = 0.0;
  e[0] = 0.0;
  work[0] = 0.0;
  s[1] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  s[2] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  for (k = 0; k < 9; k++) {
    b_A[k] = A[k];
    U[k] = 0.0;
    V[k] = 0.0;
  }
  doscale = false;
  anrm = xzlangeM(A);
  cscale = anrm;
  if ((anrm > 0.0) && (anrm < 6.717876107567089E-139)) {
    doscale = true;
    cscale = 6.717876107567089E-139;
    xzlascl(anrm, cscale, b_A);
  } else if (anrm > 1.488565707357403E+138) {
    doscale = true;
    cscale = 1.488565707357403E+138;
    xzlascl(anrm, cscale, b_A);
  }
  for (q = 0; q < 2; q++) {
    boolean_T apply_transform;
    qp1 = q + 2;
    qs = q + 3 * q;
    qq = qs + 1;
    apply_transform = false;
    nrm = xnrm2(3 - q, b_A, qs + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qs] < 0.0) {
        nrm = -nrm;
      }
      s[q] = nrm;
      if (fabs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        qjj = (qs - q) + 3;
        for (k = qq; k <= qjj; k++) {
          b_A[k - 1] *= nrm;
        }
      } else {
        qjj = (qs - q) + 3;
        for (k = qq; k <= qjj; k++) {
          b_A[k - 1] /= s[q];
        }
      }
      b_A[qs]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }
    for (k = qp1; k < 4; k++) {
      qjj = q + 3 * (k - 1);
      if (apply_transform) {
        xaxpy(3 - q, -(xdotc(3 - q, b_A, qs + 1, b_A, qjj + 1) / b_A[qs]),
              qs + 1, b_A, qjj + 1);
      }
      e[k - 1] = b_A[qjj];
    }
    for (k = q + 1; k < 4; k++) {
      qjj = (k + 3 * q) - 1;
      U[qjj] = b_A[qjj];
    }
    if (q <= 0) {
      nrm = c_xnrm2(e);
      if (nrm == 0.0) {
        e[0] = 0.0;
      } else {
        if (e[1] < 0.0) {
          e[0] = -nrm;
        } else {
          e[0] = nrm;
        }
        nrm = e[0];
        if (fabs(e[0]) >= 1.0020841800044864E-292) {
          nrm = 1.0 / e[0];
          for (k = qp1; k < 4; k++) {
            e[k - 1] *= nrm;
          }
        } else {
          for (k = qp1; k < 4; k++) {
            e[k - 1] /= nrm;
          }
        }
        e[1]++;
        e[0] = -e[0];
        for (k = qp1; k < 4; k++) {
          work[k - 1] = 0.0;
        }
        for (k = qp1; k < 4; k++) {
          b_xaxpy(e[k - 1], b_A, 3 * (k - 1) + 2, work);
        }
        for (k = qp1; k < 4; k++) {
          c_xaxpy(-e[k - 1] / e[1], work, b_A, 3 * (k - 1) + 2);
        }
      }
      for (k = qp1; k < 4; k++) {
        V[k - 1] = e[k - 1];
      }
    }
  }
  m = 2;
  s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (q = 1; q >= 0; q--) {
    qq = q + 3 * q;
    if (s[q] != 0.0) {
      for (k = q + 2; k < 4; k++) {
        qjj = (q + 3 * (k - 1)) + 1;
        xaxpy(3 - q, -(xdotc(3 - q, U, qq + 1, U, qjj) / U[qq]), qq + 1, U,
              qjj);
      }
      for (k = q + 1; k < 4; k++) {
        qjj = (k + 3 * q) - 1;
        U[qjj] = -U[qjj];
      }
      U[qq]++;
      if (q - 1 >= 0) {
        U[3 * q] = 0.0;
      }
    } else {
      U[3 * q] = 0.0;
      U[3 * q + 1] = 0.0;
      U[3 * q + 2] = 0.0;
      U[qq] = 1.0;
    }
  }
  for (k = 2; k >= 0; k--) {
    if ((k <= 0) && (e[0] != 0.0)) {
      xaxpy(2, -(xdotc(2, V, 2, V, 5) / V[1]), 2, V, 5);
      xaxpy(2, -(xdotc(2, V, 2, V, 8) / V[1]), 2, V, 8);
    }
    V[3 * k] = 0.0;
    V[3 * k + 1] = 0.0;
    V[3 * k + 2] = 0.0;
    V[k + 3 * k] = 1.0;
  }
  qp1 = 0;
  snorm = 0.0;
  for (q = 0; q < 3; q++) {
    nrm = s[q];
    if (nrm != 0.0) {
      rt = fabs(nrm);
      nrm /= rt;
      s[q] = rt;
      if (q + 1 < 3) {
        e[q] /= nrm;
      }
      qjj = 3 * q + 1;
      for (k = qjj; k <= qjj + 2; k++) {
        U[k - 1] *= nrm;
      }
    }
    if (q + 1 < 3) {
      nrm = e[q];
      if (nrm != 0.0) {
        rt = fabs(nrm);
        nrm = rt / nrm;
        e[q] = rt;
        s[q + 1] *= nrm;
        qjj = 3 * (q + 1) + 1;
        for (k = qjj; k <= qjj + 2; k++) {
          V[k - 1] *= nrm;
        }
      }
    }
    snorm = fmax(snorm, fmax(fabs(s[q]), fabs(e[q])));
  }
  while ((m + 1 > 0) && (qp1 < 75)) {
    boolean_T exitg1;
    qq = m;
    exitg1 = false;
    while (!(exitg1 || (qq == 0))) {
      nrm = fabs(e[qq - 1]);
      if ((nrm <= 2.220446049250313E-16 * (fabs(s[qq - 1]) + fabs(s[qq]))) ||
          (nrm <= 1.0020841800044864E-292) ||
          ((qp1 > 20) && (nrm <= 2.220446049250313E-16 * snorm))) {
        e[qq - 1] = 0.0;
        exitg1 = true;
      } else {
        qq--;
      }
    }
    if (qq == m) {
      qjj = 4;
    } else {
      qs = m + 1;
      qjj = m + 1;
      exitg1 = false;
      while (!exitg1 && (qjj >= qq)) {
        qs = qjj;
        if (qjj == qq) {
          exitg1 = true;
        } else {
          nrm = 0.0;
          if (qjj < m + 1) {
            nrm = fabs(e[qjj - 1]);
          }
          if (qjj > qq + 1) {
            nrm += fabs(e[qjj - 2]);
          }
          rt = fabs(s[qjj - 1]);
          if ((rt <= 2.220446049250313E-16 * nrm) ||
              (rt <= 1.0020841800044864E-292)) {
            s[qjj - 1] = 0.0;
            exitg1 = true;
          } else {
            qjj--;
          }
        }
      }
      if (qs == qq) {
        qjj = 3;
      } else if (qs == m + 1) {
        qjj = 1;
      } else {
        qjj = 2;
        qq = qs;
      }
    }
    switch (qjj) {
    case 1:
      f = e[m - 1];
      e[m - 1] = 0.0;
      for (k = m; k >= qq + 1; k--) {
        rt = xrotg(&s[k - 1], &f, &nrm);
        if (k > qq + 1) {
          f = -nrm * e[0];
          e[0] *= rt;
        }
        xrot(V, 3 * (k - 1) + 1, 3 * m + 1, rt, nrm);
      }
      break;
    case 2:
      f = e[qq - 1];
      e[qq - 1] = 0.0;
      for (k = qq + 1; k <= m + 1; k++) {
        rt = xrotg(&s[k - 1], &f, &nrm);
        b = e[k - 1];
        f = -nrm * b;
        e[k - 1] = b * rt;
        xrot(U, 3 * (k - 1) + 1, 3 * (qq - 1) + 1, rt, nrm);
      }
      break;
    case 3: {
      double scale;
      nrm = s[m - 1];
      rt = e[m - 1];
      scale =
          fmax(fmax(fmax(fmax(fabs(s[m]), fabs(nrm)), fabs(rt)), fabs(s[qq])),
               fabs(e[qq]));
      sm = s[m] / scale;
      nrm /= scale;
      rt /= scale;
      sqds = s[qq] / scale;
      b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
      nrm = sm * rt;
      nrm *= nrm;
      if ((b != 0.0) || (nrm != 0.0)) {
        rt = sqrt(b * b + nrm);
        if (b < 0.0) {
          rt = -rt;
        }
        rt = nrm / (b + rt);
      } else {
        rt = 0.0;
      }
      f = (sqds + sm) * (sqds - sm) + rt;
      nrm = sqds * (e[qq] / scale);
      for (k = qq + 1; k <= m; k++) {
        b = xrotg(&f, &nrm, &sm);
        if (k > qq + 1) {
          e[0] = f;
        }
        nrm = e[k - 1];
        rt = s[k - 1];
        e[k - 1] = b * nrm - sm * rt;
        sqds = sm * s[k];
        s[k] *= b;
        qjj = 3 * (k - 1) + 1;
        qs = 3 * k + 1;
        xrot(V, qjj, qs, b, sm);
        s[k - 1] = b * rt + sm * nrm;
        rt = xrotg(&s[k - 1], &sqds, &b);
        nrm = e[k - 1];
        f = rt * nrm + b * s[k];
        s[k] = -b * nrm + rt * s[k];
        nrm = b * e[k];
        e[k] *= rt;
        xrot(U, qjj, qs, rt, b);
      }
      e[m - 1] = f;
      qp1++;
    } break;
    default:
      if (s[qq] < 0.0) {
        s[qq] = -s[qq];
        qjj = 3 * qq + 1;
        for (k = qjj; k <= qjj + 2; k++) {
          V[k - 1] = -V[k - 1];
        }
      }
      qp1 = qq + 1;
      while ((qq + 1 < 3) && (s[qq] < s[qp1])) {
        rt = s[qq];
        s[qq] = s[qp1];
        s[qp1] = rt;
        qs = 3 * qq + 1;
        qjj = 3 * (qq + 1) + 1;
        xswap(V, qs, qjj);
        xswap(U, qs, qjj);
        qq = qp1;
        qp1++;
      }
      qp1 = 0;
      m--;
      break;
    }
  }
  if (doscale) {
    b_xzlascl(cscale, anrm, 3, s, 1);
  }
}

/*
 * File trailer for svd1.c
 *
 * [EOF]
 */
