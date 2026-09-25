/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzgebal.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xzgebal.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double A[9]
 *                int *ihi
 *                double scale[3]
 * Return Type  : int
 */
int xzgebal(double A[9], int *ihi, double scale[3])
{
  double temp;
  int b_j;
  int exitg5;
  int ilo;
  int ix;
  int iy;
  int j;
  int k;
  int l;
  int temp_tmp;
  boolean_T notdone;
  boolean_T skipThisRow;
  scale[0] = 1.0;
  scale[1] = 1.0;
  scale[2] = 1.0;
  ilo = 1;
  l = 3;
  notdone = true;
  do {
    exitg5 = 0;
    if (notdone) {
      int exitg4;
      notdone = false;
      j = l;
      do {
        exitg4 = 0;
        if (j > 0) {
          boolean_T exitg6;
          skipThisRow = false;
          ix = 0;
          exitg6 = false;
          while (!exitg6 && (ix <= l - 1)) {
            if ((ix + 1 == j) || !(A[(j + 3 * ix) - 1] != 0.0)) {
              ix++;
            } else {
              skipThisRow = true;
              exitg6 = true;
            }
          }
          if (skipThisRow) {
            j--;
          } else {
            scale[l - 1] = j;
            if (j != l) {
              ix = (j - 1) * 3;
              iy = (l - 1) * 3;
              for (k = 0; k < l; k++) {
                temp_tmp = ix + k;
                temp = A[temp_tmp];
                b_j = iy + k;
                A[temp_tmp] = A[b_j];
                A[b_j] = temp;
              }
              temp = A[j - 1];
              A[j - 1] = A[l - 1];
              A[l - 1] = temp;
              temp = A[j + 2];
              A[j + 2] = A[l + 2];
              A[l + 2] = temp;
              temp = A[j + 5];
              A[j + 5] = A[l + 5];
              A[l + 5] = temp;
            }
            exitg4 = 1;
          }
        } else {
          exitg4 = 2;
        }
      } while (exitg4 == 0);
      if (exitg4 == 1) {
        if (l == 1) {
          ilo = 1;
          *ihi = 1;
          exitg5 = 1;
        } else {
          l--;
          notdone = true;
        }
      }
    } else {
      notdone = true;
      while (notdone) {
        boolean_T exitg6;
        notdone = false;
        b_j = ilo;
        exitg6 = false;
        while (!exitg6 && (b_j <= l)) {
          boolean_T exitg7;
          skipThisRow = false;
          ix = ilo;
          exitg7 = false;
          while (!exitg7 && (ix <= l)) {
            if ((ix == b_j) || !(A[(ix + 3 * (b_j - 1)) - 1] != 0.0)) {
              ix++;
            } else {
              skipThisRow = true;
              exitg7 = true;
            }
          }
          if (skipThisRow) {
            b_j++;
          } else {
            scale[ilo - 1] = b_j;
            if (b_j != ilo) {
              int b_ix;
              ix = (b_j - 1) * 3;
              j = (ilo - 1) * 3;
              for (k = 0; k < l; k++) {
                iy = ix + k;
                temp = A[iy];
                temp_tmp = j + k;
                A[iy] = A[temp_tmp];
                A[temp_tmp] = temp;
              }
              b_ix = (j + b_j) - 1;
              ix = (j + ilo) - 1;
              iy = (unsigned char)(4 - ilo);
              for (k = 0; k < iy; k++) {
                temp_tmp = b_ix + k * 3;
                temp = A[temp_tmp];
                b_j = ix + k * 3;
                A[temp_tmp] = A[b_j];
                A[b_j] = temp;
              }
            }
            ilo++;
            notdone = true;
            exitg6 = true;
          }
        }
      }
      *ihi = l;
      skipThisRow = false;
      exitg5 = 2;
    }
  } while (exitg5 == 0);
  if (exitg5 != 1) {
    boolean_T exitg3;
    exitg3 = false;
    while (!exitg3 && !skipThisRow) {
      int exitg2;
      skipThisRow = true;
      b_j = ilo - 1;
      do {
        exitg2 = 0;
        if (b_j + 1 <= l) {
          double b_s;
          double c;
          double ca;
          double r;
          double s;
          ix = (l - ilo) + 1;
          c = xnrm2(ix, A, b_j * 3 + ilo);
          temp_tmp = (ilo - 1) * 3 + b_j;
          j = temp_tmp + 1;
          r = 0.0;
          if (ix >= 1) {
            if (ix == 1) {
              r = fabs(A[temp_tmp]);
            } else {
              temp = 3.312168642111238E-170;
              iy = (temp_tmp + (ix - 1) * 3) + 1;
              for (k = j; k <= iy; k += 3) {
                s = fabs(A[k - 1]);
                if (s > temp) {
                  b_s = temp / s;
                  r = r * b_s * b_s + 1.0;
                  temp = s;
                } else {
                  b_s = s / temp;
                  r += b_s * b_s;
                }
              }
              r = temp * sqrt(r);
              if (rtIsNaN(r)) {
                ix = temp_tmp + 1;
                int exitg8;
                do {
                  exitg8 = 0;
                  if (ix <= iy) {
                    if (rtIsNaN(A[ix - 1])) {
                      exitg8 = 1;
                    } else {
                      ix += 3;
                    }
                  } else {
                    r = rtInf;
                    exitg8 = 1;
                  }
                } while (exitg8 == 0);
              }
            }
          }
          ix = b_j * 3;
          iy = 1;
          if (l > 1) {
            temp = fabs(A[ix]);
            for (k = 2; k <= l; k++) {
              s = fabs(A[(ix + k) - 1]);
              if (s > temp) {
                iy = k;
                temp = s;
              }
            }
          }
          ca = fabs(A[(iy + 3 * b_j) - 1]);
          ix = 4 - ilo;
          if (4 - ilo < 1) {
            iy = 0;
          } else {
            iy = 1;
            if (4 - ilo > 1) {
              temp = fabs(A[temp_tmp]);
              for (k = 2; k <= ix; k++) {
                s = fabs(A[temp_tmp + (k - 1) * 3]);
                if (s > temp) {
                  iy = k;
                  temp = s;
                }
              }
            }
          }
          temp = fabs(A[b_j + 3 * ((iy + ilo) - 2)]);
          if ((c == 0.0) || (r == 0.0)) {
            b_j++;
          } else {
            double f;
            int exitg1;
            s = r / 2.0;
            f = 1.0;
            b_s = c + r;
            do {
              exitg1 = 0;
              if ((c < s) && (fmax(f, fmax(c, ca)) < 4.9896007738368E+291) &&
                  (fmin(r, fmin(s, temp)) > 2.004168360008973E-292)) {
                if (rtIsNaN(((((c + f) + ca) + r) + s) + temp)) {
                  exitg1 = 1;
                } else {
                  f *= 2.0;
                  c *= 2.0;
                  ca *= 2.0;
                  r /= 2.0;
                  s /= 2.0;
                  temp /= 2.0;
                }
              } else {
                s = c / 2.0;
                while (
                    (s >= r) && (fmax(r, temp) < 4.9896007738368E+291) &&
                    (fmin(fmin(f, c), fmin(s, ca)) > 2.004168360008973E-292)) {
                  f /= 2.0;
                  c /= 2.0;
                  s /= 2.0;
                  ca /= 2.0;
                  r *= 2.0;
                  temp *= 2.0;
                }
                if (!(c + r >= 0.95 * b_s) &&
                    (!(f < 1.0) || !(scale[b_j] < 1.0) ||
                     !(f * scale[b_j] <= 1.0020841800044864E-292)) &&
                    (!(f > 1.0) || !(scale[b_j] > 1.0) ||
                     !(scale[b_j] >= 9.9792015476736E+291 / f))) {
                  temp = 1.0 / f;
                  scale[b_j] *= f;
                  ix = (temp_tmp + 3 * (3 - ilo)) + 1;
                  for (k = j; k <= ix; k += 3) {
                    A[k - 1] *= temp;
                  }
                  ix = b_j * 3;
                  iy = ix + l;
                  for (k = ix + 1; k <= iy; k++) {
                    A[k - 1] *= f;
                  }
                  skipThisRow = false;
                }
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = 2;
            } else {
              b_j++;
            }
          }
        } else {
          exitg2 = 1;
        }
      } while (exitg2 == 0);
      if (exitg2 != 1) {
        exitg3 = true;
      }
    }
  }
  return ilo;
}

/*
 * File trailer for xzgebal.c
 *
 * [EOF]
 */
