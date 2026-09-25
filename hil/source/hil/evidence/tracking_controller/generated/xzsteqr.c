/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzsteqr.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "xzsteqr.h"
#include "rt_nonfinite.h"
#include "xdlaev2.h"
#include "xzlartg.h"
#include "xzlascl.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static void b_rotateRight(int n, double z[9], int iz0, const double cs[4],
                          int ic0, int is0);

static void rotateRight(int n, double z[9], int iz0, const double cs[4],
                        int ic0, int is0);

/* Function Definitions */
/*
 * Arguments    : int n
 *                double z[9]
 *                int iz0
 *                const double cs[4]
 *                int ic0
 *                int is0
 * Return Type  : void
 */
static void b_rotateRight(int n, double z[9], int iz0, const double cs[4],
                          int ic0, int is0)
{
  int j;
  for (j = 0; j <= n - 2; j++) {
    double ctemp;
    double stemp;
    int offsetj;
    int offsetjp1;
    ctemp = cs[(ic0 + j) - 1];
    stemp = cs[(is0 + j) - 1];
    offsetj = (j * 3 + iz0) - 2;
    offsetjp1 = ((j + 1) * 3 + iz0) - 2;
    if ((ctemp != 1.0) || (stemp != 0.0)) {
      double d;
      double temp;
      temp = z[offsetjp1 + 1];
      d = z[offsetj + 1];
      z[offsetjp1 + 1] = ctemp * temp - stemp * d;
      d = stemp * temp + ctemp * d;
      z[offsetj + 1] = d;
      temp = z[offsetjp1 + 2];
      d = z[offsetj + 2];
      z[offsetjp1 + 2] = ctemp * temp - stemp * d;
      d = stemp * temp + ctemp * d;
      z[offsetj + 2] = d;
      temp = z[offsetjp1 + 3];
      d = z[offsetj + 3];
      z[offsetjp1 + 3] = ctemp * temp - stemp * d;
      d = stemp * temp + ctemp * d;
      z[offsetj + 3] = d;
    }
  }
}

/*
 * Arguments    : int n
 *                double z[9]
 *                int iz0
 *                const double cs[4]
 *                int ic0
 *                int is0
 * Return Type  : void
 */
static void rotateRight(int n, double z[9], int iz0, const double cs[4],
                        int ic0, int is0)
{
  int j;
  for (j = n - 1; j >= 1; j--) {
    double ctemp;
    double stemp;
    int offsetj;
    int offsetjp1;
    ctemp = cs[(ic0 + j) - 2];
    stemp = cs[(is0 + j) - 2];
    offsetj = ((j - 1) * 3 + iz0) - 2;
    offsetjp1 = (j * 3 + iz0) - 2;
    if ((ctemp != 1.0) || (stemp != 0.0)) {
      double d;
      double temp;
      temp = z[offsetjp1 + 1];
      d = z[offsetj + 1];
      z[offsetjp1 + 1] = ctemp * temp - stemp * d;
      d = stemp * temp + ctemp * d;
      z[offsetj + 1] = d;
      temp = z[offsetjp1 + 2];
      d = z[offsetj + 2];
      z[offsetjp1 + 2] = ctemp * temp - stemp * d;
      d = stemp * temp + ctemp * d;
      z[offsetj + 2] = d;
      temp = z[offsetjp1 + 3];
      d = z[offsetj + 3];
      z[offsetjp1 + 3] = ctemp * temp - stemp * d;
      d = stemp * temp + ctemp * d;
      z[offsetj + 3] = d;
    }
  }
}

/*
 * Arguments    : double d[3]
 *                double e[2]
 *                double z[9]
 * Return Type  : int
 */
int xzsteqr(double d[3], double e[2], double z[9])
{
  double work[4];
  double g_tmp;
  double r;
  double s;
  double temp;
  int ii;
  int info;
  int j;
  int jtot;
  int l1;
  info = 0;
  work[0] = 0.0;
  work[1] = 0.0;
  work[2] = 0.0;
  work[3] = 0.0;
  jtot = 0;
  l1 = 1;
  int exitg1;
  do {
    exitg1 = 0;
    if (l1 > 3) {
      for (ii = 0; ii < 2; ii++) {
        double p;
        int k;
        k = ii;
        p = d[ii];
        for (j = ii + 2; j < 4; j++) {
          temp = d[j - 1];
          if (temp < p) {
            k = j - 1;
            p = temp;
          }
        }
        if (k != ii) {
          int ix;
          d[k] = d[ii];
          d[ii] = p;
          ix = ii * 3;
          k *= 3;
          temp = z[ix];
          z[ix] = z[k];
          z[k] = temp;
          temp = z[ix + 1];
          z[ix + 1] = z[k + 1];
          z[k + 1] = temp;
          temp = z[ix + 2];
          z[ix + 2] = z[k + 2];
          z[k + 2] = temp;
        }
      }
      exitg1 = 1;
    } else {
      int l;
      int lend;
      int lendsv;
      int lsv;
      int m;
      boolean_T exitg2;
      if (l1 > 1) {
        e[l1 - 2] = 0.0;
      }
      m = l1;
      exitg2 = false;
      while (!exitg2 && (m < 3)) {
        temp = fabs(e[m - 1]);
        if (temp == 0.0) {
          exitg2 = true;
        } else if (temp <= sqrt(fabs(d[m - 1])) * sqrt(fabs(d[m])) *
                               2.220446049250313E-16) {
          e[m - 1] = 0.0;
          exitg2 = true;
        } else {
          m++;
        }
      }
      l = l1 - 1;
      lsv = l1;
      lend = m;
      lendsv = m;
      l1 = m + 1;
      if (m != l + 1) {
        double anorm;
        int k;
        int n_tmp;
        n_tmp = m - l;
        if (n_tmp <= 0) {
          anorm = 0.0;
        } else {
          anorm = fabs(d[(l + n_tmp) - 1]);
          k = 0;
          exitg2 = false;
          while (!exitg2 && (k <= n_tmp - 2)) {
            int ix;
            ix = l + k;
            temp = fabs(d[ix]);
            if (rtIsNaN(temp)) {
              anorm = rtNaN;
              exitg2 = true;
            } else {
              if (temp > anorm) {
                anorm = temp;
              }
              temp = fabs(e[ix]);
              if (rtIsNaN(temp)) {
                anorm = rtNaN;
                exitg2 = true;
              } else {
                if (temp > anorm) {
                  anorm = temp;
                }
                k++;
              }
            }
          }
        }
        k = 0;
        if (!(anorm == 0.0)) {
          if (rtIsInf(anorm) || rtIsNaN(anorm)) {
            d[0] = rtNaN;
            d[1] = rtNaN;
            d[2] = rtNaN;
            for (j = 0; j < 9; j++) {
              z[j] = rtNaN;
            }
            exitg1 = 1;
          } else {
            if (anorm > 2.2346346549904327E+153) {
              k = 1;
              b_xzlascl(anorm, 2.2346346549904327E+153, n_tmp, d, l + 1);
              c_xzlascl(anorm, 2.2346346549904327E+153, n_tmp - 1, e, l + 1);
            } else if (anorm < 3.02546243347603E-123) {
              k = 2;
              b_xzlascl(anorm, 3.02546243347603E-123, n_tmp, d, l + 1);
              c_xzlascl(anorm, 3.02546243347603E-123, n_tmp - 1, e, l + 1);
            }
            if (fabs(d[m - 1]) < fabs(d[l])) {
              lend = lsv;
              l = m - 1;
            }
            if (lend > l + 1) {
              int exitg4;
              do {
                exitg4 = 0;
                if (l + 1 != lend) {
                  m = l;
                  exitg2 = false;
                  while (!exitg2 && (m + 1 < lend)) {
                    temp = fabs(e[m]);
                    if (temp * temp <=
                        4.930380657631324E-32 * fabs(d[m]) * fabs(d[m + 1]) +
                            2.2250738585072014E-308) {
                      exitg2 = true;
                    } else {
                      m++;
                    }
                  }
                } else {
                  m = lend - 1;
                }
                if (m + 1 < lend) {
                  e[m] = 0.0;
                }
                if (m + 1 == l + 1) {
                  l++;
                  if (l + 1 > lend) {
                    exitg4 = 1;
                  }
                } else if (m + 1 == l + 2) {
                  d[l] = xdlaev2(d[l], e[l], d[l + 1], &temp, &work[l], &r);
                  d[l + 1] = temp;
                  work[l + 2] = r;
                  rotateRight(2, z, l * 3 + 1, work, l + 1, l + 3);
                  e[l] = 0.0;
                  l += 2;
                  if (l + 1 > lend) {
                    exitg4 = 1;
                  }
                } else if (jtot == 90) {
                  exitg4 = 1;
                } else {
                  double c;
                  double g;
                  double p;
                  jtot++;
                  g = (d[l + 1] - d[l]) / (2.0 * e[l]);
                  temp = fabs(g);
                  if (temp < 1.0) {
                    temp = sqrt(temp * temp + 1.0);
                  } else if (temp > 1.0) {
                    r = 1.0 / temp;
                    temp *= sqrt(r * r + 1.0);
                  } else {
                    temp *= 1.4142135623730951;
                  }
                  if (!(g >= 0.0)) {
                    temp = -temp;
                  }
                  g = (d[m] - d[l]) + e[l] / (g + temp);
                  s = 1.0;
                  c = 1.0;
                  p = 0.0;
                  for (j = m; j >= l + 1; j--) {
                    double b;
                    temp = e[j - 1];
                    b = c * temp;
                    c = xzlartg(g, s * temp, &s, &r);
                    if (j != m) {
                      e[1] = r;
                    }
                    g = d[j] - p;
                    temp = (d[j - 1] - g) * s + 2.0 * c * b;
                    p = s * temp;
                    d[j] = g + p;
                    g = c * temp - b;
                    work[j - 1] = c;
                    work[j + 1] = -s;
                  }
                  rotateRight((m - l) + 1, z, l * 3 + 1, work, l + 1, l + 3);
                  d[l] -= p;
                  e[l] = g;
                }
              } while (exitg4 == 0);
            } else {
              int exitg3;
              do {
                exitg3 = 0;
                if (l + 1 != lend) {
                  m = l + 1;
                  exitg2 = false;
                  while (!exitg2 && (m > lend)) {
                    temp = fabs(e[m - 2]);
                    if (temp * temp <= 4.930380657631324E-32 * fabs(d[m - 1]) *
                                               fabs(d[m - 2]) +
                                           2.2250738585072014E-308) {
                      exitg2 = true;
                    } else {
                      m--;
                    }
                  }
                } else {
                  m = lend;
                }
                if (m > lend) {
                  e[m - 2] = 0.0;
                }
                if (m == l + 1) {
                  l--;
                  if (l + 1 < lend) {
                    exitg3 = 1;
                  }
                } else if (m == l) {
                  d[l - 1] = xdlaev2(d[l - 1], e[l - 1], d[l], &temp,
                                     &work[m - 1], &r);
                  d[l] = temp;
                  work[m + 1] = r;
                  b_rotateRight(2, z, (l - 1) * 3 + 1, work, m, m + 2);
                  e[l - 1] = 0.0;
                  l -= 2;
                  if (l + 1 < lend) {
                    exitg3 = 1;
                  }
                } else if (jtot == 90) {
                  exitg3 = 1;
                } else {
                  double c;
                  double g;
                  double p;
                  jtot++;
                  g_tmp = e[l - 1];
                  g = (d[l - 1] - d[l]) / (2.0 * g_tmp);
                  temp = fabs(g);
                  if (temp < 1.0) {
                    temp = sqrt(temp * temp + 1.0);
                  } else if (temp > 1.0) {
                    r = 1.0 / temp;
                    temp *= sqrt(r * r + 1.0);
                  } else {
                    temp *= 1.4142135623730951;
                  }
                  if (!(g >= 0.0)) {
                    temp = -temp;
                  }
                  g = (d[m - 1] - d[l]) + g_tmp / (g + temp);
                  s = 1.0;
                  c = 1.0;
                  p = 0.0;
                  for (j = m; j <= l; j++) {
                    double b;
                    temp = e[j - 1];
                    b = c * temp;
                    c = xzlartg(g, s * temp, &s, &g_tmp);
                    if (j != m) {
                      e[0] = g_tmp;
                    }
                    g = d[j - 1] - p;
                    temp = (d[j] - g) * s + 2.0 * c * b;
                    p = s * temp;
                    d[j - 1] = g + p;
                    g = c * temp - b;
                    work[j - 1] = c;
                    work[j + 1] = s;
                  }
                  b_rotateRight((l - m) + 2, z, (m - 1) * 3 + 1, work, m,
                                m + 2);
                  d[l] -= p;
                  e[l - 1] = g;
                }
              } while (exitg3 == 0);
            }
            if (k == 1) {
              k = lendsv - lsv;
              b_xzlascl(2.2346346549904327E+153, anorm, k + 1, d, lsv);
              c_xzlascl(2.2346346549904327E+153, anorm, k, e, lsv);
            } else if (k == 2) {
              k = lendsv - lsv;
              b_xzlascl(3.02546243347603E-123, anorm, k + 1, d, lsv);
              c_xzlascl(3.02546243347603E-123, anorm, k, e, lsv);
            }
            if (jtot >= 90) {
              if (e[0] != 0.0) {
                info = 1;
              }
              if (e[1] != 0.0) {
                info++;
              }
              exitg1 = 1;
            }
          }
        }
      }
    }
  } while (exitg1 == 0);
  return info;
}

/*
 * File trailer for xzsteqr.c
 *
 * [EOF]
 */
