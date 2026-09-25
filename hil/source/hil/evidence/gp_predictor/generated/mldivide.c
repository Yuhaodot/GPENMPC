/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mldivide.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-06 17:53:25
 */

/* Include Files */
#include "mldivide.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double A[65536]
 *                double B[256]
 * Return Type  : void
 */
void mldivide(const double A[65536], double B[256])
{
  static double b_A[65536];
  double smax;
  int b_k;
  int j;
  int jA;
  int k;
  short ipiv[256];
  short iv[8];
  memcpy(&b_A[0], &A[0], 65536U * sizeof(double));
  for (k = 0; k <= 248; k += 8) {
    __m128i r;
    iv[0] = (short)k;
    iv[1] = (short)(k + 1);
    iv[2] = (short)(k + 2);
    iv[3] = (short)(k + 3);
    iv[4] = (short)(k + 4);
    iv[5] = (short)(k + 5);
    iv[6] = (short)(k + 6);
    iv[7] = (short)(k + 7);
    r = _mm_loadu_si128((const __m128i *)&iv[0]);
    _mm_storeu_si128((__m128i *)&ipiv[k], _mm_add_epi16(r, _mm_set1_epi16(1)));
  }
  for (j = 0; j < 255; j++) {
    int a;
    int b;
    int jj;
    int mmj;
    short i;
    mmj = 254 - j;
    b = j * 257;
    jj = j * 257;
    jA = 257 - j;
    a = 0;
    smax = fabs(b_A[jj]);
    for (b_k = 2; b_k < jA; b_k++) {
      double s;
      s = fabs(b_A[(b + b_k) - 1]);
      if (s > smax) {
        a = b_k - 1;
        smax = s;
      }
    }
    if (b_A[jj + a] != 0.0) {
      if (a != 0) {
        a += j;
        ipiv[j] = (short)(a + 1);
        for (k = 0; k < 256; k++) {
          int temp_tmp;
          jA = k << 8;
          temp_tmp = j + jA;
          smax = b_A[temp_tmp];
          jA += a;
          b_A[temp_tmp] = b_A[jA];
          b_A[jA] = smax;
        }
      }
      jA = (jj - j) + 256;
      for (k = b + 2; k <= jA; k++) {
        b_A[k - 1] /= b_A[jj];
      }
    }
    jA = jj;
    for (k = 0; k <= mmj; k++) {
      smax = b_A[(b + (k << 8)) + 256];
      if (smax != 0.0) {
        a = (jA - j) + 512;
        for (b_k = jA + 258; b_k <= a; b_k++) {
          b_A[b_k - 1] += b_A[((jj + b_k) - jA) - 257] * -smax;
        }
      }
      jA += 256;
    }
    i = ipiv[j];
    if (i != j + 1) {
      smax = B[j];
      B[j] = B[i - 1];
      B[i - 1] = smax;
    }
  }
  for (k = 0; k < 256; k++) {
    jA = k << 8;
    if (B[k] != 0.0) {
      for (b_k = k + 2; b_k < 257; b_k++) {
        B[b_k - 1] -= B[k] * b_A[(b_k + jA) - 1];
      }
    }
  }
  for (k = 255; k >= 0; k--) {
    jA = k << 8;
    smax = B[k];
    if (smax != 0.0) {
      smax /= b_A[k + jA];
      B[k] = smax;
      for (b_k = 0; b_k < k; b_k++) {
        B[b_k] -= B[k] * b_A[b_k + jA];
      }
    }
  }
}

/*
 * File trailer for mldivide.c
 *
 * [EOF]
 */
