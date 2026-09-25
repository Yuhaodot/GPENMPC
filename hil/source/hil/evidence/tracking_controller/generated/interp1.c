/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: interp1.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "interp1.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : e_gpenmpcNative_canonicalLocalIn *SD
 *                const double varargin_1_data[]
 *                int varargin_1_size
 *                const double varargin_2_data[]
 *                const int varargin_2_size[2]
 *                double varargin_3
 *                double Vq[3]
 * Return Type  : void
 */
void interp1(e_gpenmpcNative_canonicalLocalIn *SD,
             const double varargin_1_data[], int varargin_1_size,
             const double varargin_2_data[], const int varargin_2_size[2],
             double varargin_3, double Vq[3])
{
  double xtmp;
  int b_j1;
  int k;
  int low_i;
  int n;
  int nd2;
  int offset;
  n = varargin_2_size[0] * 3;
  memcpy(&SD->u1.f1.y_data[0], &varargin_2_data[0],
         (unsigned int)n * sizeof(double));
  memcpy(&SD->u1.f1.x_data[0], &varargin_1_data[0],
         (unsigned int)varargin_1_size * sizeof(double));
  if (varargin_1_data[1] < varargin_1_data[0]) {
    n = varargin_1_size >> 1;
    for (b_j1 = 0; b_j1 < n; b_j1++) {
      xtmp = SD->u1.f1.x_data[b_j1];
      nd2 = (varargin_1_size - b_j1) - 1;
      SD->u1.f1.x_data[b_j1] = SD->u1.f1.x_data[nd2];
      SD->u1.f1.x_data[nd2] = xtmp;
    }
    n = varargin_2_size[0] - 1;
    nd2 = varargin_2_size[0] >> 1;
    for (b_j1 = 0; b_j1 < 3; b_j1++) {
      offset = b_j1 * varargin_2_size[0];
      for (k = 0; k < nd2; k++) {
        int i;
        low_i = offset + k;
        xtmp = SD->u1.f1.y_data[low_i];
        i = (offset + n) - k;
        SD->u1.f1.y_data[low_i] = SD->u1.f1.y_data[i];
        SD->u1.f1.y_data[i] = xtmp;
      }
    }
  }
  Vq[0] = rtNaN;
  Vq[1] = rtNaN;
  Vq[2] = rtNaN;
  if (!(varargin_3 > SD->u1.f1.x_data[varargin_1_size - 1]) &&
      !(varargin_3 < SD->u1.f1.x_data[0])) {
    n = varargin_1_size;
    low_i = 1;
    nd2 = 2;
    while (n > nd2) {
      offset = (low_i >> 1) + (n >> 1);
      if ((((unsigned int)low_i & 1U) == 1U) &&
          (((unsigned int)n & 1U) == 1U)) {
        offset++;
      }
      if (varargin_3 >= SD->u1.f1.x_data[offset - 1]) {
        low_i = offset;
        nd2 = offset + 1;
      } else {
        n = offset;
      }
    }
    xtmp = SD->u1.f1.x_data[low_i - 1];
    xtmp = (varargin_3 - xtmp) / (SD->u1.f1.x_data[low_i] - xtmp);
    if (xtmp == 0.0) {
      Vq[0] = SD->u1.f1.y_data[low_i - 1];
      Vq[1] = SD->u1.f1.y_data[(low_i + varargin_2_size[0]) - 1];
      Vq[2] = SD->u1.f1.y_data[(low_i + (varargin_2_size[0] << 1)) - 1];
    } else if (xtmp == 1.0) {
      Vq[0] = SD->u1.f1.y_data[low_i];
      Vq[1] = SD->u1.f1.y_data[low_i + varargin_2_size[0]];
      Vq[2] = SD->u1.f1.y_data[low_i + (varargin_2_size[0] << 1)];
    } else {
      double b_y1;
      double y2;
      b_y1 = SD->u1.f1.y_data[low_i - 1];
      y2 = SD->u1.f1.y_data[low_i];
      if (b_y1 == y2) {
        Vq[0] = b_y1;
      } else {
        Vq[0] = (1.0 - xtmp) * b_y1 + xtmp * y2;
      }
      n = low_i + varargin_2_size[0];
      b_y1 = SD->u1.f1.y_data[n - 1];
      y2 = SD->u1.f1.y_data[n];
      if (b_y1 == y2) {
        Vq[1] = b_y1;
      } else {
        Vq[1] = (1.0 - xtmp) * b_y1 + xtmp * y2;
      }
      n = low_i + (varargin_2_size[0] << 1);
      b_y1 = SD->u1.f1.y_data[n - 1];
      y2 = SD->u1.f1.y_data[n];
      if (b_y1 == y2) {
        Vq[2] = b_y1;
      } else {
        Vq[2] = (1.0 - xtmp) * b_y1 + xtmp * y2;
      }
    }
  }
}

/*
 * File trailer for interp1.c
 *
 * [EOF]
 */
