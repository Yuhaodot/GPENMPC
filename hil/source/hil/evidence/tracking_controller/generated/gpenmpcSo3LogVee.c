/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcSo3LogVee.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcSo3LogVee.h"
#include "dot.h"
#include "eig.h"
#include "eye.h"
#include "minOrMax.h"
#include "norm.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCSO3LOGVEE Principal SO(3) logarithm as a three-vector.
 *
 * Arguments    : e_gpenmpcNative_canonicalLocalIn *SD
 *                const double rotation[9]
 *                double vector[3]
 * Return Type  : void
 */
void gpenmpcSo3LogVee(e_gpenmpcNative_canonicalLocalIn *SD,
                     const double rotation[9], double vector[3])
{
  double matrix[9];
  double angle;
  double t;
  int k;
  int matrix_tmp;
  t = 0.0;
  for (k = 0; k < 3; k++) {
    t += rotation[k + 3 * k];
    matrix[3 * k] = rotation[3 * k] - rotation[k];
    matrix_tmp = 3 * k + 1;
    matrix[matrix_tmp] = rotation[matrix_tmp] - rotation[k + 3];
    matrix_tmp = 3 * k + 2;
    matrix[matrix_tmp] = rotation[matrix_tmp] - rotation[k + 6];
  }
  angle = acos(fmin(fmax((t - 1.0) / 2.0, -1.0), 1.0));
  /* GPENMPCVEE Vee map for a 3-by-3 skew-symmetric matrix. */
  vector[0] = 0.5 * matrix[5];
  vector[1] = 0.5 * matrix[6];
  vector[2] = 0.5 * matrix[1];
  if (!(angle < 1.0E-8)) {
    if (3.141592653589793 - angle < 1.0E-6) {
      double axis[3];
      eye(matrix);
      for (k = 0; k < 9; k++) {
        matrix[k] = 0.5 * (rotation[k] + matrix[k]);
      }
      eig(matrix, SD->u1.f0.eigenvectors, SD->u1.f0.eigenvalues);
      axis[0] = SD->u1.f0.eigenvalues[0].re;
      axis[1] = SD->u1.f0.eigenvalues[4].re;
      axis[2] = SD->u1.f0.eigenvalues[8].re;
      maximum(axis, &matrix_tmp);
      matrix_tmp = 3 * (matrix_tmp - 1);
      axis[0] = SD->u1.f0.eigenvectors[matrix_tmp].re;
      axis[1] = SD->u1.f0.eigenvectors[matrix_tmp + 1].re;
      axis[2] = SD->u1.f0.eigenvectors[matrix_tmp + 2].re;
      t = fmax(c_norm(axis), 1.0E-15);
      axis[0] /= t;
      axis[1] /= t;
      axis[2] /= t;
      if (dot(axis, vector) < 0.0) {
        axis[0] = -axis[0];
        axis[1] = -axis[1];
        axis[2] = -axis[2];
      }
      vector[0] = angle * axis[0];
      vector[1] = angle * axis[1];
      vector[2] = angle * axis[2];
    } else {
      t = angle / sin(angle);
      vector[0] *= t;
      vector[1] *= t;
      vector[2] *= t;
    }
  }
}

/*
 * File trailer for gpenmpcSo3LogVee.c
 *
 * [EOF]
 */
