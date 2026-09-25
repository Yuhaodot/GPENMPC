/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcFrenetFrame.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcFrenetFrame.h"
#include "norm.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_rtwutil.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCFRENETFRAME Build the inherited horizontal Frenet frame.
 *
 * Arguments    : const double reference_velocity_mps[3]
 *                const double reference_acceleration_mps2[3]
 *                double frameIFromF[9]
 *                double *signedYawRate
 * Return Type  : double
 */
double gpenmpcFrenetFrame(const double reference_velocity_mps[3],
                         const double reference_acceleration_mps2[3],
                         double frameIFromF[9], double *signedYawRate)
{
  double curvature;
  double horizontalSpeed;
  double tangent_idx_0;
  horizontalSpeed = b_norm(&reference_velocity_mps[0]);
  if (horizontalSpeed < 1.0E-9) {
    tangent_idx_0 = 1.0;
    curvature = 0.0;
  } else {
    tangent_idx_0 = reference_velocity_mps[0] / horizontalSpeed;
    curvature = reference_velocity_mps[1] / horizontalSpeed;
  }
  frameIFromF[0] = tangent_idx_0;
  frameIFromF[3] = -curvature;
  frameIFromF[6] = 0.0;
  frameIFromF[1] = curvature;
  frameIFromF[4] = tangent_idx_0;
  frameIFromF[7] = 0.0;
  frameIFromF[2] = 0.0;
  frameIFromF[5] = 0.0;
  frameIFromF[8] = 1.0;
  curvature = 0.0;
  *signedYawRate = 0.0;
  if (horizontalSpeed >= 0.75) {
    tangent_idx_0 = reference_velocity_mps[0] * reference_acceleration_mps2[1] -
                    reference_acceleration_mps2[0] * reference_velocity_mps[1];
    curvature = fabs(tangent_idx_0) / rt_powd_snf(horizontalSpeed, 3.0);
    *signedYawRate = tangent_idx_0 / (horizontalSpeed * horizontalSpeed);
  }
  return curvature;
}

/*
 * File trailer for gpenmpcFrenetFrame.c
 *
 * [EOF]
 */
