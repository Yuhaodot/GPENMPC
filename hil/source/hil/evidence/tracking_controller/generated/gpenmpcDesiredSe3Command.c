/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcDesiredSe3Command.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcDesiredSe3Command.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCDESIREDSE3COMMAND Build the common desired force and raw attitude.
 *
 *  Robust-SE(3) command construction shared by the optional attitude-continuity
 *  layer and the tracking-controller path.
 *
 * Arguments    : const double plantState[19]
 *                const double reference_position_m[3]
 *                const double reference_velocity_mps[3]
 *                const double reference_acceleration_mps2[3]
 *                double payloadKg
 *                const double windEstimateXyMps[2]
 *                const double augmentationIMps2[3]
 *                double command_desired_force_raw_n[3]
 *                double c_command_desired_force_project[3]
 *                double command_desired_rotation[9]
 * Return Type  : double
 */
double gpenmpcDesiredSe3Command(const double plantState[19],
                               const double reference_position_m[3],
                               const double reference_velocity_mps[3],
                               const double reference_acceleration_mps2[3],
                               double payloadKg,
                               const double windEstimateXyMps[2],
                               const double augmentationIMps2[3],
                               double command_desired_force_raw_n[3],
                               double c_command_desired_force_project[3],
                               double command_desired_rotation[9])
{
  double b2[3];
  double b2_tmp;
  double b3_idx_0;
  double b3_idx_1;
  double b3_idx_2;
  double c_command_force_projection_norm;
  double horizontal;
  double maximumHorizontal;
  command_desired_force_raw_n[0] =
      reference_velocity_mps[0] - windEstimateXyMps[0];
  command_desired_force_raw_n[1] =
      reference_velocity_mps[1] - windEstimateXyMps[1];
  /* GPENMPCPROJECTFORCE Apply the common force-norm and tilt authority limits.
   */
  b2_tmp =
      (payloadKg + 9.5) *
          (((reference_acceleration_mps2[0] +
             0.42250000000000004 * (reference_position_m[0] - plantState[0])) +
            1.1700000000000002 * (reference_velocity_mps[0] - plantState[3])) +
           augmentationIMps2[0]) +
      0.0634905529323215 * command_desired_force_raw_n[0] *
          fabs(command_desired_force_raw_n[0]);
  command_desired_force_raw_n[0] = b2_tmp;
  c_command_desired_force_project[0] = b2_tmp;
  b2_tmp =
      (payloadKg + 9.5) *
          (((reference_acceleration_mps2[1] +
             0.42250000000000004 * (reference_position_m[1] - plantState[1])) +
            1.1700000000000002 * (reference_velocity_mps[1] - plantState[4])) +
           augmentationIMps2[1]) +
      0.0634905529323215 * command_desired_force_raw_n[1] *
          fabs(command_desired_force_raw_n[1]);
  command_desired_force_raw_n[1] = b2_tmp;
  c_command_desired_force_project[1] = b2_tmp;
  b2_tmp = (payloadKg + 9.5) *
               ((((reference_acceleration_mps2[2] +
                   0.5625 * (reference_position_m[2] - plantState[2])) +
                  1.35 * (reference_velocity_mps[2] - plantState[5])) +
                 9.80665) +
                augmentationIMps2[2]) +
           0.0634905529323215 * reference_velocity_mps[2] *
               fabs(reference_velocity_mps[2]);
  command_desired_force_raw_n[2] = b2_tmp;
  c_command_desired_force_project[2] = b2_tmp;
  horizontal = c_norm(command_desired_force_raw_n);
  if (horizontal > 192.8743620548095) {
    horizontal = 192.8743620548095 / horizontal;
    c_command_desired_force_project[0] =
        command_desired_force_raw_n[0] * horizontal;
    c_command_desired_force_project[1] =
        command_desired_force_raw_n[1] * horizontal;
    c_command_desired_force_project[2] = b2_tmp * horizontal;
  }
  horizontal = b_norm(&c_command_desired_force_project[0]);
  maximumHorizontal =
      0.4663076581549986 * fmax(c_command_desired_force_project[2], 1.0E-9);
  if (horizontal > maximumHorizontal) {
    horizontal = maximumHorizontal / horizontal;
    c_command_desired_force_project[0] *= horizontal;
    c_command_desired_force_project[1] *= horizontal;
  }
  horizontal = fmax(c_norm(c_command_desired_force_project), 1.0E-15);
  b2[0] = command_desired_force_raw_n[0] - c_command_desired_force_project[0];
  b3_idx_0 = c_command_desired_force_project[0] / horizontal;
  b2[1] = command_desired_force_raw_n[1] - c_command_desired_force_project[1];
  b3_idx_1 = c_command_desired_force_project[1] / horizontal;
  b2[2] = b2_tmp - c_command_desired_force_project[2];
  b3_idx_2 = c_command_desired_force_project[2] / horizontal;
  c_command_force_projection_norm = c_norm(b2);
  horizontal = b3_idx_1 * 0.0;
  maximumHorizontal = 0.0 * b3_idx_2;
  b2[0] = horizontal - maximumHorizontal;
  b2_tmp = b3_idx_0 * 0.0;
  b2[1] = b3_idx_2 - b2_tmp;
  b2[2] = b2_tmp - b3_idx_1;
  if (c_norm(b2) < 1.0E-9) {
    b2[0] = horizontal - b3_idx_2;
    b2[1] = maximumHorizontal - b2_tmp;
    b2[2] = b3_idx_0 - horizontal;
  }
  horizontal = fmax(c_norm(b2), 1.0E-15);
  maximumHorizontal = b2[0] / horizontal;
  b2[0] = maximumHorizontal;
  command_desired_rotation[3] = maximumHorizontal;
  command_desired_rotation[6] = b3_idx_0;
  maximumHorizontal = b2[1] / horizontal;
  b2[1] = maximumHorizontal;
  command_desired_rotation[4] = maximumHorizontal;
  command_desired_rotation[7] = b3_idx_1;
  maximumHorizontal = b2[2] / horizontal;
  command_desired_rotation[5] = maximumHorizontal;
  command_desired_rotation[8] = b3_idx_2;
  command_desired_rotation[0] = b2[1] * b3_idx_2 - b3_idx_1 * maximumHorizontal;
  command_desired_rotation[1] = b3_idx_0 * maximumHorizontal - b2[0] * b3_idx_2;
  command_desired_rotation[2] = b2[0] * b3_idx_1 - b3_idx_0 * b2[1];
  return c_command_force_projection_norm;
}

/*
 * File trailer for gpenmpcDesiredSe3Command.c
 *
 * [EOF]
 */
