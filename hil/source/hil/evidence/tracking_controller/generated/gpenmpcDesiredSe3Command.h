/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcDesiredSe3Command.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCDESIREDSE3COMMAND_H
#define GPENMPCDESIREDSE3COMMAND_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double gpenmpcDesiredSe3Command(const double plantState[19],
                               const double reference_position_m[3],
                               const double reference_velocity_mps[3],
                               const double reference_acceleration_mps2[3],
                               double payloadKg,
                               const double windEstimateXyMps[2],
                               const double augmentationIMps2[3],
                               double command_desired_force_raw_n[3],
                               double c_command_desired_force_project[3],
                               double command_desired_rotation[9]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcDesiredSe3Command.h
 *
 * [EOF]
 */
