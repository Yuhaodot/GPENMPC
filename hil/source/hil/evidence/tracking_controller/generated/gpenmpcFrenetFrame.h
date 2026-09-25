/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcFrenetFrame.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCFRENETFRAME_H
#define GPENMPCFRENETFRAME_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double gpenmpcFrenetFrame(const double reference_velocity_mps[3],
                         const double reference_acceleration_mps2[3],
                         double frameIFromF[9], double *signedYawRate);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcFrenetFrame.h
 *
 * [EOF]
 */
