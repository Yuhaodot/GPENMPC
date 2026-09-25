/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcM600Allocation.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCM600ALLOCATION_H
#define GPENMPCM600ALLOCATION_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double gpenmpcM600Allocation(const double c_calibration_rotor_allocation_[6],
                            const double d_calibration_rotor_allocation_[6],
                            double allocation_matrix[24],
                            double allocation_pseudoinverse[24],
                            double *allocation_total_thrust_upper_n,
                            double *c_allocation_actuator_time_cons,
                            double allocation_inertia_kg_m2[9],
                            double c_allocation_nominal_drag_n_per[3],
                            double *allocation_maximum_tilt_rad);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcM600Allocation.h
 *
 * [EOF]
 */
