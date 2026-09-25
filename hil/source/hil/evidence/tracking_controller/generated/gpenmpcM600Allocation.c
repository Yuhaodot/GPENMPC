/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcM600Allocation.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcM600Allocation.h"
#include "diag.h"
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Fixed-shape allocation adapter for MATLAB Coder.
 *  Predeclare the eight output fields because Coder locks a struct's field
 *  set at its first read.
 *
 * Arguments    : const double c_calibration_rotor_allocation_[6]
 *                const double d_calibration_rotor_allocation_[6]
 *                double allocation_matrix[24]
 *                double allocation_pseudoinverse[24]
 *                double *allocation_total_thrust_upper_n
 *                double *c_allocation_actuator_time_cons
 *                double allocation_inertia_kg_m2[9]
 *                double c_allocation_nominal_drag_n_per[3]
 *                double *allocation_maximum_tilt_rad
 * Return Type  : double
 */
double gpenmpcM600Allocation(const double c_calibration_rotor_allocation_[6],
                            const double d_calibration_rotor_allocation_[6],
                            double allocation_matrix[24],
                            double allocation_pseudoinverse[24],
                            double *allocation_total_thrust_upper_n,
                            double *c_allocation_actuator_time_cons,
                            double allocation_inertia_kg_m2[9],
                            double c_allocation_nominal_drag_n_per[3],
                            double *allocation_maximum_tilt_rad)
{
  double b_allocation_matrix[16];
  double allocation_per_rotor_upper_n;
  int allocation_matrix_tmp;
  int i;
  int k;
  for (k = 0; k < 6; k++) {
    double d;
    allocation_per_rotor_upper_n =
        0.017453292519943295 * c_calibration_rotor_allocation_[k];
    d = sin(allocation_per_rotor_upper_n);
    allocation_per_rotor_upper_n = cos(allocation_per_rotor_upper_n);
    allocation_matrix_tmp = k << 2;
    allocation_matrix[allocation_matrix_tmp] = 1.0;
    allocation_matrix[allocation_matrix_tmp + 1] = 0.5665 * d;
    allocation_matrix[allocation_matrix_tmp + 2] =
        -0.5665 * allocation_per_rotor_upper_n;
    allocation_matrix[allocation_matrix_tmp + 3] =
        0.025 * d_calibration_rotor_allocation_[k];
  }
  for (k = 0; k < 4; k++) {
    for (i = 0; i < 6; i++) {
      allocation_pseudoinverse[i + 6 * k] = allocation_matrix[k + (i << 2)];
    }
  }
  memset(&b_allocation_matrix[0], 0, sizeof(double) << 4);
  for (k = 0; k < 4; k++) {
    allocation_matrix_tmp = k << 2;
    for (i = 0; i < 6; i++) {
      int b_allocation_matrix_tmp;
      allocation_per_rotor_upper_n = allocation_pseudoinverse[i + 6 * k];
      b_allocation_matrix_tmp = i << 2;
      b_allocation_matrix[allocation_matrix_tmp] +=
          allocation_matrix[b_allocation_matrix_tmp] *
          allocation_per_rotor_upper_n;
      b_allocation_matrix[allocation_matrix_tmp + 1] +=
          allocation_matrix[b_allocation_matrix_tmp + 1] *
          allocation_per_rotor_upper_n;
      b_allocation_matrix[allocation_matrix_tmp + 2] +=
          allocation_matrix[b_allocation_matrix_tmp + 2] *
          allocation_per_rotor_upper_n;
      b_allocation_matrix[allocation_matrix_tmp + 3] +=
          allocation_matrix[b_allocation_matrix_tmp + 3] *
          allocation_per_rotor_upper_n;
    }
  }
  mrdiv(allocation_pseudoinverse, b_allocation_matrix);
  allocation_per_rotor_upper_n = 32.145727009134916;
  *allocation_total_thrust_upper_n = 192.8743620548095;
  *c_allocation_actuator_time_cons = 0.12;
  diag(allocation_inertia_kg_m2);
  c_allocation_nominal_drag_n_per[0] = 0.0634905529323215;
  c_allocation_nominal_drag_n_per[1] = 0.0634905529323215;
  c_allocation_nominal_drag_n_per[2] = 0.0634905529323215;
  *allocation_maximum_tilt_rad = 0.4363323129985824;
  return allocation_per_rotor_upper_n;
}

/*
 * File trailer for gpenmpcM600Allocation.c
 *
 * [EOF]
 */
