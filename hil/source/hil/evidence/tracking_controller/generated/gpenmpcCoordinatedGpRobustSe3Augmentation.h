/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcCoordinatedGpRobustSe3Augmentation.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCCOORDINATEDGPROBUSTSE3AUGMENTATION_H
#define GPENMPCCOORDINATEDGPROBUSTSE3AUGMENTATION_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_gpenmpcCoordinatedGpRobustSe3A(
    const double plantState[19], const double reference_position_m[3],
    const double reference_velocity_mps[3], i_struct_T *state, double dt,
    boolean_T gpEvidence_available, boolean_T gpEvidence_hard_invalid,
    double gpEvidence_trust, const double gpEvidence_gp_frame_i_from_f[9],
    const double c_gpEvidence_predicted_mean_f_m[3],
    const double c_gpEvidence_calibrated_half_wi[3],
    boolean_T c_gpEvidence_observed_innovatio,
    const double d_gpEvidence_observed_innovatio[3],
    const double gpAgreementWeightF[3], double augmentation[3],
    c_struct_T *diagnostic);

void exactB1Fallback(const double plantState[19],
                     const double reference_position_m[3],
                     const double reference_velocity_mps[3], i_struct_T *state,
                     double dt, c_struct_T *diagnostic, double augmentation[3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcCoordinatedGpRobustSe3Augmentation.h
 *
 * [EOF]
 */
