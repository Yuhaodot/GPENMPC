/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcUpdateGpAgreementWeight.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCUPDATEGPAGREEMENTWEIGHT_H
#define GPENMPCUPDATEGPAGREEMENTWEIGHT_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void gpenmpcUpdateGpAgreementWeight(
    const double previousWeightF[3], boolean_T evidence_available,
    boolean_T evidence_hard_invalid, double evidence_trust,
    const double evidence_predicted_mean_f_mps2[3],
    const double c_evidence_calibrated_half_widt[3],
    boolean_T c_evidence_observed_innovation_,
    const double d_evidence_observed_innovation_[3], double actualDtS,
    double nextWeightF[3], n_struct_T *diagnostic);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcUpdateGpAgreementWeight.h
 *
 * [EOF]
 */
