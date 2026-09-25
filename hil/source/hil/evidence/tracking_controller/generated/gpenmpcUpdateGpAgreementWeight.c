/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcUpdateGpAgreementWeight.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcUpdateGpAgreementWeight.h"
#include "mean.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCUPDATEGPAGREEMENTWEIGHT Causal per-axis GP responsibility update.
 *
 *  Each GP axis receives a continuous score from its normalized innovation.
 *  The score is filtered with actual dt and the already established 0.50 s
 *  causal memory. No centered or future window is read.
 *
 * Arguments    : const double previousWeightF[3]
 *                boolean_T evidence_available
 *                boolean_T evidence_hard_invalid
 *                double evidence_trust
 *                const double evidence_predicted_mean_f_mps2[3]
 *                const double c_evidence_calibrated_half_widt[3]
 *                boolean_T c_evidence_observed_innovation_
 *                const double d_evidence_observed_innovation_[3]
 *                double actualDtS
 *                double nextWeightF[3]
 *                n_struct_T *diagnostic
 * Return Type  : void
 */
void gpenmpcUpdateGpAgreementWeight(
    const double previousWeightF[3], boolean_T evidence_available,
    boolean_T evidence_hard_invalid, double evidence_trust,
    const double evidence_predicted_mean_f_mps2[3],
    const double c_evidence_calibrated_half_widt[3],
    boolean_T c_evidence_observed_innovation_,
    const double d_evidence_observed_innovation_[3], double actualDtS,
    double nextWeightF[3], n_struct_T *diagnostic)
{
  double x;
  boolean_T valid;
  /*  Predeclare output fields for the MATLAB Coder fixed-shape interface. */
  diagnostic->raw_weight_f[0] = 0.0;
  diagnostic->raw_weight_f[1] = 0.0;
  diagnostic->raw_weight_f[2] = 0.0;
  if (evidence_available && c_evidence_observed_innovation_ &&
      !evidence_hard_invalid && (evidence_trust >= 0.25)) {
    valid = true;
  } else {
    valid = false;
  }
  diagnostic->normalized_innovation_error_f[0] = rtInf;
  diagnostic->normalized_innovation_error_f[1] = rtInf;
  diagnostic->normalized_innovation_error_f[2] = rtInf;
  x = exp(-actualDtS / 0.5);
  if (valid) {
    double d;
    d = fabs(d_evidence_observed_innovation_[0] -
             evidence_predicted_mean_f_mps2[0]) /
        fmax(c_evidence_calibrated_half_widt[0], 1.0E-12);
    diagnostic->normalized_innovation_error_f[0] = d;
    d = 1.0 / (d * d + 1.0);
    diagnostic->raw_weight_f[0] = d;
    nextWeightF[0] = previousWeightF[0] + (1.0 - x) * (d - previousWeightF[0]);
    d = fabs(d_evidence_observed_innovation_[1] -
             evidence_predicted_mean_f_mps2[1]) /
        fmax(c_evidence_calibrated_half_widt[1], 1.0E-12);
    diagnostic->normalized_innovation_error_f[1] = d;
    d = 1.0 / (d * d + 1.0);
    diagnostic->raw_weight_f[1] = d;
    nextWeightF[1] = previousWeightF[1] + (1.0 - x) * (d - previousWeightF[1]);
    d = fabs(d_evidence_observed_innovation_[2] -
             evidence_predicted_mean_f_mps2[2]) /
        fmax(c_evidence_calibrated_half_widt[2], 1.0E-12);
    diagnostic->normalized_innovation_error_f[2] = d;
    d = 1.0 / (d * d + 1.0);
    diagnostic->raw_weight_f[2] = d;
    nextWeightF[2] = previousWeightF[2] + (1.0 - x) * (d - previousWeightF[2]);
  } else {
    /*  Stale, low-trust, or hard-invalid evidence removes GP authority in the
     */
    /*  current update; recovery must be earned again by closed observations. */
    nextWeightF[0] = 0.0;
    nextWeightF[1] = 0.0;
    nextWeightF[2] = 0.0;
  }
  nextWeightF[0] = fmin(fmax(nextWeightF[0], 0.0), 1.0);
  nextWeightF[1] = fmin(fmax(nextWeightF[1], 0.0), 1.0);
  nextWeightF[2] = fmin(fmax(nextWeightF[2], 0.0), 1.0);
  diagnostic->valid_closed_evidence = valid;
  diagnostic->c_instantaneous_consistency_sco[0] = diagnostic->raw_weight_f[0];
  diagnostic->next_weight_f[0] = nextWeightF[0];
  diagnostic->c_instantaneous_consistency_sco[1] = diagnostic->raw_weight_f[1];
  diagnostic->next_weight_f[1] = nextWeightF[1];
  diagnostic->c_instantaneous_consistency_sco[2] = diagnostic->raw_weight_f[2];
  diagnostic->next_weight_f[2] = nextWeightF[2];
  diagnostic->aggregate_consistency_score = mean(nextWeightF);
  diagnostic->enter_eligible = (diagnostic->aggregate_consistency_score >= 0.5);
  diagnostic->exit_low = (diagnostic->aggregate_consistency_score < 0.25);
}

/*
 * File trailer for gpenmpcUpdateGpAgreementWeight.c
 *
 * [EOF]
 */
