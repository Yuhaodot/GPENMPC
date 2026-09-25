/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcCloseGpInnovationEvidence.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcCloseGpInnovationEvidence.h"
#include "all.h"
#include "mean.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCCLOSEGPINNOVATIONEVIDENCE Pair k prediction with the k+1 label.
 *
 * Arguments    : const m_struct_T *pending
 *                const double observedInnovationI[3]
 *                boolean_T observedAvailable
 *                m_struct_T *evidence
 * Return Type  : void
 */
void c_gpenmpcCloseGpInnovationEvidence(const m_struct_T *pending,
                                     const double observedInnovationI[3],
                                     boolean_T observedAvailable,
                                     m_struct_T *evidence)
{
  static const char b_cv[44] = {
      'G', 'P', 'E', 'N', 'M', 'P', 'C', '_', 'C', 'L', 'O', 'S', 'E', 'D', '_',
      'S', 'A', 'M', 'E', '_', 'S', 'A', 'M', 'P', 'L', 'E', '_', 'G', 'P', '_',
      'E', 'V', 'I', 'D', 'E', 'N', 'C', 'E', '_', 'V', '1', '\0', '\0', '\0'};
  int k;
  *evidence = *pending;
  for (k = 0; k < 44; k++) {
    evidence->schema.Value.data[k] = b_cv[k];
  }
  if (observedAvailable) {
    boolean_T bv[3];
    bv[0] =
        (!rtIsInf(observedInnovationI[0]) && !rtIsNaN(observedInnovationI[0]));
    bv[1] =
        (!rtIsInf(observedInnovationI[1]) && !rtIsNaN(observedInnovationI[1]));
    bv[2] =
        (!rtIsInf(observedInnovationI[2]) && !rtIsNaN(observedInnovationI[2]));
    if (all(bv)) {
      evidence->observed_innovation_available = true;
    } else {
      evidence->observed_innovation_available = false;
    }
  } else {
    evidence->observed_innovation_available = false;
  }
  if (!pending->available || !evidence->observed_innovation_available) {
    evidence->observed_innovation_consistent = false;
  } else {
    double d;
    double d1;
    double d2;
    d = observedInnovationI[0];
    d1 = observedInnovationI[1];
    d2 = observedInnovationI[2];
    for (k = 0; k < 3; k++) {
      double d3;
      d3 = (pending->gp_frame_i_from_f[3 * k] * d +
            pending->gp_frame_i_from_f[3 * k + 1] * d1) +
           pending->gp_frame_i_from_f[3 * k + 2] * d2;
      evidence->observed_innovation_f_mps2[k] = d3;
      d3 -= pending->predicted_mean_f_mps2[k];
      evidence->innovation_error_f_mps2[k] = d3;
      d3 = fabs(d3) / fmax(pending->calibrated_half_width_f_mps2[k], 1.0E-12);
      evidence->innovation_consistency_score_f[k] = 1.0 / (d3 * d3 + 1.0);
      evidence->c_observed_innovation_consisten[k] = (d3 <= 1.000000000001);
    }
    /*  Retained as a descriptive compatibility field only.  Supervisor logic */
    /*  consumes the causal EWMA score attached by
     * gpenmpcUpdateGpAgreementWeight. */
    evidence->observed_innovation_consistent =
        (mean(evidence->innovation_consistency_score_f) >= 0.5);
  }
}

/*
 * File trailer for gpenmpcCloseGpInnovationEvidence.c
 *
 * [EOF]
 */
