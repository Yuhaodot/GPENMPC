/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcComputeGpResponsibility.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcComputeGpResponsibility.h"
#include "all.h"
#include "mean.h"
#include "minOrMax.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCCOMPUTEGPRESPONSIBILITY One causal, axis-wise GP authority rule.
 *
 *  This pure function is shared by the outer decision path, the eNMPC
 *  rollout and the runtime residual composition.  No future value, method
 *  outcome or plant-private signal is consumed.
 *
 * Arguments    : double responsibilityBlend
 *                double trust
 *                const double agreementF[3]
 *                const double meanF[3]
 *                boolean_T *c_responsibility_exact_b1_requi
 *                char c_responsibility_reason_Value_d[]
 *                int c_responsibility_reason_Value_s[2]
 *                double *responsibility_blend
 *                double *responsibility_trust
 *                double responsibility_agreement_f[3]
 *                double c_responsibility_alpha_effectiv[3]
 *                double *d_responsibility_alpha_effectiv
 *                double responsibility_mean_f_mps2[3]
 *                double c_responsibility_weighted_mean_[3]
 *                double *c_responsibility_future_samples
 * Return Type  : boolean_T
 */
boolean_T c_gpenmpcComputeGpResponsibility(
    double responsibilityBlend, double trust, const double agreementF[3],
    const double meanF[3], boolean_T *c_responsibility_exact_b1_requi,
    char c_responsibility_reason_Value_d[],
    int c_responsibility_reason_Value_s[2], double *responsibility_blend,
    double *responsibility_trust, double responsibility_agreement_f[3],
    double c_responsibility_alpha_effectiv[3],
    double *d_responsibility_alpha_effectiv,
    double responsibility_mean_f_mps2[3],
    double c_responsibility_weighted_mean_[3],
    double *c_responsibility_future_samples)
{
  static const char b_cv2[24] = {'Z', 'E', 'R', 'O', '_', 'E', 'F', 'F',
                                 'E', 'C', 'T', 'I', 'V', 'E', '_', 'A',
                                 'U', 'T', 'H', 'O', 'R', 'I', 'T', 'Y'};
  static const char b_cv1[19] = {'T', 'R', 'U', 'S', 'T', '_', 'B',
                                 'E', 'L', 'O', 'W', '_', 'M', 'I',
                                 'N', 'I', 'M', 'U', 'M'};
  static const char cv4[19] = {'Z', 'E', 'R', 'O', '_', 'E', 'F', 'F', 'E', 'C',
                               'T', 'I', 'V', 'E', '_', 'M', 'E', 'A', 'N'};
  static const char b_cv[18] = {'N', 'O', 'N', 'F', 'I', 'N', 'I', 'T', 'E',
                                '_', 'G', 'P', '_', 'S', 'T', 'A', 'T', 'E'};
  static const char cv3[16] = {'A', 'C', 'T', 'I', 'V', 'E', '_', 'C',
                               'A', 'U', 'S', 'A', 'L', '_', 'G', 'P'};
  int i;
  boolean_T exactB1;
  boolean_T finiteInputs;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T guard3;
  boolean_T responsibility_valid;
  guard1 = false;
  guard2 = false;
  guard3 = false;
  if (!rtIsInf(responsibilityBlend) && !rtIsNaN(responsibilityBlend) &&
      (!rtIsInf(trust) && !rtIsNaN(trust))) {
    boolean_T bv[3];
    bv[0] = (!rtIsInf(agreementF[0]) && !rtIsNaN(agreementF[0]));
    bv[1] = (!rtIsInf(agreementF[1]) && !rtIsNaN(agreementF[1]));
    bv[2] = (!rtIsInf(agreementF[2]) && !rtIsNaN(agreementF[2]));
    if (all(bv)) {
      bv[0] = (!rtIsInf(meanF[0]) && !rtIsNaN(meanF[0]));
      bv[1] = (!rtIsInf(meanF[1]) && !rtIsNaN(meanF[1]));
      bv[2] = (!rtIsInf(meanF[2]) && !rtIsNaN(meanF[2]));
      if (all(bv)) {
        finiteInputs = true;
        if (trust >= 0.25) {
          double d;
          double d1;
          responsibility_valid = true;
          d = responsibilityBlend * trust;
          d1 = fmin(fmax(d * agreementF[0], 0.0), 1.0);
          c_responsibility_alpha_effectiv[0] = d1;
          c_responsibility_weighted_mean_[0] = d1 * meanF[0];
          d1 = fmin(fmax(d * agreementF[1], 0.0), 1.0);
          c_responsibility_alpha_effectiv[1] = d1;
          c_responsibility_weighted_mean_[1] = d1 * meanF[1];
          d1 = fmin(fmax(d * agreementF[2], 0.0), 1.0);
          c_responsibility_alpha_effectiv[2] = d1;
          c_responsibility_weighted_mean_[2] = d1 * meanF[2];
          if ((b_maximum(c_responsibility_alpha_effectiv) <= 1.0E-12) ||
              (c_norm(c_responsibility_weighted_mean_) <= 1.0E-12)) {
            guard1 = true;
          } else {
            exactB1 = false;
          }
        } else {
          guard2 = true;
        }
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }
  } else {
    guard3 = true;
  }
  if (guard3) {
    finiteInputs = false;
    guard2 = true;
  }
  if (guard2) {
    responsibility_valid = false;
    guard1 = true;
  }
  if (guard1) {
    exactB1 = true;
    c_responsibility_alpha_effectiv[0] = 0.0;
    c_responsibility_weighted_mean_[0] = 0.0;
    c_responsibility_alpha_effectiv[1] = 0.0;
    c_responsibility_weighted_mean_[1] = 0.0;
    c_responsibility_alpha_effectiv[2] = 0.0;
    c_responsibility_weighted_mean_[2] = 0.0;
  }
  if (!finiteInputs) {
    c_responsibility_reason_Value_s[0] = 1;
    c_responsibility_reason_Value_s[1] = 18;
    for (i = 0; i < 18; i++) {
      c_responsibility_reason_Value_d[i] = b_cv[i];
    }
  } else if (trust < 0.25) {
    c_responsibility_reason_Value_s[0] = 1;
    c_responsibility_reason_Value_s[1] = 19;
    for (i = 0; i < 19; i++) {
      c_responsibility_reason_Value_d[i] = b_cv1[i];
    }
  } else if (b_maximum(c_responsibility_alpha_effectiv) <= 1.0E-12) {
    c_responsibility_reason_Value_s[0] = 1;
    c_responsibility_reason_Value_s[1] = 24;
    for (i = 0; i < 24; i++) {
      c_responsibility_reason_Value_d[i] = b_cv2[i];
    }
  } else if (c_norm(c_responsibility_weighted_mean_) <= 1.0E-12) {
    c_responsibility_reason_Value_s[0] = 1;
    c_responsibility_reason_Value_s[1] = 19;
    for (i = 0; i < 19; i++) {
      c_responsibility_reason_Value_d[i] = cv4[i];
    }
  } else {
    c_responsibility_reason_Value_s[0] = 1;
    c_responsibility_reason_Value_s[1] = 16;
    for (i = 0; i < 16; i++) {
      c_responsibility_reason_Value_d[i] = cv3[i];
    }
  }
  *c_responsibility_exact_b1_requi = exactB1;
  *responsibility_blend = fmin(fmax(responsibilityBlend, 0.0), 1.0);
  *responsibility_trust = trust;
  *d_responsibility_alpha_effectiv = mean(c_responsibility_alpha_effectiv);
  responsibility_agreement_f[0] = agreementF[0];
  responsibility_mean_f_mps2[0] = meanF[0];
  responsibility_agreement_f[1] = agreementF[1];
  responsibility_mean_f_mps2[1] = meanF[1];
  responsibility_agreement_f[2] = agreementF[2];
  responsibility_mean_f_mps2[2] = meanF[2];
  *c_responsibility_future_samples = 0.0;
  return responsibility_valid;
}

/*
 * File trailer for gpenmpcComputeGpResponsibility.c
 *
 * [EOF]
 */
