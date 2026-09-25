/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcNative_canonicalReferenceTransitionFromJet.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcNative_canonicalReferenceTransitionFromJet.h"
#include "norm.h"
#include "gpenmpcFrenetFrame.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_rtwutil.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Type Definitions */
#ifndef typedef_captured_var
#define typedef_captured_var
typedef struct {
  double contents[3];
} captured_var;
#endif /* typedef_captured_var */

#ifndef typedef_b_captured_var
#define typedef_b_captured_var
typedef struct {
  double contents;
} b_captured_var;
#endif /* typedef_b_captured_var */

#ifndef typedef_c_captured_var
#define typedef_c_captured_var
typedef struct {
  double contents[12];
} c_captured_var;
#endif /* typedef_c_captured_var */

/* Function Declarations */
static double
evaluateAt(const c_captured_var *trajectoryJet,
           const b_captured_var *previousPhaseAcceleration,
           const b_captured_var *phaseDelta,
           const captured_var *previousOuterCorrectionI,
           const captured_var *outerDelta, const b_captured_var *dtS,
           const b_captured_var *progressRate, double alpha,
           double reference_position_m[3], double reference_velocity_mps[3],
           double reference_acceleration_mps2[3], double reference_jerk_mps3[3],
           double *phaseJerk, double outerCorrectionI[3]);

/* Function Definitions */
/*
 * Arguments    : const c_captured_var *trajectoryJet
 *                const b_captured_var *previousPhaseAcceleration
 *                const b_captured_var *phaseDelta
 *                const captured_var *previousOuterCorrectionI
 *                const captured_var *outerDelta
 *                const b_captured_var *dtS
 *                const b_captured_var *progressRate
 *                double alpha
 *                double reference_position_m[3]
 *                double reference_velocity_mps[3]
 *                double reference_acceleration_mps2[3]
 *                double reference_jerk_mps3[3]
 *                double *phaseJerk
 *                double outerCorrectionI[3]
 * Return Type  : double
 */
static double
evaluateAt(const c_captured_var *trajectoryJet,
           const b_captured_var *previousPhaseAcceleration,
           const b_captured_var *phaseDelta,
           const captured_var *previousOuterCorrectionI,
           const captured_var *outerDelta, const b_captured_var *dtS,
           const b_captured_var *progressRate, double alpha,
           double reference_position_m[3], double reference_velocity_mps[3],
           double reference_acceleration_mps2[3], double reference_jerk_mps3[3],
           double *phaseJerk, double outerCorrectionI[3])
{
  double b_y;
  double d;
  double d1;
  double outerJerkI_idx_0;
  double outerJerkI_idx_1;
  double outerJerkI_idx_2;
  double phaseAcceleration;
  double rate;
  double y;
  phaseAcceleration =
      previousPhaseAcceleration->contents + alpha * phaseDelta->contents;
  outerCorrectionI[0] =
      previousOuterCorrectionI->contents[0] + alpha * outerDelta->contents[0];
  outerCorrectionI[1] =
      previousOuterCorrectionI->contents[1] + alpha * outerDelta->contents[1];
  outerCorrectionI[2] =
      previousOuterCorrectionI->contents[2] + alpha * outerDelta->contents[2];
  outerJerkI_idx_2 = dtS->contents;
  if (!rtIsInf(outerJerkI_idx_2) && !rtIsNaN(outerJerkI_idx_2) &&
      (dtS->contents > 0.0)) {
    *phaseJerk = (phaseAcceleration - previousPhaseAcceleration->contents) /
                 dtS->contents;
    outerJerkI_idx_0 =
        (outerCorrectionI[0] - previousOuterCorrectionI->contents[0]) /
        outerJerkI_idx_2;
    outerJerkI_idx_1 =
        (outerCorrectionI[1] - previousOuterCorrectionI->contents[1]) /
        outerJerkI_idx_2;
    outerJerkI_idx_2 =
        (outerCorrectionI[2] - previousOuterCorrectionI->contents[2]) /
        outerJerkI_idx_2;
  } else {
    *phaseJerk = 0.0;
    outerJerkI_idx_0 = 0.0;
    outerJerkI_idx_1 = 0.0;
    outerJerkI_idx_2 = 0.0;
  }
  rate = progressRate->contents;
  y = rate * rate;
  b_y = rt_powd_snf(rate, 3.0);
  reference_position_m[0] = trajectoryJet->contents[0];
  d = trajectoryJet->contents[3];
  reference_velocity_mps[0] = d * rate;
  d1 = trajectoryJet->contents[6];
  reference_acceleration_mps2[0] =
      (d1 * y + d * phaseAcceleration) + outerCorrectionI[0];
  reference_jerk_mps3[0] = ((trajectoryJet->contents[9] * b_y +
                             3.0 * d1 * rate * phaseAcceleration) +
                            d * *phaseJerk) +
                           outerJerkI_idx_0;
  reference_position_m[1] = trajectoryJet->contents[1];
  d = trajectoryJet->contents[4];
  reference_velocity_mps[1] = d * rate;
  d1 = trajectoryJet->contents[7];
  reference_acceleration_mps2[1] =
      (d1 * y + d * phaseAcceleration) + outerCorrectionI[1];
  reference_jerk_mps3[1] = ((trajectoryJet->contents[10] * b_y +
                             3.0 * d1 * rate * phaseAcceleration) +
                            d * *phaseJerk) +
                           outerJerkI_idx_1;
  reference_position_m[2] = trajectoryJet->contents[2];
  d = trajectoryJet->contents[5];
  reference_velocity_mps[2] = d * rate;
  d1 = trajectoryJet->contents[8];
  reference_acceleration_mps2[2] =
      (d1 * y + d * phaseAcceleration) + outerCorrectionI[2];
  reference_jerk_mps3[2] = ((trajectoryJet->contents[11] * b_y +
                             3.0 * d1 * rate * phaseAcceleration) +
                            d * *phaseJerk) +
                           outerJerkI_idx_2;
  return phaseAcceleration;
}

/*
 * Jerk-bounded reference transition from a trajectory jet.
 *  Supply p/v/a/j as columns evaluated at the same current phase.
 *  The caller owns phase advancement, source validation and output publication.
 *  An invalid dt returns a zero transition fraction.
 *
 * Arguments    : const double trajectoryJet[12]
 *                double progressRate
 *                double previousPhaseAcceleration
 *                double targetPhaseAcceleration
 *                const double previousOuterCorrectionI[3]
 *                const double targetOuterCorrectionF[3]
 *                double dtS
 *                double jerkLimitMps3
 *                struct55_T *transition
 * Return Type  : void
 */
void gpenmpcNative_canonicalReferenceTransitionFromJet(
    const double trajectoryJet[12], double progressRate,
    double previousPhaseAcceleration, double targetPhaseAcceleration,
    const double previousOuterCorrectionI[3],
    const double targetOuterCorrectionF[3], double dtS, double jerkLimitMps3,
    struct55_T *transition)
{
  b_captured_var b_dtS;
  b_captured_var b_previousPhaseAcceleration;
  b_captured_var b_progressRate;
  b_captured_var phaseDelta;
  c_captured_var b_trajectoryJet;
  captured_var b_previousOuterCorrectionI;
  captured_var outerDelta;
  double base_acceleration_mps2[3];
  double base_velocity_mps[3];
  double b_y;
  double constant;
  double fraction;
  double linear;
  double y;
  int i;
  boolean_T b;
  boolean_T b1;
  memcpy(&b_trajectoryJet.contents[0], &trajectoryJet[0], 12U * sizeof(double));
  b_progressRate.contents = progressRate;
  b_previousPhaseAcceleration.contents = previousPhaseAcceleration;
  b_dtS.contents = dtS;
  y = progressRate * progressRate;
  b_previousOuterCorrectionI.contents[0] = previousOuterCorrectionI[0];
  base_velocity_mps[0] = trajectoryJet[3] * progressRate;
  base_acceleration_mps2[0] = trajectoryJet[6] * y + trajectoryJet[3] * 0.0;
  b_previousOuterCorrectionI.contents[1] = previousOuterCorrectionI[1];
  base_velocity_mps[1] = trajectoryJet[4] * progressRate;
  base_acceleration_mps2[1] = trajectoryJet[7] * y + trajectoryJet[4] * 0.0;
  b_previousOuterCorrectionI.contents[2] = previousOuterCorrectionI[2];
  base_velocity_mps[2] = trajectoryJet[5] * progressRate;
  gpenmpcFrenetFrame(base_velocity_mps, base_acceleration_mps2,
                    transition->frame_i_from_f, &fraction);
  phaseDelta.contents = targetPhaseAcceleration - previousPhaseAcceleration;
  fraction = targetOuterCorrectionF[0];
  linear = targetOuterCorrectionF[1];
  constant = targetOuterCorrectionF[2];
  for (i = 0; i < 3; i++) {
    outerDelta.contents[i] = ((transition->frame_i_from_f[i] * fraction +
                               transition->frame_i_from_f[i + 3] * linear) +
                              transition->frame_i_from_f[i + 6] * constant) -
                             previousOuterCorrectionI[i];
  }
  b = rtIsInf(dtS);
  b1 = rtIsNaN(dtS);
  if (b || b1 || (dtS <= 0.0)) {
    fraction = 0.0;
  } else {
    double base_jerk_mps3[3];
    double base_position_m[3];
    double referenceOne_jerk_mps3[3];
    evaluateAt(&b_trajectoryJet, &b_previousPhaseAcceleration, &phaseDelta,
               &b_previousOuterCorrectionI, &outerDelta, &b_dtS,
               &b_progressRate, 0.0, base_position_m, base_velocity_mps,
               base_acceleration_mps2, base_jerk_mps3, &fraction,
               transition->outer_correction_i_mps2);
    evaluateAt(&b_trajectoryJet, &b_previousPhaseAcceleration, &phaseDelta,
               &b_previousOuterCorrectionI, &outerDelta, &b_dtS,
               &b_progressRate, 1.0, base_velocity_mps, base_acceleration_mps2,
               base_position_m, referenceOne_jerk_mps3, &fraction,
               transition->outer_correction_i_mps2);
    transition->outer_correction_i_mps2[0] =
        referenceOne_jerk_mps3[0] - base_jerk_mps3[0];
    transition->outer_correction_i_mps2[1] =
        referenceOne_jerk_mps3[1] - base_jerk_mps3[1];
    transition->outer_correction_i_mps2[2] =
        referenceOne_jerk_mps3[2] - base_jerk_mps3[2];
    if (c_norm(referenceOne_jerk_mps3) <= jerkLimitMps3 + 1.0E-12) {
      fraction = 1.0;
    } else if (c_norm(base_jerk_mps3) > jerkLimitMps3 + 1.0E-12) {
      fraction = 0.0;
    } else {
      transition->outer_correction_jerk_i_mps3[0] =
          transition->outer_correction_i_mps2[0] *
          transition->outer_correction_i_mps2[0];
      transition->outer_correction_jerk_i_mps3[1] =
          transition->outer_correction_i_mps2[1] *
          transition->outer_correction_i_mps2[1];
      transition->outer_correction_jerk_i_mps3[2] =
          transition->outer_correction_i_mps2[2] *
          transition->outer_correction_i_mps2[2];
      fraction = (transition->outer_correction_jerk_i_mps3[0] +
                  transition->outer_correction_jerk_i_mps3[1]) +
                 transition->outer_correction_jerk_i_mps3[2];
      transition->outer_correction_i_mps2[0] *= base_jerk_mps3[0];
      transition->outer_correction_i_mps2[1] *= base_jerk_mps3[1];
      transition->outer_correction_i_mps2[2] *= base_jerk_mps3[2];
      linear = 2.0 * ((transition->outer_correction_i_mps2[0] +
                       transition->outer_correction_i_mps2[1]) +
                      transition->outer_correction_i_mps2[2]);
      transition->outer_correction_jerk_i_mps3[0] =
          base_jerk_mps3[0] * base_jerk_mps3[0];
      transition->outer_correction_jerk_i_mps3[1] =
          base_jerk_mps3[1] * base_jerk_mps3[1];
      transition->outer_correction_jerk_i_mps3[2] =
          base_jerk_mps3[2] * base_jerk_mps3[2];
      constant = ((transition->outer_correction_jerk_i_mps3[0] +
                   transition->outer_correction_jerk_i_mps3[1]) +
                  transition->outer_correction_jerk_i_mps3[2]) -
                 jerkLimitMps3 * jerkLimitMps3;
      if (fraction <= 1.0E-24) {
        if (linear <= 0.0) {
          fraction = 0.0;
        } else {
          fraction = -constant / linear;
        }
      } else {
        fraction =
            (-linear +
             sqrt(fmax(0.0, linear * linear - 4.0 * fraction * constant))) /
            (2.0 * fraction);
      }
      fraction = fmin(fmax(fraction, 0.0), 1.0);
    }
  }
  linear = previousPhaseAcceleration + fraction * phaseDelta.contents;
  transition->outer_correction_i_mps2[0] =
      previousOuterCorrectionI[0] + fraction * outerDelta.contents[0];
  transition->outer_correction_i_mps2[1] =
      previousOuterCorrectionI[1] + fraction * outerDelta.contents[1];
  transition->outer_correction_i_mps2[2] =
      previousOuterCorrectionI[2] + fraction * outerDelta.contents[2];
  if (!b && !b1 && (dtS > 0.0)) {
    constant = (linear - previousPhaseAcceleration) / dtS;
    transition->outer_correction_jerk_i_mps3[0] =
        (transition->outer_correction_i_mps2[0] - previousOuterCorrectionI[0]) /
        dtS;
    transition->outer_correction_jerk_i_mps3[1] =
        (transition->outer_correction_i_mps2[1] - previousOuterCorrectionI[1]) /
        dtS;
    transition->outer_correction_jerk_i_mps3[2] =
        (transition->outer_correction_i_mps2[2] - previousOuterCorrectionI[2]) /
        dtS;
  } else {
    constant = 0.0;
    transition->outer_correction_jerk_i_mps3[0] = 0.0;
    transition->outer_correction_jerk_i_mps3[1] = 0.0;
    transition->outer_correction_jerk_i_mps3[2] = 0.0;
  }
  b_y = rt_powd_snf(progressRate, 3.0);
  transition->reference.position_m[0] = b_trajectoryJet.contents[0];
  transition->reference.velocity_mps[0] =
      b_trajectoryJet.contents[3] * progressRate;
  transition->reference.acceleration_mps2[0] =
      (b_trajectoryJet.contents[6] * y + b_trajectoryJet.contents[3] * linear) +
      transition->outer_correction_i_mps2[0];
  transition->reference.jerk_mps3[0] =
      ((b_trajectoryJet.contents[9] * b_y +
        3.0 * b_trajectoryJet.contents[6] * progressRate * linear) +
       b_trajectoryJet.contents[3] * constant) +
      transition->outer_correction_jerk_i_mps3[0];
  transition->reference.position_m[1] = b_trajectoryJet.contents[1];
  transition->reference.velocity_mps[1] =
      b_trajectoryJet.contents[4] * progressRate;
  transition->reference.acceleration_mps2[1] =
      (b_trajectoryJet.contents[7] * y + b_trajectoryJet.contents[4] * linear) +
      transition->outer_correction_i_mps2[1];
  transition->reference.jerk_mps3[1] =
      ((b_trajectoryJet.contents[10] * b_y +
        3.0 * b_trajectoryJet.contents[7] * progressRate * linear) +
       b_trajectoryJet.contents[4] * constant) +
      transition->outer_correction_jerk_i_mps3[1];
  transition->reference.position_m[2] = b_trajectoryJet.contents[2];
  transition->reference.velocity_mps[2] =
      b_trajectoryJet.contents[5] * progressRate;
  transition->reference.acceleration_mps2[2] =
      (b_trajectoryJet.contents[8] * y + b_trajectoryJet.contents[5] * linear) +
      transition->outer_correction_i_mps2[2];
  transition->reference.jerk_mps3[2] =
      ((b_trajectoryJet.contents[11] * b_y +
        3.0 * b_trajectoryJet.contents[8] * progressRate * linear) +
       b_trajectoryJet.contents[5] * constant) +
      transition->outer_correction_jerk_i_mps3[2];
  transition->reference_curvature =
      gpenmpcFrenetFrame(transition->reference.velocity_mps,
                        transition->reference.acceleration_mps2,
                        transition->reference_frame_i_from_f,
                        &transition->reference_signed_yaw_rate);
  transition->phase_acceleration_s_inv = linear;
  transition->phase_jerk_s_inv2 = constant;
  transition->fraction = fraction;
}

/*
 * File trailer for gpenmpcNative_canonicalReferenceTransitionFromJet.c
 *
 * [EOF]
 */
