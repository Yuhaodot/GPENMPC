/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: prepareCanonicalCurrentGpPrediction.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "prepareCanonicalCurrentGpPrediction.h"
#include "atan2.h"
#include "norm.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_rtwutil.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * PREPARECANONICALCURRENTGPPREDICTION Exact local half of the canonical query.
 *  Numerical partition of gpenmpcPredictCurrentGpEvidence, not a transport,
 *  scheduling, source-association, or control-authority interface. The caller
 *  obtains gpModelAvailable from its verified model binding. No GP model is
 *  copied here. A required query is exactly:
 *    prediction = gpenmpcSparseGpPredict(model, prepared.features_f17);
 *  The prediction remains open until the original k+1 innovation closure.
 *
 * Arguments    : const double velocityI[3]
 *                const double reference_velocity_mps[3]
 *                const double reference_acceleration_mps2[3]
 *                const double windEstimateXY[2]
 *                double payloadKg
 *                const double desiredForceI[3]
 *                const double previousRotorCommandN[6]
 *                const double residualHistoryF[3]
 *                boolean_T causalValid
 *                char prepared_schema_Value[39]
 *                e_struct_T *prepared_pending
 *                double prepared_features_f17[17]
 *                double *prepared_gp_mean_scale
 * Return Type  : boolean_T
 */
boolean_T c_prepareCanonicalCurrentGpPred(
    const double velocityI[3], const double reference_velocity_mps[3],
    const double reference_acceleration_mps2[3], const double windEstimateXY[2],
    double payloadKg, const double desiredForceI[3],
    const double previousRotorCommandN[6], const double residualHistoryF[3],
    boolean_T causalValid, char prepared_schema_Value[39],
    e_struct_T *prepared_pending, double prepared_features_f17[17],
    double *prepared_gp_mean_scale)
{
  static const char b_cv[39] = {
      'C', 'A', 'N', 'O', 'N', 'I', 'C', 'A', 'L', '_', 'C', 'U', 'R',
      'R', 'E', 'N', 'T', '_', 'G', 'P', '_', 'N', 'U', 'M', 'E', 'R',
      'I', 'C', 'A', 'L', '_', 'S', 'P', 'L', 'I', 'T', '_', 'V', '1'};
  double history[4];
  double airF[3];
  double referenceAccelerationF[3];
  double curvature;
  double horizontalSpeed;
  double tangent_idx_0;
  double tangent_idx_1;
  int i;
  boolean_T prepared_prediction_required;
  for (i = 0; i < 39; i++) {
    prepared_schema_Value[i] = b_cv[i];
  }
  prepared_pending->causal_valid = causalValid;
  /* GPENMPCFRENETFRAME Build the inherited horizontal Frenet frame. */
  horizontalSpeed = b_norm(&reference_velocity_mps[0]);
  if (horizontalSpeed < 1.0E-9) {
    tangent_idx_0 = 1.0;
    tangent_idx_1 = 0.0;
  } else {
    tangent_idx_0 = reference_velocity_mps[0] / horizontalSpeed;
    tangent_idx_1 = reference_velocity_mps[1] / horizontalSpeed;
  }
  prepared_pending->gp_frame_i_from_f[0] = tangent_idx_0;
  prepared_pending->gp_frame_i_from_f[3] = -tangent_idx_1;
  prepared_pending->gp_frame_i_from_f[6] = 0.0;
  prepared_pending->gp_frame_i_from_f[1] = tangent_idx_1;
  prepared_pending->gp_frame_i_from_f[4] = tangent_idx_0;
  prepared_pending->gp_frame_i_from_f[7] = 0.0;
  prepared_pending->gp_frame_i_from_f[2] = 0.0;
  prepared_pending->gp_frame_i_from_f[5] = 0.0;
  prepared_pending->gp_frame_i_from_f[8] = 1.0;
  curvature = 0.0;
  if (horizontalSpeed >= 0.75) {
    curvature =
        fabs(reference_velocity_mps[0] * reference_acceleration_mps2[1] -
             reference_acceleration_mps2[0] * reference_velocity_mps[1]) /
        rt_powd_snf(horizontalSpeed, 3.0);
  }
  /*  Field order, types, initial NaNs, and flags match the canonical function.
   */
  prepared_prediction_required = false;
  for (i = 0; i < 17; i++) {
    prepared_features_f17[i] = rtNaN;
  }
  *prepared_gp_mean_scale = rtNaN;
  if (causalValid) {
    double airF_tmp[9];
    double d;
    double d1;
    double d2;
    double d3;
    double d4;
    double d5;
    /* GPENMPCNORMALIZEDTOTALROTORCOMMAND Shared causal F17 command feature. */
    /*  */
    /*  The feature is the previous post-allocation six-rotor command sum
     * divided */
    /*  by nominal weight.  Training extraction, online execution and predictive
     */
    /*  rollout call this helper so actuator lag or saturation cannot silently
     */
    /*  change the feature definition between those paths. */
    tangent_idx_0 = previousRotorCommandN[0];
    for (i = 0; i < 5; i++) {
      tangent_idx_0 += previousRotorCommandN[i + 1];
    }
    history[0] = fmin(
        fmax(tangent_idx_0 / fmax((payloadKg + 9.5) * 9.80665, 1.0E-12), 0.0),
        2.0);
    /* GPENMPCBUILDAEROF17FEATURES Build the shared causal AERO_PHYSICS_F17 row.
     */
    /*  */
    /*  Shared feature construction for the B2 rollout and read-only B1 shadow query. */
    referenceAccelerationF[0] = velocityI[0] - windEstimateXY[0];
    referenceAccelerationF[1] = velocityI[1] - windEstimateXY[1];
    referenceAccelerationF[2] = velocityI[2];
    memset(&airF[0], 0, 3U * sizeof(double));
    d = airF[0];
    d1 = airF[1];
    d2 = airF[2];
    for (i = 0; i < 3; i++) {
      int b_i;
      history[i + 1] = residualHistoryF[i];
      tangent_idx_0 = prepared_pending->gp_frame_i_from_f[i];
      airF_tmp[3 * i] = tangent_idx_0;
      tangent_idx_1 = prepared_pending->gp_frame_i_from_f[i + 3];
      airF_tmp[3 * i + 1] = tangent_idx_1;
      b_i = (int)prepared_pending->gp_frame_i_from_f[i + 6];
      airF_tmp[3 * i + 2] = b_i;
      horizontalSpeed = referenceAccelerationF[i];
      d += tangent_idx_0 * horizontalSpeed;
      d1 += tangent_idx_1 * horizontalSpeed;
      d2 += (double)b_i * horizontalSpeed;
    }
    airF[2] = d2;
    airF[1] = d1;
    airF[0] = d;
    memset(&referenceAccelerationF[0], 0, 3U * sizeof(double));
    d3 = referenceAccelerationF[0];
    d4 = referenceAccelerationF[1];
    d5 = referenceAccelerationF[2];
    for (i = 0; i < 3; i++) {
      tangent_idx_0 = reference_acceleration_mps2[i];
      d3 += airF_tmp[3 * i] * tangent_idx_0;
      d4 += airF_tmp[3 * i + 1] * tangent_idx_0;
      d5 += airF_tmp[3 * i + 2] * tangent_idx_0;
    }
    double dragT_tmp;
    tangent_idx_1 =
        b_atan2(b_norm(&desiredForceI[0]), fmax(desiredForceI[2], 1.0E-9));
    horizontalSpeed = payloadKg / 4.54;
    tangent_idx_0 = sin(tangent_idx_1);
    dragT_tmp = fabs(d);
    prepared_prediction_required = true;
    prepared_features_f17[0] = horizontalSpeed;
    prepared_features_f17[4] = c_norm(airF);
    tangent_idx_0 =
        horizontalSpeed * (0.35 * (tangent_idx_0 * tangent_idx_0) + 1.0);
    prepared_features_f17[5] = tangent_idx_0 * d * dragT_tmp;
    prepared_features_f17[6] = tangent_idx_0 * d1 * fabs(d1);
    prepared_features_f17[7] = curvature * d * dragT_tmp + d1 * dragT_tmp;
    prepared_features_f17[11] = tangent_idx_1;
    prepared_features_f17[12] = history[0];
    prepared_features_f17[13] =
        history[0] * (horizontalSpeed + 1.0) * fmax(0.0, d5 / 9.80665 + 1.0);
    prepared_features_f17[1] = d;
    prepared_features_f17[8] = d3;
    prepared_features_f17[14] = history[1];
    prepared_features_f17[2] = d1;
    prepared_features_f17[9] = d4;
    prepared_features_f17[15] = history[2];
    prepared_features_f17[3] = d2;
    prepared_features_f17[10] = d5;
    prepared_features_f17[16] = history[3];
    *prepared_gp_mean_scale = 1.0;
  }
  return prepared_prediction_required;
}

/*
 * File trailer for prepareCanonicalCurrentGpPrediction.c
 *
 * [EOF]
 */
