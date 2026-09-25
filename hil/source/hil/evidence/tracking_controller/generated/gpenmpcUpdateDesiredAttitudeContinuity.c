/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcUpdateDesiredAttitudeContinuity.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcUpdateDesiredAttitudeContinuity.h"
#include "det.h"
#include "eye.h"
#include "norm.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_data.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_rtwutil.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "gpenmpcSo3LogVee.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCUPDATEDESIREDATTITUDECONTINUITY Causal geodesic SO(3) filtering.
 *
 *  Only the current raw desired rotation, past filter state and actual elapsed
 *  time are used. A reset adopts the current raw rotation without injecting
 *  stale angular velocity or angular acceleration.
 *
 * Arguments    : e_gpenmpcNative_canonicalLocalIn *SD
 *                boolean_T state_initialized
 *                boolean_T state_angular_velocity_valid
 *                const double state_filtered_rotation[9]
 *                const double c_state_desired_angular_velocit[3]
 *                const double c_state_desired_angular_acceler[3]
 *                double state_update_count
 *                double state_reset_count
 *                const double rawDesiredRotation[9]
 *                double actualDtS
 *                boolean_T resetRequested
 *                k_struct_T *command
 *                o_struct_T *nextState
 * Return Type  : void
 */
void c_gpenmpcUpdateDesiredAttitudeCo(
    e_gpenmpcNative_canonicalLocalIn *SD, boolean_T state_initialized,
    boolean_T state_angular_velocity_valid,
    const double state_filtered_rotation[9],
    const double c_state_desired_angular_velocit[3],
    const double c_state_desired_angular_acceler[3], double state_update_count,
    double state_reset_count, const double rawDesiredRotation[9],
    double actualDtS, boolean_T resetRequested, k_struct_T *command,
    o_struct_T *nextState)
{
  double b_y_tmp[9];
  double correction[9];
  double previousRotation[9];
  double y_tmp[9];
  double angle;
  double lengthValue;
  int a__1_tmp;
  int b_a__1_tmp;
  int i;
  int i1;
  int i2;
  boolean_T accelerationValid;
  boolean_T resetActive;
  boolean_T timeDiscontinuity;
  boolean_T velocityValid;
  timeDiscontinuity = (rtIsInf(actualDtS) || rtIsNaN(actualDtS));
  if (resetRequested || timeDiscontinuity || !state_initialized) {
    resetActive = true;
  } else {
    resetActive = false;
  }
  svd(rawDesiredRotation, SD->u2.f2.left, SD->u2.f2.a__1, SD->u2.f2.right);
  for (i = 0; i < 9; i++) {
    correction[i] = iv[i];
  }
  memset(&SD->u2.f2.a__1[0], 0, 9U * sizeof(double));
  for (i = 0; i < 3; i++) {
    angle = SD->u2.f2.a__1[3 * i];
    a__1_tmp = 3 * i + 1;
    b_a__1_tmp = 3 * i + 2;
    for (i1 = 0; i1 < 3; i1++) {
      lengthValue = SD->u2.f2.right[i + 3 * i1];
      y_tmp[i1 + 3 * i] = lengthValue;
      angle += SD->u2.f2.left[3 * i1] * lengthValue;
      SD->u2.f2.a__1[a__1_tmp] += SD->u2.f2.left[3 * i1 + 1] * lengthValue;
      SD->u2.f2.a__1[b_a__1_tmp] += SD->u2.f2.left[3 * i1 + 2] * lengthValue;
    }
    SD->u2.f2.a__1[3 * i] = angle;
  }
  correction[8] = det(SD->u2.f2.a__1);
  if (rtIsNaN(correction[8])) {
    correction[8] = rtNaN;
  } else if (correction[8] < 0.0) {
    correction[8] = -1.0;
  } else {
    correction[8] = (correction[8] > 0.0);
  }
  if (correction[8] == 0.0) {
    correction[8] = 1.0;
  }
  memset(&SD->u2.f2.a__1[0], 0, 9U * sizeof(double));
  for (i = 0; i < 3; i++) {
    angle = SD->u2.f2.a__1[3 * i];
    a__1_tmp = 3 * i + 1;
    b_a__1_tmp = 3 * i + 2;
    for (i1 = 0; i1 < 3; i1++) {
      i2 = i1 + 3 * i;
      lengthValue = correction[i2];
      angle += SD->u2.f2.left[3 * i1] * lengthValue;
      SD->u2.f2.a__1[a__1_tmp] += SD->u2.f2.left[3 * i1 + 1] * lengthValue;
      SD->u2.f2.a__1[b_a__1_tmp] += SD->u2.f2.left[3 * i1 + 2] * lengthValue;
      command->desired_rotation[i2] = 0.0;
    }
    SD->u2.f2.a__1[3 * i] = angle;
  }
  for (i = 0; i < 3; i++) {
    angle = command->desired_rotation[3 * i];
    a__1_tmp = 3 * i + 1;
    b_a__1_tmp = 3 * i + 2;
    for (i1 = 0; i1 < 3; i1++) {
      lengthValue = y_tmp[i1 + 3 * i];
      angle += SD->u2.f2.a__1[3 * i1] * lengthValue;
      command->desired_rotation[a__1_tmp] +=
          SD->u2.f2.a__1[3 * i1 + 1] * lengthValue;
      command->desired_rotation[b_a__1_tmp] +=
          SD->u2.f2.a__1[3 * i1 + 2] * lengthValue;
    }
    command->desired_rotation[3 * i] = angle;
  }
  if (resetActive) {
    command->c_desired_angular_velocity_body[0] = 0.0;
    command->c_desired_angular_acceleration_[0] = 0.0;
    command->c_desired_angular_velocity_body[1] = 0.0;
    command->c_desired_angular_acceleration_[1] = 0.0;
    command->c_desired_angular_velocity_body[2] = 0.0;
    command->c_desired_angular_acceleration_[2] = 0.0;
    velocityValid = false;
    accelerationValid = false;
  } else {
    double a;
    double geodesicGain_tmp;
    double y;
    svd(state_filtered_rotation, SD->u2.f2.left, SD->u2.f2.a__1,
        SD->u2.f2.right);
    for (i = 0; i < 9; i++) {
      correction[i] = iv[i];
    }
    memset(&SD->u2.f2.a__1[0], 0, 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
      angle = SD->u2.f2.a__1[3 * i];
      a__1_tmp = 3 * i + 1;
      b_a__1_tmp = 3 * i + 2;
      for (i1 = 0; i1 < 3; i1++) {
        lengthValue = SD->u2.f2.right[i + 3 * i1];
        y_tmp[i1 + 3 * i] = lengthValue;
        angle += SD->u2.f2.left[3 * i1] * lengthValue;
        SD->u2.f2.a__1[a__1_tmp] += SD->u2.f2.left[3 * i1 + 1] * lengthValue;
        SD->u2.f2.a__1[b_a__1_tmp] += SD->u2.f2.left[3 * i1 + 2] * lengthValue;
      }
      SD->u2.f2.a__1[3 * i] = angle;
    }
    correction[8] = det(SD->u2.f2.a__1);
    if (rtIsNaN(correction[8])) {
      correction[8] = rtNaN;
    } else if (correction[8] < 0.0) {
      correction[8] = -1.0;
    } else {
      correction[8] = (correction[8] > 0.0);
    }
    if (correction[8] == 0.0) {
      correction[8] = 1.0;
    }
    memset(&SD->u2.f2.a__1[0], 0, 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
      angle = SD->u2.f2.a__1[3 * i];
      a__1_tmp = 3 * i + 1;
      b_a__1_tmp = 3 * i + 2;
      for (i1 = 0; i1 < 3; i1++) {
        lengthValue = correction[i1 + 3 * i];
        angle += SD->u2.f2.left[3 * i1] * lengthValue;
        SD->u2.f2.a__1[a__1_tmp] += SD->u2.f2.left[3 * i1 + 1] * lengthValue;
        SD->u2.f2.a__1[b_a__1_tmp] += SD->u2.f2.left[3 * i1 + 2] * lengthValue;
      }
      SD->u2.f2.a__1[3 * i] = angle;
    }
    memset(&previousRotation[0], 0, 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
      angle = previousRotation[3 * i];
      a__1_tmp = 3 * i + 1;
      b_a__1_tmp = 3 * i + 2;
      for (i1 = 0; i1 < 3; i1++) {
        lengthValue = y_tmp[i1 + 3 * i];
        angle += SD->u2.f2.a__1[3 * i1] * lengthValue;
        previousRotation[a__1_tmp] += SD->u2.f2.a__1[3 * i1 + 1] * lengthValue;
        previousRotation[b_a__1_tmp] +=
            SD->u2.f2.a__1[3 * i1 + 2] * lengthValue;
      }
      previousRotation[3 * i] = angle;
    }
    for (i = 0; i < 3; i++) {
      y_tmp[3 * i] = previousRotation[i];
      y_tmp[3 * i + 1] = previousRotation[i + 3];
      y_tmp[3 * i + 2] = previousRotation[i + 6];
    }
    memset(&b_y_tmp[0], 0, 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
      angle = b_y_tmp[3 * i];
      a__1_tmp = 3 * i + 1;
      b_a__1_tmp = 3 * i + 2;
      for (i1 = 0; i1 < 3; i1++) {
        lengthValue = command->desired_rotation[i1 + 3 * i];
        angle += y_tmp[3 * i1] * lengthValue;
        b_y_tmp[a__1_tmp] += y_tmp[3 * i1 + 1] * lengthValue;
        b_y_tmp[b_a__1_tmp] += y_tmp[3 * i1 + 2] * lengthValue;
      }
      b_y_tmp[3 * i] = angle;
    }
    gpenmpcSo3LogVee(SD, b_y_tmp, command->c_desired_angular_velocity_body);
    geodesicGain_tmp = exp(-actualDtS / 0.08);
    command->c_desired_angular_velocity_body[0] *= 1.0 - geodesicGain_tmp;
    command->c_desired_angular_velocity_body[1] *= 1.0 - geodesicGain_tmp;
    command->c_desired_angular_velocity_body[2] *= 1.0 - geodesicGain_tmp;
    angle = 1.5 * actualDtS;
    lengthValue = c_norm(command->c_desired_angular_velocity_body);
    if (!rtIsInf(angle) && !rtIsNaN(angle) && (lengthValue > angle)) {
      angle /= fmax(lengthValue, 1.0E-15);
      command->c_desired_angular_velocity_body[0] *= angle;
      command->c_desired_angular_velocity_body[1] *= angle;
      command->c_desired_angular_velocity_body[2] *= angle;
    }
    /* GPENMPCSO3EXP Exponential map from a three-vector to SO(3). */
    angle = c_norm(command->c_desired_angular_velocity_body);
    correction[0] = 0.0;
    correction[3] = -command->c_desired_angular_velocity_body[2];
    correction[6] = command->c_desired_angular_velocity_body[1];
    correction[1] = command->c_desired_angular_velocity_body[2];
    correction[4] = 0.0;
    correction[7] = -command->c_desired_angular_velocity_body[0];
    correction[2] = -command->c_desired_angular_velocity_body[1];
    correction[5] = command->c_desired_angular_velocity_body[0];
    correction[8] = 0.0;
    if (angle < 1.0E-7) {
      y = rt_powd_snf(angle, 4.0);
      lengthValue = angle * angle;
      a = (1.0 - lengthValue / 6.0) + y / 120.0;
      lengthValue = (0.5 - lengthValue / 24.0) + y / 720.0;
    } else {
      a = sin(angle) / angle;
      lengthValue = (1.0 - cos(angle)) / (angle * angle);
    }
    eye(b_y_tmp);
    memset(&SD->u2.f2.a__1[0], 0, 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
      angle = SD->u2.f2.a__1[3 * i];
      a__1_tmp = 3 * i + 1;
      b_a__1_tmp = 3 * i + 2;
      for (i1 = 0; i1 < 3; i1++) {
        y = correction[i1 + 3 * i];
        angle += correction[3 * i1] * y;
        SD->u2.f2.a__1[a__1_tmp] += correction[3 * i1 + 1] * y;
        SD->u2.f2.a__1[b_a__1_tmp] += correction[3 * i1 + 2] * y;
      }
      SD->u2.f2.a__1[3 * i] = angle;
    }
    for (i = 0; i < 9; i++) {
      b_y_tmp[i] =
          (b_y_tmp[i] + a * correction[i]) + lengthValue * SD->u2.f2.a__1[i];
    }
    memset(&correction[0], 0, 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
      a__1_tmp = 3 * i + 1;
      b_a__1_tmp = 3 * i + 2;
      for (i1 = 0; i1 < 3; i1++) {
        angle = b_y_tmp[i1 + 3 * i];
        correction[3 * i] += previousRotation[3 * i1] * angle;
        correction[a__1_tmp] += previousRotation[3 * i1 + 1] * angle;
        correction[b_a__1_tmp] += previousRotation[3 * i1 + 2] * angle;
      }
    }
    svd(correction, SD->u2.f2.left, SD->u2.f2.a__1, SD->u2.f2.right);
    for (i = 0; i < 9; i++) {
      correction[i] = iv[i];
    }
    memset(&SD->u2.f2.a__1[0], 0, 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
      angle = SD->u2.f2.a__1[3 * i];
      a__1_tmp = 3 * i + 1;
      b_a__1_tmp = 3 * i + 2;
      for (i1 = 0; i1 < 3; i1++) {
        lengthValue = SD->u2.f2.right[i + 3 * i1];
        b_y_tmp[i1 + 3 * i] = lengthValue;
        angle += SD->u2.f2.left[3 * i1] * lengthValue;
        SD->u2.f2.a__1[a__1_tmp] += SD->u2.f2.left[3 * i1 + 1] * lengthValue;
        SD->u2.f2.a__1[b_a__1_tmp] += SD->u2.f2.left[3 * i1 + 2] * lengthValue;
      }
      SD->u2.f2.a__1[3 * i] = angle;
    }
    correction[8] = det(SD->u2.f2.a__1);
    if (rtIsNaN(correction[8])) {
      correction[8] = rtNaN;
    } else if (correction[8] < 0.0) {
      correction[8] = -1.0;
    } else {
      correction[8] = (correction[8] > 0.0);
    }
    if (correction[8] == 0.0) {
      correction[8] = 1.0;
    }
    memset(&SD->u2.f2.a__1[0], 0, 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
      angle = SD->u2.f2.a__1[3 * i];
      a__1_tmp = 3 * i + 1;
      b_a__1_tmp = 3 * i + 2;
      for (i1 = 0; i1 < 3; i1++) {
        i2 = i1 + 3 * i;
        lengthValue = correction[i2];
        angle += SD->u2.f2.left[3 * i1] * lengthValue;
        SD->u2.f2.a__1[a__1_tmp] += SD->u2.f2.left[3 * i1 + 1] * lengthValue;
        SD->u2.f2.a__1[b_a__1_tmp] += SD->u2.f2.left[3 * i1 + 2] * lengthValue;
        command->desired_rotation[i2] = 0.0;
      }
      SD->u2.f2.a__1[3 * i] = angle;
    }
    for (i = 0; i < 3; i++) {
      angle = command->desired_rotation[3 * i];
      a__1_tmp = 3 * i + 1;
      b_a__1_tmp = 3 * i + 2;
      for (i1 = 0; i1 < 3; i1++) {
        lengthValue = b_y_tmp[i1 + 3 * i];
        angle += SD->u2.f2.a__1[3 * i1] * lengthValue;
        command->desired_rotation[a__1_tmp] +=
            SD->u2.f2.a__1[3 * i1 + 1] * lengthValue;
        command->desired_rotation[b_a__1_tmp] +=
            SD->u2.f2.a__1[3 * i1 + 2] * lengthValue;
      }
      command->desired_rotation[3 * i] = angle;
    }
    memset(&b_y_tmp[0], 0, 9U * sizeof(double));
    for (i = 0; i < 3; i++) {
      angle = b_y_tmp[3 * i];
      a__1_tmp = 3 * i + 1;
      b_a__1_tmp = 3 * i + 2;
      for (i1 = 0; i1 < 3; i1++) {
        lengthValue = command->desired_rotation[i1 + 3 * i];
        angle += y_tmp[3 * i1] * lengthValue;
        b_y_tmp[a__1_tmp] += y_tmp[3 * i1 + 1] * lengthValue;
        b_y_tmp[b_a__1_tmp] += y_tmp[3 * i1 + 2] * lengthValue;
      }
      b_y_tmp[3 * i] = angle;
    }
    gpenmpcSo3LogVee(SD, b_y_tmp, command->c_desired_angular_velocity_body);
    command->c_desired_angular_velocity_body[0] /= actualDtS;
    command->c_desired_angular_velocity_body[1] /= actualDtS;
    command->c_desired_angular_velocity_body[2] /= actualDtS;
    angle = c_norm(command->c_desired_angular_velocity_body);
    if (angle > 1.5) {
      lengthValue = 1.5 / angle;
      command->c_desired_angular_velocity_body[0] *= lengthValue;
      command->c_desired_angular_velocity_body[1] *= lengthValue;
      command->c_desired_angular_velocity_body[2] *= lengthValue;
    }
    velocityValid = true;
    for (i = 0; i < 3; i++) {
      lengthValue = previousRotation[3 * i];
      angle = previousRotation[3 * i + 1];
      y = previousRotation[3 * i + 2];
      for (i1 = 0; i1 < 3; i1++) {
        correction[i1 + 3 * i] =
            (command->desired_rotation[3 * i1] * lengthValue +
             command->desired_rotation[3 * i1 + 1] * angle) +
            command->desired_rotation[3 * i1 + 2] * y;
      }
    }
    if (state_angular_velocity_valid) {
      double rawOmegaDot[3];
      double d;
      lengthValue = c_state_desired_angular_velocit[0];
      angle = c_state_desired_angular_velocit[1];
      y = c_state_desired_angular_velocit[2];
      for (i = 0; i < 3; i++) {
        rawOmegaDot[i] =
            (command->c_desired_angular_velocity_body[i] -
             ((correction[i] * lengthValue + correction[i + 3] * angle) +
              correction[i + 6] * y)) /
            actualDtS;
      }
      lengthValue = c_norm(rawOmegaDot);
      if (lengthValue > 8.0) {
        angle = 8.0 / lengthValue;
        rawOmegaDot[0] *= angle;
        rawOmegaDot[1] *= angle;
        rawOmegaDot[2] *= angle;
      }
      y = 0.0;
      a = 0.0;
      d = 0.0;
      for (i = 0; i < 3; i++) {
        lengthValue = c_state_desired_angular_acceler[i];
        y += correction[3 * i] * lengthValue;
        a += correction[3 * i + 1] * lengthValue;
        d += correction[3 * i + 2] * lengthValue;
      }
      rawOmegaDot[0] =
          (y + (1.0 - geodesicGain_tmp) * (rawOmegaDot[0] - y)) - y;
      rawOmegaDot[1] =
          (a + (1.0 - geodesicGain_tmp) * (rawOmegaDot[1] - a)) - a;
      rawOmegaDot[2] =
          (d + (1.0 - geodesicGain_tmp) * (rawOmegaDot[2] - d)) - d;
      lengthValue = 50.0 * actualDtS;
      angle = c_norm(rawOmegaDot);
      if (!rtIsInf(lengthValue) && !rtIsNaN(lengthValue) &&
          (angle > lengthValue)) {
        lengthValue /= fmax(angle, 1.0E-15);
        rawOmegaDot[0] *= lengthValue;
        rawOmegaDot[1] *= lengthValue;
        rawOmegaDot[2] *= lengthValue;
      }
      command->c_desired_angular_acceleration_[0] = y + rawOmegaDot[0];
      command->c_desired_angular_acceleration_[1] = a + rawOmegaDot[1];
      command->c_desired_angular_acceleration_[2] = d + rawOmegaDot[2];
      angle = c_norm(command->c_desired_angular_acceleration_);
      if (angle > 8.0) {
        angle = 8.0 / angle;
        command->c_desired_angular_acceleration_[0] *= angle;
        command->c_desired_angular_acceleration_[1] *= angle;
        command->c_desired_angular_acceleration_[2] *= angle;
      }
      accelerationValid = true;
    } else {
      command->c_desired_angular_acceleration_[0] = 0.0;
      command->c_desired_angular_acceleration_[1] = 0.0;
      command->c_desired_angular_acceleration_[2] = 0.0;
      accelerationValid = false;
    }
  }
  nextState->initialized = true;
  nextState->angular_velocity_valid = velocityValid;
  nextState->angular_acceleration_valid = accelerationValid;
  memcpy(&nextState->filtered_rotation[0], &command->desired_rotation[0],
         9U * sizeof(double));
  nextState->c_desired_angular_velocity_body[0] =
      command->c_desired_angular_velocity_body[0];
  nextState->c_desired_angular_acceleration_[0] =
      command->c_desired_angular_acceleration_[0];
  nextState->c_desired_angular_velocity_body[1] =
      command->c_desired_angular_velocity_body[1];
  nextState->c_desired_angular_acceleration_[1] =
      command->c_desired_angular_acceleration_[1];
  nextState->c_desired_angular_velocity_body[2] =
      command->c_desired_angular_velocity_body[2];
  nextState->c_desired_angular_acceleration_[2] =
      command->c_desired_angular_acceleration_[2];
  nextState->update_count = state_update_count + 1.0;
  nextState->reset_count = state_reset_count + (double)resetActive;
}

/*
 * File trailer for gpenmpcUpdateDesiredAttitudeContinuity.c
 *
 * [EOF]
 */
