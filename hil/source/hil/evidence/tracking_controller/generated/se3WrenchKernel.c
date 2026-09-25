/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: se3WrenchKernel.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "se3WrenchKernel.h"
#include "allOrAny.h"
#include "det.h"
#include "dot.h"
#include "eye.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * SE(3) wrench and rotor allocation in +Z-up coordinates. The caller supplies
 *  coordinated residual and attitude-continuity outputs and owns GP/eNMPC
 *  and residual/continuity-state updates. An invalid return must suppress
 *  publication; its zero wrench is not a motor command.
 *  Source oracle: gpenmpcDesiredSe3Command + gpenmpcRobustSe3Control.
 *
 * Arguments    : const double x[19]
 *                const double refP[3]
 *                const double refV[3]
 *                const double refA[3]
 *                double payload
 *                const double windXY[2]
 *                const double augmentation[3]
 *                const double commandR[9]
 *                const double commandOmega[3]
 *                const double commandOmegaDot[3]
 *                double wrench[4]
 *                double rotor[6]
 *                double diagnostic[51]
 * Return Type  : boolean_T
 */
boolean_T
se3WrenchKernel(const double x[19], const double refP[3], const double refV[3],
                const double refA[3], double payload, const double windXY[2],
                const double augmentation[3], const double commandR[9],
                const double commandOmega[3], const double commandOmegaDot[3],
                double wrench[4], double rotor[6], double diagnostic[51])
{
  static const double c_a[24] = {
      0.16666666666666666,   0.16666666666666663, 0.16666666666666663,
      0.16666666666666666,   0.16666666666666669, 0.16666666666666669,
      -1.1590111841446E-16,  0.5095765835742504,  0.5095765835742505,
      9.186378620109457E-17, -0.5095765835742505, -0.5095765835742505,
      -0.5884083553986466,   -0.2942041776993232, 0.29420417769932317,
      0.5884083553986466,    0.29420417769932344, -0.29420417769932333,
      6.666666666666667,     -6.666666666666666,  6.666666666666666,
      -6.666666666666667,    6.666666666666666,   -6.666666666666666};
  static const double a[9] = {-1.6, -0.0, -0.0, -0.0, -1.6,
                              -0.0, -0.0, -0.0, -3.0};
  static const double b_a[9] = {1.6, 0.0, 0.0, 0.0, 1.6, 0.0, 0.0, 0.0, 3.0};
  double desiredToCurrent[9];
  double rawRotor[6];
  double b2[3];
  double b_desiredToCurrent[3];
  double desiredOmegaCurrent[3];
  double feedforward[3];
  double referenceAir[3];
  int b_i;
  int b_index;
  int i;
  boolean_T exitg1;
  boolean_T tf;
  boolean_T valid;
  wrench[0] = 0.0;
  wrench[1] = 0.0;
  wrench[2] = 0.0;
  wrench[3] = 0.0;
  for (i = 0; i < 6; i++) {
    rotor[i] = 0.0;
  }
  memset(&diagnostic[0], 0, 51U * sizeof(double));
  valid = false;
  /*  Check each fixed-size input in place.  Do not concatenate the complete */
  /*  input set here: MATLAB Coder otherwise materialises a 101-double scratch
   */
  /*  array in the generated function's stack frame.  The element-wise helper */
  /*  preserves the same finite-input predicate while remaining suitable for */
  /*  the FMUv6C task stack. */
  /*  Scalar short-circuit implementation avoids generated temporary vectors. */
  tf = true;
  b_index = 0;
  exitg1 = false;
  while (!exitg1 && (b_index < 19)) {
    if (rtIsInf(x[b_index]) || rtIsNaN(x[b_index])) {
      tf = false;
      exitg1 = true;
    } else {
      b_index++;
    }
  }
  if (tf) {
    /*  Scalar short-circuit implementation avoids generated temporary vectors.
     */
    tf = true;
    b_index = 0;
    exitg1 = false;
    while (!exitg1 && (b_index < 3)) {
      if (rtIsInf(refP[b_index]) || rtIsNaN(refP[b_index])) {
        tf = false;
        exitg1 = true;
      } else {
        b_index++;
      }
    }
    if (tf) {
      /*  Scalar short-circuit implementation avoids generated temporary
       * vectors. */
      tf = true;
      b_index = 0;
      exitg1 = false;
      while (!exitg1 && (b_index < 3)) {
        if (rtIsInf(refV[b_index]) || rtIsNaN(refV[b_index])) {
          tf = false;
          exitg1 = true;
        } else {
          b_index++;
        }
      }
      if (tf) {
        /*  Scalar short-circuit implementation avoids generated temporary
         * vectors. */
        tf = true;
        b_index = 0;
        exitg1 = false;
        while (!exitg1 && (b_index < 3)) {
          if (rtIsInf(refA[b_index]) || rtIsNaN(refA[b_index])) {
            tf = false;
            exitg1 = true;
          } else {
            b_index++;
          }
        }
        if (tf && (!rtIsInf(payload) && !rtIsNaN(payload))) {
          /*  Scalar short-circuit implementation avoids generated temporary
           * vectors. */
          tf = true;
          b_index = 0;
          exitg1 = false;
          while (!exitg1 && (b_index < 2)) {
            if (rtIsInf(windXY[b_index]) || rtIsNaN(windXY[b_index])) {
              tf = false;
              exitg1 = true;
            } else {
              b_index++;
            }
          }
          if (tf) {
            /*  Scalar short-circuit implementation avoids generated temporary
             * vectors. */
            tf = true;
            b_index = 0;
            exitg1 = false;
            while (!exitg1 && (b_index < 3)) {
              if (rtIsInf(augmentation[b_index]) ||
                  rtIsNaN(augmentation[b_index])) {
                tf = false;
                exitg1 = true;
              } else {
                b_index++;
              }
            }
            if (tf) {
              /*  Scalar short-circuit implementation avoids generated temporary
               * vectors. */
              tf = true;
              b_index = 0;
              exitg1 = false;
              while (!exitg1 && (b_index < 9)) {
                if (rtIsInf(commandR[b_index]) || rtIsNaN(commandR[b_index])) {
                  tf = false;
                  exitg1 = true;
                } else {
                  b_index++;
                }
              }
              if (tf) {
                /*  Scalar short-circuit implementation avoids generated
                 * temporary vectors. */
                tf = true;
                b_index = 0;
                exitg1 = false;
                while (!exitg1 && (b_index < 3)) {
                  if (rtIsInf(commandOmega[b_index]) ||
                      rtIsNaN(commandOmega[b_index])) {
                    tf = false;
                    exitg1 = true;
                  } else {
                    b_index++;
                  }
                }
                if (tf) {
                  /*  Scalar short-circuit implementation avoids generated
                   * temporary vectors. */
                  tf = true;
                  b_index = 0;
                  exitg1 = false;
                  while (!exitg1 && (b_index < 3)) {
                    if (rtIsInf(commandOmegaDot[b_index]) ||
                        rtIsNaN(commandOmegaDot[b_index])) {
                      tf = false;
                      exitg1 = true;
                    } else {
                      b_index++;
                    }
                  }
                  if (tf) {
                    double horizontal;
                    /*  Scalar short-circuit implementation avoids generated
                     * temporary vectors. */
                    horizontal = d_norm(&x[6]);
                    if (!(horizontal < 1.0E-12)) {
                      boolean_T force_data[17];
                      force_data[0] = false;
                      force_data[1] = false;
                      force_data[2] = false;
                      if (!vectorAny(force_data, 3)) {
                        double R[9];
                        double y_tmp[9];
                        double R_tmp;
                        double maximumHorizontal;
                        double q_idx_3;
                        for (i = 0; i < 3; i++) {
                          y_tmp[3 * i] = commandR[i];
                          y_tmp[3 * i + 1] = commandR[i + 3];
                          y_tmp[3 * i + 2] = commandR[i + 6];
                        }
                        eye(R);
                        for (i = 0; i < 3; i++) {
                          maximumHorizontal = y_tmp[i];
                          R_tmp = y_tmp[i + 3];
                          q_idx_3 = y_tmp[i + 6];
                          for (b_i = 0; b_i < 3; b_i++) {
                            b_index = i + 3 * b_i;
                            desiredToCurrent[b_index] =
                                ((maximumHorizontal * commandR[3 * b_i] +
                                  R_tmp * commandR[3 * b_i + 1]) +
                                 q_idx_3 * commandR[3 * b_i + 2]) -
                                R[b_index];
                          }
                        }
                        if (!(e_norm(desiredToCurrent) > 1.0E-8) &&
                            !(fabs(det(commandR) - 1.0) > 1.0E-8)) {
                          double skew[9];
                          double b_desiredOmegaCurrent[3];
                          double eOmega[3];
                          double force[3];
                          double b3_idx_0;
                          double b3_idx_1;
                          double b3_idx_2;
                          double b_R_tmp;
                          double d;
                          double eR_idx_1;
                          double eR_idx_2;
                          double q_idx_0;
                          double q_idx_1;
                          double q_idx_2;
                          horizontal = fmax(horizontal, 1.0E-15);
                          q_idx_0 = x[6] / horizontal;
                          q_idx_1 = x[7] / horizontal;
                          q_idx_2 = x[8] / horizontal;
                          q_idx_3 = x[9] / horizontal;
                          R_tmp = q_idx_3 * q_idx_3;
                          eR_idx_1 = q_idx_2 * q_idx_2;
                          R[0] = 1.0 - 2.0 * (eR_idx_1 + R_tmp);
                          horizontal = q_idx_1 * q_idx_2;
                          maximumHorizontal = q_idx_0 * q_idx_3;
                          R[3] = 2.0 * (horizontal - maximumHorizontal);
                          eR_idx_2 = q_idx_1 * q_idx_3;
                          b3_idx_2 = q_idx_0 * q_idx_2;
                          R[6] = 2.0 * (eR_idx_2 + b3_idx_2);
                          R[1] = 2.0 * (horizontal + maximumHorizontal);
                          b_R_tmp = q_idx_1 * q_idx_1;
                          R[4] = 1.0 - 2.0 * (b_R_tmp + R_tmp);
                          maximumHorizontal = q_idx_2 * q_idx_3;
                          horizontal = q_idx_0 * q_idx_1;
                          R[7] = 2.0 * (maximumHorizontal - horizontal);
                          R[2] = 2.0 * (eR_idx_2 - b3_idx_2);
                          R[5] = 2.0 * (maximumHorizontal + horizontal);
                          R[8] = 1.0 - 2.0 * (b_R_tmp + eR_idx_1);
                          referenceAir[0] = refV[0] - windXY[0];
                          referenceAir[1] = refV[1] - windXY[1];
                          d = (payload + 9.5) *
                                  (((refA[0] +
                                     0.42250000000000004 * (refP[0] - x[0])) +
                                    1.1700000000000002 * (refV[0] - x[3])) +
                                   augmentation[0]) +
                              0.0634905529323215 * referenceAir[0] *
                                  fabs(referenceAir[0]);
                          referenceAir[0] = d;
                          force[0] = d;
                          d = (payload + 9.5) *
                                  (((refA[1] +
                                     0.42250000000000004 * (refP[1] - x[1])) +
                                    1.1700000000000002 * (refV[1] - x[4])) +
                                   augmentation[1]) +
                              0.0634905529323215 * referenceAir[1] *
                                  fabs(referenceAir[1]);
                          referenceAir[1] = d;
                          force[1] = d;
                          d = (payload + 9.5) *
                                  ((((refA[2] + 0.5625 * (refP[2] - x[2])) +
                                     1.35 * (refV[2] - x[5])) +
                                    9.80665) +
                                   augmentation[2]) +
                              0.0634905529323215 * refV[2] * fabs(refV[2]);
                          referenceAir[2] = d;
                          force[2] = d;
                          horizontal = c_norm(referenceAir);
                          if (horizontal > 192.8743620548095) {
                            horizontal = 192.8743620548095 / horizontal;
                            force[0] = referenceAir[0] * horizontal;
                            force[1] = referenceAir[1] * horizontal;
                            force[2] = d * horizontal;
                          }
                          horizontal = b_norm(&force[0]);
                          maximumHorizontal =
                              0.4663076581549986 * fmax(force[2], 1.0E-9);
                          if (horizontal > maximumHorizontal) {
                            horizontal = maximumHorizontal / horizontal;
                            force[0] *= horizontal;
                            force[1] *= horizontal;
                          }
                          horizontal = fmax(c_norm(force), 1.0E-15);
                          b3_idx_0 = force[0] / horizontal;
                          b3_idx_1 = force[1] / horizontal;
                          b3_idx_2 = force[2] / horizontal;
                          horizontal = b3_idx_1 * 0.0;
                          maximumHorizontal = 0.0 * b3_idx_2;
                          b2[0] = horizontal - maximumHorizontal;
                          R_tmp = b3_idx_0 * 0.0;
                          b2[1] = b3_idx_2 - R_tmp;
                          b2[2] = R_tmp - b3_idx_1;
                          if (c_norm(b2) < 1.0E-9) {
                            b2[0] = horizontal - b3_idx_2;
                            b2[1] = maximumHorizontal - R_tmp;
                            b2[2] = b3_idx_0 - horizontal;
                          }
                          horizontal = fmax(c_norm(b2), 1.0E-15);
                          for (i = 0; i < 3; i++) {
                            b2[i] /= horizontal;
                            maximumHorizontal = commandR[3 * i];
                            q_idx_3 = commandR[3 * i + 1];
                            R_tmp = commandR[3 * i + 2];
                            for (b_i = 0; b_i < 3; b_i++) {
                              desiredToCurrent[b_i + 3 * i] =
                                  (R[3 * b_i] * maximumHorizontal +
                                   R[3 * b_i + 1] * q_idx_3) +
                                  R[3 * b_i + 2] * R_tmp;
                            }
                          }
                          memset(&desiredOmegaCurrent[0], 0,
                                 3U * sizeof(double));
                          for (i = 0; i < 3; i++) {
                            horizontal = y_tmp[i];
                            maximumHorizontal = y_tmp[i + 3];
                            q_idx_3 = y_tmp[i + 6];
                            for (b_i = 0; b_i < 3; b_i++) {
                              b_index = i + 3 * b_i;
                              skew[b_index] =
                                  ((horizontal * R[3 * b_i] +
                                    maximumHorizontal * R[3 * b_i + 1]) +
                                   q_idx_3 * R[3 * b_i + 2]) -
                                  desiredToCurrent[b_index];
                            }
                            horizontal = commandOmega[i];
                            desiredOmegaCurrent[0] +=
                                desiredToCurrent[3 * i] * horizontal;
                            desiredOmegaCurrent[1] +=
                                desiredToCurrent[3 * i + 1] * horizontal;
                            desiredOmegaCurrent[2] +=
                                desiredToCurrent[3 * i + 2] * horizontal;
                          }
                          q_idx_1 = 0.5 * skew[5];
                          eR_idx_1 = 0.5 * skew[6];
                          eR_idx_2 = 0.5 * skew[1];
                          b_desiredOmegaCurrent[0] =
                              desiredOmegaCurrent[2] * x[11] -
                              desiredOmegaCurrent[1] * x[12];
                          b_desiredOmegaCurrent[1] =
                              desiredOmegaCurrent[0] * x[12] -
                              desiredOmegaCurrent[2] * x[10];
                          b_desiredOmegaCurrent[2] =
                              desiredOmegaCurrent[1] * x[10] -
                              desiredOmegaCurrent[0] * x[11];
                          memset(&b_desiredToCurrent[0], 0,
                                 3U * sizeof(double));
                          horizontal = b_desiredToCurrent[0];
                          maximumHorizontal = b_desiredToCurrent[1];
                          q_idx_3 = b_desiredToCurrent[2];
                          for (i = 0; i < 3; i++) {
                            eOmega[i] = x[i + 10] - desiredOmegaCurrent[i];
                            R_tmp = commandOmegaDot[i];
                            horizontal += desiredToCurrent[3 * i] * R_tmp;
                            maximumHorizontal +=
                                desiredToCurrent[3 * i + 1] * R_tmp;
                            q_idx_3 += desiredToCurrent[3 * i + 2] * R_tmp;
                          }
                          b_desiredToCurrent[2] = q_idx_3;
                          b_desiredToCurrent[1] = maximumHorizontal;
                          b_desiredToCurrent[0] = horizontal;
                          memset(&feedforward[0], 0, 3U * sizeof(double));
                          b_R_tmp = feedforward[0];
                          q_idx_2 = feedforward[1];
                          q_idx_0 = feedforward[2];
                          for (i = 0; i < 3; i++) {
                            horizontal = b_desiredOmegaCurrent[i] -
                                         b_desiredToCurrent[i];
                            b_desiredOmegaCurrent[i] = horizontal;
                            b_R_tmp += a[3 * i] * horizontal;
                            q_idx_2 += a[3 * i + 1] * horizontal;
                            q_idx_0 += a[3 * i + 2] * horizontal;
                          }
                          memset(&b_desiredToCurrent[0], 0,
                                 3U * sizeof(double));
                          horizontal = b_desiredToCurrent[0];
                          maximumHorizontal = b_desiredToCurrent[1];
                          q_idx_3 = b_desiredToCurrent[2];
                          for (i = 0; i < 3; i++) {
                            R_tmp = x[i + 10];
                            horizontal += b_a[3 * i] * R_tmp;
                            maximumHorizontal += b_a[3 * i + 1] * R_tmp;
                            q_idx_3 += b_a[3 * i + 2] * R_tmp;
                          }
                          wrench[0] = fmax(dot(force, &R[6]), 0.0);
                          wrench[1] =
                              ((-12.543999999999999 * q_idx_1 -
                                7.616 * eOmega[0]) +
                               (q_idx_3 * x[11] - maximumHorizontal * x[12])) +
                              b_R_tmp;
                          wrench[2] = ((-12.543999999999999 * eR_idx_1 -
                                        7.616 * eOmega[1]) +
                                       (horizontal * x[12] - q_idx_3 * x[10])) +
                                      q_idx_2;
                          wrench[3] = ((-12.0 * eR_idx_2 - 10.2 * eOmega[2]) +
                                       (maximumHorizontal * x[10] -
                                        horizontal * x[11])) +
                                      q_idx_0;
                          memset(&rawRotor[0], 0, 6U * sizeof(double));
                          for (i = 0; i < 4; i++) {
                            horizontal = wrench[i];
                            for (b_i = 0; b_i < 6; b_i++) {
                              rawRotor[b_i] += c_a[b_i + 6 * i] * horizontal;
                            }
                          }
                          /*  Fixed diagnostic layout is documented/tested
                           * against the source oracle. */
                          desiredToCurrent[0] =
                              b2[1] * b3_idx_2 - b3_idx_1 * b2[2];
                          desiredToCurrent[1] =
                              b3_idx_0 * b2[2] - b2[0] * b3_idx_2;
                          desiredToCurrent[2] =
                              b2[0] * b3_idx_1 - b3_idx_0 * b2[1];
                          desiredToCurrent[3] = b2[0];
                          desiredToCurrent[6] = b3_idx_0;
                          b_desiredToCurrent[0] = referenceAir[0] - force[0];
                          desiredToCurrent[4] = b2[1];
                          desiredToCurrent[7] = b3_idx_1;
                          b_desiredToCurrent[1] = referenceAir[1] - force[1];
                          desiredToCurrent[5] = b2[2];
                          desiredToCurrent[8] = b3_idx_2;
                          b_desiredToCurrent[2] = d - force[2];
                          for (i = 0; i < 6; i++) {
                            horizontal = rawRotor[i];
                            maximumHorizontal =
                                fmin(fmax(horizontal, 0.0), 32.145727009134916);
                            rotor[i] = maximumHorizontal;
                            force_data[i] = (fabs(maximumHorizontal -
                                                  horizontal) > 1.0E-10);
                          }
                          diagnostic[0] = referenceAir[0];
                          diagnostic[3] = force[0];
                          diagnostic[1] = referenceAir[1];
                          diagnostic[4] = force[1];
                          diagnostic[2] = d;
                          diagnostic[5] = force[2];
                          memcpy(&diagnostic[6], &commandR[0],
                                 9U * sizeof(double));
                          memcpy(&diagnostic[15], &desiredToCurrent[0],
                                 9U * sizeof(double));
                          diagnostic[24] = q_idx_1;
                          diagnostic[27] = commandOmega[0];
                          diagnostic[30] = commandOmegaDot[0];
                          diagnostic[33] = desiredOmegaCurrent[0];
                          diagnostic[36] = eOmega[0];
                          diagnostic[39] = b_R_tmp;
                          diagnostic[25] = eR_idx_1;
                          diagnostic[28] = commandOmega[1];
                          diagnostic[31] = commandOmegaDot[1];
                          diagnostic[34] = desiredOmegaCurrent[1];
                          diagnostic[37] = eOmega[1];
                          diagnostic[40] = q_idx_2;
                          diagnostic[26] = eR_idx_2;
                          diagnostic[29] = commandOmega[2];
                          diagnostic[32] = commandOmegaDot[2];
                          diagnostic[35] = desiredOmegaCurrent[2];
                          diagnostic[38] = eOmega[2];
                          diagnostic[41] = q_idx_0;
                          for (i = 0; i < 6; i++) {
                            diagnostic[i + 42] = rawRotor[i];
                          }
                          diagnostic[48] = c_norm(b_desiredToCurrent);
                          diagnostic[49] = acos(fmax(R[8], -1.0));
                          diagnostic[50] = vectorAny(force_data, 6);
                          /*  As above, validate outputs without constructing a
                           * 61-double aggregate. */
                          /*  Scalar short-circuit implementation avoids
                           * generated temporary vectors. */
                          tf = true;
                          b_index = 0;
                          exitg1 = false;
                          while (!exitg1 && (b_index < 4)) {
                            if (rtIsInf(wrench[b_index]) ||
                                rtIsNaN(wrench[b_index])) {
                              tf = false;
                              exitg1 = true;
                            } else {
                              b_index++;
                            }
                          }
                          if (tf) {
                            /*  Scalar short-circuit implementation avoids
                             * generated temporary vectors. */
                            tf = true;
                            b_index = 0;
                            exitg1 = false;
                            while (!exitg1 && (b_index < 51)) {
                              if (rtIsInf(diagnostic[b_index]) ||
                                  rtIsNaN(diagnostic[b_index])) {
                                tf = false;
                                exitg1 = true;
                              } else {
                                b_index++;
                              }
                            }
                            if (tf) {
                              valid = true;
                            } else {
                              valid = false;
                            }
                          } else {
                            valid = false;
                          }
                          if (!valid) {
                            wrench[0] = 0.0;
                            wrench[1] = 0.0;
                            wrench[2] = 0.0;
                            wrench[3] = 0.0;
                            for (i = 0; i < 6; i++) {
                              rotor[i] = 0.0;
                            }
                            memset(&diagnostic[0], 0, 51U * sizeof(double));
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  return valid;
}

/*
 * File trailer for se3WrenchKernel.c
 *
 * [EOF]
 */
