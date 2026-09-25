/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: GPENMPC_Rfly_Canonical_Controller.c
 *
 * Code generated for Simulink model 'GPENMPC_Rfly_Canonical_Controller'.
 *
 * Model version                  : 1.2
 * Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
 * C/C++ source code generated on : Sun Sep  6 10:12:00 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "GPENMPC_Rfly_Canonical_Controller.h"
#include "rtwtypes.h"
#include <string.h>
#include "rt_nonfinite.h"
#include <math.h>

/* External inputs (root inport signals with default storage) */
ExtU_GPENMPC_Rfly_Canonical_Co_T GPENMPC_Rfly_Canonical_Control_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_GPENMPC_Rfly_Canonical_Co_T GPENMPC_Rfly_Canonical_Control_Y;

/* Real-time model */
static RT_MODEL_GPENMPC_Rfly_Canonica_T GPENMPC_Rfly_Canonical_Contro_M_;
RT_MODEL_GPENMPC_Rfly_Canonica_T *const GPENMPC_Rfly_Canonical_Contro_M =
  &GPENMPC_Rfly_Canonical_Contro_M_;

/* Forward declaration for local functions */
static boolean_T GPENMPC_Rfly_Cano_allFiniteFixed(const real_T values[3]);
static real_T GPENMPC_Rfly_Canonical_Cont_norm(const real_T x[4]);
static boolean_T GPENMPC_Rfly_Canonical_Contr_any(const boolean_T x[3]);
static void GPENMPC_Rfly_Canonical_Contr_eye(real_T b_I[9]);
static real_T GPENMPC_Rfly_Canonical_Co_norm_n(const real_T x[9]);
static real_T GPENMPC_Rfly_Canonical_Contr_det(const real_T x[9]);
static real_T GPENMPC_Rfly_Canonical_C_norm_n3(const real_T x[3]);
static real_T GPENMPC_Rfly_Canonical__norm_n3i(const real_T x[2]);
static void GPENMPC_Rfly_Canonical_Con_cross(const real_T a[3], const real_T b[3],
  real_T c[3]);
static boolean_T GPENMPC_Rfly_Canonical_Con_any_n(const boolean_T x[6]);

/* Function for MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding' */
static boolean_T GPENMPC_Rfly_Cano_allFiniteFixed(const real_T values[3])
{
  int32_T b_index;
  boolean_T exitg1;
  boolean_T tf;
  tf = true;
  b_index = 0;
  exitg1 = false;
  while (!exitg1 && (b_index < 3)) {
    if (rtIsInf(values[b_index]) || rtIsNaN(values[b_index])) {
      tf = false;
      exitg1 = true;
    } else {
      b_index++;
    }
  }

  return tf;
}

/* Function for MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding' */
static real_T GPENMPC_Rfly_Canonical_Cont_norm(const real_T x[4])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.312168642111238E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.312168642111238E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.312168642111238E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrt(y);
  if (rtIsNaN(y)) {
    int32_T b_k;
    b_k = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_k < 4) {
        if (rtIsNaN(x[b_k])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = (rtInf);
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding' */
static boolean_T GPENMPC_Rfly_Canonical_Contr_any(const boolean_T x[3])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k < 3)) {
    if (x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding' */
static void GPENMPC_Rfly_Canonical_Contr_eye(real_T b_I[9])
{
  memset(&b_I[0], 0, 9U * sizeof(real_T));
  b_I[0] = 1.0;
  b_I[4] = 1.0;
  b_I[8] = 1.0;
}

/* Function for MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding' */
static real_T GPENMPC_Rfly_Canonical_Co_norm_n(const real_T x[9])
{
  real_T scale;
  real_T y;
  int32_T k;
  y = 0.0;
  scale = 3.312168642111238E-170;
  for (k = 0; k < 9; k++) {
    real_T absxk;
    absxk = fabs(x[k]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  y = scale * sqrt(y);
  if (rtIsNaN(y)) {
    k = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (k < 9) {
        if (rtIsNaN(x[k])) {
          exitg1 = 1;
        } else {
          k++;
        }
      } else {
        y = (rtInf);
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding' */
static real_T GPENMPC_Rfly_Canonical_Contr_det(const real_T x[9])
{
  real_T A[9];
  real_T y;
  int32_T c_k;
  int32_T ijA;
  int32_T j;
  int32_T jA;
  int8_T ipiv[3];
  boolean_T isodd;
  memcpy(&A[0], &x[0], 9U * sizeof(real_T));
  ipiv[0] = 1;
  ipiv[1] = 2;
  for (j = 0; j < 2; j++) {
    real_T smax;
    int32_T jj;
    int32_T n;
    jj = j << 2;
    n = 4 - j;
    jA = 0;
    smax = fabs(A[jj]);
    for (c_k = 2; c_k < n; c_k++) {
      real_T s;
      s = fabs(A[(jj + c_k) - 1]);
      if (s > smax) {
        jA = c_k - 1;
        smax = s;
      }
    }

    if (A[jj + jA] != 0.0) {
      if (jA != 0) {
        n = j + jA;
        ipiv[j] = (int8_T)(n + 1);
        smax = A[j];
        A[j] = A[n];
        A[n] = smax;
        smax = A[j + 3];
        A[j + 3] = A[n + 3];
        A[n + 3] = smax;
        smax = A[j + 6];
        A[j + 6] = A[n + 6];
        A[n + 6] = smax;
      }

      n = (jj - j) + 3;
      for (jA = jj + 2; jA <= n; jA++) {
        A[jA - 1] /= A[jj];
      }
    }

    n = 1 - j;
    jA = jj + 5;
    for (c_k = 0; c_k <= n; c_k++) {
      smax = A[(c_k * 3 + jj) + 3];
      if (smax != 0.0) {
        int32_T c;
        c = (jA - j) + 1;
        for (ijA = jA; ijA <= c; ijA++) {
          A[ijA - 1] += A[((jj + ijA) - jA) + 1] * -smax;
        }
      }

      jA += 3;
    }
  }

  isodd = (ipiv[0] > 1);
  y = A[0] * A[4] * A[8];
  if (ipiv[1] > 2) {
    isodd = !isodd;
  }

  if (isodd) {
    y = -y;
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding' */
static real_T GPENMPC_Rfly_Canonical_C_norm_n3(const real_T x[3])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.312168642111238E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.312168642111238E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.312168642111238E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrt(y);
  if (rtIsNaN(y)) {
    int32_T b_k;
    b_k = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_k < 3) {
        if (rtIsNaN(x[b_k])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = (rtInf);
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding' */
static real_T GPENMPC_Rfly_Canonical__norm_n3i(const real_T x[2])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.312168642111238E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.312168642111238E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.312168642111238E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrt(y);
  if (rtIsNaN(y)) {
    int32_T b_k;
    b_k = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_k < 2) {
        if (rtIsNaN(x[b_k])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = (rtInf);
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding' */
static void GPENMPC_Rfly_Canonical_Con_cross(const real_T a[3], const real_T b[3],
  real_T c[3])
{
  c[0] = a[1] * b[2] - b[1] * a[2];
  c[1] = b[0] * a[2] - a[0] * b[2];
  c[2] = a[0] * b[1] - b[0] * a[1];
}

/* Function for MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding' */
static boolean_T GPENMPC_Rfly_Canonical_Con_any_n(const boolean_T x[6])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k < 6)) {
    if (x[k]) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

/* Model step function */
void GPENMPC_Rfly_Canonical_Controller_step(void)
{
  real_T d[51];
  real_T R[9];
  real_T rawR[9];
  real_T skew[9];
  real_T skew_tmp[9];
  real_T r[6];
  real_T rawRotor[6];
  real_T q[4];
  real_T tmp[4];
  real_T b2[3];
  real_T desiredOmegaCurrent[3];
  real_T force[3];
  real_T rawForce[3];
  real_T rawForce_0[3];
  real_T referenceAir[3];
  real_T tmp_0[3];
  real_T tmp_1[3];
  real_T R_tmp;
  real_T desiredOmegaDot_idx_2;
  real_T eOmega_idx_0;
  real_T eOmega_idx_1;
  real_T eOmega_idx_2;
  real_T horizontal;
  real_T maximumHorizontal;
  real_T maxval;
  real_T r_0;
  real_T tmp_2;
  real_T tmp_3;
  int32_T b_index;
  int32_T i;
  int32_T skew_tmp_0;
  boolean_T y[6];
  boolean_T referenceAir_0[3];
  boolean_T f_tf;
  boolean_T tf;
  static const int8_T b[6] = { 5, 1, 4, 6, 2, 3 };

  static const real_T e[3] = { 1.0, 0.0, 0.0 };

  static const real_T f[3] = { 0.0, 1.0, 0.0 };

  const real_T *desiredR;
  real_T feedforward_idx_0;
  real_T feedforward_idx_1;
  real_T feedforward_idx_2;
  boolean_T exitg1;
  boolean_T guard1;

  /* Outport: '<Root>/Controls16' incorporates:
   *  MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding'
   */
  memset(&GPENMPC_Rfly_Canonical_Control_Y.Controls16[0], 0, sizeof(real32_T) <<
         4U);

  /* Outport: '<Root>/FullKernel61' incorporates:
   *  MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding'
   */
  memset(&GPENMPC_Rfly_Canonical_Control_Y.FullKernel61[0], 0, 61U * sizeof
         (real_T));

  /* Outport: '<Root>/OutputValid' incorporates:
   *  MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding'
   */
  GPENMPC_Rfly_Canonical_Control_Y.OutputValid = false;

  /* MATLAB Function: '<Root>/CanonicalControllerAndOfficialEncoding' incorporates:
   *  Inport: '<Root>/ContinuityEnabled'
   *  Inport: '<Root>/InputGenerationAccepted'
   *  Inport: '<Root>/KernelArguments101'
   *  Outport: '<Root>/Controls16'
   */
  if (GPENMPC_Rfly_Canonical_Control_U.InputGenerationAccepted) {
    memcpy(&skew[0], &GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[34], 9U
           * sizeof(real_T));
    q[0] = 0.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;
    for (i = 0; i < 6; i++) {
      r[i] = 0.0;
    }

    memset(&d[0], 0, 51U * sizeof(real_T));
    f_tf = false;
    tf = true;
    b_index = 0;
    exitg1 = false;
    while (!exitg1 && (b_index < 19)) {
      if (rtIsInf(GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[b_index]) ||
          rtIsNaN(GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[b_index]))
      {
        tf = false;
        exitg1 = true;
      } else {
        b_index++;
      }
    }

    if (tf) {
      if (GPENMPC_Rfly_Cano_allFiniteFixed
          (&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[19])) {
        if (GPENMPC_Rfly_Cano_allFiniteFixed
            (&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[22])) {
          if (GPENMPC_Rfly_Cano_allFiniteFixed
              (&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[25])) {
            if (!rtIsInf(GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[28])
                && !rtIsNaN(GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                            [28])) {
              tf = true;
              b_index = 29;
              exitg1 = false;
              while (!exitg1 && (b_index - 29 < 2)) {
                if (rtIsInf
                    (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[b_index])
                    || rtIsNaN
                    (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[b_index]))
                {
                  tf = false;
                  exitg1 = true;
                } else {
                  b_index++;
                }
              }

              if (tf) {
                if (GPENMPC_Rfly_Cano_allFiniteFixed
                    (&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[31])) {
                  tf = true;
                  b_index = 34;
                  exitg1 = false;
                  while (!exitg1 && (b_index - 34 < 9)) {
                    if (rtIsInf
                        (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[b_index])
                        || rtIsNaN
                        (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[b_index]))
                    {
                      tf = false;
                      exitg1 = true;
                    } else {
                      b_index++;
                    }
                  }

                  if (tf) {
                    if (GPENMPC_Rfly_Cano_allFiniteFixed
                        (&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[43]))
                    {
                      if (GPENMPC_Rfly_Cano_allFiniteFixed
                          (&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                           [46])) {
                        if (GPENMPC_Rfly_Cano_allFiniteFixed
                            (&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[
                             49])) {
                          if (GPENMPC_Rfly_Cano_allFiniteFixed
                              (&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                               [52])) {
                            if (GPENMPC_Rfly_Cano_allFiniteFixed
                                (&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                 [55])) {
                              if (GPENMPC_Rfly_Cano_allFiniteFixed
                                  (&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                   [58])) {
                                if (GPENMPC_Rfly_Cano_allFiniteFixed
                                    (&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                     [61])) {
                                  tf = true;
                                  b_index = 0;
                                  exitg1 = false;
                                  while (!exitg1 && (b_index < 9)) {
                                    if (rtIsInf
                                        ((&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                          [64])[b_index]) || rtIsNaN
                                        ((&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                          [64])[b_index])) {
                                      tf = false;
                                      exitg1 = true;
                                    } else {
                                      b_index++;
                                    }
                                  }

                                  if (tf) {
                                    tf = true;
                                    b_index = 0;
                                    exitg1 = false;
                                    while (!exitg1 && (b_index < 24)) {
                                      if (rtIsInf
                                          ((&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                            [73])[b_index]) || rtIsNaN
                                          ((&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                            [73])[b_index])) {
                                        tf = false;
                                        exitg1 = true;
                                      } else {
                                        b_index++;
                                      }
                                    }

                                    tf = (tf && (!rtIsInf
                                                 (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                                  [97]) && !rtIsNaN
                                                 (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                                  [97]) && (!rtIsInf
                                            (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                             [98]) && !rtIsNaN
                                            (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                             [98]) && (!rtIsInf
                                                       (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                                        [99]) && !rtIsNaN
                                                       (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                                        [99]) && (!rtIsInf
                                              (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                               [100]) && !rtIsNaN
                                              (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                                               [100]))))));
                                  } else {
                                    tf = false;
                                  }
                                } else {
                                  tf = false;
                                }
                              } else {
                                tf = false;
                              }
                            } else {
                              tf = false;
                            }
                          } else {
                            tf = false;
                          }
                        } else {
                          tf = false;
                        }
                      } else {
                        tf = false;
                      }
                    } else {
                      tf = false;
                    }
                  } else {
                    tf = false;
                  }
                } else {
                  tf = false;
                }
              } else {
                tf = false;
              }
            } else {
              tf = false;
            }
          } else {
            tf = false;
          }
        } else {
          tf = false;
        }
      } else {
        tf = false;
      }
    } else {
      tf = false;
    }

    if (tf) {
      tmp[0] = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[6];
      tmp[1] = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[7];
      tmp[2] = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[8];
      tmp[3] = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[9];
      if (!(GPENMPC_Rfly_Canonical_Cont_norm(tmp) < 1.0E-12) &&
          !(GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[28] < 0.0) &&
          !(GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[97] <= 0.0) &&
          !(GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[99] <= 0.0) &&
          !(GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[98] <= 0.0) &&
          !(GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[100] <= 0.0) &&
          !(GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[100] >=
            1.5707963267948966)) {
        referenceAir_0[0] =
          ((&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[64])[0] <= 0.0);
        referenceAir_0[1] =
          ((&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[64])[4] <= 0.0);
        referenceAir_0[2] =
          ((&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[64])[8] <= 0.0);
        if (!GPENMPC_Rfly_Canonical_Contr_any(referenceAir_0)) {
          guard1 = false;
          if (GPENMPC_Rfly_Canonical_Control_U.ContinuityEnabled) {
            GPENMPC_Rfly_Canonical_Contr_eye(skew_tmp);
            for (b_index = 0; b_index < 3; b_index++) {
              for (i = 0; i < 3; i++) {
                skew_tmp_0 = 3 * i + b_index;
                R[skew_tmp_0] = ((skew[3 * b_index + 1] * skew[3 * i + 1] +
                                  skew[3 * b_index] * skew[3 * i]) + skew[3 *
                                 b_index + 2] * skew[3 * i + 2]) -
                  skew_tmp[skew_tmp_0];
              }
            }

            if ((GPENMPC_Rfly_Canonical_Co_norm_n(R) > 1.0E-8) || (fabs
                 (GPENMPC_Rfly_Canonical_Contr_det
                  (&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[34]) -
                  1.0) > 1.0E-8)) {
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }

          if (guard1) {
            tmp[0] = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[6];
            tmp[1] = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[7];
            tmp[2] = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[8];
            tmp[3] = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[9];
            maxval = fmax(GPENMPC_Rfly_Canonical_Cont_norm(tmp), 1.0E-15);
            q[0] = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[6] /
              maxval;
            q[1] = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[7] /
              maxval;
            q[2] = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[8] /
              maxval;
            q[3] = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[9] /
              maxval;
            maxval = q[3] * q[3];
            horizontal = q[2] * q[2];
            R[0] = 1.0 - (horizontal + maxval) * 2.0;
            maximumHorizontal = q[1] * q[2];
            desiredOmegaDot_idx_2 = q[0] * q[3];
            R[3] = (maximumHorizontal - desiredOmegaDot_idx_2) * 2.0;
            R_tmp = q[1] * q[3];
            eOmega_idx_0 = q[0] * q[2];
            R[6] = (R_tmp + eOmega_idx_0) * 2.0;
            R[1] = (maximumHorizontal + desiredOmegaDot_idx_2) * 2.0;
            maximumHorizontal = q[1] * q[1];
            R[4] = 1.0 - (maximumHorizontal + maxval) * 2.0;
            maxval = q[2] * q[3];
            desiredOmegaDot_idx_2 = q[0] * q[1];
            R[7] = (maxval - desiredOmegaDot_idx_2) * 2.0;
            R[2] = (R_tmp - eOmega_idx_0) * 2.0;
            R[5] = (maxval + desiredOmegaDot_idx_2) * 2.0;
            R[8] = 1.0 - (maximumHorizontal + horizontal) * 2.0;
            R_tmp = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[28] +
              GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[97];
            maxval = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[22] -
              GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[29];
            maxval = ((((GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[19]
                         - GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[0])
                        * GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[49]
                        + GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[25])
                       + (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[22]
                          - GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[3])
                       * GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[52])
                      + GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[31]) *
              R_tmp + maxval *
              GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[61] * fabs
              (maxval);
            rawForce[0] = maxval;
            force[0] = maxval;
            maxval = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[23] -
              GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[30];
            maxval = ((((GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[20]
                         - GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[1])
                        * GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[50]
                        + GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[26])
                       + (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[23]
                          - GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[4])
                       * GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[53])
                      + GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[32]) *
              R_tmp + maxval *
              GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[62] * fabs
              (maxval);
            rawForce[1] = maxval;
            force[1] = maxval;
            maxval = (((((GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[21]
                          - GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[2])
                         * GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[51]
                         + GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[27])
                        + (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[24]
                           - GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                           [5]) *
                        GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[54])
                       + 9.80665) +
                      GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[33]) *
              R_tmp + GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[24] *
              GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[63] * fabs
              (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[24]);
            rawForce[2] = maxval;
            force[2] = maxval;
            R_tmp = GPENMPC_Rfly_Canonical_C_norm_n3(rawForce);
            if (R_tmp > GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[98])
            {
              horizontal = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[98]
                / R_tmp;
              force[0] = horizontal * rawForce[0];
              force[1] = horizontal * rawForce[1];
              force[2] = horizontal * maxval;
            }

            horizontal = GPENMPC_Rfly_Canonical__norm_n3i(&force[0]);
            maximumHorizontal = tan
              (GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[100]) * fmax
              (force[2], 1.0E-9);
            if ((horizontal > maximumHorizontal) && (horizontal > 0.0)) {
              maximumHorizontal /= horizontal;
              force[0] *= maximumHorizontal;
              force[1] *= maximumHorizontal;
            }

            horizontal = fmax(GPENMPC_Rfly_Canonical_C_norm_n3(force), 1.0E-15);
            referenceAir[0] = force[0] / horizontal;
            referenceAir[1] = force[1] / horizontal;
            referenceAir[2] = force[2] / horizontal;
            GPENMPC_Rfly_Canonical_Con_cross(referenceAir, e, b2);
            if (GPENMPC_Rfly_Canonical_C_norm_n3(b2) < 1.0E-9) {
              GPENMPC_Rfly_Canonical_Con_cross(referenceAir, f, b2);
            }

            horizontal = fmax(GPENMPC_Rfly_Canonical_C_norm_n3(b2), 1.0E-15);
            b2[0] /= horizontal;
            b2[1] /= horizontal;
            b2[2] /= horizontal;
            GPENMPC_Rfly_Canonical_Con_cross(b2, referenceAir, rawForce_0);
            rawR[0] = rawForce_0[0];
            rawR[3] = b2[0];
            rawR[6] = referenceAir[0];
            rawR[1] = rawForce_0[1];
            rawR[4] = b2[1];
            rawR[7] = referenceAir[1];
            rawR[2] = rawForce_0[2];
            rawR[5] = b2[2];
            rawR[8] = referenceAir[2];
            desiredR = &rawR[0];
            referenceAir[0] = 0.0;
            horizontal = 0.0;
            referenceAir[1] = 0.0;
            maximumHorizontal = 0.0;
            referenceAir[2] = 0.0;
            desiredOmegaDot_idx_2 = 0.0;
            if (GPENMPC_Rfly_Canonical_Control_U.ContinuityEnabled) {
              desiredR = &GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[34];
              referenceAir[0] =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[43];
              horizontal = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[46];
              referenceAir[1] =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[44];
              maximumHorizontal =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[47];
              referenceAir[2] =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[45];
              desiredOmegaDot_idx_2 =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[48];
            }

            for (b_index = 0; b_index < 3; b_index++) {
              R_tmp = desiredR[3 * b_index + 1];
              eOmega_idx_0 = desiredR[3 * b_index];
              eOmega_idx_1 = desiredR[3 * b_index + 2];
              for (i = 0; i < 3; i++) {
                skew_tmp[i + 3 * b_index] = (R[3 * i + 1] * R_tmp + R[3 * i] *
                  eOmega_idx_0) + R[3 * i + 2] * eOmega_idx_1;
              }
            }

            for (i = 0; i < 3; i++) {
              R_tmp = desiredR[3 * i + 1];
              eOmega_idx_0 = desiredR[3 * i];
              eOmega_idx_1 = desiredR[3 * i + 2];
              for (b_index = 0; b_index < 3; b_index++) {
                skew_tmp_0 = 3 * b_index + i;
                skew[skew_tmp_0] = ((R[3 * b_index + 1] * R_tmp + R[3 * b_index]
                                     * eOmega_idx_0) + R[3 * b_index + 2] *
                                    eOmega_idx_1) - skew_tmp[skew_tmp_0];
              }

              desiredOmegaCurrent[i] = 0.0;
            }

            b2[0] = 0.5 * skew[5];
            b2[1] = 0.5 * skew[6];
            b2[2] = 0.5 * skew[1];
            eOmega_idx_0 = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[10];
            feedforward_idx_0 = 0.0;
            eOmega_idx_1 = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[11];
            feedforward_idx_1 = 0.0;
            eOmega_idx_2 = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[12];
            feedforward_idx_2 = 0.0;
            if (GPENMPC_Rfly_Canonical_Control_U.ContinuityEnabled) {
              eOmega_idx_0 = 0.0;
              eOmega_idx_1 = 0.0;
              eOmega_idx_2 = 0.0;
              for (b_index = 0; b_index < 3; b_index++) {
                R_tmp = referenceAir[b_index];
                eOmega_idx_0 += skew_tmp[3 * b_index] * R_tmp;
                eOmega_idx_1 += skew_tmp[3 * b_index + 1] * R_tmp;
                eOmega_idx_2 += skew_tmp[3 * b_index + 2] * R_tmp;
              }

              desiredOmegaCurrent[2] = eOmega_idx_2;
              desiredOmegaCurrent[1] = eOmega_idx_1;
              desiredOmegaCurrent[0] = eOmega_idx_0;
              eOmega_idx_0 = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                [10] - eOmega_idx_0;
              rawForce_0[0] =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[10];
              eOmega_idx_1 = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                [11] - eOmega_idx_1;
              rawForce_0[1] =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[11];
              eOmega_idx_2 = GPENMPC_Rfly_Canonical_Control_U.KernelArguments101
                [12] - eOmega_idx_2;
              rawForce_0[2] =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[12];
              GPENMPC_Rfly_Canonical_Con_cross(rawForce_0, desiredOmegaCurrent,
                tmp_0);
              for (b_index = 0; b_index < 9; b_index++) {
                skew[b_index] =
                  -(&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[64])
                  [b_index];
              }

              feedforward_idx_0 = 0.0;
              feedforward_idx_1 = 0.0;
              feedforward_idx_2 = 0.0;
              for (b_index = 0; b_index < 3; b_index++) {
                rawForce_0[b_index] = tmp_0[b_index] - ((skew_tmp[b_index + 3] *
                  maximumHorizontal + skew_tmp[b_index] * horizontal) +
                  skew_tmp[b_index + 6] * desiredOmegaDot_idx_2);
                R_tmp = rawForce_0[b_index];
                feedforward_idx_0 += skew[3 * b_index] * R_tmp;
                feedforward_idx_1 += skew[3 * b_index + 1] * R_tmp;
                feedforward_idx_2 += skew[3 * b_index + 2] * R_tmp;
              }

              memcpy(&skew_tmp[0],
                     &GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[64], 9U
                     * sizeof(real_T));
              rawForce_0[0] =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[10];
              rawForce_0[1] =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[11];
              rawForce_0[2] =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[12];
              R_tmp = 0.0;
              r_0 = 0.0;
              tmp_3 = 0.0;
              for (b_index = 0; b_index < 3; b_index++) {
                tmp_2 =
                  GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[b_index +
                  10];
                R_tmp += skew_tmp[3 * b_index] * tmp_2;
                r_0 += skew_tmp[3 * b_index + 1] * tmp_2;
                tmp_3 += skew_tmp[3 * b_index + 2] * tmp_2;
              }

              tmp_0[2] = tmp_3;
              tmp_0[1] = r_0;
              tmp_0[0] = R_tmp;
              GPENMPC_Rfly_Canonical_Con_cross(rawForce_0, tmp_0, tmp_1);
              q[1] = ((-GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[55] *
                       b2[0] -
                       GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[58] *
                       eOmega_idx_0) + tmp_1[0]) + feedforward_idx_0;
              q[2] = ((-GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[56] *
                       b2[1] -
                       GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[59] *
                       eOmega_idx_1) + tmp_1[1]) + feedforward_idx_1;
              q[3] = ((-GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[57] *
                       b2[2] -
                       GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[60] *
                       eOmega_idx_2) + tmp_1[2]) + feedforward_idx_2;
            } else {
              memcpy(&skew_tmp[0],
                     &GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[64], 9U
                     * sizeof(real_T));
              rawForce_0[0] =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[10];
              rawForce_0[1] =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[11];
              rawForce_0[2] =
                GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[12];
              R_tmp = 0.0;
              r_0 = 0.0;
              tmp_3 = 0.0;
              for (b_index = 0; b_index < 3; b_index++) {
                tmp_2 =
                  GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[b_index +
                  10];
                R_tmp += skew_tmp[3 * b_index] * tmp_2;
                r_0 += skew_tmp[3 * b_index + 1] * tmp_2;
                tmp_3 += skew_tmp[3 * b_index + 2] * tmp_2;
              }

              tmp_0[2] = tmp_3;
              tmp_0[1] = r_0;
              tmp_0[0] = R_tmp;
              GPENMPC_Rfly_Canonical_Con_cross(rawForce_0, tmp_0, tmp_1);
              q[1] = (-GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[55] *
                      b2[0] -
                      GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[10] *
                      GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[58]) +
                tmp_1[0];
              q[2] = (-GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[56] *
                      b2[1] -
                      GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[11] *
                      GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[59]) +
                tmp_1[1];
              q[3] = (-GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[57] *
                      b2[2] -
                      GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[12] *
                      GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[60]) +
                tmp_1[2];
            }

            q[0] = fmax((force[0] * R[6] + force[1] * R[7]) + force[2] * R[8],
                        0.0);
            for (b_index = 0; b_index < 6; b_index++) {
              rawRotor[b_index] = 0.0;
            }

            for (b_index = 0; b_index < 4; b_index++) {
              R_tmp = q[b_index];
              for (i = 0; i < 6; i++) {
                rawRotor[i] +=
                  (&GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[73])[6 *
                  b_index + i] * R_tmp;
              }
            }

            rawForce_0[0] = rawForce[0] - force[0];
            rawForce_0[1] = rawForce[1] - force[1];
            rawForce_0[2] = maxval - force[2];
            for (b_index = 0; b_index < 6; b_index++) {
              R_tmp = rawRotor[b_index];
              r_0 = fmin(fmax(R_tmp, 0.0),
                         GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[99]);
              r[b_index] = r_0;
              y[b_index] = (fabs(r_0 - R_tmp) > 1.0E-10);
            }

            d[0] = rawForce[0];
            d[3] = force[0];
            d[1] = rawForce[1];
            d[4] = force[1];
            d[2] = maxval;
            d[5] = force[2];
            memcpy(&d[6], &desiredR[0], 9U * sizeof(real_T));
            memcpy(&d[15], &rawR[0], 9U * sizeof(real_T));
            d[24] = b2[0];
            d[27] = referenceAir[0];
            d[30] = horizontal;
            d[33] = desiredOmegaCurrent[0];
            d[36] = eOmega_idx_0;
            d[39] = feedforward_idx_0;
            d[25] = b2[1];
            d[28] = referenceAir[1];
            d[31] = maximumHorizontal;
            d[34] = desiredOmegaCurrent[1];
            d[37] = eOmega_idx_1;
            d[40] = feedforward_idx_1;
            d[26] = b2[2];
            d[29] = referenceAir[2];
            d[32] = desiredOmegaDot_idx_2;
            d[35] = desiredOmegaCurrent[2];
            d[38] = eOmega_idx_2;
            d[41] = feedforward_idx_2;
            for (b_index = 0; b_index < 6; b_index++) {
              d[b_index + 42] = rawRotor[b_index];
            }

            d[48] = GPENMPC_Rfly_Canonical_C_norm_n3(rawForce_0);
            d[49] = acos(fmax(R[8], -1.0));
            d[50] = GPENMPC_Rfly_Canonical_Con_any_n(y);
            f_tf = true;
            b_index = 0;
            exitg1 = false;
            while (!exitg1 && (b_index < 4)) {
              if (rtIsInf(q[b_index]) || rtIsNaN(q[b_index])) {
                f_tf = false;
                exitg1 = true;
              } else {
                b_index++;
              }
            }

            if (f_tf) {
              f_tf = true;
              b_index = 0;
              exitg1 = false;
              while (!exitg1 && (b_index < 6)) {
                if (rtIsInf(r[b_index])) {
                  f_tf = false;
                  exitg1 = true;
                } else {
                  b_index++;
                }
              }

              if (f_tf) {
                f_tf = true;
                b_index = 0;
                exitg1 = false;
                while (!exitg1 && (b_index < 51)) {
                  if (rtIsInf(d[b_index]) || rtIsNaN(d[b_index])) {
                    f_tf = false;
                    exitg1 = true;
                  } else {
                    b_index++;
                  }
                }
              } else {
                f_tf = false;
              }
            } else {
              f_tf = false;
            }

            if (!f_tf) {
              q[0] = 0.0;
              q[1] = 0.0;
              q[2] = 0.0;
              q[3] = 0.0;
              for (i = 0; i < 6; i++) {
                r[i] = 0.0;
              }

              memset(&d[0], 0, 51U * sizeof(real_T));
            }
          }
        }
      }
    }

    /* Outport: '<Root>/FullKernel61' incorporates:
     *  Inport: '<Root>/ContinuityEnabled'
     *  Inport: '<Root>/KernelArguments101'
     */
    GPENMPC_Rfly_Canonical_Control_Y.FullKernel61[0] = q[0];
    GPENMPC_Rfly_Canonical_Control_Y.FullKernel61[1] = q[1];
    GPENMPC_Rfly_Canonical_Control_Y.FullKernel61[2] = q[2];
    GPENMPC_Rfly_Canonical_Control_Y.FullKernel61[3] = q[3];
    for (i = 0; i < 6; i++) {
      GPENMPC_Rfly_Canonical_Control_Y.FullKernel61[i + 4] = r[i];
    }

    memcpy(&GPENMPC_Rfly_Canonical_Control_Y.FullKernel61[10], &d[0], 51U *
           sizeof(real_T));
    if (f_tf && !(GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[99] !=
                  32.145727009134916)) {
      for (b_index = 0; b_index < 6; b_index++) {
        y[b_index] = rtIsInf(r[b_index]);
      }

      if (!GPENMPC_Rfly_Canonical_Con_any_n(y)) {
        for (b_index = 0; b_index < 6; b_index++) {
          y[b_index] = (r[b_index] < 0.0);
        }

        if (!GPENMPC_Rfly_Canonical_Con_any_n(y)) {
          for (b_index = 0; b_index < 6; b_index++) {
            y[b_index] = (r[b_index] > 32.145727009134916);
          }

          if (!GPENMPC_Rfly_Canonical_Con_any_n(y)) {
            for (b_index = 0; b_index < 6; b_index++) {
              GPENMPC_Rfly_Canonical_Control_Y.Controls16[b[b_index] - 1] =
                (real32_T)(r[b_index] / 32.145727009134916);
            }

            /* Outport: '<Root>/OutputValid' incorporates:
             *  Outport: '<Root>/Controls16'
             */
            GPENMPC_Rfly_Canonical_Control_Y.OutputValid = true;
          }
        }
      }
    }
  }
}

/* Model initialize function */
void GPENMPC_Rfly_Canonical_Controller_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void GPENMPC_Rfly_Canonical_Controller_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
