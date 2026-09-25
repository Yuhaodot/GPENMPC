/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: GPENMPC_Rfly_Canonical_Controller.h
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

#ifndef GPENMPC_Rfly_Canonical_Controller_h_
#define GPENMPC_Rfly_Canonical_Controller_h_
#ifndef GPENMPC_Rfly_Canonical_Controller_COMMON_INCLUDES_
#define GPENMPC_Rfly_Canonical_Controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                   /* GPENMPC_Rfly_Canonical_Controller_COMMON_INCLUDES_ */

#include "GPENMPC_Rfly_Canonical_Controller_types.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T KernelArguments101[101];      /* '<Root>/KernelArguments101' */
  boolean_T ContinuityEnabled;         /* '<Root>/ContinuityEnabled' */
  boolean_T InputGenerationAccepted;   /* '<Root>/InputGenerationAccepted' */
} ExtU_GPENMPC_Rfly_Canonical_Co_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T Controls16[16];             /* '<Root>/Controls16' */
  real_T FullKernel61[61];             /* '<Root>/FullKernel61' */
  boolean_T OutputValid;               /* '<Root>/OutputValid' */
} ExtY_GPENMPC_Rfly_Canonical_Co_T;

/* Real-time Model Data Structure */
struct tag_RTM_GPENMPC_Rfly_Canonical_T {
  const char_T * volatile errorStatus;
};

/* External inputs (root inport signals with default storage) */
extern ExtU_GPENMPC_Rfly_Canonical_Co_T GPENMPC_Rfly_Canonical_Control_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_GPENMPC_Rfly_Canonical_Co_T GPENMPC_Rfly_Canonical_Control_Y;

/* Model entry point functions */
extern void GPENMPC_Rfly_Canonical_Controller_initialize(void);
extern void GPENMPC_Rfly_Canonical_Controller_step(void);
extern void GPENMPC_Rfly_Canonical_Controller_terminate(void);

/* Real-time Model object */
extern RT_MODEL_GPENMPC_Rfly_Canonica_T *const GPENMPC_Rfly_Canonical_Contro_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'GPENMPC_Rfly_Canonical_Controller'
 * '<S1>'   : 'GPENMPC_Rfly_Canonical_Controller/CanonicalControllerAndOfficialEncoding'
 */
#endif                                 /* GPENMPC_Rfly_Canonical_Controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
