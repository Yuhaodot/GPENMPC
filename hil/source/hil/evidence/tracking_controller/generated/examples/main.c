/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_initialize.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_terminate.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditStep.h"
#include "gpenmpcNative_canonicalReferenceTransitionFromJet.h"
#include "gpenmpcNative_queryCanonicalReferenceWindow.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static e_gpenmpcNative_canonicalLocalIn c_gpenmpcNative_canonicalLocalIn;

/* Function Declarations */
static void argInit_256x1_real_T(double result[256]);

static void argInit_256x3x4_real_T(double result[3072]);

static void argInit_2x1_uint64_T(unsigned long long result[2]);

static void argInit_32x1_uint8_T(unsigned char result[32]);

static void argInit_36x1_real_T(double result[36]);

static void argInit_3x1_real_T(double result[3]);

static void argInit_3x4_real_T(double result[12]);

static void argInit_3x8x4x2_real_T(double result[192]);

static void argInit_64x1_real_T(double result[64]);

static void argInit_70x1_real_T(double result[70]);

static double argInit_real_T(void);

static void argInit_struct51_T(struct51_T *result);

static struct52_T argInit_struct52_T(void);

static struct53_T argInit_struct53_T(void);

static unsigned short argInit_uint16_T(void);

static unsigned int argInit_uint32_T(void);

static unsigned long long argInit_uint64_T(void);

static unsigned char argInit_uint8_T(void);

/* Function Definitions */
/*
 * Arguments    : double result[256]
 * Return Type  : void
 */
static void argInit_256x1_real_T(double result[256])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 256; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[3072]
 * Return Type  : void
 */
static void argInit_256x3x4_real_T(double result[3072])
{
  int idx0;
  int idx1;
  int idx2;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 256; idx0++) {
    for (idx1 = 0; idx1 < 3; idx1++) {
      for (idx2 = 0; idx2 < 4; idx2++) {
        /* Set the value of the array element.
Change this value to the value that the application requires. */
        result[(idx0 + (idx1 << 8)) + 768 * idx2] = argInit_real_T();
      }
    }
  }
}

/*
 * Arguments    : unsigned long long result[2]
 * Return Type  : void
 */
static void argInit_2x1_uint64_T(unsigned long long result[2])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 2; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_uint64_T();
  }
}

/*
 * Arguments    : unsigned char result[32]
 * Return Type  : void
 */
static void argInit_32x1_uint8_T(unsigned char result[32])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 32; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_uint8_T();
  }
}

/*
 * Arguments    : double result[36]
 * Return Type  : void
 */
static void argInit_36x1_real_T(double result[36])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 36; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[3]
 * Return Type  : void
 */
static void argInit_3x1_real_T(double result[3])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[12]
 * Return Type  : void
 */
static void argInit_3x4_real_T(double result[12])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 12; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[192]
 * Return Type  : void
 */
static void argInit_3x8x4x2_real_T(double result[192])
{
  int idx0;
  int idx1;
  int idx2;
  int idx3;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    for (idx1 = 0; idx1 < 8; idx1++) {
      for (idx2 = 0; idx2 < 4; idx2++) {
        for (idx3 = 0; idx3 < 2; idx3++) {
          /* Set the value of the array element.
Change this value to the value that the application requires. */
          result[((idx0 + 3 * idx1) + 24 * idx2) + 96 * idx3] =
              argInit_real_T();
        }
      }
    }
  }
}

/*
 * Arguments    : double result[64]
 * Return Type  : void
 */
static void argInit_64x1_real_T(double result[64])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 64; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[70]
 * Return Type  : void
 */
static void argInit_70x1_real_T(double result[70])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 70; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : struct51_T *result
 * Return Type  : void
 */
static void argInit_struct51_T(struct51_T *result)
{
  double c_result_tmp;
  unsigned int result_tmp;
  unsigned short b_result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_uint32_T();
  b_result_tmp = argInit_uint16_T();
  c_result_tmp = argInit_real_T();
  argInit_3x4_real_T(result->ground_jet);
  result->schema = result_tmp;
  result->capacity = b_result_tmp;
  argInit_32x1_uint8_T(result->reference_asset_sha256);
  result->leg_index = result_tmp;
  result->window_generation = argInit_uint64_T();
  result->source_first_row = result_tmp;
  result->source_total_rows = result_tmp;
  result->row_count = b_result_tmp;
  argInit_256x1_real_T(result->time_s);
  argInit_256x3x4_real_T(result->nominal_jet);
  result->nominal_duration_s = c_result_tmp;
  result->total_duration_s = c_result_tmp;
  result->binding_mode = argInit_uint8_T();
  result->prefix_duration_s = c_result_tmp;
  argInit_3x8x4x2_real_T(result->prefix_coefficients);
  result->relaunch_duration_s = c_result_tmp;
  argInit_3x1_real_T(result->relaunch_offset_ned_m);
  result->vertical_frame_offset_ned_m = c_result_tmp;
  memcpy(&result->rest_jet[0], &result->ground_jet[0], 12U * sizeof(double));
}

/*
 * Arguments    : void
 * Return Type  : struct52_T
 */
static struct52_T argInit_struct52_T(void)
{
  struct52_T result;
  unsigned long long result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_uint64_T();
  argInit_32x1_uint8_T(result.reference_asset_sha256);
  result.leg_index = argInit_uint32_T();
  result.window_generation = result_tmp;
  result.last_accepted_sequence = result_tmp;
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct53_T
 */
static struct53_T argInit_struct53_T(void)
{
  struct53_T result;
  unsigned long long result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_uint64_T();
  argInit_32x1_uint8_T(result.reference_asset_sha256);
  result.leg_index = argInit_uint32_T();
  result.window_generation = result_tmp;
  result.query_sequence = result_tmp;
  result.progress_s = argInit_real_T();
  return result;
}

/*
 * Arguments    : void
 * Return Type  : unsigned short
 */
static unsigned short argInit_uint16_T(void)
{
  return 0U;
}

/*
 * Arguments    : void
 * Return Type  : unsigned int
 */
static unsigned int argInit_uint32_T(void)
{
  return 0U;
}

/*
 * Arguments    : void
 * Return Type  : unsigned long long
 */
static unsigned long long argInit_uint64_T(void)
{
  return 0ULL;
}

/*
 * Arguments    : void
 * Return Type  : unsigned char
 */
static unsigned char argInit_uint8_T(void)
{
  return 0U;
}

/*
 * Arguments    : int argc
 *                char **argv
 * Return Type  : int
 */
int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* Initialize the application.
You do not need to do this more than one time. */
  gpenmpcNative_canonicalLocalInnerWithAuditFirst_initialize();
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_gpenmpcNative_canonicalLocalInnerWithAuditFirst();
  main_gpenmpcNative_canonicalLocalInnerWithAuditStep();
  main_gpenmpcNative_queryCanonicalReferenceWindow();
  main_gpenmpcNative_canonicalReferenceTransitionFromJet();
  /* Terminate the application.
You do not need to do this more than one time. */
  gpenmpcNative_canonicalLocalInnerWithAuditFirst_terminate();
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_gpenmpcNative_canonicalLocalInnerWithAuditFirst(void)
{
  static double scaffold70[70];
  static double next64[64];
  static double kernel61[61];
  static double b_dv[36];
  static double request19[19];
  static double learning12[12];
  double closed5[5];
  unsigned long long uv[2];
  /* Initialize function 'gpenmpcNative_canonicalLocalInnerWithAuditFirst' input
   * arguments. */
  /* Initialize function input argument 'input36'. */
  /* Initialize function input argument 'inputTags2'. */
  /* Call the entry-point 'gpenmpcNative_canonicalLocalInnerWithAuditFirst'. */
  argInit_36x1_real_T(b_dv);
  argInit_2x1_uint64_T(uv);
  gpenmpcNative_canonicalLocalInnerWithAuditFirst(
      &c_gpenmpcNative_canonicalLocalIn, b_dv, uv, next64, kernel61, scaffold70,
      request19, closed5, learning12);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_gpenmpcNative_canonicalLocalInnerWithAuditStep(void)
{
  static double dv2[70];
  static double scaffold70[70];
  static double b_dv[64];
  static double next64[64];
  static double kernel61[61];
  static double b_dv1[36];
  static double request19[19];
  static double learning12[12];
  double closed5[5];
  unsigned long long stateTags2_tmp[2];
  /* Initialize function 'gpenmpcNative_canonicalLocalInnerWithAuditStep' input
   * arguments. */
  /* Initialize function input argument 'state64'. */
  /* Initialize function input argument 'stateTags2'. */
  argInit_2x1_uint64_T(stateTags2_tmp);
  /* Initialize function input argument 'input36'. */
  /* Initialize function input argument 'inputTags2'. */
  /* Initialize function input argument 'pending70'. */
  /* Initialize function input argument 'pendingTags2'. */
  /* Call the entry-point 'gpenmpcNative_canonicalLocalInnerWithAuditStep'. */
  argInit_64x1_real_T(b_dv);
  argInit_36x1_real_T(b_dv1);
  argInit_70x1_real_T(dv2);
  gpenmpcNative_canonicalLocalInnerWithAuditStep(
      &c_gpenmpcNative_canonicalLocalIn, b_dv, stateTags2_tmp, b_dv1,
      stateTags2_tmp, dv2, stateTags2_tmp, next64, kernel61, scaffold70,
      request19, closed5, learning12);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_gpenmpcNative_canonicalReferenceTransitionFromJet(void)
{
  struct55_T transition;
  double b_dv[12];
  double previousOuterCorrectionI_tmp[3];
  double progressRate_tmp;
  /* Initialize function 'gpenmpcNative_canonicalReferenceTransitionFromJet'
   * input arguments. */
  /* Initialize function input argument 'trajectoryJet'. */
  progressRate_tmp = argInit_real_T();
  /* Initialize function input argument 'previousOuterCorrectionI'. */
  argInit_3x1_real_T(previousOuterCorrectionI_tmp);
  /* Initialize function input argument 'targetOuterCorrectionF'. */
  /* Call the entry-point 'gpenmpcNative_canonicalReferenceTransitionFromJet'. */
  argInit_3x4_real_T(b_dv);
  gpenmpcNative_canonicalReferenceTransitionFromJet(
      b_dv, progressRate_tmp, progressRate_tmp, progressRate_tmp,
      previousOuterCorrectionI_tmp, previousOuterCorrectionI_tmp,
      progressRate_tmp, progressRate_tmp, &transition);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_gpenmpcNative_queryCanonicalReferenceWindow(void)
{
  static struct51_T r;
  struct52_T next;
  struct52_T r1;
  struct53_T r2;
  struct54_T receipt;
  double jet[12];
  /* Initialize function 'gpenmpcNative_queryCanonicalReferenceWindow' input
   * arguments. */
  /* Initialize function input argument 'window'. */
  /* Initialize function input argument 'state'. */
  /* Initialize function input argument 'request'. */
  /* Call the entry-point 'gpenmpcNative_queryCanonicalReferenceWindow'. */
  argInit_struct51_T(&r);
  r1 = argInit_struct52_T();
  r2 = argInit_struct53_T();
  gpenmpcNative_queryCanonicalReferenceWindow(
      &c_gpenmpcNative_canonicalLocalIn, &r, &r1, &r2, &next, jet, &receipt);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
