/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_gpenmpcNative_canonicalLocalInnerWithAuditFirst_api.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef _CODER_GPENMPCNATIVE_CANONICALLOCALINNERWITHAUDITFIRST_API_H
#define _CODER_GPENMPCNATIVE_CANONICALLOCALINNERWITHAUDITFIRST_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Type Definitions */
#ifndef typedef_struct52_T
#define typedef_struct52_T
typedef struct {
  uint8_T reference_asset_sha256[32];
  uint32_T leg_index;
  uint64_T window_generation;
  uint64_T last_accepted_sequence;
} struct52_T;
#endif /* typedef_struct52_T */

#ifndef typedef_struct53_T
#define typedef_struct53_T
typedef struct {
  uint8_T reference_asset_sha256[32];
  uint32_T leg_index;
  uint64_T window_generation;
  uint64_T query_sequence;
  real_T progress_s;
} struct53_T;
#endif /* typedef_struct53_T */

#ifndef typedef_struct54_T
#define typedef_struct54_T
typedef struct {
  boolean_T accepted;
  uint8_T reason;
  real_T query_progress_s;
  real_T effective_nominal_progress_s;
  uint64_T query_sequence;
  uint32_T source_first_row;
  uint32_T source_last_row;
  boolean_T analytic_prefix_used;
  boolean_T reference_resampled;
  uint32_T hardware_actions;
} struct54_T;
#endif /* typedef_struct54_T */

#ifndef typedef_struct56_T
#define typedef_struct56_T
typedef struct {
  real_T position_m[3];
  real_T velocity_mps[3];
  real_T acceleration_mps2[3];
  real_T jerk_mps3[3];
} struct56_T;
#endif /* typedef_struct56_T */

#ifndef typedef_struct51_T
#define typedef_struct51_T
typedef struct {
  uint32_T schema;
  uint16_T capacity;
  uint8_T reference_asset_sha256[32];
  uint32_T leg_index;
  uint64_T window_generation;
  uint32_T source_first_row;
  uint32_T source_total_rows;
  uint16_T row_count;
  real_T time_s[256];
  real_T nominal_jet[3072];
  real_T nominal_duration_s;
  real_T total_duration_s;
  uint8_T binding_mode;
  real_T prefix_duration_s;
  real_T prefix_coefficients[192];
  real_T ground_jet[12];
  real_T rest_jet[12];
  real_T relaunch_duration_s;
  real_T relaunch_offset_ned_m[3];
  real_T vertical_frame_offset_ned_m;
} struct51_T;
#endif /* typedef_struct51_T */

#ifndef typedef_struct55_T
#define typedef_struct55_T
typedef struct {
  struct56_T reference;
  real_T phase_acceleration_s_inv;
  real_T phase_jerk_s_inv2;
  real_T outer_correction_i_mps2[3];
  real_T outer_correction_jerk_i_mps3[3];
  real_T fraction;
  real_T frame_i_from_f[9];
  real_T reference_frame_i_from_f[9];
  real_T reference_curvature;
  real_T reference_signed_yaw_rate;
} struct55_T;
#endif /* typedef_struct55_T */

#ifndef c_typedef_d_gpenmpcNative_queryC
#define c_typedef_d_gpenmpcNative_queryC
typedef struct {
  struct51_T window;
} d_gpenmpcNative_queryCanonicalRe;
#endif /* c_typedef_d_gpenmpcNative_queryC */

#ifndef c_typedef_e_gpenmpcNative_canoni
#define c_typedef_e_gpenmpcNative_canoni
typedef struct {
  d_gpenmpcNative_queryCanonicalRe f0;
} e_gpenmpcNative_canonicalLocalIn;
#endif /* c_typedef_e_gpenmpcNative_canoni */

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_gpenmpcNative_canonicalLocalIn(const mxArray *const prhs[3], int32_T nlhs,
                                     const mxArray *plhs[6]);

void c_gpenmpcNative_canonicalReferen(const mxArray *const prhs[8],
                                     const mxArray **plhs);

void c_gpenmpcNative_queryCanonicalRe(e_gpenmpcNative_canonicalLocalIn *SD,
                                     const mxArray *const prhs[3], int32_T nlhs,
                                     const mxArray *plhs[3]);

void d_gpenmpcNative_canonicalLocalIn(const mxArray *const prhs[7], int32_T nlhs,
                                     const mxArray *plhs[6]);

void gpenmpcNative_canonicalLocalInnerWithAuditFirst(
    const real_T input36[36], const uint64_T inputTags2[2], real_T next64[64],
    real_T kernel61[61], real_T scaffold70[70], real_T request19[19],
    real_T closed5[5], real_T learning12[12]);

void gpenmpcNative_canonicalLocalInnerWithAuditFirst_atexit(void);

void gpenmpcNative_canonicalLocalInnerWithAuditFirst_initialize(void);

void gpenmpcNative_canonicalLocalInnerWithAuditFirst_terminate(void);

void gpenmpcNative_canonicalLocalInnerWithAuditFirst_xil_shutdown(void);

void gpenmpcNative_canonicalLocalInnerWithAuditFirst_xil_terminate(void);

void gpenmpcNative_canonicalLocalInnerWithAuditStep(
    const real_T state64[64], const uint64_T stateTags2[2],
    const real_T input36[36], const uint64_T inputTags2[2],
    const real_T pending70[70], const uint64_T pendingTags2[2],
    real_T next64[64], real_T kernel61[61], real_T scaffold70[70],
    real_T request19[19], real_T closed5[5], real_T learning12[12]);

void gpenmpcNative_canonicalReferenceTransitionFromJet(
    const real_T trajectoryJet[12], real_T progressRate,
    real_T previousPhaseAcceleration, real_T targetPhaseAcceleration,
    const real_T previousOuterCorrectionI[3], real_T targetOuterCorrectionF[3],
    real_T dtS, real_T jerkLimitMps3, struct55_T *transition);

void gpenmpcNative_queryCanonicalReferenceWindow(const struct51_T *window,
                                                const struct52_T *state,
                                                const struct53_T *request,
                                                struct52_T *next,
                                                real_T jet[12],
                                                struct54_T *receipt);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_gpenmpcNative_canonicalLocalInnerWithAuditFirst_api.h
 *
 * [EOF]
 */
