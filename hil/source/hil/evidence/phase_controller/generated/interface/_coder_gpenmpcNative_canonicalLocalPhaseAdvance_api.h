/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_gpenmpcNative_canonicalLocalPhaseAdvance_api.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-06 21:28:59
 */

#ifndef _CODER_GPENMPCNATIVE_CANONICALLOCALPHASEADVANCE_API_H
#define _CODER_GPENMPCNATIVE_CANONICALLOCALPHASEADVANCE_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_gpenmpcNative_canonicalLocalPh(const mxArray *const prhs[1],
                                     const mxArray **plhs);

void gpenmpcNative_canonicalLocalPhaseAdvance(real_T input7[7], real_T next[2]);

void gpenmpcNative_canonicalLocalPhaseAdvance_atexit(void);

void gpenmpcNative_canonicalLocalPhaseAdvance_initialize(void);

void gpenmpcNative_canonicalLocalPhaseAdvance_terminate(void);

void gpenmpcNative_canonicalLocalPhaseAdvance_xil_shutdown(void);

void gpenmpcNative_canonicalLocalPhaseAdvance_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_gpenmpcNative_canonicalLocalPhaseAdvance_api.h
 *
 * [EOF]
 */
