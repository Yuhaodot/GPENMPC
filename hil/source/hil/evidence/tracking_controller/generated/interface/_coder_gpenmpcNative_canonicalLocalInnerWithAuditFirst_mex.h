/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_gpenmpcNative_canonicalLocalInnerWithAuditFirst_mex.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef _CODER_GPENMPCNATIVE_CANONICALLOCALINNERWITHAUDITFIRST_MEX_H
#define _CODER_GPENMPCNATIVE_CANONICALLOCALINNERWITHAUDITFIRST_MEX_H

/* Include Files */
#include "_coder_gpenmpcNative_canonicalLocalInnerWithAuditFirst_api.h"
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void unsafe_gpenmpcNative_canonicalLocalInnerWithAuditFirst_mexFunction(
    int32_T nlhs, mxArray *plhs[6], int32_T nrhs, const mxArray *prhs[3]);

void unsafe_gpenmpcNative_canonicalLocalInnerWithAuditStep_mexFunction(
    int32_T nlhs, mxArray *plhs[6], int32_T nrhs, const mxArray *prhs[7]);

void unsafe_gpenmpcNative_canonicalReferenceTransitionFromJet_mexFunction(
    int32_T nlhs, mxArray *plhs[1], int32_T nrhs, const mxArray *prhs[8]);

void unsafe_gpenmpcNative_queryCanonicalReferenceWindow_mexFunction(
    e_gpenmpcNative_canonicalLocalIn *SD, int32_T nlhs, mxArray *plhs[3],
    int32_T nrhs, const mxArray *prhs[3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_gpenmpcNative_canonicalLocalInnerWithAuditFirst_mex.h
 *
 * [EOF]
 */
