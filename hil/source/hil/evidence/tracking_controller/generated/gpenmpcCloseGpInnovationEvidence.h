/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcCloseGpInnovationEvidence.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCCLOSEGPINNOVATIONEVIDENCE_H
#define GPENMPCCLOSEGPINNOVATIONEVIDENCE_H

/* Include Files */
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_gpenmpcCloseGpInnovationEvidence(const m_struct_T *pending,
                                     const double observedInnovationI[3],
                                     boolean_T observedAvailable,
                                     m_struct_T *evidence);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for gpenmpcCloseGpInnovationEvidence.h
 *
 * [EOF]
 */
