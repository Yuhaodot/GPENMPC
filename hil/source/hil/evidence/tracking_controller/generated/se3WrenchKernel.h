/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: se3WrenchKernel.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef SE3WRENCHKERNEL_H
#define SE3WRENCHKERNEL_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
boolean_T
se3WrenchKernel(const double x[19], const double refP[3], const double refV[3],
                const double refA[3], double payload, const double windXY[2],
                const double augmentation[3], const double commandR[9],
                const double commandOmega[3], const double commandOmegaDot[3],
                double wrench[4], double rotor[6], double diagnostic[51]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for se3WrenchKernel.h
 *
 * [EOF]
 */
