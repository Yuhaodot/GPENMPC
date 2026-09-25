/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: isequal.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "isequal.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const unsigned char varargin_1[32]
 *                const unsigned char varargin_2[32]
 * Return Type  : boolean_T
 */
boolean_T isequal(const unsigned char varargin_1[32],
                  const unsigned char varargin_2[32])
{
  int k;
  boolean_T exitg1;
  boolean_T p;
  p = true;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k < 32)) {
    if (varargin_1[k] != varargin_2[k]) {
      p = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return p;
}

/*
 * File trailer for isequal.c
 *
 * [EOF]
 */
