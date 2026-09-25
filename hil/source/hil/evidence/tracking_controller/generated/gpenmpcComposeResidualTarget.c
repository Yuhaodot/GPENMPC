/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcComposeResidualTarget.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcComposeResidualTarget.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_internal_types.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * GPENMPCCOMPOSERESIDUALTARGET Shared prediction/runtime composition.
 *
 *  A1 applies the GP cancellation term physically and assigns the remaining
 *  observer and robust responsibility per axis.  A2 keeps the same GP plant
 *  residual in prediction but uses the exact B1 physical target.
 *
 * Arguments    : const double slidingF[3]
 *                const double c_responsibility_alpha_effectiv[3]
 *                const double c_responsibility_weighted_mean_[3]
 *                const double gpFrameIFromF[9]
 *                const double robustFrameIFromF[9]
 *                const double observerShadowI[3]
 *                h_struct_T *composition
 * Return Type  : void
 */
void c_gpenmpcComposeResidualTarget(
    const double slidingF[3], const double c_responsibility_alpha_effectiv[3],
    const double c_responsibility_weighted_mean_[3],
    const double gpFrameIFromF[9], const double robustFrameIFromF[9],
    const double observerShadowI[3], h_struct_T *composition)
{
  static const double dv2[3] = {0.21698470225998595, 0.18274184577986932,
                                0.4880129590754405};
  static const double dv3[3] = {0.09628312516980447, 0.09921806447104192,
                                0.21544575197142757};
  double b_robustFrameIFromF[3];
  double x[3];
  double d;
  double d1;
  double d2;
  double d3;
  double d4;
  double d5;
  double d6;
  int i;
  d = 0.0;
  d1 = 0.0;
  d2 = 0.0;
  for (i = 0; i < 3; i++) {
    d3 = c_responsibility_weighted_mean_[i];
    d += gpFrameIFromF[3 * i] * d3;
    d1 += gpFrameIFromF[3 * i + 1] * d3;
    d2 += gpFrameIFromF[3 * i + 2] * d3;
  }
  composition->prediction_residual_i_mps2[2] = d2;
  composition->prediction_residual_i_mps2[1] = d1;
  composition->prediction_residual_i_mps2[0] = d;
  d = observerShadowI[2];
  for (i = 0; i < 3; i++) {
    d1 = c_responsibility_alpha_effectiv[i];
    composition->alpha_physical_f[i] = d1;
    composition->rho_applied_f_mps2[i] = (1.0 - d1) * dv2[i] + d1 * dv3[i];
    composition->gp_control_i_mps2[i] =
        -composition->prediction_residual_i_mps2[i];
    b_robustFrameIFromF[i] =
        (1.0 - d1) *
        ((robustFrameIFromF[3 * i] * 0.0 + robustFrameIFromF[3 * i + 1] * 0.0) +
         robustFrameIFromF[3 * i + 2] * d);
    composition->observer_i_mps2[i] = 0.0;
  }
  x[0] = -composition->rho_applied_f_mps2[0] * tanh(slidingF[0] / 0.2);
  x[1] = -composition->rho_applied_f_mps2[1] * tanh(slidingF[1] / 0.2);
  x[2] = -composition->rho_applied_f_mps2[2] * tanh(slidingF[2] / 0.2);
  d1 = composition->observer_i_mps2[0];
  d2 = composition->observer_i_mps2[1];
  d3 = composition->observer_i_mps2[2];
  d4 = 0.0;
  d5 = 0.0;
  d6 = 0.0;
  for (i = 0; i < 3; i++) {
    double d7;
    double d8;
    double d9;
    d7 = robustFrameIFromF[3 * i];
    d = b_robustFrameIFromF[i];
    d1 += d7 * d;
    d8 = robustFrameIFromF[3 * i + 1];
    d2 += d8 * d;
    d9 = robustFrameIFromF[3 * i + 2];
    d3 += d9 * d;
    d = x[i];
    d4 += d7 * d;
    d5 += d8 * d;
    d6 += d9 * d;
  }
  composition->robust_i_mps2[2] = d6;
  composition->robust_i_mps2[1] = d5;
  composition->robust_i_mps2[0] = d4;
  composition->observer_i_mps2[2] = d3;
  composition->observer_i_mps2[1] = d2;
  composition->observer_i_mps2[0] = d1;
  composition->raw_target_i_mps2[0] =
      (d4 + composition->gp_control_i_mps2[0]) + d1;
  composition->raw_target_i_mps2[1] =
      (d5 + composition->gp_control_i_mps2[1]) + d2;
  composition->raw_target_i_mps2[2] =
      (d6 + composition->gp_control_i_mps2[2]) + d3;
}

/*
 * File trailer for gpenmpcComposeResidualTarget.c
 *
 * [EOF]
 */
