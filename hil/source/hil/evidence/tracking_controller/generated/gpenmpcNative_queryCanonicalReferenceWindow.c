/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcNative_queryCanonicalReferenceWindow.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "gpenmpcNative_queryCanonicalReferenceWindow.h"
#include "allOrAny.h"
#include "any.h"
#include "diff.h"
#include "interp1.h"
#include "isequal.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_rtwutil.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Pure fixed-array jet query; no trajectory handle or caller state prediction.
 *  REQUEST carries actual requested phase q, asset/leg/window and monotone
 *  query_sequence. A rejected call returns no valid jet and leaves state as-is.
 *  Reason: 0 pass, 1 shape/content, 2 identity, 3 sequence, 4 query, 5 window.
 *  q may be a legitimate FUTURE REFERENCE phase; this grants no future-state,
 *  clock, actuator, or control authority. Phase/jerk transition stays original.
 *
 * Arguments    : e_gpenmpcNative_canonicalLocalIn *SD
 *                const struct51_T *window
 *                const struct52_T *state
 *                const struct53_T *request
 *                struct52_T *next
 *                double jet[12]
 *                struct54_T *receipt
 * Return Type  : void
 */
void gpenmpcNative_queryCanonicalReferenceWindow(
    e_gpenmpcNative_canonicalLocalIn *SD, const struct51_T *window,
    const struct52_T *state, const struct53_T *request, struct52_T *next,
    double jet[12], struct54_T *receipt)
{
  double c_receipt_contents_query_progre;
  double nominalQ;
  double tau;
  unsigned long long receipt_contents_query_sequence;
  int window_size[2];
  unsigned int c_receipt_contents_source_first;
  int i;
  int i1;
  unsigned int qY;
  int varargin_2;
  unsigned char receipt_contents_reason;
  boolean_T c_receipt_contents_analytic_pre;
  boolean_T p;
  boolean_T receipt_contents_accepted;
  *next = *state;
  for (varargin_2 = 0; varargin_2 < 12; varargin_2++) {
    jet[varargin_2] = rtNaN;
  }
  receipt_contents_accepted = false;
  receipt_contents_reason = 1U;
  c_receipt_contents_query_progre = rtNaN;
  nominalQ = rtNaN;
  receipt_contents_query_sequence = 0ULL;
  c_receipt_contents_source_first = 0U;
  qY = 0U;
  c_receipt_contents_analytic_pre = false;
  /*  Same required fields, scalar predicates so MATLAB Coder can resolve the */
  /*  fixed ABI. Coder does not support the equivalent isfield(s,cellstr) form.
   */
  p = (window->schema == 1U);
  if (p && (window->capacity == 256) && (window->row_count >= 2) &&
      (window->row_count <= 256) && any(window->reference_asset_sha256) &&
      (window->leg_index >= 1U) && (window->leg_index <= 5U) &&
      (window->window_generation != 0ULL) &&
      (!rtIsInf(window->nominal_duration_s) &&
       !rtIsNaN(window->nominal_duration_s)) &&
      (!rtIsInf(window->total_duration_s) &&
       !rtIsNaN(window->total_duration_s)) &&
      (!rtIsInf(window->prefix_duration_s) &&
       !rtIsNaN(window->prefix_duration_s)) &&
      (!rtIsInf(window->relaunch_duration_s) &&
       !rtIsNaN(window->relaunch_duration_s))) {
    int loop_ub;
    boolean_T tmp_data[256];
    loop_ub = window->row_count;
    memcpy(&SD->u2.f3.t_data[0], &window->time_s[0],
           (unsigned int)loop_ub * sizeof(double));
    loop_ub = window->row_count;
    for (varargin_2 = 0; varargin_2 < loop_ub; varargin_2++) {
      tau = window->time_s[varargin_2];
      tmp_data[varargin_2] = (rtIsInf(tau) || rtIsNaN(tau));
    }
    if (!b_any(tmp_data, window->row_count)) {
      boolean_T b_tmp_data[255];
      loop_ub = diff(SD->u2.f3.t_data, window->row_count, SD->u2.f3.tmp_data);
      for (varargin_2 = 0; varargin_2 < loop_ub; varargin_2++) {
        b_tmp_data[varargin_2] = (SD->u2.f3.tmp_data[varargin_2] <= 0.0);
      }
      if (!b_any(b_tmp_data, loop_ub)) {
        int b_loop_ub;
        loop_ub = window->row_count;
        b_loop_ub = window->row_count;
        for (varargin_2 = 0; varargin_2 < 4; varargin_2++) {
          for (i = 0; i < 3; i++) {
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              tau = window->nominal_jet[(i1 + (i << 8)) + 768 * varargin_2];
              SD->u2.f3.x_data[(i1 + loop_ub * i) + loop_ub * 3 * varargin_2] =
                  (rtIsInf(tau) || rtIsNaN(tau));
            }
          }
        }
        if (!b_any(SD->u2.f3.x_data, (window->row_count * 3) << 2) &&
            !(window->nominal_duration_s <= 0.0) &&
            !(window->time_s[0] < 0.0)) {
          double b;
          b = window->time_s[window->row_count - 1];
          if (!(b > window->nominal_duration_s) &&
              (window->source_first_row >= 1U) &&
              !(((double)window->source_first_row + (double)window->row_count) -
                    1.0 >
                window->source_total_rows)) {
            if (!isequal(request->reference_asset_sha256,
                         window->reference_asset_sha256) ||
                (request->leg_index != window->leg_index) ||
                (request->window_generation != window->window_generation) ||
                !isequal(state->reference_asset_sha256,
                         window->reference_asset_sha256) ||
                (state->leg_index != window->leg_index) ||
                (state->window_generation != window->window_generation)) {
              receipt_contents_reason = 2U;
            } else if (request->query_sequence <=
                       state->last_accepted_sequence) {
              receipt_contents_reason = 3U;
            } else {
              double q;
              q = request->progress_s;
              if (rtIsInf(request->progress_s) ||
                  rtIsNaN(request->progress_s)) {
                receipt_contents_reason = 4U;
              } else {
                boolean_T bv[12];
                boolean_T guard1;
                boolean_T guard2;
                c_receipt_contents_query_progre = request->progress_s;
                receipt_contents_query_sequence = request->query_sequence;
                guard1 = false;
                guard2 = false;
                if (window->binding_mode == 1) {
                  if (!(window->prefix_duration_s != 25.0) &&
                      !(window->total_duration_s !=
                        window->nominal_duration_s + 25.0)) {
                    for (varargin_2 = 0; varargin_2 < 12; varargin_2++) {
                      tau = window->ground_jet[varargin_2];
                      bv[varargin_2] = (rtIsInf(tau) || rtIsNaN(tau));
                    }
                    if (!vectorAny(bv, 12)) {
                      for (varargin_2 = 0; varargin_2 < 12; varargin_2++) {
                        tau = window->rest_jet[varargin_2];
                        bv[varargin_2] = (rtIsInf(tau) || rtIsNaN(tau));
                      }
                      if (!vectorAny(bv, 12)) {
                        q = fmin(fmax(request->progress_s, 0.0),
                                 window->total_duration_s);
                        if (q < 25.0) {
                          c_receipt_contents_analytic_pre = true;
                          if (q == 0.0) {
                            memcpy(&jet[0], &window->ground_jet[0],
                                   12U * sizeof(double));
                            guard2 = true;
                          } else if (q == 20.0) {
                            memcpy(&jet[0], &window->rest_jet[0],
                                   12U * sizeof(double));
                            guard2 = true;
                          } else {
                            int order;
                            int segment;
                            segment = 0;
                            if (q > 20.0) {
                              segment = 1;
                              q -= 20.0;
                            }
                            order = 0;
                            int exitg1;
                            do {
                              exitg1 = 0;
                              if (order < 4) {
                                boolean_T x_data[24];
                                loop_ub = 7 - order;
                                for (varargin_2 = 0; varargin_2 <= loop_ub;
                                     varargin_2++) {
                                  b_loop_ub = (3 * varargin_2 + 24 * order) +
                                              96 * segment;
                                  tau = window->prefix_coefficients[b_loop_ub];
                                  x_data[3 * varargin_2] =
                                      (rtIsInf(tau) || rtIsNaN(tau));
                                  tau =
                                      window
                                          ->prefix_coefficients[b_loop_ub + 1];
                                  x_data[3 * varargin_2 + 1] =
                                      (rtIsInf(tau) || rtIsNaN(tau));
                                  tau =
                                      window
                                          ->prefix_coefficients[b_loop_ub + 2];
                                  x_data[3 * varargin_2 + 2] =
                                      (rtIsInf(tau) || rtIsNaN(tau));
                                }
                                if (b_any(x_data, 3 * (8 - order))) {
                                  for (varargin_2 = 0; varargin_2 < 12;
                                       varargin_2++) {
                                    jet[varargin_2] = rtNaN;
                                  }
                                  exitg1 = 1;
                                } else {
                                  double d_tmp_data[8];
                                  int jet_tmp;
                                  loop_ub = 7 - order;
                                  for (varargin_2 = 0; varargin_2 <= loop_ub;
                                       varargin_2++) {
                                    d_tmp_data[varargin_2] =
                                        rt_powd_snf(q, varargin_2);
                                  }
                                  loop_ub = 8 - order;
                                  jet[3 * order] = 0.0;
                                  b_loop_ub = 3 * order + 1;
                                  jet[b_loop_ub] = 0.0;
                                  jet_tmp = 3 * order + 2;
                                  jet[jet_tmp] = 0.0;
                                  for (varargin_2 = 0; varargin_2 < loop_ub;
                                       varargin_2++) {
                                    int b_jet_tmp;
                                    b_jet_tmp = (3 * varargin_2 + 24 * order) +
                                                96 * segment;
                                    tau = d_tmp_data[varargin_2];
                                    jet[3 * order] +=
                                        window->prefix_coefficients[b_jet_tmp] *
                                        tau;
                                    jet[b_loop_ub] +=
                                        window->prefix_coefficients[b_jet_tmp +
                                                                    1] *
                                        tau;
                                    jet[jet_tmp] +=
                                        window->prefix_coefficients[b_jet_tmp +
                                                                    2] *
                                        tau;
                                  }
                                  order++;
                                }
                              } else {
                                guard2 = true;
                                exitg1 = 1;
                              }
                            } while (exitg1 == 0);
                          }
                        } else {
                          tau = q - 25.0;
                          /*  Preserve the actual binary64 subtraction; never
                           * snap. */
                          guard1 = true;
                        }
                      }
                    }
                  }
                } else if ((window->binding_mode == 2) &&
                           !(window->relaunch_duration_s != 11.0) &&
                           !(window->total_duration_s !=
                             window->nominal_duration_s)) {
                  boolean_T c_tmp_data[17];
                  c_tmp_data[0] = (rtIsInf(window->relaunch_offset_ned_m[0]) ||
                                   rtIsNaN(window->relaunch_offset_ned_m[0]));
                  c_tmp_data[1] = (rtIsInf(window->relaunch_offset_ned_m[1]) ||
                                   rtIsNaN(window->relaunch_offset_ned_m[1]));
                  c_tmp_data[2] = (rtIsInf(window->relaunch_offset_ned_m[2]) ||
                                   rtIsNaN(window->relaunch_offset_ned_m[2]));
                  if (!vectorAny(c_tmp_data, 3) &&
                      (!rtIsInf(window->vertical_frame_offset_ned_m) &&
                       !rtIsNaN(window->vertical_frame_offset_ned_m))) {
                    tau = request->progress_s;
                    guard1 = true;
                  }
                }
                if (guard2) {
                  for (varargin_2 = 0; varargin_2 < 12; varargin_2++) {
                    tau = jet[varargin_2];
                    bv[varargin_2] = (rtIsInf(tau) || rtIsNaN(tau));
                  }
                  if (vectorAny(bv, 12)) {
                    for (varargin_2 = 0; varargin_2 < 12; varargin_2++) {
                      jet[varargin_2] = rtNaN;
                    }
                  } else {
                    next->last_accepted_sequence = request->query_sequence;
                    receipt_contents_accepted = true;
                    receipt_contents_reason = 0U;
                  }
                }
                if (guard1) {
                  nominalQ = fmin(fmax(tau, 0.0), window->nominal_duration_s);
                  if ((nominalQ < window->time_s[0]) || (nominalQ > b)) {
                    receipt_contents_reason = 5U;
                  } else {
                    loop_ub = window->row_count;
                    window_size[0] = window->row_count;
                    for (varargin_2 = 0; varargin_2 < 4; varargin_2++) {
                      /*  Same four independent linear interpolations on
                       * ORIGINAL nominal rows. */
                      for (i = 0; i < 3; i++) {
                        for (i1 = 0; i1 < loop_ub; i1++) {
                          SD->u2.f3.window_data[i1 + window_size[0] * i] =
                              window->nominal_jet[(i1 + (i << 8)) +
                                                  768 * varargin_2];
                        }
                      }
                      double b_dv[3];
                      interp1(SD, SD->u2.f3.t_data, window->row_count,
                              SD->u2.f3.window_data, window_size, nominalQ,
                              b_dv);
                      jet[3 * varargin_2] = b_dv[0];
                      jet[3 * varargin_2 + 1] = b_dv[1];
                      jet[3 * varargin_2 + 2] = b_dv[2];
                    }
                    c_receipt_contents_source_first = window->source_first_row;
                    qY = (window->source_first_row + window->row_count) - 1U;
                    if (qY < window->source_first_row) {
                      qY = MAX_uint32_T;
                    }
                    if (window->binding_mode == 2) {
                      double duration;
                      duration = window->relaunch_duration_s;
                      tau =
                          fmin(fmax(q / window->relaunch_duration_s, 0.0), 1.0);
                      for (varargin_2 = 0; varargin_2 < 4; varargin_2++) {
                        switch (varargin_2) {
                        case 0:
                          b = 1.0 - (((35.0 * rt_powd_snf(tau, 4.0) -
                                       84.0 * rt_powd_snf(tau, 5.0)) +
                                      70.0 * rt_powd_snf(tau, 6.0)) -
                                     20.0 * rt_powd_snf(tau, 7.0));
                          break;
                        case 1:
                          b = -(((140.0 * rt_powd_snf(tau, 3.0) -
                                  420.0 * rt_powd_snf(tau, 4.0)) +
                                 420.0 * rt_powd_snf(tau, 5.0)) -
                                140.0 * rt_powd_snf(tau, 6.0)) /
                              duration;
                          break;
                        case 2:
                          b = -(((420.0 * (tau * tau) -
                                  1680.0 * rt_powd_snf(tau, 3.0)) +
                                 2100.0 * rt_powd_snf(tau, 4.0)) -
                                840.0 * rt_powd_snf(tau, 5.0)) /
                              (duration * duration);
                          break;
                        default:
                          b = -(((840.0 * tau - 5040.0 * (tau * tau)) +
                                 8400.0 * rt_powd_snf(tau, 3.0)) -
                                4200.0 * rt_powd_snf(tau, 4.0)) /
                              rt_powd_snf(duration, 3.0);
                          break;
                        }
                        jet[3 * varargin_2] +=
                            window->relaunch_offset_ned_m[0] * b;
                        loop_ub = 3 * varargin_2 + 1;
                        jet[loop_ub] += window->relaunch_offset_ned_m[1] * b;
                        loop_ub = 3 * varargin_2 + 2;
                        jet[loop_ub] += window->relaunch_offset_ned_m[2] * b;
                        if (varargin_2 == 0) {
                          jet[2] -= window->vertical_frame_offset_ned_m;
                        }
                      }
                    }
                    for (varargin_2 = 0; varargin_2 < 12; varargin_2++) {
                      tau = jet[varargin_2];
                      bv[varargin_2] = (rtIsInf(tau) || rtIsNaN(tau));
                    }
                    if (vectorAny(bv, 12)) {
                      for (varargin_2 = 0; varargin_2 < 12; varargin_2++) {
                        jet[varargin_2] = rtNaN;
                      }
                    } else {
                      next->last_accepted_sequence = request->query_sequence;
                      receipt_contents_accepted = true;
                      receipt_contents_reason = 0U;
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  receipt->accepted = receipt_contents_accepted;
  receipt->reason = receipt_contents_reason;
  receipt->query_progress_s = c_receipt_contents_query_progre;
  receipt->effective_nominal_progress_s = nominalQ;
  receipt->query_sequence = receipt_contents_query_sequence;
  receipt->source_first_row = c_receipt_contents_source_first;
  receipt->source_last_row = qY;
  receipt->analytic_prefix_used = c_receipt_contents_analytic_pre;
  receipt->reference_resampled = false;
  receipt->hardware_actions = 0U;
}

/*
 * File trailer for gpenmpcNative_queryCanonicalReferenceWindow.c
 *
 * [EOF]
 */
