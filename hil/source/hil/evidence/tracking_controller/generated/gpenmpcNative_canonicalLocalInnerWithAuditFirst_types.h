/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

#ifndef GPENMPCNATIVE_CANONICALLOCALINNERWITHAUDITFIRST_TYPES_H
#define GPENMPCNATIVE_CANONICALLOCALINNERWITHAUDITFIRST_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_struct56_T
#define typedef_struct56_T
typedef struct {
  double position_m[3];
  double velocity_mps[3];
  double acceleration_mps2[3];
  double jerk_mps3[3];
} struct56_T;
#endif /* typedef_struct56_T */

#ifndef typedef_struct52_T
#define typedef_struct52_T
typedef struct {
  unsigned char reference_asset_sha256[32];
  unsigned int leg_index;
  unsigned long long window_generation;
  unsigned long long last_accepted_sequence;
} struct52_T;
#endif /* typedef_struct52_T */

#ifndef typedef_struct53_T
#define typedef_struct53_T
typedef struct {
  unsigned char reference_asset_sha256[32];
  unsigned int leg_index;
  unsigned long long window_generation;
  unsigned long long query_sequence;
  double progress_s;
} struct53_T;
#endif /* typedef_struct53_T */

#ifndef typedef_struct54_T
#define typedef_struct54_T
typedef struct {
  boolean_T accepted;
  unsigned char reason;
  double query_progress_s;
  double effective_nominal_progress_s;
  unsigned long long query_sequence;
  unsigned int source_first_row;
  unsigned int source_last_row;
  boolean_T analytic_prefix_used;
  boolean_T reference_resampled;
  unsigned int hardware_actions;
} struct54_T;
#endif /* typedef_struct54_T */

#ifndef typedef_rtString
#define typedef_rtString
typedef struct {
  char Value[39];
} rtString;
#endif /* typedef_rtString */

#ifndef typedef_struct51_T
#define typedef_struct51_T
typedef struct {
  unsigned int schema;
  unsigned short capacity;
  unsigned char reference_asset_sha256[32];
  unsigned int leg_index;
  unsigned long long window_generation;
  unsigned int source_first_row;
  unsigned int source_total_rows;
  unsigned short row_count;
  double time_s[256];
  double nominal_jet[3072];
  double nominal_duration_s;
  double total_duration_s;
  unsigned char binding_mode;
  double prefix_duration_s;
  double prefix_coefficients[192];
  double ground_jet[12];
  double rest_jet[12];
  double relaunch_duration_s;
  double relaunch_offset_ned_m[3];
  double vertical_frame_offset_ned_m;
} struct51_T;
#endif /* typedef_struct51_T */

#ifndef typedef_struct55_T
#define typedef_struct55_T
typedef struct {
  struct56_T reference;
  double phase_acceleration_s_inv;
  double phase_jerk_s_inv2;
  double outer_correction_i_mps2[3];
  double outer_correction_jerk_i_mps3[3];
  double fraction;
  double frame_i_from_f[9];
  double reference_frame_i_from_f[9];
  double reference_curvature;
  double reference_signed_yaw_rate;
} struct55_T;
#endif /* typedef_struct55_T */

#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  double x13[13];
  double rotor_thrust_state_n[6];
  struct56_T reference_up;
  double payload_kg;
  double wind_estimate_xy_mps[2];
  double dt_s;
  double leg_index;
  unsigned long long source_timestamp_ns;
  unsigned long long source_generation;
} struct_T;
#endif /* typedef_struct_T */

#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct {
  boolean_T updated;
  boolean_T reset;
  double residual_z_mps2;
  double estimate_z_mps2;
} b_struct_T;
#endif /* typedef_b_struct_T */

#ifndef struct_emxArray_char_T_1x44
#define struct_emxArray_char_T_1x44
struct emxArray_char_T_1x44 {
  char data[44];
};
#endif /* struct_emxArray_char_T_1x44 */
#ifndef typedef_emxArray_char_T_1x44
#define typedef_emxArray_char_T_1x44
typedef struct emxArray_char_T_1x44 emxArray_char_T_1x44;
#endif /* typedef_emxArray_char_T_1x44 */

#ifndef typedef_b_rtString
#define typedef_b_rtString
typedef struct {
  emxArray_char_T_1x44 Value;
} b_rtString;
#endif /* typedef_b_rtString */

#ifndef typedef_c_struct_T
#define typedef_c_struct_T
typedef struct {
  b_rtString fallback_reason;
  boolean_T exact_b1_fallback;
  double gp_trust;
  double gp_prediction_axis_authority_f[3];
  double gp_physical_axis_authority_f[3];
  double gp_mean_f_mps2[3];
  double gp_frame_i_from_f[9];
  double robust_frame_i_from_f[9];
  double sliding_i_mps[3];
  double sliding_f_mps[3];
  double vertical_observer_shadow_i_mps2[3];
  double target_i_mps2[3];
  double gp_responsibility_blend;
} c_struct_T;
#endif /* typedef_c_struct_T */

#ifndef typedef_d_struct_T
#define typedef_d_struct_T
typedef struct {
  double velocity_up_mps[3];
  double nominal_acceleration_up_mps2[3];
  double frame_i_from_f[9];
  double leg_index;
} d_struct_T;
#endif /* typedef_d_struct_T */

#ifndef typedef_e_struct_T
#define typedef_e_struct_T
typedef struct {
  boolean_T causal_valid;
  double gp_frame_i_from_f[9];
} e_struct_T;
#endif /* typedef_e_struct_T */

#ifndef typedef_f_struct_T
#define typedef_f_struct_T
typedef struct {
  rtString schema;
  boolean_T prediction_required;
  e_struct_T pending;
  double features_f17[17];
  double gp_mean_scale;
} f_struct_T;
#endif /* typedef_f_struct_T */

#ifndef typedef_g_struct_T
#define typedef_g_struct_T
typedef struct {
  boolean_T available;
  boolean_T hard_invalid;
  double trust;
  double features_f17[17];
  double gp_frame_i_from_f[9];
  double predicted_mean_f_mps2[3];
  double calibrated_half_width_f_mps2[3];
  boolean_T observed_innovation_available;
  double observed_innovation_f_mps2[3];
} g_struct_T;
#endif /* typedef_g_struct_T */

#ifndef typedef_b_gpenmpcSo3LogVee
#define typedef_b_gpenmpcSo3LogVee
typedef struct {
  creal_T eigenvectors[9];
  creal_T eigenvalues[9];
} b_gpenmpcSo3LogVee;
#endif /* typedef_b_gpenmpcSo3LogVee */

#ifndef typedef_b_interp1
#define typedef_b_interp1
typedef struct {
  double y_data[768];
  double x_data[256];
} b_interp1;
#endif /* typedef_b_interp1 */

#ifndef c_typedef_c_gpenmpcNative_queryC
#define c_typedef_c_gpenmpcNative_queryC
typedef struct {
  double window_data[768];
  boolean_T x_data[3072];
  double t_data[256];
  double tmp_data[255];
} c_gpenmpcNative_queryCanonicalRe;
#endif /* c_typedef_c_gpenmpcNative_queryC */

#ifndef c_typedef_c_canonicalLocalInner
#define c_typedef_c_canonicalLocalInner
typedef struct {
  double d[51];
  double t18_desired_rotation[9];
} c_canonicalLocalInnerPreControl;
#endif /* c_typedef_c_canonicalLocalInner */

#ifndef typedef_i_struct_T
#define typedef_i_struct_T
typedef struct {
  double filtered_compensation_i_mps2[3];
  double authority_scale;
  double last_tangent_xy[2];
  double vertical_disturbance_ewma_mps2;
  double c_vertical_observer_previous_ve;
  double c_vertical_observer_previous_no;
  double c_vertical_observer_previous_ti;
  double c_vertical_observer_previous_le;
  double c_vertical_observer_previous_pa;
  boolean_T c_vertical_observer_observation;
  boolean_T c_vertical_observer_update_enab;
  double vertical_observer_reset_count;
  double c_vertical_observer_antiwindup_;
  double responsibility_innovation_ratio_ewma_f[3];
  boolean_T gp_responsibility_mode_active;
  double c_gp_responsibility_enter_elapsed;
  double c_gp_responsibility_exit_elapsed;
  double gp_responsibility_blend;
  double responsibility_filtered_gp_mean_f_mps2[3];
  double gp_responsibility_mode_transition_count;
} i_struct_T;
#endif /* typedef_i_struct_T */

#ifndef typedef_j_struct_T
#define typedef_j_struct_T
typedef struct {
  double desired_force_projected_up_n[3];
  double rotor_command_n[6];
  boolean_T rotor_saturated;
  double c_force_projection_norm_mismatc;
} j_struct_T;
#endif /* typedef_j_struct_T */

#ifndef typedef_k_struct_T
#define typedef_k_struct_T
typedef struct {
  double desired_rotation[9];
  double c_desired_angular_velocity_body[3];
  double c_desired_angular_acceleration_[3];
} k_struct_T;
#endif /* typedef_k_struct_T */

#ifndef typedef_l_struct_T
#define typedef_l_struct_T
typedef struct {
  struct_T input;
  double x19[19];
  j_struct_T kernel_control_candidate;
  b_struct_T vertical_observer;
  c_struct_T physical_diagnostic;
  double augmentation_up_mps2[3];
  k_struct_T attitude_command;
  double kernel_output61[61];
} l_struct_T;
#endif /* typedef_l_struct_T */

#ifndef c_typedef_d_gpenmpcUpdateDesired
#define c_typedef_d_gpenmpcUpdateDesired
typedef struct {
  double left[9];
  double a__1[9];
  double right[9];
} d_gpenmpcUpdateDesiredAttitudeCo;
#endif /* c_typedef_d_gpenmpcUpdateDesired */

#ifndef typedef_m_struct_T
#define typedef_m_struct_T
typedef struct {
  b_rtString schema;
  boolean_T read_only;
  boolean_T available;
  boolean_T causal_valid;
  boolean_T gp_model_available;
  boolean_T hard_invalid;
  double trust;
  double minimum_soft_trust;
  double support_distance;
  double latent_variance_max;
  double features_f17[17];
  double gp_frame_i_from_f[9];
  double predicted_mean_f_mps2[3];
  double runtime_weighted_mean_f_mps2[3];
  double calibrated_half_width_f_mps2[3];
  boolean_T observed_innovation_available;
  double observed_innovation_f_mps2[3];
  double innovation_error_f_mps2[3];
  boolean_T observed_innovation_consistent;
  double innovation_consistency_score_f[3];
  boolean_T c_observed_innovation_consisten[3];
} m_struct_T;
#endif /* typedef_m_struct_T */

#ifndef typedef_n_struct_T
#define typedef_n_struct_T
typedef struct {
  boolean_T valid_closed_evidence;
  double raw_weight_f[3];
  double normalized_innovation_error_f[3];
  double c_instantaneous_consistency_sco[3];
  double next_weight_f[3];
  double aggregate_consistency_score;
  boolean_T enter_eligible;
  boolean_T exit_low;
} n_struct_T;
#endif /* typedef_n_struct_T */

#ifndef c_typedef_d_canonicalLocalInner
#define c_typedef_d_canonicalLocalInner
typedef struct {
  m_struct_T closed;
  double d[51];
  n_struct_T consistency;
  double t10_desired_rotation[9];
} d_canonicalLocalInnerPreControl;
#endif /* c_typedef_d_canonicalLocalInner */

#ifndef typedef_o_struct_T
#define typedef_o_struct_T
typedef struct {
  boolean_T initialized;
  boolean_T angular_velocity_valid;
  boolean_T angular_acceleration_valid;
  double filtered_rotation[9];
  double c_desired_angular_velocity_body[3];
  double c_desired_angular_acceleration_[3];
  double update_count;
  double reset_count;
} o_struct_T;
#endif /* typedef_o_struct_T */

#ifndef typedef_p_struct_T
#define typedef_p_struct_T
typedef struct {
  i_struct_T robust_state;
  o_struct_T attitude_continuity_state;
  double residual_history_f_mps2[3];
  double gp_agreement_weight_f[3];
  double previous_rotor_command_n[6];
  double leg_index;
} p_struct_T;
#endif /* typedef_p_struct_T */

#ifndef c_typedef_b_canonicalLocalInner
#define c_typedef_b_canonicalLocalInner
typedef struct {
  l_struct_T out;
  p_struct_T candidate;
  f_struct_T query;
  i_struct_T next_robust_state;
  d_struct_T scaffold;
  double input_x13[13];
  double c_next_attitude_continuity_stat[9];
} b_canonicalLocalInnerFixedAbi;
#endif /* c_typedef_b_canonicalLocalInner */

#ifndef typedef_q_struct_T
#define typedef_q_struct_T
typedef struct {
  struct_T input;
  double x19[19];
  j_struct_T kernel_control_candidate;
  g_struct_T closed_gp_evidence;
  b_struct_T vertical_observer;
  c_struct_T physical_diagnostic;
  double augmentation_up_mps2[3];
  k_struct_T attitude_command;
  double kernel_output61[61];
} q_struct_T;
#endif /* typedef_q_struct_T */

#ifndef typedef_r_struct_T
#define typedef_r_struct_T
typedef struct {
  m_struct_T prediction;
  double velocity_up_mps[3];
  double nominal_acceleration_up_mps2[3];
  double frame_i_from_f[9];
} r_struct_T;
#endif /* typedef_r_struct_T */

#ifndef typedef_s_struct_T
#define typedef_s_struct_T
typedef struct {
  i_struct_T robust_state;
  o_struct_T attitude_continuity_state;
  double residual_history_f_mps2[3];
  double gp_agreement_weight_f[3];
  boolean_T causal_valid;
  double previous_rotor_command_n[6];
  double leg_index;
  unsigned long long last_sample_timestamp_ns;
  unsigned long long source_generation;
} s_struct_T;
#endif /* typedef_s_struct_T */

#ifndef c_typedef_d_gpenmpcNative_canoni
#define c_typedef_d_gpenmpcNative_canoni
typedef struct {
  q_struct_T out;
  r_struct_T expl_temp;
  s_struct_T candidate;
  struct_T b_expl_temp;
  f_struct_T query;
  i_struct_T next_robust_state;
  o_struct_T next_attitude_continuity_state;
  d_struct_T scaffold;
} d_gpenmpcNative_canonicalLocalIn;
#endif /* c_typedef_d_gpenmpcNative_canoni */

#ifndef c_typedef_e_gpenmpcNative_canoni
#define c_typedef_e_gpenmpcNative_canoni
typedef struct {
  union {
    b_gpenmpcSo3LogVee f0;
    b_interp1 f1;
  } u1;
  union {
    d_gpenmpcUpdateDesiredAttitudeCo f2;
    c_gpenmpcNative_queryCanonicalRe f3;
  } u2;
  union {
    c_canonicalLocalInnerPreControl f4;
    d_canonicalLocalInnerPreControl f5;
  } u3;
  union {
    b_canonicalLocalInnerFixedAbi f6;
    d_gpenmpcNative_canonicalLocalIn f7;
  } u4;
} e_gpenmpcNative_canonicalLocalIn;
#endif /* c_typedef_e_gpenmpcNative_canoni */

#endif
/*
 * File trailer for gpenmpcNative_canonicalLocalInnerWithAuditFirst_types.h
 *
 * [EOF]
 */
