/****************************************************************************
 *
 *   Copyright (C) 2013-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


// auto-generated file

#pragma once

#include <ucdr/microcdr.h>
#include <string.h>
#include <uORB/topics/g_p_e_n_m_p_c_touchdown_governor_status.h>


static inline constexpr int ucdr_topic_size_g_p_e_n_m_p_c_touchdown_governor_status()
{
	return 176;
}

static inline bool ucdr_serialize_g_p_e_n_m_p_c_touchdown_governor_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const g_p_e_n_m_p_c_touchdown_governor_status_s& topic = *static_cast<const g_p_e_n_m_p_c_touchdown_governor_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.frame_status_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.frame_status_timestamp, sizeof(topic.frame_status_timestamp));
	buf.iterator += sizeof(topic.frame_status_timestamp);
	buf.offset += sizeof(topic.frame_status_timestamp);
	static_assert(sizeof(topic.state) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.state, sizeof(topic.state));
	buf.iterator += sizeof(topic.state);
	buf.offset += sizeof(topic.state);
	static_assert(sizeof(topic.eligible) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.eligible, sizeof(topic.eligible));
	buf.iterator += sizeof(topic.eligible);
	buf.offset += sizeof(topic.eligible);
	static_assert(sizeof(topic.active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.active, sizeof(topic.active));
	buf.iterator += sizeof(topic.active);
	buf.offset += sizeof(topic.active);
	static_assert(sizeof(topic.feasible) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.feasible, sizeof(topic.feasible));
	buf.iterator += sizeof(topic.feasible);
	buf.offset += sizeof(topic.feasible);
	static_assert(sizeof(topic.fallback) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.fallback, sizeof(topic.fallback));
	buf.iterator += sizeof(topic.fallback);
	buf.offset += sizeof(topic.fallback);
	static_assert(sizeof(topic.plant_truth_used) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.plant_truth_used, sizeof(topic.plant_truth_used));
	buf.iterator += sizeof(topic.plant_truth_used);
	buf.offset += sizeof(topic.plant_truth_used);
	buf.iterator += 2; // padding
	buf.offset += 2; // padding
	static_assert(sizeof(topic.robust_h_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.robust_h_m, sizeof(topic.robust_h_m));
	buf.iterator += sizeof(topic.robust_h_m);
	buf.offset += sizeof(topic.robust_h_m);
	static_assert(sizeof(topic.robust_v_d_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.robust_v_d_mps, sizeof(topic.robust_v_d_mps));
	buf.iterator += sizeof(topic.robust_v_d_mps);
	buf.offset += sizeof(topic.robust_v_d_mps);
	static_assert(sizeof(topic.robust_touch_limit_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.robust_touch_limit_mps, sizeof(topic.robust_touch_limit_mps));
	buf.iterator += sizeof(topic.robust_touch_limit_mps);
	buf.offset += sizeof(topic.robust_touch_limit_mps);
	static_assert(sizeof(topic.predicted_touch_speed_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.predicted_touch_speed_mps, sizeof(topic.predicted_touch_speed_mps));
	buf.iterator += sizeof(topic.predicted_touch_speed_mps);
	buf.offset += sizeof(topic.predicted_touch_speed_mps);
	static_assert(sizeof(topic.unconstrained_v_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.unconstrained_v_mps, sizeof(topic.unconstrained_v_mps));
	buf.iterator += sizeof(topic.unconstrained_v_mps);
	buf.offset += sizeof(topic.unconstrained_v_mps);
	static_assert(sizeof(topic.constrained_v_target_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.constrained_v_target_mps, sizeof(topic.constrained_v_target_mps));
	buf.iterator += sizeof(topic.constrained_v_target_mps);
	buf.offset += sizeof(topic.constrained_v_target_mps);
	static_assert(sizeof(topic.constrained_v_applied_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.constrained_v_applied_mps, sizeof(topic.constrained_v_applied_mps));
	buf.iterator += sizeof(topic.constrained_v_applied_mps);
	buf.offset += sizeof(topic.constrained_v_applied_mps);
	static_assert(sizeof(topic.local_v_sp_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.local_v_sp_mps, sizeof(topic.local_v_sp_mps));
	buf.iterator += sizeof(topic.local_v_sp_mps);
	buf.offset += sizeof(topic.local_v_sp_mps);
	static_assert(sizeof(topic.local_z_sp_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.local_z_sp_m, sizeof(topic.local_z_sp_m));
	buf.iterator += sizeof(topic.local_z_sp_m);
	buf.offset += sizeof(topic.local_z_sp_m);
	static_assert(sizeof(topic.command_accel_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.command_accel_mps2, sizeof(topic.command_accel_mps2));
	buf.iterator += sizeof(topic.command_accel_mps2);
	buf.offset += sizeof(topic.command_accel_mps2);
	static_assert(sizeof(topic.command_jerk_mps3) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.command_jerk_mps3, sizeof(topic.command_jerk_mps3));
	buf.iterator += sizeof(topic.command_jerk_mps3);
	buf.offset += sizeof(topic.command_jerk_mps3);
	static_assert(sizeof(topic.hover_thrust_estimate) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hover_thrust_estimate, sizeof(topic.hover_thrust_estimate));
	buf.iterator += sizeof(topic.hover_thrust_estimate);
	buf.offset += sizeof(topic.hover_thrust_estimate);
	static_assert(sizeof(topic.hover_thrust_target) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hover_thrust_target, sizeof(topic.hover_thrust_target));
	buf.iterator += sizeof(topic.hover_thrust_target);
	buf.offset += sizeof(topic.hover_thrust_target);
	static_assert(sizeof(topic.hover_thrust_variance) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hover_thrust_variance, sizeof(topic.hover_thrust_variance));
	buf.iterator += sizeof(topic.hover_thrust_variance);
	buf.offset += sizeof(topic.hover_thrust_variance);
	static_assert(sizeof(topic.trim_mismatch_accel_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.trim_mismatch_accel_mps2, sizeof(topic.trim_mismatch_accel_mps2));
	buf.iterator += sizeof(topic.trim_mismatch_accel_mps2);
	buf.offset += sizeof(topic.trim_mismatch_accel_mps2);
	static_assert(sizeof(topic.settling_blend) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.settling_blend, sizeof(topic.settling_blend));
	buf.iterator += sizeof(topic.settling_blend);
	buf.offset += sizeof(topic.settling_blend);
	static_assert(sizeof(topic.settling_accel_target_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.settling_accel_target_mps2, sizeof(topic.settling_accel_target_mps2));
	buf.iterator += sizeof(topic.settling_accel_target_mps2);
	buf.offset += sizeof(topic.settling_accel_target_mps2);
	static_assert(sizeof(topic.settling_accel_applied_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.settling_accel_applied_mps2, sizeof(topic.settling_accel_applied_mps2));
	buf.iterator += sizeof(topic.settling_accel_applied_mps2);
	buf.offset += sizeof(topic.settling_accel_applied_mps2);
	static_assert(sizeof(topic.support_candidate) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.support_candidate, sizeof(topic.support_candidate));
	buf.iterator += sizeof(topic.support_candidate);
	buf.offset += sizeof(topic.support_candidate);
	static_assert(sizeof(topic.support_release_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.support_release_active, sizeof(topic.support_release_active));
	buf.iterator += sizeof(topic.support_release_active);
	buf.offset += sizeof(topic.support_release_active);
	buf.iterator += 2; // padding
	buf.offset += 2; // padding
	static_assert(sizeof(topic.support_dwell_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.support_dwell_s, sizeof(topic.support_dwell_s));
	buf.iterator += sizeof(topic.support_dwell_s);
	buf.offset += sizeof(topic.support_dwell_s);
	static_assert(sizeof(topic.support_dwell_required_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.support_dwell_required_s, sizeof(topic.support_dwell_required_s));
	buf.iterator += sizeof(topic.support_dwell_required_s);
	buf.offset += sizeof(topic.support_dwell_required_s);
	static_assert(sizeof(topic.recovery_latched) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.recovery_latched, sizeof(topic.recovery_latched));
	buf.iterator += sizeof(topic.recovery_latched);
	buf.offset += sizeof(topic.recovery_latched);
	static_assert(sizeof(topic.recovery_completed) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.recovery_completed, sizeof(topic.recovery_completed));
	buf.iterator += sizeof(topic.recovery_completed);
	buf.offset += sizeof(topic.recovery_completed);
	static_assert(sizeof(topic.recovery_clear_candidate) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.recovery_clear_candidate, sizeof(topic.recovery_clear_candidate));
	buf.iterator += sizeof(topic.recovery_clear_candidate);
	buf.offset += sizeof(topic.recovery_clear_candidate);
	static_assert(sizeof(topic.recovery_timed_out) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.recovery_timed_out, sizeof(topic.recovery_timed_out));
	buf.iterator += sizeof(topic.recovery_timed_out);
	buf.offset += sizeof(topic.recovery_timed_out);
	static_assert(sizeof(topic.recovery_clear_dwell_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.recovery_clear_dwell_s, sizeof(topic.recovery_clear_dwell_s));
	buf.iterator += sizeof(topic.recovery_clear_dwell_s);
	buf.offset += sizeof(topic.recovery_clear_dwell_s);
	static_assert(sizeof(topic.recovery_episode_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.recovery_episode_s, sizeof(topic.recovery_episode_s));
	buf.iterator += sizeof(topic.recovery_episode_s);
	buf.offset += sizeof(topic.recovery_episode_s);
	static_assert(sizeof(topic.ground_plane_arrival_latched) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.ground_plane_arrival_latched, sizeof(topic.ground_plane_arrival_latched));
	buf.iterator += sizeof(topic.ground_plane_arrival_latched);
	buf.offset += sizeof(topic.ground_plane_arrival_latched);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.ground_plane_settling_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.ground_plane_settling_s, sizeof(topic.ground_plane_settling_s));
	buf.iterator += sizeof(topic.ground_plane_settling_s);
	buf.offset += sizeof(topic.ground_plane_settling_s);
	static_assert(sizeof(topic.frame_velocity_cancellation_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.frame_velocity_cancellation_active, sizeof(topic.frame_velocity_cancellation_active));
	buf.iterator += sizeof(topic.frame_velocity_cancellation_active);
	buf.offset += sizeof(topic.frame_velocity_cancellation_active);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.frame_velocity_residual_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.frame_velocity_residual_mps, sizeof(topic.frame_velocity_residual_mps));
	buf.iterator += sizeof(topic.frame_velocity_residual_mps);
	buf.offset += sizeof(topic.frame_velocity_residual_mps);
	static_assert(sizeof(topic.frame_velocity_cancellation_target_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.frame_velocity_cancellation_target_mps2, sizeof(topic.frame_velocity_cancellation_target_mps2));
	buf.iterator += sizeof(topic.frame_velocity_cancellation_target_mps2);
	buf.offset += sizeof(topic.frame_velocity_cancellation_target_mps2);
	static_assert(sizeof(topic.pre_touch_velocity_tracking_closure_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.pre_touch_velocity_tracking_closure_active, sizeof(topic.pre_touch_velocity_tracking_closure_active));
	buf.iterator += sizeof(topic.pre_touch_velocity_tracking_closure_active);
	buf.offset += sizeof(topic.pre_touch_velocity_tracking_closure_active);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.pre_touch_velocity_tracking_residual_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.pre_touch_velocity_tracking_residual_mps, sizeof(topic.pre_touch_velocity_tracking_residual_mps));
	buf.iterator += sizeof(topic.pre_touch_velocity_tracking_residual_mps);
	buf.offset += sizeof(topic.pre_touch_velocity_tracking_residual_mps);
	static_assert(sizeof(topic.pre_touch_frame_cancellation_co_ramp_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.pre_touch_frame_cancellation_co_ramp_active, sizeof(topic.pre_touch_frame_cancellation_co_ramp_active));
	buf.iterator += sizeof(topic.pre_touch_frame_cancellation_co_ramp_active);
	buf.offset += sizeof(topic.pre_touch_frame_cancellation_co_ramp_active);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.pre_touch_frame_cancellation_co_ramp_target_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.pre_touch_frame_cancellation_co_ramp_target_mps2, sizeof(topic.pre_touch_frame_cancellation_co_ramp_target_mps2));
	buf.iterator += sizeof(topic.pre_touch_frame_cancellation_co_ramp_target_mps2);
	buf.offset += sizeof(topic.pre_touch_frame_cancellation_co_ramp_target_mps2);
	static_assert(sizeof(topic.constraint_horizon_command_tracking_closure_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.constraint_horizon_command_tracking_closure_active, sizeof(topic.constraint_horizon_command_tracking_closure_active));
	buf.iterator += sizeof(topic.constraint_horizon_command_tracking_closure_active);
	buf.offset += sizeof(topic.constraint_horizon_command_tracking_closure_active);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.constraint_horizon_command_tracking_residual_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.constraint_horizon_command_tracking_residual_mps, sizeof(topic.constraint_horizon_command_tracking_residual_mps));
	buf.iterator += sizeof(topic.constraint_horizon_command_tracking_residual_mps);
	buf.offset += sizeof(topic.constraint_horizon_command_tracking_residual_mps);
	static_assert(sizeof(topic.post_ground_settling_peak_hold_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.post_ground_settling_peak_hold_active, sizeof(topic.post_ground_settling_peak_hold_active));
	buf.iterator += sizeof(topic.post_ground_settling_peak_hold_active);
	buf.offset += sizeof(topic.post_ground_settling_peak_hold_active);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.post_ground_settling_peak_target_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.post_ground_settling_peak_target_mps2, sizeof(topic.post_ground_settling_peak_target_mps2));
	buf.iterator += sizeof(topic.post_ground_settling_peak_target_mps2);
	buf.offset += sizeof(topic.post_ground_settling_peak_target_mps2);
	static_assert(sizeof(topic.horizon_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.horizon_s, sizeof(topic.horizon_s));
	buf.iterator += sizeof(topic.horizon_s);
	buf.offset += sizeof(topic.horizon_s);
	return true;
}

static inline bool ucdr_deserialize_g_p_e_n_m_p_c_touchdown_governor_status(ucdrBuffer& buf, g_p_e_n_m_p_c_touchdown_governor_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.frame_status_timestamp) == 8, "size mismatch");
	memcpy(&topic.frame_status_timestamp, buf.iterator, sizeof(topic.frame_status_timestamp));
	buf.iterator += sizeof(topic.frame_status_timestamp);
	buf.offset += sizeof(topic.frame_status_timestamp);
	static_assert(sizeof(topic.state) == 1, "size mismatch");
	memcpy(&topic.state, buf.iterator, sizeof(topic.state));
	buf.iterator += sizeof(topic.state);
	buf.offset += sizeof(topic.state);
	static_assert(sizeof(topic.eligible) == 1, "size mismatch");
	memcpy(&topic.eligible, buf.iterator, sizeof(topic.eligible));
	buf.iterator += sizeof(topic.eligible);
	buf.offset += sizeof(topic.eligible);
	static_assert(sizeof(topic.active) == 1, "size mismatch");
	memcpy(&topic.active, buf.iterator, sizeof(topic.active));
	buf.iterator += sizeof(topic.active);
	buf.offset += sizeof(topic.active);
	static_assert(sizeof(topic.feasible) == 1, "size mismatch");
	memcpy(&topic.feasible, buf.iterator, sizeof(topic.feasible));
	buf.iterator += sizeof(topic.feasible);
	buf.offset += sizeof(topic.feasible);
	static_assert(sizeof(topic.fallback) == 1, "size mismatch");
	memcpy(&topic.fallback, buf.iterator, sizeof(topic.fallback));
	buf.iterator += sizeof(topic.fallback);
	buf.offset += sizeof(topic.fallback);
	static_assert(sizeof(topic.plant_truth_used) == 1, "size mismatch");
	memcpy(&topic.plant_truth_used, buf.iterator, sizeof(topic.plant_truth_used));
	buf.iterator += sizeof(topic.plant_truth_used);
	buf.offset += sizeof(topic.plant_truth_used);
	buf.iterator += 2; // padding
	buf.offset += 2; // padding
	static_assert(sizeof(topic.robust_h_m) == 4, "size mismatch");
	memcpy(&topic.robust_h_m, buf.iterator, sizeof(topic.robust_h_m));
	buf.iterator += sizeof(topic.robust_h_m);
	buf.offset += sizeof(topic.robust_h_m);
	static_assert(sizeof(topic.robust_v_d_mps) == 4, "size mismatch");
	memcpy(&topic.robust_v_d_mps, buf.iterator, sizeof(topic.robust_v_d_mps));
	buf.iterator += sizeof(topic.robust_v_d_mps);
	buf.offset += sizeof(topic.robust_v_d_mps);
	static_assert(sizeof(topic.robust_touch_limit_mps) == 4, "size mismatch");
	memcpy(&topic.robust_touch_limit_mps, buf.iterator, sizeof(topic.robust_touch_limit_mps));
	buf.iterator += sizeof(topic.robust_touch_limit_mps);
	buf.offset += sizeof(topic.robust_touch_limit_mps);
	static_assert(sizeof(topic.predicted_touch_speed_mps) == 4, "size mismatch");
	memcpy(&topic.predicted_touch_speed_mps, buf.iterator, sizeof(topic.predicted_touch_speed_mps));
	buf.iterator += sizeof(topic.predicted_touch_speed_mps);
	buf.offset += sizeof(topic.predicted_touch_speed_mps);
	static_assert(sizeof(topic.unconstrained_v_mps) == 4, "size mismatch");
	memcpy(&topic.unconstrained_v_mps, buf.iterator, sizeof(topic.unconstrained_v_mps));
	buf.iterator += sizeof(topic.unconstrained_v_mps);
	buf.offset += sizeof(topic.unconstrained_v_mps);
	static_assert(sizeof(topic.constrained_v_target_mps) == 4, "size mismatch");
	memcpy(&topic.constrained_v_target_mps, buf.iterator, sizeof(topic.constrained_v_target_mps));
	buf.iterator += sizeof(topic.constrained_v_target_mps);
	buf.offset += sizeof(topic.constrained_v_target_mps);
	static_assert(sizeof(topic.constrained_v_applied_mps) == 4, "size mismatch");
	memcpy(&topic.constrained_v_applied_mps, buf.iterator, sizeof(topic.constrained_v_applied_mps));
	buf.iterator += sizeof(topic.constrained_v_applied_mps);
	buf.offset += sizeof(topic.constrained_v_applied_mps);
	static_assert(sizeof(topic.local_v_sp_mps) == 4, "size mismatch");
	memcpy(&topic.local_v_sp_mps, buf.iterator, sizeof(topic.local_v_sp_mps));
	buf.iterator += sizeof(topic.local_v_sp_mps);
	buf.offset += sizeof(topic.local_v_sp_mps);
	static_assert(sizeof(topic.local_z_sp_m) == 4, "size mismatch");
	memcpy(&topic.local_z_sp_m, buf.iterator, sizeof(topic.local_z_sp_m));
	buf.iterator += sizeof(topic.local_z_sp_m);
	buf.offset += sizeof(topic.local_z_sp_m);
	static_assert(sizeof(topic.command_accel_mps2) == 4, "size mismatch");
	memcpy(&topic.command_accel_mps2, buf.iterator, sizeof(topic.command_accel_mps2));
	buf.iterator += sizeof(topic.command_accel_mps2);
	buf.offset += sizeof(topic.command_accel_mps2);
	static_assert(sizeof(topic.command_jerk_mps3) == 4, "size mismatch");
	memcpy(&topic.command_jerk_mps3, buf.iterator, sizeof(topic.command_jerk_mps3));
	buf.iterator += sizeof(topic.command_jerk_mps3);
	buf.offset += sizeof(topic.command_jerk_mps3);
	static_assert(sizeof(topic.hover_thrust_estimate) == 4, "size mismatch");
	memcpy(&topic.hover_thrust_estimate, buf.iterator, sizeof(topic.hover_thrust_estimate));
	buf.iterator += sizeof(topic.hover_thrust_estimate);
	buf.offset += sizeof(topic.hover_thrust_estimate);
	static_assert(sizeof(topic.hover_thrust_target) == 4, "size mismatch");
	memcpy(&topic.hover_thrust_target, buf.iterator, sizeof(topic.hover_thrust_target));
	buf.iterator += sizeof(topic.hover_thrust_target);
	buf.offset += sizeof(topic.hover_thrust_target);
	static_assert(sizeof(topic.hover_thrust_variance) == 4, "size mismatch");
	memcpy(&topic.hover_thrust_variance, buf.iterator, sizeof(topic.hover_thrust_variance));
	buf.iterator += sizeof(topic.hover_thrust_variance);
	buf.offset += sizeof(topic.hover_thrust_variance);
	static_assert(sizeof(topic.trim_mismatch_accel_mps2) == 4, "size mismatch");
	memcpy(&topic.trim_mismatch_accel_mps2, buf.iterator, sizeof(topic.trim_mismatch_accel_mps2));
	buf.iterator += sizeof(topic.trim_mismatch_accel_mps2);
	buf.offset += sizeof(topic.trim_mismatch_accel_mps2);
	static_assert(sizeof(topic.settling_blend) == 4, "size mismatch");
	memcpy(&topic.settling_blend, buf.iterator, sizeof(topic.settling_blend));
	buf.iterator += sizeof(topic.settling_blend);
	buf.offset += sizeof(topic.settling_blend);
	static_assert(sizeof(topic.settling_accel_target_mps2) == 4, "size mismatch");
	memcpy(&topic.settling_accel_target_mps2, buf.iterator, sizeof(topic.settling_accel_target_mps2));
	buf.iterator += sizeof(topic.settling_accel_target_mps2);
	buf.offset += sizeof(topic.settling_accel_target_mps2);
	static_assert(sizeof(topic.settling_accel_applied_mps2) == 4, "size mismatch");
	memcpy(&topic.settling_accel_applied_mps2, buf.iterator, sizeof(topic.settling_accel_applied_mps2));
	buf.iterator += sizeof(topic.settling_accel_applied_mps2);
	buf.offset += sizeof(topic.settling_accel_applied_mps2);
	static_assert(sizeof(topic.support_candidate) == 1, "size mismatch");
	memcpy(&topic.support_candidate, buf.iterator, sizeof(topic.support_candidate));
	buf.iterator += sizeof(topic.support_candidate);
	buf.offset += sizeof(topic.support_candidate);
	static_assert(sizeof(topic.support_release_active) == 1, "size mismatch");
	memcpy(&topic.support_release_active, buf.iterator, sizeof(topic.support_release_active));
	buf.iterator += sizeof(topic.support_release_active);
	buf.offset += sizeof(topic.support_release_active);
	buf.iterator += 2; // padding
	buf.offset += 2; // padding
	static_assert(sizeof(topic.support_dwell_s) == 4, "size mismatch");
	memcpy(&topic.support_dwell_s, buf.iterator, sizeof(topic.support_dwell_s));
	buf.iterator += sizeof(topic.support_dwell_s);
	buf.offset += sizeof(topic.support_dwell_s);
	static_assert(sizeof(topic.support_dwell_required_s) == 4, "size mismatch");
	memcpy(&topic.support_dwell_required_s, buf.iterator, sizeof(topic.support_dwell_required_s));
	buf.iterator += sizeof(topic.support_dwell_required_s);
	buf.offset += sizeof(topic.support_dwell_required_s);
	static_assert(sizeof(topic.recovery_latched) == 1, "size mismatch");
	memcpy(&topic.recovery_latched, buf.iterator, sizeof(topic.recovery_latched));
	buf.iterator += sizeof(topic.recovery_latched);
	buf.offset += sizeof(topic.recovery_latched);
	static_assert(sizeof(topic.recovery_completed) == 1, "size mismatch");
	memcpy(&topic.recovery_completed, buf.iterator, sizeof(topic.recovery_completed));
	buf.iterator += sizeof(topic.recovery_completed);
	buf.offset += sizeof(topic.recovery_completed);
	static_assert(sizeof(topic.recovery_clear_candidate) == 1, "size mismatch");
	memcpy(&topic.recovery_clear_candidate, buf.iterator, sizeof(topic.recovery_clear_candidate));
	buf.iterator += sizeof(topic.recovery_clear_candidate);
	buf.offset += sizeof(topic.recovery_clear_candidate);
	static_assert(sizeof(topic.recovery_timed_out) == 1, "size mismatch");
	memcpy(&topic.recovery_timed_out, buf.iterator, sizeof(topic.recovery_timed_out));
	buf.iterator += sizeof(topic.recovery_timed_out);
	buf.offset += sizeof(topic.recovery_timed_out);
	static_assert(sizeof(topic.recovery_clear_dwell_s) == 4, "size mismatch");
	memcpy(&topic.recovery_clear_dwell_s, buf.iterator, sizeof(topic.recovery_clear_dwell_s));
	buf.iterator += sizeof(topic.recovery_clear_dwell_s);
	buf.offset += sizeof(topic.recovery_clear_dwell_s);
	static_assert(sizeof(topic.recovery_episode_s) == 4, "size mismatch");
	memcpy(&topic.recovery_episode_s, buf.iterator, sizeof(topic.recovery_episode_s));
	buf.iterator += sizeof(topic.recovery_episode_s);
	buf.offset += sizeof(topic.recovery_episode_s);
	static_assert(sizeof(topic.ground_plane_arrival_latched) == 1, "size mismatch");
	memcpy(&topic.ground_plane_arrival_latched, buf.iterator, sizeof(topic.ground_plane_arrival_latched));
	buf.iterator += sizeof(topic.ground_plane_arrival_latched);
	buf.offset += sizeof(topic.ground_plane_arrival_latched);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.ground_plane_settling_s) == 4, "size mismatch");
	memcpy(&topic.ground_plane_settling_s, buf.iterator, sizeof(topic.ground_plane_settling_s));
	buf.iterator += sizeof(topic.ground_plane_settling_s);
	buf.offset += sizeof(topic.ground_plane_settling_s);
	static_assert(sizeof(topic.frame_velocity_cancellation_active) == 1, "size mismatch");
	memcpy(&topic.frame_velocity_cancellation_active, buf.iterator, sizeof(topic.frame_velocity_cancellation_active));
	buf.iterator += sizeof(topic.frame_velocity_cancellation_active);
	buf.offset += sizeof(topic.frame_velocity_cancellation_active);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.frame_velocity_residual_mps) == 4, "size mismatch");
	memcpy(&topic.frame_velocity_residual_mps, buf.iterator, sizeof(topic.frame_velocity_residual_mps));
	buf.iterator += sizeof(topic.frame_velocity_residual_mps);
	buf.offset += sizeof(topic.frame_velocity_residual_mps);
	static_assert(sizeof(topic.frame_velocity_cancellation_target_mps2) == 4, "size mismatch");
	memcpy(&topic.frame_velocity_cancellation_target_mps2, buf.iterator, sizeof(topic.frame_velocity_cancellation_target_mps2));
	buf.iterator += sizeof(topic.frame_velocity_cancellation_target_mps2);
	buf.offset += sizeof(topic.frame_velocity_cancellation_target_mps2);
	static_assert(sizeof(topic.pre_touch_velocity_tracking_closure_active) == 1, "size mismatch");
	memcpy(&topic.pre_touch_velocity_tracking_closure_active, buf.iterator, sizeof(topic.pre_touch_velocity_tracking_closure_active));
	buf.iterator += sizeof(topic.pre_touch_velocity_tracking_closure_active);
	buf.offset += sizeof(topic.pre_touch_velocity_tracking_closure_active);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.pre_touch_velocity_tracking_residual_mps) == 4, "size mismatch");
	memcpy(&topic.pre_touch_velocity_tracking_residual_mps, buf.iterator, sizeof(topic.pre_touch_velocity_tracking_residual_mps));
	buf.iterator += sizeof(topic.pre_touch_velocity_tracking_residual_mps);
	buf.offset += sizeof(topic.pre_touch_velocity_tracking_residual_mps);
	static_assert(sizeof(topic.pre_touch_frame_cancellation_co_ramp_active) == 1, "size mismatch");
	memcpy(&topic.pre_touch_frame_cancellation_co_ramp_active, buf.iterator, sizeof(topic.pre_touch_frame_cancellation_co_ramp_active));
	buf.iterator += sizeof(topic.pre_touch_frame_cancellation_co_ramp_active);
	buf.offset += sizeof(topic.pre_touch_frame_cancellation_co_ramp_active);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.pre_touch_frame_cancellation_co_ramp_target_mps2) == 4, "size mismatch");
	memcpy(&topic.pre_touch_frame_cancellation_co_ramp_target_mps2, buf.iterator, sizeof(topic.pre_touch_frame_cancellation_co_ramp_target_mps2));
	buf.iterator += sizeof(topic.pre_touch_frame_cancellation_co_ramp_target_mps2);
	buf.offset += sizeof(topic.pre_touch_frame_cancellation_co_ramp_target_mps2);
	static_assert(sizeof(topic.constraint_horizon_command_tracking_closure_active) == 1, "size mismatch");
	memcpy(&topic.constraint_horizon_command_tracking_closure_active, buf.iterator, sizeof(topic.constraint_horizon_command_tracking_closure_active));
	buf.iterator += sizeof(topic.constraint_horizon_command_tracking_closure_active);
	buf.offset += sizeof(topic.constraint_horizon_command_tracking_closure_active);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.constraint_horizon_command_tracking_residual_mps) == 4, "size mismatch");
	memcpy(&topic.constraint_horizon_command_tracking_residual_mps, buf.iterator, sizeof(topic.constraint_horizon_command_tracking_residual_mps));
	buf.iterator += sizeof(topic.constraint_horizon_command_tracking_residual_mps);
	buf.offset += sizeof(topic.constraint_horizon_command_tracking_residual_mps);
	static_assert(sizeof(topic.post_ground_settling_peak_hold_active) == 1, "size mismatch");
	memcpy(&topic.post_ground_settling_peak_hold_active, buf.iterator, sizeof(topic.post_ground_settling_peak_hold_active));
	buf.iterator += sizeof(topic.post_ground_settling_peak_hold_active);
	buf.offset += sizeof(topic.post_ground_settling_peak_hold_active);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.post_ground_settling_peak_target_mps2) == 4, "size mismatch");
	memcpy(&topic.post_ground_settling_peak_target_mps2, buf.iterator, sizeof(topic.post_ground_settling_peak_target_mps2));
	buf.iterator += sizeof(topic.post_ground_settling_peak_target_mps2);
	buf.offset += sizeof(topic.post_ground_settling_peak_target_mps2);
	static_assert(sizeof(topic.horizon_s) == 4, "size mismatch");
	memcpy(&topic.horizon_s, buf.iterator, sizeof(topic.horizon_s));
	buf.iterator += sizeof(topic.horizon_s);
	buf.offset += sizeof(topic.horizon_s);
	return true;
}
