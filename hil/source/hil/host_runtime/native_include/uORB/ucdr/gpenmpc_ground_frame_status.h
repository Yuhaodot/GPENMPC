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
#include <uORB/topics/gpenmpc_ground_frame_status.h>


static inline constexpr int ucdr_topic_size_gpenmpc_ground_frame_status()
{
	return 212;
}

static inline bool ucdr_serialize_gpenmpc_ground_frame_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const gpenmpc_ground_frame_status_s& topic = *static_cast<const gpenmpc_ground_frame_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.local_ts) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.local_ts, sizeof(topic.local_ts));
	buf.iterator += sizeof(topic.local_ts);
	buf.offset += sizeof(topic.local_ts);
	static_assert(sizeof(topic.ground_h_ts) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.ground_h_ts, sizeof(topic.ground_h_ts));
	buf.iterator += sizeof(topic.ground_h_ts);
	buf.offset += sizeof(topic.ground_h_ts);
	static_assert(sizeof(topic.ground_v_ts) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.ground_v_ts, sizeof(topic.ground_v_ts));
	buf.iterator += sizeof(topic.ground_v_ts);
	buf.offset += sizeof(topic.ground_v_ts);
	static_assert(sizeof(topic.state) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.state, sizeof(topic.state));
	buf.iterator += sizeof(topic.state);
	buf.offset += sizeof(topic.state);
	static_assert(sizeof(topic.reason) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.reason, sizeof(topic.reason));
	buf.iterator += sizeof(topic.reason);
	buf.offset += sizeof(topic.reason);
	static_assert(sizeof(topic.initialized) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.initialized, sizeof(topic.initialized));
	buf.iterator += sizeof(topic.initialized);
	buf.offset += sizeof(topic.initialized);
	static_assert(sizeof(topic.valid) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.valid, sizeof(topic.valid));
	buf.iterator += sizeof(topic.valid);
	buf.offset += sizeof(topic.valid);
	static_assert(sizeof(topic.fresh) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.fresh, sizeof(topic.fresh));
	buf.iterator += sizeof(topic.fresh);
	buf.offset += sizeof(topic.fresh);
	static_assert(sizeof(topic.hard_fault) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.hard_fault, sizeof(topic.hard_fault));
	buf.iterator += sizeof(topic.hard_fault);
	buf.offset += sizeof(topic.hard_fault);
	static_assert(sizeof(topic.hybrid_transition) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.hybrid_transition, sizeof(topic.hybrid_transition));
	buf.iterator += sizeof(topic.hybrid_transition);
	buf.offset += sizeof(topic.hybrid_transition);
	static_assert(sizeof(topic.innovation_rejected) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.innovation_rejected, sizeof(topic.innovation_rejected));
	buf.iterator += sizeof(topic.innovation_rejected);
	buf.offset += sizeof(topic.innovation_rejected);
	static_assert(sizeof(topic.bias_hold_near_ground) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.bias_hold_near_ground, sizeof(topic.bias_hold_near_ground));
	buf.iterator += sizeof(topic.bias_hold_near_ground);
	buf.offset += sizeof(topic.bias_hold_near_ground);
	static_assert(sizeof(topic.local_z_accepted) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.local_z_accepted, sizeof(topic.local_z_accepted));
	buf.iterator += sizeof(topic.local_z_accepted);
	buf.offset += sizeof(topic.local_z_accepted);
	static_assert(sizeof(topic.local_vz_accepted) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.local_vz_accepted, sizeof(topic.local_vz_accepted));
	buf.iterator += sizeof(topic.local_vz_accepted);
	buf.offset += sizeof(topic.local_vz_accepted);
	static_assert(sizeof(topic.ground_h_accepted) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.ground_h_accepted, sizeof(topic.ground_h_accepted));
	buf.iterator += sizeof(topic.ground_h_accepted);
	buf.offset += sizeof(topic.ground_h_accepted);
	static_assert(sizeof(topic.ground_v_accepted) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.ground_v_accepted, sizeof(topic.ground_v_accepted));
	buf.iterator += sizeof(topic.ground_v_accepted);
	buf.offset += sizeof(topic.ground_v_accepted);
	static_assert(sizeof(topic.alignment_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.alignment_active, sizeof(topic.alignment_active));
	buf.iterator += sizeof(topic.alignment_active);
	buf.offset += sizeof(topic.alignment_active);
	static_assert(sizeof(topic.alignment_uncertainty_ok) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.alignment_uncertainty_ok, sizeof(topic.alignment_uncertainty_ok));
	buf.iterator += sizeof(topic.alignment_uncertainty_ok);
	buf.offset += sizeof(topic.alignment_uncertainty_ok);
	static_assert(sizeof(topic.alignment_hard_fault) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.alignment_hard_fault, sizeof(topic.alignment_hard_fault));
	buf.iterator += sizeof(topic.alignment_hard_fault);
	buf.offset += sizeof(topic.alignment_hard_fault);
	static_assert(sizeof(topic.plant_truth_used) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.plant_truth_used, sizeof(topic.plant_truth_used));
	buf.iterator += sizeof(topic.plant_truth_used);
	buf.offset += sizeof(topic.plant_truth_used);
	static_assert(sizeof(topic.prearm_reason) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_reason, sizeof(topic.prearm_reason));
	buf.iterator += sizeof(topic.prearm_reason);
	buf.offset += sizeof(topic.prearm_reason);
	static_assert(sizeof(topic.prearm_predicate) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_predicate, sizeof(topic.prearm_predicate));
	buf.iterator += sizeof(topic.prearm_predicate);
	buf.offset += sizeof(topic.prearm_predicate);
	static_assert(sizeof(topic.prearm_ready) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_ready, sizeof(topic.prearm_ready));
	buf.iterator += sizeof(topic.prearm_ready);
	buf.offset += sizeof(topic.prearm_ready);
	static_assert(sizeof(topic.prearm_baseline_ok) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_baseline_ok, sizeof(topic.prearm_baseline_ok));
	buf.iterator += sizeof(topic.prearm_baseline_ok);
	buf.offset += sizeof(topic.prearm_baseline_ok);
	static_assert(sizeof(topic.prearm_source_identity_ok) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_source_identity_ok, sizeof(topic.prearm_source_identity_ok));
	buf.iterator += sizeof(topic.prearm_source_identity_ok);
	buf.offset += sizeof(topic.prearm_source_identity_ok);
	static_assert(sizeof(topic.prearm_covariance_ok) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_covariance_ok, sizeof(topic.prearm_covariance_ok));
	buf.iterator += sizeof(topic.prearm_covariance_ok);
	buf.offset += sizeof(topic.prearm_covariance_ok);
	static_assert(sizeof(topic.prearm_position_ok) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_position_ok, sizeof(topic.prearm_position_ok));
	buf.iterator += sizeof(topic.prearm_position_ok);
	buf.offset += sizeof(topic.prearm_position_ok);
	static_assert(sizeof(topic.prearm_velocity_ok) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_velocity_ok, sizeof(topic.prearm_velocity_ok));
	buf.iterator += sizeof(topic.prearm_velocity_ok);
	buf.offset += sizeof(topic.prearm_velocity_ok);
	static_assert(sizeof(topic.prearm_alignment_ok) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_alignment_ok, sizeof(topic.prearm_alignment_ok));
	buf.iterator += sizeof(topic.prearm_alignment_ok);
	buf.offset += sizeof(topic.prearm_alignment_ok);
	static_assert(sizeof(topic.prearm_reset_ok) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_reset_ok, sizeof(topic.prearm_reset_ok));
	buf.iterator += sizeof(topic.prearm_reset_ok);
	buf.offset += sizeof(topic.prearm_reset_ok);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.reset_events) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.reset_events, sizeof(topic.reset_events));
	buf.iterator += sizeof(topic.reset_events);
	buf.offset += sizeof(topic.reset_events);
	static_assert(sizeof(topic.accepted_updates) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.accepted_updates, sizeof(topic.accepted_updates));
	buf.iterator += sizeof(topic.accepted_updates);
	buf.offset += sizeof(topic.accepted_updates);
	static_assert(sizeof(topic.rejected_updates) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.rejected_updates, sizeof(topic.rejected_updates));
	buf.iterator += sizeof(topic.rejected_updates);
	buf.offset += sizeof(topic.rejected_updates);
	static_assert(sizeof(topic.prearm_dwell_reset_count) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_dwell_reset_count, sizeof(topic.prearm_dwell_reset_count));
	buf.iterator += sizeof(topic.prearm_dwell_reset_count);
	buf.offset += sizeof(topic.prearm_dwell_reset_count);
	static_assert(sizeof(topic.prearm_frame_reset_events) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_frame_reset_events, sizeof(topic.prearm_frame_reset_events));
	buf.iterator += sizeof(topic.prearm_frame_reset_events);
	buf.offset += sizeof(topic.prearm_frame_reset_events);
	static_assert(sizeof(topic.prearm_vertical_reset_counter) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_vertical_reset_counter, sizeof(topic.prearm_vertical_reset_counter));
	buf.iterator += sizeof(topic.prearm_vertical_reset_counter);
	buf.offset += sizeof(topic.prearm_vertical_reset_counter);
	buf.iterator += 4; // padding
	buf.offset += 4; // padding
	static_assert(sizeof(topic.prearm_dwell_started) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_dwell_started, sizeof(topic.prearm_dwell_started));
	buf.iterator += sizeof(topic.prearm_dwell_started);
	buf.offset += sizeof(topic.prearm_dwell_started);
	static_assert(sizeof(topic.h_ground_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.h_ground_m, sizeof(topic.h_ground_m));
	buf.iterator += sizeof(topic.h_ground_m);
	buf.offset += sizeof(topic.h_ground_m);
	static_assert(sizeof(topic.v_ground_d_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.v_ground_d_mps, sizeof(topic.v_ground_d_mps));
	buf.iterator += sizeof(topic.v_ground_d_mps);
	buf.offset += sizeof(topic.v_ground_d_mps);
	static_assert(sizeof(topic.b_z_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.b_z_m, sizeof(topic.b_z_m));
	buf.iterator += sizeof(topic.b_z_m);
	buf.offset += sizeof(topic.b_z_m);
	static_assert(sizeof(topic.b_v_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.b_v_mps, sizeof(topic.b_v_mps));
	buf.iterator += sizeof(topic.b_v_mps);
	buf.offset += sizeof(topic.b_v_mps);
	static_assert(sizeof(topic.h_3sigma_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.h_3sigma_m, sizeof(topic.h_3sigma_m));
	buf.iterator += sizeof(topic.h_3sigma_m);
	buf.offset += sizeof(topic.h_3sigma_m);
	static_assert(sizeof(topic.v_3sigma_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.v_3sigma_mps, sizeof(topic.v_3sigma_mps));
	buf.iterator += sizeof(topic.v_3sigma_mps);
	buf.offset += sizeof(topic.v_3sigma_mps);
	static_assert(sizeof(topic.b_z_3sigma_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.b_z_3sigma_m, sizeof(topic.b_z_3sigma_m));
	buf.iterator += sizeof(topic.b_z_3sigma_m);
	buf.offset += sizeof(topic.b_z_3sigma_m);
	static_assert(sizeof(topic.b_v_3sigma_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.b_v_3sigma_mps, sizeof(topic.b_v_3sigma_mps));
	buf.iterator += sizeof(topic.b_v_3sigma_mps);
	buf.offset += sizeof(topic.b_v_3sigma_mps);
	static_assert(sizeof(topic.z_aligned_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.z_aligned_m, sizeof(topic.z_aligned_m));
	buf.iterator += sizeof(topic.z_aligned_m);
	buf.offset += sizeof(topic.z_aligned_m);
	static_assert(sizeof(topic.vz_aligned_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.vz_aligned_mps, sizeof(topic.vz_aligned_mps));
	buf.iterator += sizeof(topic.vz_aligned_mps);
	buf.offset += sizeof(topic.vz_aligned_mps);
	static_assert(sizeof(topic.innovation_nsigma) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.innovation_nsigma, sizeof(topic.innovation_nsigma));
	buf.iterator += sizeof(topic.innovation_nsigma);
	buf.offset += sizeof(topic.innovation_nsigma);
	static_assert(sizeof(topic.local_age_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.local_age_s, sizeof(topic.local_age_s));
	buf.iterator += sizeof(topic.local_age_s);
	buf.offset += sizeof(topic.local_age_s);
	static_assert(sizeof(topic.ground_h_age_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.ground_h_age_s, sizeof(topic.ground_h_age_s));
	buf.iterator += sizeof(topic.ground_h_age_s);
	buf.offset += sizeof(topic.ground_h_age_s);
	static_assert(sizeof(topic.ground_v_age_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.ground_v_age_s, sizeof(topic.ground_v_age_s));
	buf.iterator += sizeof(topic.ground_v_age_s);
	buf.offset += sizeof(topic.ground_v_age_s);
	static_assert(sizeof(topic.b_z_target_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.b_z_target_m, sizeof(topic.b_z_target_m));
	buf.iterator += sizeof(topic.b_z_target_m);
	buf.offset += sizeof(topic.b_z_target_m);
	static_assert(sizeof(topic.b_z_applied_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.b_z_applied_m, sizeof(topic.b_z_applied_m));
	buf.iterator += sizeof(topic.b_z_applied_m);
	buf.offset += sizeof(topic.b_z_applied_m);
	static_assert(sizeof(topic.b_z_rate_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.b_z_rate_mps, sizeof(topic.b_z_rate_mps));
	buf.iterator += sizeof(topic.b_z_rate_mps);
	buf.offset += sizeof(topic.b_z_rate_mps);
	static_assert(sizeof(topic.b_v_target_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.b_v_target_mps, sizeof(topic.b_v_target_mps));
	buf.iterator += sizeof(topic.b_v_target_mps);
	buf.offset += sizeof(topic.b_v_target_mps);
	static_assert(sizeof(topic.b_v_applied_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.b_v_applied_mps, sizeof(topic.b_v_applied_mps));
	buf.iterator += sizeof(topic.b_v_applied_mps);
	buf.offset += sizeof(topic.b_v_applied_mps);
	static_assert(sizeof(topic.b_v_rate_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.b_v_rate_mps2, sizeof(topic.b_v_rate_mps2));
	buf.iterator += sizeof(topic.b_v_rate_mps2);
	buf.offset += sizeof(topic.b_v_rate_mps2);
	static_assert(sizeof(topic.prearm_dwell_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_dwell_s, sizeof(topic.prearm_dwell_s));
	buf.iterator += sizeof(topic.prearm_dwell_s);
	buf.offset += sizeof(topic.prearm_dwell_s);
	static_assert(sizeof(topic.prearm_z_residual_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_z_residual_m, sizeof(topic.prearm_z_residual_m));
	buf.iterator += sizeof(topic.prearm_z_residual_m);
	buf.offset += sizeof(topic.prearm_z_residual_m);
	static_assert(sizeof(topic.prearm_vz_residual_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_vz_residual_mps, sizeof(topic.prearm_vz_residual_mps));
	buf.iterator += sizeof(topic.prearm_vz_residual_mps);
	buf.offset += sizeof(topic.prearm_vz_residual_mps);
	static_assert(sizeof(topic.prearm_z_envelope_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_z_envelope_m, sizeof(topic.prearm_z_envelope_m));
	buf.iterator += sizeof(topic.prearm_z_envelope_m);
	buf.offset += sizeof(topic.prearm_z_envelope_m);
	static_assert(sizeof(topic.prearm_vz_envelope_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_vz_envelope_mps, sizeof(topic.prearm_vz_envelope_mps));
	buf.iterator += sizeof(topic.prearm_vz_envelope_mps);
	buf.offset += sizeof(topic.prearm_vz_envelope_mps);
	static_assert(sizeof(topic.prearm_z_ratio) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_z_ratio, sizeof(topic.prearm_z_ratio));
	buf.iterator += sizeof(topic.prearm_z_ratio);
	buf.offset += sizeof(topic.prearm_z_ratio);
	static_assert(sizeof(topic.prearm_vz_ratio) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_vz_ratio, sizeof(topic.prearm_vz_ratio));
	buf.iterator += sizeof(topic.prearm_vz_ratio);
	buf.offset += sizeof(topic.prearm_vz_ratio);
	static_assert(sizeof(topic.prearm_alignment_z_residual_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_alignment_z_residual_m, sizeof(topic.prearm_alignment_z_residual_m));
	buf.iterator += sizeof(topic.prearm_alignment_z_residual_m);
	buf.offset += sizeof(topic.prearm_alignment_z_residual_m);
	static_assert(sizeof(topic.prearm_alignment_vz_residual_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.prearm_alignment_vz_residual_mps, sizeof(topic.prearm_alignment_vz_residual_mps));
	buf.iterator += sizeof(topic.prearm_alignment_vz_residual_mps);
	buf.offset += sizeof(topic.prearm_alignment_vz_residual_mps);
	return true;
}

static inline bool ucdr_deserialize_gpenmpc_ground_frame_status(ucdrBuffer& buf, gpenmpc_ground_frame_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.local_ts) == 8, "size mismatch");
	memcpy(&topic.local_ts, buf.iterator, sizeof(topic.local_ts));
	buf.iterator += sizeof(topic.local_ts);
	buf.offset += sizeof(topic.local_ts);
	static_assert(sizeof(topic.ground_h_ts) == 8, "size mismatch");
	memcpy(&topic.ground_h_ts, buf.iterator, sizeof(topic.ground_h_ts));
	buf.iterator += sizeof(topic.ground_h_ts);
	buf.offset += sizeof(topic.ground_h_ts);
	static_assert(sizeof(topic.ground_v_ts) == 8, "size mismatch");
	memcpy(&topic.ground_v_ts, buf.iterator, sizeof(topic.ground_v_ts));
	buf.iterator += sizeof(topic.ground_v_ts);
	buf.offset += sizeof(topic.ground_v_ts);
	static_assert(sizeof(topic.state) == 1, "size mismatch");
	memcpy(&topic.state, buf.iterator, sizeof(topic.state));
	buf.iterator += sizeof(topic.state);
	buf.offset += sizeof(topic.state);
	static_assert(sizeof(topic.reason) == 1, "size mismatch");
	memcpy(&topic.reason, buf.iterator, sizeof(topic.reason));
	buf.iterator += sizeof(topic.reason);
	buf.offset += sizeof(topic.reason);
	static_assert(sizeof(topic.initialized) == 1, "size mismatch");
	memcpy(&topic.initialized, buf.iterator, sizeof(topic.initialized));
	buf.iterator += sizeof(topic.initialized);
	buf.offset += sizeof(topic.initialized);
	static_assert(sizeof(topic.valid) == 1, "size mismatch");
	memcpy(&topic.valid, buf.iterator, sizeof(topic.valid));
	buf.iterator += sizeof(topic.valid);
	buf.offset += sizeof(topic.valid);
	static_assert(sizeof(topic.fresh) == 1, "size mismatch");
	memcpy(&topic.fresh, buf.iterator, sizeof(topic.fresh));
	buf.iterator += sizeof(topic.fresh);
	buf.offset += sizeof(topic.fresh);
	static_assert(sizeof(topic.hard_fault) == 1, "size mismatch");
	memcpy(&topic.hard_fault, buf.iterator, sizeof(topic.hard_fault));
	buf.iterator += sizeof(topic.hard_fault);
	buf.offset += sizeof(topic.hard_fault);
	static_assert(sizeof(topic.hybrid_transition) == 1, "size mismatch");
	memcpy(&topic.hybrid_transition, buf.iterator, sizeof(topic.hybrid_transition));
	buf.iterator += sizeof(topic.hybrid_transition);
	buf.offset += sizeof(topic.hybrid_transition);
	static_assert(sizeof(topic.innovation_rejected) == 1, "size mismatch");
	memcpy(&topic.innovation_rejected, buf.iterator, sizeof(topic.innovation_rejected));
	buf.iterator += sizeof(topic.innovation_rejected);
	buf.offset += sizeof(topic.innovation_rejected);
	static_assert(sizeof(topic.bias_hold_near_ground) == 1, "size mismatch");
	memcpy(&topic.bias_hold_near_ground, buf.iterator, sizeof(topic.bias_hold_near_ground));
	buf.iterator += sizeof(topic.bias_hold_near_ground);
	buf.offset += sizeof(topic.bias_hold_near_ground);
	static_assert(sizeof(topic.local_z_accepted) == 1, "size mismatch");
	memcpy(&topic.local_z_accepted, buf.iterator, sizeof(topic.local_z_accepted));
	buf.iterator += sizeof(topic.local_z_accepted);
	buf.offset += sizeof(topic.local_z_accepted);
	static_assert(sizeof(topic.local_vz_accepted) == 1, "size mismatch");
	memcpy(&topic.local_vz_accepted, buf.iterator, sizeof(topic.local_vz_accepted));
	buf.iterator += sizeof(topic.local_vz_accepted);
	buf.offset += sizeof(topic.local_vz_accepted);
	static_assert(sizeof(topic.ground_h_accepted) == 1, "size mismatch");
	memcpy(&topic.ground_h_accepted, buf.iterator, sizeof(topic.ground_h_accepted));
	buf.iterator += sizeof(topic.ground_h_accepted);
	buf.offset += sizeof(topic.ground_h_accepted);
	static_assert(sizeof(topic.ground_v_accepted) == 1, "size mismatch");
	memcpy(&topic.ground_v_accepted, buf.iterator, sizeof(topic.ground_v_accepted));
	buf.iterator += sizeof(topic.ground_v_accepted);
	buf.offset += sizeof(topic.ground_v_accepted);
	static_assert(sizeof(topic.alignment_active) == 1, "size mismatch");
	memcpy(&topic.alignment_active, buf.iterator, sizeof(topic.alignment_active));
	buf.iterator += sizeof(topic.alignment_active);
	buf.offset += sizeof(topic.alignment_active);
	static_assert(sizeof(topic.alignment_uncertainty_ok) == 1, "size mismatch");
	memcpy(&topic.alignment_uncertainty_ok, buf.iterator, sizeof(topic.alignment_uncertainty_ok));
	buf.iterator += sizeof(topic.alignment_uncertainty_ok);
	buf.offset += sizeof(topic.alignment_uncertainty_ok);
	static_assert(sizeof(topic.alignment_hard_fault) == 1, "size mismatch");
	memcpy(&topic.alignment_hard_fault, buf.iterator, sizeof(topic.alignment_hard_fault));
	buf.iterator += sizeof(topic.alignment_hard_fault);
	buf.offset += sizeof(topic.alignment_hard_fault);
	static_assert(sizeof(topic.plant_truth_used) == 1, "size mismatch");
	memcpy(&topic.plant_truth_used, buf.iterator, sizeof(topic.plant_truth_used));
	buf.iterator += sizeof(topic.plant_truth_used);
	buf.offset += sizeof(topic.plant_truth_used);
	static_assert(sizeof(topic.prearm_reason) == 1, "size mismatch");
	memcpy(&topic.prearm_reason, buf.iterator, sizeof(topic.prearm_reason));
	buf.iterator += sizeof(topic.prearm_reason);
	buf.offset += sizeof(topic.prearm_reason);
	static_assert(sizeof(topic.prearm_predicate) == 1, "size mismatch");
	memcpy(&topic.prearm_predicate, buf.iterator, sizeof(topic.prearm_predicate));
	buf.iterator += sizeof(topic.prearm_predicate);
	buf.offset += sizeof(topic.prearm_predicate);
	static_assert(sizeof(topic.prearm_ready) == 1, "size mismatch");
	memcpy(&topic.prearm_ready, buf.iterator, sizeof(topic.prearm_ready));
	buf.iterator += sizeof(topic.prearm_ready);
	buf.offset += sizeof(topic.prearm_ready);
	static_assert(sizeof(topic.prearm_baseline_ok) == 1, "size mismatch");
	memcpy(&topic.prearm_baseline_ok, buf.iterator, sizeof(topic.prearm_baseline_ok));
	buf.iterator += sizeof(topic.prearm_baseline_ok);
	buf.offset += sizeof(topic.prearm_baseline_ok);
	static_assert(sizeof(topic.prearm_source_identity_ok) == 1, "size mismatch");
	memcpy(&topic.prearm_source_identity_ok, buf.iterator, sizeof(topic.prearm_source_identity_ok));
	buf.iterator += sizeof(topic.prearm_source_identity_ok);
	buf.offset += sizeof(topic.prearm_source_identity_ok);
	static_assert(sizeof(topic.prearm_covariance_ok) == 1, "size mismatch");
	memcpy(&topic.prearm_covariance_ok, buf.iterator, sizeof(topic.prearm_covariance_ok));
	buf.iterator += sizeof(topic.prearm_covariance_ok);
	buf.offset += sizeof(topic.prearm_covariance_ok);
	static_assert(sizeof(topic.prearm_position_ok) == 1, "size mismatch");
	memcpy(&topic.prearm_position_ok, buf.iterator, sizeof(topic.prearm_position_ok));
	buf.iterator += sizeof(topic.prearm_position_ok);
	buf.offset += sizeof(topic.prearm_position_ok);
	static_assert(sizeof(topic.prearm_velocity_ok) == 1, "size mismatch");
	memcpy(&topic.prearm_velocity_ok, buf.iterator, sizeof(topic.prearm_velocity_ok));
	buf.iterator += sizeof(topic.prearm_velocity_ok);
	buf.offset += sizeof(topic.prearm_velocity_ok);
	static_assert(sizeof(topic.prearm_alignment_ok) == 1, "size mismatch");
	memcpy(&topic.prearm_alignment_ok, buf.iterator, sizeof(topic.prearm_alignment_ok));
	buf.iterator += sizeof(topic.prearm_alignment_ok);
	buf.offset += sizeof(topic.prearm_alignment_ok);
	static_assert(sizeof(topic.prearm_reset_ok) == 1, "size mismatch");
	memcpy(&topic.prearm_reset_ok, buf.iterator, sizeof(topic.prearm_reset_ok));
	buf.iterator += sizeof(topic.prearm_reset_ok);
	buf.offset += sizeof(topic.prearm_reset_ok);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.reset_events) == 4, "size mismatch");
	memcpy(&topic.reset_events, buf.iterator, sizeof(topic.reset_events));
	buf.iterator += sizeof(topic.reset_events);
	buf.offset += sizeof(topic.reset_events);
	static_assert(sizeof(topic.accepted_updates) == 4, "size mismatch");
	memcpy(&topic.accepted_updates, buf.iterator, sizeof(topic.accepted_updates));
	buf.iterator += sizeof(topic.accepted_updates);
	buf.offset += sizeof(topic.accepted_updates);
	static_assert(sizeof(topic.rejected_updates) == 4, "size mismatch");
	memcpy(&topic.rejected_updates, buf.iterator, sizeof(topic.rejected_updates));
	buf.iterator += sizeof(topic.rejected_updates);
	buf.offset += sizeof(topic.rejected_updates);
	static_assert(sizeof(topic.prearm_dwell_reset_count) == 4, "size mismatch");
	memcpy(&topic.prearm_dwell_reset_count, buf.iterator, sizeof(topic.prearm_dwell_reset_count));
	buf.iterator += sizeof(topic.prearm_dwell_reset_count);
	buf.offset += sizeof(topic.prearm_dwell_reset_count);
	static_assert(sizeof(topic.prearm_frame_reset_events) == 4, "size mismatch");
	memcpy(&topic.prearm_frame_reset_events, buf.iterator, sizeof(topic.prearm_frame_reset_events));
	buf.iterator += sizeof(topic.prearm_frame_reset_events);
	buf.offset += sizeof(topic.prearm_frame_reset_events);
	static_assert(sizeof(topic.prearm_vertical_reset_counter) == 4, "size mismatch");
	memcpy(&topic.prearm_vertical_reset_counter, buf.iterator, sizeof(topic.prearm_vertical_reset_counter));
	buf.iterator += sizeof(topic.prearm_vertical_reset_counter);
	buf.offset += sizeof(topic.prearm_vertical_reset_counter);
	buf.iterator += 4; // padding
	buf.offset += 4; // padding
	static_assert(sizeof(topic.prearm_dwell_started) == 8, "size mismatch");
	memcpy(&topic.prearm_dwell_started, buf.iterator, sizeof(topic.prearm_dwell_started));
	buf.iterator += sizeof(topic.prearm_dwell_started);
	buf.offset += sizeof(topic.prearm_dwell_started);
	static_assert(sizeof(topic.h_ground_m) == 4, "size mismatch");
	memcpy(&topic.h_ground_m, buf.iterator, sizeof(topic.h_ground_m));
	buf.iterator += sizeof(topic.h_ground_m);
	buf.offset += sizeof(topic.h_ground_m);
	static_assert(sizeof(topic.v_ground_d_mps) == 4, "size mismatch");
	memcpy(&topic.v_ground_d_mps, buf.iterator, sizeof(topic.v_ground_d_mps));
	buf.iterator += sizeof(topic.v_ground_d_mps);
	buf.offset += sizeof(topic.v_ground_d_mps);
	static_assert(sizeof(topic.b_z_m) == 4, "size mismatch");
	memcpy(&topic.b_z_m, buf.iterator, sizeof(topic.b_z_m));
	buf.iterator += sizeof(topic.b_z_m);
	buf.offset += sizeof(topic.b_z_m);
	static_assert(sizeof(topic.b_v_mps) == 4, "size mismatch");
	memcpy(&topic.b_v_mps, buf.iterator, sizeof(topic.b_v_mps));
	buf.iterator += sizeof(topic.b_v_mps);
	buf.offset += sizeof(topic.b_v_mps);
	static_assert(sizeof(topic.h_3sigma_m) == 4, "size mismatch");
	memcpy(&topic.h_3sigma_m, buf.iterator, sizeof(topic.h_3sigma_m));
	buf.iterator += sizeof(topic.h_3sigma_m);
	buf.offset += sizeof(topic.h_3sigma_m);
	static_assert(sizeof(topic.v_3sigma_mps) == 4, "size mismatch");
	memcpy(&topic.v_3sigma_mps, buf.iterator, sizeof(topic.v_3sigma_mps));
	buf.iterator += sizeof(topic.v_3sigma_mps);
	buf.offset += sizeof(topic.v_3sigma_mps);
	static_assert(sizeof(topic.b_z_3sigma_m) == 4, "size mismatch");
	memcpy(&topic.b_z_3sigma_m, buf.iterator, sizeof(topic.b_z_3sigma_m));
	buf.iterator += sizeof(topic.b_z_3sigma_m);
	buf.offset += sizeof(topic.b_z_3sigma_m);
	static_assert(sizeof(topic.b_v_3sigma_mps) == 4, "size mismatch");
	memcpy(&topic.b_v_3sigma_mps, buf.iterator, sizeof(topic.b_v_3sigma_mps));
	buf.iterator += sizeof(topic.b_v_3sigma_mps);
	buf.offset += sizeof(topic.b_v_3sigma_mps);
	static_assert(sizeof(topic.z_aligned_m) == 4, "size mismatch");
	memcpy(&topic.z_aligned_m, buf.iterator, sizeof(topic.z_aligned_m));
	buf.iterator += sizeof(topic.z_aligned_m);
	buf.offset += sizeof(topic.z_aligned_m);
	static_assert(sizeof(topic.vz_aligned_mps) == 4, "size mismatch");
	memcpy(&topic.vz_aligned_mps, buf.iterator, sizeof(topic.vz_aligned_mps));
	buf.iterator += sizeof(topic.vz_aligned_mps);
	buf.offset += sizeof(topic.vz_aligned_mps);
	static_assert(sizeof(topic.innovation_nsigma) == 4, "size mismatch");
	memcpy(&topic.innovation_nsigma, buf.iterator, sizeof(topic.innovation_nsigma));
	buf.iterator += sizeof(topic.innovation_nsigma);
	buf.offset += sizeof(topic.innovation_nsigma);
	static_assert(sizeof(topic.local_age_s) == 4, "size mismatch");
	memcpy(&topic.local_age_s, buf.iterator, sizeof(topic.local_age_s));
	buf.iterator += sizeof(topic.local_age_s);
	buf.offset += sizeof(topic.local_age_s);
	static_assert(sizeof(topic.ground_h_age_s) == 4, "size mismatch");
	memcpy(&topic.ground_h_age_s, buf.iterator, sizeof(topic.ground_h_age_s));
	buf.iterator += sizeof(topic.ground_h_age_s);
	buf.offset += sizeof(topic.ground_h_age_s);
	static_assert(sizeof(topic.ground_v_age_s) == 4, "size mismatch");
	memcpy(&topic.ground_v_age_s, buf.iterator, sizeof(topic.ground_v_age_s));
	buf.iterator += sizeof(topic.ground_v_age_s);
	buf.offset += sizeof(topic.ground_v_age_s);
	static_assert(sizeof(topic.b_z_target_m) == 4, "size mismatch");
	memcpy(&topic.b_z_target_m, buf.iterator, sizeof(topic.b_z_target_m));
	buf.iterator += sizeof(topic.b_z_target_m);
	buf.offset += sizeof(topic.b_z_target_m);
	static_assert(sizeof(topic.b_z_applied_m) == 4, "size mismatch");
	memcpy(&topic.b_z_applied_m, buf.iterator, sizeof(topic.b_z_applied_m));
	buf.iterator += sizeof(topic.b_z_applied_m);
	buf.offset += sizeof(topic.b_z_applied_m);
	static_assert(sizeof(topic.b_z_rate_mps) == 4, "size mismatch");
	memcpy(&topic.b_z_rate_mps, buf.iterator, sizeof(topic.b_z_rate_mps));
	buf.iterator += sizeof(topic.b_z_rate_mps);
	buf.offset += sizeof(topic.b_z_rate_mps);
	static_assert(sizeof(topic.b_v_target_mps) == 4, "size mismatch");
	memcpy(&topic.b_v_target_mps, buf.iterator, sizeof(topic.b_v_target_mps));
	buf.iterator += sizeof(topic.b_v_target_mps);
	buf.offset += sizeof(topic.b_v_target_mps);
	static_assert(sizeof(topic.b_v_applied_mps) == 4, "size mismatch");
	memcpy(&topic.b_v_applied_mps, buf.iterator, sizeof(topic.b_v_applied_mps));
	buf.iterator += sizeof(topic.b_v_applied_mps);
	buf.offset += sizeof(topic.b_v_applied_mps);
	static_assert(sizeof(topic.b_v_rate_mps2) == 4, "size mismatch");
	memcpy(&topic.b_v_rate_mps2, buf.iterator, sizeof(topic.b_v_rate_mps2));
	buf.iterator += sizeof(topic.b_v_rate_mps2);
	buf.offset += sizeof(topic.b_v_rate_mps2);
	static_assert(sizeof(topic.prearm_dwell_s) == 4, "size mismatch");
	memcpy(&topic.prearm_dwell_s, buf.iterator, sizeof(topic.prearm_dwell_s));
	buf.iterator += sizeof(topic.prearm_dwell_s);
	buf.offset += sizeof(topic.prearm_dwell_s);
	static_assert(sizeof(topic.prearm_z_residual_m) == 4, "size mismatch");
	memcpy(&topic.prearm_z_residual_m, buf.iterator, sizeof(topic.prearm_z_residual_m));
	buf.iterator += sizeof(topic.prearm_z_residual_m);
	buf.offset += sizeof(topic.prearm_z_residual_m);
	static_assert(sizeof(topic.prearm_vz_residual_mps) == 4, "size mismatch");
	memcpy(&topic.prearm_vz_residual_mps, buf.iterator, sizeof(topic.prearm_vz_residual_mps));
	buf.iterator += sizeof(topic.prearm_vz_residual_mps);
	buf.offset += sizeof(topic.prearm_vz_residual_mps);
	static_assert(sizeof(topic.prearm_z_envelope_m) == 4, "size mismatch");
	memcpy(&topic.prearm_z_envelope_m, buf.iterator, sizeof(topic.prearm_z_envelope_m));
	buf.iterator += sizeof(topic.prearm_z_envelope_m);
	buf.offset += sizeof(topic.prearm_z_envelope_m);
	static_assert(sizeof(topic.prearm_vz_envelope_mps) == 4, "size mismatch");
	memcpy(&topic.prearm_vz_envelope_mps, buf.iterator, sizeof(topic.prearm_vz_envelope_mps));
	buf.iterator += sizeof(topic.prearm_vz_envelope_mps);
	buf.offset += sizeof(topic.prearm_vz_envelope_mps);
	static_assert(sizeof(topic.prearm_z_ratio) == 4, "size mismatch");
	memcpy(&topic.prearm_z_ratio, buf.iterator, sizeof(topic.prearm_z_ratio));
	buf.iterator += sizeof(topic.prearm_z_ratio);
	buf.offset += sizeof(topic.prearm_z_ratio);
	static_assert(sizeof(topic.prearm_vz_ratio) == 4, "size mismatch");
	memcpy(&topic.prearm_vz_ratio, buf.iterator, sizeof(topic.prearm_vz_ratio));
	buf.iterator += sizeof(topic.prearm_vz_ratio);
	buf.offset += sizeof(topic.prearm_vz_ratio);
	static_assert(sizeof(topic.prearm_alignment_z_residual_m) == 4, "size mismatch");
	memcpy(&topic.prearm_alignment_z_residual_m, buf.iterator, sizeof(topic.prearm_alignment_z_residual_m));
	buf.iterator += sizeof(topic.prearm_alignment_z_residual_m);
	buf.offset += sizeof(topic.prearm_alignment_z_residual_m);
	static_assert(sizeof(topic.prearm_alignment_vz_residual_mps) == 4, "size mismatch");
	memcpy(&topic.prearm_alignment_vz_residual_mps, buf.iterator, sizeof(topic.prearm_alignment_vz_residual_mps));
	buf.iterator += sizeof(topic.prearm_alignment_vz_residual_mps);
	buf.offset += sizeof(topic.prearm_alignment_vz_residual_mps);
	return true;
}
