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
#include <uORB/topics/gpenmpc_ground_observability_status.h>


static inline constexpr int ucdr_topic_size_gpenmpc_ground_observability_status()
{
	return 160;
}

static inline bool ucdr_serialize_gpenmpc_ground_observability_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const gpenmpc_ground_observability_status_s& topic = *static_cast<const gpenmpc_ground_observability_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.baro_timestamp_sample) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.baro_timestamp_sample, sizeof(topic.baro_timestamp_sample));
	buf.iterator += sizeof(topic.baro_timestamp_sample);
	buf.offset += sizeof(topic.baro_timestamp_sample);
	static_assert(sizeof(topic.gps_timestamp_sample) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.gps_timestamp_sample, sizeof(topic.gps_timestamp_sample));
	buf.iterator += sizeof(topic.gps_timestamp_sample);
	buf.offset += sizeof(topic.gps_timestamp_sample);
	static_assert(sizeof(topic.state) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.state, sizeof(topic.state));
	buf.iterator += sizeof(topic.state);
	buf.offset += sizeof(topic.state);
	static_assert(sizeof(topic.reason) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.reason, sizeof(topic.reason));
	buf.iterator += sizeof(topic.reason);
	buf.offset += sizeof(topic.reason);
	static_assert(sizeof(topic.usable) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.usable, sizeof(topic.usable));
	buf.iterator += sizeof(topic.usable);
	buf.offset += sizeof(topic.usable);
	static_assert(sizeof(topic.baseline_locked) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.baseline_locked, sizeof(topic.baseline_locked));
	buf.iterator += sizeof(topic.baseline_locked);
	buf.offset += sizeof(topic.baseline_locked);
	static_assert(sizeof(topic.safe_baseline_state) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.safe_baseline_state, sizeof(topic.safe_baseline_state));
	buf.iterator += sizeof(topic.safe_baseline_state);
	buf.offset += sizeof(topic.safe_baseline_state);
	static_assert(sizeof(topic.baro_fresh) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.baro_fresh, sizeof(topic.baro_fresh));
	buf.iterator += sizeof(topic.baro_fresh);
	buf.offset += sizeof(topic.baro_fresh);
	static_assert(sizeof(topic.gps_fresh) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.gps_fresh, sizeof(topic.gps_fresh));
	buf.iterator += sizeof(topic.gps_fresh);
	buf.offset += sizeof(topic.gps_fresh);
	static_assert(sizeof(topic.interval_consistent) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.interval_consistent, sizeof(topic.interval_consistent));
	buf.iterator += sizeof(topic.interval_consistent);
	buf.offset += sizeof(topic.interval_consistent);
	static_assert(sizeof(topic.velocity_usable) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.velocity_usable, sizeof(topic.velocity_usable));
	buf.iterator += sizeof(topic.velocity_usable);
	buf.offset += sizeof(topic.velocity_usable);
	static_assert(sizeof(topic.precise_hil_sensor_contract) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.precise_hil_sensor_contract, sizeof(topic.precise_hil_sensor_contract));
	buf.iterator += sizeof(topic.precise_hil_sensor_contract);
	buf.offset += sizeof(topic.precise_hil_sensor_contract);
	static_assert(sizeof(topic.plant_truth_used) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.plant_truth_used, sizeof(topic.plant_truth_used));
	buf.iterator += sizeof(topic.plant_truth_used);
	buf.offset += sizeof(topic.plant_truth_used);
	static_assert(sizeof(topic.reference_alignment_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.reference_alignment_active, sizeof(topic.reference_alignment_active));
	buf.iterator += sizeof(topic.reference_alignment_active);
	buf.offset += sizeof(topic.reference_alignment_active);
	static_assert(sizeof(topic.reference_alignment_hard_fault) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.reference_alignment_hard_fault, sizeof(topic.reference_alignment_hard_fault));
	buf.iterator += sizeof(topic.reference_alignment_hard_fault);
	buf.offset += sizeof(topic.reference_alignment_hard_fault);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.baro_baseline_samples) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.baro_baseline_samples, sizeof(topic.baro_baseline_samples));
	buf.iterator += sizeof(topic.baro_baseline_samples);
	buf.offset += sizeof(topic.baro_baseline_samples);
	static_assert(sizeof(topic.gps_baseline_samples) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.gps_baseline_samples, sizeof(topic.gps_baseline_samples));
	buf.iterator += sizeof(topic.gps_baseline_samples);
	buf.offset += sizeof(topic.gps_baseline_samples);
	static_assert(sizeof(topic.vertical_reset_counter) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.vertical_reset_counter, sizeof(topic.vertical_reset_counter));
	buf.iterator += sizeof(topic.vertical_reset_counter);
	buf.offset += sizeof(topic.vertical_reset_counter);
	static_assert(sizeof(topic.baro_altitude_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.baro_altitude_m, sizeof(topic.baro_altitude_m));
	buf.iterator += sizeof(topic.baro_altitude_m);
	buf.offset += sizeof(topic.baro_altitude_m);
	static_assert(sizeof(topic.gps_altitude_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.gps_altitude_m, sizeof(topic.gps_altitude_m));
	buf.iterator += sizeof(topic.gps_altitude_m);
	buf.offset += sizeof(topic.gps_altitude_m);
	static_assert(sizeof(topic.baro_baseline_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.baro_baseline_m, sizeof(topic.baro_baseline_m));
	buf.iterator += sizeof(topic.baro_baseline_m);
	buf.offset += sizeof(topic.baro_baseline_m);
	static_assert(sizeof(topic.gps_baseline_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.gps_baseline_m, sizeof(topic.gps_baseline_m));
	buf.iterator += sizeof(topic.gps_baseline_m);
	buf.offset += sizeof(topic.gps_baseline_m);
	static_assert(sizeof(topic.baro_relative_height_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.baro_relative_height_m, sizeof(topic.baro_relative_height_m));
	buf.iterator += sizeof(topic.baro_relative_height_m);
	buf.offset += sizeof(topic.baro_relative_height_m);
	static_assert(sizeof(topic.gps_relative_height_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.gps_relative_height_m, sizeof(topic.gps_relative_height_m));
	buf.iterator += sizeof(topic.gps_relative_height_m);
	buf.offset += sizeof(topic.gps_relative_height_m);
	static_assert(sizeof(topic.gps_vertical_velocity_d_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.gps_vertical_velocity_d_mps, sizeof(topic.gps_vertical_velocity_d_mps));
	buf.iterator += sizeof(topic.gps_vertical_velocity_d_mps);
	buf.offset += sizeof(topic.gps_vertical_velocity_d_mps);
	static_assert(sizeof(topic.vertical_velocity_d_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.vertical_velocity_d_mps, sizeof(topic.vertical_velocity_d_mps));
	buf.iterator += sizeof(topic.vertical_velocity_d_mps);
	buf.offset += sizeof(topic.vertical_velocity_d_mps);
	static_assert(sizeof(topic.vertical_velocity_lower_d_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.vertical_velocity_lower_d_mps, sizeof(topic.vertical_velocity_lower_d_mps));
	buf.iterator += sizeof(topic.vertical_velocity_lower_d_mps);
	buf.offset += sizeof(topic.vertical_velocity_lower_d_mps);
	static_assert(sizeof(topic.vertical_velocity_upper_d_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.vertical_velocity_upper_d_mps, sizeof(topic.vertical_velocity_upper_d_mps));
	buf.iterator += sizeof(topic.vertical_velocity_upper_d_mps);
	buf.offset += sizeof(topic.vertical_velocity_upper_d_mps);
	static_assert(sizeof(topic.height_center_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.height_center_m, sizeof(topic.height_center_m));
	buf.iterator += sizeof(topic.height_center_m);
	buf.offset += sizeof(topic.height_center_m);
	static_assert(sizeof(topic.height_lower_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.height_lower_m, sizeof(topic.height_lower_m));
	buf.iterator += sizeof(topic.height_lower_m);
	buf.offset += sizeof(topic.height_lower_m);
	static_assert(sizeof(topic.height_upper_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.height_upper_m, sizeof(topic.height_upper_m));
	buf.iterator += sizeof(topic.height_upper_m);
	buf.offset += sizeof(topic.height_upper_m);
	static_assert(sizeof(topic.baro_age_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.baro_age_s, sizeof(topic.baro_age_s));
	buf.iterator += sizeof(topic.baro_age_s);
	buf.offset += sizeof(topic.baro_age_s);
	static_assert(sizeof(topic.gps_age_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.gps_age_s, sizeof(topic.gps_age_s));
	buf.iterator += sizeof(topic.gps_age_s);
	buf.offset += sizeof(topic.gps_age_s);
	static_assert(sizeof(topic.baro_height_bound_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.baro_height_bound_m, sizeof(topic.baro_height_bound_m));
	buf.iterator += sizeof(topic.baro_height_bound_m);
	buf.offset += sizeof(topic.baro_height_bound_m);
	static_assert(sizeof(topic.gps_height_bound_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.gps_height_bound_m, sizeof(topic.gps_height_bound_m));
	buf.iterator += sizeof(topic.gps_height_bound_m);
	buf.offset += sizeof(topic.gps_height_bound_m);
	static_assert(sizeof(topic.gps_advertised_epv_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.gps_advertised_epv_m, sizeof(topic.gps_advertised_epv_m));
	buf.iterator += sizeof(topic.gps_advertised_epv_m);
	buf.offset += sizeof(topic.gps_advertised_epv_m);
	static_assert(sizeof(topic.gps_vertical_velocity_bound_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.gps_vertical_velocity_bound_mps, sizeof(topic.gps_vertical_velocity_bound_mps));
	buf.iterator += sizeof(topic.gps_vertical_velocity_bound_mps);
	buf.offset += sizeof(topic.gps_vertical_velocity_bound_mps);
	static_assert(sizeof(topic.local_position_d_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.local_position_d_m, sizeof(topic.local_position_d_m));
	buf.iterator += sizeof(topic.local_position_d_m);
	buf.offset += sizeof(topic.local_position_d_m);
	static_assert(sizeof(topic.local_velocity_d_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.local_velocity_d_mps, sizeof(topic.local_velocity_d_mps));
	buf.iterator += sizeof(topic.local_velocity_d_mps);
	buf.offset += sizeof(topic.local_velocity_d_mps);
	static_assert(sizeof(topic.reference_position_offset_target_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.reference_position_offset_target_m, sizeof(topic.reference_position_offset_target_m));
	buf.iterator += sizeof(topic.reference_position_offset_target_m);
	buf.offset += sizeof(topic.reference_position_offset_target_m);
	static_assert(sizeof(topic.reference_position_offset_applied_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.reference_position_offset_applied_m, sizeof(topic.reference_position_offset_applied_m));
	buf.iterator += sizeof(topic.reference_position_offset_applied_m);
	buf.offset += sizeof(topic.reference_position_offset_applied_m);
	static_assert(sizeof(topic.reference_position_offset_rate_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.reference_position_offset_rate_mps, sizeof(topic.reference_position_offset_rate_mps));
	buf.iterator += sizeof(topic.reference_position_offset_rate_mps);
	buf.offset += sizeof(topic.reference_position_offset_rate_mps);
	static_assert(sizeof(topic.reference_velocity_offset_target_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.reference_velocity_offset_target_mps, sizeof(topic.reference_velocity_offset_target_mps));
	buf.iterator += sizeof(topic.reference_velocity_offset_target_mps);
	buf.offset += sizeof(topic.reference_velocity_offset_target_mps);
	static_assert(sizeof(topic.reference_velocity_offset_applied_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.reference_velocity_offset_applied_mps, sizeof(topic.reference_velocity_offset_applied_mps));
	buf.iterator += sizeof(topic.reference_velocity_offset_applied_mps);
	buf.offset += sizeof(topic.reference_velocity_offset_applied_mps);
	static_assert(sizeof(topic.reference_velocity_offset_rate_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.reference_velocity_offset_rate_mps2, sizeof(topic.reference_velocity_offset_rate_mps2));
	buf.iterator += sizeof(topic.reference_velocity_offset_rate_mps2);
	buf.offset += sizeof(topic.reference_velocity_offset_rate_mps2);
	return true;
}

static inline bool ucdr_deserialize_gpenmpc_ground_observability_status(ucdrBuffer& buf, gpenmpc_ground_observability_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.baro_timestamp_sample) == 8, "size mismatch");
	memcpy(&topic.baro_timestamp_sample, buf.iterator, sizeof(topic.baro_timestamp_sample));
	buf.iterator += sizeof(topic.baro_timestamp_sample);
	buf.offset += sizeof(topic.baro_timestamp_sample);
	static_assert(sizeof(topic.gps_timestamp_sample) == 8, "size mismatch");
	memcpy(&topic.gps_timestamp_sample, buf.iterator, sizeof(topic.gps_timestamp_sample));
	buf.iterator += sizeof(topic.gps_timestamp_sample);
	buf.offset += sizeof(topic.gps_timestamp_sample);
	static_assert(sizeof(topic.state) == 1, "size mismatch");
	memcpy(&topic.state, buf.iterator, sizeof(topic.state));
	buf.iterator += sizeof(topic.state);
	buf.offset += sizeof(topic.state);
	static_assert(sizeof(topic.reason) == 1, "size mismatch");
	memcpy(&topic.reason, buf.iterator, sizeof(topic.reason));
	buf.iterator += sizeof(topic.reason);
	buf.offset += sizeof(topic.reason);
	static_assert(sizeof(topic.usable) == 1, "size mismatch");
	memcpy(&topic.usable, buf.iterator, sizeof(topic.usable));
	buf.iterator += sizeof(topic.usable);
	buf.offset += sizeof(topic.usable);
	static_assert(sizeof(topic.baseline_locked) == 1, "size mismatch");
	memcpy(&topic.baseline_locked, buf.iterator, sizeof(topic.baseline_locked));
	buf.iterator += sizeof(topic.baseline_locked);
	buf.offset += sizeof(topic.baseline_locked);
	static_assert(sizeof(topic.safe_baseline_state) == 1, "size mismatch");
	memcpy(&topic.safe_baseline_state, buf.iterator, sizeof(topic.safe_baseline_state));
	buf.iterator += sizeof(topic.safe_baseline_state);
	buf.offset += sizeof(topic.safe_baseline_state);
	static_assert(sizeof(topic.baro_fresh) == 1, "size mismatch");
	memcpy(&topic.baro_fresh, buf.iterator, sizeof(topic.baro_fresh));
	buf.iterator += sizeof(topic.baro_fresh);
	buf.offset += sizeof(topic.baro_fresh);
	static_assert(sizeof(topic.gps_fresh) == 1, "size mismatch");
	memcpy(&topic.gps_fresh, buf.iterator, sizeof(topic.gps_fresh));
	buf.iterator += sizeof(topic.gps_fresh);
	buf.offset += sizeof(topic.gps_fresh);
	static_assert(sizeof(topic.interval_consistent) == 1, "size mismatch");
	memcpy(&topic.interval_consistent, buf.iterator, sizeof(topic.interval_consistent));
	buf.iterator += sizeof(topic.interval_consistent);
	buf.offset += sizeof(topic.interval_consistent);
	static_assert(sizeof(topic.velocity_usable) == 1, "size mismatch");
	memcpy(&topic.velocity_usable, buf.iterator, sizeof(topic.velocity_usable));
	buf.iterator += sizeof(topic.velocity_usable);
	buf.offset += sizeof(topic.velocity_usable);
	static_assert(sizeof(topic.precise_hil_sensor_contract) == 1, "size mismatch");
	memcpy(&topic.precise_hil_sensor_contract, buf.iterator, sizeof(topic.precise_hil_sensor_contract));
	buf.iterator += sizeof(topic.precise_hil_sensor_contract);
	buf.offset += sizeof(topic.precise_hil_sensor_contract);
	static_assert(sizeof(topic.plant_truth_used) == 1, "size mismatch");
	memcpy(&topic.plant_truth_used, buf.iterator, sizeof(topic.plant_truth_used));
	buf.iterator += sizeof(topic.plant_truth_used);
	buf.offset += sizeof(topic.plant_truth_used);
	static_assert(sizeof(topic.reference_alignment_active) == 1, "size mismatch");
	memcpy(&topic.reference_alignment_active, buf.iterator, sizeof(topic.reference_alignment_active));
	buf.iterator += sizeof(topic.reference_alignment_active);
	buf.offset += sizeof(topic.reference_alignment_active);
	static_assert(sizeof(topic.reference_alignment_hard_fault) == 1, "size mismatch");
	memcpy(&topic.reference_alignment_hard_fault, buf.iterator, sizeof(topic.reference_alignment_hard_fault));
	buf.iterator += sizeof(topic.reference_alignment_hard_fault);
	buf.offset += sizeof(topic.reference_alignment_hard_fault);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.baro_baseline_samples) == 4, "size mismatch");
	memcpy(&topic.baro_baseline_samples, buf.iterator, sizeof(topic.baro_baseline_samples));
	buf.iterator += sizeof(topic.baro_baseline_samples);
	buf.offset += sizeof(topic.baro_baseline_samples);
	static_assert(sizeof(topic.gps_baseline_samples) == 4, "size mismatch");
	memcpy(&topic.gps_baseline_samples, buf.iterator, sizeof(topic.gps_baseline_samples));
	buf.iterator += sizeof(topic.gps_baseline_samples);
	buf.offset += sizeof(topic.gps_baseline_samples);
	static_assert(sizeof(topic.vertical_reset_counter) == 4, "size mismatch");
	memcpy(&topic.vertical_reset_counter, buf.iterator, sizeof(topic.vertical_reset_counter));
	buf.iterator += sizeof(topic.vertical_reset_counter);
	buf.offset += sizeof(topic.vertical_reset_counter);
	static_assert(sizeof(topic.baro_altitude_m) == 4, "size mismatch");
	memcpy(&topic.baro_altitude_m, buf.iterator, sizeof(topic.baro_altitude_m));
	buf.iterator += sizeof(topic.baro_altitude_m);
	buf.offset += sizeof(topic.baro_altitude_m);
	static_assert(sizeof(topic.gps_altitude_m) == 4, "size mismatch");
	memcpy(&topic.gps_altitude_m, buf.iterator, sizeof(topic.gps_altitude_m));
	buf.iterator += sizeof(topic.gps_altitude_m);
	buf.offset += sizeof(topic.gps_altitude_m);
	static_assert(sizeof(topic.baro_baseline_m) == 4, "size mismatch");
	memcpy(&topic.baro_baseline_m, buf.iterator, sizeof(topic.baro_baseline_m));
	buf.iterator += sizeof(topic.baro_baseline_m);
	buf.offset += sizeof(topic.baro_baseline_m);
	static_assert(sizeof(topic.gps_baseline_m) == 4, "size mismatch");
	memcpy(&topic.gps_baseline_m, buf.iterator, sizeof(topic.gps_baseline_m));
	buf.iterator += sizeof(topic.gps_baseline_m);
	buf.offset += sizeof(topic.gps_baseline_m);
	static_assert(sizeof(topic.baro_relative_height_m) == 4, "size mismatch");
	memcpy(&topic.baro_relative_height_m, buf.iterator, sizeof(topic.baro_relative_height_m));
	buf.iterator += sizeof(topic.baro_relative_height_m);
	buf.offset += sizeof(topic.baro_relative_height_m);
	static_assert(sizeof(topic.gps_relative_height_m) == 4, "size mismatch");
	memcpy(&topic.gps_relative_height_m, buf.iterator, sizeof(topic.gps_relative_height_m));
	buf.iterator += sizeof(topic.gps_relative_height_m);
	buf.offset += sizeof(topic.gps_relative_height_m);
	static_assert(sizeof(topic.gps_vertical_velocity_d_mps) == 4, "size mismatch");
	memcpy(&topic.gps_vertical_velocity_d_mps, buf.iterator, sizeof(topic.gps_vertical_velocity_d_mps));
	buf.iterator += sizeof(topic.gps_vertical_velocity_d_mps);
	buf.offset += sizeof(topic.gps_vertical_velocity_d_mps);
	static_assert(sizeof(topic.vertical_velocity_d_mps) == 4, "size mismatch");
	memcpy(&topic.vertical_velocity_d_mps, buf.iterator, sizeof(topic.vertical_velocity_d_mps));
	buf.iterator += sizeof(topic.vertical_velocity_d_mps);
	buf.offset += sizeof(topic.vertical_velocity_d_mps);
	static_assert(sizeof(topic.vertical_velocity_lower_d_mps) == 4, "size mismatch");
	memcpy(&topic.vertical_velocity_lower_d_mps, buf.iterator, sizeof(topic.vertical_velocity_lower_d_mps));
	buf.iterator += sizeof(topic.vertical_velocity_lower_d_mps);
	buf.offset += sizeof(topic.vertical_velocity_lower_d_mps);
	static_assert(sizeof(topic.vertical_velocity_upper_d_mps) == 4, "size mismatch");
	memcpy(&topic.vertical_velocity_upper_d_mps, buf.iterator, sizeof(topic.vertical_velocity_upper_d_mps));
	buf.iterator += sizeof(topic.vertical_velocity_upper_d_mps);
	buf.offset += sizeof(topic.vertical_velocity_upper_d_mps);
	static_assert(sizeof(topic.height_center_m) == 4, "size mismatch");
	memcpy(&topic.height_center_m, buf.iterator, sizeof(topic.height_center_m));
	buf.iterator += sizeof(topic.height_center_m);
	buf.offset += sizeof(topic.height_center_m);
	static_assert(sizeof(topic.height_lower_m) == 4, "size mismatch");
	memcpy(&topic.height_lower_m, buf.iterator, sizeof(topic.height_lower_m));
	buf.iterator += sizeof(topic.height_lower_m);
	buf.offset += sizeof(topic.height_lower_m);
	static_assert(sizeof(topic.height_upper_m) == 4, "size mismatch");
	memcpy(&topic.height_upper_m, buf.iterator, sizeof(topic.height_upper_m));
	buf.iterator += sizeof(topic.height_upper_m);
	buf.offset += sizeof(topic.height_upper_m);
	static_assert(sizeof(topic.baro_age_s) == 4, "size mismatch");
	memcpy(&topic.baro_age_s, buf.iterator, sizeof(topic.baro_age_s));
	buf.iterator += sizeof(topic.baro_age_s);
	buf.offset += sizeof(topic.baro_age_s);
	static_assert(sizeof(topic.gps_age_s) == 4, "size mismatch");
	memcpy(&topic.gps_age_s, buf.iterator, sizeof(topic.gps_age_s));
	buf.iterator += sizeof(topic.gps_age_s);
	buf.offset += sizeof(topic.gps_age_s);
	static_assert(sizeof(topic.baro_height_bound_m) == 4, "size mismatch");
	memcpy(&topic.baro_height_bound_m, buf.iterator, sizeof(topic.baro_height_bound_m));
	buf.iterator += sizeof(topic.baro_height_bound_m);
	buf.offset += sizeof(topic.baro_height_bound_m);
	static_assert(sizeof(topic.gps_height_bound_m) == 4, "size mismatch");
	memcpy(&topic.gps_height_bound_m, buf.iterator, sizeof(topic.gps_height_bound_m));
	buf.iterator += sizeof(topic.gps_height_bound_m);
	buf.offset += sizeof(topic.gps_height_bound_m);
	static_assert(sizeof(topic.gps_advertised_epv_m) == 4, "size mismatch");
	memcpy(&topic.gps_advertised_epv_m, buf.iterator, sizeof(topic.gps_advertised_epv_m));
	buf.iterator += sizeof(topic.gps_advertised_epv_m);
	buf.offset += sizeof(topic.gps_advertised_epv_m);
	static_assert(sizeof(topic.gps_vertical_velocity_bound_mps) == 4, "size mismatch");
	memcpy(&topic.gps_vertical_velocity_bound_mps, buf.iterator, sizeof(topic.gps_vertical_velocity_bound_mps));
	buf.iterator += sizeof(topic.gps_vertical_velocity_bound_mps);
	buf.offset += sizeof(topic.gps_vertical_velocity_bound_mps);
	static_assert(sizeof(topic.local_position_d_m) == 4, "size mismatch");
	memcpy(&topic.local_position_d_m, buf.iterator, sizeof(topic.local_position_d_m));
	buf.iterator += sizeof(topic.local_position_d_m);
	buf.offset += sizeof(topic.local_position_d_m);
	static_assert(sizeof(topic.local_velocity_d_mps) == 4, "size mismatch");
	memcpy(&topic.local_velocity_d_mps, buf.iterator, sizeof(topic.local_velocity_d_mps));
	buf.iterator += sizeof(topic.local_velocity_d_mps);
	buf.offset += sizeof(topic.local_velocity_d_mps);
	static_assert(sizeof(topic.reference_position_offset_target_m) == 4, "size mismatch");
	memcpy(&topic.reference_position_offset_target_m, buf.iterator, sizeof(topic.reference_position_offset_target_m));
	buf.iterator += sizeof(topic.reference_position_offset_target_m);
	buf.offset += sizeof(topic.reference_position_offset_target_m);
	static_assert(sizeof(topic.reference_position_offset_applied_m) == 4, "size mismatch");
	memcpy(&topic.reference_position_offset_applied_m, buf.iterator, sizeof(topic.reference_position_offset_applied_m));
	buf.iterator += sizeof(topic.reference_position_offset_applied_m);
	buf.offset += sizeof(topic.reference_position_offset_applied_m);
	static_assert(sizeof(topic.reference_position_offset_rate_mps) == 4, "size mismatch");
	memcpy(&topic.reference_position_offset_rate_mps, buf.iterator, sizeof(topic.reference_position_offset_rate_mps));
	buf.iterator += sizeof(topic.reference_position_offset_rate_mps);
	buf.offset += sizeof(topic.reference_position_offset_rate_mps);
	static_assert(sizeof(topic.reference_velocity_offset_target_mps) == 4, "size mismatch");
	memcpy(&topic.reference_velocity_offset_target_mps, buf.iterator, sizeof(topic.reference_velocity_offset_target_mps));
	buf.iterator += sizeof(topic.reference_velocity_offset_target_mps);
	buf.offset += sizeof(topic.reference_velocity_offset_target_mps);
	static_assert(sizeof(topic.reference_velocity_offset_applied_mps) == 4, "size mismatch");
	memcpy(&topic.reference_velocity_offset_applied_mps, buf.iterator, sizeof(topic.reference_velocity_offset_applied_mps));
	buf.iterator += sizeof(topic.reference_velocity_offset_applied_mps);
	buf.offset += sizeof(topic.reference_velocity_offset_applied_mps);
	static_assert(sizeof(topic.reference_velocity_offset_rate_mps2) == 4, "size mismatch");
	memcpy(&topic.reference_velocity_offset_rate_mps2, buf.iterator, sizeof(topic.reference_velocity_offset_rate_mps2));
	buf.iterator += sizeof(topic.reference_velocity_offset_rate_mps2);
	buf.offset += sizeof(topic.reference_velocity_offset_rate_mps2);
	return true;
}
