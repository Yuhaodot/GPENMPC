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
#include <uORB/topics/gpenmpc_se3_control_status.h>


static inline constexpr int ucdr_topic_size_gpenmpc_se3_control_status()
{
	return 224;
}

static inline bool ucdr_serialize_gpenmpc_se3_control_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const gpenmpc_se3_control_status_s& topic = *static_cast<const gpenmpc_se3_control_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.sample_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.sample_timestamp, sizeof(topic.sample_timestamp));
	buf.iterator += sizeof(topic.sample_timestamp);
	buf.offset += sizeof(topic.sample_timestamp);
	static_assert(sizeof(topic.reference_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.reference_timestamp, sizeof(topic.reference_timestamp));
	buf.iterator += sizeof(topic.reference_timestamp);
	buf.offset += sizeof(topic.reference_timestamp);
	static_assert(sizeof(topic.segment_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.segment_timestamp, sizeof(topic.segment_timestamp));
	buf.iterator += sizeof(topic.segment_timestamp);
	buf.offset += sizeof(topic.segment_timestamp);
	static_assert(sizeof(topic.control_mode) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.control_mode, sizeof(topic.control_mode));
	buf.iterator += sizeof(topic.control_mode);
	buf.offset += sizeof(topic.control_mode);
	static_assert(sizeof(topic.failure_reason) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.failure_reason, sizeof(topic.failure_reason));
	buf.iterator += sizeof(topic.failure_reason);
	buf.offset += sizeof(topic.failure_reason);
	static_assert(sizeof(topic.active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.active, sizeof(topic.active));
	buf.iterator += sizeof(topic.active);
	buf.offset += sizeof(topic.active);
	static_assert(sizeof(topic.reference_fresh) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.reference_fresh, sizeof(topic.reference_fresh));
	buf.iterator += sizeof(topic.reference_fresh);
	buf.offset += sizeof(topic.reference_fresh);
	static_assert(sizeof(topic.segment_fresh) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.segment_fresh, sizeof(topic.segment_fresh));
	buf.iterator += sizeof(topic.segment_fresh);
	buf.offset += sizeof(topic.segment_fresh);
	static_assert(sizeof(topic.state_valid) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.state_valid, sizeof(topic.state_valid));
	buf.iterator += sizeof(topic.state_valid);
	buf.offset += sizeof(topic.state_valid);
	static_assert(sizeof(topic.hover_thrust_fresh) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.hover_thrust_fresh, sizeof(topic.hover_thrust_fresh));
	buf.iterator += sizeof(topic.hover_thrust_fresh);
	buf.offset += sizeof(topic.hover_thrust_fresh);
	static_assert(sizeof(topic.native_position_controller_disabled) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.native_position_controller_disabled, sizeof(topic.native_position_controller_disabled));
	buf.iterator += sizeof(topic.native_position_controller_disabled);
	buf.offset += sizeof(topic.native_position_controller_disabled);
	static_assert(sizeof(topic.native_attitude_rate_allocator_enabled) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.native_attitude_rate_allocator_enabled, sizeof(topic.native_attitude_rate_allocator_enabled));
	buf.iterator += sizeof(topic.native_attitude_rate_allocator_enabled);
	buf.offset += sizeof(topic.native_attitude_rate_allocator_enabled);
	static_assert(sizeof(topic.single_publisher_contract_pass) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.single_publisher_contract_pass, sizeof(topic.single_publisher_contract_pass));
	buf.iterator += sizeof(topic.single_publisher_contract_pass);
	buf.offset += sizeof(topic.single_publisher_contract_pass);
	buf.iterator += 6; // padding
	buf.offset += 6; // padding
	static_assert(sizeof(topic.input_sample_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.input_sample_count, sizeof(topic.input_sample_count));
	buf.iterator += sizeof(topic.input_sample_count);
	buf.offset += sizeof(topic.input_sample_count);
	static_assert(sizeof(topic.output_publish_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.output_publish_count, sizeof(topic.output_publish_count));
	buf.iterator += sizeof(topic.output_publish_count);
	buf.offset += sizeof(topic.output_publish_count);
	static_assert(sizeof(topic.rejected_sample_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.rejected_sample_count, sizeof(topic.rejected_sample_count));
	buf.iterator += sizeof(topic.rejected_sample_count);
	buf.offset += sizeof(topic.rejected_sample_count);
	static_assert(sizeof(topic.reset_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.reset_count, sizeof(topic.reset_count));
	buf.iterator += sizeof(topic.reset_count);
	buf.offset += sizeof(topic.reset_count);
	static_assert(sizeof(topic.dt_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.dt_s, sizeof(topic.dt_s));
	buf.iterator += sizeof(topic.dt_s);
	buf.offset += sizeof(topic.dt_s);
	static_assert(sizeof(topic.reference_age_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.reference_age_s, sizeof(topic.reference_age_s));
	buf.iterator += sizeof(topic.reference_age_s);
	buf.offset += sizeof(topic.reference_age_s);
	static_assert(sizeof(topic.segment_age_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.segment_age_s, sizeof(topic.segment_age_s));
	buf.iterator += sizeof(topic.segment_age_s);
	buf.offset += sizeof(topic.segment_age_s);
	static_assert(sizeof(topic.hover_thrust) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hover_thrust, sizeof(topic.hover_thrust));
	buf.iterator += sizeof(topic.hover_thrust);
	buf.offset += sizeof(topic.hover_thrust);
	static_assert(sizeof(topic.total_mass_kg) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.total_mass_kg, sizeof(topic.total_mass_kg));
	buf.iterator += sizeof(topic.total_mass_kg);
	buf.offset += sizeof(topic.total_mass_kg);
	static_assert(sizeof(topic.yaw_setpoint_rad) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.yaw_setpoint_rad, sizeof(topic.yaw_setpoint_rad));
	buf.iterator += sizeof(topic.yaw_setpoint_rad);
	buf.offset += sizeof(topic.yaw_setpoint_rad);
	static_assert(sizeof(topic.position_ned_m) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.position_ned_m, sizeof(topic.position_ned_m));
	buf.iterator += sizeof(topic.position_ned_m);
	buf.offset += sizeof(topic.position_ned_m);
	static_assert(sizeof(topic.velocity_ned_mps) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.velocity_ned_mps, sizeof(topic.velocity_ned_mps));
	buf.iterator += sizeof(topic.velocity_ned_mps);
	buf.offset += sizeof(topic.velocity_ned_mps);
	static_assert(sizeof(topic.reference_position_ned_m) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.reference_position_ned_m, sizeof(topic.reference_position_ned_m));
	buf.iterator += sizeof(topic.reference_position_ned_m);
	buf.offset += sizeof(topic.reference_position_ned_m);
	static_assert(sizeof(topic.reference_velocity_ned_mps) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.reference_velocity_ned_mps, sizeof(topic.reference_velocity_ned_mps));
	buf.iterator += sizeof(topic.reference_velocity_ned_mps);
	buf.offset += sizeof(topic.reference_velocity_ned_mps);
	static_assert(sizeof(topic.reference_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.reference_acceleration_ned_mps2, sizeof(topic.reference_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.reference_acceleration_ned_mps2);
	buf.offset += sizeof(topic.reference_acceleration_ned_mps2);
	static_assert(sizeof(topic.nominal_feedback_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.nominal_feedback_acceleration_ned_mps2, sizeof(topic.nominal_feedback_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.nominal_feedback_acceleration_ned_mps2);
	buf.offset += sizeof(topic.nominal_feedback_acceleration_ned_mps2);
	static_assert(sizeof(topic.drag_feedforward_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.drag_feedforward_acceleration_ned_mps2, sizeof(topic.drag_feedforward_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.drag_feedforward_acceleration_ned_mps2);
	buf.offset += sizeof(topic.drag_feedforward_acceleration_ned_mps2);
	static_assert(sizeof(topic.robust_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.robust_acceleration_ned_mps2, sizeof(topic.robust_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.robust_acceleration_ned_mps2);
	buf.offset += sizeof(topic.robust_acceleration_ned_mps2);
	static_assert(sizeof(topic.commanded_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.commanded_acceleration_ned_mps2, sizeof(topic.commanded_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.commanded_acceleration_ned_mps2);
	buf.offset += sizeof(topic.commanded_acceleration_ned_mps2);
	static_assert(sizeof(topic.normalized_thrust_ned) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.normalized_thrust_ned, sizeof(topic.normalized_thrust_ned));
	buf.iterator += sizeof(topic.normalized_thrust_ned);
	buf.offset += sizeof(topic.normalized_thrust_ned);
	return true;
}

static inline bool ucdr_deserialize_gpenmpc_se3_control_status(ucdrBuffer& buf, gpenmpc_se3_control_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.sample_timestamp) == 8, "size mismatch");
	memcpy(&topic.sample_timestamp, buf.iterator, sizeof(topic.sample_timestamp));
	buf.iterator += sizeof(topic.sample_timestamp);
	buf.offset += sizeof(topic.sample_timestamp);
	static_assert(sizeof(topic.reference_timestamp) == 8, "size mismatch");
	memcpy(&topic.reference_timestamp, buf.iterator, sizeof(topic.reference_timestamp));
	buf.iterator += sizeof(topic.reference_timestamp);
	buf.offset += sizeof(topic.reference_timestamp);
	static_assert(sizeof(topic.segment_timestamp) == 8, "size mismatch");
	memcpy(&topic.segment_timestamp, buf.iterator, sizeof(topic.segment_timestamp));
	buf.iterator += sizeof(topic.segment_timestamp);
	buf.offset += sizeof(topic.segment_timestamp);
	static_assert(sizeof(topic.control_mode) == 1, "size mismatch");
	memcpy(&topic.control_mode, buf.iterator, sizeof(topic.control_mode));
	buf.iterator += sizeof(topic.control_mode);
	buf.offset += sizeof(topic.control_mode);
	static_assert(sizeof(topic.failure_reason) == 1, "size mismatch");
	memcpy(&topic.failure_reason, buf.iterator, sizeof(topic.failure_reason));
	buf.iterator += sizeof(topic.failure_reason);
	buf.offset += sizeof(topic.failure_reason);
	static_assert(sizeof(topic.active) == 1, "size mismatch");
	memcpy(&topic.active, buf.iterator, sizeof(topic.active));
	buf.iterator += sizeof(topic.active);
	buf.offset += sizeof(topic.active);
	static_assert(sizeof(topic.reference_fresh) == 1, "size mismatch");
	memcpy(&topic.reference_fresh, buf.iterator, sizeof(topic.reference_fresh));
	buf.iterator += sizeof(topic.reference_fresh);
	buf.offset += sizeof(topic.reference_fresh);
	static_assert(sizeof(topic.segment_fresh) == 1, "size mismatch");
	memcpy(&topic.segment_fresh, buf.iterator, sizeof(topic.segment_fresh));
	buf.iterator += sizeof(topic.segment_fresh);
	buf.offset += sizeof(topic.segment_fresh);
	static_assert(sizeof(topic.state_valid) == 1, "size mismatch");
	memcpy(&topic.state_valid, buf.iterator, sizeof(topic.state_valid));
	buf.iterator += sizeof(topic.state_valid);
	buf.offset += sizeof(topic.state_valid);
	static_assert(sizeof(topic.hover_thrust_fresh) == 1, "size mismatch");
	memcpy(&topic.hover_thrust_fresh, buf.iterator, sizeof(topic.hover_thrust_fresh));
	buf.iterator += sizeof(topic.hover_thrust_fresh);
	buf.offset += sizeof(topic.hover_thrust_fresh);
	static_assert(sizeof(topic.native_position_controller_disabled) == 1, "size mismatch");
	memcpy(&topic.native_position_controller_disabled, buf.iterator, sizeof(topic.native_position_controller_disabled));
	buf.iterator += sizeof(topic.native_position_controller_disabled);
	buf.offset += sizeof(topic.native_position_controller_disabled);
	static_assert(sizeof(topic.native_attitude_rate_allocator_enabled) == 1, "size mismatch");
	memcpy(&topic.native_attitude_rate_allocator_enabled, buf.iterator, sizeof(topic.native_attitude_rate_allocator_enabled));
	buf.iterator += sizeof(topic.native_attitude_rate_allocator_enabled);
	buf.offset += sizeof(topic.native_attitude_rate_allocator_enabled);
	static_assert(sizeof(topic.single_publisher_contract_pass) == 1, "size mismatch");
	memcpy(&topic.single_publisher_contract_pass, buf.iterator, sizeof(topic.single_publisher_contract_pass));
	buf.iterator += sizeof(topic.single_publisher_contract_pass);
	buf.offset += sizeof(topic.single_publisher_contract_pass);
	buf.iterator += 6; // padding
	buf.offset += 6; // padding
	static_assert(sizeof(topic.input_sample_count) == 8, "size mismatch");
	memcpy(&topic.input_sample_count, buf.iterator, sizeof(topic.input_sample_count));
	buf.iterator += sizeof(topic.input_sample_count);
	buf.offset += sizeof(topic.input_sample_count);
	static_assert(sizeof(topic.output_publish_count) == 8, "size mismatch");
	memcpy(&topic.output_publish_count, buf.iterator, sizeof(topic.output_publish_count));
	buf.iterator += sizeof(topic.output_publish_count);
	buf.offset += sizeof(topic.output_publish_count);
	static_assert(sizeof(topic.rejected_sample_count) == 8, "size mismatch");
	memcpy(&topic.rejected_sample_count, buf.iterator, sizeof(topic.rejected_sample_count));
	buf.iterator += sizeof(topic.rejected_sample_count);
	buf.offset += sizeof(topic.rejected_sample_count);
	static_assert(sizeof(topic.reset_count) == 8, "size mismatch");
	memcpy(&topic.reset_count, buf.iterator, sizeof(topic.reset_count));
	buf.iterator += sizeof(topic.reset_count);
	buf.offset += sizeof(topic.reset_count);
	static_assert(sizeof(topic.dt_s) == 4, "size mismatch");
	memcpy(&topic.dt_s, buf.iterator, sizeof(topic.dt_s));
	buf.iterator += sizeof(topic.dt_s);
	buf.offset += sizeof(topic.dt_s);
	static_assert(sizeof(topic.reference_age_s) == 4, "size mismatch");
	memcpy(&topic.reference_age_s, buf.iterator, sizeof(topic.reference_age_s));
	buf.iterator += sizeof(topic.reference_age_s);
	buf.offset += sizeof(topic.reference_age_s);
	static_assert(sizeof(topic.segment_age_s) == 4, "size mismatch");
	memcpy(&topic.segment_age_s, buf.iterator, sizeof(topic.segment_age_s));
	buf.iterator += sizeof(topic.segment_age_s);
	buf.offset += sizeof(topic.segment_age_s);
	static_assert(sizeof(topic.hover_thrust) == 4, "size mismatch");
	memcpy(&topic.hover_thrust, buf.iterator, sizeof(topic.hover_thrust));
	buf.iterator += sizeof(topic.hover_thrust);
	buf.offset += sizeof(topic.hover_thrust);
	static_assert(sizeof(topic.total_mass_kg) == 4, "size mismatch");
	memcpy(&topic.total_mass_kg, buf.iterator, sizeof(topic.total_mass_kg));
	buf.iterator += sizeof(topic.total_mass_kg);
	buf.offset += sizeof(topic.total_mass_kg);
	static_assert(sizeof(topic.yaw_setpoint_rad) == 4, "size mismatch");
	memcpy(&topic.yaw_setpoint_rad, buf.iterator, sizeof(topic.yaw_setpoint_rad));
	buf.iterator += sizeof(topic.yaw_setpoint_rad);
	buf.offset += sizeof(topic.yaw_setpoint_rad);
	static_assert(sizeof(topic.position_ned_m) == 12, "size mismatch");
	memcpy(&topic.position_ned_m, buf.iterator, sizeof(topic.position_ned_m));
	buf.iterator += sizeof(topic.position_ned_m);
	buf.offset += sizeof(topic.position_ned_m);
	static_assert(sizeof(topic.velocity_ned_mps) == 12, "size mismatch");
	memcpy(&topic.velocity_ned_mps, buf.iterator, sizeof(topic.velocity_ned_mps));
	buf.iterator += sizeof(topic.velocity_ned_mps);
	buf.offset += sizeof(topic.velocity_ned_mps);
	static_assert(sizeof(topic.reference_position_ned_m) == 12, "size mismatch");
	memcpy(&topic.reference_position_ned_m, buf.iterator, sizeof(topic.reference_position_ned_m));
	buf.iterator += sizeof(topic.reference_position_ned_m);
	buf.offset += sizeof(topic.reference_position_ned_m);
	static_assert(sizeof(topic.reference_velocity_ned_mps) == 12, "size mismatch");
	memcpy(&topic.reference_velocity_ned_mps, buf.iterator, sizeof(topic.reference_velocity_ned_mps));
	buf.iterator += sizeof(topic.reference_velocity_ned_mps);
	buf.offset += sizeof(topic.reference_velocity_ned_mps);
	static_assert(sizeof(topic.reference_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(&topic.reference_acceleration_ned_mps2, buf.iterator, sizeof(topic.reference_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.reference_acceleration_ned_mps2);
	buf.offset += sizeof(topic.reference_acceleration_ned_mps2);
	static_assert(sizeof(topic.nominal_feedback_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(&topic.nominal_feedback_acceleration_ned_mps2, buf.iterator, sizeof(topic.nominal_feedback_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.nominal_feedback_acceleration_ned_mps2);
	buf.offset += sizeof(topic.nominal_feedback_acceleration_ned_mps2);
	static_assert(sizeof(topic.drag_feedforward_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(&topic.drag_feedforward_acceleration_ned_mps2, buf.iterator, sizeof(topic.drag_feedforward_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.drag_feedforward_acceleration_ned_mps2);
	buf.offset += sizeof(topic.drag_feedforward_acceleration_ned_mps2);
	static_assert(sizeof(topic.robust_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(&topic.robust_acceleration_ned_mps2, buf.iterator, sizeof(topic.robust_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.robust_acceleration_ned_mps2);
	buf.offset += sizeof(topic.robust_acceleration_ned_mps2);
	static_assert(sizeof(topic.commanded_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(&topic.commanded_acceleration_ned_mps2, buf.iterator, sizeof(topic.commanded_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.commanded_acceleration_ned_mps2);
	buf.offset += sizeof(topic.commanded_acceleration_ned_mps2);
	static_assert(sizeof(topic.normalized_thrust_ned) == 12, "size mismatch");
	memcpy(&topic.normalized_thrust_ned, buf.iterator, sizeof(topic.normalized_thrust_ned));
	buf.iterator += sizeof(topic.normalized_thrust_ned);
	buf.offset += sizeof(topic.normalized_thrust_ned);
	return true;
}
