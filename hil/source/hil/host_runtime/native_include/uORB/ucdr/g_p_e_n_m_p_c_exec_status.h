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
#include <uORB/topics/g_p_e_n_m_p_c_exec_status.h>


static inline constexpr int ucdr_topic_size_g_p_e_n_m_p_c_exec_status()
{
	return 316;
}

static inline bool ucdr_serialize_g_p_e_n_m_p_c_exec_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const g_p_e_n_m_p_c_exec_status_s& topic = *static_cast<const g_p_e_n_m_p_c_exec_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.generated_c_call_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.generated_c_call_count, sizeof(topic.generated_c_call_count));
	buf.iterator += sizeof(topic.generated_c_call_count);
	buf.offset += sizeof(topic.generated_c_call_count);
	static_assert(sizeof(topic.active_publish_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.active_publish_count, sizeof(topic.active_publish_count));
	buf.iterator += sizeof(topic.active_publish_count);
	buf.offset += sizeof(topic.active_publish_count);
	static_assert(sizeof(topic.rejected_input_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.rejected_input_count, sizeof(topic.rejected_input_count));
	buf.iterator += sizeof(topic.rejected_input_count);
	buf.offset += sizeof(topic.rejected_input_count);
	static_assert(sizeof(topic.shadow_sample_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_sample_count, sizeof(topic.shadow_sample_count));
	buf.iterator += sizeof(topic.shadow_sample_count);
	buf.offset += sizeof(topic.shadow_sample_count);
	static_assert(sizeof(topic.shadow_invalid_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_invalid_count, sizeof(topic.shadow_invalid_count));
	buf.iterator += sizeof(topic.shadow_invalid_count);
	buf.offset += sizeof(topic.shadow_invalid_count);
	static_assert(sizeof(topic.same_timestamp_schedule_reuse_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.same_timestamp_schedule_reuse_count, sizeof(topic.same_timestamp_schedule_reuse_count));
	buf.iterator += sizeof(topic.same_timestamp_schedule_reuse_count);
	buf.offset += sizeof(topic.same_timestamp_schedule_reuse_count);
	static_assert(sizeof(topic.sequence) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.sequence, sizeof(topic.sequence));
	buf.iterator += sizeof(topic.sequence);
	buf.offset += sizeof(topic.sequence);
	static_assert(sizeof(topic.failure_reason) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.failure_reason, sizeof(topic.failure_reason));
	buf.iterator += sizeof(topic.failure_reason);
	buf.offset += sizeof(topic.failure_reason);
	static_assert(sizeof(topic.source_hash) == 32, "size mismatch");
	memcpy(buf.iterator, &topic.source_hash, sizeof(topic.source_hash));
	buf.iterator += sizeof(topic.source_hash);
	buf.offset += sizeof(topic.source_hash);
	static_assert(sizeof(topic.firmware_contract_hash) == 32, "size mismatch");
	memcpy(buf.iterator, &topic.firmware_contract_hash, sizeof(topic.firmware_contract_hash));
	buf.iterator += sizeof(topic.firmware_contract_hash);
	buf.offset += sizeof(topic.firmware_contract_hash);
	static_assert(sizeof(topic.profile_hash) == 32, "size mismatch");
	memcpy(buf.iterator, &topic.profile_hash, sizeof(topic.profile_hash));
	buf.iterator += sizeof(topic.profile_hash);
	buf.offset += sizeof(topic.profile_hash);
	static_assert(sizeof(topic.component_hash) == 32, "size mismatch");
	memcpy(buf.iterator, &topic.component_hash, sizeof(topic.component_hash));
	buf.iterator += sizeof(topic.component_hash);
	buf.offset += sizeof(topic.component_hash);
	static_assert(sizeof(topic.writer_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.writer_active, sizeof(topic.writer_active));
	buf.iterator += sizeof(topic.writer_active);
	buf.offset += sizeof(topic.writer_active);
	static_assert(sizeof(topic.active_reference_published) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.active_reference_published, sizeof(topic.active_reference_published));
	buf.iterator += sizeof(topic.active_reference_published);
	buf.offset += sizeof(topic.active_reference_published);
	static_assert(sizeof(topic.input_valid) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.input_valid, sizeof(topic.input_valid));
	buf.iterator += sizeof(topic.input_valid);
	buf.offset += sizeof(topic.input_valid);
	static_assert(sizeof(topic.fail_closed) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.fail_closed, sizeof(topic.fail_closed));
	buf.iterator += sizeof(topic.fail_closed);
	buf.offset += sizeof(topic.fail_closed);
	static_assert(sizeof(topic.shadow_valid) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_valid, sizeof(topic.shadow_valid));
	buf.iterator += sizeof(topic.shadow_valid);
	buf.offset += sizeof(topic.shadow_valid);
	static_assert(sizeof(topic.shadow_proposed) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_proposed, sizeof(topic.shadow_proposed));
	buf.iterator += sizeof(topic.shadow_proposed);
	buf.offset += sizeof(topic.shadow_proposed);
	static_assert(sizeof(topic.shadow_authorized) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_authorized, sizeof(topic.shadow_authorized));
	buf.iterator += sizeof(topic.shadow_authorized);
	buf.offset += sizeof(topic.shadow_authorized);
	static_assert(sizeof(topic.shadow_rejected) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_rejected, sizeof(topic.shadow_rejected));
	buf.iterator += sizeof(topic.shadow_rejected);
	buf.offset += sizeof(topic.shadow_rejected);
	static_assert(sizeof(topic.shadow_locked) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_locked, sizeof(topic.shadow_locked));
	buf.iterator += sizeof(topic.shadow_locked);
	buf.offset += sizeof(topic.shadow_locked);
	static_assert(sizeof(topic.shadow_released) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_released, sizeof(topic.shadow_released));
	buf.iterator += sizeof(topic.shadow_released);
	buf.offset += sizeof(topic.shadow_released);
	buf.iterator += 2; // padding
	buf.offset += 2; // padding
	static_assert(sizeof(topic.progress) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.progress, sizeof(topic.progress));
	buf.iterator += sizeof(topic.progress);
	buf.offset += sizeof(topic.progress);
	static_assert(sizeof(topic.progress_rate) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.progress_rate, sizeof(topic.progress_rate));
	buf.iterator += sizeof(topic.progress_rate);
	buf.offset += sizeof(topic.progress_rate);
	static_assert(sizeof(topic.execution_time_us) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.execution_time_us, sizeof(topic.execution_time_us));
	buf.iterator += sizeof(topic.execution_time_us);
	buf.offset += sizeof(topic.execution_time_us);
	static_assert(sizeof(topic.maximum_execution_time_us) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.maximum_execution_time_us, sizeof(topic.maximum_execution_time_us));
	buf.iterator += sizeof(topic.maximum_execution_time_us);
	buf.offset += sizeof(topic.maximum_execution_time_us);
	static_assert(sizeof(topic.rotor_reserve) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.rotor_reserve, sizeof(topic.rotor_reserve));
	buf.iterator += sizeof(topic.rotor_reserve);
	buf.offset += sizeof(topic.rotor_reserve);
	static_assert(sizeof(topic.active_position_ned_m) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.active_position_ned_m, sizeof(topic.active_position_ned_m));
	buf.iterator += sizeof(topic.active_position_ned_m);
	buf.offset += sizeof(topic.active_position_ned_m);
	static_assert(sizeof(topic.active_velocity_ned_mps) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.active_velocity_ned_mps, sizeof(topic.active_velocity_ned_mps));
	buf.iterator += sizeof(topic.active_velocity_ned_mps);
	buf.offset += sizeof(topic.active_velocity_ned_mps);
	static_assert(sizeof(topic.active_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.active_acceleration_ned_mps2, sizeof(topic.active_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.active_acceleration_ned_mps2);
	buf.offset += sizeof(topic.active_acceleration_ned_mps2);
	static_assert(sizeof(topic.active_jerk_ned_mps3) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.active_jerk_ned_mps3, sizeof(topic.active_jerk_ned_mps3));
	buf.iterator += sizeof(topic.active_jerk_ned_mps3);
	buf.offset += sizeof(topic.active_jerk_ned_mps3);
	static_assert(sizeof(topic.active_yaw_rad) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.active_yaw_rad, sizeof(topic.active_yaw_rad));
	buf.iterator += sizeof(topic.active_yaw_rad);
	buf.offset += sizeof(topic.active_yaw_rad);
	static_assert(sizeof(topic.active_yaw_rate_rad_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.active_yaw_rate_rad_s, sizeof(topic.active_yaw_rate_rad_s));
	buf.iterator += sizeof(topic.active_yaw_rate_rad_s);
	buf.offset += sizeof(topic.active_yaw_rate_rad_s);
	static_assert(sizeof(topic.predicted_rotor_thrust_n) == 24, "size mismatch");
	memcpy(buf.iterator, &topic.predicted_rotor_thrust_n, sizeof(topic.predicted_rotor_thrust_n));
	buf.iterator += sizeof(topic.predicted_rotor_thrust_n);
	buf.offset += sizeof(topic.predicted_rotor_thrust_n);
	static_assert(sizeof(topic.shadow_predicted_position_risk_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_predicted_position_risk_m, sizeof(topic.shadow_predicted_position_risk_m));
	buf.iterator += sizeof(topic.shadow_predicted_position_risk_m);
	buf.offset += sizeof(topic.shadow_predicted_position_risk_m);
	static_assert(sizeof(topic.shadow_recoverability_margin) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_recoverability_margin, sizeof(topic.shadow_recoverability_margin));
	buf.iterator += sizeof(topic.shadow_recoverability_margin);
	buf.offset += sizeof(topic.shadow_recoverability_margin);
	static_assert(sizeof(topic.shadow_proposed_progress_rate) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_proposed_progress_rate, sizeof(topic.shadow_proposed_progress_rate));
	buf.iterator += sizeof(topic.shadow_proposed_progress_rate);
	buf.offset += sizeof(topic.shadow_proposed_progress_rate);
	return true;
}

static inline bool ucdr_deserialize_g_p_e_n_m_p_c_exec_status(ucdrBuffer& buf, g_p_e_n_m_p_c_exec_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.generated_c_call_count) == 8, "size mismatch");
	memcpy(&topic.generated_c_call_count, buf.iterator, sizeof(topic.generated_c_call_count));
	buf.iterator += sizeof(topic.generated_c_call_count);
	buf.offset += sizeof(topic.generated_c_call_count);
	static_assert(sizeof(topic.active_publish_count) == 8, "size mismatch");
	memcpy(&topic.active_publish_count, buf.iterator, sizeof(topic.active_publish_count));
	buf.iterator += sizeof(topic.active_publish_count);
	buf.offset += sizeof(topic.active_publish_count);
	static_assert(sizeof(topic.rejected_input_count) == 8, "size mismatch");
	memcpy(&topic.rejected_input_count, buf.iterator, sizeof(topic.rejected_input_count));
	buf.iterator += sizeof(topic.rejected_input_count);
	buf.offset += sizeof(topic.rejected_input_count);
	static_assert(sizeof(topic.shadow_sample_count) == 8, "size mismatch");
	memcpy(&topic.shadow_sample_count, buf.iterator, sizeof(topic.shadow_sample_count));
	buf.iterator += sizeof(topic.shadow_sample_count);
	buf.offset += sizeof(topic.shadow_sample_count);
	static_assert(sizeof(topic.shadow_invalid_count) == 8, "size mismatch");
	memcpy(&topic.shadow_invalid_count, buf.iterator, sizeof(topic.shadow_invalid_count));
	buf.iterator += sizeof(topic.shadow_invalid_count);
	buf.offset += sizeof(topic.shadow_invalid_count);
	static_assert(sizeof(topic.same_timestamp_schedule_reuse_count) == 8, "size mismatch");
	memcpy(&topic.same_timestamp_schedule_reuse_count, buf.iterator, sizeof(topic.same_timestamp_schedule_reuse_count));
	buf.iterator += sizeof(topic.same_timestamp_schedule_reuse_count);
	buf.offset += sizeof(topic.same_timestamp_schedule_reuse_count);
	static_assert(sizeof(topic.sequence) == 4, "size mismatch");
	memcpy(&topic.sequence, buf.iterator, sizeof(topic.sequence));
	buf.iterator += sizeof(topic.sequence);
	buf.offset += sizeof(topic.sequence);
	static_assert(sizeof(topic.failure_reason) == 4, "size mismatch");
	memcpy(&topic.failure_reason, buf.iterator, sizeof(topic.failure_reason));
	buf.iterator += sizeof(topic.failure_reason);
	buf.offset += sizeof(topic.failure_reason);
	static_assert(sizeof(topic.source_hash) == 32, "size mismatch");
	memcpy(&topic.source_hash, buf.iterator, sizeof(topic.source_hash));
	buf.iterator += sizeof(topic.source_hash);
	buf.offset += sizeof(topic.source_hash);
	static_assert(sizeof(topic.firmware_contract_hash) == 32, "size mismatch");
	memcpy(&topic.firmware_contract_hash, buf.iterator, sizeof(topic.firmware_contract_hash));
	buf.iterator += sizeof(topic.firmware_contract_hash);
	buf.offset += sizeof(topic.firmware_contract_hash);
	static_assert(sizeof(topic.profile_hash) == 32, "size mismatch");
	memcpy(&topic.profile_hash, buf.iterator, sizeof(topic.profile_hash));
	buf.iterator += sizeof(topic.profile_hash);
	buf.offset += sizeof(topic.profile_hash);
	static_assert(sizeof(topic.component_hash) == 32, "size mismatch");
	memcpy(&topic.component_hash, buf.iterator, sizeof(topic.component_hash));
	buf.iterator += sizeof(topic.component_hash);
	buf.offset += sizeof(topic.component_hash);
	static_assert(sizeof(topic.writer_active) == 1, "size mismatch");
	memcpy(&topic.writer_active, buf.iterator, sizeof(topic.writer_active));
	buf.iterator += sizeof(topic.writer_active);
	buf.offset += sizeof(topic.writer_active);
	static_assert(sizeof(topic.active_reference_published) == 1, "size mismatch");
	memcpy(&topic.active_reference_published, buf.iterator, sizeof(topic.active_reference_published));
	buf.iterator += sizeof(topic.active_reference_published);
	buf.offset += sizeof(topic.active_reference_published);
	static_assert(sizeof(topic.input_valid) == 1, "size mismatch");
	memcpy(&topic.input_valid, buf.iterator, sizeof(topic.input_valid));
	buf.iterator += sizeof(topic.input_valid);
	buf.offset += sizeof(topic.input_valid);
	static_assert(sizeof(topic.fail_closed) == 1, "size mismatch");
	memcpy(&topic.fail_closed, buf.iterator, sizeof(topic.fail_closed));
	buf.iterator += sizeof(topic.fail_closed);
	buf.offset += sizeof(topic.fail_closed);
	static_assert(sizeof(topic.shadow_valid) == 1, "size mismatch");
	memcpy(&topic.shadow_valid, buf.iterator, sizeof(topic.shadow_valid));
	buf.iterator += sizeof(topic.shadow_valid);
	buf.offset += sizeof(topic.shadow_valid);
	static_assert(sizeof(topic.shadow_proposed) == 1, "size mismatch");
	memcpy(&topic.shadow_proposed, buf.iterator, sizeof(topic.shadow_proposed));
	buf.iterator += sizeof(topic.shadow_proposed);
	buf.offset += sizeof(topic.shadow_proposed);
	static_assert(sizeof(topic.shadow_authorized) == 1, "size mismatch");
	memcpy(&topic.shadow_authorized, buf.iterator, sizeof(topic.shadow_authorized));
	buf.iterator += sizeof(topic.shadow_authorized);
	buf.offset += sizeof(topic.shadow_authorized);
	static_assert(sizeof(topic.shadow_rejected) == 1, "size mismatch");
	memcpy(&topic.shadow_rejected, buf.iterator, sizeof(topic.shadow_rejected));
	buf.iterator += sizeof(topic.shadow_rejected);
	buf.offset += sizeof(topic.shadow_rejected);
	static_assert(sizeof(topic.shadow_locked) == 1, "size mismatch");
	memcpy(&topic.shadow_locked, buf.iterator, sizeof(topic.shadow_locked));
	buf.iterator += sizeof(topic.shadow_locked);
	buf.offset += sizeof(topic.shadow_locked);
	static_assert(sizeof(topic.shadow_released) == 1, "size mismatch");
	memcpy(&topic.shadow_released, buf.iterator, sizeof(topic.shadow_released));
	buf.iterator += sizeof(topic.shadow_released);
	buf.offset += sizeof(topic.shadow_released);
	buf.iterator += 2; // padding
	buf.offset += 2; // padding
	static_assert(sizeof(topic.progress) == 4, "size mismatch");
	memcpy(&topic.progress, buf.iterator, sizeof(topic.progress));
	buf.iterator += sizeof(topic.progress);
	buf.offset += sizeof(topic.progress);
	static_assert(sizeof(topic.progress_rate) == 4, "size mismatch");
	memcpy(&topic.progress_rate, buf.iterator, sizeof(topic.progress_rate));
	buf.iterator += sizeof(topic.progress_rate);
	buf.offset += sizeof(topic.progress_rate);
	static_assert(sizeof(topic.execution_time_us) == 4, "size mismatch");
	memcpy(&topic.execution_time_us, buf.iterator, sizeof(topic.execution_time_us));
	buf.iterator += sizeof(topic.execution_time_us);
	buf.offset += sizeof(topic.execution_time_us);
	static_assert(sizeof(topic.maximum_execution_time_us) == 4, "size mismatch");
	memcpy(&topic.maximum_execution_time_us, buf.iterator, sizeof(topic.maximum_execution_time_us));
	buf.iterator += sizeof(topic.maximum_execution_time_us);
	buf.offset += sizeof(topic.maximum_execution_time_us);
	static_assert(sizeof(topic.rotor_reserve) == 4, "size mismatch");
	memcpy(&topic.rotor_reserve, buf.iterator, sizeof(topic.rotor_reserve));
	buf.iterator += sizeof(topic.rotor_reserve);
	buf.offset += sizeof(topic.rotor_reserve);
	static_assert(sizeof(topic.active_position_ned_m) == 12, "size mismatch");
	memcpy(&topic.active_position_ned_m, buf.iterator, sizeof(topic.active_position_ned_m));
	buf.iterator += sizeof(topic.active_position_ned_m);
	buf.offset += sizeof(topic.active_position_ned_m);
	static_assert(sizeof(topic.active_velocity_ned_mps) == 12, "size mismatch");
	memcpy(&topic.active_velocity_ned_mps, buf.iterator, sizeof(topic.active_velocity_ned_mps));
	buf.iterator += sizeof(topic.active_velocity_ned_mps);
	buf.offset += sizeof(topic.active_velocity_ned_mps);
	static_assert(sizeof(topic.active_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(&topic.active_acceleration_ned_mps2, buf.iterator, sizeof(topic.active_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.active_acceleration_ned_mps2);
	buf.offset += sizeof(topic.active_acceleration_ned_mps2);
	static_assert(sizeof(topic.active_jerk_ned_mps3) == 12, "size mismatch");
	memcpy(&topic.active_jerk_ned_mps3, buf.iterator, sizeof(topic.active_jerk_ned_mps3));
	buf.iterator += sizeof(topic.active_jerk_ned_mps3);
	buf.offset += sizeof(topic.active_jerk_ned_mps3);
	static_assert(sizeof(topic.active_yaw_rad) == 4, "size mismatch");
	memcpy(&topic.active_yaw_rad, buf.iterator, sizeof(topic.active_yaw_rad));
	buf.iterator += sizeof(topic.active_yaw_rad);
	buf.offset += sizeof(topic.active_yaw_rad);
	static_assert(sizeof(topic.active_yaw_rate_rad_s) == 4, "size mismatch");
	memcpy(&topic.active_yaw_rate_rad_s, buf.iterator, sizeof(topic.active_yaw_rate_rad_s));
	buf.iterator += sizeof(topic.active_yaw_rate_rad_s);
	buf.offset += sizeof(topic.active_yaw_rate_rad_s);
	static_assert(sizeof(topic.predicted_rotor_thrust_n) == 24, "size mismatch");
	memcpy(&topic.predicted_rotor_thrust_n, buf.iterator, sizeof(topic.predicted_rotor_thrust_n));
	buf.iterator += sizeof(topic.predicted_rotor_thrust_n);
	buf.offset += sizeof(topic.predicted_rotor_thrust_n);
	static_assert(sizeof(topic.shadow_predicted_position_risk_m) == 4, "size mismatch");
	memcpy(&topic.shadow_predicted_position_risk_m, buf.iterator, sizeof(topic.shadow_predicted_position_risk_m));
	buf.iterator += sizeof(topic.shadow_predicted_position_risk_m);
	buf.offset += sizeof(topic.shadow_predicted_position_risk_m);
	static_assert(sizeof(topic.shadow_recoverability_margin) == 4, "size mismatch");
	memcpy(&topic.shadow_recoverability_margin, buf.iterator, sizeof(topic.shadow_recoverability_margin));
	buf.iterator += sizeof(topic.shadow_recoverability_margin);
	buf.offset += sizeof(topic.shadow_recoverability_margin);
	static_assert(sizeof(topic.shadow_proposed_progress_rate) == 4, "size mismatch");
	memcpy(&topic.shadow_proposed_progress_rate, buf.iterator, sizeof(topic.shadow_proposed_progress_rate));
	buf.iterator += sizeof(topic.shadow_proposed_progress_rate);
	buf.offset += sizeof(topic.shadow_proposed_progress_rate);
	return true;
}
