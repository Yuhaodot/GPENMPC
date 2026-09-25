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
#include <uORB/topics/g_p_e_n_m_p_c_exec_test_status.h>


static inline constexpr int ucdr_topic_size_g_p_e_n_m_p_c_exec_test_status()
{
	return 144;
}

static inline bool ucdr_serialize_g_p_e_n_m_p_c_exec_test_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const g_p_e_n_m_p_c_exec_test_status_s& topic = *static_cast<const g_p_e_n_m_p_c_exec_test_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.segment_publish_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.segment_publish_count, sizeof(topic.segment_publish_count));
	buf.iterator += sizeof(topic.segment_publish_count);
	buf.offset += sizeof(topic.segment_publish_count);
	static_assert(sizeof(topic.shadow_publish_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_publish_count, sizeof(topic.shadow_publish_count));
	buf.iterator += sizeof(topic.shadow_publish_count);
	buf.offset += sizeof(topic.shadow_publish_count);
	static_assert(sizeof(topic.observed_exec_status_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.observed_exec_status_count, sizeof(topic.observed_exec_status_count));
	buf.iterator += sizeof(topic.observed_exec_status_count);
	buf.offset += sizeof(topic.observed_exec_status_count);
	static_assert(sizeof(topic.observed_active_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.observed_active_count, sizeof(topic.observed_active_count));
	buf.iterator += sizeof(topic.observed_active_count);
	buf.offset += sizeof(topic.observed_active_count);
	static_assert(sizeof(topic.shadow_exact_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_exact_count, sizeof(topic.shadow_exact_count));
	buf.iterator += sizeof(topic.shadow_exact_count);
	buf.offset += sizeof(topic.shadow_exact_count);
	static_assert(sizeof(topic.shadow_mismatch_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.shadow_mismatch_count, sizeof(topic.shadow_mismatch_count));
	buf.iterator += sizeof(topic.shadow_mismatch_count);
	buf.offset += sizeof(topic.shadow_mismatch_count);
	static_assert(sizeof(topic.active_setpoint_mismatch_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.active_setpoint_mismatch_count, sizeof(topic.active_setpoint_mismatch_count));
	buf.iterator += sizeof(topic.active_setpoint_mismatch_count);
	buf.offset += sizeof(topic.active_setpoint_mismatch_count);
	static_assert(sizeof(topic.expected_fail_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.expected_fail_count, sizeof(topic.expected_fail_count));
	buf.iterator += sizeof(topic.expected_fail_count);
	buf.offset += sizeof(topic.expected_fail_count);
	static_assert(sizeof(topic.unexpected_result_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.unexpected_result_count, sizeof(topic.unexpected_result_count));
	buf.iterator += sizeof(topic.unexpected_result_count);
	buf.offset += sizeof(topic.unexpected_result_count);
	static_assert(sizeof(topic.generated_c_call_count) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.generated_c_call_count, sizeof(topic.generated_c_call_count));
	buf.iterator += sizeof(topic.generated_c_call_count);
	buf.offset += sizeof(topic.generated_c_call_count);
	static_assert(sizeof(topic.mode) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.mode, sizeof(topic.mode));
	buf.iterator += sizeof(topic.mode);
	buf.offset += sizeof(topic.mode);
	static_assert(sizeof(topic.observed_exec_failure_reason) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.observed_exec_failure_reason, sizeof(topic.observed_exec_failure_reason));
	buf.iterator += sizeof(topic.observed_exec_failure_reason);
	buf.offset += sizeof(topic.observed_exec_failure_reason);
	static_assert(sizeof(topic.writer_lease_held) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.writer_lease_held, sizeof(topic.writer_lease_held));
	buf.iterator += sizeof(topic.writer_lease_held);
	buf.offset += sizeof(topic.writer_lease_held);
	static_assert(sizeof(topic.active_reference_published) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.active_reference_published, sizeof(topic.active_reference_published));
	buf.iterator += sizeof(topic.active_reference_published);
	buf.offset += sizeof(topic.active_reference_published);
	static_assert(sizeof(topic.exec_fail_closed) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.exec_fail_closed, sizeof(topic.exec_fail_closed));
	buf.iterator += sizeof(topic.exec_fail_closed);
	buf.offset += sizeof(topic.exec_fail_closed);
	static_assert(sizeof(topic.expected_result_observed) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.expected_result_observed, sizeof(topic.expected_result_observed));
	buf.iterator += sizeof(topic.expected_result_observed);
	buf.offset += sizeof(topic.expected_result_observed);
	static_assert(sizeof(topic.active_matches_constant_c) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.active_matches_constant_c, sizeof(topic.active_matches_constant_c));
	buf.iterator += sizeof(topic.active_matches_constant_c);
	buf.offset += sizeof(topic.active_matches_constant_c);
	static_assert(sizeof(topic.test_pass) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.test_pass, sizeof(topic.test_pass));
	buf.iterator += sizeof(topic.test_pass);
	buf.offset += sizeof(topic.test_pass);
	buf.iterator += 2; // padding
	buf.offset += 2; // padding
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
	static_assert(sizeof(topic.active_yaw_rad) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.active_yaw_rad, sizeof(topic.active_yaw_rad));
	buf.iterator += sizeof(topic.active_yaw_rad);
	buf.offset += sizeof(topic.active_yaw_rad);
	return true;
}

static inline bool ucdr_deserialize_g_p_e_n_m_p_c_exec_test_status(ucdrBuffer& buf, g_p_e_n_m_p_c_exec_test_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.segment_publish_count) == 8, "size mismatch");
	memcpy(&topic.segment_publish_count, buf.iterator, sizeof(topic.segment_publish_count));
	buf.iterator += sizeof(topic.segment_publish_count);
	buf.offset += sizeof(topic.segment_publish_count);
	static_assert(sizeof(topic.shadow_publish_count) == 8, "size mismatch");
	memcpy(&topic.shadow_publish_count, buf.iterator, sizeof(topic.shadow_publish_count));
	buf.iterator += sizeof(topic.shadow_publish_count);
	buf.offset += sizeof(topic.shadow_publish_count);
	static_assert(sizeof(topic.observed_exec_status_count) == 8, "size mismatch");
	memcpy(&topic.observed_exec_status_count, buf.iterator, sizeof(topic.observed_exec_status_count));
	buf.iterator += sizeof(topic.observed_exec_status_count);
	buf.offset += sizeof(topic.observed_exec_status_count);
	static_assert(sizeof(topic.observed_active_count) == 8, "size mismatch");
	memcpy(&topic.observed_active_count, buf.iterator, sizeof(topic.observed_active_count));
	buf.iterator += sizeof(topic.observed_active_count);
	buf.offset += sizeof(topic.observed_active_count);
	static_assert(sizeof(topic.shadow_exact_count) == 8, "size mismatch");
	memcpy(&topic.shadow_exact_count, buf.iterator, sizeof(topic.shadow_exact_count));
	buf.iterator += sizeof(topic.shadow_exact_count);
	buf.offset += sizeof(topic.shadow_exact_count);
	static_assert(sizeof(topic.shadow_mismatch_count) == 8, "size mismatch");
	memcpy(&topic.shadow_mismatch_count, buf.iterator, sizeof(topic.shadow_mismatch_count));
	buf.iterator += sizeof(topic.shadow_mismatch_count);
	buf.offset += sizeof(topic.shadow_mismatch_count);
	static_assert(sizeof(topic.active_setpoint_mismatch_count) == 8, "size mismatch");
	memcpy(&topic.active_setpoint_mismatch_count, buf.iterator, sizeof(topic.active_setpoint_mismatch_count));
	buf.iterator += sizeof(topic.active_setpoint_mismatch_count);
	buf.offset += sizeof(topic.active_setpoint_mismatch_count);
	static_assert(sizeof(topic.expected_fail_count) == 8, "size mismatch");
	memcpy(&topic.expected_fail_count, buf.iterator, sizeof(topic.expected_fail_count));
	buf.iterator += sizeof(topic.expected_fail_count);
	buf.offset += sizeof(topic.expected_fail_count);
	static_assert(sizeof(topic.unexpected_result_count) == 8, "size mismatch");
	memcpy(&topic.unexpected_result_count, buf.iterator, sizeof(topic.unexpected_result_count));
	buf.iterator += sizeof(topic.unexpected_result_count);
	buf.offset += sizeof(topic.unexpected_result_count);
	static_assert(sizeof(topic.generated_c_call_count) == 8, "size mismatch");
	memcpy(&topic.generated_c_call_count, buf.iterator, sizeof(topic.generated_c_call_count));
	buf.iterator += sizeof(topic.generated_c_call_count);
	buf.offset += sizeof(topic.generated_c_call_count);
	static_assert(sizeof(topic.mode) == 4, "size mismatch");
	memcpy(&topic.mode, buf.iterator, sizeof(topic.mode));
	buf.iterator += sizeof(topic.mode);
	buf.offset += sizeof(topic.mode);
	static_assert(sizeof(topic.observed_exec_failure_reason) == 4, "size mismatch");
	memcpy(&topic.observed_exec_failure_reason, buf.iterator, sizeof(topic.observed_exec_failure_reason));
	buf.iterator += sizeof(topic.observed_exec_failure_reason);
	buf.offset += sizeof(topic.observed_exec_failure_reason);
	static_assert(sizeof(topic.writer_lease_held) == 1, "size mismatch");
	memcpy(&topic.writer_lease_held, buf.iterator, sizeof(topic.writer_lease_held));
	buf.iterator += sizeof(topic.writer_lease_held);
	buf.offset += sizeof(topic.writer_lease_held);
	static_assert(sizeof(topic.active_reference_published) == 1, "size mismatch");
	memcpy(&topic.active_reference_published, buf.iterator, sizeof(topic.active_reference_published));
	buf.iterator += sizeof(topic.active_reference_published);
	buf.offset += sizeof(topic.active_reference_published);
	static_assert(sizeof(topic.exec_fail_closed) == 1, "size mismatch");
	memcpy(&topic.exec_fail_closed, buf.iterator, sizeof(topic.exec_fail_closed));
	buf.iterator += sizeof(topic.exec_fail_closed);
	buf.offset += sizeof(topic.exec_fail_closed);
	static_assert(sizeof(topic.expected_result_observed) == 1, "size mismatch");
	memcpy(&topic.expected_result_observed, buf.iterator, sizeof(topic.expected_result_observed));
	buf.iterator += sizeof(topic.expected_result_observed);
	buf.offset += sizeof(topic.expected_result_observed);
	static_assert(sizeof(topic.active_matches_constant_c) == 1, "size mismatch");
	memcpy(&topic.active_matches_constant_c, buf.iterator, sizeof(topic.active_matches_constant_c));
	buf.iterator += sizeof(topic.active_matches_constant_c);
	buf.offset += sizeof(topic.active_matches_constant_c);
	static_assert(sizeof(topic.test_pass) == 1, "size mismatch");
	memcpy(&topic.test_pass, buf.iterator, sizeof(topic.test_pass));
	buf.iterator += sizeof(topic.test_pass);
	buf.offset += sizeof(topic.test_pass);
	buf.iterator += 2; // padding
	buf.offset += 2; // padding
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
	static_assert(sizeof(topic.active_yaw_rad) == 4, "size mismatch");
	memcpy(&topic.active_yaw_rad, buf.iterator, sizeof(topic.active_yaw_rad));
	buf.iterator += sizeof(topic.active_yaw_rad);
	buf.offset += sizeof(topic.active_yaw_rad);
	return true;
}
