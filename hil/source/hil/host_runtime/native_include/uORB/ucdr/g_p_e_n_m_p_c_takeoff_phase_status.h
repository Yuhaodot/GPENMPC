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
#include <uORB/topics/g_p_e_n_m_p_c_takeoff_phase_status.h>


static inline constexpr int ucdr_topic_size_g_p_e_n_m_p_c_takeoff_phase_status()
{
	return 68;
}

static inline bool ucdr_serialize_g_p_e_n_m_p_c_takeoff_phase_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const g_p_e_n_m_p_c_takeoff_phase_status_s& topic = *static_cast<const g_p_e_n_m_p_c_takeoff_phase_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.takeoff_status_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.takeoff_status_timestamp, sizeof(topic.takeoff_status_timestamp));
	buf.iterator += sizeof(topic.takeoff_status_timestamp);
	buf.offset += sizeof(topic.takeoff_status_timestamp);
	static_assert(sizeof(topic.evidence_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.evidence_timestamp, sizeof(topic.evidence_timestamp));
	buf.iterator += sizeof(topic.evidence_timestamp);
	buf.offset += sizeof(topic.evidence_timestamp);
	static_assert(sizeof(topic.phase_start_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.phase_start_timestamp, sizeof(topic.phase_start_timestamp));
	buf.iterator += sizeof(topic.phase_start_timestamp);
	buf.offset += sizeof(topic.phase_start_timestamp);
	static_assert(sizeof(topic.ground_frame_reset_event_count) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.ground_frame_reset_event_count, sizeof(topic.ground_frame_reset_event_count));
	buf.iterator += sizeof(topic.ground_frame_reset_event_count);
	buf.offset += sizeof(topic.ground_frame_reset_event_count);
	static_assert(sizeof(topic.state) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.state, sizeof(topic.state));
	buf.iterator += sizeof(topic.state);
	buf.offset += sizeof(topic.state);
	static_assert(sizeof(topic.reason) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.reason, sizeof(topic.reason));
	buf.iterator += sizeof(topic.reason);
	buf.offset += sizeof(topic.reason);
	static_assert(sizeof(topic.stock_takeoff_state) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.stock_takeoff_state, sizeof(topic.stock_takeoff_state));
	buf.iterator += sizeof(topic.stock_takeoff_state);
	buf.offset += sizeof(topic.stock_takeoff_state);
	static_assert(sizeof(topic.liftoff_evidence) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.liftoff_evidence, sizeof(topic.liftoff_evidence));
	buf.iterator += sizeof(topic.liftoff_evidence);
	buf.offset += sizeof(topic.liftoff_evidence);
	static_assert(sizeof(topic.phase_scheduled) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.phase_scheduled, sizeof(topic.phase_scheduled));
	buf.iterator += sizeof(topic.phase_scheduled);
	buf.offset += sizeof(topic.phase_scheduled);
	static_assert(sizeof(topic.task_progress_enabled) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.task_progress_enabled, sizeof(topic.task_progress_enabled));
	buf.iterator += sizeof(topic.task_progress_enabled);
	buf.offset += sizeof(topic.task_progress_enabled);
	static_assert(sizeof(topic.intent_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.intent_active, sizeof(topic.intent_active));
	buf.iterator += sizeof(topic.intent_active);
	buf.offset += sizeof(topic.intent_active);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.conservative_height_lower_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.conservative_height_lower_m, sizeof(topic.conservative_height_lower_m));
	buf.iterator += sizeof(topic.conservative_height_lower_m);
	buf.offset += sizeof(topic.conservative_height_lower_m);
	static_assert(sizeof(topic.conservative_velocity_upper_d_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.conservative_velocity_upper_d_mps, sizeof(topic.conservative_velocity_upper_d_mps));
	buf.iterator += sizeof(topic.conservative_velocity_upper_d_mps);
	buf.offset += sizeof(topic.conservative_velocity_upper_d_mps);
	static_assert(sizeof(topic.intent_gain) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.intent_gain, sizeof(topic.intent_gain));
	buf.iterator += sizeof(topic.intent_gain);
	buf.offset += sizeof(topic.intent_gain);
	static_assert(sizeof(topic.intent_velocity_d_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.intent_velocity_d_mps, sizeof(topic.intent_velocity_d_mps));
	buf.iterator += sizeof(topic.intent_velocity_d_mps);
	buf.offset += sizeof(topic.intent_velocity_d_mps);
	static_assert(sizeof(topic.intent_acceleration_d_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.intent_acceleration_d_mps2, sizeof(topic.intent_acceleration_d_mps2));
	buf.iterator += sizeof(topic.intent_acceleration_d_mps2);
	buf.offset += sizeof(topic.intent_acceleration_d_mps2);
	static_assert(sizeof(topic.intent_jerk_d_mps3) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.intent_jerk_d_mps3, sizeof(topic.intent_jerk_d_mps3));
	buf.iterator += sizeof(topic.intent_jerk_d_mps3);
	buf.offset += sizeof(topic.intent_jerk_d_mps3);
	return true;
}

static inline bool ucdr_deserialize_g_p_e_n_m_p_c_takeoff_phase_status(ucdrBuffer& buf, g_p_e_n_m_p_c_takeoff_phase_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.takeoff_status_timestamp) == 8, "size mismatch");
	memcpy(&topic.takeoff_status_timestamp, buf.iterator, sizeof(topic.takeoff_status_timestamp));
	buf.iterator += sizeof(topic.takeoff_status_timestamp);
	buf.offset += sizeof(topic.takeoff_status_timestamp);
	static_assert(sizeof(topic.evidence_timestamp) == 8, "size mismatch");
	memcpy(&topic.evidence_timestamp, buf.iterator, sizeof(topic.evidence_timestamp));
	buf.iterator += sizeof(topic.evidence_timestamp);
	buf.offset += sizeof(topic.evidence_timestamp);
	static_assert(sizeof(topic.phase_start_timestamp) == 8, "size mismatch");
	memcpy(&topic.phase_start_timestamp, buf.iterator, sizeof(topic.phase_start_timestamp));
	buf.iterator += sizeof(topic.phase_start_timestamp);
	buf.offset += sizeof(topic.phase_start_timestamp);
	static_assert(sizeof(topic.ground_frame_reset_event_count) == 4, "size mismatch");
	memcpy(&topic.ground_frame_reset_event_count, buf.iterator, sizeof(topic.ground_frame_reset_event_count));
	buf.iterator += sizeof(topic.ground_frame_reset_event_count);
	buf.offset += sizeof(topic.ground_frame_reset_event_count);
	static_assert(sizeof(topic.state) == 1, "size mismatch");
	memcpy(&topic.state, buf.iterator, sizeof(topic.state));
	buf.iterator += sizeof(topic.state);
	buf.offset += sizeof(topic.state);
	static_assert(sizeof(topic.reason) == 1, "size mismatch");
	memcpy(&topic.reason, buf.iterator, sizeof(topic.reason));
	buf.iterator += sizeof(topic.reason);
	buf.offset += sizeof(topic.reason);
	static_assert(sizeof(topic.stock_takeoff_state) == 1, "size mismatch");
	memcpy(&topic.stock_takeoff_state, buf.iterator, sizeof(topic.stock_takeoff_state));
	buf.iterator += sizeof(topic.stock_takeoff_state);
	buf.offset += sizeof(topic.stock_takeoff_state);
	static_assert(sizeof(topic.liftoff_evidence) == 1, "size mismatch");
	memcpy(&topic.liftoff_evidence, buf.iterator, sizeof(topic.liftoff_evidence));
	buf.iterator += sizeof(topic.liftoff_evidence);
	buf.offset += sizeof(topic.liftoff_evidence);
	static_assert(sizeof(topic.phase_scheduled) == 1, "size mismatch");
	memcpy(&topic.phase_scheduled, buf.iterator, sizeof(topic.phase_scheduled));
	buf.iterator += sizeof(topic.phase_scheduled);
	buf.offset += sizeof(topic.phase_scheduled);
	static_assert(sizeof(topic.task_progress_enabled) == 1, "size mismatch");
	memcpy(&topic.task_progress_enabled, buf.iterator, sizeof(topic.task_progress_enabled));
	buf.iterator += sizeof(topic.task_progress_enabled);
	buf.offset += sizeof(topic.task_progress_enabled);
	static_assert(sizeof(topic.intent_active) == 1, "size mismatch");
	memcpy(&topic.intent_active, buf.iterator, sizeof(topic.intent_active));
	buf.iterator += sizeof(topic.intent_active);
	buf.offset += sizeof(topic.intent_active);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.conservative_height_lower_m) == 4, "size mismatch");
	memcpy(&topic.conservative_height_lower_m, buf.iterator, sizeof(topic.conservative_height_lower_m));
	buf.iterator += sizeof(topic.conservative_height_lower_m);
	buf.offset += sizeof(topic.conservative_height_lower_m);
	static_assert(sizeof(topic.conservative_velocity_upper_d_mps) == 4, "size mismatch");
	memcpy(&topic.conservative_velocity_upper_d_mps, buf.iterator, sizeof(topic.conservative_velocity_upper_d_mps));
	buf.iterator += sizeof(topic.conservative_velocity_upper_d_mps);
	buf.offset += sizeof(topic.conservative_velocity_upper_d_mps);
	static_assert(sizeof(topic.intent_gain) == 4, "size mismatch");
	memcpy(&topic.intent_gain, buf.iterator, sizeof(topic.intent_gain));
	buf.iterator += sizeof(topic.intent_gain);
	buf.offset += sizeof(topic.intent_gain);
	static_assert(sizeof(topic.intent_velocity_d_mps) == 4, "size mismatch");
	memcpy(&topic.intent_velocity_d_mps, buf.iterator, sizeof(topic.intent_velocity_d_mps));
	buf.iterator += sizeof(topic.intent_velocity_d_mps);
	buf.offset += sizeof(topic.intent_velocity_d_mps);
	static_assert(sizeof(topic.intent_acceleration_d_mps2) == 4, "size mismatch");
	memcpy(&topic.intent_acceleration_d_mps2, buf.iterator, sizeof(topic.intent_acceleration_d_mps2));
	buf.iterator += sizeof(topic.intent_acceleration_d_mps2);
	buf.offset += sizeof(topic.intent_acceleration_d_mps2);
	static_assert(sizeof(topic.intent_jerk_d_mps3) == 4, "size mismatch");
	memcpy(&topic.intent_jerk_d_mps3, buf.iterator, sizeof(topic.intent_jerk_d_mps3));
	buf.iterator += sizeof(topic.intent_jerk_d_mps3);
	buf.offset += sizeof(topic.intent_jerk_d_mps3);
	return true;
}
