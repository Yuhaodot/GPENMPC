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
#include <uORB/topics/gpenmpc_trajectory_segment.h>


static inline constexpr int ucdr_topic_size_gpenmpc_trajectory_segment()
{
	return 404;
}

static inline bool ucdr_serialize_gpenmpc_trajectory_segment(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const gpenmpc_trajectory_segment_s& topic = *static_cast<const gpenmpc_trajectory_segment_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.segment_start_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.segment_start_timestamp, sizeof(topic.segment_start_timestamp));
	buf.iterator += sizeof(topic.segment_start_timestamp);
	buf.offset += sizeof(topic.segment_start_timestamp);
	static_assert(sizeof(topic.schema_version) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.schema_version, sizeof(topic.schema_version));
	buf.iterator += sizeof(topic.schema_version);
	buf.offset += sizeof(topic.schema_version);
	static_assert(sizeof(topic.sequence) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.sequence, sizeof(topic.sequence));
	buf.iterator += sizeof(topic.sequence);
	buf.offset += sizeof(topic.sequence);
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
	static_assert(sizeof(topic.control_point_count) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.control_point_count, sizeof(topic.control_point_count));
	buf.iterator += sizeof(topic.control_point_count);
	buf.offset += sizeof(topic.control_point_count);
	buf.iterator += 7; // padding
	buf.offset += 7; // padding
	static_assert(sizeof(topic.control_points_ned_m) == 144, "size mismatch");
	memcpy(buf.iterator, &topic.control_points_ned_m, sizeof(topic.control_points_ned_m));
	buf.iterator += sizeof(topic.control_points_ned_m);
	buf.offset += sizeof(topic.control_points_ned_m);
	static_assert(sizeof(topic.duration_s) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.duration_s, sizeof(topic.duration_s));
	buf.iterator += sizeof(topic.duration_s);
	buf.offset += sizeof(topic.duration_s);
	static_assert(sizeof(topic.initial_progress) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.initial_progress, sizeof(topic.initial_progress));
	buf.iterator += sizeof(topic.initial_progress);
	buf.offset += sizeof(topic.initial_progress);
	static_assert(sizeof(topic.progress_rate) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.progress_rate, sizeof(topic.progress_rate));
	buf.iterator += sizeof(topic.progress_rate);
	buf.offset += sizeof(topic.progress_rate);
	static_assert(sizeof(topic.yaw_start_rad) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.yaw_start_rad, sizeof(topic.yaw_start_rad));
	buf.iterator += sizeof(topic.yaw_start_rad);
	buf.offset += sizeof(topic.yaw_start_rad);
	static_assert(sizeof(topic.yaw_end_rad) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.yaw_end_rad, sizeof(topic.yaw_end_rad));
	buf.iterator += sizeof(topic.yaw_end_rad);
	buf.offset += sizeof(topic.yaw_end_rad);
	static_assert(sizeof(topic.yaw_rate_limit_rad_s) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.yaw_rate_limit_rad_s, sizeof(topic.yaw_rate_limit_rad_s));
	buf.iterator += sizeof(topic.yaw_rate_limit_rad_s);
	buf.offset += sizeof(topic.yaw_rate_limit_rad_s);
	static_assert(sizeof(topic.estimated_wind_ned_mps) == 24, "size mismatch");
	memcpy(buf.iterator, &topic.estimated_wind_ned_mps, sizeof(topic.estimated_wind_ned_mps));
	buf.iterator += sizeof(topic.estimated_wind_ned_mps);
	buf.offset += sizeof(topic.estimated_wind_ned_mps);
	static_assert(sizeof(topic.base_mass_kg) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.base_mass_kg, sizeof(topic.base_mass_kg));
	buf.iterator += sizeof(topic.base_mass_kg);
	buf.offset += sizeof(topic.base_mass_kg);
	static_assert(sizeof(topic.payload_remaining_kg) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.payload_remaining_kg, sizeof(topic.payload_remaining_kg));
	buf.iterator += sizeof(topic.payload_remaining_kg);
	buf.offset += sizeof(topic.payload_remaining_kg);
	static_assert(sizeof(topic.delivered_payload_kg) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.delivered_payload_kg, sizeof(topic.delivered_payload_kg));
	buf.iterator += sizeof(topic.delivered_payload_kg);
	buf.offset += sizeof(topic.delivered_payload_kg);
	static_assert(sizeof(topic.service_edge) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.service_edge, sizeof(topic.service_edge));
	buf.iterator += sizeof(topic.service_edge);
	buf.offset += sizeof(topic.service_edge);
	static_assert(sizeof(topic.reset) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.reset, sizeof(topic.reset));
	buf.iterator += sizeof(topic.reset);
	buf.offset += sizeof(topic.reset);
	static_assert(sizeof(topic.payload_transition_enabled) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.payload_transition_enabled, sizeof(topic.payload_transition_enabled));
	buf.iterator += sizeof(topic.payload_transition_enabled);
	buf.offset += sizeof(topic.payload_transition_enabled);
	static_assert(sizeof(topic.host_feedforward_disabled) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.host_feedforward_disabled, sizeof(topic.host_feedforward_disabled));
	buf.iterator += sizeof(topic.host_feedforward_disabled);
	buf.offset += sizeof(topic.host_feedforward_disabled);
	return true;
}

static inline bool ucdr_deserialize_gpenmpc_trajectory_segment(ucdrBuffer& buf, gpenmpc_trajectory_segment_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.segment_start_timestamp) == 8, "size mismatch");
	memcpy(&topic.segment_start_timestamp, buf.iterator, sizeof(topic.segment_start_timestamp));
	buf.iterator += sizeof(topic.segment_start_timestamp);
	buf.offset += sizeof(topic.segment_start_timestamp);
	static_assert(sizeof(topic.schema_version) == 4, "size mismatch");
	memcpy(&topic.schema_version, buf.iterator, sizeof(topic.schema_version));
	buf.iterator += sizeof(topic.schema_version);
	buf.offset += sizeof(topic.schema_version);
	static_assert(sizeof(topic.sequence) == 4, "size mismatch");
	memcpy(&topic.sequence, buf.iterator, sizeof(topic.sequence));
	buf.iterator += sizeof(topic.sequence);
	buf.offset += sizeof(topic.sequence);
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
	static_assert(sizeof(topic.control_point_count) == 1, "size mismatch");
	memcpy(&topic.control_point_count, buf.iterator, sizeof(topic.control_point_count));
	buf.iterator += sizeof(topic.control_point_count);
	buf.offset += sizeof(topic.control_point_count);
	buf.iterator += 7; // padding
	buf.offset += 7; // padding
	static_assert(sizeof(topic.control_points_ned_m) == 144, "size mismatch");
	memcpy(&topic.control_points_ned_m, buf.iterator, sizeof(topic.control_points_ned_m));
	buf.iterator += sizeof(topic.control_points_ned_m);
	buf.offset += sizeof(topic.control_points_ned_m);
	static_assert(sizeof(topic.duration_s) == 8, "size mismatch");
	memcpy(&topic.duration_s, buf.iterator, sizeof(topic.duration_s));
	buf.iterator += sizeof(topic.duration_s);
	buf.offset += sizeof(topic.duration_s);
	static_assert(sizeof(topic.initial_progress) == 8, "size mismatch");
	memcpy(&topic.initial_progress, buf.iterator, sizeof(topic.initial_progress));
	buf.iterator += sizeof(topic.initial_progress);
	buf.offset += sizeof(topic.initial_progress);
	static_assert(sizeof(topic.progress_rate) == 8, "size mismatch");
	memcpy(&topic.progress_rate, buf.iterator, sizeof(topic.progress_rate));
	buf.iterator += sizeof(topic.progress_rate);
	buf.offset += sizeof(topic.progress_rate);
	static_assert(sizeof(topic.yaw_start_rad) == 8, "size mismatch");
	memcpy(&topic.yaw_start_rad, buf.iterator, sizeof(topic.yaw_start_rad));
	buf.iterator += sizeof(topic.yaw_start_rad);
	buf.offset += sizeof(topic.yaw_start_rad);
	static_assert(sizeof(topic.yaw_end_rad) == 8, "size mismatch");
	memcpy(&topic.yaw_end_rad, buf.iterator, sizeof(topic.yaw_end_rad));
	buf.iterator += sizeof(topic.yaw_end_rad);
	buf.offset += sizeof(topic.yaw_end_rad);
	static_assert(sizeof(topic.yaw_rate_limit_rad_s) == 8, "size mismatch");
	memcpy(&topic.yaw_rate_limit_rad_s, buf.iterator, sizeof(topic.yaw_rate_limit_rad_s));
	buf.iterator += sizeof(topic.yaw_rate_limit_rad_s);
	buf.offset += sizeof(topic.yaw_rate_limit_rad_s);
	static_assert(sizeof(topic.estimated_wind_ned_mps) == 24, "size mismatch");
	memcpy(&topic.estimated_wind_ned_mps, buf.iterator, sizeof(topic.estimated_wind_ned_mps));
	buf.iterator += sizeof(topic.estimated_wind_ned_mps);
	buf.offset += sizeof(topic.estimated_wind_ned_mps);
	static_assert(sizeof(topic.base_mass_kg) == 8, "size mismatch");
	memcpy(&topic.base_mass_kg, buf.iterator, sizeof(topic.base_mass_kg));
	buf.iterator += sizeof(topic.base_mass_kg);
	buf.offset += sizeof(topic.base_mass_kg);
	static_assert(sizeof(topic.payload_remaining_kg) == 8, "size mismatch");
	memcpy(&topic.payload_remaining_kg, buf.iterator, sizeof(topic.payload_remaining_kg));
	buf.iterator += sizeof(topic.payload_remaining_kg);
	buf.offset += sizeof(topic.payload_remaining_kg);
	static_assert(sizeof(topic.delivered_payload_kg) == 8, "size mismatch");
	memcpy(&topic.delivered_payload_kg, buf.iterator, sizeof(topic.delivered_payload_kg));
	buf.iterator += sizeof(topic.delivered_payload_kg);
	buf.offset += sizeof(topic.delivered_payload_kg);
	static_assert(sizeof(topic.service_edge) == 1, "size mismatch");
	memcpy(&topic.service_edge, buf.iterator, sizeof(topic.service_edge));
	buf.iterator += sizeof(topic.service_edge);
	buf.offset += sizeof(topic.service_edge);
	static_assert(sizeof(topic.reset) == 1, "size mismatch");
	memcpy(&topic.reset, buf.iterator, sizeof(topic.reset));
	buf.iterator += sizeof(topic.reset);
	buf.offset += sizeof(topic.reset);
	static_assert(sizeof(topic.payload_transition_enabled) == 1, "size mismatch");
	memcpy(&topic.payload_transition_enabled, buf.iterator, sizeof(topic.payload_transition_enabled));
	buf.iterator += sizeof(topic.payload_transition_enabled);
	buf.offset += sizeof(topic.payload_transition_enabled);
	static_assert(sizeof(topic.host_feedforward_disabled) == 1, "size mismatch");
	memcpy(&topic.host_feedforward_disabled, buf.iterator, sizeof(topic.host_feedforward_disabled));
	buf.iterator += sizeof(topic.host_feedforward_disabled);
	buf.offset += sizeof(topic.host_feedforward_disabled);
	return true;
}
