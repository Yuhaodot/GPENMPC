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
#include <uORB/topics/gpenmpc_shadow_suggestion.h>


static inline constexpr int ucdr_topic_size_gpenmpc_shadow_suggestion()
{
	return 148;
}

static inline bool ucdr_serialize_gpenmpc_shadow_suggestion(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const gpenmpc_shadow_suggestion_s& topic = *static_cast<const gpenmpc_shadow_suggestion_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.sequence) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.sequence, sizeof(topic.sequence));
	buf.iterator += sizeof(topic.sequence);
	buf.offset += sizeof(topic.sequence);
	static_assert(sizeof(topic.source_hash) == 32, "size mismatch");
	memcpy(buf.iterator, &topic.source_hash, sizeof(topic.source_hash));
	buf.iterator += sizeof(topic.source_hash);
	buf.offset += sizeof(topic.source_hash);
	static_assert(sizeof(topic.profile_hash) == 32, "size mismatch");
	memcpy(buf.iterator, &topic.profile_hash, sizeof(topic.profile_hash));
	buf.iterator += sizeof(topic.profile_hash);
	buf.offset += sizeof(topic.profile_hash);
	static_assert(sizeof(topic.proposed) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.proposed, sizeof(topic.proposed));
	buf.iterator += sizeof(topic.proposed);
	buf.offset += sizeof(topic.proposed);
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
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.predicted_position_risk_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.predicted_position_risk_m, sizeof(topic.predicted_position_risk_m));
	buf.iterator += sizeof(topic.predicted_position_risk_m);
	buf.offset += sizeof(topic.predicted_position_risk_m);
	static_assert(sizeof(topic.predicted_acceleration_risk_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.predicted_acceleration_risk_mps2, sizeof(topic.predicted_acceleration_risk_mps2));
	buf.iterator += sizeof(topic.predicted_acceleration_risk_mps2);
	buf.offset += sizeof(topic.predicted_acceleration_risk_mps2);
	static_assert(sizeof(topic.predicted_jerk_risk_mps3) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.predicted_jerk_risk_mps3, sizeof(topic.predicted_jerk_risk_mps3));
	buf.iterator += sizeof(topic.predicted_jerk_risk_mps3);
	buf.offset += sizeof(topic.predicted_jerk_risk_mps3);
	static_assert(sizeof(topic.recoverability_margin) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.recoverability_margin, sizeof(topic.recoverability_margin));
	buf.iterator += sizeof(topic.recoverability_margin);
	buf.offset += sizeof(topic.recoverability_margin);
	static_assert(sizeof(topic.proposed_progress_rate) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.proposed_progress_rate, sizeof(topic.proposed_progress_rate));
	buf.iterator += sizeof(topic.proposed_progress_rate);
	buf.offset += sizeof(topic.proposed_progress_rate);
	static_assert(sizeof(topic.proposed_position_ned_m) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.proposed_position_ned_m, sizeof(topic.proposed_position_ned_m));
	buf.iterator += sizeof(topic.proposed_position_ned_m);
	buf.offset += sizeof(topic.proposed_position_ned_m);
	static_assert(sizeof(topic.proposed_velocity_ned_mps) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.proposed_velocity_ned_mps, sizeof(topic.proposed_velocity_ned_mps));
	buf.iterator += sizeof(topic.proposed_velocity_ned_mps);
	buf.offset += sizeof(topic.proposed_velocity_ned_mps);
	static_assert(sizeof(topic.proposed_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.proposed_acceleration_ned_mps2, sizeof(topic.proposed_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.proposed_acceleration_ned_mps2);
	buf.offset += sizeof(topic.proposed_acceleration_ned_mps2);
	static_assert(sizeof(topic.proposed_yaw_rad) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.proposed_yaw_rad, sizeof(topic.proposed_yaw_rad));
	buf.iterator += sizeof(topic.proposed_yaw_rad);
	buf.offset += sizeof(topic.proposed_yaw_rad);
	static_assert(sizeof(topic.proposed_yaw_rate_rad_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.proposed_yaw_rate_rad_s, sizeof(topic.proposed_yaw_rate_rad_s));
	buf.iterator += sizeof(topic.proposed_yaw_rate_rad_s);
	buf.offset += sizeof(topic.proposed_yaw_rate_rad_s);
	return true;
}

static inline bool ucdr_deserialize_gpenmpc_shadow_suggestion(ucdrBuffer& buf, gpenmpc_shadow_suggestion_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.sequence) == 4, "size mismatch");
	memcpy(&topic.sequence, buf.iterator, sizeof(topic.sequence));
	buf.iterator += sizeof(topic.sequence);
	buf.offset += sizeof(topic.sequence);
	static_assert(sizeof(topic.source_hash) == 32, "size mismatch");
	memcpy(&topic.source_hash, buf.iterator, sizeof(topic.source_hash));
	buf.iterator += sizeof(topic.source_hash);
	buf.offset += sizeof(topic.source_hash);
	static_assert(sizeof(topic.profile_hash) == 32, "size mismatch");
	memcpy(&topic.profile_hash, buf.iterator, sizeof(topic.profile_hash));
	buf.iterator += sizeof(topic.profile_hash);
	buf.offset += sizeof(topic.profile_hash);
	static_assert(sizeof(topic.proposed) == 1, "size mismatch");
	memcpy(&topic.proposed, buf.iterator, sizeof(topic.proposed));
	buf.iterator += sizeof(topic.proposed);
	buf.offset += sizeof(topic.proposed);
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
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.predicted_position_risk_m) == 4, "size mismatch");
	memcpy(&topic.predicted_position_risk_m, buf.iterator, sizeof(topic.predicted_position_risk_m));
	buf.iterator += sizeof(topic.predicted_position_risk_m);
	buf.offset += sizeof(topic.predicted_position_risk_m);
	static_assert(sizeof(topic.predicted_acceleration_risk_mps2) == 4, "size mismatch");
	memcpy(&topic.predicted_acceleration_risk_mps2, buf.iterator, sizeof(topic.predicted_acceleration_risk_mps2));
	buf.iterator += sizeof(topic.predicted_acceleration_risk_mps2);
	buf.offset += sizeof(topic.predicted_acceleration_risk_mps2);
	static_assert(sizeof(topic.predicted_jerk_risk_mps3) == 4, "size mismatch");
	memcpy(&topic.predicted_jerk_risk_mps3, buf.iterator, sizeof(topic.predicted_jerk_risk_mps3));
	buf.iterator += sizeof(topic.predicted_jerk_risk_mps3);
	buf.offset += sizeof(topic.predicted_jerk_risk_mps3);
	static_assert(sizeof(topic.recoverability_margin) == 4, "size mismatch");
	memcpy(&topic.recoverability_margin, buf.iterator, sizeof(topic.recoverability_margin));
	buf.iterator += sizeof(topic.recoverability_margin);
	buf.offset += sizeof(topic.recoverability_margin);
	static_assert(sizeof(topic.proposed_progress_rate) == 4, "size mismatch");
	memcpy(&topic.proposed_progress_rate, buf.iterator, sizeof(topic.proposed_progress_rate));
	buf.iterator += sizeof(topic.proposed_progress_rate);
	buf.offset += sizeof(topic.proposed_progress_rate);
	static_assert(sizeof(topic.proposed_position_ned_m) == 12, "size mismatch");
	memcpy(&topic.proposed_position_ned_m, buf.iterator, sizeof(topic.proposed_position_ned_m));
	buf.iterator += sizeof(topic.proposed_position_ned_m);
	buf.offset += sizeof(topic.proposed_position_ned_m);
	static_assert(sizeof(topic.proposed_velocity_ned_mps) == 12, "size mismatch");
	memcpy(&topic.proposed_velocity_ned_mps, buf.iterator, sizeof(topic.proposed_velocity_ned_mps));
	buf.iterator += sizeof(topic.proposed_velocity_ned_mps);
	buf.offset += sizeof(topic.proposed_velocity_ned_mps);
	static_assert(sizeof(topic.proposed_acceleration_ned_mps2) == 12, "size mismatch");
	memcpy(&topic.proposed_acceleration_ned_mps2, buf.iterator, sizeof(topic.proposed_acceleration_ned_mps2));
	buf.iterator += sizeof(topic.proposed_acceleration_ned_mps2);
	buf.offset += sizeof(topic.proposed_acceleration_ned_mps2);
	static_assert(sizeof(topic.proposed_yaw_rad) == 4, "size mismatch");
	memcpy(&topic.proposed_yaw_rad, buf.iterator, sizeof(topic.proposed_yaw_rad));
	buf.iterator += sizeof(topic.proposed_yaw_rad);
	buf.offset += sizeof(topic.proposed_yaw_rad);
	static_assert(sizeof(topic.proposed_yaw_rate_rad_s) == 4, "size mismatch");
	memcpy(&topic.proposed_yaw_rate_rad_s, buf.iterator, sizeof(topic.proposed_yaw_rate_rad_s));
	buf.iterator += sizeof(topic.proposed_yaw_rate_rad_s);
	buf.offset += sizeof(topic.proposed_yaw_rate_rad_s);
	return true;
}
