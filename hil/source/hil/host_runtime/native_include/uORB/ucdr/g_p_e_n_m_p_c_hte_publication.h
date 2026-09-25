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
#include <uORB/topics/g_p_e_n_m_p_c_hte_publication.h>


static inline constexpr int ucdr_topic_size_g_p_e_n_m_p_c_hte_publication()
{
	return 100;
}

static inline bool ucdr_serialize_g_p_e_n_m_p_c_hte_publication(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const g_p_e_n_m_p_c_hte_publication_s& topic = *static_cast<const g_p_e_n_m_p_c_hte_publication_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.timestamp_sample) == 8, "size mismatch");
	const uint64_t timestamp_sample_adjusted = topic.timestamp_sample + time_offset;
	memcpy(buf.iterator, &timestamp_sample_adjusted, sizeof(topic.timestamp_sample));
	buf.iterator += sizeof(topic.timestamp_sample);
	buf.offset += sizeof(topic.timestamp_sample);
	static_assert(sizeof(topic.hover_thrust) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hover_thrust, sizeof(topic.hover_thrust));
	buf.iterator += sizeof(topic.hover_thrust);
	buf.offset += sizeof(topic.hover_thrust);
	static_assert(sizeof(topic.hover_thrust_var) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hover_thrust_var, sizeof(topic.hover_thrust_var));
	buf.iterator += sizeof(topic.hover_thrust_var);
	buf.offset += sizeof(topic.hover_thrust_var);
	static_assert(sizeof(topic.valid) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.valid, sizeof(topic.valid));
	buf.iterator += sizeof(topic.valid);
	buf.offset += sizeof(topic.valid);
	static_assert(sizeof(topic.eligibility_reason) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.eligibility_reason, sizeof(topic.eligibility_reason));
	buf.iterator += sizeof(topic.eligibility_reason);
	buf.offset += sizeof(topic.eligibility_reason);
	static_assert(sizeof(topic.last_eligibility_failure_reason) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.last_eligibility_failure_reason, sizeof(topic.last_eligibility_failure_reason));
	buf.iterator += sizeof(topic.last_eligibility_failure_reason);
	buf.offset += sizeof(topic.last_eligibility_failure_reason);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.eligibility_failure_count) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.eligibility_failure_count, sizeof(topic.eligibility_failure_count));
	buf.iterator += sizeof(topic.eligibility_failure_count);
	buf.offset += sizeof(topic.eligibility_failure_count);
	static_assert(sizeof(topic.last_eligibility_failure_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.last_eligibility_failure_timestamp, sizeof(topic.last_eligibility_failure_timestamp));
	buf.iterator += sizeof(topic.last_eligibility_failure_timestamp);
	buf.offset += sizeof(topic.last_eligibility_failure_timestamp);
	static_assert(sizeof(topic.eligibility_available) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.eligibility_available, sizeof(topic.eligibility_available));
	buf.iterator += sizeof(topic.eligibility_available);
	buf.offset += sizeof(topic.eligibility_available);
	static_assert(sizeof(topic.eligibility_armed) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.eligibility_armed, sizeof(topic.eligibility_armed));
	buf.iterator += sizeof(topic.eligibility_armed);
	buf.offset += sizeof(topic.eligibility_armed);
	static_assert(sizeof(topic.eligibility_in_air) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.eligibility_in_air, sizeof(topic.eligibility_in_air));
	buf.iterator += sizeof(topic.eligibility_in_air);
	buf.offset += sizeof(topic.eligibility_in_air);
	static_assert(sizeof(topic.eligibility_landed) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.eligibility_landed, sizeof(topic.eligibility_landed));
	buf.iterator += sizeof(topic.eligibility_landed);
	buf.offset += sizeof(topic.eligibility_landed);
	static_assert(sizeof(topic.local_position_dt) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.local_position_dt, sizeof(topic.local_position_dt));
	buf.iterator += sizeof(topic.local_position_dt);
	buf.offset += sizeof(topic.local_position_dt);
	static_assert(sizeof(topic.local_position_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.local_position_timestamp, sizeof(topic.local_position_timestamp));
	buf.iterator += sizeof(topic.local_position_timestamp);
	buf.offset += sizeof(topic.local_position_timestamp);
	static_assert(sizeof(topic.local_position_timestamp_sample) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.local_position_timestamp_sample, sizeof(topic.local_position_timestamp_sample));
	buf.iterator += sizeof(topic.local_position_timestamp_sample);
	buf.offset += sizeof(topic.local_position_timestamp_sample);
	static_assert(sizeof(topic.local_position_sample_advanced) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.local_position_sample_advanced, sizeof(topic.local_position_sample_advanced));
	buf.iterator += sizeof(topic.local_position_sample_advanced);
	buf.offset += sizeof(topic.local_position_sample_advanced);
	static_assert(sizeof(topic.local_position_z_reset_counter) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.local_position_z_reset_counter, sizeof(topic.local_position_z_reset_counter));
	buf.iterator += sizeof(topic.local_position_z_reset_counter);
	buf.offset += sizeof(topic.local_position_z_reset_counter);
	static_assert(sizeof(topic.local_position_vz_reset_counter) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.local_position_vz_reset_counter, sizeof(topic.local_position_vz_reset_counter));
	buf.iterator += sizeof(topic.local_position_vz_reset_counter);
	buf.offset += sizeof(topic.local_position_vz_reset_counter);
	static_assert(sizeof(topic.local_position_dist_bottom_reset_counter) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.local_position_dist_bottom_reset_counter, sizeof(topic.local_position_dist_bottom_reset_counter));
	buf.iterator += sizeof(topic.local_position_dist_bottom_reset_counter);
	buf.offset += sizeof(topic.local_position_dist_bottom_reset_counter);
	static_assert(sizeof(topic.last_failure_armed) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.last_failure_armed, sizeof(topic.last_failure_armed));
	buf.iterator += sizeof(topic.last_failure_armed);
	buf.offset += sizeof(topic.last_failure_armed);
	static_assert(sizeof(topic.last_failure_in_air) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.last_failure_in_air, sizeof(topic.last_failure_in_air));
	buf.iterator += sizeof(topic.last_failure_in_air);
	buf.offset += sizeof(topic.last_failure_in_air);
	static_assert(sizeof(topic.last_failure_landed) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.last_failure_landed, sizeof(topic.last_failure_landed));
	buf.iterator += sizeof(topic.last_failure_landed);
	buf.offset += sizeof(topic.last_failure_landed);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.last_failure_local_position_dt) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.last_failure_local_position_dt, sizeof(topic.last_failure_local_position_dt));
	buf.iterator += sizeof(topic.last_failure_local_position_dt);
	buf.offset += sizeof(topic.last_failure_local_position_dt);
	buf.iterator += 4; // padding
	buf.offset += 4; // padding
	static_assert(sizeof(topic.last_failure_local_position_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.last_failure_local_position_timestamp, sizeof(topic.last_failure_local_position_timestamp));
	buf.iterator += sizeof(topic.last_failure_local_position_timestamp);
	buf.offset += sizeof(topic.last_failure_local_position_timestamp);
	static_assert(sizeof(topic.last_failure_local_position_timestamp_sample) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.last_failure_local_position_timestamp_sample, sizeof(topic.last_failure_local_position_timestamp_sample));
	buf.iterator += sizeof(topic.last_failure_local_position_timestamp_sample);
	buf.offset += sizeof(topic.last_failure_local_position_timestamp_sample);
	static_assert(sizeof(topic.last_failure_local_position_sample_advanced) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.last_failure_local_position_sample_advanced, sizeof(topic.last_failure_local_position_sample_advanced));
	buf.iterator += sizeof(topic.last_failure_local_position_sample_advanced);
	buf.offset += sizeof(topic.last_failure_local_position_sample_advanced);
	static_assert(sizeof(topic.last_failure_z_reset_counter) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.last_failure_z_reset_counter, sizeof(topic.last_failure_z_reset_counter));
	buf.iterator += sizeof(topic.last_failure_z_reset_counter);
	buf.offset += sizeof(topic.last_failure_z_reset_counter);
	static_assert(sizeof(topic.last_failure_vz_reset_counter) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.last_failure_vz_reset_counter, sizeof(topic.last_failure_vz_reset_counter));
	buf.iterator += sizeof(topic.last_failure_vz_reset_counter);
	buf.offset += sizeof(topic.last_failure_vz_reset_counter);
	static_assert(sizeof(topic.last_failure_dist_bottom_reset_counter) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.last_failure_dist_bottom_reset_counter, sizeof(topic.last_failure_dist_bottom_reset_counter));
	buf.iterator += sizeof(topic.last_failure_dist_bottom_reset_counter);
	buf.offset += sizeof(topic.last_failure_dist_bottom_reset_counter);
	return true;
}

static inline bool ucdr_deserialize_g_p_e_n_m_p_c_hte_publication(ucdrBuffer& buf, g_p_e_n_m_p_c_hte_publication_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.timestamp_sample) == 8, "size mismatch");
	memcpy(&topic.timestamp_sample, buf.iterator, sizeof(topic.timestamp_sample));
	if (topic.timestamp_sample == 0) topic.timestamp_sample = hrt_absolute_time();
	else topic.timestamp_sample = math::min(topic.timestamp_sample - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp_sample);
	buf.offset += sizeof(topic.timestamp_sample);
	static_assert(sizeof(topic.hover_thrust) == 4, "size mismatch");
	memcpy(&topic.hover_thrust, buf.iterator, sizeof(topic.hover_thrust));
	buf.iterator += sizeof(topic.hover_thrust);
	buf.offset += sizeof(topic.hover_thrust);
	static_assert(sizeof(topic.hover_thrust_var) == 4, "size mismatch");
	memcpy(&topic.hover_thrust_var, buf.iterator, sizeof(topic.hover_thrust_var));
	buf.iterator += sizeof(topic.hover_thrust_var);
	buf.offset += sizeof(topic.hover_thrust_var);
	static_assert(sizeof(topic.valid) == 1, "size mismatch");
	memcpy(&topic.valid, buf.iterator, sizeof(topic.valid));
	buf.iterator += sizeof(topic.valid);
	buf.offset += sizeof(topic.valid);
	static_assert(sizeof(topic.eligibility_reason) == 1, "size mismatch");
	memcpy(&topic.eligibility_reason, buf.iterator, sizeof(topic.eligibility_reason));
	buf.iterator += sizeof(topic.eligibility_reason);
	buf.offset += sizeof(topic.eligibility_reason);
	static_assert(sizeof(topic.last_eligibility_failure_reason) == 1, "size mismatch");
	memcpy(&topic.last_eligibility_failure_reason, buf.iterator, sizeof(topic.last_eligibility_failure_reason));
	buf.iterator += sizeof(topic.last_eligibility_failure_reason);
	buf.offset += sizeof(topic.last_eligibility_failure_reason);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.eligibility_failure_count) == 4, "size mismatch");
	memcpy(&topic.eligibility_failure_count, buf.iterator, sizeof(topic.eligibility_failure_count));
	buf.iterator += sizeof(topic.eligibility_failure_count);
	buf.offset += sizeof(topic.eligibility_failure_count);
	static_assert(sizeof(topic.last_eligibility_failure_timestamp) == 8, "size mismatch");
	memcpy(&topic.last_eligibility_failure_timestamp, buf.iterator, sizeof(topic.last_eligibility_failure_timestamp));
	buf.iterator += sizeof(topic.last_eligibility_failure_timestamp);
	buf.offset += sizeof(topic.last_eligibility_failure_timestamp);
	static_assert(sizeof(topic.eligibility_available) == 1, "size mismatch");
	memcpy(&topic.eligibility_available, buf.iterator, sizeof(topic.eligibility_available));
	buf.iterator += sizeof(topic.eligibility_available);
	buf.offset += sizeof(topic.eligibility_available);
	static_assert(sizeof(topic.eligibility_armed) == 1, "size mismatch");
	memcpy(&topic.eligibility_armed, buf.iterator, sizeof(topic.eligibility_armed));
	buf.iterator += sizeof(topic.eligibility_armed);
	buf.offset += sizeof(topic.eligibility_armed);
	static_assert(sizeof(topic.eligibility_in_air) == 1, "size mismatch");
	memcpy(&topic.eligibility_in_air, buf.iterator, sizeof(topic.eligibility_in_air));
	buf.iterator += sizeof(topic.eligibility_in_air);
	buf.offset += sizeof(topic.eligibility_in_air);
	static_assert(sizeof(topic.eligibility_landed) == 1, "size mismatch");
	memcpy(&topic.eligibility_landed, buf.iterator, sizeof(topic.eligibility_landed));
	buf.iterator += sizeof(topic.eligibility_landed);
	buf.offset += sizeof(topic.eligibility_landed);
	static_assert(sizeof(topic.local_position_dt) == 4, "size mismatch");
	memcpy(&topic.local_position_dt, buf.iterator, sizeof(topic.local_position_dt));
	buf.iterator += sizeof(topic.local_position_dt);
	buf.offset += sizeof(topic.local_position_dt);
	static_assert(sizeof(topic.local_position_timestamp) == 8, "size mismatch");
	memcpy(&topic.local_position_timestamp, buf.iterator, sizeof(topic.local_position_timestamp));
	buf.iterator += sizeof(topic.local_position_timestamp);
	buf.offset += sizeof(topic.local_position_timestamp);
	static_assert(sizeof(topic.local_position_timestamp_sample) == 8, "size mismatch");
	memcpy(&topic.local_position_timestamp_sample, buf.iterator, sizeof(topic.local_position_timestamp_sample));
	buf.iterator += sizeof(topic.local_position_timestamp_sample);
	buf.offset += sizeof(topic.local_position_timestamp_sample);
	static_assert(sizeof(topic.local_position_sample_advanced) == 1, "size mismatch");
	memcpy(&topic.local_position_sample_advanced, buf.iterator, sizeof(topic.local_position_sample_advanced));
	buf.iterator += sizeof(topic.local_position_sample_advanced);
	buf.offset += sizeof(topic.local_position_sample_advanced);
	static_assert(sizeof(topic.local_position_z_reset_counter) == 1, "size mismatch");
	memcpy(&topic.local_position_z_reset_counter, buf.iterator, sizeof(topic.local_position_z_reset_counter));
	buf.iterator += sizeof(topic.local_position_z_reset_counter);
	buf.offset += sizeof(topic.local_position_z_reset_counter);
	static_assert(sizeof(topic.local_position_vz_reset_counter) == 1, "size mismatch");
	memcpy(&topic.local_position_vz_reset_counter, buf.iterator, sizeof(topic.local_position_vz_reset_counter));
	buf.iterator += sizeof(topic.local_position_vz_reset_counter);
	buf.offset += sizeof(topic.local_position_vz_reset_counter);
	static_assert(sizeof(topic.local_position_dist_bottom_reset_counter) == 1, "size mismatch");
	memcpy(&topic.local_position_dist_bottom_reset_counter, buf.iterator, sizeof(topic.local_position_dist_bottom_reset_counter));
	buf.iterator += sizeof(topic.local_position_dist_bottom_reset_counter);
	buf.offset += sizeof(topic.local_position_dist_bottom_reset_counter);
	static_assert(sizeof(topic.last_failure_armed) == 1, "size mismatch");
	memcpy(&topic.last_failure_armed, buf.iterator, sizeof(topic.last_failure_armed));
	buf.iterator += sizeof(topic.last_failure_armed);
	buf.offset += sizeof(topic.last_failure_armed);
	static_assert(sizeof(topic.last_failure_in_air) == 1, "size mismatch");
	memcpy(&topic.last_failure_in_air, buf.iterator, sizeof(topic.last_failure_in_air));
	buf.iterator += sizeof(topic.last_failure_in_air);
	buf.offset += sizeof(topic.last_failure_in_air);
	static_assert(sizeof(topic.last_failure_landed) == 1, "size mismatch");
	memcpy(&topic.last_failure_landed, buf.iterator, sizeof(topic.last_failure_landed));
	buf.iterator += sizeof(topic.last_failure_landed);
	buf.offset += sizeof(topic.last_failure_landed);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.last_failure_local_position_dt) == 4, "size mismatch");
	memcpy(&topic.last_failure_local_position_dt, buf.iterator, sizeof(topic.last_failure_local_position_dt));
	buf.iterator += sizeof(topic.last_failure_local_position_dt);
	buf.offset += sizeof(topic.last_failure_local_position_dt);
	buf.iterator += 4; // padding
	buf.offset += 4; // padding
	static_assert(sizeof(topic.last_failure_local_position_timestamp) == 8, "size mismatch");
	memcpy(&topic.last_failure_local_position_timestamp, buf.iterator, sizeof(topic.last_failure_local_position_timestamp));
	buf.iterator += sizeof(topic.last_failure_local_position_timestamp);
	buf.offset += sizeof(topic.last_failure_local_position_timestamp);
	static_assert(sizeof(topic.last_failure_local_position_timestamp_sample) == 8, "size mismatch");
	memcpy(&topic.last_failure_local_position_timestamp_sample, buf.iterator, sizeof(topic.last_failure_local_position_timestamp_sample));
	buf.iterator += sizeof(topic.last_failure_local_position_timestamp_sample);
	buf.offset += sizeof(topic.last_failure_local_position_timestamp_sample);
	static_assert(sizeof(topic.last_failure_local_position_sample_advanced) == 1, "size mismatch");
	memcpy(&topic.last_failure_local_position_sample_advanced, buf.iterator, sizeof(topic.last_failure_local_position_sample_advanced));
	buf.iterator += sizeof(topic.last_failure_local_position_sample_advanced);
	buf.offset += sizeof(topic.last_failure_local_position_sample_advanced);
	static_assert(sizeof(topic.last_failure_z_reset_counter) == 1, "size mismatch");
	memcpy(&topic.last_failure_z_reset_counter, buf.iterator, sizeof(topic.last_failure_z_reset_counter));
	buf.iterator += sizeof(topic.last_failure_z_reset_counter);
	buf.offset += sizeof(topic.last_failure_z_reset_counter);
	static_assert(sizeof(topic.last_failure_vz_reset_counter) == 1, "size mismatch");
	memcpy(&topic.last_failure_vz_reset_counter, buf.iterator, sizeof(topic.last_failure_vz_reset_counter));
	buf.iterator += sizeof(topic.last_failure_vz_reset_counter);
	buf.offset += sizeof(topic.last_failure_vz_reset_counter);
	static_assert(sizeof(topic.last_failure_dist_bottom_reset_counter) == 1, "size mismatch");
	memcpy(&topic.last_failure_dist_bottom_reset_counter, buf.iterator, sizeof(topic.last_failure_dist_bottom_reset_counter));
	buf.iterator += sizeof(topic.last_failure_dist_bottom_reset_counter);
	buf.offset += sizeof(topic.last_failure_dist_bottom_reset_counter);
	return true;
}
