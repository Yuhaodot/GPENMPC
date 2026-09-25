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
#include <uORB/topics/gpenmpc_vertical_state_coupling_status.h>


static inline constexpr int ucdr_topic_size_gpenmpc_vertical_state_coupling_status()
{
	return 48;
}

static inline bool ucdr_serialize_gpenmpc_vertical_state_coupling_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const gpenmpc_vertical_state_coupling_status_s& topic = *static_cast<const gpenmpc_vertical_state_coupling_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.valid) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.valid, sizeof(topic.valid));
	buf.iterator += sizeof(topic.valid);
	buf.offset += sizeof(topic.valid);
	static_assert(sizeof(topic.active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.active, sizeof(topic.active));
	buf.iterator += sizeof(topic.active);
	buf.offset += sizeof(topic.active);
	static_assert(sizeof(topic.hard_fault) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.hard_fault, sizeof(topic.hard_fault));
	buf.iterator += sizeof(topic.hard_fault);
	buf.offset += sizeof(topic.hard_fault);
	static_assert(sizeof(topic.source_available) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.source_available, sizeof(topic.source_available));
	buf.iterator += sizeof(topic.source_available);
	buf.offset += sizeof(topic.source_available);
	static_assert(sizeof(topic.plant_truth_used) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.plant_truth_used, sizeof(topic.plant_truth_used));
	buf.iterator += sizeof(topic.plant_truth_used);
	buf.offset += sizeof(topic.plant_truth_used);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.availability_hold_s) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.availability_hold_s, sizeof(topic.availability_hold_s));
	buf.iterator += sizeof(topic.availability_hold_s);
	buf.offset += sizeof(topic.availability_hold_s);
	static_assert(sizeof(topic.position_target_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.position_target_m, sizeof(topic.position_target_m));
	buf.iterator += sizeof(topic.position_target_m);
	buf.offset += sizeof(topic.position_target_m);
	static_assert(sizeof(topic.position_applied_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.position_applied_m, sizeof(topic.position_applied_m));
	buf.iterator += sizeof(topic.position_applied_m);
	buf.offset += sizeof(topic.position_applied_m);
	static_assert(sizeof(topic.position_rate_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.position_rate_mps, sizeof(topic.position_rate_mps));
	buf.iterator += sizeof(topic.position_rate_mps);
	buf.offset += sizeof(topic.position_rate_mps);
	static_assert(sizeof(topic.velocity_target_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.velocity_target_mps, sizeof(topic.velocity_target_mps));
	buf.iterator += sizeof(topic.velocity_target_mps);
	buf.offset += sizeof(topic.velocity_target_mps);
	static_assert(sizeof(topic.velocity_applied_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.velocity_applied_mps, sizeof(topic.velocity_applied_mps));
	buf.iterator += sizeof(topic.velocity_applied_mps);
	buf.offset += sizeof(topic.velocity_applied_mps);
	static_assert(sizeof(topic.acceleration_applied_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.acceleration_applied_mps2, sizeof(topic.acceleration_applied_mps2));
	buf.iterator += sizeof(topic.acceleration_applied_mps2);
	buf.offset += sizeof(topic.acceleration_applied_mps2);
	static_assert(sizeof(topic.jerk_applied_mps3) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.jerk_applied_mps3, sizeof(topic.jerk_applied_mps3));
	buf.iterator += sizeof(topic.jerk_applied_mps3);
	buf.offset += sizeof(topic.jerk_applied_mps3);
	return true;
}

static inline bool ucdr_deserialize_gpenmpc_vertical_state_coupling_status(ucdrBuffer& buf, gpenmpc_vertical_state_coupling_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.valid) == 1, "size mismatch");
	memcpy(&topic.valid, buf.iterator, sizeof(topic.valid));
	buf.iterator += sizeof(topic.valid);
	buf.offset += sizeof(topic.valid);
	static_assert(sizeof(topic.active) == 1, "size mismatch");
	memcpy(&topic.active, buf.iterator, sizeof(topic.active));
	buf.iterator += sizeof(topic.active);
	buf.offset += sizeof(topic.active);
	static_assert(sizeof(topic.hard_fault) == 1, "size mismatch");
	memcpy(&topic.hard_fault, buf.iterator, sizeof(topic.hard_fault));
	buf.iterator += sizeof(topic.hard_fault);
	buf.offset += sizeof(topic.hard_fault);
	static_assert(sizeof(topic.source_available) == 1, "size mismatch");
	memcpy(&topic.source_available, buf.iterator, sizeof(topic.source_available));
	buf.iterator += sizeof(topic.source_available);
	buf.offset += sizeof(topic.source_available);
	static_assert(sizeof(topic.plant_truth_used) == 1, "size mismatch");
	memcpy(&topic.plant_truth_used, buf.iterator, sizeof(topic.plant_truth_used));
	buf.iterator += sizeof(topic.plant_truth_used);
	buf.offset += sizeof(topic.plant_truth_used);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.availability_hold_s) == 4, "size mismatch");
	memcpy(&topic.availability_hold_s, buf.iterator, sizeof(topic.availability_hold_s));
	buf.iterator += sizeof(topic.availability_hold_s);
	buf.offset += sizeof(topic.availability_hold_s);
	static_assert(sizeof(topic.position_target_m) == 4, "size mismatch");
	memcpy(&topic.position_target_m, buf.iterator, sizeof(topic.position_target_m));
	buf.iterator += sizeof(topic.position_target_m);
	buf.offset += sizeof(topic.position_target_m);
	static_assert(sizeof(topic.position_applied_m) == 4, "size mismatch");
	memcpy(&topic.position_applied_m, buf.iterator, sizeof(topic.position_applied_m));
	buf.iterator += sizeof(topic.position_applied_m);
	buf.offset += sizeof(topic.position_applied_m);
	static_assert(sizeof(topic.position_rate_mps) == 4, "size mismatch");
	memcpy(&topic.position_rate_mps, buf.iterator, sizeof(topic.position_rate_mps));
	buf.iterator += sizeof(topic.position_rate_mps);
	buf.offset += sizeof(topic.position_rate_mps);
	static_assert(sizeof(topic.velocity_target_mps) == 4, "size mismatch");
	memcpy(&topic.velocity_target_mps, buf.iterator, sizeof(topic.velocity_target_mps));
	buf.iterator += sizeof(topic.velocity_target_mps);
	buf.offset += sizeof(topic.velocity_target_mps);
	static_assert(sizeof(topic.velocity_applied_mps) == 4, "size mismatch");
	memcpy(&topic.velocity_applied_mps, buf.iterator, sizeof(topic.velocity_applied_mps));
	buf.iterator += sizeof(topic.velocity_applied_mps);
	buf.offset += sizeof(topic.velocity_applied_mps);
	static_assert(sizeof(topic.acceleration_applied_mps2) == 4, "size mismatch");
	memcpy(&topic.acceleration_applied_mps2, buf.iterator, sizeof(topic.acceleration_applied_mps2));
	buf.iterator += sizeof(topic.acceleration_applied_mps2);
	buf.offset += sizeof(topic.acceleration_applied_mps2);
	static_assert(sizeof(topic.jerk_applied_mps3) == 4, "size mismatch");
	memcpy(&topic.jerk_applied_mps3, buf.iterator, sizeof(topic.jerk_applied_mps3));
	buf.iterator += sizeof(topic.jerk_applied_mps3);
	buf.offset += sizeof(topic.jerk_applied_mps3);
	return true;
}
