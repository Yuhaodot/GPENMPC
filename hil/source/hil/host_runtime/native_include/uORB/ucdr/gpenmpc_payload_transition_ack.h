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
#include <uORB/topics/gpenmpc_payload_transition_ack.h>


static inline constexpr int ucdr_topic_size_gpenmpc_payload_transition_ack()
{
	return 76;
}

static inline bool ucdr_serialize_gpenmpc_payload_transition_ack(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const gpenmpc_payload_transition_ack_s& topic = *static_cast<const gpenmpc_payload_transition_ack_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.source_status_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.source_status_timestamp, sizeof(topic.source_status_timestamp));
	buf.iterator += sizeof(topic.source_status_timestamp);
	buf.offset += sizeof(topic.source_status_timestamp);
	static_assert(sizeof(topic.last_application_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.last_application_timestamp, sizeof(topic.last_application_timestamp));
	buf.iterator += sizeof(topic.last_application_timestamp);
	buf.offset += sizeof(topic.last_application_timestamp);
	static_assert(sizeof(topic.service_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.service_timestamp, sizeof(topic.service_timestamp));
	buf.iterator += sizeof(topic.service_timestamp);
	buf.offset += sizeof(topic.service_timestamp);
	static_assert(sizeof(topic.latest_hte_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.latest_hte_timestamp, sizeof(topic.latest_hte_timestamp));
	buf.iterator += sizeof(topic.latest_hte_timestamp);
	buf.offset += sizeof(topic.latest_hte_timestamp);
	static_assert(sizeof(topic.application_count) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.application_count, sizeof(topic.application_count));
	buf.iterator += sizeof(topic.application_count);
	buf.offset += sizeof(topic.application_count);
	static_assert(sizeof(topic.service_count) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.service_count, sizeof(topic.service_count));
	buf.iterator += sizeof(topic.service_count);
	buf.offset += sizeof(topic.service_count);
	static_assert(sizeof(topic.service_sequence) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.service_sequence, sizeof(topic.service_sequence));
	buf.iterator += sizeof(topic.service_sequence);
	buf.offset += sizeof(topic.service_sequence);
	static_assert(sizeof(topic.hte_update_suppression_count) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hte_update_suppression_count, sizeof(topic.hte_update_suppression_count));
	buf.iterator += sizeof(topic.hte_update_suppression_count);
	buf.offset += sizeof(topic.hte_update_suppression_count);
	static_assert(sizeof(topic.same_timestamp_zoh_reuse_count) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.same_timestamp_zoh_reuse_count, sizeof(topic.same_timestamp_zoh_reuse_count));
	buf.iterator += sizeof(topic.same_timestamp_zoh_reuse_count);
	buf.offset += sizeof(topic.same_timestamp_zoh_reuse_count);
	static_assert(sizeof(topic.fault) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.fault, sizeof(topic.fault));
	buf.iterator += sizeof(topic.fault);
	buf.offset += sizeof(topic.fault);
	static_assert(sizeof(topic.application_valid) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.application_valid, sizeof(topic.application_valid));
	buf.iterator += sizeof(topic.application_valid);
	buf.offset += sizeof(topic.application_valid);
	static_assert(sizeof(topic.new_application) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.new_application, sizeof(topic.new_application));
	buf.iterator += sizeof(topic.new_application);
	buf.offset += sizeof(topic.new_application);
	static_assert(sizeof(topic.transition_enabled) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.transition_enabled, sizeof(topic.transition_enabled));
	buf.iterator += sizeof(topic.transition_enabled);
	buf.offset += sizeof(topic.transition_enabled);
	static_assert(sizeof(topic.host_feedforward_disabled) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.host_feedforward_disabled, sizeof(topic.host_feedforward_disabled));
	buf.iterator += sizeof(topic.host_feedforward_disabled);
	buf.offset += sizeof(topic.host_feedforward_disabled);
	static_assert(sizeof(topic.hte_target_hold_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.hte_target_hold_active, sizeof(topic.hte_target_hold_active));
	buf.iterator += sizeof(topic.hte_target_hold_active);
	buf.offset += sizeof(topic.hte_target_hold_active);
	buf.iterator += 2; // padding
	buf.offset += 2; // padding
	static_assert(sizeof(topic.applied_hover_thrust) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.applied_hover_thrust, sizeof(topic.applied_hover_thrust));
	buf.iterator += sizeof(topic.applied_hover_thrust);
	buf.offset += sizeof(topic.applied_hover_thrust);
	static_assert(sizeof(topic.latest_native_hte) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.latest_native_hte, sizeof(topic.latest_native_hte));
	buf.iterator += sizeof(topic.latest_native_hte);
	buf.offset += sizeof(topic.latest_native_hte);
	return true;
}

static inline bool ucdr_deserialize_gpenmpc_payload_transition_ack(ucdrBuffer& buf, gpenmpc_payload_transition_ack_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.source_status_timestamp) == 8, "size mismatch");
	memcpy(&topic.source_status_timestamp, buf.iterator, sizeof(topic.source_status_timestamp));
	buf.iterator += sizeof(topic.source_status_timestamp);
	buf.offset += sizeof(topic.source_status_timestamp);
	static_assert(sizeof(topic.last_application_timestamp) == 8, "size mismatch");
	memcpy(&topic.last_application_timestamp, buf.iterator, sizeof(topic.last_application_timestamp));
	buf.iterator += sizeof(topic.last_application_timestamp);
	buf.offset += sizeof(topic.last_application_timestamp);
	static_assert(sizeof(topic.service_timestamp) == 8, "size mismatch");
	memcpy(&topic.service_timestamp, buf.iterator, sizeof(topic.service_timestamp));
	buf.iterator += sizeof(topic.service_timestamp);
	buf.offset += sizeof(topic.service_timestamp);
	static_assert(sizeof(topic.latest_hte_timestamp) == 8, "size mismatch");
	memcpy(&topic.latest_hte_timestamp, buf.iterator, sizeof(topic.latest_hte_timestamp));
	buf.iterator += sizeof(topic.latest_hte_timestamp);
	buf.offset += sizeof(topic.latest_hte_timestamp);
	static_assert(sizeof(topic.application_count) == 4, "size mismatch");
	memcpy(&topic.application_count, buf.iterator, sizeof(topic.application_count));
	buf.iterator += sizeof(topic.application_count);
	buf.offset += sizeof(topic.application_count);
	static_assert(sizeof(topic.service_count) == 4, "size mismatch");
	memcpy(&topic.service_count, buf.iterator, sizeof(topic.service_count));
	buf.iterator += sizeof(topic.service_count);
	buf.offset += sizeof(topic.service_count);
	static_assert(sizeof(topic.service_sequence) == 4, "size mismatch");
	memcpy(&topic.service_sequence, buf.iterator, sizeof(topic.service_sequence));
	buf.iterator += sizeof(topic.service_sequence);
	buf.offset += sizeof(topic.service_sequence);
	static_assert(sizeof(topic.hte_update_suppression_count) == 4, "size mismatch");
	memcpy(&topic.hte_update_suppression_count, buf.iterator, sizeof(topic.hte_update_suppression_count));
	buf.iterator += sizeof(topic.hte_update_suppression_count);
	buf.offset += sizeof(topic.hte_update_suppression_count);
	static_assert(sizeof(topic.same_timestamp_zoh_reuse_count) == 4, "size mismatch");
	memcpy(&topic.same_timestamp_zoh_reuse_count, buf.iterator, sizeof(topic.same_timestamp_zoh_reuse_count));
	buf.iterator += sizeof(topic.same_timestamp_zoh_reuse_count);
	buf.offset += sizeof(topic.same_timestamp_zoh_reuse_count);
	static_assert(sizeof(topic.fault) == 1, "size mismatch");
	memcpy(&topic.fault, buf.iterator, sizeof(topic.fault));
	buf.iterator += sizeof(topic.fault);
	buf.offset += sizeof(topic.fault);
	static_assert(sizeof(topic.application_valid) == 1, "size mismatch");
	memcpy(&topic.application_valid, buf.iterator, sizeof(topic.application_valid));
	buf.iterator += sizeof(topic.application_valid);
	buf.offset += sizeof(topic.application_valid);
	static_assert(sizeof(topic.new_application) == 1, "size mismatch");
	memcpy(&topic.new_application, buf.iterator, sizeof(topic.new_application));
	buf.iterator += sizeof(topic.new_application);
	buf.offset += sizeof(topic.new_application);
	static_assert(sizeof(topic.transition_enabled) == 1, "size mismatch");
	memcpy(&topic.transition_enabled, buf.iterator, sizeof(topic.transition_enabled));
	buf.iterator += sizeof(topic.transition_enabled);
	buf.offset += sizeof(topic.transition_enabled);
	static_assert(sizeof(topic.host_feedforward_disabled) == 1, "size mismatch");
	memcpy(&topic.host_feedforward_disabled, buf.iterator, sizeof(topic.host_feedforward_disabled));
	buf.iterator += sizeof(topic.host_feedforward_disabled);
	buf.offset += sizeof(topic.host_feedforward_disabled);
	static_assert(sizeof(topic.hte_target_hold_active) == 1, "size mismatch");
	memcpy(&topic.hte_target_hold_active, buf.iterator, sizeof(topic.hte_target_hold_active));
	buf.iterator += sizeof(topic.hte_target_hold_active);
	buf.offset += sizeof(topic.hte_target_hold_active);
	buf.iterator += 2; // padding
	buf.offset += 2; // padding
	static_assert(sizeof(topic.applied_hover_thrust) == 4, "size mismatch");
	memcpy(&topic.applied_hover_thrust, buf.iterator, sizeof(topic.applied_hover_thrust));
	buf.iterator += sizeof(topic.applied_hover_thrust);
	buf.offset += sizeof(topic.applied_hover_thrust);
	static_assert(sizeof(topic.latest_native_hte) == 4, "size mismatch");
	memcpy(&topic.latest_native_hte, buf.iterator, sizeof(topic.latest_native_hte));
	buf.iterator += sizeof(topic.latest_native_hte);
	buf.offset += sizeof(topic.latest_native_hte);
	return true;
}
