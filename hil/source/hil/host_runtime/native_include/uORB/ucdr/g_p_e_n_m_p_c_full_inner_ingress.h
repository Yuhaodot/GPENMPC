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
#include <uORB/topics/g_p_e_n_m_p_c_full_inner_ingress.h>


static inline constexpr int ucdr_topic_size_g_p_e_n_m_p_c_full_inner_ingress()
{
	return 208;
}

static inline bool ucdr_serialize_g_p_e_n_m_p_c_full_inner_ingress(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const g_p_e_n_m_p_c_full_inner_ingress_s& topic = *static_cast<const g_p_e_n_m_p_c_full_inner_ingress_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.reception_sequence) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.reception_sequence, sizeof(topic.reception_sequence));
	buf.iterator += sizeof(topic.reception_sequence);
	buf.offset += sizeof(topic.reception_sequence);
	static_assert(sizeof(topic.receiver_instance) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.receiver_instance, sizeof(topic.receiver_instance));
	buf.iterator += sizeof(topic.receiver_instance);
	buf.offset += sizeof(topic.receiver_instance);
	static_assert(sizeof(topic.source_system) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.source_system, sizeof(topic.source_system));
	buf.iterator += sizeof(topic.source_system);
	buf.offset += sizeof(topic.source_system);
	static_assert(sizeof(topic.source_component) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.source_component, sizeof(topic.source_component));
	buf.iterator += sizeof(topic.source_component);
	buf.offset += sizeof(topic.source_component);
	static_assert(sizeof(topic.target_system) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.target_system, sizeof(topic.target_system));
	buf.iterator += sizeof(topic.target_system);
	buf.offset += sizeof(topic.target_system);
	static_assert(sizeof(topic.target_component) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.target_component, sizeof(topic.target_component));
	buf.iterator += sizeof(topic.target_component);
	buf.offset += sizeof(topic.target_component);
	static_assert(sizeof(topic.mavlink_sequence) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.mavlink_sequence, sizeof(topic.mavlink_sequence));
	buf.iterator += sizeof(topic.mavlink_sequence);
	buf.offset += sizeof(topic.mavlink_sequence);
	static_assert(sizeof(topic.wire_payload_length) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.wire_payload_length, sizeof(topic.wire_payload_length));
	buf.iterator += sizeof(topic.wire_payload_length);
	buf.offset += sizeof(topic.wire_payload_length);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.payload_type) == 2, "size mismatch");
	memcpy(buf.iterator, &topic.payload_type, sizeof(topic.payload_type));
	buf.iterator += sizeof(topic.payload_type);
	buf.offset += sizeof(topic.payload_type);
	static_assert(sizeof(topic.payload_length) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.payload_length, sizeof(topic.payload_length));
	buf.iterator += sizeof(topic.payload_length);
	buf.offset += sizeof(topic.payload_length);
	static_assert(sizeof(topic.payload) == 128, "size mismatch");
	memcpy(buf.iterator, &topic.payload, sizeof(topic.payload));
	buf.iterator += sizeof(topic.payload);
	buf.offset += sizeof(topic.payload);
	static_assert(sizeof(topic.ingress_first_fault) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.ingress_first_fault, sizeof(topic.ingress_first_fault));
	buf.iterator += sizeof(topic.ingress_first_fault);
	buf.offset += sizeof(topic.ingress_first_fault);
	buf.iterator += 4; // padding
	buf.offset += 4; // padding
	static_assert(sizeof(topic.ingress_first_fault_hrt) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.ingress_first_fault_hrt, sizeof(topic.ingress_first_fault_hrt));
	buf.iterator += sizeof(topic.ingress_first_fault_hrt);
	buf.offset += sizeof(topic.ingress_first_fault_hrt);
	static_assert(sizeof(topic.ingress_last_fault) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.ingress_last_fault, sizeof(topic.ingress_last_fault));
	buf.iterator += sizeof(topic.ingress_last_fault);
	buf.offset += sizeof(topic.ingress_last_fault);
	buf.iterator += 7; // padding
	buf.offset += 7; // padding
	static_assert(sizeof(topic.ingress_last_fault_hrt) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.ingress_last_fault_hrt, sizeof(topic.ingress_last_fault_hrt));
	buf.iterator += sizeof(topic.ingress_last_fault_hrt);
	buf.offset += sizeof(topic.ingress_last_fault_hrt);
	static_assert(sizeof(topic.ingress_rejected_total) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.ingress_rejected_total, sizeof(topic.ingress_rejected_total));
	buf.iterator += sizeof(topic.ingress_rejected_total);
	buf.offset += sizeof(topic.ingress_rejected_total);
	static_assert(sizeof(topic.ingress_queue_overflows) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.ingress_queue_overflows, sizeof(topic.ingress_queue_overflows));
	buf.iterator += sizeof(topic.ingress_queue_overflows);
	buf.offset += sizeof(topic.ingress_queue_overflows);
	static_assert(sizeof(topic.ingress_publication_failures) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.ingress_publication_failures, sizeof(topic.ingress_publication_failures));
	buf.iterator += sizeof(topic.ingress_publication_failures);
	buf.offset += sizeof(topic.ingress_publication_failures);
	return true;
}

static inline bool ucdr_deserialize_g_p_e_n_m_p_c_full_inner_ingress(ucdrBuffer& buf, g_p_e_n_m_p_c_full_inner_ingress_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.reception_sequence) == 8, "size mismatch");
	memcpy(&topic.reception_sequence, buf.iterator, sizeof(topic.reception_sequence));
	buf.iterator += sizeof(topic.reception_sequence);
	buf.offset += sizeof(topic.reception_sequence);
	static_assert(sizeof(topic.receiver_instance) == 1, "size mismatch");
	memcpy(&topic.receiver_instance, buf.iterator, sizeof(topic.receiver_instance));
	buf.iterator += sizeof(topic.receiver_instance);
	buf.offset += sizeof(topic.receiver_instance);
	static_assert(sizeof(topic.source_system) == 1, "size mismatch");
	memcpy(&topic.source_system, buf.iterator, sizeof(topic.source_system));
	buf.iterator += sizeof(topic.source_system);
	buf.offset += sizeof(topic.source_system);
	static_assert(sizeof(topic.source_component) == 1, "size mismatch");
	memcpy(&topic.source_component, buf.iterator, sizeof(topic.source_component));
	buf.iterator += sizeof(topic.source_component);
	buf.offset += sizeof(topic.source_component);
	static_assert(sizeof(topic.target_system) == 1, "size mismatch");
	memcpy(&topic.target_system, buf.iterator, sizeof(topic.target_system));
	buf.iterator += sizeof(topic.target_system);
	buf.offset += sizeof(topic.target_system);
	static_assert(sizeof(topic.target_component) == 1, "size mismatch");
	memcpy(&topic.target_component, buf.iterator, sizeof(topic.target_component));
	buf.iterator += sizeof(topic.target_component);
	buf.offset += sizeof(topic.target_component);
	static_assert(sizeof(topic.mavlink_sequence) == 1, "size mismatch");
	memcpy(&topic.mavlink_sequence, buf.iterator, sizeof(topic.mavlink_sequence));
	buf.iterator += sizeof(topic.mavlink_sequence);
	buf.offset += sizeof(topic.mavlink_sequence);
	static_assert(sizeof(topic.wire_payload_length) == 1, "size mismatch");
	memcpy(&topic.wire_payload_length, buf.iterator, sizeof(topic.wire_payload_length));
	buf.iterator += sizeof(topic.wire_payload_length);
	buf.offset += sizeof(topic.wire_payload_length);
	buf.iterator += 1; // padding
	buf.offset += 1; // padding
	static_assert(sizeof(topic.payload_type) == 2, "size mismatch");
	memcpy(&topic.payload_type, buf.iterator, sizeof(topic.payload_type));
	buf.iterator += sizeof(topic.payload_type);
	buf.offset += sizeof(topic.payload_type);
	static_assert(sizeof(topic.payload_length) == 1, "size mismatch");
	memcpy(&topic.payload_length, buf.iterator, sizeof(topic.payload_length));
	buf.iterator += sizeof(topic.payload_length);
	buf.offset += sizeof(topic.payload_length);
	static_assert(sizeof(topic.payload) == 128, "size mismatch");
	memcpy(&topic.payload, buf.iterator, sizeof(topic.payload));
	buf.iterator += sizeof(topic.payload);
	buf.offset += sizeof(topic.payload);
	static_assert(sizeof(topic.ingress_first_fault) == 1, "size mismatch");
	memcpy(&topic.ingress_first_fault, buf.iterator, sizeof(topic.ingress_first_fault));
	buf.iterator += sizeof(topic.ingress_first_fault);
	buf.offset += sizeof(topic.ingress_first_fault);
	buf.iterator += 4; // padding
	buf.offset += 4; // padding
	static_assert(sizeof(topic.ingress_first_fault_hrt) == 8, "size mismatch");
	memcpy(&topic.ingress_first_fault_hrt, buf.iterator, sizeof(topic.ingress_first_fault_hrt));
	buf.iterator += sizeof(topic.ingress_first_fault_hrt);
	buf.offset += sizeof(topic.ingress_first_fault_hrt);
	static_assert(sizeof(topic.ingress_last_fault) == 1, "size mismatch");
	memcpy(&topic.ingress_last_fault, buf.iterator, sizeof(topic.ingress_last_fault));
	buf.iterator += sizeof(topic.ingress_last_fault);
	buf.offset += sizeof(topic.ingress_last_fault);
	buf.iterator += 7; // padding
	buf.offset += 7; // padding
	static_assert(sizeof(topic.ingress_last_fault_hrt) == 8, "size mismatch");
	memcpy(&topic.ingress_last_fault_hrt, buf.iterator, sizeof(topic.ingress_last_fault_hrt));
	buf.iterator += sizeof(topic.ingress_last_fault_hrt);
	buf.offset += sizeof(topic.ingress_last_fault_hrt);
	static_assert(sizeof(topic.ingress_rejected_total) == 8, "size mismatch");
	memcpy(&topic.ingress_rejected_total, buf.iterator, sizeof(topic.ingress_rejected_total));
	buf.iterator += sizeof(topic.ingress_rejected_total);
	buf.offset += sizeof(topic.ingress_rejected_total);
	static_assert(sizeof(topic.ingress_queue_overflows) == 8, "size mismatch");
	memcpy(&topic.ingress_queue_overflows, buf.iterator, sizeof(topic.ingress_queue_overflows));
	buf.iterator += sizeof(topic.ingress_queue_overflows);
	buf.offset += sizeof(topic.ingress_queue_overflows);
	static_assert(sizeof(topic.ingress_publication_failures) == 8, "size mismatch");
	memcpy(&topic.ingress_publication_failures, buf.iterator, sizeof(topic.ingress_publication_failures));
	buf.iterator += sizeof(topic.ingress_publication_failures);
	buf.offset += sizeof(topic.ingress_publication_failures);
	return true;
}
