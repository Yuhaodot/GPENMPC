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
#include <uORB/topics/g_p_e_n_m_p_c_original_hil_receipt.h>


static inline constexpr int ucdr_topic_size_g_p_e_n_m_p_c_original_hil_receipt()
{
	return 144;
}

static inline bool ucdr_serialize_g_p_e_n_m_p_c_original_hil_receipt(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const g_p_e_n_m_p_c_original_hil_receipt_s& topic = *static_cast<const g_p_e_n_m_p_c_original_hil_receipt_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.wire_time_usec) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.wire_time_usec, sizeof(topic.wire_time_usec));
	buf.iterator += sizeof(topic.wire_time_usec);
	buf.offset += sizeof(topic.wire_time_usec);
	static_assert(sizeof(topic.receiver_address) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.receiver_address, sizeof(topic.receiver_address));
	buf.iterator += sizeof(topic.receiver_address);
	buf.offset += sizeof(topic.receiver_address);
	static_assert(sizeof(topic.link_address) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.link_address, sizeof(topic.link_address));
	buf.iterator += sizeof(topic.link_address);
	buf.offset += sizeof(topic.link_address);
	static_assert(sizeof(topic.original_event_sequence) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.original_event_sequence, sizeof(topic.original_event_sequence));
	buf.iterator += sizeof(topic.original_event_sequence);
	buf.offset += sizeof(topic.original_event_sequence);
	static_assert(sizeof(topic.prior_publication_failures) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.prior_publication_failures, sizeof(topic.prior_publication_failures));
	buf.iterator += sizeof(topic.prior_publication_failures);
	buf.offset += sizeof(topic.prior_publication_failures);
	static_assert(sizeof(topic.receiver_instance) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.receiver_instance, sizeof(topic.receiver_instance));
	buf.iterator += sizeof(topic.receiver_instance);
	buf.offset += sizeof(topic.receiver_instance);
	static_assert(sizeof(topic.channel) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.channel, sizeof(topic.channel));
	buf.iterator += sizeof(topic.channel);
	buf.offset += sizeof(topic.channel);
	static_assert(sizeof(topic.fields_updated) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.fields_updated, sizeof(topic.fields_updated));
	buf.iterator += sizeof(topic.fields_updated);
	buf.offset += sizeof(topic.fields_updated);
	static_assert(sizeof(topic.gyro_topic_instance) == 2, "size mismatch");
	memcpy(buf.iterator, &topic.gyro_topic_instance, sizeof(topic.gyro_topic_instance));
	buf.iterator += sizeof(topic.gyro_topic_instance);
	buf.offset += sizeof(topic.gyro_topic_instance);
	static_assert(sizeof(topic.accel_topic_instance) == 2, "size mismatch");
	memcpy(buf.iterator, &topic.accel_topic_instance, sizeof(topic.accel_topic_instance));
	buf.iterator += sizeof(topic.accel_topic_instance);
	buf.offset += sizeof(topic.accel_topic_instance);
	static_assert(sizeof(topic.gyro_device_id) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.gyro_device_id, sizeof(topic.gyro_device_id));
	buf.iterator += sizeof(topic.gyro_device_id);
	buf.offset += sizeof(topic.gyro_device_id);
	static_assert(sizeof(topic.accel_device_id) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.accel_device_id, sizeof(topic.accel_device_id));
	buf.iterator += sizeof(topic.accel_device_id);
	buf.offset += sizeof(topic.accel_device_id);
	static_assert(sizeof(topic.system_id) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.system_id, sizeof(topic.system_id));
	buf.iterator += sizeof(topic.system_id);
	buf.offset += sizeof(topic.system_id);
	static_assert(sizeof(topic.component_id) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.component_id, sizeof(topic.component_id));
	buf.iterator += sizeof(topic.component_id);
	buf.offset += sizeof(topic.component_id);
	static_assert(sizeof(topic.mavlink_sequence) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.mavlink_sequence, sizeof(topic.mavlink_sequence));
	buf.iterator += sizeof(topic.mavlink_sequence);
	buf.offset += sizeof(topic.mavlink_sequence);
	static_assert(sizeof(topic.payload_length) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.payload_length, sizeof(topic.payload_length));
	buf.iterator += sizeof(topic.payload_length);
	buf.offset += sizeof(topic.payload_length);
	static_assert(sizeof(topic.sensor_id) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.sensor_id, sizeof(topic.sensor_id));
	buf.iterator += sizeof(topic.sensor_id);
	buf.offset += sizeof(topic.sensor_id);
	static_assert(sizeof(topic.original_payload) == 65, "size mismatch");
	memcpy(buf.iterator, &topic.original_payload, sizeof(topic.original_payload));
	buf.iterator += sizeof(topic.original_payload);
	buf.offset += sizeof(topic.original_payload);
	static_assert(sizeof(topic.gyro_update_called) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.gyro_update_called, sizeof(topic.gyro_update_called));
	buf.iterator += sizeof(topic.gyro_update_called);
	buf.offset += sizeof(topic.gyro_update_called);
	static_assert(sizeof(topic.accel_update_called) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.accel_update_called, sizeof(topic.accel_update_called));
	buf.iterator += sizeof(topic.accel_update_called);
	buf.offset += sizeof(topic.accel_update_called);
	return true;
}

static inline bool ucdr_deserialize_g_p_e_n_m_p_c_original_hil_receipt(ucdrBuffer& buf, g_p_e_n_m_p_c_original_hil_receipt_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.wire_time_usec) == 8, "size mismatch");
	memcpy(&topic.wire_time_usec, buf.iterator, sizeof(topic.wire_time_usec));
	buf.iterator += sizeof(topic.wire_time_usec);
	buf.offset += sizeof(topic.wire_time_usec);
	static_assert(sizeof(topic.receiver_address) == 8, "size mismatch");
	memcpy(&topic.receiver_address, buf.iterator, sizeof(topic.receiver_address));
	buf.iterator += sizeof(topic.receiver_address);
	buf.offset += sizeof(topic.receiver_address);
	static_assert(sizeof(topic.link_address) == 8, "size mismatch");
	memcpy(&topic.link_address, buf.iterator, sizeof(topic.link_address));
	buf.iterator += sizeof(topic.link_address);
	buf.offset += sizeof(topic.link_address);
	static_assert(sizeof(topic.original_event_sequence) == 8, "size mismatch");
	memcpy(&topic.original_event_sequence, buf.iterator, sizeof(topic.original_event_sequence));
	buf.iterator += sizeof(topic.original_event_sequence);
	buf.offset += sizeof(topic.original_event_sequence);
	static_assert(sizeof(topic.prior_publication_failures) == 8, "size mismatch");
	memcpy(&topic.prior_publication_failures, buf.iterator, sizeof(topic.prior_publication_failures));
	buf.iterator += sizeof(topic.prior_publication_failures);
	buf.offset += sizeof(topic.prior_publication_failures);
	static_assert(sizeof(topic.receiver_instance) == 4, "size mismatch");
	memcpy(&topic.receiver_instance, buf.iterator, sizeof(topic.receiver_instance));
	buf.iterator += sizeof(topic.receiver_instance);
	buf.offset += sizeof(topic.receiver_instance);
	static_assert(sizeof(topic.channel) == 4, "size mismatch");
	memcpy(&topic.channel, buf.iterator, sizeof(topic.channel));
	buf.iterator += sizeof(topic.channel);
	buf.offset += sizeof(topic.channel);
	static_assert(sizeof(topic.fields_updated) == 4, "size mismatch");
	memcpy(&topic.fields_updated, buf.iterator, sizeof(topic.fields_updated));
	buf.iterator += sizeof(topic.fields_updated);
	buf.offset += sizeof(topic.fields_updated);
	static_assert(sizeof(topic.gyro_topic_instance) == 2, "size mismatch");
	memcpy(&topic.gyro_topic_instance, buf.iterator, sizeof(topic.gyro_topic_instance));
	buf.iterator += sizeof(topic.gyro_topic_instance);
	buf.offset += sizeof(topic.gyro_topic_instance);
	static_assert(sizeof(topic.accel_topic_instance) == 2, "size mismatch");
	memcpy(&topic.accel_topic_instance, buf.iterator, sizeof(topic.accel_topic_instance));
	buf.iterator += sizeof(topic.accel_topic_instance);
	buf.offset += sizeof(topic.accel_topic_instance);
	static_assert(sizeof(topic.gyro_device_id) == 4, "size mismatch");
	memcpy(&topic.gyro_device_id, buf.iterator, sizeof(topic.gyro_device_id));
	buf.iterator += sizeof(topic.gyro_device_id);
	buf.offset += sizeof(topic.gyro_device_id);
	static_assert(sizeof(topic.accel_device_id) == 4, "size mismatch");
	memcpy(&topic.accel_device_id, buf.iterator, sizeof(topic.accel_device_id));
	buf.iterator += sizeof(topic.accel_device_id);
	buf.offset += sizeof(topic.accel_device_id);
	static_assert(sizeof(topic.system_id) == 1, "size mismatch");
	memcpy(&topic.system_id, buf.iterator, sizeof(topic.system_id));
	buf.iterator += sizeof(topic.system_id);
	buf.offset += sizeof(topic.system_id);
	static_assert(sizeof(topic.component_id) == 1, "size mismatch");
	memcpy(&topic.component_id, buf.iterator, sizeof(topic.component_id));
	buf.iterator += sizeof(topic.component_id);
	buf.offset += sizeof(topic.component_id);
	static_assert(sizeof(topic.mavlink_sequence) == 1, "size mismatch");
	memcpy(&topic.mavlink_sequence, buf.iterator, sizeof(topic.mavlink_sequence));
	buf.iterator += sizeof(topic.mavlink_sequence);
	buf.offset += sizeof(topic.mavlink_sequence);
	static_assert(sizeof(topic.payload_length) == 1, "size mismatch");
	memcpy(&topic.payload_length, buf.iterator, sizeof(topic.payload_length));
	buf.iterator += sizeof(topic.payload_length);
	buf.offset += sizeof(topic.payload_length);
	static_assert(sizeof(topic.sensor_id) == 1, "size mismatch");
	memcpy(&topic.sensor_id, buf.iterator, sizeof(topic.sensor_id));
	buf.iterator += sizeof(topic.sensor_id);
	buf.offset += sizeof(topic.sensor_id);
	static_assert(sizeof(topic.original_payload) == 65, "size mismatch");
	memcpy(&topic.original_payload, buf.iterator, sizeof(topic.original_payload));
	buf.iterator += sizeof(topic.original_payload);
	buf.offset += sizeof(topic.original_payload);
	static_assert(sizeof(topic.gyro_update_called) == 1, "size mismatch");
	memcpy(&topic.gyro_update_called, buf.iterator, sizeof(topic.gyro_update_called));
	buf.iterator += sizeof(topic.gyro_update_called);
	buf.offset += sizeof(topic.gyro_update_called);
	static_assert(sizeof(topic.accel_update_called) == 1, "size mismatch");
	memcpy(&topic.accel_update_called, buf.iterator, sizeof(topic.accel_update_called));
	buf.iterator += sizeof(topic.accel_update_called);
	buf.offset += sizeof(topic.accel_update_called);
	return true;
}
