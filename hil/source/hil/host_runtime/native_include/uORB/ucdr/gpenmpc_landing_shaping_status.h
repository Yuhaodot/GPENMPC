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
#include <uORB/topics/gpenmpc_landing_shaping_status.h>


static inline constexpr int ucdr_topic_size_gpenmpc_landing_shaping_status()
{
	return 146;
}

static inline bool ucdr_serialize_gpenmpc_landing_shaping_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const gpenmpc_landing_shaping_status_s& topic = *static_cast<const gpenmpc_landing_shaping_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.payload_status_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.payload_status_timestamp, sizeof(topic.payload_status_timestamp));
	buf.iterator += sizeof(topic.payload_status_timestamp);
	buf.offset += sizeof(topic.payload_status_timestamp);
	static_assert(sizeof(topic.payload_service_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.payload_service_timestamp, sizeof(topic.payload_service_timestamp));
	buf.iterator += sizeof(topic.payload_service_timestamp);
	buf.offset += sizeof(topic.payload_service_timestamp);
	static_assert(sizeof(topic.service_count) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.service_count, sizeof(topic.service_count));
	buf.iterator += sizeof(topic.service_count);
	buf.offset += sizeof(topic.service_count);
	static_assert(sizeof(topic.service_sequence) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.service_sequence, sizeof(topic.service_sequence));
	buf.iterator += sizeof(topic.service_sequence);
	buf.offset += sizeof(topic.service_sequence);
	static_assert(sizeof(topic.direct_application_ack_count) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.direct_application_ack_count, sizeof(topic.direct_application_ack_count));
	buf.iterator += sizeof(topic.direct_application_ack_count);
	buf.offset += sizeof(topic.direct_application_ack_count);
	static_assert(sizeof(topic.distance_source) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.distance_source, sizeof(topic.distance_source));
	buf.iterator += sizeof(topic.distance_source);
	buf.offset += sizeof(topic.distance_source);
	static_assert(sizeof(topic.payload_fault) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.payload_fault, sizeof(topic.payload_fault));
	buf.iterator += sizeof(topic.payload_fault);
	buf.offset += sizeof(topic.payload_fault);
	static_assert(sizeof(topic.payload_status_fresh) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.payload_status_fresh, sizeof(topic.payload_status_fresh));
	buf.iterator += sizeof(topic.payload_status_fresh);
	buf.offset += sizeof(topic.payload_status_fresh);
	static_assert(sizeof(topic.gpenmpc_contract_valid) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.gpenmpc_contract_valid, sizeof(topic.gpenmpc_contract_valid));
	buf.iterator += sizeof(topic.gpenmpc_contract_valid);
	buf.offset += sizeof(topic.gpenmpc_contract_valid);
	static_assert(sizeof(topic.transition_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.transition_active, sizeof(topic.transition_active));
	buf.iterator += sizeof(topic.transition_active);
	buf.offset += sizeof(topic.transition_active);
	static_assert(sizeof(topic.recovery_requested) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.recovery_requested, sizeof(topic.recovery_requested));
	buf.iterator += sizeof(topic.recovery_requested);
	buf.offset += sizeof(topic.recovery_requested);
	static_assert(sizeof(topic.range_distance_available) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.range_distance_available, sizeof(topic.range_distance_available));
	buf.iterator += sizeof(topic.range_distance_available);
	buf.offset += sizeof(topic.range_distance_available);
	static_assert(sizeof(topic.estimated_ground_crawl_eligible) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.estimated_ground_crawl_eligible, sizeof(topic.estimated_ground_crawl_eligible));
	buf.iterator += sizeof(topic.estimated_ground_crawl_eligible);
	buf.offset += sizeof(topic.estimated_ground_crawl_eligible);
	static_assert(sizeof(topic.crawl_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.crawl_active, sizeof(topic.crawl_active));
	buf.iterator += sizeof(topic.crawl_active);
	buf.offset += sizeof(topic.crawl_active);
	static_assert(sizeof(topic.touchdown_authority_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.touchdown_authority_active, sizeof(topic.touchdown_authority_active));
	buf.iterator += sizeof(topic.touchdown_authority_active);
	buf.offset += sizeof(topic.touchdown_authority_active);
	static_assert(sizeof(topic.ground_observer_status_fresh) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.ground_observer_status_fresh, sizeof(topic.ground_observer_status_fresh));
	buf.iterator += sizeof(topic.ground_observer_status_fresh);
	buf.offset += sizeof(topic.ground_observer_status_fresh);
	static_assert(sizeof(topic.ground_observer_usable) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.ground_observer_usable, sizeof(topic.ground_observer_usable));
	buf.iterator += sizeof(topic.ground_observer_usable);
	buf.offset += sizeof(topic.ground_observer_usable);
	static_assert(sizeof(topic.ground_observer_precise_hil_contract) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.ground_observer_precise_hil_contract, sizeof(topic.ground_observer_precise_hil_contract));
	buf.iterator += sizeof(topic.ground_observer_precise_hil_contract);
	buf.offset += sizeof(topic.ground_observer_precise_hil_contract);
	static_assert(sizeof(topic.ground_observer_plant_truth_used) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.ground_observer_plant_truth_used, sizeof(topic.ground_observer_plant_truth_used));
	buf.iterator += sizeof(topic.ground_observer_plant_truth_used);
	buf.offset += sizeof(topic.ground_observer_plant_truth_used);
	buf.iterator += 2; // padding
	buf.offset += 2; // padding
	static_assert(sizeof(topic.distance_to_ground_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.distance_to_ground_m, sizeof(topic.distance_to_ground_m));
	buf.iterator += sizeof(topic.distance_to_ground_m);
	buf.offset += sizeof(topic.distance_to_ground_m);
	static_assert(sizeof(topic.distance_to_bottom_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.distance_to_bottom_m, sizeof(topic.distance_to_bottom_m));
	buf.iterator += sizeof(topic.distance_to_bottom_m);
	buf.offset += sizeof(topic.distance_to_bottom_m);
	static_assert(sizeof(topic.land_alt3_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.land_alt3_m, sizeof(topic.land_alt3_m));
	buf.iterator += sizeof(topic.land_alt3_m);
	buf.offset += sizeof(topic.land_alt3_m);
	static_assert(sizeof(topic.land_speed_unshaped_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.land_speed_unshaped_mps, sizeof(topic.land_speed_unshaped_mps));
	buf.iterator += sizeof(topic.land_speed_unshaped_mps);
	buf.offset += sizeof(topic.land_speed_unshaped_mps);
	static_assert(sizeof(topic.land_crawl_speed_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.land_crawl_speed_mps, sizeof(topic.land_crawl_speed_mps));
	buf.iterator += sizeof(topic.land_crawl_speed_mps);
	buf.offset += sizeof(topic.land_crawl_speed_mps);
	static_assert(sizeof(topic.selected_vertical_speed_mps) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.selected_vertical_speed_mps, sizeof(topic.selected_vertical_speed_mps));
	buf.iterator += sizeof(topic.selected_vertical_speed_mps);
	buf.offset += sizeof(topic.selected_vertical_speed_mps);
	static_assert(sizeof(topic.touchdown_authority_entry_distance_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.touchdown_authority_entry_distance_m, sizeof(topic.touchdown_authority_entry_distance_m));
	buf.iterator += sizeof(topic.touchdown_authority_entry_distance_m);
	buf.offset += sizeof(topic.touchdown_authority_entry_distance_m);
	static_assert(sizeof(topic.touchdown_authority_full_distance_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.touchdown_authority_full_distance_m, sizeof(topic.touchdown_authority_full_distance_m));
	buf.iterator += sizeof(topic.touchdown_authority_full_distance_m);
	buf.offset += sizeof(topic.touchdown_authority_full_distance_m);
	static_assert(sizeof(topic.touchdown_authority_max_acceleration_d_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.touchdown_authority_max_acceleration_d_mps2, sizeof(topic.touchdown_authority_max_acceleration_d_mps2));
	buf.iterator += sizeof(topic.touchdown_authority_max_acceleration_d_mps2);
	buf.offset += sizeof(topic.touchdown_authority_max_acceleration_d_mps2);
	static_assert(sizeof(topic.touchdown_authority_blend) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.touchdown_authority_blend, sizeof(topic.touchdown_authority_blend));
	buf.iterator += sizeof(topic.touchdown_authority_blend);
	buf.offset += sizeof(topic.touchdown_authority_blend);
	static_assert(sizeof(topic.touchdown_authority_acceleration_d_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.touchdown_authority_acceleration_d_mps2, sizeof(topic.touchdown_authority_acceleration_d_mps2));
	buf.iterator += sizeof(topic.touchdown_authority_acceleration_d_mps2);
	buf.offset += sizeof(topic.touchdown_authority_acceleration_d_mps2);
	static_assert(sizeof(topic.touchdown_authority_desired_acceleration_d_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.touchdown_authority_desired_acceleration_d_mps2, sizeof(topic.touchdown_authority_desired_acceleration_d_mps2));
	buf.iterator += sizeof(topic.touchdown_authority_desired_acceleration_d_mps2);
	buf.offset += sizeof(topic.touchdown_authority_desired_acceleration_d_mps2);
	static_assert(sizeof(topic.touchdown_authority_rise_rate_mps3) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.touchdown_authority_rise_rate_mps3, sizeof(topic.touchdown_authority_rise_rate_mps3));
	buf.iterator += sizeof(topic.touchdown_authority_rise_rate_mps3);
	buf.offset += sizeof(topic.touchdown_authority_rise_rate_mps3);
	static_assert(sizeof(topic.touchdown_authority_fall_rate_mps3) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.touchdown_authority_fall_rate_mps3, sizeof(topic.touchdown_authority_fall_rate_mps3));
	buf.iterator += sizeof(topic.touchdown_authority_fall_rate_mps3);
	buf.offset += sizeof(topic.touchdown_authority_fall_rate_mps3);
	static_assert(sizeof(topic.touchdown_settling_blend) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.touchdown_settling_blend, sizeof(topic.touchdown_settling_blend));
	buf.iterator += sizeof(topic.touchdown_settling_blend);
	buf.offset += sizeof(topic.touchdown_settling_blend);
	static_assert(sizeof(topic.touchdown_hover_thrust_estimate) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.touchdown_hover_thrust_estimate, sizeof(topic.touchdown_hover_thrust_estimate));
	buf.iterator += sizeof(topic.touchdown_hover_thrust_estimate);
	buf.offset += sizeof(topic.touchdown_hover_thrust_estimate);
	static_assert(sizeof(topic.touchdown_hover_thrust_target) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.touchdown_hover_thrust_target, sizeof(topic.touchdown_hover_thrust_target));
	buf.iterator += sizeof(topic.touchdown_hover_thrust_target);
	buf.offset += sizeof(topic.touchdown_hover_thrust_target);
	static_assert(sizeof(topic.touchdown_trim_mismatch_acceleration_d_mps2) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.touchdown_trim_mismatch_acceleration_d_mps2, sizeof(topic.touchdown_trim_mismatch_acceleration_d_mps2));
	buf.iterator += sizeof(topic.touchdown_trim_mismatch_acceleration_d_mps2);
	buf.offset += sizeof(topic.touchdown_trim_mismatch_acceleration_d_mps2);
	static_assert(sizeof(topic.ground_observer_height_center_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.ground_observer_height_center_m, sizeof(topic.ground_observer_height_center_m));
	buf.iterator += sizeof(topic.ground_observer_height_center_m);
	buf.offset += sizeof(topic.ground_observer_height_center_m);
	static_assert(sizeof(topic.ground_observer_height_lower_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.ground_observer_height_lower_m, sizeof(topic.ground_observer_height_lower_m));
	buf.iterator += sizeof(topic.ground_observer_height_lower_m);
	buf.offset += sizeof(topic.ground_observer_height_lower_m);
	static_assert(sizeof(topic.ground_observer_height_upper_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.ground_observer_height_upper_m, sizeof(topic.ground_observer_height_upper_m));
	buf.iterator += sizeof(topic.ground_observer_height_upper_m);
	buf.offset += sizeof(topic.ground_observer_height_upper_m);
	static_assert(sizeof(topic.ground_observer_baro_bound_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.ground_observer_baro_bound_m, sizeof(topic.ground_observer_baro_bound_m));
	buf.iterator += sizeof(topic.ground_observer_baro_bound_m);
	buf.offset += sizeof(topic.ground_observer_baro_bound_m);
	static_assert(sizeof(topic.ground_observer_gps_bound_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.ground_observer_gps_bound_m, sizeof(topic.ground_observer_gps_bound_m));
	buf.iterator += sizeof(topic.ground_observer_gps_bound_m);
	buf.offset += sizeof(topic.ground_observer_gps_bound_m);
	static_assert(sizeof(topic.ground_observer_state) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.ground_observer_state, sizeof(topic.ground_observer_state));
	buf.iterator += sizeof(topic.ground_observer_state);
	buf.offset += sizeof(topic.ground_observer_state);
	static_assert(sizeof(topic.ground_observer_reason) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.ground_observer_reason, sizeof(topic.ground_observer_reason));
	buf.iterator += sizeof(topic.ground_observer_reason);
	buf.offset += sizeof(topic.ground_observer_reason);
	return true;
}

static inline bool ucdr_deserialize_gpenmpc_landing_shaping_status(ucdrBuffer& buf, gpenmpc_landing_shaping_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.payload_status_timestamp) == 8, "size mismatch");
	memcpy(&topic.payload_status_timestamp, buf.iterator, sizeof(topic.payload_status_timestamp));
	buf.iterator += sizeof(topic.payload_status_timestamp);
	buf.offset += sizeof(topic.payload_status_timestamp);
	static_assert(sizeof(topic.payload_service_timestamp) == 8, "size mismatch");
	memcpy(&topic.payload_service_timestamp, buf.iterator, sizeof(topic.payload_service_timestamp));
	buf.iterator += sizeof(topic.payload_service_timestamp);
	buf.offset += sizeof(topic.payload_service_timestamp);
	static_assert(sizeof(topic.service_count) == 4, "size mismatch");
	memcpy(&topic.service_count, buf.iterator, sizeof(topic.service_count));
	buf.iterator += sizeof(topic.service_count);
	buf.offset += sizeof(topic.service_count);
	static_assert(sizeof(topic.service_sequence) == 4, "size mismatch");
	memcpy(&topic.service_sequence, buf.iterator, sizeof(topic.service_sequence));
	buf.iterator += sizeof(topic.service_sequence);
	buf.offset += sizeof(topic.service_sequence);
	static_assert(sizeof(topic.direct_application_ack_count) == 4, "size mismatch");
	memcpy(&topic.direct_application_ack_count, buf.iterator, sizeof(topic.direct_application_ack_count));
	buf.iterator += sizeof(topic.direct_application_ack_count);
	buf.offset += sizeof(topic.direct_application_ack_count);
	static_assert(sizeof(topic.distance_source) == 1, "size mismatch");
	memcpy(&topic.distance_source, buf.iterator, sizeof(topic.distance_source));
	buf.iterator += sizeof(topic.distance_source);
	buf.offset += sizeof(topic.distance_source);
	static_assert(sizeof(topic.payload_fault) == 1, "size mismatch");
	memcpy(&topic.payload_fault, buf.iterator, sizeof(topic.payload_fault));
	buf.iterator += sizeof(topic.payload_fault);
	buf.offset += sizeof(topic.payload_fault);
	static_assert(sizeof(topic.payload_status_fresh) == 1, "size mismatch");
	memcpy(&topic.payload_status_fresh, buf.iterator, sizeof(topic.payload_status_fresh));
	buf.iterator += sizeof(topic.payload_status_fresh);
	buf.offset += sizeof(topic.payload_status_fresh);
	static_assert(sizeof(topic.gpenmpc_contract_valid) == 1, "size mismatch");
	memcpy(&topic.gpenmpc_contract_valid, buf.iterator, sizeof(topic.gpenmpc_contract_valid));
	buf.iterator += sizeof(topic.gpenmpc_contract_valid);
	buf.offset += sizeof(topic.gpenmpc_contract_valid);
	static_assert(sizeof(topic.transition_active) == 1, "size mismatch");
	memcpy(&topic.transition_active, buf.iterator, sizeof(topic.transition_active));
	buf.iterator += sizeof(topic.transition_active);
	buf.offset += sizeof(topic.transition_active);
	static_assert(sizeof(topic.recovery_requested) == 1, "size mismatch");
	memcpy(&topic.recovery_requested, buf.iterator, sizeof(topic.recovery_requested));
	buf.iterator += sizeof(topic.recovery_requested);
	buf.offset += sizeof(topic.recovery_requested);
	static_assert(sizeof(topic.range_distance_available) == 1, "size mismatch");
	memcpy(&topic.range_distance_available, buf.iterator, sizeof(topic.range_distance_available));
	buf.iterator += sizeof(topic.range_distance_available);
	buf.offset += sizeof(topic.range_distance_available);
	static_assert(sizeof(topic.estimated_ground_crawl_eligible) == 1, "size mismatch");
	memcpy(&topic.estimated_ground_crawl_eligible, buf.iterator, sizeof(topic.estimated_ground_crawl_eligible));
	buf.iterator += sizeof(topic.estimated_ground_crawl_eligible);
	buf.offset += sizeof(topic.estimated_ground_crawl_eligible);
	static_assert(sizeof(topic.crawl_active) == 1, "size mismatch");
	memcpy(&topic.crawl_active, buf.iterator, sizeof(topic.crawl_active));
	buf.iterator += sizeof(topic.crawl_active);
	buf.offset += sizeof(topic.crawl_active);
	static_assert(sizeof(topic.touchdown_authority_active) == 1, "size mismatch");
	memcpy(&topic.touchdown_authority_active, buf.iterator, sizeof(topic.touchdown_authority_active));
	buf.iterator += sizeof(topic.touchdown_authority_active);
	buf.offset += sizeof(topic.touchdown_authority_active);
	static_assert(sizeof(topic.ground_observer_status_fresh) == 1, "size mismatch");
	memcpy(&topic.ground_observer_status_fresh, buf.iterator, sizeof(topic.ground_observer_status_fresh));
	buf.iterator += sizeof(topic.ground_observer_status_fresh);
	buf.offset += sizeof(topic.ground_observer_status_fresh);
	static_assert(sizeof(topic.ground_observer_usable) == 1, "size mismatch");
	memcpy(&topic.ground_observer_usable, buf.iterator, sizeof(topic.ground_observer_usable));
	buf.iterator += sizeof(topic.ground_observer_usable);
	buf.offset += sizeof(topic.ground_observer_usable);
	static_assert(sizeof(topic.ground_observer_precise_hil_contract) == 1, "size mismatch");
	memcpy(&topic.ground_observer_precise_hil_contract, buf.iterator, sizeof(topic.ground_observer_precise_hil_contract));
	buf.iterator += sizeof(topic.ground_observer_precise_hil_contract);
	buf.offset += sizeof(topic.ground_observer_precise_hil_contract);
	static_assert(sizeof(topic.ground_observer_plant_truth_used) == 1, "size mismatch");
	memcpy(&topic.ground_observer_plant_truth_used, buf.iterator, sizeof(topic.ground_observer_plant_truth_used));
	buf.iterator += sizeof(topic.ground_observer_plant_truth_used);
	buf.offset += sizeof(topic.ground_observer_plant_truth_used);
	buf.iterator += 2; // padding
	buf.offset += 2; // padding
	static_assert(sizeof(topic.distance_to_ground_m) == 4, "size mismatch");
	memcpy(&topic.distance_to_ground_m, buf.iterator, sizeof(topic.distance_to_ground_m));
	buf.iterator += sizeof(topic.distance_to_ground_m);
	buf.offset += sizeof(topic.distance_to_ground_m);
	static_assert(sizeof(topic.distance_to_bottom_m) == 4, "size mismatch");
	memcpy(&topic.distance_to_bottom_m, buf.iterator, sizeof(topic.distance_to_bottom_m));
	buf.iterator += sizeof(topic.distance_to_bottom_m);
	buf.offset += sizeof(topic.distance_to_bottom_m);
	static_assert(sizeof(topic.land_alt3_m) == 4, "size mismatch");
	memcpy(&topic.land_alt3_m, buf.iterator, sizeof(topic.land_alt3_m));
	buf.iterator += sizeof(topic.land_alt3_m);
	buf.offset += sizeof(topic.land_alt3_m);
	static_assert(sizeof(topic.land_speed_unshaped_mps) == 4, "size mismatch");
	memcpy(&topic.land_speed_unshaped_mps, buf.iterator, sizeof(topic.land_speed_unshaped_mps));
	buf.iterator += sizeof(topic.land_speed_unshaped_mps);
	buf.offset += sizeof(topic.land_speed_unshaped_mps);
	static_assert(sizeof(topic.land_crawl_speed_mps) == 4, "size mismatch");
	memcpy(&topic.land_crawl_speed_mps, buf.iterator, sizeof(topic.land_crawl_speed_mps));
	buf.iterator += sizeof(topic.land_crawl_speed_mps);
	buf.offset += sizeof(topic.land_crawl_speed_mps);
	static_assert(sizeof(topic.selected_vertical_speed_mps) == 4, "size mismatch");
	memcpy(&topic.selected_vertical_speed_mps, buf.iterator, sizeof(topic.selected_vertical_speed_mps));
	buf.iterator += sizeof(topic.selected_vertical_speed_mps);
	buf.offset += sizeof(topic.selected_vertical_speed_mps);
	static_assert(sizeof(topic.touchdown_authority_entry_distance_m) == 4, "size mismatch");
	memcpy(&topic.touchdown_authority_entry_distance_m, buf.iterator, sizeof(topic.touchdown_authority_entry_distance_m));
	buf.iterator += sizeof(topic.touchdown_authority_entry_distance_m);
	buf.offset += sizeof(topic.touchdown_authority_entry_distance_m);
	static_assert(sizeof(topic.touchdown_authority_full_distance_m) == 4, "size mismatch");
	memcpy(&topic.touchdown_authority_full_distance_m, buf.iterator, sizeof(topic.touchdown_authority_full_distance_m));
	buf.iterator += sizeof(topic.touchdown_authority_full_distance_m);
	buf.offset += sizeof(topic.touchdown_authority_full_distance_m);
	static_assert(sizeof(topic.touchdown_authority_max_acceleration_d_mps2) == 4, "size mismatch");
	memcpy(&topic.touchdown_authority_max_acceleration_d_mps2, buf.iterator, sizeof(topic.touchdown_authority_max_acceleration_d_mps2));
	buf.iterator += sizeof(topic.touchdown_authority_max_acceleration_d_mps2);
	buf.offset += sizeof(topic.touchdown_authority_max_acceleration_d_mps2);
	static_assert(sizeof(topic.touchdown_authority_blend) == 4, "size mismatch");
	memcpy(&topic.touchdown_authority_blend, buf.iterator, sizeof(topic.touchdown_authority_blend));
	buf.iterator += sizeof(topic.touchdown_authority_blend);
	buf.offset += sizeof(topic.touchdown_authority_blend);
	static_assert(sizeof(topic.touchdown_authority_acceleration_d_mps2) == 4, "size mismatch");
	memcpy(&topic.touchdown_authority_acceleration_d_mps2, buf.iterator, sizeof(topic.touchdown_authority_acceleration_d_mps2));
	buf.iterator += sizeof(topic.touchdown_authority_acceleration_d_mps2);
	buf.offset += sizeof(topic.touchdown_authority_acceleration_d_mps2);
	static_assert(sizeof(topic.touchdown_authority_desired_acceleration_d_mps2) == 4, "size mismatch");
	memcpy(&topic.touchdown_authority_desired_acceleration_d_mps2, buf.iterator, sizeof(topic.touchdown_authority_desired_acceleration_d_mps2));
	buf.iterator += sizeof(topic.touchdown_authority_desired_acceleration_d_mps2);
	buf.offset += sizeof(topic.touchdown_authority_desired_acceleration_d_mps2);
	static_assert(sizeof(topic.touchdown_authority_rise_rate_mps3) == 4, "size mismatch");
	memcpy(&topic.touchdown_authority_rise_rate_mps3, buf.iterator, sizeof(topic.touchdown_authority_rise_rate_mps3));
	buf.iterator += sizeof(topic.touchdown_authority_rise_rate_mps3);
	buf.offset += sizeof(topic.touchdown_authority_rise_rate_mps3);
	static_assert(sizeof(topic.touchdown_authority_fall_rate_mps3) == 4, "size mismatch");
	memcpy(&topic.touchdown_authority_fall_rate_mps3, buf.iterator, sizeof(topic.touchdown_authority_fall_rate_mps3));
	buf.iterator += sizeof(topic.touchdown_authority_fall_rate_mps3);
	buf.offset += sizeof(topic.touchdown_authority_fall_rate_mps3);
	static_assert(sizeof(topic.touchdown_settling_blend) == 4, "size mismatch");
	memcpy(&topic.touchdown_settling_blend, buf.iterator, sizeof(topic.touchdown_settling_blend));
	buf.iterator += sizeof(topic.touchdown_settling_blend);
	buf.offset += sizeof(topic.touchdown_settling_blend);
	static_assert(sizeof(topic.touchdown_hover_thrust_estimate) == 4, "size mismatch");
	memcpy(&topic.touchdown_hover_thrust_estimate, buf.iterator, sizeof(topic.touchdown_hover_thrust_estimate));
	buf.iterator += sizeof(topic.touchdown_hover_thrust_estimate);
	buf.offset += sizeof(topic.touchdown_hover_thrust_estimate);
	static_assert(sizeof(topic.touchdown_hover_thrust_target) == 4, "size mismatch");
	memcpy(&topic.touchdown_hover_thrust_target, buf.iterator, sizeof(topic.touchdown_hover_thrust_target));
	buf.iterator += sizeof(topic.touchdown_hover_thrust_target);
	buf.offset += sizeof(topic.touchdown_hover_thrust_target);
	static_assert(sizeof(topic.touchdown_trim_mismatch_acceleration_d_mps2) == 4, "size mismatch");
	memcpy(&topic.touchdown_trim_mismatch_acceleration_d_mps2, buf.iterator, sizeof(topic.touchdown_trim_mismatch_acceleration_d_mps2));
	buf.iterator += sizeof(topic.touchdown_trim_mismatch_acceleration_d_mps2);
	buf.offset += sizeof(topic.touchdown_trim_mismatch_acceleration_d_mps2);
	static_assert(sizeof(topic.ground_observer_height_center_m) == 4, "size mismatch");
	memcpy(&topic.ground_observer_height_center_m, buf.iterator, sizeof(topic.ground_observer_height_center_m));
	buf.iterator += sizeof(topic.ground_observer_height_center_m);
	buf.offset += sizeof(topic.ground_observer_height_center_m);
	static_assert(sizeof(topic.ground_observer_height_lower_m) == 4, "size mismatch");
	memcpy(&topic.ground_observer_height_lower_m, buf.iterator, sizeof(topic.ground_observer_height_lower_m));
	buf.iterator += sizeof(topic.ground_observer_height_lower_m);
	buf.offset += sizeof(topic.ground_observer_height_lower_m);
	static_assert(sizeof(topic.ground_observer_height_upper_m) == 4, "size mismatch");
	memcpy(&topic.ground_observer_height_upper_m, buf.iterator, sizeof(topic.ground_observer_height_upper_m));
	buf.iterator += sizeof(topic.ground_observer_height_upper_m);
	buf.offset += sizeof(topic.ground_observer_height_upper_m);
	static_assert(sizeof(topic.ground_observer_baro_bound_m) == 4, "size mismatch");
	memcpy(&topic.ground_observer_baro_bound_m, buf.iterator, sizeof(topic.ground_observer_baro_bound_m));
	buf.iterator += sizeof(topic.ground_observer_baro_bound_m);
	buf.offset += sizeof(topic.ground_observer_baro_bound_m);
	static_assert(sizeof(topic.ground_observer_gps_bound_m) == 4, "size mismatch");
	memcpy(&topic.ground_observer_gps_bound_m, buf.iterator, sizeof(topic.ground_observer_gps_bound_m));
	buf.iterator += sizeof(topic.ground_observer_gps_bound_m);
	buf.offset += sizeof(topic.ground_observer_gps_bound_m);
	static_assert(sizeof(topic.ground_observer_state) == 1, "size mismatch");
	memcpy(&topic.ground_observer_state, buf.iterator, sizeof(topic.ground_observer_state));
	buf.iterator += sizeof(topic.ground_observer_state);
	buf.offset += sizeof(topic.ground_observer_state);
	static_assert(sizeof(topic.ground_observer_reason) == 1, "size mismatch");
	memcpy(&topic.ground_observer_reason, buf.iterator, sizeof(topic.ground_observer_reason));
	buf.iterator += sizeof(topic.ground_observer_reason);
	buf.offset += sizeof(topic.ground_observer_reason);
	return true;
}
