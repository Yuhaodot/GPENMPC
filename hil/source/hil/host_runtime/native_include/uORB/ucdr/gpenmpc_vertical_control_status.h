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
#include <uORB/topics/gpenmpc_vertical_control_status.h>


static inline constexpr int ucdr_topic_size_gpenmpc_vertical_control_status()
{
	return 160;
}

static inline bool ucdr_serialize_gpenmpc_vertical_control_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const gpenmpc_vertical_control_status_s& topic = *static_cast<const gpenmpc_vertical_control_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.sample_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.sample_timestamp, sizeof(topic.sample_timestamp));
	buf.iterator += sizeof(topic.sample_timestamp);
	buf.offset += sizeof(topic.sample_timestamp);
	static_assert(sizeof(topic.route_status_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.route_status_timestamp, sizeof(topic.route_status_timestamp));
	buf.iterator += sizeof(topic.route_status_timestamp);
	buf.offset += sizeof(topic.route_status_timestamp);
	static_assert(sizeof(topic.service_timestamp) == 8, "size mismatch");
	memcpy(buf.iterator, &topic.service_timestamp, sizeof(topic.service_timestamp));
	buf.iterator += sizeof(topic.service_timestamp);
	buf.offset += sizeof(topic.service_timestamp);
	static_assert(sizeof(topic.phase) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.phase, sizeof(topic.phase));
	buf.iterator += sizeof(topic.phase);
	buf.offset += sizeof(topic.phase);
	static_assert(sizeof(topic.hte_availability_state) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.hte_availability_state, sizeof(topic.hte_availability_state));
	buf.iterator += sizeof(topic.hte_availability_state);
	buf.offset += sizeof(topic.hte_availability_state);
	static_assert(sizeof(topic.route_status_fresh) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.route_status_fresh, sizeof(topic.route_status_fresh));
	buf.iterator += sizeof(topic.route_status_fresh);
	buf.offset += sizeof(topic.route_status_fresh);
	static_assert(sizeof(topic.route_reference_valid) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.route_reference_valid, sizeof(topic.route_reference_valid));
	buf.iterator += sizeof(topic.route_reference_valid);
	buf.offset += sizeof(topic.route_reference_valid);
	static_assert(sizeof(topic.route_motion_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.route_motion_active, sizeof(topic.route_motion_active));
	buf.iterator += sizeof(topic.route_motion_active);
	buf.offset += sizeof(topic.route_motion_active);
	static_assert(sizeof(topic.local_position_ready) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.local_position_ready, sizeof(topic.local_position_ready));
	buf.iterator += sizeof(topic.local_position_ready);
	buf.offset += sizeof(topic.local_position_ready);
	static_assert(sizeof(topic.native_airborne) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.native_airborne, sizeof(topic.native_airborne));
	buf.iterator += sizeof(topic.native_airborne);
	buf.offset += sizeof(topic.native_airborne);
	static_assert(sizeof(topic.service_guard_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.service_guard_active, sizeof(topic.service_guard_active));
	buf.iterator += sizeof(topic.service_guard_active);
	buf.offset += sizeof(topic.service_guard_active);
	static_assert(sizeof(topic.native_land_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.native_land_active, sizeof(topic.native_land_active));
	buf.iterator += sizeof(topic.native_land_active);
	buf.offset += sizeof(topic.native_land_active);
	static_assert(sizeof(topic.integrator_update_allowed) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.integrator_update_allowed, sizeof(topic.integrator_update_allowed));
	buf.iterator += sizeof(topic.integrator_update_allowed);
	buf.offset += sizeof(topic.integrator_update_allowed);
	static_assert(sizeof(topic.vertical_anti_windup) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.vertical_anti_windup, sizeof(topic.vertical_anti_windup));
	buf.iterator += sizeof(topic.vertical_anti_windup);
	buf.offset += sizeof(topic.vertical_anti_windup);
	static_assert(sizeof(topic.vertical_integral_updated) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.vertical_integral_updated, sizeof(topic.vertical_integral_updated));
	buf.iterator += sizeof(topic.vertical_integral_updated);
	buf.offset += sizeof(topic.vertical_integral_updated);
	static_assert(sizeof(topic.initial_hold_integral_recenter_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.initial_hold_integral_recenter_active, sizeof(topic.initial_hold_integral_recenter_active));
	buf.iterator += sizeof(topic.initial_hold_integral_recenter_active);
	buf.offset += sizeof(topic.initial_hold_integral_recenter_active);
	static_assert(sizeof(topic.prep_recenter) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.prep_recenter, sizeof(topic.prep_recenter));
	buf.iterator += sizeof(topic.prep_recenter);
	buf.offset += sizeof(topic.prep_recenter);
	static_assert(sizeof(topic.tko_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.tko_active, sizeof(topic.tko_active));
	buf.iterator += sizeof(topic.tko_active);
	buf.offset += sizeof(topic.tko_active);
	static_assert(sizeof(topic.hte_fresh) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.hte_fresh, sizeof(topic.hte_fresh));
	buf.iterator += sizeof(topic.hte_fresh);
	buf.offset += sizeof(topic.hte_fresh);
	static_assert(sizeof(topic.route_motion_seen) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.route_motion_seen, sizeof(topic.route_motion_seen));
	buf.iterator += sizeof(topic.route_motion_seen);
	buf.offset += sizeof(topic.route_motion_seen);
	static_assert(sizeof(topic.hte_bounded_hold_authority) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.hte_bounded_hold_authority, sizeof(topic.hte_bounded_hold_authority));
	buf.iterator += sizeof(topic.hte_bounded_hold_authority);
	buf.offset += sizeof(topic.hte_bounded_hold_authority);
	static_assert(sizeof(topic.payload_application_valid) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.payload_application_valid, sizeof(topic.payload_application_valid));
	buf.iterator += sizeof(topic.payload_application_valid);
	buf.offset += sizeof(topic.payload_application_valid);
	static_assert(sizeof(topic.payload_target_hold_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.payload_target_hold_active, sizeof(topic.payload_target_hold_active));
	buf.iterator += sizeof(topic.payload_target_hold_active);
	buf.offset += sizeof(topic.payload_target_hold_active);
	static_assert(sizeof(topic.payload_fault_free) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.payload_fault_free, sizeof(topic.payload_fault_free));
	buf.iterator += sizeof(topic.payload_fault_free);
	buf.offset += sizeof(topic.payload_fault_free);
	static_assert(sizeof(topic.landing_status_fresh) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.landing_status_fresh, sizeof(topic.landing_status_fresh));
	buf.iterator += sizeof(topic.landing_status_fresh);
	buf.offset += sizeof(topic.landing_status_fresh);
	static_assert(sizeof(topic.terminal_integral_authority_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.terminal_integral_authority_active, sizeof(topic.terminal_integral_authority_active));
	buf.iterator += sizeof(topic.terminal_integral_authority_active);
	buf.offset += sizeof(topic.terminal_integral_authority_active);
	static_assert(sizeof(topic.terminal_integral_floor_slew_active) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.terminal_integral_floor_slew_active, sizeof(topic.terminal_integral_floor_slew_active));
	buf.iterator += sizeof(topic.terminal_integral_floor_slew_active);
	buf.offset += sizeof(topic.terminal_integral_floor_slew_active);
	static_assert(sizeof(topic.terminal_negative_integral_update_blocked) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.terminal_negative_integral_update_blocked, sizeof(topic.terminal_negative_integral_update_blocked));
	buf.iterator += sizeof(topic.terminal_negative_integral_update_blocked);
	buf.offset += sizeof(topic.terminal_negative_integral_update_blocked);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.horizontal_reference_speed) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.horizontal_reference_speed, sizeof(topic.horizontal_reference_speed));
	buf.iterator += sizeof(topic.horizontal_reference_speed);
	buf.offset += sizeof(topic.horizontal_reference_speed);
	static_assert(sizeof(topic.nominal_integral_gain) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.nominal_integral_gain, sizeof(topic.nominal_integral_gain));
	buf.iterator += sizeof(topic.nominal_integral_gain);
	buf.offset += sizeof(topic.nominal_integral_gain);
	static_assert(sizeof(topic.effective_integral_gain) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.effective_integral_gain, sizeof(topic.effective_integral_gain));
	buf.iterator += sizeof(topic.effective_integral_gain);
	buf.offset += sizeof(topic.effective_integral_gain);
	static_assert(sizeof(topic.target_integral_gain) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.target_integral_gain, sizeof(topic.target_integral_gain));
	buf.iterator += sizeof(topic.target_integral_gain);
	buf.offset += sizeof(topic.target_integral_gain);
	static_assert(sizeof(topic.integral_gain_slew_rate) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.integral_gain_slew_rate, sizeof(topic.integral_gain_slew_rate));
	buf.iterator += sizeof(topic.integral_gain_slew_rate);
	buf.offset += sizeof(topic.integral_gain_slew_rate);
	static_assert(sizeof(topic.initial_hold_integral_recenter_rate_mps3) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.initial_hold_integral_recenter_rate_mps3, sizeof(topic.initial_hold_integral_recenter_rate_mps3));
	buf.iterator += sizeof(topic.initial_hold_integral_recenter_rate_mps3);
	buf.offset += sizeof(topic.initial_hold_integral_recenter_rate_mps3);
	static_assert(sizeof(topic.initial_hold_integral_recenter_target_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.initial_hold_integral_recenter_target_d, sizeof(topic.initial_hold_integral_recenter_target_d));
	buf.iterator += sizeof(topic.initial_hold_integral_recenter_target_d);
	buf.offset += sizeof(topic.initial_hold_integral_recenter_target_d);
	static_assert(sizeof(topic.position_setpoint_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.position_setpoint_d, sizeof(topic.position_setpoint_d));
	buf.iterator += sizeof(topic.position_setpoint_d);
	buf.offset += sizeof(topic.position_setpoint_d);
	static_assert(sizeof(topic.position_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.position_d, sizeof(topic.position_d));
	buf.iterator += sizeof(topic.position_d);
	buf.offset += sizeof(topic.position_d);
	static_assert(sizeof(topic.position_term_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.position_term_d, sizeof(topic.position_term_d));
	buf.iterator += sizeof(topic.position_term_d);
	buf.offset += sizeof(topic.position_term_d);
	static_assert(sizeof(topic.velocity_feedforward_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.velocity_feedforward_d, sizeof(topic.velocity_feedforward_d));
	buf.iterator += sizeof(topic.velocity_feedforward_d);
	buf.offset += sizeof(topic.velocity_feedforward_d);
	static_assert(sizeof(topic.velocity_setpoint_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.velocity_setpoint_d, sizeof(topic.velocity_setpoint_d));
	buf.iterator += sizeof(topic.velocity_setpoint_d);
	buf.offset += sizeof(topic.velocity_setpoint_d);
	static_assert(sizeof(topic.velocity_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.velocity_d, sizeof(topic.velocity_d));
	buf.iterator += sizeof(topic.velocity_d);
	buf.offset += sizeof(topic.velocity_d);
	static_assert(sizeof(topic.velocity_error_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.velocity_error_d, sizeof(topic.velocity_error_d));
	buf.iterator += sizeof(topic.velocity_error_d);
	buf.offset += sizeof(topic.velocity_error_d);
	static_assert(sizeof(topic.acceleration_feedforward_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.acceleration_feedforward_d, sizeof(topic.acceleration_feedforward_d));
	buf.iterator += sizeof(topic.acceleration_feedforward_d);
	buf.offset += sizeof(topic.acceleration_feedforward_d);
	static_assert(sizeof(topic.acceleration_p_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.acceleration_p_d, sizeof(topic.acceleration_p_d));
	buf.iterator += sizeof(topic.acceleration_p_d);
	buf.offset += sizeof(topic.acceleration_p_d);
	static_assert(sizeof(topic.acceleration_i_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.acceleration_i_d, sizeof(topic.acceleration_i_d));
	buf.iterator += sizeof(topic.acceleration_i_d);
	buf.offset += sizeof(topic.acceleration_i_d);
	static_assert(sizeof(topic.acceleration_d_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.acceleration_d_d, sizeof(topic.acceleration_d_d));
	buf.iterator += sizeof(topic.acceleration_d_d);
	buf.offset += sizeof(topic.acceleration_d_d);
	static_assert(sizeof(topic.acceleration_setpoint_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.acceleration_setpoint_d, sizeof(topic.acceleration_setpoint_d));
	buf.iterator += sizeof(topic.acceleration_setpoint_d);
	buf.offset += sizeof(topic.acceleration_setpoint_d);
	static_assert(sizeof(topic.thrust_setpoint_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.thrust_setpoint_d, sizeof(topic.thrust_setpoint_d));
	buf.iterator += sizeof(topic.thrust_setpoint_d);
	buf.offset += sizeof(topic.thrust_setpoint_d);
	static_assert(sizeof(topic.hover_thrust) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.hover_thrust, sizeof(topic.hover_thrust));
	buf.iterator += sizeof(topic.hover_thrust);
	buf.offset += sizeof(topic.hover_thrust);
	static_assert(sizeof(topic.native_hte) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.native_hte, sizeof(topic.native_hte));
	buf.iterator += sizeof(topic.native_hte);
	buf.offset += sizeof(topic.native_hte);
	static_assert(sizeof(topic.terminal_estimated_distance_to_ground_m) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.terminal_estimated_distance_to_ground_m, sizeof(topic.terminal_estimated_distance_to_ground_m));
	buf.iterator += sizeof(topic.terminal_estimated_distance_to_ground_m);
	buf.offset += sizeof(topic.terminal_estimated_distance_to_ground_m);
	static_assert(sizeof(topic.terminal_integral_floor_target_d) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.terminal_integral_floor_target_d, sizeof(topic.terminal_integral_floor_target_d));
	buf.iterator += sizeof(topic.terminal_integral_floor_target_d);
	buf.offset += sizeof(topic.terminal_integral_floor_target_d);
	static_assert(sizeof(topic.terminal_integral_floor_slew_rate_mps3) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.terminal_integral_floor_slew_rate_mps3, sizeof(topic.terminal_integral_floor_slew_rate_mps3));
	buf.iterator += sizeof(topic.terminal_integral_floor_slew_rate_mps3);
	buf.offset += sizeof(topic.terminal_integral_floor_slew_rate_mps3);
	return true;
}

static inline bool ucdr_deserialize_gpenmpc_vertical_control_status(ucdrBuffer& buf, gpenmpc_vertical_control_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.sample_timestamp) == 8, "size mismatch");
	memcpy(&topic.sample_timestamp, buf.iterator, sizeof(topic.sample_timestamp));
	buf.iterator += sizeof(topic.sample_timestamp);
	buf.offset += sizeof(topic.sample_timestamp);
	static_assert(sizeof(topic.route_status_timestamp) == 8, "size mismatch");
	memcpy(&topic.route_status_timestamp, buf.iterator, sizeof(topic.route_status_timestamp));
	buf.iterator += sizeof(topic.route_status_timestamp);
	buf.offset += sizeof(topic.route_status_timestamp);
	static_assert(sizeof(topic.service_timestamp) == 8, "size mismatch");
	memcpy(&topic.service_timestamp, buf.iterator, sizeof(topic.service_timestamp));
	buf.iterator += sizeof(topic.service_timestamp);
	buf.offset += sizeof(topic.service_timestamp);
	static_assert(sizeof(topic.phase) == 1, "size mismatch");
	memcpy(&topic.phase, buf.iterator, sizeof(topic.phase));
	buf.iterator += sizeof(topic.phase);
	buf.offset += sizeof(topic.phase);
	static_assert(sizeof(topic.hte_availability_state) == 1, "size mismatch");
	memcpy(&topic.hte_availability_state, buf.iterator, sizeof(topic.hte_availability_state));
	buf.iterator += sizeof(topic.hte_availability_state);
	buf.offset += sizeof(topic.hte_availability_state);
	static_assert(sizeof(topic.route_status_fresh) == 1, "size mismatch");
	memcpy(&topic.route_status_fresh, buf.iterator, sizeof(topic.route_status_fresh));
	buf.iterator += sizeof(topic.route_status_fresh);
	buf.offset += sizeof(topic.route_status_fresh);
	static_assert(sizeof(topic.route_reference_valid) == 1, "size mismatch");
	memcpy(&topic.route_reference_valid, buf.iterator, sizeof(topic.route_reference_valid));
	buf.iterator += sizeof(topic.route_reference_valid);
	buf.offset += sizeof(topic.route_reference_valid);
	static_assert(sizeof(topic.route_motion_active) == 1, "size mismatch");
	memcpy(&topic.route_motion_active, buf.iterator, sizeof(topic.route_motion_active));
	buf.iterator += sizeof(topic.route_motion_active);
	buf.offset += sizeof(topic.route_motion_active);
	static_assert(sizeof(topic.local_position_ready) == 1, "size mismatch");
	memcpy(&topic.local_position_ready, buf.iterator, sizeof(topic.local_position_ready));
	buf.iterator += sizeof(topic.local_position_ready);
	buf.offset += sizeof(topic.local_position_ready);
	static_assert(sizeof(topic.native_airborne) == 1, "size mismatch");
	memcpy(&topic.native_airborne, buf.iterator, sizeof(topic.native_airborne));
	buf.iterator += sizeof(topic.native_airborne);
	buf.offset += sizeof(topic.native_airborne);
	static_assert(sizeof(topic.service_guard_active) == 1, "size mismatch");
	memcpy(&topic.service_guard_active, buf.iterator, sizeof(topic.service_guard_active));
	buf.iterator += sizeof(topic.service_guard_active);
	buf.offset += sizeof(topic.service_guard_active);
	static_assert(sizeof(topic.native_land_active) == 1, "size mismatch");
	memcpy(&topic.native_land_active, buf.iterator, sizeof(topic.native_land_active));
	buf.iterator += sizeof(topic.native_land_active);
	buf.offset += sizeof(topic.native_land_active);
	static_assert(sizeof(topic.integrator_update_allowed) == 1, "size mismatch");
	memcpy(&topic.integrator_update_allowed, buf.iterator, sizeof(topic.integrator_update_allowed));
	buf.iterator += sizeof(topic.integrator_update_allowed);
	buf.offset += sizeof(topic.integrator_update_allowed);
	static_assert(sizeof(topic.vertical_anti_windup) == 1, "size mismatch");
	memcpy(&topic.vertical_anti_windup, buf.iterator, sizeof(topic.vertical_anti_windup));
	buf.iterator += sizeof(topic.vertical_anti_windup);
	buf.offset += sizeof(topic.vertical_anti_windup);
	static_assert(sizeof(topic.vertical_integral_updated) == 1, "size mismatch");
	memcpy(&topic.vertical_integral_updated, buf.iterator, sizeof(topic.vertical_integral_updated));
	buf.iterator += sizeof(topic.vertical_integral_updated);
	buf.offset += sizeof(topic.vertical_integral_updated);
	static_assert(sizeof(topic.initial_hold_integral_recenter_active) == 1, "size mismatch");
	memcpy(&topic.initial_hold_integral_recenter_active, buf.iterator, sizeof(topic.initial_hold_integral_recenter_active));
	buf.iterator += sizeof(topic.initial_hold_integral_recenter_active);
	buf.offset += sizeof(topic.initial_hold_integral_recenter_active);
	static_assert(sizeof(topic.prep_recenter) == 1, "size mismatch");
	memcpy(&topic.prep_recenter, buf.iterator, sizeof(topic.prep_recenter));
	buf.iterator += sizeof(topic.prep_recenter);
	buf.offset += sizeof(topic.prep_recenter);
	static_assert(sizeof(topic.tko_active) == 1, "size mismatch");
	memcpy(&topic.tko_active, buf.iterator, sizeof(topic.tko_active));
	buf.iterator += sizeof(topic.tko_active);
	buf.offset += sizeof(topic.tko_active);
	static_assert(sizeof(topic.hte_fresh) == 1, "size mismatch");
	memcpy(&topic.hte_fresh, buf.iterator, sizeof(topic.hte_fresh));
	buf.iterator += sizeof(topic.hte_fresh);
	buf.offset += sizeof(topic.hte_fresh);
	static_assert(sizeof(topic.route_motion_seen) == 1, "size mismatch");
	memcpy(&topic.route_motion_seen, buf.iterator, sizeof(topic.route_motion_seen));
	buf.iterator += sizeof(topic.route_motion_seen);
	buf.offset += sizeof(topic.route_motion_seen);
	static_assert(sizeof(topic.hte_bounded_hold_authority) == 1, "size mismatch");
	memcpy(&topic.hte_bounded_hold_authority, buf.iterator, sizeof(topic.hte_bounded_hold_authority));
	buf.iterator += sizeof(topic.hte_bounded_hold_authority);
	buf.offset += sizeof(topic.hte_bounded_hold_authority);
	static_assert(sizeof(topic.payload_application_valid) == 1, "size mismatch");
	memcpy(&topic.payload_application_valid, buf.iterator, sizeof(topic.payload_application_valid));
	buf.iterator += sizeof(topic.payload_application_valid);
	buf.offset += sizeof(topic.payload_application_valid);
	static_assert(sizeof(topic.payload_target_hold_active) == 1, "size mismatch");
	memcpy(&topic.payload_target_hold_active, buf.iterator, sizeof(topic.payload_target_hold_active));
	buf.iterator += sizeof(topic.payload_target_hold_active);
	buf.offset += sizeof(topic.payload_target_hold_active);
	static_assert(sizeof(topic.payload_fault_free) == 1, "size mismatch");
	memcpy(&topic.payload_fault_free, buf.iterator, sizeof(topic.payload_fault_free));
	buf.iterator += sizeof(topic.payload_fault_free);
	buf.offset += sizeof(topic.payload_fault_free);
	static_assert(sizeof(topic.landing_status_fresh) == 1, "size mismatch");
	memcpy(&topic.landing_status_fresh, buf.iterator, sizeof(topic.landing_status_fresh));
	buf.iterator += sizeof(topic.landing_status_fresh);
	buf.offset += sizeof(topic.landing_status_fresh);
	static_assert(sizeof(topic.terminal_integral_authority_active) == 1, "size mismatch");
	memcpy(&topic.terminal_integral_authority_active, buf.iterator, sizeof(topic.terminal_integral_authority_active));
	buf.iterator += sizeof(topic.terminal_integral_authority_active);
	buf.offset += sizeof(topic.terminal_integral_authority_active);
	static_assert(sizeof(topic.terminal_integral_floor_slew_active) == 1, "size mismatch");
	memcpy(&topic.terminal_integral_floor_slew_active, buf.iterator, sizeof(topic.terminal_integral_floor_slew_active));
	buf.iterator += sizeof(topic.terminal_integral_floor_slew_active);
	buf.offset += sizeof(topic.terminal_integral_floor_slew_active);
	static_assert(sizeof(topic.terminal_negative_integral_update_blocked) == 1, "size mismatch");
	memcpy(&topic.terminal_negative_integral_update_blocked, buf.iterator, sizeof(topic.terminal_negative_integral_update_blocked));
	buf.iterator += sizeof(topic.terminal_negative_integral_update_blocked);
	buf.offset += sizeof(topic.terminal_negative_integral_update_blocked);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.horizontal_reference_speed) == 4, "size mismatch");
	memcpy(&topic.horizontal_reference_speed, buf.iterator, sizeof(topic.horizontal_reference_speed));
	buf.iterator += sizeof(topic.horizontal_reference_speed);
	buf.offset += sizeof(topic.horizontal_reference_speed);
	static_assert(sizeof(topic.nominal_integral_gain) == 4, "size mismatch");
	memcpy(&topic.nominal_integral_gain, buf.iterator, sizeof(topic.nominal_integral_gain));
	buf.iterator += sizeof(topic.nominal_integral_gain);
	buf.offset += sizeof(topic.nominal_integral_gain);
	static_assert(sizeof(topic.effective_integral_gain) == 4, "size mismatch");
	memcpy(&topic.effective_integral_gain, buf.iterator, sizeof(topic.effective_integral_gain));
	buf.iterator += sizeof(topic.effective_integral_gain);
	buf.offset += sizeof(topic.effective_integral_gain);
	static_assert(sizeof(topic.target_integral_gain) == 4, "size mismatch");
	memcpy(&topic.target_integral_gain, buf.iterator, sizeof(topic.target_integral_gain));
	buf.iterator += sizeof(topic.target_integral_gain);
	buf.offset += sizeof(topic.target_integral_gain);
	static_assert(sizeof(topic.integral_gain_slew_rate) == 4, "size mismatch");
	memcpy(&topic.integral_gain_slew_rate, buf.iterator, sizeof(topic.integral_gain_slew_rate));
	buf.iterator += sizeof(topic.integral_gain_slew_rate);
	buf.offset += sizeof(topic.integral_gain_slew_rate);
	static_assert(sizeof(topic.initial_hold_integral_recenter_rate_mps3) == 4, "size mismatch");
	memcpy(&topic.initial_hold_integral_recenter_rate_mps3, buf.iterator, sizeof(topic.initial_hold_integral_recenter_rate_mps3));
	buf.iterator += sizeof(topic.initial_hold_integral_recenter_rate_mps3);
	buf.offset += sizeof(topic.initial_hold_integral_recenter_rate_mps3);
	static_assert(sizeof(topic.initial_hold_integral_recenter_target_d) == 4, "size mismatch");
	memcpy(&topic.initial_hold_integral_recenter_target_d, buf.iterator, sizeof(topic.initial_hold_integral_recenter_target_d));
	buf.iterator += sizeof(topic.initial_hold_integral_recenter_target_d);
	buf.offset += sizeof(topic.initial_hold_integral_recenter_target_d);
	static_assert(sizeof(topic.position_setpoint_d) == 4, "size mismatch");
	memcpy(&topic.position_setpoint_d, buf.iterator, sizeof(topic.position_setpoint_d));
	buf.iterator += sizeof(topic.position_setpoint_d);
	buf.offset += sizeof(topic.position_setpoint_d);
	static_assert(sizeof(topic.position_d) == 4, "size mismatch");
	memcpy(&topic.position_d, buf.iterator, sizeof(topic.position_d));
	buf.iterator += sizeof(topic.position_d);
	buf.offset += sizeof(topic.position_d);
	static_assert(sizeof(topic.position_term_d) == 4, "size mismatch");
	memcpy(&topic.position_term_d, buf.iterator, sizeof(topic.position_term_d));
	buf.iterator += sizeof(topic.position_term_d);
	buf.offset += sizeof(topic.position_term_d);
	static_assert(sizeof(topic.velocity_feedforward_d) == 4, "size mismatch");
	memcpy(&topic.velocity_feedforward_d, buf.iterator, sizeof(topic.velocity_feedforward_d));
	buf.iterator += sizeof(topic.velocity_feedforward_d);
	buf.offset += sizeof(topic.velocity_feedforward_d);
	static_assert(sizeof(topic.velocity_setpoint_d) == 4, "size mismatch");
	memcpy(&topic.velocity_setpoint_d, buf.iterator, sizeof(topic.velocity_setpoint_d));
	buf.iterator += sizeof(topic.velocity_setpoint_d);
	buf.offset += sizeof(topic.velocity_setpoint_d);
	static_assert(sizeof(topic.velocity_d) == 4, "size mismatch");
	memcpy(&topic.velocity_d, buf.iterator, sizeof(topic.velocity_d));
	buf.iterator += sizeof(topic.velocity_d);
	buf.offset += sizeof(topic.velocity_d);
	static_assert(sizeof(topic.velocity_error_d) == 4, "size mismatch");
	memcpy(&topic.velocity_error_d, buf.iterator, sizeof(topic.velocity_error_d));
	buf.iterator += sizeof(topic.velocity_error_d);
	buf.offset += sizeof(topic.velocity_error_d);
	static_assert(sizeof(topic.acceleration_feedforward_d) == 4, "size mismatch");
	memcpy(&topic.acceleration_feedforward_d, buf.iterator, sizeof(topic.acceleration_feedforward_d));
	buf.iterator += sizeof(topic.acceleration_feedforward_d);
	buf.offset += sizeof(topic.acceleration_feedforward_d);
	static_assert(sizeof(topic.acceleration_p_d) == 4, "size mismatch");
	memcpy(&topic.acceleration_p_d, buf.iterator, sizeof(topic.acceleration_p_d));
	buf.iterator += sizeof(topic.acceleration_p_d);
	buf.offset += sizeof(topic.acceleration_p_d);
	static_assert(sizeof(topic.acceleration_i_d) == 4, "size mismatch");
	memcpy(&topic.acceleration_i_d, buf.iterator, sizeof(topic.acceleration_i_d));
	buf.iterator += sizeof(topic.acceleration_i_d);
	buf.offset += sizeof(topic.acceleration_i_d);
	static_assert(sizeof(topic.acceleration_d_d) == 4, "size mismatch");
	memcpy(&topic.acceleration_d_d, buf.iterator, sizeof(topic.acceleration_d_d));
	buf.iterator += sizeof(topic.acceleration_d_d);
	buf.offset += sizeof(topic.acceleration_d_d);
	static_assert(sizeof(topic.acceleration_setpoint_d) == 4, "size mismatch");
	memcpy(&topic.acceleration_setpoint_d, buf.iterator, sizeof(topic.acceleration_setpoint_d));
	buf.iterator += sizeof(topic.acceleration_setpoint_d);
	buf.offset += sizeof(topic.acceleration_setpoint_d);
	static_assert(sizeof(topic.thrust_setpoint_d) == 4, "size mismatch");
	memcpy(&topic.thrust_setpoint_d, buf.iterator, sizeof(topic.thrust_setpoint_d));
	buf.iterator += sizeof(topic.thrust_setpoint_d);
	buf.offset += sizeof(topic.thrust_setpoint_d);
	static_assert(sizeof(topic.hover_thrust) == 4, "size mismatch");
	memcpy(&topic.hover_thrust, buf.iterator, sizeof(topic.hover_thrust));
	buf.iterator += sizeof(topic.hover_thrust);
	buf.offset += sizeof(topic.hover_thrust);
	static_assert(sizeof(topic.native_hte) == 4, "size mismatch");
	memcpy(&topic.native_hte, buf.iterator, sizeof(topic.native_hte));
	buf.iterator += sizeof(topic.native_hte);
	buf.offset += sizeof(topic.native_hte);
	static_assert(sizeof(topic.terminal_estimated_distance_to_ground_m) == 4, "size mismatch");
	memcpy(&topic.terminal_estimated_distance_to_ground_m, buf.iterator, sizeof(topic.terminal_estimated_distance_to_ground_m));
	buf.iterator += sizeof(topic.terminal_estimated_distance_to_ground_m);
	buf.offset += sizeof(topic.terminal_estimated_distance_to_ground_m);
	static_assert(sizeof(topic.terminal_integral_floor_target_d) == 4, "size mismatch");
	memcpy(&topic.terminal_integral_floor_target_d, buf.iterator, sizeof(topic.terminal_integral_floor_target_d));
	buf.iterator += sizeof(topic.terminal_integral_floor_target_d);
	buf.offset += sizeof(topic.terminal_integral_floor_target_d);
	static_assert(sizeof(topic.terminal_integral_floor_slew_rate_mps3) == 4, "size mismatch");
	memcpy(&topic.terminal_integral_floor_slew_rate_mps3, buf.iterator, sizeof(topic.terminal_integral_floor_slew_rate_mps3));
	buf.iterator += sizeof(topic.terminal_integral_floor_slew_rate_mps3);
	buf.offset += sizeof(topic.terminal_integral_floor_slew_rate_mps3);
	return true;
}
