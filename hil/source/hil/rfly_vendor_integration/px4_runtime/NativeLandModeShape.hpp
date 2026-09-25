#pragma once
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>

namespace gpenmpc_rfly_px4 {
// Exact ordinary multirotor AUTO_LAND shape in this target's Commander.
// This is not an ACK, original freshness check, landing/contact evidence or
// authority by itself; the actual reader and selected stream both check it.
inline bool native_land_mode_shape(const vehicle_status_s &s,const vehicle_control_mode_s &m) noexcept
{
    return s.vehicle_type==vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !s.is_vtol &&
        !s.in_transition_mode && !s.in_transition_to_fw &&
        s.hil_state==vehicle_status_s::HIL_STATE_ON &&
        s.arming_state==vehicle_status_s::ARMING_STATE_ARMED && m.flag_armed &&
        s.nav_state==vehicle_status_s::NAVIGATION_STATE_AUTO_LAND && m.flag_control_auto_enabled &&
        m.flag_multicopter_position_control_enabled && m.flag_control_position_enabled &&
        m.flag_control_velocity_enabled && m.flag_control_altitude_enabled && m.flag_control_climb_rate_enabled &&
        m.flag_control_attitude_enabled && m.flag_control_rates_enabled && m.flag_control_allocation_enabled &&
        !m.flag_control_offboard_enabled && !m.flag_control_manual_enabled && !m.flag_control_termination_enabled;
}
inline bool disarmed_control_shape(const vehicle_status_s &s,const vehicle_control_mode_s &m) noexcept
{
    return s.arming_state==vehicle_status_s::ARMING_STATE_DISARMED && !m.flag_armed &&
        !m.flag_control_termination_enabled;
}
} // namespace gpenmpc_rfly_px4
