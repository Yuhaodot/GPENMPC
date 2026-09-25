#pragma once

// PX4 v1.16/RflySim packet boundary only. This does NOT select a controller,
// start a stream, publish uORB, or establish board authority. The actual
// sender must supply the already-validated source and its original expiry.
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <mavlink.h>
#include <cmath>
#include <cstdint>

namespace gpenmpc_rfly_packet {
enum class Error : std::uint8_t { None, NotSelected, NotHil, InvalidTime, Expired,
    Nonfinite, Range, UnexpectedUnusedControl };

struct Result {
    bool accepted{false};
    Error error{Error::None};
    // RflySim's official writer leaves noutputs at zero. Preserve it for
    // diagnosis; its wire contract is the fixed float[16] field, not noutputs.
    std::uint32_t raw_noutputs{0};
    mavlink_hil_actuator_controls_t packet{};
};

inline Result encode(const actuator_outputs_s &actual_output,
                     const vehicle_status_s &status,
                     const vehicle_control_mode_s &mode,
                     bool independently_selected,
                     std::uint64_t now_us,
                     std::uint64_t original_valid_until_us) noexcept
{
    Result r{};
    r.raw_noutputs=actual_output.noutputs;
    auto reject=[&r](Error e) { r.error=e; return r; };
    if(!independently_selected)return reject(Error::NotSelected);
    if(status.hil_state!=vehicle_status_s::HIL_STATE_ON)return reject(Error::NotHil);
    if(!actual_output.timestamp || !now_us || actual_output.timestamp>now_us ||
       original_valid_until_us<actual_output.timestamp)return reject(Error::InvalidTime);
    if(now_us>original_valid_until_us)return reject(Error::Expired);
    // This is the canonical six-rotor normalized-thrust subset of the official
    // [-1,1] 16-control transport; no clipping/reallocation is permitted here.
    for(unsigned i=0;i<16;++i){
        const float v=actual_output.output[i];
        if(!std::isfinite(v))return reject(Error::Nonfinite);
        if(v<0.0f || v>1.0f)return reject(Error::Range);
        // After the finite nonnegative range check, >0 is exactly the nonzero
        // test (including signed zero), without relaxing float-equal warnings.
        if(i>=6 && v>0.0f)return reject(Error::UnexpectedUnusedControl);
    }
    r.packet.time_usec=actual_output.timestamp;
    for(unsigned i=0;i<16;++i)r.packet.controls[i]=actual_output.output[i];
    r.packet.flags=123; // Official HIL16CtrlsNorm vendor branch, not native flags=0.
    r.packet.mode=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED|MAV_MODE_FLAG_HIL_ENABLED;
    if(mode.flag_control_auto_enabled)r.packet.mode|=MAV_MODE_FLAG_AUTO_ENABLED;
    if(mode.flag_control_manual_enabled)r.packet.mode|=MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if(mode.flag_control_attitude_enabled)r.packet.mode|=MAV_MODE_FLAG_STABILIZE_ENABLED;
    if(status.arming_state==vehicle_status_s::ARMING_STATE_ARMED)r.packet.mode|=MAV_MODE_FLAG_SAFETY_ARMED;
    if(status.nav_state==vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)r.packet.mode|=MAV_MODE_FLAG_GUIDED_ENABLED;
    r.accepted=true;
    return r;
}
} // namespace gpenmpc_rfly_packet
