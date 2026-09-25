#include "Px4ReadOnlyGuard.hpp"
#include "NativeLandModeShape.hpp"
#include "mavlink_main.h"
#include <board_config.h>
#include <px4_platform_common/board_common.h>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <cstring>

extern "C" bool gpenmpc_readonly_pwm_out_running() noexcept;
extern "C" bool gpenmpc_readonly_px4io_running() noexcept;
extern "C" bool gpenmpc_readonly_dshot_running() noexcept;

namespace gpenmpc_rfly_px4 {
namespace {
void reason(BoardSafetyEvidence &e, GuardReason r) noexcept
{
    if (e.reason == GuardReason::None) { e.reason = r; }
}

bool read_int32(const char *name, std::int32_t &value, BoardSafetyEvidence &e) noexcept
{
    // The no-notification lookup does not mark the parameter used or mutate it.
    const param_t p = param_find_no_notification(name);
    GuardReason failure = GuardReason::None;
    if (p == PARAM_INVALID) { failure = GuardReason::ParameterMissing; }
    else if (param_type(p) != PARAM_TYPE_INT32) { failure = GuardReason::ParameterType; }
    else if (param_get(p, &value) != 0) { failure = GuardReason::ParameterRead; }
    if (failure == GuardReason::None) { return true; }
    reason(e, failure);
    if (!e.first_bad_parameter[0]) {
        std::strncpy(e.first_bad_parameter, name, sizeof(e.first_bad_parameter) - 1);
    }
    return false;
}

bool fresh(std::uint64_t stamp, std::uint64_t now, std::uint64_t age) noexcept
{
    const auto end = guard_original_expiry(stamp, age);
    return end && stamp <= now && now <= end;
}
std::uint64_t smaller(std::uint64_t a, std::uint64_t b) noexcept { return a < b ? a : b; }
bool read_usb_link(const void *pointer,void *result) noexcept
{
    // Mavlink's getter is non-const but reads only _is_usb_uart, which is fixed
    // before lifetime activation. Pointer exists only during the PI-lock borrow.
    *static_cast<bool *>(result)=static_cast<Mavlink *>(const_cast<void *>(pointer))->is_usb_uart();
    return true;
}
} // namespace

Px4ReadOnlyGuard::Px4ReadOnlyGuard(Mavlink &actual_link, std::uint64_t frozen_telemetry_max_age_us,
    std::uint64_t frozen_commander_max_age_us) noexcept :
    max_age_us_(frozen_telemetry_max_age_us),
    commander_max_age_us_(frozen_commander_max_age_us ? frozen_commander_max_age_us : frozen_telemetry_max_age_us)
{
    // Never auto-register caller input: only the actual constructor/task hooks
    // can establish a live link. Missing activation leaves an unusable token.
    auto *registry=gpenmpc_rfly_stream::link_lifetime_registry();
    if(registry)(void)registry->lookup(&actual_link,link_token_);
}

bool Px4ReadOnlyGuard::observe(BoardSafetyEvidence &e) noexcept
{
    e = {};
    if (!max_age_us_) { e.reason = GuardReason::Configuration; return false; }
    bool usb=false;
    auto *registry=gpenmpc_rfly_stream::link_lifetime_registry();
    if(!registry || registry->read(link_token_,read_usb_link,&usb)!=gpenmpc_rfly_stream::LinkAccess::Read){
        e.reason=GuardReason::Configuration;return false;
    }
#if defined(BOARD_HAS_NO_UUID) || !defined(CONFIG_ARCH_BOARD_PX4_FMU_V6C)
    e.reason = GuardReason::UidUnavailable;
    return false;
#else
    // Exactly the AUTOPILOT_VERSION uid algorithm in mavlink_main.cpp:1120.
    uuid_uint32_t words{};
    board_get_uuid32(words);
    e.identity.uid = (std::uint64_t(words[PX4_CPU_UUID_WORD32_UNIQUE_M]) << 32) |
                      words[PX4_CPU_UUID_WORD32_UNIQUE_H];
    e.uid = e.identity.uid ? GuardFact::Pass : GuardFact::Fail;
    if (e.uid != GuardFact::Pass) { reason(e, GuardReason::UidUnavailable); }
#endif
    e.identity.system = mavlink_system.sysid;
    e.identity.component = mavlink_system.compid;
    // No host-expected boot value is an observation. A board-owned execution
    // session is established by the local owner separately, not by this read.
    e.identity.boot_generation = 0;
    e.boot_session = GuardFact::Unknown;

    std::int32_t sys = 0, comp = 0, hitl = 0;
    const bool identity_params = read_int32("MAV_SYS_ID", sys, e) && read_int32("MAV_COMP_ID", comp, e);
    if (identity_params) {
        e.mavlink_identity = sys > 0 && sys <= 255 && comp > 0 && comp <= 255 &&
            sys == e.identity.system && comp == e.identity.component ? GuardFact::Pass : GuardFact::Fail;
        if (e.mavlink_identity != GuardFact::Pass) { reason(e, GuardReason::IdentityMismatch); }
    }
    e.usb_transport = usb ? GuardFact::Pass : GuardFact::Fail;
    if (e.usb_transport != GuardFact::Pass) { reason(e, GuardReason::NotUsb); }
    if (read_int32("SYS_HITL", hitl, e)) {
        e.hil_configuration = hitl == 1 ? GuardFact::Pass : GuardFact::Fail;
        if (e.hil_configuration != GuardFact::Pass) { reason(e, GuardReason::HilConfiguration); }
    }

    // Source: gpenmpc_se3_control_params.c RA_CTRL_MODE=0 retains the native
    // position chain; commander_params.c COM_OBL_RC_ACT value4 is LAND. Read
    // actual typed parameters each observation. No defaults are observations,
    // no PARAM_SET occurs, and COM_OF_LOSS_T remains Commander's own delay.
    const bool have_ra_mode=read_int32("RA_CTRL_MODE",e.raw_ra_ctrl_mode,e);
    const bool have_offboard_action=read_int32("COM_OBL_RC_ACT",e.raw_com_obl_rc_act,e);
    if(have_ra_mode&&have_offboard_action){
        e.native_recovery_configuration=e.raw_ra_ctrl_mode==0&&e.raw_com_obl_rc_act==4?GuardFact::Pass:GuardFact::Fail;
        if(e.native_recovery_configuration!=GuardFact::Pass)reason(e,GuardReason::NativeRecoveryConfiguration);
    }

    // This FMUv6C build's real generated parameter metadata has exactly these
    // 16 Int32 functions: physical IO MAIN1..8 and physical FMU AUX1..8.
    static const char *const functions[] = {
        "PWM_MAIN_FUNC1", "PWM_MAIN_FUNC2", "PWM_MAIN_FUNC3", "PWM_MAIN_FUNC4",
        "PWM_MAIN_FUNC5", "PWM_MAIN_FUNC6", "PWM_MAIN_FUNC7", "PWM_MAIN_FUNC8",
        "PWM_AUX_FUNC1", "PWM_AUX_FUNC2", "PWM_AUX_FUNC3", "PWM_AUX_FUNC4",
        "PWM_AUX_FUNC5", "PWM_AUX_FUNC6", "PWM_AUX_FUNC7", "PWM_AUX_FUNC8"
    };
    e.pwm_functions_zero = GuardFact::Pass;
    for (const char *name : functions) {
        std::int32_t value = 0;
        if (!read_int32(name, value, e)) {
            if (e.pwm_functions_zero != GuardFact::Fail) { e.pwm_functions_zero = GuardFact::Unknown; }
            continue;
        }
        ++e.checked_pwm_functions;
        if (value != 0) {
            e.pwm_functions_zero = GuardFact::Fail;
            if (!e.first_nonzero_function) { e.first_nonzero_function = value; }
            if (!e.first_bad_parameter[0]) { std::strncpy(e.first_bad_parameter, name, sizeof(e.first_bad_parameter) - 1); }
            reason(e, GuardReason::PhysicalFunctionEnabled);
        }
    }
    e.pwm_out_stopped = gpenmpc_readonly_pwm_out_running() ? GuardFact::Fail : GuardFact::Pass;
    if (e.pwm_out_stopped != GuardFact::Pass) { reason(e, GuardReason::PwmOutRunning); }
    // Read-only functions in the driver translation units query the drivers'
    // ModuleBase task slots directly.
    e.io_driver_stopped = gpenmpc_readonly_px4io_running() ? GuardFact::Fail : GuardFact::Pass;
    e.dshot_stopped = gpenmpc_readonly_dshot_running() ? GuardFact::Fail : GuardFact::Pass;
    if (e.io_driver_stopped != GuardFact::Pass) { reason(e, GuardReason::IoDriverRunning); }
    if (e.dshot_stopped != GuardFact::Pass) { reason(e, GuardReason::DshotRunning); }
    e.external_physical_isolation = GuardFact::Unknown;

    vehicle_status_s status{};
    vehicle_control_mode_s mode{};
    offboard_control_mode_s offboard{};
    system_power_s power{};
    const bool have_status = status_.copy(&status);
    const bool have_mode = mode_.copy(&mode);
    const bool have_offboard = offboard_.copy(&offboard);
    const bool have_power = power_.copy(&power);
    const std::uint64_t now = hrt_absolute_time();
    e.observation_us = now;
    e.status_timestamp_us = status.timestamp; e.mode_timestamp_us = mode.timestamp;
    e.offboard_timestamp_us = offboard.timestamp; e.power_timestamp_us = power.timestamp;
    if (have_status && (status.system_id != e.identity.system || status.component_id != e.identity.component)) {
        e.mavlink_identity = GuardFact::Fail;
        reason(e, GuardReason::IdentityMismatch);
    }
    if (have_power) {
        e.raw_usb_connected = power.usb_connected; e.raw_usb_valid = power.usb_valid;
        e.raw_brick_valid = power.brick_valid; e.raw_servo_valid = power.servo_valid;
        if (fresh(power.timestamp, now, max_age_us_)) {
            e.usb_power_observed = power.usb_connected == 1 && power.usb_valid == 1 && power.brick_valid == 0 ?
                GuardFact::Pass : GuardFact::Fail;
        }
        // FMUv6C BOARD_ADC_SERVO_VALID is hardcoded 1. It is NOT a measurement
        // that proves an unpowered servo rail; raw_servo_valid stays diagnostic.
    }
    // Identity-only observations may be made while disarmed, before any host
    // offboard producer exists. A real status timestamp bounds that observation;
    // missing mode/offboard still cannot satisfy the active-direct facts.
    // Commander publishes status/control_mode at 2 Hz or on change. Only an
    // explicitly selected local owner uses its separate source-age contract;
    // legacy callers retain their original common age. Offboard/power do not.
    e.original_valid_until_us = have_status ? guard_original_expiry(status.timestamp, commander_max_age_us_) : 0;
    e.identity_valid_until_us=e.original_valid_until_us;
    if(have_mode)e.identity_valid_until_us=smaller(e.identity_valid_until_us,guard_original_expiry(mode.timestamp,commander_max_age_us_));
    if(have_power)e.identity_valid_until_us=smaller(e.identity_valid_until_us,guard_original_expiry(power.timestamp,max_age_us_));
    if(have_status && have_mode && fresh(status.timestamp,now,commander_max_age_us_) && fresh(mode.timestamp,now,commander_max_age_us_)){
        e.native_land_mode=native_land_mode_shape(status,mode)?GuardFact::Pass:GuardFact::Fail;
        e.disarmed_control=disarmed_control_shape(status,mode)?GuardFact::Pass:GuardFact::Fail;
    }
    if (!have_status || !have_mode || !have_offboard) {
        reason(e, GuardReason::MissingTelemetry);
        return e.require_identity_only() && have_status && fresh(status.timestamp, now, commander_max_age_us_);
    }
    const bool telemetry_fresh = fresh(status.timestamp, now, commander_max_age_us_) &&
        fresh(mode.timestamp, now, commander_max_age_us_) && fresh(offboard.timestamp, now, max_age_us_);
    e.telemetry_fresh = telemetry_fresh ? GuardFact::Pass : GuardFact::Fail;
    if (!telemetry_fresh) { reason(e, GuardReason::StaleTelemetry); }
    e.original_valid_until_us = smaller(guard_original_expiry(status.timestamp, commander_max_age_us_),
        smaller(guard_original_expiry(mode.timestamp, commander_max_age_us_), guard_original_expiry(offboard.timestamp, max_age_us_)));
    if (have_power) { e.original_valid_until_us = smaller(e.original_valid_until_us, guard_original_expiry(power.timestamp, max_age_us_)); }
    e.active_direct_mode = status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
        !status.is_vtol && !status.in_transition_mode && !status.in_transition_to_fw &&
        status.hil_state == vehicle_status_s::HIL_STATE_ON &&
        status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
        status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD &&
        mode.flag_armed && mode.flag_control_offboard_enabled &&
        offboard.direct_actuator && !offboard.position && !offboard.velocity && !offboard.acceleration &&
        !offboard.attitude && !offboard.body_rate && !offboard.thrust_and_torque ? GuardFact::Pass : GuardFact::Fail;
    e.native_controllers_disabled = !mode.flag_multicopter_position_control_enabled &&
        !mode.flag_control_manual_enabled && !mode.flag_control_auto_enabled && !mode.flag_control_position_enabled &&
        !mode.flag_control_velocity_enabled && !mode.flag_control_altitude_enabled && !mode.flag_control_climb_rate_enabled &&
        !mode.flag_control_acceleration_enabled && !mode.flag_control_attitude_enabled && !mode.flag_control_rates_enabled &&
        !mode.flag_control_allocation_enabled && !mode.flag_control_termination_enabled ? GuardFact::Pass : GuardFact::Fail;
    if (e.active_direct_mode != GuardFact::Pass || e.native_controllers_disabled != GuardFact::Pass) { reason(e, GuardReason::ControlShape); }
    if (e.reason == GuardReason::None) { e.reason = GuardReason::BootSessionUnknown; }
    return e.require_identity_only() && fresh(status.timestamp, now, commander_max_age_us_);
}

} // namespace gpenmpc_rfly_px4
