#pragma once

#include "../../px4_full_inner/consumption/ConsumptionBinding.hpp"

namespace gpenmpc_rfly_px4 {

enum class GuardFact : std::uint8_t { Unknown, Pass, Fail };
enum class GuardReason : std::uint8_t {
    None, Configuration, MissingTelemetry, StaleTelemetry, UidUnavailable,
    IdentityMismatch, NotUsb, HilConfiguration, ControlShape, ParameterMissing,
    ParameterType, ParameterRead, PhysicalFunctionEnabled, PwmOutRunning,
    BootSessionUnknown, IoDriverUnknown, ExternalPhysicalFactsUnknown, IoDriverRunning, DshotRunning,
    NativeRecoveryConfiguration
};

struct BoardSafetyEvidence {
    gpenmpc_consumption::Identity identity{};
    std::uint64_t observation_us{0}, original_valid_until_us{0};
    // Identity/safety observations do not renew or require an old OFFBOARD
    // producer after a real mode transition. Direct mode still uses its own
    // original_valid_until_us and telemetry_fresh, including OFFBOARD age.
    std::uint64_t identity_valid_until_us{0};
    std::uint64_t status_timestamp_us{0}, mode_timestamp_us{0}, offboard_timestamp_us{0}, power_timestamp_us{0};
    GuardFact uid{GuardFact::Unknown}, mavlink_identity{GuardFact::Unknown};
    GuardFact boot_session{GuardFact::Unknown};
    GuardFact usb_transport{GuardFact::Unknown}, hil_configuration{GuardFact::Unknown};
    GuardFact telemetry_fresh{GuardFact::Unknown}, active_direct_mode{GuardFact::Unknown};
    GuardFact native_controllers_disabled{GuardFact::Unknown};
    GuardFact native_land_mode{GuardFact::Unknown}, disarmed_control{GuardFact::Unknown};
    // Observed controller/failsafe configuration, not physical isolation or
    // heartbeat liveness. It gates new canonical acquisition/prestream only;
    // native recovery remains governed by its actual observed mode/outputs.
    GuardFact native_recovery_configuration{GuardFact::Unknown};
    std::int32_t raw_ra_ctrl_mode{0},raw_com_obl_rc_act{0};
    GuardFact pwm_functions_zero{GuardFact::Unknown}, pwm_out_stopped{GuardFact::Unknown};
    GuardFact io_driver_stopped{GuardFact::Unknown}, dshot_stopped{GuardFact::Unknown};
    GuardFact usb_power_observed{GuardFact::Unknown};
    GuardFact external_physical_isolation{GuardFact::Unknown};
    GuardReason reason{GuardReason::None};
    std::uint8_t checked_pwm_functions{0};
    std::uint8_t raw_usb_connected{0}, raw_usb_valid{0}, raw_brick_valid{0}, raw_servo_valid{0};
    std::int32_t first_nonzero_function{0};
    char first_bad_parameter[20]{};

    // Identity observation is separate from arming and direct-mode checks.
    // Bind boot_generation to the board-created execution session.
    bool require_identity_only() const noexcept
    {
        return uid == GuardFact::Pass && mavlink_identity == GuardFact::Pass;
    }
    bool require_active_direct() const noexcept
    {
        return require_identity_only() && usb_transport == GuardFact::Pass &&
            hil_configuration == GuardFact::Pass && telemetry_fresh == GuardFact::Pass &&
            active_direct_mode == GuardFact::Pass && native_controllers_disabled == GuardFact::Pass;
    }
    bool require_physical_facts() const noexcept
    {
        return pwm_functions_zero == GuardFact::Pass && pwm_out_stopped == GuardFact::Pass &&
            io_driver_stopped == GuardFact::Pass && dshot_stopped == GuardFact::Pass &&
            usb_power_observed == GuardFact::Pass &&
            external_physical_isolation == GuardFact::Pass;
    }
};

inline std::uint64_t guard_original_expiry(std::uint64_t stamp, std::uint64_t max_age) noexcept
{
    return stamp && max_age && max_age <= UINT64_MAX - stamp ? stamp + max_age : 0;
}

} // namespace gpenmpc_rfly_px4
