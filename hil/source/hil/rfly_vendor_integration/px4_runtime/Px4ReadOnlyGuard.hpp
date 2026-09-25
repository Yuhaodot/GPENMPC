#pragma once

#include "BoardSafetyEvidence.hpp"
#include "../px4_stream/LinkLifetimeRegistry.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/system_power.h>

class Mavlink;

namespace gpenmpc_rfly_px4 {

class Px4ReadOnlyGuard final {
public:
    Px4ReadOnlyGuard(Mavlink &actual_link, std::uint64_t frozen_telemetry_max_age_us,
        std::uint64_t frozen_commander_max_age_us = 0) noexcept;
    // Caller serializes observations. True means identity observations were
    // collected, NOT permission: all required GuardFacts must be Pass. Missing
    // direct-control samples do not by themselves reject identity-only capture.
    // Unknown physical/session facts remain visible and can never grant.
    bool observe(BoardSafetyEvidence &out) noexcept;
private:
    gpenmpc_rfly_stream::LinkToken link_token_{};
    const std::uint64_t max_age_us_;
    const std::uint64_t commander_max_age_us_;
    uORB::Subscription status_{ORB_ID(vehicle_status)};
    uORB::Subscription mode_{ORB_ID(vehicle_control_mode)};
    uORB::Subscription offboard_{ORB_ID(offboard_control_mode)};
    uORB::Subscription power_{ORB_ID(system_power)};
};

} // namespace gpenmpc_rfly_px4
