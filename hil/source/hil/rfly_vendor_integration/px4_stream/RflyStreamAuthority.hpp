#pragma once

#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <cstdint>

namespace gpenmpc_rfly_stream {

enum class Source : std::uint8_t { Unavailable, NativeOutputsSim, RflyOutputs };

struct Observation {
    Source source{Source::Unavailable};
    actuator_outputs_s output{};
    vehicle_status_s status{};
    vehicle_control_mode_s control_mode{};
    unsigned output_uorb_generation{0};
    unsigned status_uorb_generation{0};
    unsigned mode_uorb_generation{0};
};

struct OriginalValidity {
    std::uint64_t valid_until_us{0};
};

// The integrator supplies source identity, selection, expiry and physical
// isolation validation. The authority outlives the stream; calls run on
// the MAVLink stream task.
class Authority {
public:
    virtual ~Authority() = default;
    virtual Source choose(std::uint64_t now_us, const vehicle_status_s &status,
                          const vehicle_control_mode_s &control_mode) noexcept = 0;

    // Atomically revalidate selection, session/producer and exact payload/time/
    // uORB generation; validate HIL + physical-output isolation and original
    // state evidence freshness; enforce transition/replay/one-use policy; then
    // consume the independent transmission receipt. The output expiry must be
    // the ORIGINAL bound expiry, not now + an invented age. A false result must
    // never prompt a native fallback. A claimed true result is not an ACK from
    // MAVLink, CopterSim or hardware. Failed sends are not automatically retried.
    virtual bool accept(const Observation &observation, std::uint64_t now_us,
                        OriginalValidity &original) noexcept = 0;
};

} // namespace gpenmpc_rfly_stream
