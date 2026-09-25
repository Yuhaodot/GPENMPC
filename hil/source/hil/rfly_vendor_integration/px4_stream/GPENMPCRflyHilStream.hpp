#ifndef GPENMPC_RFLY_HIL_STREAM_HPP
#define GPENMPC_RFLY_HIL_STREAM_HPP

// PX4 v1.16 HIL_ACTUATOR_CONTROLS stream implementation.
// Uses the sole registry entry and does not issue arm, mode or vehicle commands.
#include "mavlink_main.h"
#include "mavlink_stream.h"
#include <uORB/Subscription.hpp>
#include "RflyHilPacket.hpp"
#include "RflyStreamAuthority.hpp"
#include "SharedOutputRegistry.hpp"

class MavlinkStreamGPENMPCRflyHILActuatorControls final : public MavlinkStream
{
public:
    enum class Rejection : std::uint8_t {
        None, AuthorityUnavailable, SelectionUnavailable, NoNewOutput,
        MissingStatus, MissingControlMode, AuthorityRejected, PacketRejected,
        NativeInvalidTime, NativeNotHil, NativeInvalidControl
    };

    // Real PX4 factory routes to the one module owner bound to this exact
    // Mavlink instance. No binding -> Unavailable, never a native fallback.
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamGPENMPCRflyHILActuatorControls(mavlink);
    }

    static constexpr const char *get_name_static() { return "HIL_ACTUATOR_CONTROLS"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS; }
    const char *get_name() const override { return get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }
    Rejection last_rejection() const { return _last_rejection; }
    gpenmpc_rfly_packet::Error last_packet_error() const { return _packet_error; }
    std::uint32_t last_raw_noutputs() const { return _raw_noutputs; }

    unsigned get_size() override
    {
        // Do not poll the unselected data source just to estimate bandwidth.
        return MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    explicit MavlinkStreamGPENMPCRflyHILActuatorControls(Mavlink *mavlink) :
        MavlinkStream(mavlink),
        _router(gpenmpc_rfly_stream::shared_output_registry(),mavlink) {}

    gpenmpc_rfly_stream::Router _router;
    uORB::Subscription _rfly_sub{ORB_ID(actuator_outputs_rfly)};
    uORB::Subscription _native_sub{ORB_ID(actuator_outputs_sim)};
    uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _mode_sub{ORB_ID(vehicle_control_mode)};
    Rejection _last_rejection{Rejection::AuthorityUnavailable};
    gpenmpc_rfly_packet::Error _packet_error{gpenmpc_rfly_packet::Error::None};
    std::uint32_t _raw_noutputs{0};

    bool reject(Rejection reason)
    {
        _last_rejection = reason;
        return false;
    }

    bool send() override
    {
        using gpenmpc_rfly_stream::Source;
        _packet_error = gpenmpc_rfly_packet::Error::None;
        gpenmpc_rfly_stream::Observation observation{};
        if (!_status_sub.copy(&observation.status)) { return reject(Rejection::MissingStatus); }
        observation.status_uorb_generation = _status_sub.get_last_generation();
        if (!_mode_sub.copy(&observation.control_mode)) { return reject(Rejection::MissingControlMode); }
        observation.mode_uorb_generation = _mode_sub.get_last_generation();
        observation.source = _router.choose(hrt_absolute_time(), observation.status, observation.control_mode);
        uORB::Subscription *selected = nullptr;
        if (observation.source == Source::RflyOutputs) { selected = &_rfly_sub; }
        else if (observation.source == Source::NativeOutputsSim) { selected = &_native_sub; }
        else { return reject(Rejection::SelectionUnavailable); }

        // Read the selected output when an update is available.
        if (!selected->update(&observation.output)) { return reject(Rejection::NoNewOutput); }
        observation.output_uorb_generation = selected->get_last_generation();
        _raw_noutputs = observation.output.noutputs;

        gpenmpc_rfly_stream::OriginalValidity original{};
        if (!_router.accept(observation, hrt_absolute_time(), original)) {
            return reject(Rejection::AuthorityRejected);
        }
        const std::uint64_t now = hrt_absolute_time();
        mavlink_hil_actuator_controls_t packet{};
        if (observation.source == Source::RflyOutputs) {
            const auto encoded = gpenmpc_rfly_packet::encode(observation.output,
                observation.status, observation.control_mode, true, now, original.valid_until_us);
            if (!encoded.accepted) {
                _packet_error = encoded.error;
                return reject(Rejection::PacketRejected);
            }
            packet = encoded.packet;
        } else {
            // PX4 native source retains its own [-1,1] field contract and flags=0;
            // it must never pass through the canonical six-rotor Rfly encoder.
            if (!observation.output.timestamp || !now || observation.output.timestamp > now ||
                original.valid_until_us < observation.output.timestamp || now > original.valid_until_us) {
                return reject(Rejection::NativeInvalidTime);
            }
            if (observation.status.hil_state != vehicle_status_s::HIL_STATE_ON) {
                return reject(Rejection::NativeNotHil);
            }
            for (unsigned i = 0; i < 16; ++i) {
                const float value = observation.output.output[i];
                if (!std::isfinite(value) || value < -1.0f || value > 1.0f) {
                    return reject(Rejection::NativeInvalidControl);
                }
                packet.controls[i] = value;
            }
            packet.time_usec = observation.output.timestamp;
            packet.flags = 0;
            packet.mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_HIL_ENABLED;
            if (observation.control_mode.flag_control_auto_enabled) { packet.mode |= MAV_MODE_FLAG_AUTO_ENABLED; }
            if (observation.control_mode.flag_control_manual_enabled) { packet.mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED; }
            if (observation.control_mode.flag_control_attitude_enabled) { packet.mode |= MAV_MODE_FLAG_STABILIZE_ENABLED; }
            if (observation.status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
                packet.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
            }
            if (observation.status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
                packet.mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
            }
        }
        // This is the real PX4/MAVLink send interface. Compilation does not run
        // it; its void return is not delivery/consumer acknowledgement.
        mavlink_msg_hil_actuator_controls_send_struct(_mavlink->get_channel(), &packet);
        _last_rejection = Rejection::None;
        return true;
    }
};

#endif // GPENMPC_RFLY_HIL_STREAM_HPP
