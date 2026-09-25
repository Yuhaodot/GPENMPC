#pragma once
// Private store -> HOST export. No HOST-provided state/identity/timestamps.
// Layout retains raw13 float32, original board times, origin and source deltas.
#include "../SlimArgumentTransport.hpp"
#include "RflySnapshotWireTypes.hpp"
namespace gpenmpc_snapshot_wire {
template<class PrivateOwner>bool encode(const PrivateOwner&owner,
    const gpenmpc_rfly_state_execution::SnapshotTicket&ticket,
    const gpenmpc_rfly_execution::Configuration&approved,Bytes&out)noexcept{
    out={};const auto*snapshot=owner.snapshot(ticket);
    if(!snapshot||!snapshot->valid()||!(snapshot->estimator().identity==approved.identity)||
       approved.configuration_payload_sha256!=gpenmpc_rfly_execution::kCanonicalConfigurationSha)return false;
    const auto&s=snapshot->estimator();const auto&r=snapshot->raw();Writer w(out.data());
    w.byte('R');w.byte('S');w.byte('P');w.byte('1');w.bytes(ticket);
    w.u64(s.identity.uid);w.u64(s.identity.boot_generation);w.byte(s.identity.system);w.byte(s.identity.component);
    w.u32(vehicle_odometry_s::MESSAGE_VERSION);w.u32(snapshot->subscription_generation());
    w.u64(r.timestamp_sample);w.u64(r.timestamp);w.u64(s.board_rx_us);
    for(double v:snapshot->task_origin_ned_m())w.real(v);
    for(float v:r.position)w.f32(v);
    for(float v:r.velocity)w.f32(v);
    for(float v:r.q)w.f32(v);
    for(float v:r.angular_velocity)w.f32(v);
    w.byte(r.pose_frame);w.byte(r.velocity_frame);w.byte(r.reset_counter);w.byte(std::uint8_t(r.quality));
    w.u64(snapshot->generation_delta());w.u64(snapshot->actual_sample_delta_us());
    w.bytes(gpenmpc_argument_transport::bytes_of(approved.configuration_payload_sha256));
    const auto sha=gpenmpc_argument_transport::digest(out.data(),message_bytes-32);w.bytes(sha);return true;
}
} // namespace gpenmpc_snapshot_wire
