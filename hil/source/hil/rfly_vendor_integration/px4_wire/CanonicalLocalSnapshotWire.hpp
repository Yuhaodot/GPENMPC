#pragma once
// RLS1 is a historical observation for the complete local inner, not the
// legacy RSP1/RAS1 execution ticket. It exports the actual private Snapshot and
// exact original HIL endpoint. It grants no EKF lineage, DLL association,
// freshness, command acceptance or actuator authority.
#include "CanonicalLocalGpWire.hpp"
#include "../CanonicalLocalInnerInputBuilder.hpp"
#include "../local_source_ingress/Px4OriginalHilReceiptReader.hpp"

namespace gpenmpc_local_snapshot_wire {
using Bytes=gpenmpc_portable::Array<std::uint8_t,382>;
using Fragment=gpenmpc_snapshot_wire::Fragment;
constexpr std::size_t message_bytes=382,fragment_count=4;
constexpr std::uint8_t schema=10;
struct Observation {
    gpenmpc_local_input::SnapshotKey source{};
    std::uint32_t odometry_instance{};
    double task_origin_ned_m[3]{},canonical_state13[13]{};
    gpenmpc_source_receipt::Record endpoint{};
    std::int32_t gyro_instance{-1},accel_instance{-1};
    std::uint32_t gyro_device_id{},accel_device_id{},original_subscription_generation{};
    bool gyro_update_called{},accel_update_called{};
};
inline std::int32_t signed32(std::uint32_t bits)noexcept{
    std::int32_t result;std::memcpy(&result,&bits,4);return result;
}
// The original 65 payload bytes are exactly reconstructible: time8 + the
// retained sensor52 + fields4 + sensor ID1 (MAVLink little-endian fields).
inline std::uint8_t payload_byte(const gpenmpc_source_receipt::Record&e,unsigned i)noexcept{
    if(i<8)return static_cast<std::uint8_t>(e.original_wire_time_us>>(8*i));
    if(i<60)return e.original_sensor52[i-8];
    if(i<64)return static_cast<std::uint8_t>(e.fields_updated>>(8*(i-60)));
    return e.sensor_id;
}
inline bool valid(const Observation&o)noexcept{
    const auto&s=o.source;const auto&e=o.endpoint;
    if(!gpenmpc_local_gp_wire::identity_valid(s.identity)||!s.sample_us||!s.source_generation||
       s.publication_us<s.sample_us||s.original_receipt_us<s.publication_us||
       gpenmpc_local_input::empty(s.state_and_origin_sha256)||o.odometry_instance>=4||
       false)return false;
    // Runtime-only observations encode an absent HIL endpoint as all zeroes.
    const bool runtime=!e.original_tap_event_ordinal&&!e.original_receiver_hrt_us&&
        !e.payload_length&&!o.original_subscription_generation&&!o.gyro_update_called;
    if(!runtime&&(e.original_receiver_hrt_us!=s.sample_us||!e.original_tap_event_ordinal||
       e.receiver_instance<0||e.channel<0||!e.payload_length||e.payload_length>65||
       (e.fields_updated&0x38u)!=0x38u||!e.all_gyro_fields_updated||!o.gyro_update_called||
       !o.original_subscription_generation||o.gyro_instance< -1||o.gyro_instance>=4||
       o.accel_instance< -1||o.accel_instance>=4))return false;
    for(auto v:o.task_origin_ned_m){if(!std::isfinite(v))return false;}
    for(auto v:o.canonical_state13){if(!std::isfinite(v))return false;}
    return true;
}
// This factory does not accept HOST x13 or a replacement observation time.
// The original endpoint Reader has already compared actual link ownership;
// this extra comparison prevents an unrelated retained record being encoded.
inline bool from_actual(const gpenmpc_odometry::Snapshot&s,
    const gpenmpc_hil_endpoint_reader::Receipt&r,Observation&out)noexcept{
    out={};const auto&e=r.endpoint;const auto&t=r.original.topic;
    if(!s.valid()||s.source_topic()!=ORB_ID(vehicle_odometry)||
       r.original_source_topic!=s.source_topic()||r.original_source_instance!=s.source_instance()||
       !e.exact_unique_endpoint||e.snapshot_sample_us!=s.estimator().timestamp_sample_us||
       e.snapshot_generation!=s.estimator().generation||
       e.original.original_receiver_hrt_us!=t.timestamp||e.original.original_wire_time_us!=t.wire_time_usec||
       e.original.original_tap_event_ordinal!=t.original_event_sequence||
       e.original.fields_updated!=t.fields_updated||e.original.receiver_instance!=t.receiver_instance||
       e.original.channel!=t.channel||e.original.system!=t.system_id||e.original.component!=t.component_id||
       e.original.sequence!=t.mavlink_sequence||e.original.payload_length!=t.payload_length||
       e.original.sensor_id!=t.sensor_id||std::memcmp(e.original.original_sensor52,t.original_payload+8,52)||
       !t.gyro_update_called||t.prior_publication_failures)return false;
    for(unsigned i=0;i<65;++i)if(payload_byte(e.original,i)!=t.original_payload[i]||
        (i>=t.payload_length&&t.original_payload[i]))return false;
    if(!gpenmpc_local_input::snapshot_key(s,out.source)||
       !gpenmpc_local_input::same_source(out.source,r.original_snapshot_key))return false;
    gpenmpc_portable::Array<double,13> state{};
    if(!gpenmpc_snapshot_mapping::state13(s,state))return false;
    out.odometry_instance=s.source_instance();
    std::memcpy(out.canonical_state13,state.data(),sizeof out.canonical_state13);
    std::memcpy(out.task_origin_ned_m,s.task_origin_ned_m().data(),sizeof out.task_origin_ned_m);
    out.endpoint=e.original;out.gyro_instance=t.gyro_topic_instance;out.accel_instance=t.accel_topic_instance;
    out.gyro_device_id=t.gyro_device_id;out.accel_device_id=t.accel_device_id;
    out.original_subscription_generation=r.original.original_subscription_generation;
    out.gyro_update_called=t.gyro_update_called;out.accel_update_called=t.accel_update_called;
    return valid(out);
}
inline bool encode(const Observation&o,Bytes&out)noexcept{
    out={};if(!valid(o))return false;
    gpenmpc_snapshot_wire::Writer w(out.data());const auto&s=o.source;const auto&e=o.endpoint;
    w.byte('R');w.byte('L');w.byte('S');w.byte('1');gpenmpc_local_gp_wire::write_identity(w,s.identity);
    w.u64(s.sample_us);w.u64(s.publication_us);w.u64(s.original_receipt_us);w.u64(s.source_generation);
    w.u64(s.generation_delta);w.u64(s.sample_delta_us);w.byte(s.reset_counter);
    gpenmpc_local_gp_wire::write_hash(w,s.state_and_origin_sha256);w.u32(o.odometry_instance);
    for(auto v:o.task_origin_ned_m){w.real(v);}for(auto v:o.canonical_state13){w.real(v);}
    w.u64(e.original_receiver_hrt_us);w.u64(e.original_wire_time_us);w.u64(e.original_tap_event_ordinal);
    w.u32(e.fields_updated);w.u32(static_cast<std::uint32_t>(e.receiver_instance));w.u32(static_cast<std::uint32_t>(e.channel));
    w.byte(e.system);w.byte(e.component);w.byte(e.sequence);w.byte(e.payload_length);w.byte(e.sensor_id);
    for(auto v:e.original_sensor52){w.byte(v);}
    w.u32(static_cast<std::uint32_t>(o.gyro_instance));w.u32(static_cast<std::uint32_t>(o.accel_instance));
    w.u32(o.gyro_device_id);w.u32(o.accel_device_id);w.u32(o.original_subscription_generation);
    w.byte(o.gyro_update_called?1:0);w.byte(o.accel_update_called?1:0);
    gpenmpc_local_gp_wire::write_hash(w,gpenmpc_local_gp_wire::digest(out.data(),message_bytes-32));return true;
}
inline bool encode_from_actual(const gpenmpc_odometry::Snapshot&s,
    const gpenmpc_hil_endpoint_reader::Receipt&r,Bytes&out)noexcept{
    Observation o{};out={};return from_actual(s,r,o)&&encode(o,out);
}
inline bool encode_runtime_state(const gpenmpc_odometry::Snapshot&s,Bytes&out)noexcept{
    Observation o{};gpenmpc_portable::Array<double,13>state{};
    if(!s.valid()||s.source_topic()!=ORB_ID(vehicle_odometry)||
       !gpenmpc_local_input::snapshot_key(s,o.source)||!gpenmpc_snapshot_mapping::state13(s,state))return false;
    o.odometry_instance=s.source_instance();
    std::memcpy(o.canonical_state13,state.data(),sizeof o.canonical_state13);
    std::memcpy(o.task_origin_ned_m,s.task_origin_ned_m().data(),sizeof o.task_origin_ned_m);
    return encode(o,out);
}
inline bool decode(const Bytes&b,Observation&out)noexcept{
    out={};if(std::memcmp(b.data(),"RLS1",4)||!gpenmpc_local_gp_wire::checksum(b))return false;
    Observation o{};gpenmpc_local_gp_wire::Reader r(b.data()+4);auto&s=o.source;auto&e=o.endpoint;
    s.identity=gpenmpc_local_gp_wire::read_identity(r);s.sample_us=r.u64();s.publication_us=r.u64();
    s.original_receipt_us=r.u64();s.source_generation=r.u64();s.generation_delta=r.u64();s.sample_delta_us=r.u64();
    s.reset_counter=r.byte();s.state_and_origin_sha256=r.hash();o.odometry_instance=r.u32();
    for(auto&v:o.task_origin_ned_m){v=r.real();}for(auto&v:o.canonical_state13){v=r.real();}
    e.original_receiver_hrt_us=r.u64();e.original_wire_time_us=r.u64();e.original_tap_event_ordinal=r.u64();
    e.fields_updated=r.u32();e.receiver_instance=signed32(r.u32());e.channel=signed32(r.u32());
    e.system=r.byte();e.component=r.byte();e.sequence=r.byte();e.payload_length=r.byte();e.sensor_id=r.byte();
    for(auto&v:e.original_sensor52){v=r.byte();}
    o.gyro_instance=signed32(r.u32());o.accel_instance=signed32(r.u32());o.gyro_device_id=r.u32();o.accel_device_id=r.u32();
    o.original_subscription_generation=r.u32();const auto gyro=r.byte(),accel=r.byte();
    if(gyro>1||accel>1)return false;
    o.gyro_update_called=gyro!=0;o.accel_update_called=accel!=0;e.all_gyro_fields_updated=(e.fields_updated&0x38u)==0x38u;
    if(!valid(o))return false;
    out=o;return true;
}
inline bool fragment(const Bytes&b,unsigned i,Fragment&out)noexcept{
    out={};if(i>=fragment_count||!gpenmpc_local_gp_wire::checksum(b)||std::memcmp(b.data(),"RLS1",4))return false;
    gpenmpc_local_gp_wire::Reader r(b.data()+46);const auto generation=r.u64();if(!generation)return false;
    const unsigned offset=i*119,n=message_bytes-offset<119?unsigned(message_bytes-offset):119;
    out.payload[0]=std::uint8_t((schema<<4)|i);gpenmpc_snapshot_wire::snapshot_detail::put64(out.payload+1,generation);
    std::memcpy(out.payload+9,b.data()+offset,n);out.length=std::uint8_t(n+9);return true;
}
} // namespace gpenmpc_local_snapshot_wire
