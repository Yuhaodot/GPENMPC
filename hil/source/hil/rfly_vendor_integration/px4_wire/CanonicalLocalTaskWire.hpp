#pragma once
// Minimal input extension of the existing 42002 TUNNEL. Contains original
// rotor-lag observation, payload/estimated wind and the held .30s outer target.
// It contains NO inner state replacement, force/moment, or motor command.
#include "CanonicalLocalGpWire.hpp"
#include "../CanonicalLocalInnerInputBuilder.hpp"
#include "../../px4_full_inner/argument_transport/CanonicalArgumentTransport.hpp"

namespace gpenmpc_local_task_wire {
using Hash=gpenmpc_local_input::Hash;
using Bytes=gpenmpc_portable::Array<std::uint8_t,647>;
constexpr std::uint8_t schema=13;
constexpr unsigned fragment_count=6;
struct Message {
    Hash session_sha{},configuration_sha{},task_sha{},reference_sha{};
    std::uint32_t leg{};
    gpenmpc_local_input::SnapshotKey source{};
    std::uint8_t original_sensor52[52]{};
    std::uint64_t rotor_generation{},rotor_session{},rotor_host_receive_ns{};
    double rotor_sim_time_s{},rotor_n[6]{};
    Hash rotor_observation_sha{},rotor_association_sha{};
    std::uint64_t payload_generation{};Hash payload_evidence_sha{};double payload_kg{};
    std::uint64_t wind_generation{};Hash wind_evidence_sha{};double estimated_wind_xy[2]{};
    std::uint64_t outer_generation{},outer_source_generation{},outer_sample_us{};
    std::uint64_t outer_original_host_source_rx_ns{},outer_creation_ns{},outer_expiry_ns{};
    double outer_target4[4]{};
};
inline bool valid(const Message&m)noexcept{
    const auto&s=m.source;
    if(gpenmpc_local_input::empty(m.session_sha)||gpenmpc_local_input::empty(m.configuration_sha)||
       gpenmpc_local_input::empty(m.task_sha)||gpenmpc_local_input::empty(m.reference_sha)||!m.leg||m.leg>5||
       !s.identity.uid||!s.identity.boot_generation||!s.identity.system||!s.identity.component||
       !s.sample_us||!s.source_generation||s.publication_us<s.sample_us||s.original_receipt_us<s.publication_us||
       gpenmpc_local_input::empty(s.state_and_origin_sha256)||!m.rotor_generation||!m.rotor_session||
       !m.rotor_host_receive_ns||!std::isfinite(m.rotor_sim_time_s)||m.rotor_sim_time_s<0||
       gpenmpc_local_input::empty(m.rotor_observation_sha)||gpenmpc_local_input::empty(m.rotor_association_sha)||
       !m.payload_generation||gpenmpc_local_input::empty(m.payload_evidence_sha)||!std::isfinite(m.payload_kg)||m.payload_kg<0||
       !m.wind_generation||gpenmpc_local_input::empty(m.wind_evidence_sha)||
       !m.outer_generation||!m.outer_source_generation||m.outer_source_generation>s.source_generation||
       !m.outer_sample_us||m.outer_sample_us>s.sample_us||!m.outer_original_host_source_rx_ns||
       m.outer_creation_ns<m.outer_original_host_source_rx_ns||m.outer_expiry_ns<m.outer_creation_ns)return false;
    for(double v:m.rotor_n)if(!std::isfinite(v)||v<0)return false;
    for(double v:m.estimated_wind_xy)if(!std::isfinite(v))return false;
    for(double v:m.outer_target4)if(!std::isfinite(v))return false;
    return true;
}
// No struct padding, implicit clock casts, hash labels or bool permissions.
inline bool encode(const Message&m,Bytes&out)noexcept{
    out={};if(!valid(m))return false;
    std::size_t p=0;
    const auto b=[&](std::uint8_t v){out[p++]=v;};
    const auto u32=[&](std::uint32_t v){for(int j=3;j>=0;--j)b(std::uint8_t(v>>(8*j)));};
    const auto u64=[&](std::uint64_t v){for(int j=7;j>=0;--j)b(std::uint8_t(v>>(8*j)));};
    const auto real=[&](double v){std::uint64_t bits{};std::memcpy(&bits,&v,8);u64(bits);};
    const auto hash=[&](const Hash&h){for(auto v:h)u32(v);};
    b('R');b('L');b('I');b('1');hash(m.session_sha);hash(m.configuration_sha);hash(m.task_sha);hash(m.reference_sha);u32(m.leg);
    const auto&s=m.source;u64(s.identity.uid);u64(s.identity.boot_generation);b(s.identity.system);b(s.identity.component);
    u64(s.sample_us);u64(s.publication_us);u64(s.original_receipt_us);u64(s.source_generation);u64(s.generation_delta);u64(s.sample_delta_us);
    b(s.reset_counter);hash(s.state_and_origin_sha256);for(auto v:m.original_sensor52)b(v);
    u64(m.rotor_generation);u64(m.rotor_session);u64(m.rotor_host_receive_ns);real(m.rotor_sim_time_s);
    for(auto v:m.rotor_n){real(v);}hash(m.rotor_observation_sha);hash(m.rotor_association_sha);
    u64(m.payload_generation);hash(m.payload_evidence_sha);real(m.payload_kg);
    u64(m.wind_generation);hash(m.wind_evidence_sha);for(auto v:m.estimated_wind_xy)real(v);
    u64(m.outer_generation);u64(m.outer_source_generation);u64(m.outer_sample_us);
    u64(m.outer_original_host_source_rx_ns);u64(m.outer_creation_ns);u64(m.outer_expiry_ns);for(auto v:m.outer_target4)real(v);
    if(p!=out.size()-32)return false;
    const auto h=gpenmpc_argument_transport::digest(out.data(),p);for(auto v:h)b(v);
    return p==out.size();
}
inline bool decode(const Bytes&in,Message&m)noexcept{
    m={};if(std::memcmp(in.data(),"RLI1",4))return false;
    const auto expected=gpenmpc_argument_transport::digest(in.data(),in.size()-32);
    if(std::memcmp(expected.data(),in.data()+in.size()-32,32))return false;
    std::size_t p=4;
    const auto byte=[&](){return in[p++];};
    const auto u32=[&](){std::uint32_t v=0;for(unsigned j=0;j<4;++j)v=(v<<8)|byte();return v;};
    const auto u64=[&](){std::uint64_t v=0;for(unsigned j=0;j<8;++j)v=(v<<8)|byte();return v;};
    const auto real=[&](){const auto bits=u64();double v;std::memcpy(&v,&bits,8);return v;};
    const auto hash=[&](Hash&h){for(auto&v:h)v=u32();};
    hash(m.session_sha);hash(m.configuration_sha);hash(m.task_sha);hash(m.reference_sha);m.leg=u32();
    auto&s=m.source;s.identity.uid=u64();s.identity.boot_generation=u64();s.identity.system=byte();s.identity.component=byte();
    s.sample_us=u64();s.publication_us=u64();s.original_receipt_us=u64();s.source_generation=u64();s.generation_delta=u64();s.sample_delta_us=u64();
    s.reset_counter=byte();hash(s.state_and_origin_sha256);for(auto&v:m.original_sensor52)v=byte();
    m.rotor_generation=u64();m.rotor_session=u64();m.rotor_host_receive_ns=u64();m.rotor_sim_time_s=real();
    for(auto&v:m.rotor_n){v=real();}hash(m.rotor_observation_sha);hash(m.rotor_association_sha);
    m.payload_generation=u64();hash(m.payload_evidence_sha);m.payload_kg=real();
    m.wind_generation=u64();hash(m.wind_evidence_sha);for(auto&v:m.estimated_wind_xy)v=real();
    m.outer_generation=u64();m.outer_source_generation=u64();m.outer_sample_us=u64();
    m.outer_original_host_source_rx_ns=u64();m.outer_creation_ns=u64();m.outer_expiry_ns=u64();for(auto&v:m.outer_target4)v=real();
    return p==in.size()-32&&valid(m);
}
inline bool fragment(const Bytes&bytes,unsigned index,gpenmpc_argument_transport::Fragment&out)noexcept{
    out={};if(index>=fragment_count)return false;
    // Source generation starts at byte178 (0-based); encoded key is immutable.
    const auto source_generation=gpenmpc_argument_transport::get64(bytes.data()+178);
    if(!source_generation)return false;
    const unsigned offset=index*119,n=unsigned(bytes.size()-offset)<119?unsigned(bytes.size()-offset):119;
    out.payload[0]=std::uint8_t((schema<<4)|index);gpenmpc_argument_transport::put64(out.payload+1,source_generation);
    std::memcpy(out.payload+9,bytes.data()+offset,n);out.length=std::uint8_t(n+9);return true;
}
// Concrete owning receiver attaches here; only the shared ingress dispatcher
// calls these after its one sequence/uORB/source/fault validation path.
class Receiver {
public:
    virtual ~Receiver()=default;
    virtual bool receive(const gpenmpc_argument_transport::Arrival&,std::uint64_t)noexcept=0;
    virtual bool tick(std::uint64_t,bool disarmed_observation=false)noexcept=0;
    virtual void stop(std::uint64_t)noexcept=0;
};
}
