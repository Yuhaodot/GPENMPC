#pragma once
// RLC2 exports joint-committed state, closed evidence and phase over TUNNEL 42002.
#include "CanonicalLocalGpWire.hpp"
#include "../px4_runtime/LocalCommittedState.hpp"
#include "../local_phase/CanonicalLocalPhaseClock.hpp"
namespace gpenmpc_local_committed_wire {
using Identity=gpenmpc_consumption::Identity;
using Bytes=gpenmpc_portable::Array<std::uint8_t,1494>;
constexpr std::size_t message_bytes=1494,fragment_count=13;
constexpr std::uint8_t schema=14;
struct Observation {
    Identity identity{};
    std::uint64_t source_timestamp_ns{},source_generation{},output_generation{},original_publication_us{},
        joint_installs{},gp_fills{},window_generation{},query_sequence{},outer_generation{};
    std::uint32_t leg_index{};
    double state64[64]{},closed5[5]{},reference6[6]{},installed_phase2[2]{},kernel61[61]{};
    float published_control16[16]{};
    std::uint8_t state_kind{},closed_available{},numeric_installed{},prediction_ready{};
    std::uint8_t committed_state_sha256[32]{},configuration_sha256[32]{},reference_asset_sha256[32]{};
    double learning12[12]{};
};
inline bool valid(const Observation&o)noexcept{
    if(!gpenmpc_local_gp_wire::identity_valid(o.identity)||!o.source_timestamp_ns||!o.source_generation||
       !o.output_generation||o.original_publication_us<o.source_timestamp_ns/1000||!o.joint_installs||
       !o.window_generation||!o.query_sequence||!o.outer_generation||!o.leg_index||o.leg_index>5||
       o.state_kind<2||o.state_kind>4||o.closed_available>1||o.numeric_installed!=1||o.prediction_ready>1)return false;
    for(unsigned i=0;i<4;++i)if(!std::isfinite(o.closed5[i])||
        !((o.closed5[i]<=0&&o.closed5[i]>=0)||(o.closed5[i]<=1&&o.closed5[i]>=1)))return false;
    if(!std::isfinite(o.closed5[4])||o.closed5[4]<0||o.closed5[4]>1||
       o.closed_available!=static_cast<std::uint8_t>(o.closed5[0]))return false;
    for(auto v:o.reference6)if(!std::isfinite(v))return false;
    for(auto v:o.installed_phase2)if(!std::isfinite(v))return false;
    for(auto v:o.published_control16)if(!std::isfinite(v))return false;
    for(unsigned j=3;j<12;++j)if(!std::isfinite(o.learning12[j])||o.learning12[j]<0||o.learning12[j]>1)return false;
    if(!((o.learning12[9]<=0&&o.learning12[9]>=0)||(o.learning12[9]<=1&&o.learning12[9]>=1)))return false;
    // An unavailable close() can retain an observed inertial label and a NaN
    // Frenet innovation. Preserve both in the Exact-B1 fallback record.
    if(o.closed5[0]>0&&o.closed5[2]>0)for(unsigned j=0;j<3;++j)if(!std::isfinite(o.learning12[j]))return false;
    return true;
}
inline bool from_actual(const gpenmpc_rfly_px4::LocalNumericalObservation&n,
    const gpenmpc_local_phase::Diagnostics&p,Observation&o)noexcept{
    o={};const auto&s=n.state;const auto&r=s.reference;const auto&c=n.closed_gp;
    if(!n.copied||!n.installed_evidence_matches_state||!s.numeric_installed||!s.reference_committed||
       !n.last_installed.present||!c.exported||!c.installed||!c.learning_exported||n.control_authority||n.source_freshness_granted||
       n.kind==gpenmpc_rfly_px4::LocalStateKind::HistoricalUnusable||n.io_first_fault||
       p.first_fault!=gpenmpc_local_phase::Fault::None||p.installs!=n.actual_joint_installs||
       p.last_source_timestamp_ns!=s.original_installed_tags2[0]||p.last_source_generation!=s.original_installed_tags2[1]||
       p.last_output_generation!=r.output_generation||p.last_publication_us!=r.publication_us||
       c.original_publication_us!=r.publication_us||std::memcmp(c.original_installed_tags2,s.original_installed_tags2,16))return false;
    o.identity=n.last_installed.token.lease_envelope.identity;
    o.source_timestamp_ns=s.original_installed_tags2[0];o.source_generation=s.original_installed_tags2[1];
    o.output_generation=r.output_generation;o.original_publication_us=r.publication_us;
    o.joint_installs=n.actual_joint_installs;o.gp_fills=n.actual_gp_fills;o.window_generation=r.window_generation;
    o.query_sequence=r.last_accepted_sequence;o.outer_generation=r.outer_generation;o.leg_index=r.leg_index;
    std::memcpy(o.state64,s.state64,sizeof o.state64);std::memcpy(o.closed5,c.values5,sizeof o.closed5);
    o.reference6[0]=r.query_progress_s;o.reference6[1]=r.progress_rate;o.reference6[2]=r.phase_acceleration;
    std::memcpy(o.reference6+3,r.outer_i,sizeof r.outer_i);o.installed_phase2[0]=p.phase_s;o.installed_phase2[1]=p.phase_rate;
    std::memcpy(o.kernel61,n.last_installed.actual_kernel61,sizeof o.kernel61);
    std::memcpy(o.published_control16,n.last_installed.actual_published_control16,sizeof o.published_control16);
    o.state_kind=static_cast<std::uint8_t>(n.kind);o.closed_available=n.latest_closed_gp_evidence_available?1:0;
    o.numeric_installed=s.numeric_installed;o.prediction_ready=s.prediction_ready;
    std::memcpy(o.committed_state_sha256,s.committed_state_sha256,32);
    std::memcpy(o.configuration_sha256,n.configuration.configuration_sha256,32);std::memcpy(o.reference_asset_sha256,r.reference_asset_sha256,32);
    std::memcpy(o.learning12,c.learning12,sizeof o.learning12);
    return valid(o);
}
inline bool encode(const Observation&o,Bytes&b)noexcept{
    b={};if(!valid(o))return false;gpenmpc_snapshot_wire::Writer w(b.data());
    w.byte('R');w.byte('L');w.byte('C');w.byte('2');gpenmpc_local_gp_wire::write_identity(w,o.identity);
    w.u64(o.source_timestamp_ns);w.u64(o.source_generation);w.u64(o.output_generation);w.u64(o.original_publication_us);
    w.u64(o.joint_installs);w.u64(o.gp_fills);w.u64(o.window_generation);w.u64(o.query_sequence);w.u64(o.outer_generation);
    w.u32(o.leg_index);
    for(auto v:o.state64){w.real(v);}
    for(auto v:o.closed5){w.real(v);}
    for(auto v:o.reference6){w.real(v);}
    for(auto v:o.installed_phase2){w.real(v);}
    for(auto v:o.kernel61){w.real(v);}
    for(auto v:o.published_control16){std::uint32_t bits;std::memcpy(&bits,&v,4);w.u32(bits);}
    w.byte(o.state_kind);w.byte(o.closed_available);w.byte(o.numeric_installed);w.byte(o.prediction_ready);
    for(auto v:o.committed_state_sha256){w.byte(v);}
    for(auto v:o.configuration_sha256){w.byte(v);}
    for(auto v:o.reference_asset_sha256){w.byte(v);}
    for(auto v:o.learning12){w.real(v);}
    gpenmpc_local_gp_wire::write_hash(w,gpenmpc_local_gp_wire::digest(b.data(),message_bytes-32));return true;
}
inline bool decode(const Bytes&b,Observation&o)noexcept{
    o={};if(std::memcmp(b.data(),"RLC2",4)||!gpenmpc_local_gp_wire::checksum(b))return false;
    gpenmpc_local_gp_wire::Reader r(b.data()+4);o.identity=gpenmpc_local_gp_wire::read_identity(r);
    o.source_timestamp_ns=r.u64();o.source_generation=r.u64();o.output_generation=r.u64();o.original_publication_us=r.u64();
    o.joint_installs=r.u64();o.gp_fills=r.u64();o.window_generation=r.u64();o.query_sequence=r.u64();o.outer_generation=r.u64();
    o.leg_index=r.u32();
    for(auto&v:o.state64){v=r.real();}
    for(auto&v:o.closed5){v=r.real();}
    for(auto&v:o.reference6){v=r.real();}
    for(auto&v:o.installed_phase2){v=r.real();}
    for(auto&v:o.kernel61){v=r.real();}
    for(auto&v:o.published_control16){const auto bits=r.u32();std::memcpy(&v,&bits,4);}
    o.state_kind=r.byte();o.closed_available=r.byte();o.numeric_installed=r.byte();o.prediction_ready=r.byte();
    for(auto&v:o.committed_state_sha256){v=r.byte();}
    for(auto&v:o.configuration_sha256){v=r.byte();}
    for(auto&v:o.reference_asset_sha256){v=r.byte();}
    for(auto&v:o.learning12){v=r.real();}
    return valid(o);
}
}
