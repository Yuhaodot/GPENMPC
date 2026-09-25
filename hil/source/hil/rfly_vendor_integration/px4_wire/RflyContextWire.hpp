#pragma once
// RCT1 carries HOST times as HOST times. The board creates a binding once,
// anchored conservatively to the ORIGINAL private source sample, never now+TTL.
// This codec does not authenticate HOST clock events or confer live authority.
#include "RflySnapshotWire.hpp"
namespace gpenmpc_context_wire {
using namespace gpenmpc_argument_transport;
using Ticket=gpenmpc_rfly_state_execution::SnapshotTicket;
constexpr std::size_t message_bytes=316,fragment_count=3;
constexpr std::uint8_t schema_version=4;
using Bytes=gpenmpc_portable::Array<std::uint8_t,message_bytes>;
struct HostTimes {std::uint64_t source_receipt_ns{},creation_ns{},expiry_ns{};};
struct Context {
    Bytes32 configuration_sha256{};std::uint64_t reference_generation{},outer_generation{};
    Ticket reference_source_ticket{},outer_source_ticket{};HostTimes reference_time{},outer_time{};
    gpenmpc_portable::Array<double,11> reference_ned{}; // p3,v3,a3,yaw,yaw-rate
    gpenmpc_portable::Array<double,4> outer_payload{}; // actual phase accel + correction F3
};
inline bool valid_times(const HostTimes&t)noexcept{return t.source_receipt_ns&&t.creation_ns>=t.source_receipt_ns&&t.expiry_ns>=t.creation_ns;}
inline bool same_times(const HostTimes&a,const HostTimes&b)noexcept{return a.source_receipt_ns==b.source_receipt_ns&&a.creation_ns==b.creation_ns&&a.expiry_ns==b.expiry_ns;}
inline bool valid(const Context&c)noexcept{
    if(!c.reference_generation||!c.outer_generation||!valid_times(c.reference_time)||!valid_times(c.outer_time))return false;
    for(double v:c.reference_ned)if(!std::isfinite(v))return false;
    for(double v:c.outer_payload)if(!std::isfinite(v))return false;
    return true;
}
inline bool encode(const Context&c,Bytes&out)noexcept{
    out={};if(!valid(c))return false;gpenmpc_snapshot_wire::Writer w(out.data());
    w.byte('R');w.byte('C');w.byte('T');w.byte('1');w.bytes(c.configuration_sha256);
    w.u64(c.reference_generation);w.u64(c.outer_generation);w.bytes(c.reference_source_ticket);w.bytes(c.outer_source_ticket);
    w.u64(c.reference_time.source_receipt_ns);w.u64(c.reference_time.creation_ns);w.u64(c.reference_time.expiry_ns);
    w.u64(c.outer_time.source_receipt_ns);w.u64(c.outer_time.creation_ns);w.u64(c.outer_time.expiry_ns);
    for(double v:c.reference_ned)w.real(v);
    for(double v:c.outer_payload)w.real(v);
    w.bytes(digest(out.data(),message_bytes-32));return true;
}
inline bool decode(const Bytes&in,Context&out)noexcept{
    out={};if(in[0]!='R'||in[1]!='C'||in[2]!='T'||in[3]!='1')return false;
    const auto hash=digest(in.data(),message_bytes-32);
    if(std::memcmp(hash.data(),in.data()+message_bytes-32,32)!=0)return false;
    const auto*p=in.data()+4;copy_bytes(p,32,out.configuration_sha256.data());p+=32;
    out.reference_generation=get64(p);out.outer_generation=get64(p+8);p+=16;
    copy_bytes(p,32,out.reference_source_ticket.data());copy_bytes(p+32,32,out.outer_source_ticket.data());p+=64;
    out.reference_time={get64(p),get64(p+8),get64(p+16)};p+=24;
    out.outer_time={get64(p),get64(p+8),get64(p+16)};p+=24;
    gpenmpc_kernel_abi::Reader r(p);r.reals(out.reference_ned);r.reals(out.outer_payload);return valid(out);
}
inline bool fragment(const Bytes&message,unsigned index,Fragment&out)noexcept{
    out={};if(index>=fragment_count||!get64(message.data()+36))return false;
    const std::size_t offset=index*119,n=(message_bytes-offset)<119?(message_bytes-offset):119;
    out.payload[0]=std::uint8_t((schema_version<<4)|index);put64(out.payload+1,get64(message.data()+36));
    copy_bytes(message.data()+offset,n,out.payload+9);out.length=std::uint8_t(n+9);return true;
}
struct Anchor {Ticket ticket{};gpenmpc_consumption::Identity identity{};std::uint64_t generation{},sample_us{},publication_us{},receipt_us{};bool present{};};
// Application storage capacity. Export preserves the ticket source timestamp.
template<std::size_t Capacity>class AnchorStore final {
public:
    static_assert(Capacity>0,"explicit positive anchor capacity");
    template<class PrivateOwner>bool record(const PrivateOwner&owner,const Ticket&ticket,
        const gpenmpc_rfly_execution::Configuration&approved,std::uint64_t now,gpenmpc_snapshot_wire::Bytes&wire)noexcept{
        if(!gpenmpc_snapshot_wire::encode(owner,ticket,approved,wire))return false;
        const auto*s=owner.snapshot(ticket);const auto&e=s->estimator();
        if(!now||e.timestamp_sample_us>now||now-e.timestamp_sample_us>approved.limits.sample_max_age_us)return false;
        if(find(ticket))return true;
        for(auto&a:anchors_)if(!a.present){a={ticket,e.identity,e.generation,e.timestamp_sample_us,e.publication_us,e.board_rx_us,true};return true;}
        return false; // no silently overwritten pending outer basis
    }
    const Anchor*find(const Ticket&ticket)const noexcept{for(const auto&a:anchors_)if(a.present&&a.ticket==ticket)return &a;return nullptr;}
    std::size_t count()const noexcept{std::size_t n=0;for(const auto&a:anchors_)if(a.present)++n;return n;}
    // Owner may retire only after no held/reference/outer context needs it.
    bool retire(const Ticket&ticket)noexcept{for(auto&a:anchors_)if(a.present&&a.ticket==ticket){a={};return true;}return false;}
    void retire_expired(std::uint64_t now,std::uint64_t maximum_original_age)noexcept{
        for(auto&a:anchors_)if(a.present&&now>=a.sample_us&&now-a.sample_us>maximum_original_age)a={};
    }
private:gpenmpc_portable::Array<Anchor,Capacity>anchors_{};
};
enum class Failure:std::uint8_t{None,Configuration,Malformed,MissingAnchor,SourceIdentity,Time,Expired,Regression,Mutation,Lineage};
class ContextBinding final {
public:
    explicit ContextBinding(const gpenmpc_rfly_execution::Configuration&c)noexcept:approved_(c){
        if(!c.limits.reference_max_age_us||!c.limits.outer_max_age_us||c.configuration_payload_sha256!=gpenmpc_rfly_execution::kCanonicalConfigurationSha)fail(Failure::Configuration);
    }
    template<std::size_t N>bool accept(const Context&c,const AnchorStore<N>&anchors,std::uint64_t original_ingress_us,std::uint64_t now)noexcept{
        if(failed())return false;
        if(!valid(c)||c.configuration_sha256!=bytes_of(approved_.configuration_payload_sha256))return fail(Failure::Malformed);
        if(!original_ingress_us||original_ingress_us>now||original_ingress_us<last_ingress_)return fail(Failure::Time);
        if(have_&&(c.reference_generation<current_.reference_generation||c.outer_generation<current_.outer_generation))return fail(Failure::Regression);
        const bool new_ref=!have_||c.reference_generation!=current_.reference_generation;
        const bool new_outer=!have_||c.outer_generation!=current_.outer_generation;
        if(have_&&!new_ref&&(!same_reference(c,current_)||new_outer))return fail(Failure::Mutation);
        if(have_&&!new_outer&&!same_outer(c,current_))return fail(Failure::Mutation);
        const auto*r=anchors.find(c.reference_source_ticket);const auto*o=anchors.find(c.outer_source_ticket);
        if(!r||!o)return fail(Failure::MissingAnchor);
        if(!(r->identity==approved_.identity)||!(o->identity==approved_.identity))return fail(Failure::SourceIdentity);
        if(o->generation>r->generation||o->sample_us>r->sample_us)return fail(Failure::Lineage);
        if(r->receipt_us>original_ingress_us||o->receipt_us>original_ingress_us)return fail(Failure::Time);
        std::uint64_t ref_expiry=reference_.valid_until_us,outer_expiry=outer_.valid_until_us;
        if(new_ref&&!deadline(*r,c.reference_time,approved_.limits.reference_max_age_us,ref_expiry))return fail(Failure::Time);
        if(new_outer&&!deadline(*o,c.outer_time,approved_.limits.outer_max_age_us,outer_expiry))return fail(Failure::Time);
        if(now>ref_expiry||now>outer_expiry)return fail(Failure::Expired);
        if(have_&&((new_ref&&(original_ingress_us<=reference_.board_rx_us||c.reference_time.creation_ns<=current_.reference_time.creation_ns))||
                  (new_outer&&(original_ingress_us<=outer_.board_rx_us||c.outer_time.creation_ns<=current_.outer_time.creation_ns))))return fail(Failure::Regression);
        if(new_outer){outer_={};outer_.identity=o->identity;outer_.generation=c.outer_generation;
            outer_.based_on_sample_generation=o->generation;outer_.based_on_timestamp_sample_us=o->sample_us;
            outer_.board_rx_us=original_ingress_us;outer_.valid_until_us=outer_expiry;
            gpenmpc_portable::Array<std::uint8_t,32>payload{};gpenmpc_snapshot_wire::Writer w(payload.data());
            for(double v:c.outer_payload)w.real(v);
            outer_.payload_sha256=words_of(digest(payload.data(),payload.size()));}
        if(new_ref){reference_={};reference_.identity=r->identity;reference_.generation=c.reference_generation;reference_.outer_generation=c.outer_generation;
            // This is ORIGINAL board binding creation, NOT a cast of HOST ns.
            reference_.timestamp_us=original_ingress_us;reference_.board_rx_us=original_ingress_us;reference_.valid_until_us=ref_expiry;
            for(unsigned k=0;k<3;++k){reference_.p[k]=c.reference_ned[k];reference_.v[k]=c.reference_ned[k+3];reference_.a[k]=c.reference_ned[k+6];}
            reference_.yaw=c.reference_ned[9];reference_.yaw_rate=c.reference_ned[10];}
        current_=c;have_=true;last_ingress_=original_ingress_us;return true;
    }
    const gpenmpc_consumption::Reference&reference()const noexcept{return reference_;}
    const gpenmpc_consumption::OuterCommand&outer()const noexcept{return outer_;}
    const Context&original_host_context()const noexcept{return current_;}
    bool ready()const noexcept{return have_&&!failed();}bool failed()const noexcept{return failure_!=Failure::None;}Failure failure()const noexcept{return failure_;}
private:
    static bool deadline(const Anchor&a,const HostTimes&t,std::uint64_t limit,std::uint64_t&out)noexcept{
        if(!valid_times(t))return false;
        const auto elapsed=t.creation_ns-t.source_receipt_ns;
        if(elapsed/1000>limit||(elapsed/1000==limit&&elapsed%1000))return false;
        const auto horizon=(t.expiry_ns-t.source_receipt_ns)/1000;
        const auto duration=horizon<limit?horizon:limit;
        if(!a.sample_us||!duration||a.sample_us>UINT64_MAX-duration)return false;
        out=a.sample_us+duration;return true;
    }
    static bool same_reference(const Context&a,const Context&b)noexcept{return a.reference_source_ticket==b.reference_source_ticket&&same_times(a.reference_time,b.reference_time)&&std::memcmp(a.reference_ned.data(),b.reference_ned.data(),88)==0;}
    static bool same_outer(const Context&a,const Context&b)noexcept{return a.outer_source_ticket==b.outer_source_ticket&&same_times(a.outer_time,b.outer_time)&&std::memcmp(a.outer_payload.data(),b.outer_payload.data(),32)==0;}
    bool fail(Failure f)noexcept{if(!failed())failure_=f;return false;}
    const gpenmpc_rfly_execution::Configuration approved_;Context current_{};gpenmpc_consumption::Reference reference_{};gpenmpc_consumption::OuterCommand outer_{};
    std::uint64_t last_ingress_{};Failure failure_{Failure::None};bool have_{};
};
} // namespace gpenmpc_context_wire
