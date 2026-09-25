#pragma once
// Per-inner ORIGINAL GP query/reply on existing MAVLink TUNNEL. This is a
// numerical wire codec, NOT a source factory, age grant or actuator authority.
// RGP1/RGR1 are distinct from legacy RFC1's different stateless kernel fields.
// Owner must authenticate real link/session/arrival, permit only its installed
// pending query and call actual LocalIo.fill_gp before the next source capture.
#include "RflySnapshotWireTypes.hpp"
#include "../../px4_full_inner/consumption/ConsumptionBinding.hpp"

namespace gpenmpc_local_gp_wire {
using Identity=gpenmpc_consumption::Identity;
using Hash=gpenmpc_portable::Array<std::uint32_t,8>;
using Fragment=gpenmpc_snapshot_wire::Fragment;
constexpr std::size_t request_bytes=310,reply_bytes=286,fragment_count=3;
constexpr std::uint8_t request_schema=8,reply_schema=9;
using RequestBytes=gpenmpc_portable::Array<std::uint8_t,request_bytes>;
using ReplyBytes=gpenmpc_portable::Array<std::uint8_t,reply_bytes>;
struct Request {
    Identity identity{};
    std::uint64_t source_timestamp_ns{},source_generation{},output_generation{},
        original_publication_us{},publication_valid_until_us{};
    Hash configuration_sha256{},gp_model_sha256{};
    double request19[19]{}; // required=1, original f17, original mean scale
};
struct Reply {
    Identity identity{};
    std::uint64_t source_timestamp_ns{},source_generation{},output_generation{};
    Hash original_request_sha256{},gp_model_sha256{};
    double result18[18]{}; // GP numerical ABI, including hard-invalid state.
};
inline Hash canonical_model()noexcept{return {{0x4A09E9A3u,0xD4818B55u,0x55CD3439u,0xA6D21330u,0x26EEA0FDu,0xC2774A1Fu,0xB17F9E05u,0x486E5BB2u}};}
inline Hash canonical_configuration()noexcept{return {{0xA859433Du,0x0AA77401u,0x3341444Au,0x4B9AE971u,0xA34F12B4u,0xFB89C0A0u,0x04B2CB3Eu,0x013FCEBAu}};}
inline bool identity_valid(const Identity&i)noexcept{return i.uid&&i.boot_generation&&i.system&&i.component;}
inline bool empty(const Hash&h)noexcept{for(auto v:h)if(v)return false;return true;}
inline Hash digest(const std::uint8_t*b,std::size_t n)noexcept{
    gpenmpc_consumption::CanonicalSha256 h;for(std::size_t j=0;j<n;++j)h.byte(b[j]);return h.finish();
}
class Reader {
public:
    explicit Reader(const std::uint8_t*p)noexcept:p_(p){}
    std::uint8_t byte()noexcept{return *p_++;}
    std::uint32_t u32()noexcept{std::uint32_t x=0;for(unsigned i=0;i<4;++i)x=(x<<8)|byte();return x;}
    std::uint64_t u64()noexcept{std::uint64_t x=0;for(unsigned i=0;i<8;++i)x=(x<<8)|byte();return x;}
    double real()noexcept{const auto b=u64();double x;std::memcpy(&x,&b,8);return x;}
    Hash hash()noexcept{Hash h{};for(auto&w:h)w=u32();return h;}
private:const std::uint8_t*p_;
};
inline void write_hash(gpenmpc_snapshot_wire::Writer&w,const Hash&h)noexcept{for(auto v:h)w.u32(v);}
inline void write_identity(gpenmpc_snapshot_wire::Writer&w,const Identity&i)noexcept{
    w.u64(i.uid);w.u64(i.boot_generation);w.byte(i.system);w.byte(i.component);
}
inline Identity read_identity(Reader&r)noexcept{Identity i{};i.uid=r.u64();i.boot_generation=r.u64();i.system=r.byte();i.component=r.byte();return i;}
inline bool valid(const Request&r)noexcept{
    if(!identity_valid(r.identity)||!r.source_timestamp_ns||!r.source_generation||!r.output_generation||
       !r.original_publication_us||r.original_publication_us<r.source_timestamp_ns/1000||
       r.publication_valid_until_us<r.original_publication_us||r.configuration_sha256!=canonical_configuration()||
       r.gp_model_sha256!=canonical_model()||!(r.request19[0]>=1.0&&r.request19[0]<=1.0))return false;
    for(auto v:r.request19)if(!std::isfinite(v))return false;
    return true;
}
inline bool valid(const Reply&r)noexcept{
    if(!identity_valid(r.identity)||!r.source_timestamp_ns||!r.source_generation||!r.output_generation||
       empty(r.original_request_sha256)||r.gp_model_sha256!=canonical_model())return false;
    // Do not turn numerical hard-invalid/OOD into transport failure. Its exact
    // finite ABI value remains for the ORIGINAL numerical owner to interpret.
    for(auto v:r.result18)if(!std::isfinite(v))return false;
    return true;
}
inline bool encode(const Request&r,RequestBytes&out)noexcept{
    out={};if(!valid(r))return false;gpenmpc_snapshot_wire::Writer w(out.data());
    w.byte('R');w.byte('G');w.byte('P');w.byte('1');write_identity(w,r.identity);
    w.u64(r.source_timestamp_ns);w.u64(r.source_generation);w.u64(r.output_generation);
    w.u64(r.original_publication_us);w.u64(r.publication_valid_until_us);
    write_hash(w,r.configuration_sha256);write_hash(w,r.gp_model_sha256);
    for(auto v:r.request19){w.real(v);}
    write_hash(w,digest(out.data(),request_bytes-32));return true;
}
inline bool encode(const Reply&r,ReplyBytes&out)noexcept{
    out={};if(!valid(r))return false;gpenmpc_snapshot_wire::Writer w(out.data());
    w.byte('R');w.byte('G');w.byte('R');w.byte('1');write_identity(w,r.identity);
    w.u64(r.source_timestamp_ns);w.u64(r.source_generation);w.u64(r.output_generation);
    write_hash(w,r.original_request_sha256);write_hash(w,r.gp_model_sha256);
    for(auto v:r.result18){w.real(v);}
    write_hash(w,digest(out.data(),reply_bytes-32));return true;
}
template<std::size_t N>inline bool checksum(const gpenmpc_portable::Array<std::uint8_t,N>&b)noexcept{
    Reader r(b.data()+N-32);return r.hash()==digest(b.data(),N-32);
}
inline bool decode(const RequestBytes&b,Request&out)noexcept{
    out={};if(std::memcmp(b.data(),"RGP1",4)||!checksum(b))return false;
    Request r{};Reader q(b.data()+4);r.identity=read_identity(q);
    r.source_timestamp_ns=q.u64();r.source_generation=q.u64();r.output_generation=q.u64();
    r.original_publication_us=q.u64();r.publication_valid_until_us=q.u64();
    r.configuration_sha256=q.hash();r.gp_model_sha256=q.hash();for(auto&v:r.request19)v=q.real();
    if(!valid(r)){return false;}
    out=r;return true;
}
inline bool decode(const ReplyBytes&b,Reply&out)noexcept{
    out={};if(std::memcmp(b.data(),"RGR1",4)||!checksum(b))return false;
    Reply r{};Reader q(b.data()+4);r.identity=read_identity(q);
    r.source_timestamp_ns=q.u64();r.source_generation=q.u64();r.output_generation=q.u64();
    r.original_request_sha256=q.hash();r.gp_model_sha256=q.hash();for(auto&v:r.result18)v=q.real();
    if(!valid(r)){return false;}
    out=r;return true;
}
inline bool matches(const Request&r,const Reply&s)noexcept{
    RequestBytes b{};return encode(r,b)&&valid(s)&&r.identity==s.identity&&
        r.source_timestamp_ns==s.source_timestamp_ns&&r.source_generation==s.source_generation&&
        r.output_generation==s.output_generation&&r.gp_model_sha256==s.gp_model_sha256&&
        digest(b.data(),b.size())==s.original_request_sha256;
}
// The common 128-byte MAVLink TUNNEL payload, same 9-byte fragment header.
// Merely splits exact bytes. Actual ingress retains original arrival HRT,
// sequence and per-link validation; neither helper opens/sends anything.
template<std::size_t N>inline bool fragment(const gpenmpc_portable::Array<std::uint8_t,N>&b,unsigned i,Fragment&out)noexcept{
    static_assert(N==request_bytes||N==reply_bytes,"exact GP message shape");out={};
    if(i>=fragment_count||!checksum(b))return false;
    Reader r(b.data()+38);const auto generation=r.u64();if(!generation)return false;
    const unsigned offset=i*119,n=unsigned(N)-offset<119?unsigned(N)-offset:119;
    out.payload[0]=std::uint8_t(((N==request_bytes?request_schema:reply_schema)<<4)|i);
    gpenmpc_snapshot_wire::snapshot_detail::put64(out.payload+1,generation);
    std::memcpy(out.payload+9,b.data()+offset,n);out.length=std::uint8_t(n+9);return true;
}
} // namespace gpenmpc_local_gp_wire
