#pragma once
// Snapshot bytes/fragments only: no numerical owner, source factory, ticket
// construction, source validity, control authority, HRT read or I/O.
#include "../../px4_full_inner/portable/CanonicalPortable.hpp"
namespace gpenmpc_snapshot_wire {
constexpr std::size_t message_bytes=246,fragment_count=3;
constexpr std::uint8_t schema_version=3;
using Bytes=gpenmpc_portable::Array<std::uint8_t,message_bytes>;
struct Fragment {std::uint8_t payload[128]{};std::uint8_t length{};};
static_assert(sizeof(Fragment)==129&&offsetof(Fragment,length)==128,"original fragment POD layout");
class Writer {
public:
    explicit Writer(std::uint8_t*p)noexcept:p_(p){}
    void byte(std::uint8_t x)noexcept{*p_++=x;}
    void u32(std::uint32_t x)noexcept{for(unsigned k=0;k<4;++k)byte(std::uint8_t(x>>(24-8*k)));}
    void u64(std::uint64_t x)noexcept{for(unsigned k=0;k<8;++k)byte(std::uint8_t(x>>(56-8*k)));}
    void f32(float x)noexcept{std::uint32_t b=0;std::memcpy(&b,&x,4);u32(b);}
    void real(double x)noexcept{std::uint64_t b=0;std::memcpy(&b,&x,8);u64(b);}
    template<class T>void bytes(const T&a)noexcept{for(auto x:a)byte(x);}
private:std::uint8_t*p_;
};
namespace snapshot_detail {
inline void put64(std::uint8_t*p,std::uint64_t x) noexcept {for(unsigned k=0;k<8;++k)p[k]=static_cast<std::uint8_t>(x>>(56-8*k));}
inline void copy_bytes(const std::uint8_t*src,std::size_t n,std::uint8_t*dst) noexcept {for(std::size_t k=0;k<n;++k)dst[k]=src[k];}
}
// Legacy callers may retain their identical transport Fragment type. Neither
// accepted type admits source data: this function only chunks existing bytes.
template<class FragmentType>inline bool fragment(const Bytes&message,unsigned index,FragmentType&out)noexcept{
    static_assert(sizeof(out.payload)==128&&sizeof(out.length)==1,"original fixed fragment fields");
    out={};if(index>=fragment_count)return false;
    const std::size_t offset=index*119,n=(message_bytes-offset)<119?(message_bytes-offset):119;
    out.payload[0]=std::uint8_t((schema_version<<4)|index);
    // 32-bit original subscription generation at byte58, never HOST sequence.
    std::uint32_t generation=0;for(unsigned k=0;k<4;++k)generation=(generation<<8)|message[58+k];
    if(!generation)return false;
    snapshot_detail::put64(out.payload+1,generation);
    snapshot_detail::copy_bytes(message.data()+offset,n,out.payload+9);out.length=std::uint8_t(n+9);return true;
}
} // namespace gpenmpc_snapshot_wire
