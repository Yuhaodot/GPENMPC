#include "CanonicalLocalWindowWire.hpp"
#include <cstddef>
#include <cstring>

namespace gpenmpc_local_window_wire {
namespace {
struct Field {std::size_t offset,bytes,width;};
#define RWW_FIELD(name,count,width) {offsetof(Window,name),(count)*(width),(width)}
const Field fields[]={
    RWW_FIELD(schema,1,4),RWW_FIELD(capacity,1,4),RWW_FIELD(leg_index,1,4),
    RWW_FIELD(source_first_row,1,4),RWW_FIELD(source_total_rows,1,4),RWW_FIELD(row_count,1,4),RWW_FIELD(binding_mode,1,4),
    RWW_FIELD(reference_asset_sha256,32,1),RWW_FIELD(window_generation,1,8),
    RWW_FIELD(nominal_duration_s,1,8),RWW_FIELD(total_duration_s,1,8),RWW_FIELD(prefix_duration_s,1,8),
    RWW_FIELD(relaunch_duration_s,1,8),RWW_FIELD(vertical_frame_offset_ned_m,1,8),
    RWW_FIELD(time_s,256,8),RWW_FIELD(nominal_jet,3072,8),RWW_FIELD(prefix_coefficients,192,8),
    RWW_FIELD(ground_jet,12,8),RWW_FIELD(rest_jet,12,8),RWW_FIELD(relaunch_offset_ned_m,3,8)};
#undef RWW_FIELD
static_assert(sizeof(double)==8&&sizeof(std::uint64_t)==8,"Original binary64 window ABI");
bool nonzero(const Hash&h)noexcept{for(auto x:h)if(x)return true;return false;}
std::uint8_t hash_byte(const Hash&h,std::size_t n)noexcept{return static_cast<std::uint8_t>(h[n/4]>>(24-8*(n%4)));}
bool little_endian()noexcept{const std::uint32_t x=1;return *reinterpret_cast<const unsigned char*>(&x)==1;}
template<bool Writing>
bool transfer(const Window*source,Window*destination,std::size_t offset,std::uint8_t*output,const std::uint8_t*input,std::size_t n)noexcept{
    if(offset>window_bytes||n>window_bytes-offset||(!output&&!Writing)||(!input&&Writing))return false;
    const bool little=little_endian();
    for(const auto&f:fields){
        if(offset>=f.bytes){offset-=f.bytes;continue;}
        const auto count=n<(f.bytes-offset)?n:(f.bytes-offset);
        const auto*from=source?reinterpret_cast<const std::uint8_t*>(source)+f.offset:nullptr;
        auto*to=destination?reinterpret_cast<std::uint8_t*>(destination)+f.offset:nullptr;
        if(little){if(Writing)std::memcpy(to+offset,input,count);else std::memcpy(output,from+offset,count);}
        else for(std::size_t j=0;j<count;++j){const auto at=offset+j;
            const auto native=(at/f.width)*f.width+f.width-1-at%f.width;
            if(Writing)to[native]=input[j];else output[j]=from[native];}
        if(Writing)input+=count;else output+=count;
        n-=count;offset=0;if(!n)return true;
    }
    return n==0;
}
}
bool valid_binding(const Binding&b)noexcept{
    return b.identity.uid&&b.identity.boot_generation&&b.identity.system&&b.identity.component&&
        b.leg_index>=1&&b.leg_index<=5&&nonzero(b.execution_session_sha256)&&nonzero(b.task_sha256)&&
        nonzero(b.configuration_sha256)&&nonzero(b.reference_asset_sha256);
}
bool matches_window(const Binding&b,const Window&w)noexcept{
    if(!valid_binding(b)||!w.window_generation||w.schema!=1||w.capacity!=256||w.leg_index!=b.leg_index)return false;
    for(std::size_t i=0;i<32;++i)if(w.reference_asset_sha256[i]!=hash_byte(b.reference_asset_sha256,i))return false;
    return true; // actual generated validator, NOT this wire codec, checks math
}
void encode_manifest(const Binding&b,std::uint64_t generation,std::uint8_t out[manifest_bytes])noexcept{
    gpenmpc_snapshot_wire::Writer w(out);w.byte('R');w.byte('W');w.byte('W');w.byte('1');
    w.u64(b.identity.uid);w.u64(b.identity.boot_generation);w.byte(b.identity.system);w.byte(b.identity.component);
    for(auto x:b.execution_session_sha256)w.u32(x);
    for(auto x:b.task_sha256)w.u32(x);
    for(auto x:b.configuration_sha256)w.u32(x);
    for(auto x:b.reference_asset_sha256)w.u32(x);
    w.u32(b.leg_index);w.u64(generation);
}
bool copy_window_to_wire(const Window&w,std::size_t offset,std::uint8_t*out,std::size_t n)noexcept{
    return transfer<false>(&w,nullptr,offset,out,nullptr,n);
}
bool copy_wire_to_window(const std::uint8_t*in,std::size_t n,std::size_t offset,Window&w)noexcept{
    return transfer<true>(nullptr,&w,offset,nullptr,in,n);
}
Encoder::Encoder(const Binding&b,const Window&w)noexcept:window_(&w),generation_(w.window_generation){
    if(!matches_window(b,w))return;
    encode_manifest(b,generation_,manifest_);gpenmpc_consumption::CanonicalSha256 sha;
    for(auto v:manifest_)sha.byte(v);
    std::uint8_t buffer[64]{};
    for(std::size_t offset=0;offset<window_bytes;offset+=sizeof buffer){
        const auto n=(window_bytes-offset)<sizeof buffer?(window_bytes-offset):sizeof buffer;
        if(!copy_window_to_wire(w,offset,buffer,n))return;
        for(std::size_t j=0;j<n;++j)sha.byte(buffer[j]);
    }
    digest_=sha.finish();valid_=true;
}
bool Encoder::fragment(std::size_t index,Fragment&out)const noexcept{
    out={};if(!valid_||index>=fragment_count)return false;
    out.payload[0]=static_cast<std::uint8_t>((schema<<4)|(index%fragments_per_block));
    for(unsigned j=0;j<8;++j)out.payload[1+j]=static_cast<std::uint8_t>(generation_>>(56-8*j));
    out.payload[9]=static_cast<std::uint8_t>(index/fragments_per_block);
    const auto offset=index*data_per_fragment;
    const auto n=(message_bytes-offset)<data_per_fragment?(message_bytes-offset):data_per_fragment;
    for(std::size_t j=0;j<n;){const auto at=offset+j;
        if(at<manifest_bytes){out.payload[10+j]=manifest_[at];++j;}
        else if(at<manifest_bytes+window_bytes){
            const auto left=manifest_bytes+window_bytes-at;const auto count=(n-j)<left?(n-j):left;
            if(!copy_window_to_wire(*window_,at-manifest_bytes,out.payload+10+j,count))return false;
            j+=count;
        }else {out.payload[10+j]=hash_byte(digest_,at-manifest_bytes-window_bytes);++j;}
    }
    out.length=static_cast<std::uint8_t>(n+10);return true;
}
}
