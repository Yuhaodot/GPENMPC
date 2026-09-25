#include "CanonicalLocalWindowAssembler.hpp"
#include <cstring>

namespace gpenmpc_local_window_wire {
Assembler::Assembler(const Configuration&c)noexcept:configuration_(c){
    if(!valid_binding(c.expected)||!c.max_assembly_us)fail(Fault::Configuration,0);
}
bool Assembler::fail(Fault reason,std::uint64_t now)noexcept{
    if(!failed()){d_.first_fault=reason;d_.first_fault_processing_us=now;
        if(inside_arrival_){fault_arrival_=*inside_arrival_;have_fault_arrival_=true;}}
    active_=ready_=false;return false;
}
bool Assembler::tick(std::uint64_t now)noexcept{
    if(failed())return false;
    if(!now||now<d_.last_processing_us)return fail(Fault::Clock,now);
    d_.last_processing_us=now;
    // Owner-supplied window-transfer age bound.
    if((active_||ready_)&&(now<d_.first_original_arrival_us||now-d_.first_original_arrival_us>configuration_.max_assembly_us))
        return fail(Fault::Expired,now);
    return true;
}
bool Assembler::accept_bytes(const std::uint8_t*data,std::size_t n)noexcept{
    for(std::size_t j=0;j<n;){const auto at=bytes_received_+j;
        if(at<manifest_bytes){if(data[j]!=manifest_[at])return false;sha_.byte(data[j]);++j;}
        else if(at<manifest_bytes+window_bytes){
            const auto left=manifest_bytes+window_bytes-at;const auto count=(n-j)<left?(n-j):left;
            if(!copy_wire_to_window(data+j,count,at-manifest_bytes,window_))return false;
            for(std::size_t k=0;k<count;++k)sha_.byte(data[j+k]);
            j+=count;
        }else {received_hash_[at-manifest_bytes-window_bytes]=data[j];++j;}
    }
    bytes_received_+=n;return true;
}
bool Assembler::receive(const Arrival&a,std::uint64_t now)noexcept{
    if(failed())return false;
    inside_arrival_=&a;
    // All exits clear the temporary pointer, including first-fault retention.
    struct Clear {const Arrival*&p;~Clear(){p=nullptr;}} clear{inside_arrival_};
    if(!tick(now))return false;
    if(ready_)return fail(Fault::Busy,now);
    const auto&f=a.fields;
    if(!f.timestamp||f.timestamp>now||f.timestamp<d_.last_original_arrival_us)return fail(Fault::Clock,now);
    if((f.payload[0]>>4)!=schema)return fail(Fault::Schema,now);
    if(f.payload_length<10||f.payload_length>128)return fail(Fault::Length,now);
    for(unsigned j=f.payload_length;j<128;++j)if(f.payload[j])return fail(Fault::Padding,now);
    const auto generation=gpenmpc_argument_transport::get64(f.payload+1);
    if(!active_){
        if((f.payload[0]&15)||f.payload[9])return fail(Fault::Index,now);
        if(!generation||generation<=d_.highest_started_generation)return fail(Fault::Generation,now);
        if(now-f.timestamp>configuration_.max_assembly_us)return fail(Fault::Expired,now);
        generation_=generation;d_.highest_started_generation=generation;
        d_.first_original_arrival_us=f.timestamp;d_.completed_processing_us=0;
        encode_manifest(configuration_.expected,generation,manifest_);
        sha_=gpenmpc_consumption::CanonicalSha256{};bytes_received_=next_=0;
        // Every wire field is overwritten before ready. Previous raw staging
        // remains auditable after a partial failure; no blanket new clock/reset.
        std::memset(received_hash_,0,sizeof received_hash_);active_=true;
    }
    if(generation!=generation_)return fail(Fault::Generation,now);
    if(next_>=fragment_count||(f.payload[0]&15)!=next_%fragments_per_block||f.payload[9]!=next_/fragments_per_block)return fail(Fault::Index,now);
    const auto offset=next_*data_per_fragment;
    const auto n=(message_bytes-offset)<data_per_fragment?(message_bytes-offset):data_per_fragment;
    if(f.payload_length!=n+10)return fail(Fault::Length,now);
    if(!accept_bytes(f.payload+10,n))return fail(Fault::BindingMismatch,now);
    ++next_;++d_.received_fragments;d_.last_original_arrival_us=f.timestamp;
    if(next_!=fragment_count)return true;
    const auto hash=sha_.finish();
    for(std::size_t j=0;j<32;++j)if(received_hash_[j]!=static_cast<std::uint8_t>(hash[j/4]>>(24-8*(j%4))))return fail(Fault::Integrity,now);
    if(bytes_received_!=message_bytes||window_.window_generation!=generation_||!matches_window(configuration_.expected,window_))return fail(Fault::BindingMismatch,now);
    active_=false;ready_=true;d_.completed_processing_us=now;++d_.completed_windows;return true;
}
const Window*Assembler::ready_window(std::uint64_t now)noexcept{return tick(now)&&ready_?&window_:nullptr;}
bool Assembler::release(std::uint64_t generation)noexcept{
    if(failed())return false;
    if(!ready_||generation!=generation_)return fail(Fault::Release,d_.last_processing_us);
    ready_=false;d_.released_generation=generation;++d_.released_windows;return true;
}
void Assembler::stop(std::uint64_t now)noexcept{fail(Fault::Stopped,now);}
}
