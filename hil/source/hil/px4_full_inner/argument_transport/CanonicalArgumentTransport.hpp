#pragma once
#include "../argument_abi/CanonicalKernelArgumentCodec.hpp"
#include "../px4_ingress/GPENMPCFullInnerIngress.hpp"
#include "../portable/CanonicalPortable.hpp"

namespace gpenmpc_argument_transport {
using Bytes32=gpenmpc_portable::Array<std::uint8_t,32>;
constexpr std::size_t abi_size=gpenmpc_kernel_abi::encoded_size;
constexpr std::size_t binding_size=88, fragment_header_size=9, fragment_data_size=119;
constexpr std::size_t message_size=binding_size+abi_size+32, fragment_count=8;
constexpr std::uint8_t schema_version=1;
static_assert(message_size==949 && (message_size+fragment_data_size-1)/fragment_data_size==8,"fixed schema");
static_assert(fragment_count==gpenmpc_ingress::queue_capacity,"one full message fits only an otherwise empty eight-slot queue");
struct Metadata {
    std::uint64_t command_generation{},reference_generation{},outer_generation{};
    Bytes32 snapshot_ticket{},configuration_sha256{};
};
inline bool operator==(const Metadata&a,const Metadata&b) noexcept {
    return a.command_generation==b.command_generation && a.reference_generation==b.reference_generation &&
        a.outer_generation==b.outer_generation && a.snapshot_ticket==b.snapshot_ticket && a.configuration_sha256==b.configuration_sha256;
}
inline bool nonzero(const Bytes32&a) noexcept {for(auto v:a)if(v)return true;return false;}
inline bool valid(const Metadata&m) noexcept {return m.command_generation&&m.reference_generation&&m.outer_generation&&nonzero(m.snapshot_ticket)&&nonzero(m.configuration_sha256);}
inline void put64(std::uint8_t*p,std::uint64_t x) noexcept {for(unsigned k=0;k<8;++k)p[k]=static_cast<std::uint8_t>(x>>(56-8*k));}
inline std::uint64_t get64(const std::uint8_t*p) noexcept {std::uint64_t x=0;for(unsigned k=0;k<8;++k)x=(x<<8)|p[k];return x;}
inline Bytes32 bytes_of(const gpenmpc_portable::Array<std::uint32_t,8>&d) noexcept {
    Bytes32 x{};for(unsigned k=0;k<32;++k)x[k]=static_cast<std::uint8_t>(d[k/4]>>(24-8*(k%4)));return x;
}
inline gpenmpc_portable::Array<std::uint32_t,8> words_of(const Bytes32&d) noexcept {
    gpenmpc_portable::Array<std::uint32_t,8>x{};for(unsigned k=0;k<32;++k)x[k/4]=(x[k/4]<<8)|d[k];return x;
}
inline Bytes32 digest(const std::uint8_t*p,std::size_t n) noexcept {
    gpenmpc_consumption::CanonicalSha256 h;for(std::size_t k=0;k<n;++k)h.byte(p[k]);return bytes_of(h.finish());
}
// The 32-byte ticket/digest uses the existing core's array ABI. New transport
// storage is fixed C arrays; no additional standard-library container support.
struct Message {std::uint8_t bytes[message_size]{};};
struct Fragment {std::uint8_t payload[128]{};std::uint8_t length{};};
inline void copy_bytes(const std::uint8_t*src,std::size_t n,std::uint8_t*dst) noexcept {for(std::size_t k=0;k<n;++k)dst[k]=src[k];}
inline std::size_t chunk_size(std::size_t index) noexcept {const auto remaining=message_size-index*fragment_data_size;return remaining<fragment_data_size?remaining:fragment_data_size;}
inline bool encode(const Metadata&m,const std::uint8_t*abi,std::size_t size,Message&out) noexcept {
    out={};if(!abi||size!=abi_size||!valid(m))return false;
    auto*p=out.bytes;put64(p,m.command_generation);copy_bytes(&m.snapshot_ticket[0],32,p+8);
    put64(p+40,m.reference_generation);put64(p+48,m.outer_generation);
    copy_bytes(&m.configuration_sha256[0],32,p+56);copy_bytes(abi,abi_size,p+binding_size);
    const auto sha=digest(p,binding_size+abi_size);copy_bytes(&sha[0],32,p+binding_size+abi_size);return true;
}
inline bool fragment(const Message&m,std::size_t index,Fragment&out) noexcept {
    out={};if(index>=fragment_count||!get64(m.bytes))return false;
    const auto offset=index*fragment_data_size,n=chunk_size(index);
    out.payload[0]=static_cast<std::uint8_t>((schema_version<<4)|index);put64(out.payload+1,get64(m.bytes));
    copy_bytes(m.bytes+offset,n,out.payload+fragment_header_size);
    out.length=static_cast<std::uint8_t>(fragment_header_size+n);return true;
}
enum class Fault:std::uint8_t {None,Configuration,Pending,NoExpectation,Metadata,UnknownSchema,Index,Length,Padding,
    Source,IngressFault,SequenceGap,UorbGap,Timestamp,Expired,Interleaved,Integrity,AbiInvalid,Replay};
struct Configuration {
    std::uint8_t source_system{},source_component{},target_system{},target_component{},receiver_instance{};
    // Explicit local engineering deadline, not a controller dt or authority gate.
    std::uint64_t max_assembly_us{};
};
struct Arrival {
    gpenmpc_ingress::Fields fields{};
    std::uint64_t uorb_generation{};
    std::uint8_t ingress_first_fault{},ingress_last_fault{};
    std::uint64_t ingress_first_fault_hrt{},ingress_last_fault_hrt{},ingress_rejected_total{},ingress_queue_overflows{},ingress_publication_failures{};
};
template<class Topic> Arrival from_topic(const Topic&t,std::uint64_t actual_uorb_generation) noexcept {
    Arrival a{};auto&f=a.fields;f.timestamp=t.timestamp;f.reception_sequence=t.reception_sequence;
    f.receiver_instance=t.receiver_instance;f.source_system=t.source_system;f.source_component=t.source_component;
    f.target_system=t.target_system;f.target_component=t.target_component;f.mavlink_sequence=t.mavlink_sequence;
    f.wire_payload_length=t.wire_payload_length;f.payload_type=t.payload_type;f.payload_length=t.payload_length;
    for(std::size_t k=0;k<128;++k){f.payload[k]=t.payload[k];}
    a.uorb_generation=actual_uorb_generation;
    a.ingress_first_fault=t.ingress_first_fault;a.ingress_last_fault=t.ingress_last_fault;
    a.ingress_first_fault_hrt=t.ingress_first_fault_hrt;a.ingress_last_fault_hrt=t.ingress_last_fault_hrt;
    a.ingress_rejected_total=t.ingress_rejected_total;a.ingress_queue_overflows=t.ingress_queue_overflows;
    a.ingress_publication_failures=t.ingress_publication_failures;return a;
}
struct Trace {
    std::uint64_t original_hrt[fragment_count]{},reception_sequence[fragment_count]{},uorb_generation[fragment_count]{};
    std::uint8_t mavlink_sequence[fragment_count]{},wire_payload_length[fragment_count]{},meaningful_length[fragment_count]{};
    std::uint64_t expectation_created_hrt{},first_original_hrt{},last_original_hrt{},completed_hrt{};
    std::uint8_t source_system{},source_component{},target_system{},target_component{},receiver_instance{};
};
struct Completed {
    Metadata metadata{};std::uint8_t abi[abi_size]{};Bytes32 abi_sha256{},message_sha256{};Trace trace{};
    bool complete{false},control_authority{false};
};
class Assembler final {
public:
    explicit Assembler(Configuration c) noexcept:config_(c){
        if(!c.source_system||!c.source_component||!c.target_system||!c.target_component||!c.max_assembly_us)fail(Fault::Configuration,0);
    }
    // Expected ticket is supplied by the board-owned snapshot lookup, not by
    // the incoming host packet. This helper cannot independently verify it.
    bool expect(const Metadata&expected,std::uint64_t board_now) noexcept {
        if(failed())return false;
        if(active_||ready_)return fail(Fault::Pending,board_now);
        if(!valid(expected)||!board_now)return fail(Fault::Metadata,board_now);
        if(expected.command_generation<=last_command_)return fail(Fault::Replay,board_now);
        if(board_now<last_now_)return fail(Fault::Timestamp,board_now);
        expected_=expected;active_=true;next_=0;created_=board_now;last_now_=board_now;buffer_={};completed_={};
        completed_.trace.expectation_created_hrt=board_now;return true;
    }
    bool tick(std::uint64_t board_now) noexcept {
        if(failed())return false;
        if(board_now<last_now_)return fail(Fault::Timestamp,board_now);
        last_now_=board_now;
        if((active_||ready_)&&(board_now<created_||board_now-created_>config_.max_assembly_us))return fail(Fault::Expired,board_now);
        return true;
    }
    bool receive(const Arrival&a,std::uint64_t board_now) noexcept {
        if(!tick(board_now))return false;
        last_arrival_=a;have_arrival_=true;
        if(!active_||ready_)return fail(ready_?Fault::Pending:Fault::NoExpectation,board_now);
        const auto&f=a.fields;
        if(a.ingress_first_fault||a.ingress_last_fault||a.ingress_first_fault_hrt||a.ingress_last_fault_hrt||
           a.ingress_rejected_total||a.ingress_queue_overflows||a.ingress_publication_failures)return fail(Fault::IngressFault,board_now);
        if(f.source_system!=config_.source_system||f.source_component!=config_.source_component||
           f.target_system!=config_.target_system||f.target_component!=config_.target_component||
           f.receiver_instance!=config_.receiver_instance||f.payload_type!=gpenmpc_ingress::payload_type)return fail(Fault::Source,board_now);
        if(!f.timestamp||f.timestamp<created_||f.timestamp>board_now||f.timestamp<last_original_)return fail(Fault::Timestamp,board_now);
        if(!f.reception_sequence||(last_sequence_&&f.reception_sequence!=last_sequence_+1))return fail(Fault::SequenceGap,board_now);
        if(!a.uorb_generation||(last_uorb_&&a.uorb_generation!=last_uorb_+1))return fail(Fault::UorbGap,board_now);
        const auto version=f.payload[0]>>4;
        const auto index=static_cast<std::size_t>(f.payload[0]&15);
        if(version!=schema_version)return fail(Fault::UnknownSchema,board_now);
        if(index>=fragment_count||index!=next_)return fail(Fault::Index,board_now);
        const auto n=chunk_size(next_);
        if(f.payload_length!=n+fragment_header_size||f.wire_payload_length<5||f.wire_payload_length>133)return fail(Fault::Length,board_now);
        // A MAVLink2 nonzero suffix beyond the meaningful final payload cannot
        // be hidden by ingress zero-fill: it extends original wire length.
        if(f.wire_payload_length>5+f.payload_length)return fail(Fault::Padding,board_now);
        for(std::size_t k=f.payload_length;k<128;++k)if(f.payload[k])return fail(Fault::Padding,board_now);
        if(get64(&f.payload[1])!=expected_.command_generation)return fail(Fault::Interleaved,board_now);
        copy_bytes(&f.payload[fragment_header_size],n,buffer_.bytes+next_*fragment_data_size);
        auto&t=completed_.trace;t.original_hrt[next_]=f.timestamp;t.reception_sequence[next_]=f.reception_sequence;
        t.uorb_generation[next_]=a.uorb_generation;t.mavlink_sequence[next_]=f.mavlink_sequence;
        t.wire_payload_length[next_]=f.wire_payload_length;t.meaningful_length[next_]=f.payload_length;
        if(!next_){t.first_original_hrt=f.timestamp;t.source_system=f.source_system;t.source_component=f.source_component;
            t.target_system=f.target_system;t.target_component=f.target_component;t.receiver_instance=f.receiver_instance;}
        t.last_original_hrt=f.timestamp;last_sequence_=f.reception_sequence;last_uorb_=a.uorb_generation;last_original_=f.timestamp;++next_;
        if(next_!=fragment_count)return true;
        const auto*p=buffer_.bytes;const auto sha=digest(p,binding_size+abi_size);
        for(std::size_t k=0;k<32;++k)if(sha[k]!=p[binding_size+abi_size+k])return fail(Fault::Integrity,board_now);
        Metadata actual{};actual.command_generation=get64(p);copy_bytes(p+8,32,&actual.snapshot_ticket[0]);
        actual.reference_generation=get64(p+40);actual.outer_generation=get64(p+48);copy_bytes(p+56,32,&actual.configuration_sha256[0]);
        if(!(actual==expected_))return fail(Fault::Metadata,board_now);
        const auto abi_sha=digest(p+binding_size,abi_size);
        gpenmpc_kernel_abi::DecodeFailure decode_failure{};
        // Fixed object scratch avoids exposing partially decoded kernel inputs.
        if(!gpenmpc_kernel_abi::decode(p+binding_size,abi_size,words_of(abi_sha),decoded_,decode_failure))return fail(Fault::AbiInvalid,board_now);
        completed_.metadata=actual;copy_bytes(p+binding_size,abi_size,completed_.abi);
        completed_.abi_sha256=abi_sha;completed_.message_sha256=sha;completed_.trace.completed_hrt=board_now;
        completed_.complete=true;ready_=true;active_=false;last_command_=actual.command_generation;return true;
    }
    bool take(Completed&out,std::uint64_t board_now) noexcept {
        out={};if(!tick(board_now)||!ready_)return false;out=completed_;ready_=false;return true;
    }
    bool ready() const noexcept {return ready_&&!failed();}
    bool failed() const noexcept {return first_fault_!=Fault::None;}
    Fault fault() const noexcept {return first_fault_;}
    std::uint64_t fault_hrt() const noexcept {return first_fault_hrt_;}
    std::size_t received_fragments() const noexcept {return next_;}
    const Arrival*last_arrival() const noexcept {return have_arrival_?&last_arrival_:nullptr;}
private:
    bool fail(Fault why,std::uint64_t now) noexcept {if(!failed()){first_fault_=why;first_fault_hrt_=now;}active_=ready_=false;return false;}
    Configuration config_{};Metadata expected_{};Message buffer_{};Completed completed_{};
    gpenmpc_consumption::KernelArguments decoded_{};Arrival last_arrival_{};
    std::uint64_t created_{},last_now_{},last_sequence_{},last_uorb_{},last_original_{},last_command_{},first_fault_hrt_{};
    std::size_t next_{};Fault first_fault_{Fault::None};bool active_{},ready_{},have_arrival_{};
};
} // namespace gpenmpc_argument_transport
