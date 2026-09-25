#pragma once
// Schema 2 assembler using TUNNEL ingress and first-fault/caller-HRT semantics.
#include "SlimSnapshotExecutor.hpp"
// Scope GCC's formatting diagnostic to the compact schema-1 implementation.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#endif
#include "../px4_full_inner/argument_transport/CanonicalArgumentTransport.hpp"
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

namespace gpenmpc_slim_transport {
using namespace gpenmpc_argument_transport;
constexpr std::size_t slim_abi_size=gpenmpc_rfly_slim::encoded_size;
constexpr std::size_t slim_message_size=binding_size+slim_abi_size+32;
constexpr std::size_t slim_fragment_count=4;
constexpr std::uint8_t slim_schema_version=2;
static_assert(slim_message_size==381 && (slim_message_size+fragment_data_size-1)/fragment_data_size==4,"schema2 fixed381/four");
static_assert(slim_fragment_count<=gpenmpc_ingress::queue_capacity,"ingress queue capacity must match the schema");
struct SlimMessage {std::uint8_t bytes[slim_message_size]{};};
inline std::size_t slim_chunk_size(std::size_t index)noexcept{
    const auto remaining=slim_message_size-index*fragment_data_size;
    return remaining<fragment_data_size?remaining:fragment_data_size;
}
inline bool encode_slim(const Metadata&m,const std::uint8_t*abi,std::size_t size,SlimMessage&out)noexcept{
    out={};if(!abi||size!=slim_abi_size||!valid(m))return false;
    auto*p=out.bytes;put64(p,m.command_generation);copy_bytes(m.snapshot_ticket.data(),32,p+8);
    put64(p+40,m.reference_generation);put64(p+48,m.outer_generation);
    copy_bytes(m.configuration_sha256.data(),32,p+56);copy_bytes(abi,slim_abi_size,p+binding_size);
    const auto sha=digest(p,binding_size+slim_abi_size);copy_bytes(sha.data(),32,p+binding_size+slim_abi_size);return true;
}
inline bool fragment_slim(const SlimMessage&m,std::size_t index,Fragment&out)noexcept{
    out={};if(index>=slim_fragment_count||!get64(m.bytes))return false;
    const auto offset=index*fragment_data_size,n=slim_chunk_size(index);
    out.payload[0]=static_cast<std::uint8_t>((slim_schema_version<<4)|index);put64(out.payload+1,get64(m.bytes));
    copy_bytes(m.bytes+offset,n,out.payload+fragment_header_size);
    out.length=static_cast<std::uint8_t>(fragment_header_size+n);return true;
}
// Transport framing. The snapshot decoder validates state and parameter identity.
inline bool valid_slim_structure(const std::uint8_t*p)noexcept{
    if(p[0]!='R'||p[1]!='K'||p[2]!='S'||p[3]!='1')return false;
    gpenmpc_kernel_abi::Reader r(p+4);
    for(unsigned j=0;j<15;++j)if(!std::isfinite(r.real()))return false;
    if(r.byte()>1)return false;
    for(unsigned j=0;j<15;++j)if(!std::isfinite(r.real()))return false;
    return true; // the two exact uint64 generations remain in the payload
}
struct SlimTrace {
    std::uint64_t original_hrt[slim_fragment_count]{},reception_sequence[slim_fragment_count]{},uorb_generation[slim_fragment_count]{};
    std::uint8_t mavlink_sequence[slim_fragment_count]{},wire_payload_length[slim_fragment_count]{},meaningful_length[slim_fragment_count]{};
    std::uint64_t expectation_created_hrt{},first_original_hrt{},last_original_hrt{},completed_hrt{};
    std::uint8_t source_system{},source_component{},target_system{},target_component{},receiver_instance{};
};
struct SlimCompleted {
    Metadata metadata{};std::uint8_t abi[slim_abi_size]{};Bytes32 abi_sha256{},message_sha256{};SlimTrace trace{};
    bool complete{false},control_authority{false};
};
class SlimAssembler final {
public:
    explicit SlimAssembler(Configuration c)noexcept:config_(c){
        if(!c.source_system||!c.source_component||!c.target_system||!c.target_component||!c.max_assembly_us)fail(Fault::Configuration,0);
    }
    bool expect(const Metadata&expected,std::uint64_t board_now)noexcept{
        return expect_from_original_event(expected,board_now,board_now);
    }
    bool expect_from_original_event(const Metadata&expected,std::uint64_t original_begin,std::uint64_t board_now)noexcept{
        if(failed())return false;
        if(active_||ready_)return fail(Fault::Pending,board_now);
        if(!valid(expected)||!board_now||!original_begin)return fail(Fault::Metadata,board_now);
        if(expected.command_generation<=last_command_)return fail(Fault::Replay,board_now);
        if(board_now<last_now_||original_begin>board_now||original_begin<last_original_)return fail(Fault::Timestamp,board_now);
        expected_=expected;active_=true;next_=0;created_=original_begin;last_now_=board_now;buffer_={};completed_={};
        completed_.trace.expectation_created_hrt=original_begin;return tick(board_now);
    }
    bool tick(std::uint64_t board_now)noexcept{
        if(failed())return false;
        if(board_now<last_now_)return fail(Fault::Timestamp,board_now);
        last_now_=board_now;
        if((active_||ready_)&&(board_now<created_||board_now-created_>config_.max_assembly_us))return fail(Fault::Expired,board_now);
        return true;
    }
    bool receive(const Arrival&a,std::uint64_t board_now)noexcept{
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
        const auto version=f.payload[0]>>4;const auto index=static_cast<std::size_t>(f.payload[0]&15);
        if(version!=slim_schema_version)return fail(Fault::UnknownSchema,board_now);
        if(index>=slim_fragment_count||index!=next_)return fail(Fault::Index,board_now);
        const auto n=slim_chunk_size(next_);
        if(f.payload_length!=n+fragment_header_size||f.wire_payload_length<5||f.wire_payload_length>133)return fail(Fault::Length,board_now);
        if(f.wire_payload_length>5+f.payload_length)return fail(Fault::Padding,board_now);
        for(std::size_t k=f.payload_length;k<128;++k)if(f.payload[k])return fail(Fault::Padding,board_now);
        if(get64(f.payload+1)!=expected_.command_generation)return fail(Fault::Interleaved,board_now);
        copy_bytes(f.payload+fragment_header_size,n,buffer_.bytes+next_*fragment_data_size);
        auto&t=completed_.trace;t.original_hrt[next_]=f.timestamp;t.reception_sequence[next_]=f.reception_sequence;
        t.uorb_generation[next_]=a.uorb_generation;t.mavlink_sequence[next_]=f.mavlink_sequence;
        t.wire_payload_length[next_]=f.wire_payload_length;t.meaningful_length[next_]=f.payload_length;
        if(!next_){t.first_original_hrt=f.timestamp;t.source_system=f.source_system;t.source_component=f.source_component;
            t.target_system=f.target_system;t.target_component=f.target_component;t.receiver_instance=f.receiver_instance;}
        t.last_original_hrt=f.timestamp;last_sequence_=f.reception_sequence;last_uorb_=a.uorb_generation;last_original_=f.timestamp;++next_;
        if(next_!=slim_fragment_count)return true;
        const auto*p=buffer_.bytes;const auto sha=digest(p,binding_size+slim_abi_size);
        for(std::size_t k=0;k<32;++k)if(sha[k]!=p[binding_size+slim_abi_size+k])return fail(Fault::Integrity,board_now);
        Metadata actual{};actual.command_generation=get64(p);copy_bytes(p+8,32,actual.snapshot_ticket.data());
        actual.reference_generation=get64(p+40);actual.outer_generation=get64(p+48);copy_bytes(p+56,32,actual.configuration_sha256.data());
        if(!(actual==expected_))return fail(Fault::Metadata,board_now);
        const auto abi_sha=digest(p+binding_size,slim_abi_size);
        if(!valid_slim_structure(p+binding_size))return fail(Fault::AbiInvalid,board_now);
        completed_.metadata=actual;copy_bytes(p+binding_size,slim_abi_size,completed_.abi);
        completed_.abi_sha256=abi_sha;completed_.message_sha256=sha;completed_.trace.completed_hrt=board_now;
        completed_.complete=true;ready_=true;active_=false;last_command_=actual.command_generation;return true;
    }
    bool take(SlimCompleted&out,std::uint64_t board_now)noexcept{
        out={};if(!tick(board_now)||!ready_)return false;out=completed_;ready_=false;return true;
    }
    // Shared ingress queue: a complete schema4 context batch may precede a
    // numeric batch, never interrupt it. Observe ACTUAL sequence/HRT/uORB data;
    // do not renumber, reset the assembler, or wash any fault on this path.
    bool observe_context(const Arrival&a,std::uint64_t board_now)noexcept{
        if(!tick(board_now))return false;
        if(active_||ready_)return fail(Fault::Interleaved,board_now);
        const auto&f=a.fields;
        if(a.ingress_first_fault||a.ingress_last_fault||a.ingress_first_fault_hrt||a.ingress_last_fault_hrt||
           a.ingress_rejected_total||a.ingress_queue_overflows||a.ingress_publication_failures)return fail(Fault::IngressFault,board_now);
        if(f.source_system!=config_.source_system||f.source_component!=config_.source_component||
           f.target_system!=config_.target_system||f.target_component!=config_.target_component||
           f.receiver_instance!=config_.receiver_instance||f.payload_type!=gpenmpc_ingress::payload_type)return fail(Fault::Source,board_now);
        if(!f.timestamp||f.timestamp>board_now||f.timestamp<last_original_)return fail(Fault::Timestamp,board_now);
        if(!f.reception_sequence||(last_sequence_&&f.reception_sequence!=last_sequence_+1))return fail(Fault::SequenceGap,board_now);
        if(!a.uorb_generation||(last_uorb_&&a.uorb_generation!=last_uorb_+1))return fail(Fault::UorbGap,board_now);
        if((f.payload[0]>>4)!=4)return fail(Fault::UnknownSchema,board_now);
        last_sequence_=f.reception_sequence;last_uorb_=a.uorb_generation;last_original_=f.timestamp;
        last_arrival_=a;have_arrival_=true;return true;
    }
    bool ready()const noexcept{return ready_&&!failed();}
    bool failed()const noexcept{return first_fault_!=Fault::None;}
    Fault fault()const noexcept{return first_fault_;}
    std::uint64_t fault_hrt()const noexcept{return first_fault_hrt_;}
    std::size_t received_fragments()const noexcept{return next_;}
    const Arrival*last_arrival()const noexcept{return have_arrival_?&last_arrival_:nullptr;}
private:
    bool fail(Fault why,std::uint64_t now)noexcept{
        if(!failed()){first_fault_=why;first_fault_hrt_=now;}active_=ready_=false;return false;
    }
    Configuration config_{};Metadata expected_{};SlimMessage buffer_{};SlimCompleted completed_{};Arrival last_arrival_{};
    std::uint64_t created_{},last_now_{},last_sequence_{},last_uorb_{},last_original_{},last_command_{},first_fault_hrt_{};
    std::size_t next_{};Fault first_fault_{Fault::None};bool active_{},ready_{},have_arrival_{};
};

// Bind an assembled numerical packet to already admitted reference/outer
// context. No HOST state, source timestamp, or configuration copy is accepted.
inline bool bind_command(const SlimCompleted&done,const gpenmpc_consumption::Reference&reference,
    const gpenmpc_consumption::OuterCommand&outer,const gpenmpc_rfly_execution::Configuration&approved,
    gpenmpc_rfly_slim::Command&out)noexcept{
    out={};if(!done.complete||done.control_authority||!done.trace.last_original_hrt||
       done.metadata.configuration_sha256!=bytes_of(approved.configuration_payload_sha256)||
       reference.generation!=done.metadata.reference_generation||reference.outer_generation!=done.metadata.outer_generation||
       outer.generation!=done.metadata.outer_generation||!(reference.identity==approved.identity)||!(outer.identity==approved.identity))return false;
    out.snapshot_ticket=done.metadata.snapshot_ticket;copy_bytes(done.abi,slim_abi_size,out.arguments.data());
    out.argument_sha256=words_of(done.abi_sha256);out.configuration_sha256=words_of(done.metadata.configuration_sha256);
    out.matlab_source_sha256=approved.matlab_extraction_source_sha256;out.generated_source_sha256=approved.kernel_source_sha256;
    out.wrapper_matlab_source_sha256=approved.wrapper_matlab_source_sha256;
    out.reference=reference;out.outer=outer;out.original_board_ingress_us=done.trace.last_original_hrt;return true;
}
} // namespace gpenmpc_slim_transport
