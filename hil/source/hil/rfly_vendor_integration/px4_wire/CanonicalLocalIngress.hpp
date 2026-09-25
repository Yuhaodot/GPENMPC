#pragma once
#include "CanonicalLocalGpWire.hpp"
#include "CanonicalLocalWindowAssembler.hpp"
#include "CanonicalLocalTaskWire.hpp"
#include "../../px4_full_inner/argument_transport/CanonicalArgumentTransport.hpp"

namespace gpenmpc_local_ingress {
using Arrival=gpenmpc_argument_transport::Arrival;
using Configuration=gpenmpc_argument_transport::Configuration;
using RequestBytes=gpenmpc_local_gp_wire::RequestBytes;
using ReplyBytes=gpenmpc_local_gp_wire::ReplyBytes;
enum class Fault:std::uint8_t {None,BadConfiguration,Pending,NoExpectation,Request,
    Source,IngressFault,SequenceGap,UorbGap,CounterOverflow,Timestamp,Expired,
    UnknownSchema,Index,Length,Padding,Interleaved,Integrity,Replay,Stopped,Window,TaskInput};
struct Completed {
    ReplyBytes bytes{};
    gpenmpc_local_gp_wire::Hash original_request_sha256{};
    Arrival fragments[gpenmpc_local_gp_wire::fragment_count]{};
    std::uint64_t original_request_ready_us{},first_original_arrival_us{},last_original_arrival_us{},completed_processing_us{};
    bool complete{},control_authority{},source_freshness_granted{};
};
struct Diagnostics {
    Fault first_fault{Fault::None};std::uint64_t first_fault_processing_us{};
    std::uint64_t receive_calls{},envelopes_admitted{},requests{},completed{},taken{},calls_after_fault{};
    std::uint64_t last_reception_sequence{},last_uorb_generation{},last_original_arrival_us{},last_processing_us{};
    std::uint64_t first_reception_sequence{},first_uorb_generation{};
    bool unknown_prebaseline{true};
};
// One actual subscriber/owner serializes all methods. It supplies Arrival only
// via from_topic(actual_topic, actual_subscription.get_last_generation()). No
// method accepts substitute counters, synthesizes arrivals or clears a fault.
// The same receive()->validate_envelope()->dispatch_validated() path is the
// extension point for future concrete outer/window handlers on THIS queue.
// There is deliberately no public skip-schema or already-validated bypass.
class CanonicalLocalIngress final {
public:
    static constexpr std::size_t fragments=gpenmpc_local_gp_wire::fragment_count;
    explicit CanonicalLocalIngress(Configuration c,gpenmpc_local_window_wire::Assembler*window=nullptr,
        gpenmpc_local_task_wire::Receiver*task=nullptr,bool runtime=false)noexcept:configuration_(c),window_(window),task_(task),runtime_(runtime){
        if(!c.source_system||!c.source_component||!c.target_system||!c.target_component||!c.max_assembly_us)fail(Fault::BadConfiguration,0);
    }
    CanonicalLocalIngress(const CanonicalLocalIngress&)=delete;
    CanonicalLocalIngress&operator=(const CanonicalLocalIngress&)=delete;
    // Bytes and original_ready MUST come from the same actual Pending's
    // pending_bytes()/retained_actual_feedback().original_cycle_read_us. This
    // transport cannot manufacture or independently attest that runtime owner.
    bool expect_gp_reply(const RequestBytes&request,std::uint64_t original_ready,std::uint64_t now)noexcept{
        if(!tick(now))return false;
        if(active_||ready_||discarding_||expired_generation_)return fail(Fault::Pending,now);
        if(!gpenmpc_local_gp_wire::decode(request,request_))return fail(Fault::Request,now);
        if(request_.identity.system!=configuration_.target_system||request_.identity.component!=configuration_.target_component||
           (d_.requests&&!(request_.identity==session_identity_)))return fail(Fault::Request,now);
        if(!original_ready||original_ready<request_.original_publication_us||original_ready>now)return fail(Fault::Timestamp,now);
        if(request_.output_generation<=last_expected_output_||request_.source_generation<=last_expected_source_||request_.source_timestamp_ns<=last_expected_stamp_)return fail(Fault::Replay,now);
        if(now-original_ready>configuration_.max_assembly_us)return fail(Fault::Expired,now);
        expected_bytes_=request;expected_hash_=gpenmpc_local_gp_wire::digest(request.data(),request.size());
        session_identity_=request_.identity;
        last_expected_output_=request_.output_generation;last_expected_source_=request_.source_generation;last_expected_stamp_=request_.source_timestamp_ns;
        original_ready_=original_ready;next_=0;completed_={};completed_.original_request_ready_us=original_ready;
        completed_.original_request_sha256=expected_hash_;active_=true;++d_.requests;return true;
    }
    bool tick(std::uint64_t now,bool disarmed_observation=false)noexcept{
        if(failed()){++d_.calls_after_fault;return false;}
        if(!now||now<d_.last_processing_us)return fail(Fault::Timestamp,now);
        d_.last_processing_us=now;
        if(window_&&!window_->tick(now))return fail(Fault::Window,now);
        if(task_&&!task_->tick(now,disarmed_observation))return fail(Fault::TaskInput,now);
        // Owner-supplied transport assembly bound.
        if(discarding_&&now-completed_.first_original_arrival_us>configuration_.max_assembly_us)return fail(Fault::Expired,now);
        if((active_||ready_)&&(now<original_ready_||now-original_ready_>configuration_.max_assembly_us)){
            // Runtime GP already withdrew learning at its original 10-ms
            // deadline. A reply that started before transport retirement
            // must finish as history, never become a prediction. Preserve
            // its exact prefix and original first-arrival assembly deadline;
            // incomplete/corrupt/foreign history remains fatal below/above.
            if(!runtime_||now<original_ready_||ready_||discarding_)return fail(Fault::Expired,now);
            if(next_){
                if(!completed_.first_original_arrival_us||now<completed_.first_original_arrival_us||
                   now-completed_.first_original_arrival_us>configuration_.max_assembly_us)return fail(Fault::Expired,now);
                discarding_=true;
            }
            retired_request_=request_;have_retired_=true;
            expired_generation_=request_.output_generation;active_=false;
        }
        return true;
    }
    std::uint64_t take_unanswered_generation()noexcept{
        // Retain the request owner through complete CRC/request validation.
        // Inner control uses the unavailable branch while the reply is pending.
        if(discarding_)return 0;
        const auto value=expired_generation_;expired_generation_=0;return value;
    }
    bool receive(const Arrival&a,std::uint64_t now,bool disarmed_observation=false)noexcept{
        if(failed()){++d_.calls_after_fault;return false;}
        ++d_.receive_calls;last_arrival_=a;have_arrival_=true;inside_receive_=true;
        const bool accepted=tick(now,disarmed_observation)&&validate_envelope(a,now)&&dispatch_validated(a,now);
        inside_receive_=false;return accepted;
    }
    bool take_gp_reply(Completed&out,std::uint64_t now)noexcept{
        out={};if(!tick(now)||!ready_)return false;
        out=completed_;ready_=false;++d_.taken;return true;
    }
    // Raw first failure and every accepted fragment survive stop/timeout and
    // remain inspectable. Neither accessor supplies an executable receipt.
    const Arrival*first_fault_arrival()const noexcept{return have_fault_arrival_?&fault_arrival_:nullptr;}
    const Arrival*last_arrival()const noexcept{return have_arrival_?&last_arrival_:nullptr;}
    const Arrival*audit_fragment(std::size_t i)const noexcept{return i<next_?&completed_.fragments[i]:nullptr;}
    const RequestBytes*retained_request()const noexcept{return d_.requests?&expected_bytes_:nullptr;}
    const ReplyBytes*retained_reply()const noexcept{return next_?&completed_.bytes:nullptr;}
    std::size_t received_fragments()const noexcept{return next_;}
    bool ready()const noexcept{return ready_&&!failed();}
    bool failed()const noexcept{return d_.first_fault!=Fault::None;}
    const Diagnostics&diagnostics()const noexcept{return d_;}
    void stop(std::uint64_t now)noexcept{fail(Fault::Stopped,now);}
private:
    bool fail(Fault why,std::uint64_t now)noexcept{
        if(!failed()){d_.first_fault=why;d_.first_fault_processing_us=now;
            if(inside_receive_){fault_arrival_=last_arrival_;have_fault_arrival_=true;}
            if(window_)window_->stop(now);
            if(task_)task_->stop(now);}
        active_=ready_=false;return false;
    }
    bool validate_envelope(const Arrival&a,std::uint64_t now)noexcept{
        const auto&f=a.fields;const auto&c=configuration_;
        if(a.ingress_first_fault||a.ingress_last_fault||a.ingress_first_fault_hrt||a.ingress_last_fault_hrt||
           a.ingress_rejected_total||a.ingress_queue_overflows||a.ingress_publication_failures)return fail(Fault::IngressFault,now);
        if(f.source_system!=c.source_system||f.source_component!=c.source_component||f.target_system!=c.target_system||
           f.target_component!=c.target_component||f.receiver_instance!=c.receiver_instance||f.payload_type!=gpenmpc_ingress::payload_type)return fail(Fault::Source,now);
        if(!f.timestamp||f.timestamp>now||f.timestamp<d_.last_original_arrival_us)return fail(Fault::Timestamp,now);
        if(d_.last_reception_sequence==UINT64_MAX||d_.last_uorb_generation==UINT32_MAX)return fail(Fault::CounterOverflow,now);
        if(!f.reception_sequence||(d_.last_reception_sequence&&f.reception_sequence!=d_.last_reception_sequence+1))return fail(Fault::SequenceGap,now);
        // Real uORB Subscription generation is uint32_t; do not turn a caller
        // supplied uint64_t into a different unbounded counter namespace.
        if(!a.uorb_generation||a.uorb_generation>UINT32_MAX||(d_.last_uorb_generation&&a.uorb_generation!=d_.last_uorb_generation+1))return fail(Fault::UorbGap,now);
        if(!f.payload_length||f.payload_length>128||f.wire_payload_length<5||f.wire_payload_length>133)return fail(Fault::Length,now);
        if(f.wire_payload_length>5+f.payload_length)return fail(Fault::Padding,now);
        for(unsigned i=f.payload_length;i<128;++i)if(f.payload[i])return fail(Fault::Padding,now);
        if(!d_.envelopes_admitted){d_.first_reception_sequence=f.reception_sequence;d_.first_uorb_generation=a.uorb_generation;}
        d_.last_reception_sequence=f.reception_sequence;d_.last_uorb_generation=a.uorb_generation;d_.last_original_arrival_us=f.timestamp;++d_.envelopes_admitted;return true;
    }
    bool dispatch_validated(const Arrival&a,std::uint64_t now)noexcept{
        // Future concrete routes belong HERE, after the one central envelope
        // accounting above. Current legacy schemas 1/2/4 and all other schemas
        // are rejected; they cannot bypass or reset the common watermarks.
        switch(a.fields.payload[0]>>4){
        case gpenmpc_local_gp_wire::reply_schema:return receive_gp(a,now);
        case gpenmpc_local_window_wire::schema:
            if(!window_)return fail(Fault::UnknownSchema,now);
            return window_->receive(a,now)?true:fail(Fault::Window,now);
        case gpenmpc_local_task_wire::schema:
            if(!task_)return fail(Fault::UnknownSchema,now);
            return task_->receive(a,now)?true:fail(Fault::TaskInput,now);
        default:return fail(Fault::UnknownSchema,now);
        }
    }
    bool receive_gp(const Arrival&a,std::uint64_t now)noexcept{
        const auto generation=gpenmpc_argument_transport::get64(a.fields.payload+1);
        if(runtime_&&(discarding_||(have_retired_&&generation==retired_request_.output_generation))){
            // Reuse this same assembly storage for one exact retired query.
            // No extra queue or new protocol; never deliver it to fill_gp.
            if(ready_||(!discarding_&&next_))return fail(Fault::Interleaved,now);
            if(!discarding_){completed_={};discarding_=true;}
            if(next_>=fragments)return fail(Fault::Index,now);
            const auto&f=a.fields;const std::size_t offset=next_*119;
            const auto n=gpenmpc_local_gp_wire::reply_bytes-offset<119?gpenmpc_local_gp_wire::reply_bytes-offset:119;
            if((f.payload[0]&15)!=next_||f.payload_length!=n+9||
               generation!=retired_request_.output_generation)return fail(Fault::Interleaved,now);
            if(f.timestamp<retired_request_.original_publication_us)return fail(Fault::Timestamp,now);
            std::memcpy(completed_.bytes.data()+offset,f.payload+9,n);completed_.fragments[next_]=a;
            if(!next_)completed_.first_original_arrival_us=f.timestamp;
            completed_.last_original_arrival_us=f.timestamp;++next_;
            if(next_==fragments){
                if(!gpenmpc_local_gp_wire::decode(completed_.bytes,reply_)||
                   !gpenmpc_local_gp_wire::matches(retired_request_,reply_))return fail(Fault::Integrity,now);
                next_=0;discarding_=false;have_retired_=false;completed_={};
            }
            return true;
        }
        if(!active_||ready_)return fail(ready_?Fault::Pending:Fault::NoExpectation,now);
        const auto&f=a.fields;
        if(f.timestamp<original_ready_)return fail(Fault::Timestamp,now);
        if((f.payload[0]&15)!=next_||next_>=fragments)return fail(Fault::Index,now);
        const std::size_t offset=next_*119,n=(gpenmpc_local_gp_wire::reply_bytes-offset)<119?(gpenmpc_local_gp_wire::reply_bytes-offset):119;
        if(f.payload_length!=n+9)return fail(Fault::Length,now);
        if(gpenmpc_argument_transport::get64(f.payload+1)!=request_.output_generation)return fail(Fault::Interleaved,now);
        std::memcpy(completed_.bytes.data()+offset,f.payload+9,n);completed_.fragments[next_]=a;
        if(!next_)completed_.first_original_arrival_us=f.timestamp;
        completed_.last_original_arrival_us=f.timestamp;++next_;
        if(next_!=fragments)return true;
        if(!gpenmpc_local_gp_wire::decode(completed_.bytes,reply_)||!gpenmpc_local_gp_wire::matches(request_,reply_))return fail(Fault::Integrity,now);
        completed_.completed_processing_us=now;completed_.complete=true;ready_=true;active_=false;++d_.completed;return true;
    }
    const Configuration configuration_;
    gpenmpc_local_window_wire::Assembler*const window_; // same serialized owner; outlives this dispatcher
    gpenmpc_local_task_wire::Receiver*const task_;
    const bool runtime_;
    gpenmpc_local_gp_wire::Request retired_request_{};
    std::uint64_t expired_generation_{};
    bool have_retired_{},discarding_{};
    gpenmpc_local_gp_wire::Request request_{};gpenmpc_local_gp_wire::Reply reply_{};
    gpenmpc_local_gp_wire::Identity session_identity_{};
    RequestBytes expected_bytes_{};gpenmpc_local_gp_wire::Hash expected_hash_{};
    Completed completed_{};Arrival last_arrival_{},fault_arrival_{};Diagnostics d_{};
    std::uint64_t original_ready_{},last_expected_output_{},last_expected_source_{},last_expected_stamp_{};
    std::size_t next_{};bool active_{},ready_{},have_arrival_{},have_fault_arrival_{},inside_receive_{};
};
}
