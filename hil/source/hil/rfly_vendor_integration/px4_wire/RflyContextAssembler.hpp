#pragma once
#include "RflyContextWire.hpp"
namespace gpenmpc_context_wire {
// Use ingress Arrival/counters and dispatch one contiguous schema4 batch.
class Assembler final {
public:
    using Fault=gpenmpc_argument_transport::Fault;
    explicit Assembler(gpenmpc_argument_transport::Configuration c)noexcept:configuration_(c){
        if(!c.source_system||!c.source_component||!c.target_system||!c.target_component||!c.max_assembly_us)fail(Fault::Configuration);
    }
    bool expect(std::uint64_t reference_generation,std::uint64_t now)noexcept{
        if(failed())return false;
        if(active_||ready_||!reference_generation||!now||now<last_now_)return fail(Fault::Pending);
        expected_=reference_generation;created_=last_now_=now;next_=0;active_=true;bytes_={};first_=last_=sequence_=uorb_=0;return true;
    }
    bool tick(std::uint64_t now)noexcept{
        if(failed())return false;
        if(now<last_now_)return fail(Fault::Timestamp);
        last_now_=now;if((active_||ready_)&&now-created_>configuration_.max_assembly_us)return fail(Fault::Expired);return true;
    }
    bool receive(const Arrival&a,std::uint64_t now)noexcept{
        if(!tick(now))return false;
        if(!active_||ready_)return fail(Fault::NoExpectation);
        if(a.ingress_first_fault||a.ingress_last_fault||a.ingress_first_fault_hrt||a.ingress_last_fault_hrt||a.ingress_rejected_total||a.ingress_queue_overflows||a.ingress_publication_failures)return fail(Fault::IngressFault);
        const auto&f=a.fields;const auto&c=configuration_;
        if(f.source_system!=c.source_system||f.source_component!=c.source_component||f.target_system!=c.target_system||f.target_component!=c.target_component||f.receiver_instance!=c.receiver_instance||f.payload_type!=gpenmpc_ingress::payload_type)return fail(Fault::Source);
        if(!f.timestamp||f.timestamp<created_||f.timestamp>now||f.timestamp<last_)return fail(Fault::Timestamp);
        if(!f.reception_sequence||(sequence_&&f.reception_sequence!=sequence_+1))return fail(Fault::SequenceGap);
        if(!a.uorb_generation||(uorb_&&a.uorb_generation!=uorb_+1))return fail(Fault::UorbGap);
        if((f.payload[0]>>4)!=schema_version)return fail(Fault::UnknownSchema);
        if((f.payload[0]&15)!=next_||next_>=fragment_count)return fail(Fault::Index);
        const std::size_t offset=next_*119,n=(message_bytes-offset)<119?(message_bytes-offset):119;
        if(f.payload_length!=n+9||f.wire_payload_length<5||f.wire_payload_length>5+f.payload_length)return fail(Fault::Length);
        for(unsigned k=f.payload_length;k<128;++k)if(f.payload[k])return fail(Fault::Padding);
        if(get64(f.payload+1)!=expected_)return fail(Fault::Interleaved);
        copy_bytes(f.payload+9,n,bytes_.data()+offset);if(!next_)first_=f.timestamp;last_=f.timestamp;sequence_=f.reception_sequence;uorb_=a.uorb_generation;++next_;
        if(next_==fragment_count){if(!decode(bytes_,context_)||context_.reference_generation!=expected_)return fail(Fault::Integrity);ready_=true;active_=false;}
        return true;
    }
    bool take(Context&out,std::uint64_t&original_ingress,std::uint64_t now)noexcept{
        out={};original_ingress=0;if(!tick(now)||!ready_)return false;out=context_;original_ingress=last_;ready_=false;return true;
    }
    bool failed()const noexcept{return fault_!=Fault::None;}Fault fault()const noexcept{return fault_;}
    std::uint64_t first_original_hrt()const noexcept{return first_;}
private:
    bool fail(Fault f)noexcept{if(!failed())fault_=f;active_=ready_=false;return false;}
    const gpenmpc_argument_transport::Configuration configuration_;Bytes bytes_{};Context context_{};
    std::uint64_t expected_{},created_{},last_now_{},first_{},last_{},sequence_{},uorb_{};
    unsigned next_{};Fault fault_{Fault::None};bool active_{},ready_{};
};
} // namespace gpenmpc_context_wire
