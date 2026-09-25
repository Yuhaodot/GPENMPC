#pragma once
#include "RflyContextAssembler.hpp"
namespace gpenmpc_rfly_wire {
using TransportFault=gpenmpc_argument_transport::Fault;
// One ingress owner and sequence. Receive all context3 fragments before
// all numeric4 fragments; batches cannot interleave.
class IngressDispatch final {
public:
    explicit IngressDispatch(gpenmpc_argument_transport::Configuration c)noexcept:
        configuration_(c),context_(c),numeric_(c){if(context_.failed()||numeric_.failed())fail(TransportFault::Configuration);}
    bool expect_context(std::uint64_t generation,std::uint64_t now)noexcept{
        if(!tick(now))return false;
        if(phase_!=Phase::Idle)return fail(TransportFault::Pending);
        if(!context_.expect(generation,now))return fail(context_.fault());
        phase_=Phase::Context;return true;
    }
    // Original event belongs to the source/header-validated first ingress
    // fragment. Actual now remains authoritative for all deadline checks.
    bool expect_context_from_first_arrival(std::uint64_t generation,std::uint64_t original_begin,std::uint64_t now)noexcept{
        if(failed())return false;
        if(phase_!=Phase::Idle)return fail(TransportFault::Pending);
        if(!original_begin||original_begin>now||original_begin<last_original_||now<last_now_)return fail(TransportFault::Timestamp);
        if(!context_.expect(generation,original_begin))return fail(context_.fault());
        phase_=Phase::Context;return tick(now);
    }
    bool expect_numerical(const gpenmpc_argument_transport::Metadata&m,std::uint64_t now)noexcept{
        if(!tick(now))return false;
        if(phase_!=Phase::Idle)return fail(TransportFault::Pending);
        if(!numeric_.expect(m,now))return fail(numeric_.fault());
        phase_=Phase::Numeric;return true;
    }
    // No caller can supply a replacement begin: use only the context ending
    // HRT retained by this same dispatcher after successful actual assembly.
    bool expect_numerical_from_context(const gpenmpc_argument_transport::Metadata&m,std::uint64_t now)noexcept{
        if(!tick(now))return false;
        if(phase_!=Phase::Idle||!completed_context_original_)return fail(TransportFault::Pending);
        if(!numeric_.expect_from_original_event(m,completed_context_original_,now))return fail(numeric_.fault());
        completed_context_original_=0;phase_=Phase::Numeric;return true;
    }
    bool receive(const gpenmpc_argument_transport::Arrival&a,std::uint64_t now)noexcept{
        if(!tick(now))return false;
        const auto&f=a.fields;const auto&c=configuration_;
        if(a.ingress_first_fault||a.ingress_last_fault||a.ingress_first_fault_hrt||a.ingress_last_fault_hrt||a.ingress_rejected_total||a.ingress_queue_overflows||a.ingress_publication_failures)return fail(TransportFault::IngressFault);
        if(f.source_system!=c.source_system||f.source_component!=c.source_component||f.target_system!=c.target_system||f.target_component!=c.target_component||f.receiver_instance!=c.receiver_instance||f.payload_type!=gpenmpc_ingress::payload_type)return fail(TransportFault::Source);
        if(!f.timestamp||f.timestamp>now||f.timestamp<last_original_)return fail(TransportFault::Timestamp);
        if(!f.reception_sequence||(last_sequence_&&f.reception_sequence!=last_sequence_+1))return fail(TransportFault::SequenceGap);
        if(!a.uorb_generation||(last_uorb_&&a.uorb_generation!=last_uorb_+1))return fail(TransportFault::UorbGap);
        const auto schema=f.payload[0]>>4;
        if(schema!=2&&schema!=4)return fail(TransportFault::UnknownSchema);
        if((schema==4&&phase_!=Phase::Context)||(schema==2&&phase_!=Phase::Numeric))return fail(TransportFault::Interleaved);
        last_sequence_=f.reception_sequence;last_uorb_=a.uorb_generation;last_original_=f.timestamp;
        if(schema==4){if(!numeric_.observe_context(a,now))return fail(numeric_.fault());
            if(!context_.receive(a,now))return fail(context_.fault());}
        else if(!numeric_.receive(a,now))return fail(numeric_.fault());
        return true;
    }
    bool take_context(gpenmpc_context_wire::Context&out,std::uint64_t&original,std::uint64_t now)noexcept{
        if(!tick(now)||phase_!=Phase::Context)return false;
        if(!context_.take(out,original,now))return false;
        completed_context_original_=original;
        phase_=Phase::Idle;return true;
    }
    bool take_numerical(gpenmpc_slim_transport::SlimCompleted&out,std::uint64_t now)noexcept{
        if(!tick(now)||phase_!=Phase::Numeric)return false;
        if(!numeric_.take(out,now))return false;
        phase_=Phase::Idle;return true;
    }
    bool tick(std::uint64_t now)noexcept{
        if(failed())return false;
        if(!now||now<last_now_)return fail(TransportFault::Timestamp);
        last_now_=now;
        if(!context_.tick(now))return fail(context_.fault());
        if(!numeric_.tick(now))return fail(numeric_.fault());
        return true;
    }
    bool failed()const noexcept{return fault_!=TransportFault::None;}TransportFault fault()const noexcept{return fault_;}
    std::uint64_t last_original_hrt()const noexcept{return last_original_;}
    std::uint64_t last_reception_sequence()const noexcept{return last_sequence_;}
    std::uint64_t last_uorb_generation()const noexcept{return last_uorb_;}
private:
    enum class Phase:std::uint8_t{Idle,Context,Numeric};
    bool fail(TransportFault f)noexcept{if(!failed())fault_=f;return false;}
    const gpenmpc_argument_transport::Configuration configuration_;gpenmpc_context_wire::Assembler context_;
    gpenmpc_slim_transport::SlimAssembler numeric_;std::uint64_t last_original_{},last_sequence_{},last_uorb_{},last_now_{},completed_context_original_{};
    TransportFault fault_{TransportFault::None};Phase phase_{Phase::Idle};
};
} // namespace gpenmpc_rfly_wire
