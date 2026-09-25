#include "CanonicalLocalExecutionCycle.hpp"

namespace gpenmpc_rfly_px4 {
CanonicalLocalExecutionCycle::CanonicalLocalExecutionCycle(Px4CanonicalLocalIo&i,
    gpenmpc_local_phase::CanonicalLocalPhaseClock&p,gpenmpc_hil_endpoint_reader::Px4OriginalHilReceiptReader&s,
    const LocalCycleConfiguration&c)noexcept:io_(i),phase_(p),source_(s),config_(c){
    if(!c.reference_max_age_us||!c.context.explicit_leg||
       c.initial_interval.leg!=c.context.explicit_leg||
       c.initial_interval.configuration_sha256!=c.context.configuration_sha256||
       (c.operator_reference&&!c.component_initialization)||
       gpenmpc_local_input::empty(c.initial_interval.original_configuration_receipt_sha256))
        fail(LocalCycleFault::Configuration);
}
bool CanonicalLocalExecutionCycle::fail(LocalCycleFault f)noexcept{
    if(d_.first_fault==LocalCycleFault::None){d_.first_fault=f;d_.first_fault_us=hrt_absolute_time();}
    retain_actual_feedback();feedback_.usable_at_read=false;
    io_.stop();phase_.retire();source_.stop();return false;
}
void CanonicalLocalExecutionCycle::retain_actual_feedback()noexcept{
    // Never replace previously retained actual publication with an empty read.
    LocalExecutionFeedback actual{};
    if(io_.take_execution_feedback(actual)){feedback_.actual=actual;feedback_.present=true;feedback_pending_=true;}
}
LocalCapture CanonicalLocalExecutionCycle::capture()noexcept{
    if(d_.first_fault!=LocalCycleFault::None)return LocalCapture::Rejected;
    const bool refresh_unexecuted=pending_&&!disarmed_pending_&&!reference_started_&&
        (config_.component_initialization||config_.runtime_state_only);
    if((pending_&&!refresh_unexecuted)||feedback_pending_){fail(LocalCycleFault::Pending);return LocalCapture::Rejected;}
    if(!config_.runtime_state_only&&source_.drain()==gpenmpc_hil_endpoint_reader::Drain::Unavailable){
        fail(LocalCycleFault::SourceReceipt);return LocalCapture::Rejected;
    }
    const auto result=io_.capture_next(ticket_,false,refresh_unexecuted);
    if(result==LocalCapture::NoUpdate)return result;
    if(result!=LocalCapture::Accepted){fail(LocalCycleFault::Io);return LocalCapture::Rejected;}
    // The HIL receipt is published by the existing MAVLink receive path while
    // odometry is produced asynchronously downstream.  A receipt can become
    // visible after the bounded pre-capture drain and before this snapshot is
    // returned.  Drain once more, then retain the original exact-HRT lookup;
    // no approximate match, waiting loop, timestamp rewrite or age relaxation.
    if(!config_.runtime_state_only&&source_.drain()==gpenmpc_hil_endpoint_reader::Drain::Unavailable){
        fail(LocalCycleFault::SourceReceipt);return LocalCapture::Rejected;
    }
    const auto*s=io_.snapshot(ticket_);
    endpoint_={};
    if(!s||(!config_.runtime_state_only&&!source_.lookup(*s,endpoint_))){fail(LocalCycleFault::SourceReceipt);return LocalCapture::Rejected;}
    // The original endpoint remains endpoint-only. Independent rotor and
    // selected-EKF association must be supplied by the actual ingress owner.
    pending_=true;reference_started_=false;disarmed_pending_=false;feedback_={};++d_.captures;return result;
}
LocalCapture CanonicalLocalExecutionCycle::capture_disarmed()noexcept{
    if(d_.first_fault!=LocalCycleFault::None)return LocalCapture::Rejected;
    if(pending_||feedback_pending_||d_.attempts||phase_.diagnostics().queries){
        fail(LocalCycleFault::Pending);return LocalCapture::Rejected;
    }
    if(!config_.runtime_state_only&&source_.drain()==gpenmpc_hil_endpoint_reader::Drain::Unavailable){
        fail(LocalCycleFault::SourceReceipt);return LocalCapture::Rejected;
    }
    const auto result=io_.capture_disarmed(ticket_);
    if(result==LocalCapture::NoUpdate)return result;
    if(result!=LocalCapture::Accepted){fail(LocalCycleFault::Io);return LocalCapture::Rejected;}
    // Same bounded post-capture receipt drain as the armed path.  This closes
    // only the producer/consumer scheduling race before the first RLS1 export.
    if(!config_.runtime_state_only&&source_.drain()==gpenmpc_hil_endpoint_reader::Drain::Unavailable){
        fail(LocalCycleFault::SourceReceipt);return LocalCapture::Rejected;
    }
    const auto*s=io_.snapshot(ticket_);
    endpoint_={};
    if(!s||(!config_.runtime_state_only&&!source_.lookup(*s,endpoint_))){fail(LocalCycleFault::SourceReceipt);return LocalCapture::Rejected;}
    pending_=true;disarmed_pending_=true;reference_started_=false;feedback_={};
    ++d_.observation_captures;return result;
}
bool CanonicalLocalExecutionCycle::copy_disarmed_observation(gpenmpc_odometry::Snapshot&snapshot,
    gpenmpc_hil_endpoint_reader::Receipt&endpoint)noexcept{
    snapshot={};endpoint={};
    if(d_.first_fault!=LocalCycleFault::None||!pending_||!disarmed_pending_||
       (!config_.runtime_state_only&&source_.fault()!=gpenmpc_hil_endpoint_reader::Fault::None))return false;
    const auto*s=io_.snapshot(ticket_);if(!s)return false;
    snapshot=*s;endpoint=endpoint_;++d_.observation_exports;return true;
}
bool CanonicalLocalExecutionCycle::release_disarmed()noexcept{
    if(d_.first_fault!=LocalCycleFault::None)return false;
    if(!pending_||!disarmed_pending_)return fail(LocalCycleFault::Pending);
    // Same real Io release semantics as the existing disarmed Pump path:
    // fresh identity/disarmed telemetry is checked again, without executing.
    if(!io_.release_disarmed(ticket_))return fail(LocalCycleFault::Io);
    ++d_.observation_io_releases;
    if(!config_.runtime_state_only&&!source_.retire_through(endpoint_.endpoint.original.original_receiver_hrt_us))
        return fail(LocalCycleFault::SourceReceipt);
    if(!config_.runtime_state_only)++d_.observation_endpoint_retirements;
    pending_=false;disarmed_pending_=false;
    return true;
}
bool CanonicalLocalExecutionCycle::numerical_diagnostics(gpenmpc_full_inner_diagnostics&out)const noexcept{
    return io_.numerical_diagnostics(out);
}
bool CanonicalLocalExecutionCycle::fill_gp(const std::uint64_t tags[2],const double result[18])noexcept{
    if(d_.first_fault!=LocalCycleFault::None)return false;
    if(pending_||disarmed_pending_||phase_.pending())return fail(LocalCycleFault::Pending);
    if(!io_.fill_gp(tags,result))return fail(LocalCycleFault::Io);
    return true;
}
const gpenmpc_odometry::Snapshot*CanonicalLocalExecutionCycle::snapshot()const noexcept{
    return pending_?io_.snapshot(ticket_):nullptr;
}
const gpenmpc_hil_endpoint_reader::Receipt*CanonicalLocalExecutionCycle::original_endpoint()const noexcept{
    return pending_?&endpoint_:nullptr;
}
LocalCycleExecute CanonicalLocalExecutionCycle::execute(const LocalCycleInputs&in)noexcept{
    if(d_.first_fault!=LocalCycleFault::None)return LocalCycleExecute::Rejected;
    if(!pending_||feedback_pending_||disarmed_pending_){fail(LocalCycleFault::Pending);return LocalCycleExecute::Rejected;}
    const auto*s=io_.snapshot(ticket_);if(!s){fail(LocalCycleFault::Io);return LocalCycleExecute::Rejected;}
    const bool first=phase_.diagnostics().installs==0;
    if(!reference_started_){
        command_={};command_.ticket=ticket_;command_.input_context=config_.context;
        command_.operator_reference=config_.operator_reference;
        command_.input_context.step_kind=first?gpenmpc_local_input::StepKind::FirstOfExplicitLeg:
            gpenmpc_local_input::StepKind::SubsequentObservedSource;
        command_.initial_interval=first?&config_.initial_interval:nullptr;
        if(!phase_.begin(*s,in.outer,in.target4,in.loaded_window_generation,command_.initial_interval,command_.reference_query,config_.operator_reference)){
            fail(LocalCycleFault::Phase);return LocalCycleExecute::Rejected;
        }
        // Create the reference on the board using p/v/a from the ABI.
        const auto created=hrt_absolute_time();const auto until=expiry(created,config_.reference_max_age_us);
        if(!until||created<s->estimator().board_rx_us){fail(LocalCycleFault::Clock);return LocalCycleExecute::Rejected;}
        command_.reference_envelope.identity=s->estimator().identity;
        command_.reference_envelope.generation=command_.reference_query.reference_generation;
        command_.reference_envelope.outer_generation=command_.reference_query.outer_generation;
        command_.reference_envelope.timestamp_us=created;command_.reference_envelope.board_rx_us=created;
        command_.reference_envelope.valid_until_us=until;d_.original_reference_created_us=created;
        command_.outer_envelope=in.outer;reference_started_=true;
    }else{
        if(!gpenmpc_full_consumption::detail::same_outer(in.outer,command_.outer_envelope)||
           std::memcmp(&in.target4[0],&command_.reference_query.target_phase_acceleration,sizeof(double))!=0||
           std::memcmp(in.target4+1,command_.reference_query.target_outer_f,3*sizeof(double))!=0||
           !io_.diagnostics().awaiting_window||
           !phase_.retry_window(in.loaded_window_generation,command_.reference_query)){
            fail(LocalCycleFault::Phase);return LocalCycleExecute::Rejected;
        }
        // Refresh no other command or timestamp. LocalIo independently hashes
        // the complete bindings on retry, rejecting changed source/rotor/etc.
    }
    command_.rotor=in.rotor;command_.payload=in.payload;command_.wind=in.wind;++d_.attempts;
    const bool executed=io_.execute(command_);
    if(!executed&&io_.diagnostics().first_fault==LocalFault::None&&io_.diagnostics().awaiting_window){
        ++d_.window_misses;return LocalCycleExecute::NeedReferenceWindow;
    }
    retain_actual_feedback();
    if(!executed){fail(LocalCycleFault::Io);return LocalCycleExecute::Rejected;}
    if(!feedback_.present||!feedback_.actual.fresh_at_read||!feedback_.actual.reference_state_copied){
        fail(LocalCycleFault::Feedback);return LocalCycleExecute::Rejected;
    }
    const auto&actual=feedback_.actual;const auto&t=actual.token.lease_envelope;
    published_={};published_.source_timestamp_ns=t.timestamp_sample_us*1000;
    published_.source_generation=t.sample_generation;published_.output_generation=t.output_generation;
    published_.original_publication_us=actual.original_publication_us;
    std::memcpy(published_.actual_control16,actual.actual_control16,sizeof published_.actual_control16);
    published_.output_published=actual.publication_succeeded;
    if(!phase_.install(actual.committed_reference,published_)){fail(LocalCycleFault::Phase);return LocalCycleExecute::Rejected;}
    feedback_.phase_installed=true;
    if(!config_.runtime_state_only&&!source_.retire_through(endpoint_.endpoint.original.original_receiver_hrt_us)){
        fail(LocalCycleFault::SourceReceipt);return LocalCycleExecute::Rejected;
    }
    feedback_.source_receipt_retired=!config_.runtime_state_only;
    // Installation/retirement are irreversible historical events above. A
    // late completion cannot turn their original deadline into a new lease.
    const auto completed=hrt_absolute_time();
    if(completed<actual.original_commit_completed_us||completed>actual.original_valid_until_us){
        fail(LocalCycleFault::Clock);return LocalCycleExecute::Rejected;
    }
    pending_=false;reference_started_=false;++d_.commits;
    return LocalCycleExecute::Committed;
}
bool CanonicalLocalExecutionCycle::take_feedback(LocalCycleFeedback&out)noexcept{
    out={};if(!feedback_pending_)return false;
    feedback_.original_cycle_read_us=hrt_absolute_time();
    feedback_.usable_at_read=d_.first_fault==LocalCycleFault::None&&
        io_.diagnostics().first_fault==LocalFault::None&&
        phase_.diagnostics().first_fault==gpenmpc_local_phase::Fault::None&&
        (config_.runtime_state_only||source_.fault()==gpenmpc_hil_endpoint_reader::Fault::None)&&
        feedback_.present&&feedback_.phase_installed&&(config_.runtime_state_only||feedback_.source_receipt_retired)&&
        feedback_.actual.fresh_at_read&&
        feedback_.original_cycle_read_us>=feedback_.actual.original_commit_completed_us&&
        feedback_.original_cycle_read_us<=feedback_.actual.original_valid_until_us;
    out=feedback_;feedback_pending_=false;return true;
}
void CanonicalLocalExecutionCycle::stop()noexcept{fail(LocalCycleFault::Stopped);}
}
