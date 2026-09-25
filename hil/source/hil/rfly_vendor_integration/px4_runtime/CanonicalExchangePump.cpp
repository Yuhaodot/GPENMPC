#include "CanonicalExchangePump.hpp"
namespace gpenmpc_rfly_px4 {
CanonicalExchangePump::CanonicalExchangePump(const ExchangeConfiguration&configuration)noexcept:
    configuration_(configuration),ingress_(ORB_ID(gpenmpc_full_inner_ingress),configuration.ingress_topic_instance),
    dispatch_(configuration.transport),binding_(configuration.execution){
    if(!configuration.retained_anchor_capacity||configuration.retained_anchor_capacity>maximum_anchor_records||
       !configuration.execution.limits.sample_max_age_us||!configuration.execution.limits.reference_max_age_us||
       !configuration.execution.limits.outer_max_age_us||dispatch_.failed()||binding_.failed()||
       configuration.transport.target_system!=configuration.execution.identity.system||configuration.transport.target_component!=configuration.execution.identity.component){
        diagnostics_.fault=ExchangeFault::Configuration;diagnostics_.first_fault_us=hrt_absolute_time();outbox_.close();}
}
bool CanonicalExchangePump::fail(Px4CanonicalIo&io,ExchangeFault why)noexcept{
    if(diagnostics_.fault==ExchangeFault::None){diagnostics_.fault=why;diagnostics_.first_fault_us=hrt_absolute_time();}
    outbox_.close();feedback_outbox_.revoke();io.stop();if(owner_&&owner_!=&io)owner_->stop();return false;
}
void CanonicalExchangePump::stop(Px4CanonicalIo&io)noexcept{fail(io,ExchangeFault::Stopped);}
ExchangePoll CanonicalExchangePump::poll_disarmed_observation(Px4CanonicalIo&io)noexcept{
    ++diagnostics_.polls;if(!owner_)owner_=&io;
    if(owner_!=&io||phase_!=Phase::NeedCapture||diagnostics_.execute_attempts||diagnostics_.executed){fail(io,ExchangeFault::Io);return ExchangePoll::Fault;}
    if(diagnostics_.fault!=ExchangeFault::None||io.diagnostics().fault!=Fault::None||outbox_.failed()){
        fail(io,ExchangeFault::Io);return ExchangePoll::Fault;
    }
    Ticket ticket{};const auto result=io.capture_disarmed_observation(ticket);
    if(result==Capture::Rejected){fail(io,ExchangeFault::Io);return ExchangePoll::Fault;}
    if(result==Capture::NoUpdate)return ExchangePoll::Idle;
    ++diagnostics_.observations;
    // Preserve a busy export slot while observation and release continue.
    if(!outbox_.pending()){
        const auto now=hrt_absolute_time();const auto age=configuration_.execution.limits.reference_max_age_us>configuration_.execution.limits.outer_max_age_us?
            configuration_.execution.limits.reference_max_age_us:configuration_.execution.limits.outer_max_age_us;
        anchors_.retire_expired(now,age);const auto*s=io.snapshot(ticket);
        if(!s||anchors_.count()>=configuration_.retained_anchor_capacity||!anchors_.record(io,ticket,configuration_.execution,now,export_scratch_)){
            fail(io,ExchangeFault::Source);return ExchangePoll::Fault;
        }
        const auto sample=s->estimator().timestamp_sample_us;const auto limit=configuration_.execution.limits.sample_max_age_us;
        if(sample>UINT64_MAX-limit||!outbox_.publish(export_scratch_,sample,sample+limit,
            configuration_.transport.source_system,configuration_.transport.source_component)){
            fail(io,ExchangeFault::Outbox);return ExchangePoll::Fault;
        }
        ++diagnostics_.observation_exports;
    }
    if(!io.release_disarmed_observation(ticket)){fail(io,ExchangeFault::Io);return ExchangePoll::Fault;}
    return ExchangePoll::Progress;
}
bool CanonicalExchangePump::fresh_pending(std::uint64_t now)const noexcept{
    return pending_source_us_&&now>=pending_source_us_&&now-pending_source_us_<=configuration_.execution.limits.sample_max_age_us;
}
bool CanonicalExchangePump::first_context_header_valid(const gpenmpc_argument_transport::Arrival&a)const noexcept{
    const auto&f=a.fields;const auto&c=configuration_.transport;
    return f.source_system==c.source_system&&f.source_component==c.source_component&&f.target_system==c.target_system&&
        f.target_component==c.target_component&&f.receiver_instance==c.receiver_instance&&f.payload_type==gpenmpc_ingress::payload_type&&
        f.payload_length==128&&f.payload[0]==0x40&&gpenmpc_argument_transport::get64(f.payload+1)>0&&
        f.timestamp>=pending_receipt_us_&&!a.ingress_first_fault&&!a.ingress_last_fault&&!a.ingress_first_fault_hrt&&
        !a.ingress_last_fault_hrt&&!a.ingress_rejected_total&&!a.ingress_queue_overflows&&!a.ingress_publication_failures;
}
ExchangePoll CanonicalExchangePump::poll(Px4CanonicalIo&io)noexcept{
    ++diagnostics_.polls;
    if(!owner_)owner_=&io;
    if(owner_!=&io){fail(io,ExchangeFault::Io);return ExchangePoll::Fault;}
    if(diagnostics_.fault!=ExchangeFault::None){io.stop();return ExchangePoll::Fault;}
    if(io.diagnostics().fault!=Fault::None||outbox_.failed()||feedback_outbox_.interrupted()){fail(io,ExchangeFault::Io);return ExchangePoll::Fault;}
    bool progress=false;
    const auto age=configuration_.execution.limits.reference_max_age_us>configuration_.execution.limits.outer_max_age_us?
        configuration_.execution.limits.reference_max_age_us:configuration_.execution.limits.outer_max_age_us;
    anchors_.retire_expired(hrt_absolute_time(),age);
    if(phase_==Phase::NeedCapture){
        // Pending output may not be overwritten. No capture while Io has a
        // pending ticket; only successful execute reopens this branch.
        if(outbox_.pending()||feedback_outbox_.pending()){fail(io,ExchangeFault::Outbox);return ExchangePoll::Fault;}
        if(anchors_.count()>=configuration_.retained_anchor_capacity){fail(io,ExchangeFault::Capacity);return ExchangePoll::Fault;}
        const auto captured=io.capture_next(pending_ticket_);
        if(captured==Capture::Rejected){fail(io,ExchangeFault::Io);return ExchangePoll::Fault;}
        if(captured==Capture::NoUpdate)return ExchangePoll::Idle;
        const auto*snapshot=io.snapshot(pending_ticket_);const auto now=hrt_absolute_time();
        if(!snapshot||!anchors_.record(io,pending_ticket_,configuration_.execution,now,export_scratch_)){fail(io,ExchangeFault::Source);return ExchangePoll::Fault;}
        pending_generation_=snapshot->estimator().generation;pending_source_us_=snapshot->estimator().timestamp_sample_us;pending_receipt_us_=snapshot->estimator().board_rx_us;
        const auto limit=configuration_.execution.limits.sample_max_age_us;
        if(pending_source_us_>UINT64_MAX-limit||!outbox_.publish(export_scratch_,pending_source_us_,pending_source_us_+limit,
            configuration_.transport.source_system,configuration_.transport.source_component)){fail(io,ExchangeFault::Outbox);return ExchangePoll::Fault;}
        phase_=Phase::AwaitContextFirst;diagnostics_.pending_original_sample_us=pending_source_us_;++diagnostics_.captured;progress=true;
    }
    if(!fresh_pending(hrt_absolute_time())){fail(io,ExchangeFault::Expired);return ExchangePoll::Fault;}
    // The real topic queue is8. Bound this invocation to its actual capacity;
    // updates that race this pass retain original timestamps for a later poll.
    for(unsigned k=0;k<gpenmpc_ingress::queue_capacity&&ingress_.update(&ingress_message_);++k){
        const auto now=hrt_absolute_time();const auto generation=ingress_.get_last_generation();++diagnostics_.ingress_updates;
        arrival_=gpenmpc_argument_transport::from_topic(ingress_message_,generation);diagnostics_.last_ingress_original_us=arrival_.fields.timestamp;
        if(!fresh_pending(now)){fail(io,ExchangeFault::Expired);return ExchangePoll::Fault;}
        if(phase_==Phase::AwaitContextFirst){
            if(!first_context_header_valid(arrival_)||!dispatch_.expect_context_from_first_arrival(
                gpenmpc_argument_transport::get64(arrival_.fields.payload+1),arrival_.fields.timestamp,now)){fail(io,ExchangeFault::Ingress);return ExchangePoll::Fault;}
            phase_=Phase::Context;
        }
        if(!dispatch_.receive(arrival_,now)){fail(io,ExchangeFault::Ingress);return ExchangePoll::Fault;}
        progress=true;
        if(phase_==Phase::Context){std::uint64_t original=0;
            if(dispatch_.take_context(context_scratch_,original,now)){
                if(!binding_.accept(context_scratch_,anchors_,original,now)){fail(io,ExchangeFault::Context);return ExchangePoll::Fault;}
                ++diagnostics_.context_accepted;metadata_scratch_={};metadata_scratch_.command_generation=pending_generation_;
                metadata_scratch_.snapshot_ticket=pending_ticket_;metadata_scratch_.reference_generation=binding_.reference().generation;
                metadata_scratch_.outer_generation=binding_.outer().generation;metadata_scratch_.configuration_sha256=gpenmpc_argument_transport::bytes_of(configuration_.execution.configuration_payload_sha256);
                if(!dispatch_.expect_numerical_from_context(metadata_scratch_,now)){fail(io,ExchangeFault::Ingress);return ExchangePoll::Fault;}
                phase_=Phase::Numeric;
            }
        }else if(phase_==Phase::Numeric&&dispatch_.take_numerical(numerical_scratch_,now)){
            if(!gpenmpc_slim_transport::bind_command(numerical_scratch_,binding_.reference(),binding_.outer(),configuration_.execution,command_scratch_)){
                fail(io,ExchangeFault::Context);return ExchangePoll::Fault;}
            ++diagnostics_.execute_attempts;
            if(!io.execute(command_scratch_)){fail(io,ExchangeFault::Io);return ExchangePoll::Fault;}
            ++diagnostics_.executed; // actual successful execution remains counted if export fails
            const auto disposition=io.take_committed_feedback(feedback_scratch_);
            if(disposition==FeedbackDisposition::Empty||
                !feedback_outbox_.publish(feedback_scratch_,configuration_.transport.source_system,configuration_.transport.source_component)){
                fail(io,ExchangeFault::Outbox);return ExchangePoll::Fault;
            }
            if(disposition!=FeedbackDisposition::Fresh){fail(io,ExchangeFault::Expired);return ExchangePoll::Fault;}
            phase_=Phase::NeedCapture;pending_ticket_={};pending_generation_=pending_source_us_=pending_receipt_us_=0;
            // Queued input waits for the next source capture.
            return ExchangePoll::Progress;
        }
    }
    const auto now=hrt_absolute_time();
    if(!fresh_pending(now)){fail(io,ExchangeFault::Expired);return ExchangePoll::Fault;}
    // Idle before the first fragment is NOT an assembly event. Advancing that
    // assembler clock would reject a legitimately earlier queued first stamp.
    if(phase_!=Phase::AwaitContextFirst&&!dispatch_.tick(now)){fail(io,ExchangeFault::Ingress);return ExchangePoll::Fault;}
    return progress?ExchangePoll::Progress:ExchangePoll::Idle;
}
} // namespace gpenmpc_rfly_px4
