#include "RegisteredCanonicalModuleContext.hpp"
#include <drivers/drv_hrt.h>

namespace gpenmpc_rfly_px4 {
namespace {
Identity echo_identity(const SessionEcho &e) noexcept
{return {e.observed_identity.uid,e.process_session_generation,e.observed_identity.system,e.observed_identity.component};}
bool fresh_context(std::uint64_t stamp,std::uint64_t now,std::uint64_t age) noexcept
{return stamp&&stamp<=now&&guard_original_expiry(stamp,age)>=now;}
}
ExchangeConfiguration RegisteredCanonicalModuleContext::exchange_configuration(const RegisteredContextConfiguration &c) noexcept
{
    ExchangeConfiguration out{};out.execution=c.module.execution;out.transport=c.transport;
    out.ingress_topic_instance=c.ingress_topic_instance;out.retained_anchor_capacity=c.retained_anchor_capacity;return out;
}
RegisteredCanonicalModuleContext::RegisteredCanonicalModuleContext(RegisteredContextGuard &guard,
    const SessionEcho &echo,const RegisteredContextConfiguration &c) noexcept:
    guard_(guard),echo_(echo),configuration_(c),
    direct_(guard_,clock_,echo_identity(echo),c.module.telemetry_max_age_us),
    native_land_(guard_,clock_,echo_identity(echo),c.module.telemetry_max_age_us),
    disarmed_zero_(guard_,clock_,echo_identity(echo),c.module.telemetry_max_age_us),
    pump_(exchange_configuration(c))
{
    if(!c.native_land_tail_max_us||!c.module.telemetry_max_age_us||!c.module.poll_period_us||
       c.module.task_stack_bytes<=2240||!echo.link.pointer||!echo.link.generation||
       !(c.module.source.identity==echo_identity(echo))||!(c.module.execution.identity==echo_identity(echo))||
       c.module.source.vehicle_odometry_topic!=ORB_ID(vehicle_odometry)||
       c.transport.receiver_instance>=ORB_MULTI_MAX_INSTANCES||
       c.transport.target_system!=echo.observed_identity.system||c.transport.target_component!=echo.observed_identity.component||
       c.module.execution.configuration_payload_sha256!=echo.configuration_sha256||
       pump_.diagnostics().fault!=ExchangeFault::None)
        (void)fail(RegisteredContextFault::Configuration,hrt_absolute_time());
}
bool RegisteredCanonicalModuleContext::fail(RegisteredContextFault f,std::uint64_t now) noexcept
{
    prestream_.stop(); // permanent for this Context, never poisons SessionGuard identity
    if(diagnostics_.first_fault==RegisteredContextFault::None){diagnostics_.first_fault=f;diagnostics_.first_fault_us=now;}
    return false;
}
bool RegisteredCanonicalModuleContext::session_and_physical(BoardSafetyEvidence &out) noexcept
{
    SessionGuardEvidence session{};
    if(!guard_.evidence_snapshot(session)||session.first_fault!=SessionFault::None||
       !session.registered||!session.echo_confirmed||!session.human_declaration_bound||
       !same_session_echo(session.registration,echo_)||
       !guard_.validate_ticket_binding({execution_session_digest(echo_),echo_.process_session_generation})||
       !guard_.observe(out)||!(out.identity==echo_identity(echo_)))
        return fail(RegisteredContextFault::Session,hrt_absolute_time());
    if(!out.require_physical_facts()||out.usb_transport!=GuardFact::Pass||out.hil_configuration!=GuardFact::Pass)
        return fail(RegisteredContextFault::Physical,hrt_absolute_time());
    return true;
}
ModuleAcquire RegisteredCanonicalModuleContext::acquire(ModuleConfiguration &out) noexcept
{
    out={};
    if(diagnostics_.first_fault!=RegisteredContextFault::None)return ModuleAcquire::Rejected;
    if(diagnostics_.acquire_aborted)return ModuleAcquire::Rejected;
    if(diagnostics_.acquired){(void)fail(RegisteredContextFault::DuplicateAcquire,hrt_absolute_time());return ModuleAcquire::Rejected;}
    BoardSafetyEvidence observed{};
    if(!session_and_physical(observed))return ModuleAcquire::Unavailable;
    if(observed.native_recovery_configuration!=GuardFact::Pass||!legacy_trajectory_stopped_with_module_lock()){
        (void)fail(RegisteredContextFault::Control,hrt_absolute_time());return ModuleAcquire::Rejected;
    }
    // Singleton access creates no output/route and requires no asynchronous
    // cleanup if a subsequent ModuleBase allocation/spawn is rejected.
    outputs_=gpenmpc_rfly_stream::shared_output_registry();snapshots_=gpenmpc_rfly_stream::snapshot_route_registry();
    feedback_=gpenmpc_rfly_stream::committed_feedback_route_registry();
    if(!outputs_||!snapshots_||!feedback_||!outputs_->ready()||!snapshots_->ready()||!feedback_->ready())
        return ModuleAcquire::Unavailable;
    links_=gpenmpc_rfly_stream::link_lifetime_registry();
    if(!links_||links_->bind_canonical(echo_,configuration_.transport.source_system,
        configuration_.transport.source_component,this,reservation_)!=gpenmpc_rfly_stream::CanonicalBind::Bound){
        (void)release_reservation();return ModuleAcquire::Unavailable;
    }
    diagnostics_.reservation_bound=true;
    if(!prestream_.start(reservation_.heartbeat,configuration_.module.telemetry_max_age_us)){
        (void)release_reservation();(void)fail(RegisteredContextFault::Configuration,hrt_absolute_time());return ModuleAcquire::Rejected;
    }
    out=configuration_.module;out.authority=&direct_;diagnostics_.acquired=true;return ModuleAcquire::Ready;
}
bool RegisteredCanonicalModuleContext::release_reservation() noexcept
{
    if(!reservation_.generation)return true;
    if(!links_||links_->release_canonical(reservation_)!=gpenmpc_rfly_stream::LinkAccess::Read)return false;
    reservation_={};diagnostics_.reservation_bound=false;return true;
}
bool RegisteredCanonicalModuleContext::abort_acquire() noexcept
{
    // Only ModuleBase's synchronous failed-start path (or still-uninstalled
    // application cleanup) may use this. Never hide a live route/task/OCM.
    if(io_||routes_attempted_||diagnostics_.offboard_publication_attempts)return false;
    prestream_.stop();diagnostics_.acquire_aborted=true;
    return release_reservation();
}
bool RegisteredCanonicalModuleContext::update_prestream(Px4CanonicalIo &io,
    const BoardSafetyEvidence &observed,bool allow_emit) noexcept
{
    using namespace gpenmpc_rfly_stream;
    HostHeartbeatSnapshot heartbeat{};
    if(!links_||links_->read_heartbeat(reservation_,0,configuration_.module.telemetry_max_age_us,heartbeat)!=LinkAccess::Read)
        return fail(RegisteredContextFault::Registry,hrt_absolute_time());
    // Copy THEN sample processing time: a heartbeat can arrive while this poll
    // runs. This only classifies copied raw fields; never re-record/re-stamp it.
    const auto now=hrt_absolute_time();
    if(!now||observed.identity_valid_until_us<now||observed.native_recovery_configuration!=GuardFact::Pass)
        return fail(RegisteredContextFault::Control,now);
    if(heartbeat.receipt_generation){
        if(!heartbeat.original_receiver_hrt_us||!heartbeat.original_valid_until_us||heartbeat.original_receiver_hrt_us>now)
            return fail(RegisteredContextFault::Clock,now);
        heartbeat.freshness=now<=heartbeat.original_valid_until_us?HeartbeatFreshness::Fresh:HeartbeatFreshness::Expired;
        if(heartbeat.freshness!=HeartbeatFreshness::Fresh)return fail(RegisteredContextFault::Control,now);
    }
    const auto *source=io.latest_captured_snapshot();
    const bool source_observed=source&&io.diagnostics().fault==Fault::None;
    const auto sample=source_observed?source->estimator().timestamp_sample_us:0;
    const bool fresh_source=sample&&sample<=now&&now-sample<=configuration_.module.execution.limits.sample_max_age_us;
    // A new HB waits for one actual fresh private capture. Already-emitted HB
    // liveness uses its own original100ms window plus existing source faults;
    // the execution5ms sample limit is NOT a continuous >200Hz source gate.
    auto candidate=published_heartbeat_;
    if(heartbeat.receipt_generation==published_heartbeat_.receipt_generation||
       (allow_emit&&fresh_source))candidate=heartbeat;
    PrestreamPrerequisites prerequisites{};
    prerequisites.session=prerequisites.physical=PrestreamFact::Pass;
    prerequisites.atomic_source=source_observed?PrestreamFact::Pass:PrestreamFact::Unknown;
    prerequisites.land=observed.native_land_mode==GuardFact::Pass;
    prerequisites.fault=io.diagnostics().fault!=Fault::None;
    const auto decision=prestream_.poll(now,candidate,prerequisites);
    if(decision.action==PrestreamAction::Revoked)return fail(RegisteredContextFault::Control,now);
    if(decision.action!=PrestreamAction::EmitDirect)return true;
    offboard_control_mode_s output{};output.timestamp=decision.offboard.timestamp;
    output.position=decision.offboard.position;output.velocity=decision.offboard.velocity;
    output.acceleration=decision.offboard.acceleration;output.attitude=decision.offboard.attitude;
    output.body_rate=decision.offboard.body_rate;output.thrust_and_torque=decision.offboard.thrust_and_torque;
    output.direct_actuator=decision.offboard.direct_actuator;
    // This one emission must still be backed by its original fresh source at
    // publication admission. It does NOT shorten OCM's original HB expiry or
    // impose a persistent sample-age requirement between heartbeat events.
    const auto source_admission_until=guard_original_expiry(sample,configuration_.module.execution.limits.sample_max_age_us);
    const auto before_publication=hrt_absolute_time();
    if(!source_admission_until||before_publication>source_admission_until||
       before_publication>decision.original_valid_until_us||before_publication>observed.identity_valid_until_us)
        return fail(RegisteredContextFault::Control,before_publication);
    ++diagnostics_.offboard_publication_attempts;
    if(!offboard_output_.publish(output))return fail(RegisteredContextFault::Control,hrt_absolute_time());
    ++diagnostics_.offboard_publications;published_heartbeat_=candidate;
    diagnostics_.last_offboard_original_hrt_us=output.timestamp;
    diagnostics_.last_offboard_original_valid_until_us=decision.original_valid_until_us;
    const auto completed=hrt_absolute_time();
    if(completed>source_admission_until||completed>decision.original_valid_until_us||completed>observed.identity_valid_until_us)
        return fail(RegisteredContextFault::Control,completed); // retain occurred publication count
    return true;
}
bool RegisteredCanonicalModuleContext::bind_output(gpenmpc_rfly_stream::Authority &owner) noexcept
{
    return outputs_&&outputs_->bind(echo_.link,owner,output_registration_)==gpenmpc_rfly_stream::Bind::Registered;
}
bool RegisteredCanonicalModuleContext::bind_routes() noexcept
{
    if(routes_attempted_)return routes_bound_;
    routes_attempted_=true;
    if(!bind_output(direct_))return fail(RegisteredContextFault::Registry,hrt_absolute_time());
    if(snapshots_->bind(echo_.link,pump_.snapshot_outbox(),snapshot_registration_)!=gpenmpc_rfly_stream::SnapshotBind::Bound)
        return fail(RegisteredContextFault::Registry,hrt_absolute_time());
    if(feedback_->bind(echo_.link,pump_.feedback_outbox(),feedback_registration_)!=gpenmpc_rfly_stream::FeedbackBind::Bound)
        return fail(RegisteredContextFault::Registry,hrt_absolute_time());
    routes_bound_=true;return true;
}
ModulePoll RegisteredCanonicalModuleContext::poll(Px4CanonicalIo &io,std::uint64_t now) noexcept
{
    ++diagnostics_.polls;
    if(diagnostics_.closing||!diagnostics_.acquired||diagnostics_.first_fault!=RegisteredContextFault::None)return ModulePoll::Fault;
    if(io_&&io_!=&io){(void)fail(RegisteredContextFault::Configuration,now);return ModulePoll::Fault;}
    io_=&io;
    if(!bind_routes())return ModulePoll::Fault;
    BoardSafetyEvidence observed{};
    if(!session_and_physical(observed))return ModulePoll::Fault;
    if(!update_prestream(io,observed,false))return ModulePoll::Fault;
    ExchangePoll result=ExchangePoll::Fault;
    if(!diagnostics_.direct_entered && observed.disarmed_control==GuardFact::Pass){
        ++diagnostics_.observation_polls;result=pump_.poll_disarmed_observation(io);
    }else if(observed.require_active_direct()){
        // A HOST assertion/old OCM alone cannot bypass the original HB/source
        // prestream. Commander must still independently read back exclusivity.
        if(prestream_.state()!=PrestreamState::Enabled){(void)fail(RegisteredContextFault::Control,now);return ModulePoll::Fault;}
        if(!diagnostics_.direct_entered){
            if(!direct_.begin_direct()){(void)fail(RegisteredContextFault::Control,now);return ModulePoll::Fault;}
            diagnostics_.direct_entered=true;
        }
        ++diagnostics_.direct_polls;result=pump_.poll(io);
    }else{(void)fail(RegisteredContextFault::Control,now);return ModulePoll::Fault;}
    if(result==ExchangePoll::Fault){(void)fail(RegisteredContextFault::Exchange,now);return ModulePoll::Fault;}
    if(!update_prestream(io,observed,true))return ModulePoll::Fault;
    return result==ExchangePoll::Progress?ModulePoll::Progress:ModulePoll::Idle;
}
bool RegisteredCanonicalModuleContext::detach_output() noexcept
{
    if(!output_registration_.generation)return true;
    const auto result=outputs_->unbind(output_registration_);
    if(result!=gpenmpc_rfly_stream::Detach::Detached && result!=gpenmpc_rfly_stream::Detach::NotBound)return false;
    output_registration_={};return true;
}
bool RegisteredCanonicalModuleContext::detach_observation_routes() noexcept
{
    bool ok=true;
    if(snapshot_registration_.generation){
        const auto r=snapshots_->unbind(snapshot_registration_);
        if(r==gpenmpc_rfly_stream::SnapshotDetach::Detached||r==gpenmpc_rfly_stream::SnapshotDetach::NotBound)snapshot_registration_={};
        else ok=false;
    }
    if(feedback_registration_.generation){
        const auto r=feedback_->unbind(feedback_registration_);
        if(r==gpenmpc_rfly_stream::FeedbackDetach::Detached||r==gpenmpc_rfly_stream::FeedbackDetach::NotBound)feedback_registration_={};
        else ok=false;
    }
    return ok;
}
ModuleCloseResult RegisteredCanonicalModuleContext::close(ModuleStopReason reason,std::uint64_t now) noexcept
{
    (void)reason;ModuleCloseResult out{};
    if(detached_){out.route=ModuleDetach::Detached;out.plant=ModulePlantDisposition::DisarmedObserved;return out;}
    if(!diagnostics_.closing){
        prestream_.stop();
        diagnostics_.closing=true;diagnostics_.close_started_us=now;
        if(!now||configuration_.native_land_tail_max_us>UINT64_MAX-now){(void)fail(RegisteredContextFault::Clock,now);return out;}
        diagnostics_.original_land_deadline_us=now+configuration_.native_land_tail_max_us;
        direct_.revoke(); // Revoke the direct-output lease.
    }
    if(!direct_detached_){if(!detach_output())return out;direct_detached_=true;}
    if(!observation_detached_){
        if(!detach_observation_routes())return out;
        observation_detached_=true;
        if(io_){
            diagnostics_.actual_publication_attempts=io_->diagnostics().publish_attempts;
            diagnostics_.actual_publication_successes=io_->diagnostics().publish_succeeded;
            pump_.stop(*io_); // revokes, but deliberately retains already committed raw data
            diagnostics_.failed_raw_feedback_retained=
                io_->take_committed_feedback(failed_feedback_)!=FeedbackDisposition::Empty;
            // Unbind has quiesced the stream consumer and interrupted any
            // unfinished immutable RFC1 batch. Preserve it separately from a
            // later failed-execute Io latch; neither actual record overwrites
            // the other and neither is called a downstream acknowledgement.
            diagnostics_.interrupted_raw_feedback_retained=
                pump_.feedback_outbox().take_interrupted_raw(interrupted_feedback_)!=FeedbackDisposition::Empty;
        }
    }
    BoardSafetyEvidence observed{};
    if(!session_and_physical(observed))return out;
    vehicle_status_s status{};vehicle_control_mode_s mode{};vehicle_land_detected_s landed{};
    if(!status_.copy(&status)||!mode_.copy(&mode)||
       !fresh_context(status.timestamp,now,configuration_.module.telemetry_max_age_us)||
       !fresh_context(mode.timestamp,now,configuration_.module.telemetry_max_age_us))return out;
    out.original_observation_us=observed.observation_us;out.route=ModuleDetach::Pending;
    if(landed_.copy(&landed)&&fresh_context(landed.timestamp,now,configuration_.module.telemetry_max_age_us)&&landed.landed)
        diagnostics_.native_landed_observed=true;
    if(observed.disarmed_control==GuardFact::Pass&&disarmed_control_shape(status,mode)){
        diagnostics_.board_disarmed_observed=true;out.plant=ModulePlantDisposition::DisarmedObserved;
        if(!diagnostics_.actual_publication_attempts){
            // No control publication occurred in this context. Do not wait for
            // an artificial native zero message when virtual mappings were
            // never enabled. External finally still verifies plant/cache zero.
            native_land_.revoke();disarmed_zero_.revoke();if(!detach_output())return out;
            diagnostics_.no_own_publication_disarmed_detach=true;
            if(!release_reservation())return out;
            detached_=true;out.route=ModuleDetach::Detached;return out;
        }
        if(!zero_selected_){
            native_land_.revoke();if(!detach_output())return out;
            // After disarm, select the separate zero-only safety source.
            if(!zero_begun_){
                const auto original_zero_deadline=guard_original_expiry(status.timestamp,configuration_.module.telemetry_max_age_us);
                if(!disarmed_zero_.begin_observed_safety_state(original_zero_deadline,status,mode))return out;
                zero_begun_=true;
            }
            // A failed bind never repeats begin or renews its original expiry.
            if(!bind_output(disarmed_zero_))return out;
            zero_selected_=true;
        }
        NativeLandDiagnostics zero{};
        if(disarmed_zero_.diagnostic_snapshot(zero)&&zero.disarmed_zero_accepted){
            diagnostics_.virtual_zero_stream_accepted=true;
            disarmed_zero_.revoke();if(!detach_output())return out;
            // Actual wire/plant zero confirmation remains external; this only
            // permits destroying the detached callback-owning module storage.
            if(!release_reservation())return out;
            detached_=true;out.route=ModuleDetach::Detached;
        }
        return out;
    }
    if(now>diagnostics_.original_land_deadline_us){
        (void)fail(RegisteredContextFault::NativeTailDeadline,now);native_land_.revoke();
        (void)detach_output();return out; // keep storage; external emergency recovery must observe disarm
    }
    if(observed.native_land_mode==GuardFact::Pass&&native_land_mode_shape(status,mode)){
        diagnostics_.native_land_observed=true;out.plant=ModulePlantDisposition::NativeLandingObserved;
        if(!native_selected_){
            if(!native_begun_){
                if(!native_land_.begin_observed_safety_state(diagnostics_.original_land_deadline_us,status,mode))return out;
                native_begun_=true;
            }
            if(!bind_output(native_land_))return out;
            native_selected_=true;
        }
    }
    return out;
}
} // namespace gpenmpc_rfly_px4
