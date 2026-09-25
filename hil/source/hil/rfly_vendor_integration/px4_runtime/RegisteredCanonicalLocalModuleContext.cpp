#include "RegisteredCanonicalLocalModuleContext.hpp"
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <new>

namespace gpenmpc_rfly_local_px4 {
namespace {
Identity echo_identity(const SessionEcho &e) noexcept
{return {e.observed_identity.uid,e.process_session_generation,e.observed_identity.system,e.observed_identity.component};}
bool fresh_context(std::uint64_t stamp,std::uint64_t now,std::uint64_t age) noexcept
{return stamp&&stamp<=now&&guard_original_expiry(stamp,age)>=now;}
}
RegisteredCanonicalLocalModuleContext::RegisteredCanonicalLocalModuleContext(RegisteredLocalContextGuard &guard,
    const SessionEcho &echo,const RegisteredLocalContextConfiguration &c) noexcept:
    guard_(guard),echo_(echo),configuration_(c),
    direct_(guard_,clock_,echo_identity(echo),c.module.telemetry_max_age_us,
        c.module.commander_telemetry_max_age_us),
    native_land_(guard_,clock_,echo_identity(echo),c.module.telemetry_max_age_us,
        c.module.commander_telemetry_max_age_us),
    disarmed_zero_(guard_,clock_,echo_identity(echo),c.module.telemetry_max_age_us,
        c.module.commander_telemetry_max_age_us),
    source_(c.original_source),selected_(source_,echo_identity(echo)),phase_(c.phase)
{
    if(!c.native_land_tail_max_us||!c.module.telemetry_max_age_us||
       c.module.commander_telemetry_max_age_us<c.module.telemetry_max_age_us||!c.module.poll_period_us||
       c.module.task_stack_bytes<=2240||!echo.link.pointer||!echo.link.generation||
       !(c.module.source.identity==echo_identity(echo))||!(c.module.execution.identity==echo_identity(echo))||
       c.module.source.vehicle_odometry_topic!=ORB_ID(vehicle_odometry)||
       c.transport.receiver_instance>=ORB_MULTI_MAX_INSTANCES||
       c.transport.target_system!=echo.observed_identity.system||c.transport.target_component!=echo.observed_identity.component||
       gpenmpc_full_consumption::detail::words(c.module.execution.numerical.configuration_sha256)!=echo.configuration_sha256||
       phase_.diagnostics().first_fault!=gpenmpc_local_phase::Fault::None||source_.fault()!=gpenmpc_hil_endpoint_reader::Fault::None||
       c.original_source.expected_registered_link!=echo.link.pointer||
       c.original_source.expected_receiver_instance!=c.transport.receiver_instance||
       !(c.original_source.independently_observed_board_identity==echo_identity(echo))||
       !(c.phase.identity==echo_identity(echo))||!(c.cycle.context.observed_session==echo_identity(echo))||
       c.phase.leg_index!=c.module.execution.numerical.leg_index||c.cycle.context.explicit_leg!=c.phase.leg_index||
       c.phase.configuration_sha256!=echo.configuration_sha256||c.cycle.context.configuration_sha256!=echo.configuration_sha256||
       std::memcmp(c.phase.reference_asset_sha256,c.module.execution.numerical.reference_asset_sha256,32)||
       c.cycle.context.reference_asset_sha256!=gpenmpc_full_consumption::detail::words(c.phase.reference_asset_sha256)||
       c.cycle.context.task_sha256!=gpenmpc_full_consumption::detail::words(c.module.execution.numerical.task_sha256)||
       !c.gp_request_transport_max_age_us||!c.snapshot_transport_max_age_us)
        (void)fail(RegisteredLocalContextFault::Configuration,hrt_absolute_time());
}
RegisteredCanonicalLocalModuleContext::~RegisteredCanonicalLocalModuleContext(){
    // Owning application may destroy only after storage_releasable and actual
    // Module configure(nullptr). No side-effecting forced cleanup here.
    if(core_)core_->~CanonicalLocalExchangeCore();
    if(cycle_)cycle_->~CanonicalLocalExecutionCycle();
}
bool RegisteredCanonicalLocalModuleContext::attach(Px4CanonicalLocalIo&io)noexcept{
    if(io_)return io_==&io&&core_&&cycle_;
    io_=&io;
    cycle_=new(cycle_storage_)CanonicalLocalExecutionCycle(io,phase_,source_,configuration_.cycle);
    LocalExchangeConfiguration c{};c.transport=configuration_.transport;c.ingress_topic_instance=configuration_.ingress_topic_instance;
    c.identity=configuration_.module.source.identity;c.gp_request_transport_max_age_us=configuration_.gp_request_transport_max_age_us;
    c.snapshot_transport_max_age_us=configuration_.snapshot_transport_max_age_us;
    core_=new(core_storage_)CanonicalLocalExchangeCore(io,*cycle_,selected_,configuration_.task,c);
    return (cycle_->diagnostics().first_fault==LocalCycleFault::None&&core_->diagnostics().first_fault==LocalExchangeFault::None)||
        fail(RegisteredLocalContextFault::Exchange,hrt_absolute_time());
}
bool RegisteredCanonicalLocalModuleContext::copy_owner_state(LocalContextStateObservation&out)const noexcept{
    out={};if(!io_||!io_->copy_local_numerical_state(out.numerical))return false;
    out.phase=phase_.diagnostics();const auto&s=out.numerical.state;
    out.phase_matches_committed_source=s.numeric_installed&&s.reference_committed&&
        out.phase.last_source_timestamp_ns==s.original_installed_tags2[0]&&
        out.phase.last_source_generation==s.original_installed_tags2[1]&&
        out.phase.last_output_generation==s.reference.output_generation&&
        out.phase.last_publication_us==s.reference.publication_us;
    return true;
}
bool RegisteredCanonicalLocalModuleContext::fail(RegisteredLocalContextFault f,std::uint64_t now) noexcept
{
    prestream_.stop(); // permanent for this Context, never poisons SessionGuard identity
    if(diagnostics_.first_fault==RegisteredLocalContextFault::None){diagnostics_.first_fault=f;diagnostics_.first_fault_us=now;}
    return false;
}
bool RegisteredCanonicalLocalModuleContext::session_and_physical(BoardSafetyEvidence &out) noexcept
{
    SessionGuardEvidence session{};
    if(!guard_.evidence_snapshot(session)||session.first_fault!=SessionFault::None||
       !session.registered||!session.echo_confirmed||!session.human_declaration_bound||
       !same_session_echo(session.registration,echo_)||
       !guard_.validate_ticket_binding({execution_session_digest(echo_),echo_.process_session_generation})||
       !guard_.observe(out)||!(out.identity==echo_identity(echo_)))
        return fail(RegisteredLocalContextFault::Session,hrt_absolute_time());
    if(!out.require_physical_facts()||out.usb_transport!=GuardFact::Pass||out.hil_configuration!=GuardFact::Pass)
        return fail(RegisteredLocalContextFault::Physical,hrt_absolute_time());
    return true;
}
ModuleAcquire RegisteredCanonicalLocalModuleContext::acquire(ModuleConfiguration &out) noexcept
{
    out={};
    auto &a=diagnostics_.acquire;
    const bool first=!a.attempted;
    if(first)a.attempted=true;
    const auto finish=[&](ModuleAcquire result,unsigned reason) noexcept {
        if(first){a.result=unsigned(result);a.reason=reason;
            a.first_context_fault=diagnostics_.first_fault;
            a.first_context_fault_us=diagnostics_.first_fault_us;}
        return result;
    };
    if(diagnostics_.first_fault!=RegisteredLocalContextFault::None)return finish(ModuleAcquire::Rejected,1);
    if(diagnostics_.acquire_aborted)return finish(ModuleAcquire::Rejected,2);
    if(diagnostics_.acquired){(void)fail(RegisteredLocalContextFault::DuplicateAcquire,hrt_absolute_time());return finish(ModuleAcquire::Rejected,3);}
    BoardSafetyEvidence observed{};
    const bool safety_ok=session_and_physical(observed);
    if(first){a.observed=observed;a.guard_snapshot_valid=guard_.evidence_snapshot(a.guard);}
    if(!safety_ok)return finish(ModuleAcquire::Unavailable,4);
    bool legacy_stopped=false;
    if(observed.native_recovery_configuration==GuardFact::Pass){
        legacy_stopped=legacy_trajectory_stopped_with_module_lock();
        if(first)a.legacy_stopped=unsigned(legacy_stopped);
    }
    if(observed.native_recovery_configuration!=GuardFact::Pass||!legacy_stopped){
        (void)fail(RegisteredLocalContextFault::Control,hrt_absolute_time());return finish(ModuleAcquire::Rejected,5);
    }
    // Singleton access creates no output/route and requires no asynchronous
    // cleanup if a subsequent ModuleBase allocation/spawn is rejected.
    outputs_=gpenmpc_rfly_stream::shared_output_registry();wire_=gpenmpc_rfly_stream::canonical_local_wire_route_registry();
    if(first){a.outputs_present=unsigned(outputs_!=nullptr);a.wire_present=unsigned(wire_!=nullptr);}
    if(!outputs_||!wire_)return finish(ModuleAcquire::Unavailable,6);
    const bool outputs_ready=outputs_->ready();
    if(first)a.outputs_ready=unsigned(outputs_ready);
    if(!outputs_ready)return finish(ModuleAcquire::Unavailable,6);
    const bool wire_ready=wire_->ready();
    if(first)a.wire_ready=unsigned(wire_ready);
    if(!wire_ready)return finish(ModuleAcquire::Unavailable,6);
    links_=gpenmpc_rfly_stream::link_lifetime_registry();
    if(first)a.links_present=unsigned(links_!=nullptr);
    auto bound=gpenmpc_rfly_stream::CanonicalBind::Unavailable;
    if(links_){bound=links_->bind_canonical(echo_,configuration_.transport.source_system,
        configuration_.transport.source_component,this,reservation_);if(first)a.bind_result=unsigned(bound);}
    if(!links_||bound!=gpenmpc_rfly_stream::CanonicalBind::Bound){
        (void)release_reservation();return finish(ModuleAcquire::Unavailable,7);
    }
    diagnostics_.reservation_bound=true;
    if(!prestream_.start(reservation_.heartbeat,configuration_.module.telemetry_max_age_us)){
        (void)release_reservation();(void)fail(RegisteredLocalContextFault::Configuration,hrt_absolute_time());return finish(ModuleAcquire::Rejected,8);
    }
    out=configuration_.module;out.authority=&direct_;diagnostics_.acquired=true;return finish(ModuleAcquire::Ready,0);
}
bool RegisteredCanonicalLocalModuleContext::release_reservation() noexcept
{
    if(!reservation_.generation)return true;
    if(!links_||links_->release_canonical(reservation_)!=gpenmpc_rfly_stream::LinkAccess::Read)return false;
    reservation_={};diagnostics_.reservation_bound=false;return true;
}
bool RegisteredCanonicalLocalModuleContext::abort_acquire() noexcept
{
    // Only ModuleBase's synchronous failed-start path (or still-uninstalled
    // application cleanup) may use this. Never hide a live route/task/OCM.
    if(io_||routes_attempted_||diagnostics_.offboard_publication_attempts)return false;
    prestream_.stop();diagnostics_.acquire_aborted=true;
    return release_reservation();
}
bool RegisteredCanonicalLocalModuleContext::update_prestream(Px4CanonicalLocalIo &io,
    const BoardSafetyEvidence &observed,bool allow_emit) noexcept
{
    using namespace gpenmpc_rfly_stream;
    HostHeartbeatSnapshot heartbeat{};
    if(!links_||links_->read_heartbeat(reservation_,0,configuration_.module.telemetry_max_age_us,heartbeat)!=LinkAccess::Read)
        return fail(RegisteredLocalContextFault::Registry,hrt_absolute_time());
    // Copy THEN sample processing time: a heartbeat can arrive while this poll
    // runs. This only classifies copied raw fields; never re-record/re-stamp it.
    const auto now=hrt_absolute_time();
    if(!now||observed.identity_valid_until_us<now||observed.native_recovery_configuration!=GuardFact::Pass)
        return fail(RegisteredLocalContextFault::Control,now);
    if(heartbeat.receipt_generation){
        if(!heartbeat.original_receiver_hrt_us||!heartbeat.original_valid_until_us||heartbeat.original_receiver_hrt_us>now)
            return fail(RegisteredLocalContextFault::Clock,now);
        heartbeat.freshness=now<=heartbeat.original_valid_until_us?HeartbeatFreshness::Fresh:HeartbeatFreshness::Expired;
        if(heartbeat.freshness!=HeartbeatFreshness::Fresh)return fail(RegisteredLocalContextFault::Control,now);
    }
    // During disarmed observation, capture the newly arrived state before
    // evaluating a previously published HB against that source. The post-core
    // pass below is mandatory in this same poll and retains all original
    // source/HB expiries. Actual current HB/identity faults above still stop
    // immediately; active control, LAND and IO faults never take this path.
    if(!allow_emit&&!diagnostics_.direct_entered&&
       observed.disarmed_control==GuardFact::Pass&&observed.native_land_mode!=GuardFact::Pass&&
       io.diagnostics().first_fault==LocalFault::None)return true;
    const auto *source=io.retained_latest_snapshot();
    const bool source_observed=source&&io.diagnostics().first_fault==LocalFault::None;
    const auto sample=source_observed?source->estimator().timestamp_sample_us:0;
    const bool fresh_source=sample&&sample<=now&&now-sample<=configuration_.module.execution.limits.sample_max_age_us;
    // Initial prestream requires a fresh private capture. In direct control, OCM
    // tracks mode and heartbeat lifetime independently of the next capture.
    // Each control validates source, compute, input and output limits.
    const bool initial_source_admission=!diagnostics_.direct_entered;
    auto candidate=published_heartbeat_;
    if(heartbeat.receipt_generation==published_heartbeat_.receipt_generation||
       (source_observed&&(!initial_source_admission||(allow_emit&&fresh_source))))candidate=heartbeat;
    PrestreamPrerequisites prerequisites{};
    prerequisites.session=prerequisites.physical=PrestreamFact::Pass;
    prerequisites.atomic_source=source_observed?PrestreamFact::Pass:PrestreamFact::Unknown;
    prerequisites.land=observed.native_land_mode==GuardFact::Pass;
    prerequisites.fault=io.diagnostics().first_fault!=LocalFault::None;
    const auto decision=prestream_.poll(now,candidate,prerequisites);
    if(decision.action==PrestreamAction::Revoked)return fail(RegisteredLocalContextFault::Control,now);
    if(decision.action!=PrestreamAction::EmitDirect)return true;
    offboard_control_mode_s output{};output.timestamp=decision.offboard.timestamp;
    output.position=decision.offboard.position;output.velocity=decision.offboard.velocity;
    output.acceleration=decision.offboard.acceleration;output.attitude=decision.offboard.attitude;
    output.body_rate=decision.offboard.body_rate;output.thrust_and_torque=decision.offboard.thrust_and_torque;
    output.direct_actuator=decision.offboard.direct_actuator;
    // Only initial admission couples OCM to the fresh source. Active OCM
    // retains the original HB and identity deadlines, never authorizes a
    // numerical output or renews an estimator/input timestamp.
    const auto source_admission_until=guard_original_expiry(sample,configuration_.module.execution.limits.sample_max_age_us);
    const auto before_publication=hrt_absolute_time();
    if((initial_source_admission&&(!source_admission_until||before_publication>source_admission_until))||
       before_publication>decision.original_valid_until_us||before_publication>observed.identity_valid_until_us)
        return fail(RegisteredLocalContextFault::Control,before_publication);
    ++diagnostics_.offboard_publication_attempts;
    if(!offboard_output_.publish(output))return fail(RegisteredLocalContextFault::Control,hrt_absolute_time());
    ++diagnostics_.offboard_publications;published_heartbeat_=candidate;
    diagnostics_.last_offboard_original_hrt_us=output.timestamp;
    diagnostics_.last_offboard_original_valid_until_us=decision.original_valid_until_us;
    const auto completed=hrt_absolute_time();
    if((initial_source_admission&&completed>source_admission_until)||completed>decision.original_valid_until_us||completed>observed.identity_valid_until_us)
        return fail(RegisteredLocalContextFault::Control,completed); // retain occurred publication count
    return true;
}
bool RegisteredCanonicalLocalModuleContext::bind_output(gpenmpc_rfly_stream::Authority &owner) noexcept
{
    return outputs_&&outputs_->bind(echo_.link,owner,output_registration_)==gpenmpc_rfly_stream::Bind::Registered;
}
bool RegisteredCanonicalLocalModuleContext::bind_routes() noexcept
{
    if(routes_attempted_)return routes_bound_;
    routes_attempted_=true;
    if(!bind_output(direct_))return fail(RegisteredLocalContextFault::Registry,hrt_absolute_time());
    if(!core_||wire_->bind(echo_.link,core_->outbox(),wire_registration_)!=gpenmpc_rfly_stream::LocalWireBind::Bound)
        return fail(RegisteredLocalContextFault::Registry,hrt_absolute_time());
    routes_bound_=true;return true;
}
ModulePoll RegisteredCanonicalLocalModuleContext::poll(Px4CanonicalLocalIo &io,std::uint64_t now) noexcept
{
    ++diagnostics_.polls;
    if(diagnostics_.closing||!diagnostics_.acquired||diagnostics_.first_fault!=RegisteredLocalContextFault::None)return ModulePoll::Fault;
    if(io_&&io_!=&io){(void)fail(RegisteredLocalContextFault::Configuration,now);return ModulePoll::Fault;}
    if(!attach(io)||!bind_routes())return ModulePoll::Fault;
    BoardSafetyEvidence observed{};
    if(!session_and_physical(observed))return ModulePoll::Fault;
    if(!update_prestream(io,observed,false))return ModulePoll::Fault;
    LocalExchangePoll result=LocalExchangePoll::Fault;
    if(!diagnostics_.direct_entered && observed.disarmed_control==GuardFact::Pass){
        ++diagnostics_.observation_polls;result=core_->poll(true);
    }else if(observed.require_active_direct()){
        // A HOST assertion/old OCM alone cannot bypass the original HB/source
        // prestream. Commander must still independently read back exclusivity.
        if(prestream_.state()!=PrestreamState::Enabled){(void)fail(RegisteredLocalContextFault::Control,now);return ModulePoll::Fault;}
        if(!diagnostics_.direct_entered){
            if(!direct_.begin_direct()){(void)fail(RegisteredLocalContextFault::Control,now);return ModulePoll::Fault;}
            diagnostics_.direct_entered=true;
        }
        ++diagnostics_.direct_polls;result=core_->poll(false);
    }else{(void)fail(RegisteredLocalContextFault::Control,now);return ModulePoll::Fault;}
    if(result==LocalExchangePoll::Fault){(void)fail(RegisteredLocalContextFault::Exchange,now);return ModulePoll::Fault;}
    if(!update_prestream(io,observed,true))return ModulePoll::Fault;
    return result==LocalExchangePoll::Progress?ModulePoll::Progress:ModulePoll::Idle;
}
bool RegisteredCanonicalLocalModuleContext::detach_output() noexcept
{
    if(!output_registration_.generation)return true;
    const auto result=outputs_->unbind(output_registration_);
    if(result!=gpenmpc_rfly_stream::Detach::Detached && result!=gpenmpc_rfly_stream::Detach::NotBound)return false;
    output_registration_={};return true;
}
bool RegisteredCanonicalLocalModuleContext::detach_observation_routes() noexcept
{
    if(!wire_registration_.generation)return true;
    const auto r=wire_->unbind(wire_registration_);
    if(r!=gpenmpc_rfly_stream::LocalWireDetach::Detached&&r!=gpenmpc_rfly_stream::LocalWireDetach::NotBound)return false;
    wire_registration_={};return true;
}
ModuleCloseResult RegisteredCanonicalLocalModuleContext::close(ModuleStopReason reason,std::uint64_t now) noexcept
{
    ModuleCloseResult out{};
    if(detached_){out.route=ModuleDetach::Detached;out.plant=ModulePlantDisposition::DisarmedObserved;return out;}
    if(!diagnostics_.closing){
        diagnostics_.module_stop_reason=reason;
        prestream_.stop();
        diagnostics_.closing=true;diagnostics_.close_started_us=now;
        if(!now||configuration_.native_land_tail_max_us>UINT64_MAX-now){(void)fail(RegisteredLocalContextFault::Clock,now);return out;}
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
            if(core_){
                // Preserve the live first-fault hierarchy before stop() adds a
                // terminal Stopped disposition.  This is read-only diagnostic
                // state; it grants no authority and changes no recovery order.
                diagnostics_.exchange_before_stop=core_->diagnostics();
                diagnostics_.cycle_before_stop=cycle_->diagnostics();
                diagnostics_.local_io_before_stop=io_->diagnostics();
                diagnostics_.selected_source_before_stop=selected_.diagnostics();
                diagnostics_.original_source_diagnostics_observed=
                    source_.diagnostics(diagnostics_.original_source_before_stop);
                diagnostics_.nested_diagnostics_observed=true;
                // Emit the already retained first-fault hierarchy once, after
                // direct output and observation routes have been detached.
                // No extra admission, protocol, or work on a healthy cycle.
                if(diagnostics_.first_fault!=RegisteredLocalContextFault::None){
                    PX4_ERR("LOCAL_CTX f=%u e=%u c=%u w=%u p=%u t=%llu n=%llu",
                        unsigned(diagnostics_.first_fault),
                        unsigned(diagnostics_.exchange_before_stop.first_fault),
                        unsigned(diagnostics_.cycle_before_stop.first_fault),
                        unsigned(core_->outbox().fault()),unsigned(prestream_.first_reason()),
                        (unsigned long long)diagnostics_.first_fault_us,
                        (unsigned long long)diagnostics_.exchange_before_stop.commits);
                    // Read the already-existing authority first fault only
                    // after detachment; no added work on a healthy control tick.
                    // IO Ownership alone conflates an actual control change,
                    // an expired guard and an unconsumed stream publication.
                    LeaseDiagnostics lease{};
                    if(direct_.diagnostic_snapshot(lease))
                        PX4_ERR("LOCAL_AUTH f=%u t=%llu v=%llu i=%llu c=%llu a=%llu consumed=%u out=%llu expiry=%llu",
                            unsigned(lease.first_fault),(unsigned long long)lease.first_fault_us,
                            (unsigned long long)lease.validated,(unsigned long long)lease.installed,
                            (unsigned long long)lease.confirmed,(unsigned long long)lease.accepted,
                            unsigned(lease.consumed),(unsigned long long)lease.last_consumed_output_us,
                            (unsigned long long)lease.original_expiry_us);
                }
                const auto &gp=core_->pending_after_stop().diagnostics();
                PX4_INFO("LOCAL_GP requests=%llu fills=%llu deadline=%llu busy=%llu late=%llu",
                    (unsigned long long)gp.requests,(unsigned long long)gp.fills,
                    (unsigned long long)gp.deadline_unavailable,(unsigned long long)gp.busy_unavailable,
                    (unsigned long long)gp.late_replies_not_installed);
                core_->stop();
                failed_feedback_=core_->pending_after_stop().retained_actual_feedback();
                diagnostics_.failed_raw_feedback_retained=failed_feedback_.present;
                diagnostics_.interrupted_raw_feedback_retained=core_->outbox().audit_copy(interrupted_wire_);
            }
            diagnostics_.late_raw_feedback_retained=io_->take_execution_feedback(late_feedback_);
        }
    }
    BoardSafetyEvidence observed{};
    if(!session_and_physical(observed))return out;
    vehicle_status_s status{};vehicle_control_mode_s mode{};vehicle_land_detected_s landed{};
    if(!status_.copy(&status)||!mode_.copy(&mode)||
       !fresh_context(status.timestamp,now,configuration_.module.commander_telemetry_max_age_us)||
       !fresh_context(mode.timestamp,now,configuration_.module.commander_telemetry_max_age_us))return out;
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
                const auto original_zero_deadline=guard_original_expiry(status.timestamp,
                    configuration_.module.commander_telemetry_max_age_us);
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
        (void)fail(RegisteredLocalContextFault::NativeTailDeadline,now);native_land_.revoke();
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
} // namespace gpenmpc_rfly_local_px4
