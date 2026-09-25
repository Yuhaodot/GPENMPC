#include "CanonicalLocalApplicationOwner.hpp"

namespace gpenmpc_rfly_local_px4 {
LocalApplicationResult CanonicalLocalApplicationOwner::prepare(Mavlink&actual,
    const SessionBoardIdentity&expected,const HostSessionChallenge&challenge,
    const RegisteredLocalContextConfiguration&profile,SessionEcho&echo)noexcept
{
    echo={};
    if(record_.state!=LocalApplicationState::Empty||guard_||context_)
        return LocalApplicationResult::Rejected;
#if !defined(GPENMPC_CANONICAL_CLOSED_EVIDENCE)
    // Require the canonical controller ABI for this application.
    (void)actual;(void)expected;(void)challenge;(void)profile;
    return LocalApplicationResult::Rejected;
#else
    constexpr gpenmpc_full_consumption::Hash config_sha{{0xA859433Du,0x0AA77401u,0x3341444Au,0x4B9AE971u,0xA34F12B4u,0xFB89C0A0u,0x04B2CB3Eu,0x013FCEBAu}};
    gpenmpc_full_inner_build_identity linked{};gpenmpc_full_inner_identity(&linked);
    const auto&e=profile.module.execution;
    if(gpenmpc_full_consumption::detail::words(e.numerical.configuration_sha256)!=config_sha||
       std::memcmp(e.build.generated_source_set_sha256,linked.generated_source_set_sha256,32)||
       std::memcmp(e.build.private_archive_sha256,linked.private_archive_sha256,32)||
       std::memcmp(e.build.facade_source_sha256,linked.facade_source_sha256,32)||
       gpenmpc_full_consumption::detail::empty(gpenmpc_full_consumption::detail::words(linked.generated_source_set_sha256))||
       gpenmpc_full_consumption::detail::empty(gpenmpc_full_consumption::detail::words(linked.private_archive_sha256))||
       gpenmpc_full_consumption::detail::empty(gpenmpc_full_consumption::detail::words(linked.facade_source_sha256))||
       !profile.module.telemetry_max_age_us||
       profile.module.commander_telemetry_max_age_us<profile.module.telemetry_max_age_us||
       !profile.native_land_tail_max_us||
       !expected.uid||!expected.system||!expected.component||
       profile.transport.target_system!=expected.system||profile.transport.target_component!=expected.component||
       profile.task.owner||profile.task.read||profile.task.window||profile.task.window_receiver||profile.task.task_receiver||profile.task.observe_source)
        return LocalApplicationResult::Rejected;
    // Local source-specific contract: actual Commander 500 ms publication
    // period plus the existing 100 ms engineering margin. Offboard/power keep
    // profile.module.telemetry_max_age_us; odometry/HIL retain their 5 ms gate.
    guard_=Px4ExecutionSessionGuard::create(actual,expected,config_sha,
        profile.module.telemetry_max_age_us,profile.module.commander_telemetry_max_age_us);
    if(!guard_)return LocalApplicationResult::Unavailable;
    if(!guard_->register_session(challenge,record_.echo)){
        record_.session_observed=guard_->evidence_snapshot(record_.session);
        // A failed revoke is a real lifetime boundary: retain, do not delete.
        if(guard_->revoke()){delete guard_;guard_=nullptr;}
        record_.state=LocalApplicationState::Retired;return LocalApplicationResult::Rejected;
    }
    profile_=profile;
    const auto&r=record_.echo;
    const Identity observed{r.observed_identity.uid,r.process_session_generation,
        r.observed_identity.system,r.observed_identity.component};
    profile_.module.source.identity=observed;profile_.module.execution.identity=observed;
    profile_.module.authority=nullptr;
    profile_.phase.identity=observed;profile_.cycle.context.observed_session=observed;
    profile_.original_source.independently_observed_board_identity=observed;
    profile_.original_source.expected_registered_link=r.link.pointer;
    profile_.original_source.expected_receiver_instance=profile_.transport.receiver_instance;
    record_.state=LocalApplicationState::Registered;echo=r;
    return LocalApplicationResult::Ready;
#endif
}
LocalApplicationResult CanonicalLocalApplicationOwner::bind_inputs(const LocalTaskInputPort&port)noexcept
{
    if(record_.state!=LocalApplicationState::Registered||!guard_||context_||
       !port.owner||!port.read||!port.window||!port.window_receiver||!port.task_receiver||!port.observe_source)
        return LocalApplicationResult::Rejected;
    // The concrete receiver's source/link/hash checks remain in the Core.
    // Lifetime binding does not replace source availability checks.
    profile_.task=port;record_.state=LocalApplicationState::InputsBound;
    return LocalApplicationResult::Ready;
}
LocalApplicationResult CanonicalLocalApplicationOwner::confirm(const SessionEcho&echo,
    const SessionPhysicalDeclaration&declaration)noexcept
{
    if(record_.state!=LocalApplicationState::InputsBound||!guard_||context_)
        return LocalApplicationResult::Rejected;
    if(!guard_->confirm_echo(echo)||!guard_->bind_physical_declaration(declaration))
        return LocalApplicationResult::Rejected;
    context_=new RegisteredCanonicalLocalModuleContext(*guard_,record_.echo,profile_);
    if(!context_)return LocalApplicationResult::Unavailable;
    record_.context=context_->diagnostics_after_stop();
    if(record_.context.first_fault!=RegisteredLocalContextFault::None){
        if(context_->storage_releasable()){delete context_;context_=nullptr;}
        return LocalApplicationResult::Rejected;
    }
    record_.state=LocalApplicationState::Confirmed;return LocalApplicationResult::Ready;
}
LocalApplicationResult CanonicalLocalApplicationOwner::start()noexcept
{
    if(record_.state!=LocalApplicationState::Confirmed||!context_||!guard_)
        return LocalApplicationResult::Rejected;
    if(!GPENMPCRflyCanonicalLocalModule::configure(context_))return LocalApplicationResult::Unavailable;
    record_.state=LocalApplicationState::Installed;++record_.start_requests;
    char name[]="gpenmpc_rfly_canonical_local",command[]="start";char*argv[]{name,command,nullptr};
    record_.last_start_return=GPENMPCRflyCanonicalLocalModule::main(2,argv);
    // acquire is complete before any task spawn; only this immutable member
    // is copied while a successfully spawned task could already be polling.
    record_.context.acquire=context_->acquire_evidence();
    record_.last_task_created=GPENMPCRflyCanonicalLocalModule::last_task_created();
    return record_.last_start_return==0?LocalApplicationResult::Ready:LocalApplicationResult::Rejected;
}
LocalApplicationResult CanonicalLocalApplicationOwner::stop()noexcept
{
    if(record_.state!=LocalApplicationState::Installed||!context_)return LocalApplicationResult::Rejected;
    ++record_.stop_requests;
    char name[]="gpenmpc_rfly_canonical_local",command[]="stop";char*argv[]{name,command,nullptr};
    record_.last_stop_return=GPENMPCRflyCanonicalLocalModule::main(2,argv);
    return record_.last_stop_return==0?LocalApplicationResult::Pending:LocalApplicationResult::Unavailable;
}
bool CanonicalLocalApplicationOwner::observe(LocalApplicationObservation&out)noexcept
{
    if(guard_)record_.session_observed=guard_->evidence_snapshot(record_.session);
    // Never read task-owned numerical state/counters while it may be running.
    out=record_;return !guard_||record_.session_observed;
}
LocalApplicationResult CanonicalLocalApplicationOwner::release(LocalApplicationObservation&out)noexcept
{
    if(record_.state==LocalApplicationState::Installed&&!GPENMPCRflyCanonicalLocalModule::configure(nullptr)){
        (void)observe(out);return LocalApplicationResult::Pending;
    }
    if(context_){
        if(!context_->storage_releasable()){(void)observe(out);return LocalApplicationResult::Pending;}
        record_.context=context_->diagnostics_after_stop();record_.context_observed_after_stop=true;
        record_.failed_feedback=context_->failed_raw_feedback_after_stop();
        record_.late_feedback=context_->late_raw_feedback_after_stop();
        record_.interrupted_wire=context_->interrupted_wire_after_stop();
        delete context_;context_=nullptr;
    }
    if(guard_){
        record_.session_observed=guard_->evidence_snapshot(record_.session);
        if(!guard_->revoke()){out=record_;return LocalApplicationResult::Unavailable;}
        delete guard_;guard_=nullptr;
    }
    // Only now may the caller destroy its receiver, input owner and scratch.
    profile_.task={};record_.state=LocalApplicationState::Retired;out=record_;
    return LocalApplicationResult::Detached;
}
}
