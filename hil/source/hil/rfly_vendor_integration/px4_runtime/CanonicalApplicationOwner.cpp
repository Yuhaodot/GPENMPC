#include "CanonicalApplicationOwner.hpp"
#include "../application_parameters/CanonicalApplicationParameters.hpp"

namespace gpenmpc_rfly_px4 {
ApplicationResult CanonicalApplicationOwner::prepare(Mavlink &actual,
    const SessionBoardIdentity &expected,const HostSessionChallenge &challenge,
    const RegisteredContextConfiguration &profile,SessionEcho &echo) noexcept
{
    echo={};
    if(record_.state!=ApplicationState::Empty||guard_||context_)return ApplicationResult::Rejected;
    const auto &e=profile.module.execution;
    // Science/configuration does not become valid merely by constructing a
    // profile with a matching label. The generated core and parameters bind it.
    if(e.configuration_payload_sha256!=gpenmpc_rfly_execution::kCanonicalConfigurationSha||
       e.kernel_source_sha256!=gpenmpc_rfly_execution::kGeneratedArmSourceSha||
       e.matlab_extraction_source_sha256!=gpenmpc_rfly_execution::kMatlabExtractionSha||
       e.wrapper_matlab_source_sha256!=gpenmpc_rfly_execution::kWrapperMatlabSourceSha||
       e.approved_parameter_sha256!=kCanonicalApplicationParameterSha||
       e.approved_parameter_sha256!=gpenmpc_rfly_execution::parameter_sha256(e.approved_parameters)||
       !profile.module.telemetry_max_age_us||!profile.native_land_tail_max_us||
       !expected.uid||!expected.system||!expected.component)
        return ApplicationResult::Rejected;
    guard_=Px4ExecutionSessionGuard::create(actual,expected,e.configuration_payload_sha256,
        profile.module.telemetry_max_age_us);
    if(!guard_)return ApplicationResult::Unavailable;
    if(!guard_->register_session(challenge,record_.echo)){
        (void)guard_->evidence_snapshot(record_.session);record_.session_observed=true;
        (void)guard_->revoke();delete guard_;guard_=nullptr;
        record_.state=ApplicationState::Retired;return ApplicationResult::Rejected;
    }
    profile_=profile;
    // Only the independently assigned session generation is installed here.
    // Do not carry any caller-supplied "boot_generation" into the source.
    const auto &r=record_.echo;
    const Identity observed{r.observed_identity.uid,r.process_session_generation,
        r.observed_identity.system,r.observed_identity.component};
    profile_.module.source.identity=observed;profile_.module.execution.identity=observed;
    profile_.module.authority=nullptr;
    record_.state=ApplicationState::Registered;echo=r;return ApplicationResult::Ready;
}
ApplicationResult CanonicalApplicationOwner::confirm(const SessionEcho &echo,
    const SessionPhysicalDeclaration &declaration) noexcept
{
    if(record_.state!=ApplicationState::Registered||!guard_||context_)return ApplicationResult::Rejected;
    if(!guard_->confirm_echo(echo)||!guard_->bind_physical_declaration(declaration))
        return ApplicationResult::Rejected;
    context_=new RegisteredCanonicalModuleContext(*guard_,record_.echo,profile_);
    if(!context_)return ApplicationResult::Unavailable;
    record_.context=context_->diagnostics_after_stop();
    if(record_.context.first_fault!=RegisteredContextFault::None){
        // Constructor has no routes/callbacks; rejecting here cannot require a
        // fictitious asynchronous plant-stop receipt.
        if(context_->storage_releasable()){delete context_;context_=nullptr;}
        return ApplicationResult::Rejected;
    }
    record_.state=ApplicationState::Confirmed;return ApplicationResult::Ready;
}
ApplicationResult CanonicalApplicationOwner::start() noexcept
{
    if(record_.state!=ApplicationState::Confirmed||!context_||!guard_)return ApplicationResult::Rejected;
    if(!GPENMPCRflyCanonicalModule::configure(context_))return ApplicationResult::Unavailable;
    // From this point configure(nullptr) is mandatory even if spawn/acquire
    // fails. Preserve attempts separately from successful task startup.
    record_.state=ApplicationState::Installed;++record_.start_requests;
    char name[]="gpenmpc_rfly_canonical",command[]="start";char *argv[]{name,command,nullptr};
    record_.last_start_return=GPENMPCRflyCanonicalModule::main(2,argv);
    return record_.last_start_return==0?ApplicationResult::Ready:ApplicationResult::Rejected;
}
ApplicationResult CanonicalApplicationOwner::stop() noexcept
{
    if(record_.state!=ApplicationState::Installed||!context_)return ApplicationResult::Rejected;
    ++record_.stop_requests;
    char name[]="gpenmpc_rfly_canonical",command[]="stop";char *argv[]{name,command,nullptr};
    record_.last_stop_return=GPENMPCRflyCanonicalModule::main(2,argv);
    return record_.last_stop_return==0?ApplicationResult::Pending:ApplicationResult::Unavailable;
}
bool CanonicalApplicationOwner::observe(ApplicationObservation &out) noexcept
{
    // Context diagnostics are deliberately not read while its task can write
    // them. SessionGuard supplies its own real mutex-protected snapshot.
    if(guard_)record_.session_observed=guard_->evidence_snapshot(record_.session);
    out=record_;return !guard_||record_.session_observed;
}
ApplicationResult CanonicalApplicationOwner::release(ApplicationObservation &out) noexcept
{
    if(record_.state==ApplicationState::Installed && !GPENMPCRflyCanonicalModule::configure(nullptr)){
        (void)observe(out);return ApplicationResult::Pending;
    }
    if(context_){
        if(!context_->storage_releasable()){(void)observe(out);return ApplicationResult::Pending;}
        record_.context=context_->diagnostics_after_stop();record_.context_observed_after_stop=true;
        record_.failed_raw_feedback=context_->failed_raw_feedback_after_stop();
        record_.interrupted_raw_feedback=context_->interrupted_raw_feedback_after_stop();
        record_.raw_evidence_available=record_.failed_raw_feedback.disposition!=FeedbackDisposition::Empty||
            record_.interrupted_raw_feedback.disposition!=FeedbackDisposition::Empty;
        delete context_;context_=nullptr;
    }
    if(guard_){
        record_.session_observed=guard_->evidence_snapshot(record_.session);
        if(!guard_->revoke()){out=record_;return ApplicationResult::Unavailable;}
        delete guard_;guard_=nullptr;
    }
    record_.state=ApplicationState::Retired;out=record_;return ApplicationResult::Detached;
}
} // namespace gpenmpc_rfly_px4
