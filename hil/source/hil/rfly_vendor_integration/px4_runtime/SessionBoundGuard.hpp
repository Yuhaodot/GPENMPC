#pragma once

#include "ExecutionSessionRegistration.hpp"

namespace gpenmpc_rfly_px4 {

// Lock order: caller Authority -> this session -> short link borrow or ledger.
// No callback from the link registry may invoke this object. RawGuard is owned
// by this composition and must not also be observed independently/concurrently.
// Clock.now() is actual HRT on target; HOST fixtures explicitly supply a clock.
template<class RawGuard,class Clock>
class SessionBoundGuard final {
public:
    SessionBoundGuard(RawGuard &raw,Clock &clock,ExecutionSessionLedger &ledger,
                      gpenmpc_rfly_stream::LinkLifetimeRegistry &links,
                      const gpenmpc_rfly_stream::LinkToken &link,
                      const SessionBoardIdentity &expected,const SessionDigest &frozen_configuration) noexcept:
        raw_(raw),clock_(clock),ledger_(ledger),links_(links),link_(link),expected_(expected),configuration_(frozen_configuration)
    {
        if(!expected.uid || !expected.system || !expected.component || !link.pointer || !link.generation ||
           !nonzero_session_digest(configuration_) || !mutex_.ready())fail(SessionFault::Configuration,0);
    }
    // Captures actual UID/sys/comp and board HRT. No host-supplied generation,
    // registration time, raw safety evidence or observed identity argument exists.
    bool register_session(const HostSessionChallenge &challenge,SessionEcho &echo) noexcept
    {
        echo={};
        if(!mutex_.lock())return false;
        bool ok=healthy();
        if(ok && evidence_.registered)ok=fail(SessionFault::DuplicateRegistration,clock_.now());
        if(ok)ok=sample_raw();
        std::uint64_t generation=0;
        if(ok && ledger_.reserve(challenge,generation)!=SessionRegistration::Registered)
            ok=fail(SessionFault::RegistrationRejected,clock_.now());
        if(ok){
            // Registration time is a separate real HRT read after the ledger
            // operation, never a relabel of a source sample/publication stamp.
            const auto registered_now=clock_.now();
            if(registered_now<last_now_ || registered_now>evidence_.raw_board.identity_valid_until_us)
                ok=fail(SessionFault::HrtRegression,registered_now);
            else last_now_=registered_now;
        }
        if(ok){
            evidence_.registration.host_challenge=challenge;
            evidence_.registration.observed_identity=board_identity(evidence_.raw_board);
            evidence_.registration.board_registration_hrt_us=last_now_;
            evidence_.registration.process_session_generation=generation;
            evidence_.registration.link=link_;
            evidence_.registration.configuration_sha256=configuration_;
            evidence_.registered=true;
            echo=evidence_.registration;
        }
        return finish(ok);
    }
    // The HOST returns the exact board-created echo for causal session binding.
    // Authentication, when required, belongs to the transport layer.
    bool confirm_echo(const SessionEcho &echo) noexcept
    {
        if(!mutex_.lock())return false;
        bool ok=healthy() && evidence_.registered && sample_raw();
        if(ok && !same_session_echo(echo,evidence_.registration))ok=fail(SessionFault::EchoMismatch,clock_.now());
        if(ok)evidence_.echo_confirmed=true;
        return finish(ok);
    }
    bool bind_physical_declaration(const SessionPhysicalDeclaration &declaration) noexcept
    {
        if(!mutex_.lock())return false;
        bool ok=healthy() && evidence_.registered && evidence_.echo_confirmed && sample_raw();
        if(ok && (declaration.source!=PhysicalDeclarationSource::OperatorUsbIsolationDeclaration ||
            !nonzero_session_digest(declaration.physical_setup_record_sha256) ||
            declaration.exact_session_sha256!=execution_session_digest(evidence_.registration) ||
            declaration.usb_only!=HumanIsolationClaim::Declared || declaration.props_removed!=HumanIsolationClaim::Declared ||
            declaration.no_actuator_propulsion_power!=HumanIsolationClaim::Declared))
            ok=fail(SessionFault::PhysicalDeclarationInvalid,clock_.now());
        if(ok){evidence_.human_declaration=declaration;evidence_.human_declaration_bound=true;}
        return finish(ok);
    }
    // Validate each ticket before its snapshot and input checks.
    // The session binding is independent of the command wire schema.
    bool validate_ticket_binding(const SessionTicketBinding &binding) noexcept
    {
        if(!mutex_.lock())return false;
        bool ok=healthy() && evidence_.registered && evidence_.echo_confirmed && sample_raw();
        if(ok && (binding.process_session_generation!=evidence_.registration.process_session_generation ||
            binding.execution_session_sha256!=execution_session_digest(evidence_.registration)))
            ok=fail(SessionFault::TicketMismatch,clock_.now());
        return finish(ok);
    }
    bool observe(BoardSafetyEvidence &out) noexcept
    {
        out={};
        if(!mutex_.lock())return false;
        bool ok=healthy() && evidence_.registered && evidence_.echo_confirmed && sample_raw();
        out=evidence_.raw_board;
        // Never inherit an unexplained raw boolean/session number as authority.
        out.identity.boot_generation=0;out.boot_session=GuardFact::Unknown;
        out.external_physical_isolation=GuardFact::Unknown;
        if(ok){
            // The ABI boot_generation field carries the process-local execution
            // session generation here, as defined by SessionEcho.identity_semantics.
            // It does not represent a persistent or cross-boot identity.
            out.identity.boot_generation=evidence_.registration.process_session_generation;
            out.boot_session=GuardFact::Pass;
            if(out.reason==GuardReason::BootSessionUnknown)out.reason=GuardReason::None;
            if(evidence_.human_declaration_bound)out.external_physical_isolation=GuardFact::Pass;
            else if(out.reason==GuardReason::None)out.reason=GuardReason::ExternalPhysicalFactsUnknown;
        }
        return finish(ok);
    }
    bool evidence_snapshot(SessionGuardEvidence &out) noexcept
    {
        if(!mutex_.lock())return false;
        out=evidence_;
        return finish(true);
    }
    bool revoke() noexcept
    {
        if(!mutex_.lock())return false;
        (void)fail(SessionFault::Revoked,clock_.now());
        return finish(true); // means locally revoked, NOT plant safe/landed
    }
private:
    bool healthy()const noexcept{return evidence_.first_fault==SessionFault::None;}
    bool finish(bool result)noexcept
    {
        if(mutex_.unlock())return result;
        __atomic_store_n(&mutex_failed_,true,__ATOMIC_RELEASE);return false;
    }
    bool fail(SessionFault fault,std::uint64_t now)noexcept
    {
        if(evidence_.first_fault==SessionFault::None){evidence_.first_fault=fault;evidence_.first_fault_hrt_us=now;}
        evidence_.echo_confirmed=false;evidence_.human_declaration_bound=false;
        return false;
    }
    static SessionBoardIdentity board_identity(const BoardSafetyEvidence &raw)noexcept
    { return {raw.identity.uid,raw.identity.system,raw.identity.component}; }
    static bool live_link(const void *,void *result)noexcept{*static_cast<bool *>(result)=true;return true;}
    bool sample_raw()noexcept
    {
        if(__atomic_load_n(&mutex_failed_,__ATOMIC_ACQUIRE))return fail(SessionFault::MutexUnproven,0);
        const std::uint64_t before=clock_.now();
        BoardSafetyEvidence raw{};
        const bool observed=raw_.observe(raw);
        const std::uint64_t now=clock_.now();
        evidence_.raw_board=raw;
        if(!observed || !raw.require_identity_only() || raw.usb_transport!=GuardFact::Pass ||
           raw.hil_configuration!=GuardFact::Pass)return fail(SessionFault::RawObservation,now);
        if(!(board_identity(raw)==expected_))return fail(SessionFault::IdentityDrift,now);
        if(raw.identity.boot_generation || raw.boot_session!=GuardFact::Unknown)
            return fail(SessionFault::LegacyRawIdentity,now);
        bool link_alive=false;
        if(links_.read(link_,live_link,&link_alive)!=gpenmpc_rfly_stream::LinkAccess::Read || !link_alive)
            return fail(SessionFault::LinkUnavailable,now);
        const std::uint64_t stamps[]={raw.status_timestamp_us,raw.mode_timestamp_us,raw.offboard_timestamp_us,raw.power_timestamp_us};
        if(!before || before<last_now_ || now<before || raw.observation_us<before ||
            raw.observation_us>now || raw.observation_us<last_observation_ || raw.identity_valid_until_us<now)
            return fail(SessionFault::HrtRegression,now);
        for(unsigned i=0;i<4;++i){
            if(stamps[i]<last_stamps_[i] || stamps[i]>raw.observation_us)return fail(SessionFault::HrtRegression,now);
        }
        last_now_=now;last_observation_=raw.observation_us;
        for(unsigned i=0;i<4;++i)last_stamps_[i]=stamps[i];
        return true;
    }
    RawGuard &raw_;Clock &clock_;ExecutionSessionLedger &ledger_;
    gpenmpc_rfly_stream::LinkLifetimeRegistry &links_;
    const gpenmpc_rfly_stream::LinkToken link_;
    const SessionBoardIdentity expected_;
    const SessionDigest configuration_;
    InheritingMutex mutex_{};
    SessionGuardEvidence evidence_{};
    std::uint64_t last_now_{0},last_observation_{0},last_stamps_[4]{};
    bool mutex_failed_{false};
};
} // namespace gpenmpc_rfly_px4
