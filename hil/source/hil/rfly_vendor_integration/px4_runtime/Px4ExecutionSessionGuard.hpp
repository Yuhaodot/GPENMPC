#pragma once

#include "SessionBoundGuard.hpp"
#include "Px4ReadOnlyGuard.hpp"

namespace gpenmpc_rfly_px4 {
struct Px4SessionClock { std::uint64_t now() const noexcept; };

// Owns the actual RawGuard so the session token and USB observation cannot refer
// to different caller-provided links. Creation merely allocates a read-only
// component: it neither starts a module nor registers/arms/selects a session.
class Px4ExecutionSessionGuard final {
public:
    static Px4ExecutionSessionGuard *create(Mavlink &actual_link,
        const SessionBoardIdentity &expected,const SessionDigest &frozen_configuration,
        std::uint64_t frozen_telemetry_max_age_us,
        std::uint64_t frozen_commander_max_age_us = 0) noexcept;
    bool register_session(const HostSessionChallenge &c,SessionEcho &e)noexcept{return composed_.register_session(c,e);}
    bool confirm_echo(const SessionEcho &e)noexcept{return composed_.confirm_echo(e);}
    bool bind_physical_declaration(const SessionPhysicalDeclaration &d)noexcept
    {return composed_.bind_physical_declaration(d);}
    bool validate_ticket_binding(const SessionTicketBinding &b)noexcept{return composed_.validate_ticket_binding(b);}
    bool observe(BoardSafetyEvidence &e)noexcept{return composed_.observe(e);}
    bool evidence_snapshot(SessionGuardEvidence &e)noexcept{return composed_.evidence_snapshot(e);}
    bool revoke()noexcept{return composed_.revoke();}
private:
    Px4ExecutionSessionGuard(Mavlink &actual,const SessionBoardIdentity &expected,
        const SessionDigest &configuration,std::uint64_t age,std::uint64_t commander_age,
        ExecutionSessionLedger &ledger,gpenmpc_rfly_stream::LinkLifetimeRegistry &links,
        const gpenmpc_rfly_stream::LinkToken &token) noexcept:
        raw_(actual,age,commander_age),composed_(raw_,clock_,ledger,links,token,expected,configuration) {}
    Px4ReadOnlyGuard raw_;
    Px4SessionClock clock_{};
    SessionBoundGuard<Px4ReadOnlyGuard,Px4SessionClock> composed_;
};
} // namespace gpenmpc_rfly_px4
