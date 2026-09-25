#pragma once
#include "RegisteredCanonicalModuleContext.hpp"

namespace gpenmpc_rfly_px4 {
enum class ApplicationState : std::uint8_t {Empty,Registered,Confirmed,Installed,Retired};
enum class ApplicationResult : std::uint8_t {Ready,Rejected,Unavailable,Pending,Detached};
struct ApplicationObservation {
    ApplicationState state{ApplicationState::Empty};
    SessionEcho echo{};
    SessionGuardEvidence session{};
    RegisteredContextDiagnostics context{};
    CommittedFeedback failed_raw_feedback{};
    CommittedFeedback interrupted_raw_feedback{};
    std::uint64_t start_requests{},stop_requests{};
    int last_start_return{},last_stop_return{};
    bool session_observed{},context_observed_after_stop{},raw_evidence_available{};
};

// Serialized application/session owner. Acquire actual_link under the MAVLink
// lifetime lock. The caller supplies the challenge, declaration hash and profile
// and retains device, mode, arm, parameter and cleanup operations.
class CanonicalApplicationOwner final {
public:
    CanonicalApplicationOwner()=default;
    CanonicalApplicationOwner(const CanonicalApplicationOwner&)=delete;
    CanonicalApplicationOwner &operator=(const CanonicalApplicationOwner&)=delete;
    // No implicit release in destructor: application uses process-lifetime
    // storage and must explicitly prove task/callback detachment first.
    ApplicationResult prepare(Mavlink &actual_link,const SessionBoardIdentity &expected,
        const HostSessionChallenge &challenge,const RegisteredContextConfiguration &profile,
        SessionEcho &original_echo) noexcept;
    ApplicationResult confirm(const SessionEcho &host_returned_echo,
        const SessionPhysicalDeclaration &physical_declaration) noexcept;
    ApplicationResult start() noexcept;
    ApplicationResult stop() noexcept;
    ApplicationResult release(ApplicationObservation &retained) noexcept;
    bool observe(ApplicationObservation &out) noexcept;
private:
    Px4ExecutionSessionGuard *guard_{};
    RegisteredCanonicalModuleContext *context_{};
    RegisteredContextConfiguration profile_{};
    ApplicationObservation record_{};
};
} // namespace gpenmpc_rfly_px4
