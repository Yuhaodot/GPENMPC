#pragma once
#include "RegisteredCanonicalLocalModuleContext.hpp"

namespace gpenmpc_rfly_local_px4 {
enum class LocalApplicationState:std::uint8_t {Empty,Registered,InputsBound,Confirmed,Installed,Retired};
enum class LocalApplicationResult:std::uint8_t {Ready,Rejected,Unavailable,Pending,Detached};
struct LocalApplicationObservation {
    LocalApplicationState state{LocalApplicationState::Empty};
    SessionEcho echo{};SessionGuardEvidence session{};
    RegisteredLocalContextDiagnostics context{};
    LocalCycleFeedback failed_feedback{};
    LocalExecutionFeedback late_feedback{};
    LocalWireAudit interrupted_wire{};
    std::uint64_t start_requests{},stop_requests{};
    int last_start_return{},last_stop_return{};
    bool session_observed{},context_observed_after_stop{},last_task_created{};
};

// Serialized application/session owner for the full-local module. It
// cannot open a device or issue arm, mode, parameter, reboot or flash commands.
// prepare() is called only by the existing locked MAVLink visitor. The input
// owner is then constructed from the returned real session/link identity and
// explicitly borrowed until release() proves task AND callback detachment.
class CanonicalLocalApplicationOwner final {
public:
    CanonicalLocalApplicationOwner()=default;
    CanonicalLocalApplicationOwner(const CanonicalLocalApplicationOwner&)=delete;
    CanonicalLocalApplicationOwner&operator=(const CanonicalLocalApplicationOwner&)=delete;
    LocalApplicationResult prepare(Mavlink&,const SessionBoardIdentity&,const HostSessionChallenge&,
        const RegisteredLocalContextConfiguration&,SessionEcho&)noexcept;
    LocalApplicationResult bind_inputs(const LocalTaskInputPort&)noexcept;
    LocalApplicationResult confirm(const SessionEcho&,const SessionPhysicalDeclaration&)noexcept;
    LocalApplicationResult start()noexcept;
    LocalApplicationResult stop()noexcept;
    LocalApplicationResult release(LocalApplicationObservation&)noexcept;
    bool observe(LocalApplicationObservation&)noexcept;
private:
    Px4ExecutionSessionGuard*guard_{};
    RegisteredCanonicalLocalModuleContext*context_{};
    RegisteredLocalContextConfiguration profile_{};
    LocalApplicationObservation record_{};
};
}
