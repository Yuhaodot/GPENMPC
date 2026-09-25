#pragma once

#include "BoardSafetyEvidence.hpp"
#include "InheritingMutex.hpp"
#include "../px4_stream/LinkLifetimeRegistry.hpp"

namespace gpenmpc_rfly_px4 {

using SessionDigest = gpenmpc_portable::Array<std::uint32_t, 8>;
struct HostSessionChallenge {
    std::uint64_t high{0}, low{0};
};
inline bool operator==(const HostSessionChallenge &a,const HostSessionChallenge &b) noexcept
{ return a.high==b.high && a.low==b.low; }
struct SessionBoardIdentity {
    std::uint64_t uid{0};
    std::uint8_t system{0}, component{0};
};
inline bool operator==(const SessionBoardIdentity &a,const SessionBoardIdentity &b) noexcept
{ return a.uid==b.uid && a.system==b.system && a.component==b.component; }
enum class SessionIdentitySemantics : std::uint8_t {
    // Process-local execution-session generation, not a cross-boot identity.
    BoardRegisteredExecutionSessionGenerationV1=1
};
struct SessionEcho {
    HostSessionChallenge host_challenge{};
    SessionBoardIdentity observed_identity{};
    std::uint64_t board_registration_hrt_us{0};
    std::uint64_t process_session_generation{0};
    gpenmpc_rfly_stream::LinkToken link{};
    SessionDigest configuration_sha256{};
    SessionIdentitySemantics identity_semantics{
        SessionIdentitySemantics::BoardRegisteredExecutionSessionGenerationV1};
};
bool same_session_echo(const SessionEcho &a,const SessionEcho &b) noexcept;
SessionDigest execution_session_digest(const SessionEcho &echo) noexcept;
bool nonzero_session_digest(const SessionDigest &digest) noexcept;

// Execution-session binding carried and checked for every ingress ticket.
// This binding is separate from the RSP1 snapshot and command schemas.
struct SessionTicketBinding {
    SessionDigest execution_session_sha256{};
    std::uint64_t process_session_generation{0};
};
enum class SessionRegistration : std::uint8_t { Registered,Invalid,Replay,Full,Unproven };

// Process-lifetime ledger: no eviction, no challenge reset on session revoke.
// 64 is a storage capacity, NOT a time/safety threshold. Full fails closed.
// Use the singleton on target. Public construction permits isolated HOST tests.
class ExecutionSessionLedger final {
public:
    static constexpr unsigned capacity=64;
    bool ready()const noexcept{return mutex_.ready();}
    SessionRegistration reserve(const HostSessionChallenge &challenge,
                                std::uint64_t &board_assigned_generation) noexcept;
private:
    InheritingMutex mutex_{};
    HostSessionChallenge used_[capacity]{};
    unsigned count_{0};
    std::uint64_t generation_{0};
    bool poisoned_{false};
};
ExecutionSessionLedger *execution_session_ledger() noexcept;

enum class PhysicalDeclarationSource : std::uint8_t {
    Missing,OperatorUsbIsolationDeclaration
};
enum class HumanIsolationClaim : std::uint8_t { Unknown,Declared,Contradicted };
struct SessionPhysicalDeclaration {
    PhysicalDeclarationSource source{PhysicalDeclarationSource::Missing};
    SessionDigest physical_setup_record_sha256{};
    SessionDigest exact_session_sha256{};
    HumanIsolationClaim usb_only{HumanIsolationClaim::Unknown};
    HumanIsolationClaim props_removed{HumanIsolationClaim::Unknown};
    HumanIsolationClaim no_actuator_propulsion_power{HumanIsolationClaim::Unknown};
};
// The runner binds a recorded physical-isolation declaration to this session.
// The declaration is attributable to its author; it is not an independent
// measurement of wiring or power state, nor a cryptographic authentication.

enum class SessionFault : std::uint8_t {
    None,Configuration,DuplicateRegistration,RawObservation,IdentityDrift,
    LegacyRawIdentity,LinkUnavailable,HrtRegression,RegistrationRejected,
    EchoMismatch,PhysicalDeclarationInvalid,TicketMismatch,Revoked,MutexUnproven
};
struct SessionGuardEvidence {
    BoardSafetyEvidence raw_board{}; // Board observation.
    SessionEcho registration{};
    SessionPhysicalDeclaration human_declaration{};
    SessionFault first_fault{SessionFault::None};
    std::uint64_t first_fault_hrt_us{0};
    bool registered{false},echo_confirmed{false},human_declaration_bound{false};
    // Identity.boot_generation follows SessionIdentitySemantics.
};

} // namespace gpenmpc_rfly_px4
