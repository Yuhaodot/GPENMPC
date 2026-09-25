#pragma once
#include "GPENMPCRflyCanonicalModule.hpp"
#include "CanonicalExchangePump.hpp"
#include "CanonicalOutputAuthority.hpp"
#include "NativeLandOutputAuthority.hpp"
#include "DirectOffboardPrestream.hpp"
#include "LegacyTrajectoryReadOnly.hpp"
#include "Px4ExecutionSessionGuard.hpp"
#include "../px4_stream/SharedOutputRegistry.hpp"
#include "../px4_stream/SnapshotRouteRegistry.hpp"
#include "../px4_stream/CommittedFeedbackRouteRegistry.hpp"
#include <uORB/topics/vehicle_land_detected.h>

// Test commands alone may substitute the hardware-facing Guard with the real
// SessionBoundGuard over a mock raw observer. Never permitted in the ARM image.
#if defined(GPENMPC_CONTEXT_HOST_TEST_GUARD)
#if defined(__PX4_NUTTX) || defined(__arm__) || defined(__thumb__)
#error HOST context Guard injection is forbidden in the flight-controller build
#endif
#include GPENMPC_CONTEXT_HOST_TEST_GUARD
#else
namespace gpenmpc_rfly_px4 {using RegisteredContextGuard=Px4ExecutionSessionGuard;}
#endif

namespace gpenmpc_rfly_px4 {

struct RegisteredContextConfiguration {
    // Loaded from the explicit application configuration; acquire independently
    // compares its identity with the actual registered Guard, never vice versa.
    ModuleConfiguration module{};
    gpenmpc_argument_transport::Configuration transport{};
    std::uint8_t ingress_topic_instance{0}, retained_anchor_capacity{0};
    std::uint64_t native_land_tail_max_us{0};
};
enum class RegisteredContextFault : std::uint8_t {
    None,Configuration,Session,Physical,DuplicateAcquire,Registry,Control,Exchange,Clock,NativeTailDeadline
};
struct RegisteredContextDiagnostics {
    RegisteredContextFault first_fault{RegisteredContextFault::None};
    std::uint64_t first_fault_us{0}, polls{0}, observation_polls{0}, direct_polls{0};
    std::uint64_t close_started_us{0}, original_land_deadline_us{0};
    bool acquired{false}, direct_entered{false}, closing{false}, native_land_observed{false};
    bool native_landed_observed{false}, board_disarmed_observed{false}, virtual_zero_stream_accepted{false};
    bool plant_cache_zero_proven{false}; // only the independent HOST plant observer can establish this
    bool no_own_publication_disarmed_detach{false}, failed_raw_feedback_retained{false};
    bool interrupted_raw_feedback_retained{false};
    bool reservation_bound{false},acquire_aborted{false};
    std::uint64_t actual_publication_attempts{0}, actual_publication_successes{0};
    std::uint64_t offboard_publication_attempts{0},offboard_publications{0};
    std::uint64_t last_offboard_original_hrt_us{0},last_offboard_original_valid_until_us{0};
};

// Concrete ModuleContext. Its Guard was registered/confirmed on the actual
// Mavlink link by the application owner and retains that owner's lifetime.
// This class does not invent a CLI UID, session, physical declaration or mode
// ACK. No arm/mode/parameter command is issued. The external safety runner must
// request LAND/disarm and independently verify CopterSim/plant/cache state.
// An unproven/armed close retains all storage; it cannot be force-deleted.
class RegisteredCanonicalModuleContext final : public ModuleContext {
public:
    RegisteredCanonicalModuleContext(RegisteredContextGuard &registered_guard,
        const SessionEcho &observed_echo,const RegisteredContextConfiguration &configuration) noexcept;
    ModuleAcquire acquire(ModuleConfiguration &out) noexcept override;
    bool abort_acquire() noexcept override;
    ModulePoll poll(Px4CanonicalIo &io,std::uint64_t now_us) noexcept override;
    ModuleCloseResult close(ModuleStopReason reason,std::uint64_t now_us) noexcept override;
    // Call after ModuleBase stop and configure(nullptr), with poll/close quiescent.
    bool storage_releasable()const noexcept{return !reservation_.generation&&(detached_||(!routes_attempted_&&!io_));}
    const RegisteredContextDiagnostics &diagnostics_after_stop()const noexcept{return diagnostics_;}
    // Only after task stop. This is retained historical evidence, NOT an ACK,
    // successful telemetry flush or permission to continue observer state.
    const CommittedFeedback &failed_raw_feedback_after_stop()const noexcept{return failed_feedback_;}
    const CommittedFeedback &interrupted_raw_feedback_after_stop()const noexcept{return interrupted_feedback_;}
private:
    bool session_and_physical(BoardSafetyEvidence &out) noexcept;
    bool bind_routes() noexcept;
    bool detach_observation_routes() noexcept;
    bool detach_output() noexcept;
    bool bind_output(gpenmpc_rfly_stream::Authority &owner) noexcept;
    bool fail(RegisteredContextFault f,std::uint64_t now) noexcept;
    bool release_reservation() noexcept;
    bool update_prestream(Px4CanonicalIo &io,const BoardSafetyEvidence &observed,bool allow_emit) noexcept;
    static ExchangeConfiguration exchange_configuration(const RegisteredContextConfiguration &c) noexcept;
    RegisteredContextGuard &guard_;
    const SessionEcho echo_;
    const RegisteredContextConfiguration configuration_;
    Px4SessionClock clock_{};
    CanonicalOutputAuthority<RegisteredContextGuard,Px4SessionClock> direct_;
    NativeLandOutputAuthority<RegisteredContextGuard,Px4SessionClock> native_land_,disarmed_zero_;
    CanonicalExchangePump pump_;
    gpenmpc_rfly_stream::LinkLifetimeRegistry *links_{};
    gpenmpc_rfly_stream::CanonicalReservation reservation_{};
    DirectOffboardPrestream prestream_{};
    gpenmpc_rfly_stream::HostHeartbeatSnapshot published_heartbeat_{};
    uORB::Publication<offboard_control_mode_s> offboard_output_{ORB_ID(offboard_control_mode)};
    gpenmpc_rfly_stream::SharedOutputRegistry *outputs_{};
    gpenmpc_rfly_stream::SnapshotRouteRegistry *snapshots_{};
    gpenmpc_rfly_stream::CommittedFeedbackRouteRegistry *feedback_{};
    gpenmpc_rfly_stream::Registration output_registration_{};
    gpenmpc_rfly_stream::SnapshotRegistration snapshot_registration_{};
    gpenmpc_rfly_stream::FeedbackRegistration feedback_registration_{};
    uORB::Subscription status_{ORB_ID(vehicle_status)},mode_{ORB_ID(vehicle_control_mode)};
    uORB::Subscription landed_{ORB_ID(vehicle_land_detected)};
    Px4CanonicalIo *io_{};
    bool routes_attempted_{false}, routes_bound_{false}, detached_{false};
    bool direct_detached_{false}, observation_detached_{false}, native_selected_{false},zero_selected_{false};
    bool native_begun_{false},zero_begun_{false};
    CommittedFeedback failed_feedback_{},interrupted_feedback_{};
    RegisteredContextDiagnostics diagnostics_{};
};
} // namespace gpenmpc_rfly_px4
