#pragma once
#include "GPENMPCRflyCanonicalLocalModule.hpp"
#include "CanonicalLocalExchangeCore.hpp"
#include "CanonicalOutputAuthority.hpp"
#include "NativeLandOutputAuthority.hpp"
#include "DirectOffboardPrestream.hpp"
#include "LegacyTrajectoryReadOnly.hpp"
#include "Px4ExecutionSessionGuard.hpp"
#include "../px4_stream/SharedOutputRegistry.hpp"
#include "../px4_stream/CanonicalLocalWireRouteRegistry.hpp"
#include <uORB/topics/vehicle_land_detected.h>

#if defined(GPENMPC_LOCAL_CONTEXT_HOST_TEST_GUARD)
#if defined(__PX4_NUTTX) || defined(__arm__) || defined(__thumb__)
#error HOST local Context Guard injection is forbidden on the target
#endif
#include GPENMPC_LOCAL_CONTEXT_HOST_TEST_GUARD
#else
namespace gpenmpc_rfly_local_px4 {using RegisteredLocalContextGuard=gpenmpc_rfly_px4::Px4ExecutionSessionGuard;}
#endif

namespace gpenmpc_rfly_local_px4 {
using namespace gpenmpc_rfly_px4;
struct RegisteredLocalContextConfiguration {
    ModuleConfiguration module{};
    gpenmpc_argument_transport::Configuration transport{};
    gpenmpc_hil_endpoint_reader::Configuration original_source{};
    gpenmpc_local_phase::Configuration phase{};
    LocalCycleConfiguration cycle{};
    LocalTaskInputPort task{};
    std::uint8_t ingress_topic_instance{};
    std::uint64_t native_land_tail_max_us{},gp_request_transport_max_age_us{},snapshot_transport_max_age_us{};
};
enum class RegisteredLocalContextFault:std::uint8_t {None,Configuration,Session,Physical,
    DuplicateAcquire,Registry,Control,Exchange,Clock,NativeTailDeadline};
struct LocalAcquireEvidence {
    bool attempted{},guard_snapshot_valid{};
    unsigned result{255},reason{},outputs_present{},wire_present{},links_present{};
    unsigned outputs_ready{255},wire_ready{255},bind_result{255},legacy_stopped{255};
    RegisteredLocalContextFault first_context_fault{RegisteredLocalContextFault::None};
    std::uint64_t first_context_fault_us{};
    BoardSafetyEvidence observed{};
    SessionGuardEvidence guard{};
};
struct RegisteredLocalContextDiagnostics {
    LocalAcquireEvidence acquire{}; // Fixed after the first synchronous acquire.
    RegisteredLocalContextFault first_fault{RegisteredLocalContextFault::None};
    ModuleStopReason module_stop_reason{ModuleStopReason::None};
    std::uint64_t first_fault_us{},polls{},observation_polls{},direct_polls{},close_started_us{},original_land_deadline_us{};
    bool acquired{},direct_entered{},closing{},native_land_observed{},native_landed_observed{},board_disarmed_observed{},
        virtual_zero_stream_accepted{},plant_cache_zero_proven{},no_own_publication_disarmed_detach{},
        failed_raw_feedback_retained{},interrupted_raw_feedback_retained{},late_raw_feedback_retained{},reservation_bound{},acquire_aborted{};
    std::uint64_t actual_publication_attempts{},actual_publication_successes{},offboard_publication_attempts{},
        offboard_publications{},last_offboard_original_hrt_us{},last_offboard_original_valid_until_us{};
    // Snapshot the nested first-fault state before close() asks the core/cycle
    // to stop.  stop() deliberately latches Stopped when no earlier fault was
    // present, so post-stop inspection alone cannot identify the live boundary.
    LocalExchangeDiagnostics exchange_before_stop{};
    LocalCycleDiagnostics cycle_before_stop{};
    LocalDiagnostics local_io_before_stop{};
    gpenmpc_hil_endpoint_reader::Diagnostics original_source_before_stop{};
    gpenmpc_selected_source::Diagnostics selected_source_before_stop{};
    bool nested_diagnostics_observed{},original_source_diagnostics_observed{};
};
struct LocalContextStateObservation {
    LocalNumericalObservation numerical{};
    gpenmpc_local_phase::Diagnostics phase{};
    bool phase_matches_committed_source{};
};
// Session/Commander/native-LAND context. TaskInput storage is borrowed until
// close and Module configure(nullptr). Each instance serves one explicit leg.
class RegisteredCanonicalLocalModuleContext final:public ModuleContext {
public:
    RegisteredCanonicalLocalModuleContext(RegisteredLocalContextGuard&,const SessionEcho&,
        const RegisteredLocalContextConfiguration&)noexcept;
    ~RegisteredCanonicalLocalModuleContext()override;
    ModuleAcquire acquire(ModuleConfiguration&)noexcept override;
    bool abort_acquire()noexcept override;
    ModulePoll poll(Px4CanonicalLocalIo&,std::uint64_t)noexcept override;
    ModuleCloseResult close(ModuleStopReason,std::uint64_t)noexcept override;
    bool storage_releasable()const noexcept{return !reservation_.generation&&(detached_||(!routes_attempted_&&!io_));}
    const RegisteredLocalContextDiagnostics&diagnostics_after_stop()const noexcept{return diagnostics_;}
    const LocalAcquireEvidence&acquire_evidence()const noexcept{return diagnostics_.acquire;}
    const LocalCycleFeedback&failed_raw_feedback_after_stop()const noexcept{return failed_feedback_;}
    const LocalExecutionFeedback&late_raw_feedback_after_stop()const noexcept{return late_feedback_;}
    const LocalWireAudit&interrupted_wire_after_stop()const noexcept{return interrupted_wire_;}
    const CanonicalLocalExchangeCore*core_after_stop()const noexcept{return core_;}
    // Dedicated task only, or after Module configure(nullptr); not stream-safe.
    bool copy_owner_state(LocalContextStateObservation&)const noexcept;
private:
    bool attach(Px4CanonicalLocalIo&)noexcept;
    bool session_and_physical(BoardSafetyEvidence&)noexcept;
    bool bind_routes()noexcept;
    bool detach_observation_routes()noexcept;
    bool detach_output()noexcept;
    bool bind_output(gpenmpc_rfly_stream::Authority&)noexcept;
    bool fail(RegisteredLocalContextFault,std::uint64_t)noexcept;
    bool release_reservation()noexcept;
    bool update_prestream(Px4CanonicalLocalIo&,const BoardSafetyEvidence&,bool)noexcept;
    RegisteredLocalContextGuard&guard_;const SessionEcho echo_;const RegisteredLocalContextConfiguration configuration_;
    Px4SessionClock clock_{};
    CanonicalOutputAuthority<RegisteredLocalContextGuard,Px4SessionClock> direct_;
    NativeLandOutputAuthority<RegisteredLocalContextGuard,Px4SessionClock> native_land_,disarmed_zero_;
    gpenmpc_hil_endpoint_reader::Px4OriginalHilReceiptReader source_;
    gpenmpc_selected_source::Px4SelectedSourceReader selected_;
    gpenmpc_local_phase::CanonicalLocalPhaseClock phase_;
    alignas(CanonicalLocalExecutionCycle) unsigned char cycle_storage_[sizeof(CanonicalLocalExecutionCycle)]{};
    alignas(CanonicalLocalExchangeCore) unsigned char core_storage_[sizeof(CanonicalLocalExchangeCore)]{};
    CanonicalLocalExecutionCycle*cycle_{};CanonicalLocalExchangeCore*core_{};
    gpenmpc_rfly_stream::LinkLifetimeRegistry*links_{};gpenmpc_rfly_stream::CanonicalReservation reservation_{};
    DirectOffboardPrestream prestream_{};gpenmpc_rfly_stream::HostHeartbeatSnapshot published_heartbeat_{};
    uORB::Publication<offboard_control_mode_s> offboard_output_{ORB_ID(offboard_control_mode)};
    gpenmpc_rfly_stream::SharedOutputRegistry*outputs_{};
    gpenmpc_rfly_stream::CanonicalLocalWireRouteRegistry*wire_{};
    gpenmpc_rfly_stream::Registration output_registration_{};
    gpenmpc_rfly_stream::LocalWireRegistration wire_registration_{};
    uORB::Subscription status_{ORB_ID(vehicle_status)},mode_{ORB_ID(vehicle_control_mode)},landed_{ORB_ID(vehicle_land_detected)};
    Px4CanonicalLocalIo*io_{};
    bool routes_attempted_{},routes_bound_{},detached_{},direct_detached_{},observation_detached_{},native_selected_{},zero_selected_{},native_begun_{},zero_begun_{};
    LocalCycleFeedback failed_feedback_{};LocalExecutionFeedback late_feedback_{};LocalWireAudit interrupted_wire_{};
    RegisteredLocalContextDiagnostics diagnostics_{};
};
}
