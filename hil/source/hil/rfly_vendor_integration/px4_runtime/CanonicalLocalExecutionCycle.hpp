#pragma once
#include "Px4CanonicalLocalIo.hpp"
#include "../local_phase/CanonicalLocalPhaseClock.hpp"
#include "../local_source_ingress/Px4OriginalHilReceiptReader.hpp"

namespace gpenmpc_rfly_px4 {
struct LocalCycleConfiguration {
    gpenmpc_local_input::Context context{};
    gpenmpc_local_input::ExplicitInitialInterval initial_interval{};
    std::uint64_t reference_max_age_us{};
    bool runtime_state_only{};
    bool component_initialization{};
    bool operator_reference{};
};
struct LocalCycleInputs {
    // Associated source observations; preserve DLL/EKF lineage and actuator lag.
    const gpenmpc_local_input::BoundRotorLag*rotor{};
    const gpenmpc_local_input::BoundPayload*payload{};
    const gpenmpc_local_input::BoundWind*wind{};
    gpenmpc_consumption::OuterCommand outer{};
    double target4[4]{};
    std::uint64_t loaded_window_generation{};
};
enum class LocalCycleFault:std::uint8_t {None,Configuration,Pending,Io,SourceReceipt,
    Phase,Clock,Feedback,Stopped};
enum class LocalCycleExecute:std::uint8_t {Committed,NeedReferenceWindow,Rejected};
struct LocalCycleFeedback {
    LocalExecutionFeedback actual{};
    bool present{},phase_installed{},source_receipt_retired{};
    // Preserve actual.fresh_at_read as the historical Io read disposition.
    // This independent disposition is evaluated against original expiry now.
    bool usable_at_read{};
    std::uint64_t original_cycle_read_us{};
};
struct LocalCycleDiagnostics {
    LocalCycleFault first_fault{LocalCycleFault::None};
    std::uint64_t captures{},attempts{},window_misses{},commits{},
        original_reference_created_us{},first_fault_us{};
    // Separate actual effects: Io release may succeed before endpoint
    // retirement fails; never report a fictitious rollback of either call.
    // observation_exports counts in-process copies, not packets or delivery.
    std::uint64_t observation_captures{},observation_exports{},
        observation_io_releases{},observation_endpoint_retirements{};
};
// One actual source capture -> local original reference -> complete C inner
// -> actual uORB publication/joint install -> phase advance. No RCT3/RKS4
// HOST inner request. Window/GP ingress only fill the existing numerical owner.
// RegisteredContext must still own physical/Commander/stream/LAND lifecycle;
// this component supplies none of those permissions and sends no command.
class CanonicalLocalExecutionCycle final {
public:
    CanonicalLocalExecutionCycle(Px4CanonicalLocalIo&,gpenmpc_local_phase::CanonicalLocalPhaseClock&,
        gpenmpc_hil_endpoint_reader::Px4OriginalHilReceiptReader&,const LocalCycleConfiguration&)noexcept;
    LocalCapture capture()noexcept;
    // Export the retained observation before the first control execution.
    LocalCapture capture_disarmed()noexcept;
    bool copy_disarmed_observation(gpenmpc_odometry::Snapshot&,
        gpenmpc_hil_endpoint_reader::Receipt&)noexcept;
    bool release_disarmed()noexcept;
    // Same numerical owner only. Diagnostics are historical reads; a GP
    // completion fills the pending prediction and never executes or advances.
    bool numerical_diagnostics(gpenmpc_full_inner_diagnostics&)const noexcept;
    bool fill_gp(const std::uint64_t original_tags2[2],const double result18[18])noexcept;
    const gpenmpc_odometry::Snapshot*snapshot()const noexcept;
    const gpenmpc_hil_endpoint_reader::Receipt*original_endpoint()const noexcept;
    LocalCycleExecute execute(const LocalCycleInputs&)noexcept;
    bool take_feedback(LocalCycleFeedback&)noexcept;
    void stop()noexcept;
    const LocalCycleDiagnostics&diagnostics()const noexcept{return d_;}
    bool runtime_state_only()const noexcept{return config_.runtime_state_only;}
    bool component_initialization()const noexcept{return config_.component_initialization;}
    const gpenmpc_local_phase::Diagnostics&committed_phase_observation()const noexcept{return phase_.diagnostics();}
private:
    bool fail(LocalCycleFault)noexcept;
    void retain_actual_feedback()noexcept;
    static std::uint64_t expiry(std::uint64_t t,std::uint64_t age)noexcept{
        return t&&age&&t<=UINT64_MAX-age?t+age:0;
    }
    Px4CanonicalLocalIo&io_;gpenmpc_local_phase::CanonicalLocalPhaseClock&phase_;
    gpenmpc_hil_endpoint_reader::Px4OriginalHilReceiptReader&source_;
    const LocalCycleConfiguration config_;
    LocalTicket ticket_{};bool pending_{},reference_started_{},feedback_pending_{},disarmed_pending_{};
    gpenmpc_hil_endpoint_reader::Receipt endpoint_{};
    LocalCommand command_{};gpenmpc_full_inner_backend published_{};
    LocalCycleFeedback feedback_{};LocalCycleDiagnostics d_{};
};
}
