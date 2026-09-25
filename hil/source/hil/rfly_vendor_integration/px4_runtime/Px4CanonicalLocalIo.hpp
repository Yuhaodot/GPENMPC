#pragma once
// uORB/HRT adapter for the stateful canonical controller owner.
// Captures local state, publishes admitted outputs and installs committed state.
#include "../CanonicalFullInnerConsumption.hpp"
#include "ExecutionAuthority.hpp"
#include "LocalCommittedState.hpp"
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

ORB_DECLARE(actuator_outputs_rfly);
namespace gpenmpc_rfly_px4 {
enum class LocalCapture:std::uint8_t {NoUpdate,Accepted,Rejected};
enum class LocalFault:std::uint8_t {None,Configuration,IdentityMismatch,Pending,Source,Telemetry,
    Ownership,Reference,Input,Numeric,Consumption,Expired,Lease,Publish,Install,Feedback,Stopped};
struct LocalTicket {std::uint64_t capture{},source_generation{},sample_us{};};
inline bool operator==(const LocalTicket&a,const LocalTicket&b)noexcept{
    return a.capture==b.capture&&a.source_generation==b.source_generation&&a.sample_us==b.sample_us;
}
struct LocalDiagnostics {
    LocalFault first_fault{LocalFault::None};std::uint64_t first_fault_us{};
    gpenmpc_odometry::Failure source_adapter_fault{gpenmpc_odometry::Failure::None};
    std::uint64_t source_failure_sample_us{},source_failure_previous_sample_us{},source_failure_received_us{};
    std::uint64_t source_updates{},captured{},observations_released{},reference_prepares{},kernel_prepare_attempts{},kernel_calls{},
        publish_attempts{},publish_succeeded{},consumption_commits{},joint_installs{},gp_fills{};
    std::uint64_t original_publication_us{},original_commit_completed_us{},original_valid_until_us{};
    std::uint64_t reference_window_misses{};bool awaiting_window{};
    // Local call counters; transport delivery is tracked separately.
};
struct LocalCommand {
    bool operator_reference{};
    LocalTicket ticket{};
    gpenmpc_local_input::Context input_context{};
    const gpenmpc_local_input::BoundRotorLag*rotor{};
    const gpenmpc_local_input::BoundPayload*payload{};
    const gpenmpc_local_input::BoundWind*wind{};
    const gpenmpc_local_input::ExplicitInitialInterval*initial_interval{};
    gpenmpc_full_inner_reference_input reference_query{};
    // These retain original accepted reference/outer times. p/v/a are filled
    // by THIS owner from the actual pending canonical reference in TASK-NED.
    gpenmpc_consumption::Reference reference_envelope{};
    gpenmpc_consumption::OuterCommand outer_envelope{};
};
struct LocalExecutionFeedback {
    gpenmpc_full_consumption::StatefulToken token{};
    std::uint64_t original_publication_us{},original_commit_completed_us{},original_valid_until_us{};
    // Actual committed numerical reference, read after the ABI joint install.
    // Local phase advances from its bounded acceleration, not HOST phase/rate.
    gpenmpc_full_inner_reference_state committed_reference{};
    double request19[19]{};float prepared_control16[16]{},actual_control16[16]{};
    bool publication_attempted{},publication_succeeded{},consumption_committed{},
        numerical_reference_installed{},reference_state_copied{},authority_confirmed{},fresh_at_read{};
};
class Px4CanonicalLocalIo final {
public:
    // Persistent allocation capacity. Construction validates the current ABI
    // size and alignment; the object is never allocated on the tick stack.
    static constexpr std::size_t full_owner_capacity=61440;
    Px4CanonicalLocalIo(const gpenmpc_odometry::Configuration&,
        const gpenmpc_full_consumption::Configuration&,std::uint64_t telemetry_max_age_us,Authority*,
        std::uint64_t commander_telemetry_max_age_us=0,
        std::uint64_t output_transport_max_age_us=0)noexcept;
    ~Px4CanonicalLocalIo();
    Px4CanonicalLocalIo(const Px4CanonicalLocalIo&)=delete;
    Px4CanonicalLocalIo&operator=(const Px4CanonicalLocalIo&)=delete;
    LocalCapture capture_next(LocalTicket&,bool disarmed_observation=false,bool replace_unexecuted=false)noexcept;
    LocalCapture capture_disarmed(LocalTicket&)noexcept;
    bool release_disarmed(const LocalTicket&)noexcept;
    const gpenmpc_odometry::Snapshot*snapshot(const LocalTicket&)const noexcept;
    // Historical original capture only: no HRT read, freshness or authority
    // grant. Retained after release/fault; next successful capture replaces it.
    const gpenmpc_odometry::Snapshot*retained_latest_snapshot()const noexcept;
    // Window and GP ingress never call control or publish. Their original
    // message/clock/model authentication remains the owning runtime's job.
    bool load_window(const gpenmpc_full_inner_window&,void*scratch,std::size_t bytes)noexcept;
    bool fill_gp(const std::uint64_t original_tags2[2],const double result18[18])noexcept;
    bool execute(const LocalCommand&)noexcept;
    // Retains attempted/failed publication and post-install expiry as raw;
    // only all original flags plus original expiry can make fresh_at_read true.
    bool take_execution_feedback(LocalExecutionFeedback&)noexcept;
    void stop()noexcept;
    const LocalDiagnostics&diagnostics()const noexcept{return diagnostics_;}
    const gpenmpc_full_consumption::Diagnostics&consumption_diagnostics()const noexcept{return consumption_.diagnostics();}
    bool numerical_diagnostics(gpenmpc_full_inner_diagnostics&out)const noexcept;
    // Same dedicated task only; no concurrent stream/status call. Pure copy
    // from the actual full owner, including initial/unusable dispositions.
    // No HRT read, control call, token renewal, feedback dequeue or mutation.
    bool copy_local_numerical_state(LocalNumericalObservation&out)const noexcept;
private:
    static std::uint64_t clock()noexcept{return hrt_absolute_time();}
    static std::uint64_t expiry(std::uint64_t,std::uint64_t)noexcept;
    static std::uint64_t minimum(std::uint64_t a,std::uint64_t b)noexcept{return a<b?a:b;}
    bool identity()noexcept;
    bool refresh_control(std::uint64_t&)noexcept;
    bool refresh_disarmed()noexcept;
    bool fresh(std::uint64_t,std::uint64_t)const noexcept;
    bool fresh_commander(std::uint64_t,std::uint64_t)const noexcept;
    bool fail(LocalFault)noexcept;
    bool install_actual_publication(std::uint64_t,bool actual_published,bool consumption_ok)noexcept;
    const gpenmpc_full_consumption::Configuration configuration_;
    const std::uint64_t telemetry_max_age_us_;
    const std::uint64_t commander_telemetry_max_age_us_;
    const std::uint64_t output_transport_max_age_us_;
    Authority*const authority_;
    gpenmpc_odometry::AtomicOdometryAdapter source_;
    gpenmpc_full_consumption::CanonicalFullInnerConsumption consumption_;
    alignas(8) unsigned char full_storage_[full_owner_capacity]{};
    gpenmpc_full_inner_owner*full_owner_{};
    uORB::Subscription odometry_,status_sub_{ORB_ID(vehicle_status)},mode_sub_{ORB_ID(vehicle_control_mode)},offboard_sub_{ORB_ID(offboard_control_mode)};
    uORB::Subscription attitude_sub_{ORB_ID(vehicle_attitude)},position_sub_{ORB_ID(vehicle_local_position)};
    uORB::Publication<actuator_outputs_s> output_{ORB_ID(actuator_outputs_rfly)};
    uORB::Publication<vehicle_thrust_setpoint_s> thrust_status_{ORB_ID(vehicle_thrust_setpoint)};
    vehicle_odometry_s raw_{};gpenmpc_odometry::Snapshot snapshot_{},retained_snapshot_{};
    vehicle_status_s status_{};vehicle_control_mode_s mode_{};offboard_control_mode_s offboard_{};
    vehicle_attitude_s attitude_{};vehicle_local_position_s position_{};
    actuator_outputs_s output_message_{};
    LocalTicket pending_ticket_{};bool pending_{},disarmed_observation_{},feedback_pending_{},execute_started_{};
    std::uint64_t original_execute_start_us_{};
    gpenmpc_full_consumption::Hash original_retry_identity_{};
    gpenmpc_full_inner_reference_candidate reference_candidate_{};
    gpenmpc_local_input::BoundReference bound_reference_{};
    gpenmpc_local_input::Result assembled_{};
    gpenmpc_full_inner_state prior_state_{};
    gpenmpc_full_inner_candidate candidate_{};
    gpenmpc_full_consumption::NumericalEvidence evidence_{};
    gpenmpc_full_consumption::StatefulToken token_{};
    gpenmpc_full_consumption::Receipt consumption_receipt_{};
    gpenmpc_full_inner_backend backend_{};
    gpenmpc_full_inner_numeric_receipt numeric_receipt_{};
    gpenmpc_full_inner_reference_receipt reference_receipt_{};
    gpenmpc_consumption::Reference reference_envelope_{};
    LocalExecutionFeedback feedback_{};LocalDiagnostics diagnostics_{};
    LocalInstalledNumerics installed_numerics_{};
};
}
