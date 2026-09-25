#pragma once
#include "CanonicalLocalGpPending.hpp"
#include "CanonicalLocalWireOutbox.hpp"
#include "../local_source_ingress/Px4SelectedSourceReader.hpp"
#include "../px4_wire/CanonicalLocalIngress.hpp"
#include <uORB/topics/gpenmpc_full_inner_ingress.h>

namespace gpenmpc_rfly_px4 {
enum class LocalTaskRead:std::uint8_t {Ready,Missing,Invalid};
struct LocalTaskView {
    LocalCycleInputs inputs{};
    // Borrowed immutable original window and owner-resident conversion memory.
    // This core performs the real load; a callback's claimed generation alone
    // never changes the installed-window high water. No window interpolation.
    const gpenmpc_full_inner_window*window{};
    void*window_scratch{};std::size_t window_scratch_bytes{};
};
struct LocalTaskInputPort {
    void*owner{};
    LocalTaskRead (*read)(void*,const gpenmpc_odometry::Snapshot&,
        const gpenmpc_selected_source::Receipt&,LocalTaskView&)noexcept{};
    // Original task reference is source-independent. Obtain/load the initial
    // or newly available window before capture; LocalIo correctly forbids an
    // arbitrary initial load while a source is already pending. Refill after
    // an actual query miss still uses read()'s same-source immutable bindings.
    LocalTaskRead (*window)(void*,LocalTaskView&)noexcept{};
    // Optional concrete RWW1 owner, routed by THIS subscription/dispatcher.
    // When supplied, a callback may borrow only ready_window(); release is
    // performed here only after actual Io.load_window has succeeded.
    gpenmpc_local_window_wire::Assembler*window_receiver{};
    gpenmpc_local_task_wire::Receiver*task_receiver{};
    // Observe the real source and install completed input without control.
    // exported_snapshot is true only AFTER this source entered the actual
    // RLS outbox; unexported fast observations are not HOST reply anchors.
    LocalTaskRead (*observe_source)(void*,const gpenmpc_odometry::Snapshot&,
        const gpenmpc_selected_source::Receipt&,bool exported_snapshot)noexcept{};
    // Non-owning, same task only; owner and pointed-to descriptors/windows
    // remain alive through Context detach. read cannot supply an Authority or
    // replace the private Snapshot. Ready means data supplied, NOT provenance
    // proved; selected-source checks and LocalIo's full bindings still execute.
};
struct LocalExchangeConfiguration {
    gpenmpc_argument_transport::Configuration transport{};
    std::uint8_t ingress_topic_instance{};
    std::uint64_t gp_request_transport_max_age_us{},snapshot_transport_max_age_us{};
    gpenmpc_consumption::Identity identity{};
    bool export_active_snapshots{true};
    bool export_committed_state{true};
};
enum class LocalExchangeFault:std::uint8_t {None,Configuration,Clock,Ingress,Outbox,
    Cycle,SelectedSource,TaskInput,Window,Stopped};
enum class LocalExchangePoll:std::uint8_t {Idle,Progress,Fault};
struct LocalExchangeDiagnostics {
    LocalExchangeFault first_fault{LocalExchangeFault::None};
    std::uint64_t first_fault_us{},polls{},ingress_updates{},snapshots_enqueued{},
        gp_requests_enqueued{},gp_replies_filled{},commits{},window_loads{},
        missing_task_inputs{},missing_selected_endpoints{},last_processing_us{},committed_states_enqueued{};
    // None of these records is an outbound delivery/plant/PWM acknowledgement.
};
// Actual task-side composition. There is exactly one actual uORB subscription
// and one central ingress validator for all received schemas.
// Commander/link/physical state are owned by RegisteredLocalContext, not here.
class CanonicalLocalExchangeCore final {
public:
    CanonicalLocalExchangeCore(Px4CanonicalLocalIo&,CanonicalLocalExecutionCycle&,
        gpenmpc_selected_source::Px4SelectedSourceReader&,const LocalTaskInputPort&,
        const LocalExchangeConfiguration&)noexcept;
    LocalExchangePoll poll(bool disarmed_observation)noexcept;
    void stop()noexcept;
    CanonicalLocalWireOutbox&outbox()noexcept{return outbox_;}
    const LocalExchangeDiagnostics&diagnostics()const noexcept{return d_;}
    const CanonicalLocalGpPending&pending_after_stop()const noexcept{return pending_;}
    const gpenmpc_local_ingress::CanonicalLocalIngress&ingress_after_stop()const noexcept{return ingress_;}
private:
    bool fail(LocalExchangeFault)noexcept;
    bool receive(bool disarmed_observation)noexcept;
    bool retire_unanswered_gp()noexcept;
    bool enqueue_gp()noexcept;
    bool enqueue_committed()noexcept;
    bool load_task_window(const LocalTaskView&)noexcept;
    static LocalWireConfiguration wire_config(const LocalExchangeConfiguration&,const void*)noexcept;
    Px4CanonicalLocalIo&io_;CanonicalLocalExecutionCycle&cycle_;
    gpenmpc_selected_source::Px4SelectedSourceReader&selected_;
    const LocalTaskInputPort task_;
    const bool export_active_snapshots_;
    const bool export_committed_state_;
    CanonicalLocalGpPending pending_;
    gpenmpc_local_ingress::CanonicalLocalIngress ingress_;
    CanonicalLocalWireOutbox outbox_;
    uORB::Subscription subscription_;
    gpenmpc_full_inner_ingress_s topic_{};
    gpenmpc_local_ingress::Arrival arrival_{};
    gpenmpc_local_ingress::Completed completed_{};
    gpenmpc_selected_source::Receipt selected_receipt_{};
    LocalTaskView task_view_{};
    gpenmpc_local_snapshot_wire::Bytes snapshot_bytes_{};
    LocalNumericalObservation committed_numerical_{};
    gpenmpc_local_committed_wire::Observation committed_observation_{};
    gpenmpc_local_committed_wire::Bytes committed_bytes_{};
    bool committed_pending_{};
    bool capture_pending_{},capture_disarmed_{},snapshot_enqueued_{},gp_enqueued_{};
    std::uint64_t loaded_window_generation_{};
    std::uint64_t last_snapshot_export_us_{},last_snapshot_sample_us_{},last_committed_export_us_{};
    std::uint64_t last_control_start_us_{};
    // Input-roundtrip clock shared by both runtime-state modes.
    // Tracks input source time independently of control/observation time.
    std::uint64_t last_component_input_sample_us_{};
    LocalExchangeDiagnostics d_{};
};
}
