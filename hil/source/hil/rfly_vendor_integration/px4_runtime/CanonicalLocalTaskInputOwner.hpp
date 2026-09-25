#pragma once
#include "CanonicalLocalExchangeCore.hpp"
#include "../px4_wire/CanonicalLocalTaskWire.hpp"

namespace gpenmpc_rfly_px4 {
enum class LocalTaskInputFault:std::uint8_t {None,Configuration,Clock,Expired,Busy,Fragment,Integrity,
    IdentityMismatch,Source,Sensor,Regression,OuterAnchor,OuterMutation,OuterExpired,Stopped};
struct LocalTaskInputConfiguration {
    gpenmpc_local_window_wire::Configuration window{};
    std::uint64_t input_assembly_max_us{},outer_max_age_us{};
    bool runtime_state_only{};
    bool component_initialization{};
    bool operator_reference{};
};
struct LocalTaskInputDiagnostics {
    LocalTaskInputFault first_fault{LocalTaskInputFault::None};
    std::uint64_t first_fault_us{},last_processing_us{},fragments{},messages{},sources_retained{},
        bindings{},missing_inputs{},outer_installs{},last_source{},last_outer{};
    std::uint64_t maximum_assembly_span_us{},fault_sequence{},fault_first_arrival_us{},fault_last_arrival_us{};
    unsigned fault_next_fragment{};bool fault_ready{};
};
// Persistent task-data receiver on the Core subscription. The selected
// odometry/HIL endpoint validates HOST source binding and retains RDR provenance.
class CanonicalLocalTaskInputOwner final:public gpenmpc_local_task_wire::Receiver {
public:
    explicit CanonicalLocalTaskInputOwner(const LocalTaskInputConfiguration&)noexcept;
    LocalTaskInputPort port()noexcept;
    bool receive(const gpenmpc_argument_transport::Arrival&,std::uint64_t)noexcept override;
    bool tick(std::uint64_t,bool disarmed_observation=false)noexcept override;
    void stop(std::uint64_t)noexcept override;
    const LocalTaskInputDiagnostics&diagnostics()const noexcept{return d_;}
    const gpenmpc_local_task_wire::Bytes&retained_raw_message()const noexcept{return bytes_;}
    const gpenmpc_argument_transport::Arrival&first_fault_arrival()const noexcept{return fault_arrival_;}
private:
    static LocalTaskRead read_entry(void*,const gpenmpc_odometry::Snapshot&,
        const gpenmpc_selected_source::Receipt&,LocalTaskView&)noexcept;
    static LocalTaskRead window_entry(void*,LocalTaskView&)noexcept;
    static LocalTaskRead observe_entry(void*,const gpenmpc_odometry::Snapshot&,
        const gpenmpc_selected_source::Receipt&,bool exported_snapshot)noexcept;
    LocalTaskRead observe(const gpenmpc_odometry::Snapshot&,const gpenmpc_selected_source::Receipt&,bool exported_snapshot)noexcept;
    LocalTaskRead read(const gpenmpc_odometry::Snapshot&,const gpenmpc_selected_source::Receipt&,LocalTaskView&)noexcept;
    LocalTaskRead window(LocalTaskView&)noexcept;
    bool retain_source(const gpenmpc_local_input::SnapshotKey&)noexcept;
    bool accept_outer(const gpenmpc_local_task_wire::Message&)noexcept;
    bool fail(LocalTaskInputFault,std::uint64_t)noexcept;
    const LocalTaskInputConfiguration c_;
    gpenmpc_local_window_wire::Assembler window_;
    static constexpr std::size_t scratch_capacity=28512,anchor_capacity=64;
    alignas(8)unsigned char scratch_[scratch_capacity]{};
    gpenmpc_local_task_wire::Bytes bytes_{};
    gpenmpc_local_task_wire::Message staged_{},current_{},outer_original_{};
    gpenmpc_local_input::SnapshotKey anchors_[anchor_capacity]{};
    gpenmpc_local_input::BoundRotorLag rotor_{};
    gpenmpc_local_input::BoundPayload payload_{};
    gpenmpc_local_input::BoundWind wind_{};
    gpenmpc_consumption::OuterCommand outer_{};
    gpenmpc_argument_transport::Arrival fault_arrival_{};
    const gpenmpc_argument_transport::Arrival*inside_arrival_{};
    LocalTaskInputDiagnostics d_{};
    std::uint64_t sequence_{},first_arrival_{},last_arrival_{},ready_arrival_{},highest_started_{};
    unsigned next_{};bool assembling_{},ready_{},have_current_{},have_outer_{};
};
}
