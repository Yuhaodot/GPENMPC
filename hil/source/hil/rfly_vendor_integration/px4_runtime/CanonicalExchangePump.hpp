#pragma once
#include "Px4CanonicalIo.hpp"
#include "SnapshotOutbox.hpp"
#include "CommittedFeedbackOutbox.hpp"
#include "../px4_wire/RflyIngressDispatch.hpp"
#include <uORB/topics/gpenmpc_full_inner_ingress.h>
namespace gpenmpc_rfly_px4 {
enum class ExchangeFault:std::uint8_t{None,Configuration,Stopped,Io,Ingress,Context,Source,Expired,Capacity,Outbox};
enum class ExchangePoll:std::uint8_t{Idle,Progress,Fault};
struct ExchangeConfiguration {
    gpenmpc_rfly_execution::Configuration execution{};
    gpenmpc_argument_transport::Configuration transport{};
    std::uint8_t ingress_topic_instance{};
    // Explicit bounded storage resource, NOT a freshness/flight threshold.
    std::uint8_t retained_anchor_capacity{};
};
struct ExchangeDiagnostics {
    std::uint64_t polls{},captured{},context_accepted{},execute_attempts{},executed{},ingress_updates{};
    std::uint64_t first_fault_us{},pending_original_sample_us{},last_ingress_original_us{};
    std::uint64_t observations{},observation_exports{};
    ExchangeFault fault{ExchangeFault::None};
};
// Real production composition component. ModuleContext::poll may delegate here
// with its exact persistent Io. This class grants no Ready/identity/authority.
class CanonicalExchangePump final {
public:
    explicit CanonicalExchangePump(const ExchangeConfiguration&)noexcept;
    ExchangePoll poll(Px4CanonicalIo&)noexcept;
    ExchangePoll poll_disarmed_observation(Px4CanonicalIo&)noexcept;
    void stop(Px4CanonicalIo&)noexcept;
    SnapshotOutbox&snapshot_outbox()noexcept{return outbox_;}
    CommittedFeedbackOutbox&feedback_outbox()noexcept{return feedback_outbox_;}
    const ExchangeDiagnostics&diagnostics()const noexcept{return diagnostics_;}
private:
    static constexpr std::size_t maximum_anchor_records=64;
    enum class Phase:std::uint8_t{NeedCapture,AwaitContextFirst,Context,Numeric};
    bool fail(Px4CanonicalIo&,ExchangeFault)noexcept;
    bool fresh_pending(std::uint64_t now)const noexcept;
    bool first_context_header_valid(const gpenmpc_argument_transport::Arrival&)const noexcept;
    const ExchangeConfiguration configuration_;
    uORB::Subscription ingress_;
    gpenmpc_rfly_wire::IngressDispatch dispatch_;
    gpenmpc_context_wire::AnchorStore<maximum_anchor_records>anchors_;
    gpenmpc_context_wire::ContextBinding binding_;
    SnapshotOutbox outbox_;
    CommittedFeedbackOutbox feedback_outbox_;
    CommittedFeedback feedback_scratch_{};
    gpenmpc_full_inner_ingress_s ingress_message_{};
    gpenmpc_argument_transport::Arrival arrival_{};
    gpenmpc_snapshot_wire::Bytes export_scratch_{};
    gpenmpc_context_wire::Context context_scratch_{};
    gpenmpc_slim_transport::SlimCompleted numerical_scratch_{};
    gpenmpc_rfly_slim::Command command_scratch_{};
    gpenmpc_argument_transport::Metadata metadata_scratch_{};
    Px4CanonicalIo*owner_{nullptr};
    Ticket pending_ticket_{};std::uint64_t pending_generation_{},pending_source_us_{},pending_receipt_us_{};
    Phase phase_{Phase::NeedCapture};ExchangeDiagnostics diagnostics_{};
};
} // namespace gpenmpc_rfly_px4
