#pragma once
// Read-only actual uORB consumer. No HRT sampling, sensor write, clock mapping,
// receiver pointer dereference, controller/actuator permission or recovery.
#include "../clock_tap_overlay/ExactSourceReceiptLookup.hpp"
#include "../CanonicalLocalInnerInputBuilder.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/topics/gpenmpc_original_hil_receipt.h>

namespace gpenmpc_hil_endpoint_reader {
struct Configuration {
    // The registered-link owner supplies and retains the link; compare its address.
    const void*expected_registered_link{};
    std::int32_t expected_receiver_instance{-1},expected_channel{-1};
    std::uint8_t expected_noui_system{},expected_noui_component{};
    gpenmpc_consumption::Identity independently_observed_board_identity{};
};
enum class Fault:std::uint32_t {None,Configuration,AccessConflict,OwnerMismatch,Malformed,
    TopicGap,ProducerGap,ProducerPublicationFailure,CounterOverflow,RetentionOverflow,
    Lookup,ActualGyroNotCalled,Retirement,Stopped};
enum class Drain:std::uint8_t {NoUpdate,Observed,Unavailable};
struct Original {
    gpenmpc_original_hil_receipt_s topic{};
    std::uint32_t original_subscription_generation{};
};
struct Receipt {
    gpenmpc_source_receipt::Receipt endpoint{};
    Original original{};
    // Computed by lookup from the SAME private Snapshot, not supplied by a
    // wire caller or inferred from timestamp/generation alone.
    gpenmpc_local_input::SnapshotKey original_snapshot_key{};
    const void*original_source_topic{};
    std::uint8_t original_source_instance{UINT8_MAX};
    // Neither a finite packet nor an exact HRT endpoint supplies these proofs.
    bool receiver_to_estimator_lineage_proven{},dll_association_proven{},control_authority{};
};
struct Diagnostics {
    std::uint64_t observed_topic_records{},accepted_records{},successful_lookups{},retired_records{};
    std::uint64_t observed_receiver_address{},first_original_event_sequence{},first_original_receiver_hrt_us{},
        last_original_event_sequence{},last_original_receiver_hrt_us{};
    std::uint32_t first_original_subscription_generation{},last_original_subscription_generation{};
    std::size_t retained{};
    bool observed_baseline{},unknown_prebaseline{true};
    Fault first_fault{Fault::None};
    gpenmpc_source_receipt::Fault lookup_fault{gpenmpc_source_receipt::Fault::None};
    Original last_observed{}; // raw record survives malformed/gap/owner failure
};
class Px4OriginalHilReceiptReader final {
public:
    static constexpr std::size_t capacity=32,drain_limit=16;
    explicit Px4OriginalHilReceiptReader(const Configuration&)noexcept;
    Px4OriginalHilReceiptReader(const Px4OriginalHilReceiptReader&)=delete;
    Px4OriginalHilReceiptReader&operator=(const Px4OriginalHilReceiptReader&)=delete;
    // One owner serializes calls. An overlapping call is sticky-unavailable;
    // never waits for another thread. Startup without any actual message is
    // NoUpdate, not an invented receiver/baseline or successful receipt.
    Drain drain()noexcept;
    bool lookup(const gpenmpc_odometry::Snapshot&,Receipt&)noexcept;
    bool retire_through(std::uint64_t original_endpoint_hrt_us)noexcept;
    void stop()noexcept;
    Fault fault()const noexcept{return static_cast<Fault>(__atomic_load_n(&fault_,__ATOMIC_ACQUIRE));}
    bool diagnostics(Diagnostics&)noexcept;
    bool audit_copy(std::size_t,Original&)noexcept;
private:
    struct Access {
        Px4OriginalHilReceiptReader&self;bool held;
        explicit Access(Px4OriginalHilReceiptReader&)noexcept;~Access()noexcept;
        Access(const Access&)=delete;Access&operator=(const Access&)=delete;
    };
    bool fail(Fault)noexcept;
    bool observe(const gpenmpc_original_hil_receipt_s&,std::uint32_t generation)noexcept;
    const Configuration configuration_;
    uORB::Subscription subscription_{ORB_ID(gpenmpc_original_hil_receipt)};
    gpenmpc_source_receipt::ExactSourceReceiptLookup<capacity> lookup_{};
    Original retained_[capacity]{};
    gpenmpc_original_hil_receipt_s message_{};
    Diagnostics diagnostics_{};
    std::size_t count_{};
    bool bound_{};
    alignas(4)std::uint32_t busy_{},fault_{};
};
}
