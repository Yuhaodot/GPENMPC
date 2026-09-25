#pragma once
// Observation/numerical request transport only; no controller authority/ACK.
#include "../px4_wire/CanonicalLocalGpWire.hpp"
#include "../px4_wire/CanonicalLocalSnapshotWire.hpp"
#include "../px4_wire/CanonicalLocalCommittedWire.hpp"
namespace gpenmpc_rfly_px4 {
enum class LocalWireFault:std::uint8_t {None,Configuration,ProducerIdentity,ConsumerIdentity,
    ProducerConflict,ConsumerConflict,InvalidMessage,SessionMismatch,Generation,Clock,Expired,Integrity,CounterOverflow};
enum class LocalWirePublish:std::uint8_t {Published,Busy,Unavailable};
enum class LocalWireTake:std::uint8_t {Empty,Fragment,Expired,Stopped,Unavailable};
struct LocalWireConfiguration {
    gpenmpc_consumption::Identity expected_session{};
    const void *producer_identity{};
    std::uint64_t gp_request_transport_max_age_us{},snapshot_transport_max_age_us{};
    std::uint8_t target_system{},target_component{};
};
struct LocalWireMetadata {
    gpenmpc_local_gp_wire::Hash message_sha256{};
    std::uint64_t generation{},source_generation{},original_source_timestamp_ns{},original_source_sample_us{},
        original_anchor_us{},original_transport_valid_until_us{};
    std::uint16_t message_bytes{};
    std::uint8_t schema{},fragment_count{};
    bool component_history_only{};
    // Local history-lane scheduling. Full outer observations use bounded bursts.
    bool yield_after_fragment{};
};
struct LocalWireFragment {
    gpenmpc_snapshot_wire::Fragment fragment{};
    LocalWireMetadata original{};
    std::uint8_t target_system{},target_component{};
};
struct LocalWireAudit {
    std::uint8_t message[1494]{};
    LocalWireMetadata original{};
    std::uint64_t published_messages{},copied_fragments{},fully_copied_messages{};
    std::uint8_t next_fragment{};bool pending{},closed{};
    LocalWireFault first_fault{LocalWireFault::None};
};
class CanonicalLocalWireOutbox final {
public:
    // Storage allocation for the current wire types; review when schemas expand.
    static constexpr std::size_t capacity=1494,max_fragments=13;
    explicit CanonicalLocalWireOutbox(const LocalWireConfiguration&)noexcept;
    CanonicalLocalWireOutbox(const CanonicalLocalWireOutbox&)=delete;
    CanonicalLocalWireOutbox&operator=(const CanonicalLocalWireOutbox&)=delete;
    CanonicalLocalWireOutbox(CanonicalLocalWireOutbox&&)=delete;
    CanonicalLocalWireOutbox&operator=(CanonicalLocalWireOutbox&&)=delete;
    // Caller owns immutable input for the call. Only original codec Request
    // bytes are accepted. No raw schema/payload or authority boolean API.
    LocalWirePublish publish_gp(const gpenmpc_local_gp_wire::RequestBytes&,const void *producer)noexcept;
    LocalWirePublish publish_snapshot(const gpenmpc_local_snapshot_wire::Bytes&,const void *producer)noexcept;
    LocalWirePublish publish_committed(const gpenmpc_local_committed_wire::Bytes&,const void *producer,
        bool component_history_only=false,bool timely_outer_observation=false)noexcept;
    // One stream object for this outbox lifetime, under registry PI exclusion.
    // Copies do not acknowledge transport or permit the pending GP to close.
    // Live passes zero: sample actual HRT after ready acquire, not before a
    // blocking registry lock. Nonzero actual_now is for retained HOST fixtures.
    LocalWireTake take(const void *consumer,LocalWireFragment&,std::uint64_t actual_now)noexcept;
    void close()noexcept{__atomic_store_n(&closed_,1,__ATOMIC_RELEASE);}
    bool closed()const noexcept{return __atomic_load_n(&closed_,__ATOMIC_ACQUIRE)!=0;}
    bool numerical_pending()const noexcept{return __atomic_load_n(&ready_,__ATOMIC_ACQUIRE)!=0;}
    bool history_pending()const noexcept{return __atomic_load_n(&history_ready_,__ATOMIC_ACQUIRE)!=0;}
    bool pending()const noexcept{return numerical_pending()||history_pending();}
    LocalWireFault fault()const noexcept{return static_cast<LocalWireFault>(__atomic_load_n(&fault_,__ATOMIC_ACQUIRE));}
    // Historical audit after close/fault. Nonblocking; false until any prior
    // publisher/taker exits. Never clears pending/first fault or sends bytes.
    bool audit_copy(LocalWireAudit&)noexcept;
private:
    bool fail(LocalWireFault)noexcept;
    LocalWirePublish publish_checked(const std::uint8_t*,std::size_t,const LocalWireMetadata&,unsigned kind)noexcept;
    bool enter(std::uint8_t&)noexcept;
    void leave(std::uint8_t&)noexcept;
    const LocalWireConfiguration config_;
    std::uint8_t bytes_[capacity]{};
    LocalWireMetadata metadata_{};
    // One bounded history record on the stream, separate from the RLS/GP slot.
    std::uint8_t history_bytes_[capacity]{};
    LocalWireMetadata history_metadata_{};
    const void *consumer_{};
    // Independent typed high waters: GP source ns, RLS sample HRT us. Never
    // compare these two domains or convert them into each other's timestamps.
    std::uint64_t last_generation_[3]{},last_source_generation_[3]{},last_source_time_[3]{};
    std::uint64_t published_{},copied_{},completed_{};
    std::uint8_t next_{},ready_{},closed_{},fault_{},producer_busy_{},consumer_busy_{};
    std::uint8_t history_next_{},history_ready_{};
};
}
