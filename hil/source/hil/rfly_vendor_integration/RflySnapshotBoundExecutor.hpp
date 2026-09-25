#pragma once
// Snapshot-bound numerical executor with validated state transitions.
// Stores source-bound numerical transactions and validates 16-float receipts.
// The caller owns publication and actuator authority.
#include "../px4_full_inner/px4_state_adapter/AtomicOdometryAdapter.hpp"
#include "../px4_full_inner/argument_abi/CanonicalKernelArgumentCodec.hpp"
#include "CanonicalRflyExecutor.hpp"
#include "../px4_full_inner/portable/CanonicalPortable.hpp"

namespace gpenmpc_rfly_state_execution {
using namespace gpenmpc_consumption;
using SnapshotTicket=gpenmpc_portable::Array<std::uint8_t,32>;
using Digest=gpenmpc_portable::Array<std::uint32_t,8>;
enum class Failure : std::uint8_t {
    None, Configuration, Source, Capacity, Ticket, AlreadyConsumed, Pending,
    CommandTime, Stale, Abi, Lineage, SourceConfiguration, Executor, Commit, Exception
};
struct NumericalCommand {
    SnapshotTicket snapshot_ticket{};
    gpenmpc_portable::Array<std::uint8_t,gpenmpc_kernel_abi::encoded_size> arguments{};
    Digest argument_sha256{}, configuration_sha256{}, matlab_source_sha256{}, generated_source_sha256{};
    Digest wrapper_matlab_source_sha256{};
    Reference reference{};
    OuterCommand outer{};
    std::uint64_t original_board_ingress_us{0};
};
struct NumericalPrepared {
    bool numerically_prepared{false};
    bool board_authority_proven{false}; // never true in this library
    SnapshotTicket snapshot_ticket{};
    gpenmpc_rfly_execution::Prepared execution{};
};
struct NumericalReceipt {
    bool receipt_valid{false};
    bool board_authority_proven{false}, actual_output_consumption_proven{false};
    SnapshotTicket snapshot_ticket{};
    gpenmpc_rfly_execution::Committed execution{};
};

inline SnapshotTicket bytes(const Digest &d) noexcept {
    SnapshotTicket out{};
    for(unsigned j=0;j<8;++j)for(unsigned b=0;b<4;++b)out[4*j+b]=std::uint8_t(d[j]>>(24-8*b));
    return out;
}
inline void raw_float(CanonicalSha256 &h,float value) noexcept {
    static_assert(sizeof(float)==4&&gpenmpc_portable::Ieee754<float>::is_iec559,"actual float32 source required");
    std::uint32_t bits=0;std::memcpy(&bits,&value,4);h.u32(bits);
}
inline SnapshotTicket snapshot_digest(const gpenmpc_odometry::Snapshot &s,std::uint8_t instance,
                                      std::uint64_t stored_sequence) noexcept {
    CanonicalSha256 h;h.u32(0x52415331); // RAS1; fixed actual vehicle_odometry schema
    const auto&r=s.raw();hash_identity(h,s.estimator().identity);h.u64(stored_sequence);
    h.u32(vehicle_odometry_s::MESSAGE_VERSION);h.byte(instance);h.u32(s.subscription_generation());
    h.reals(s.task_origin_ned_m());h.u64(r.timestamp);h.u64(r.timestamp_sample);h.u64(s.estimator().board_rx_us);
    for(float v:r.position){raw_float(h,v);}
    for(float v:r.q){raw_float(h,v);}
    for(float v:r.velocity){raw_float(h,v);}
    for(float v:r.angular_velocity){raw_float(h,v);}
    for(float v:r.position_variance){raw_float(h,v);}
    for(float v:r.orientation_variance){raw_float(h,v);}
    for(float v:r.velocity_variance)raw_float(h,v);
    h.byte(r.pose_frame);h.byte(r.velocity_frame);h.byte(r.reset_counter);h.byte(std::uint8_t(r.quality));
    // Preserve the generated topic's explicit padding bytes, never implicit C++
    // struct padding or pointer values. Host echoes this opaque board ticket.
    for(auto v:r._padding0)h.byte(v);
    return bytes(h.finish());
}

template<std::size_t Capacity=4> class RflySnapshotBoundExecutor final {
    static_assert(Capacity>0,"snapshot store needs explicit fixed capacity");
    enum class SlotState : std::uint8_t { Empty, Available, Prepared, Consumed, ObservationReleased };
    struct Slot {gpenmpc_odometry::Snapshot snapshot{};SnapshotTicket ticket{};SlotState state{SlotState::Empty};};
public:
    RflySnapshotBoundExecutor(const gpenmpc_odometry::Configuration &state_config,
                          const gpenmpc_rfly_execution::Configuration &execution_config,
                          gpenmpc_rfly_execution::Clock clock) noexcept
        : state_config_(state_config),execution_config_(execution_config),clock_(clock),
          adapter_(state_config),executor_(execution_config,clock) {
        if(!clock.now_us || !(state_config.identity==execution_config.identity) ||
           state_config.sample_max_age_us!=execution_config.limits.sample_max_age_us ||
           adapter_.failure()!=gpenmpc_odometry::Failure::None || executor_.error()!=gpenmpc_rfly_execution::Error::None)
            fail(Failure::Configuration);
    }
    // Only this boundary can fill private slots; never accepts a HOST state or
    // externally constructed Snapshot. The real adapter retains original HRT.
    bool capture(const vehicle_odometry_s &raw,std::uint32_t original_generation,
                 std::uint64_t original_board_receipt_us,Identity observed_identity,
                 const void *topic,std::uint8_t instance,SnapshotTicket &ticket) noexcept {
        ticket={};if(failure_!=Failure::None)return false;
        std::size_t free=Capacity;
        for(std::size_t j=0;j<Capacity;++j)if(slots_[j].state==SlotState::Empty||slots_[j].state==SlotState::Consumed||slots_[j].state==SlotState::ObservationReleased){free=j;break;}
        if(free==Capacity || stored_sequence_==UINT64_MAX)return fail(Failure::Capacity);
        gpenmpc_odometry::Snapshot snap;
        if(!adapter_.ingest(raw,original_generation,original_board_receipt_us,clock_.now_us(clock_.context),
            observed_identity,topic,instance,snap))return fail(Failure::Source);
        auto &slot=slots_[free];slot.snapshot=snap;slot.ticket=snapshot_digest(snap,instance,++stored_sequence_);
        slot.state=SlotState::Available;ticket=slot.ticket;return true;
    }
    // Export is read-only and points to store-owned storage. A later capture
    // may reuse only consumed/released observations; copy before async IO.
    const gpenmpc_odometry::Snapshot *snapshot(const SnapshotTicket &ticket) const noexcept {
        if(failure_!=Failure::None)return nullptr;
        for(const auto&s:slots_)if(s.state!=SlotState::Empty&&s.ticket==ticket)return &s.snapshot;
        return nullptr;
    }
    // Preparation-only observation retirement. No numerical prepare or commit,
    // no change to consumption history, and never available for later execute.
    bool releaseObservation(const SnapshotTicket&ticket)noexcept{
        if(failure_!=Failure::None)return false;
        if(pending_!=Capacity||executor_.kernel_calls()||executor_.committed_outputs())return fail(Failure::Pending);
        const auto index=find(ticket);if(index==Capacity)return fail(Failure::Ticket);
        if(slots_[index].state!=SlotState::Available)return fail(Failure::AlreadyConsumed);
        slots_[index].state=SlotState::ObservationReleased;return true;
    }
    bool prepareNumericalOnly(const NumericalCommand &command,NumericalPrepared &out) noexcept {
        out={};out.execution.rfly_controls16.fill(gpenmpc_portable::Ieee754<float>::quiet_NaN());
        if(failure_!=Failure::None)return false;
        if(pending_!=Capacity)return fail(Failure::Pending);
        const auto index=find(command.snapshot_ticket);if(index==Capacity)return fail(Failure::Ticket);
        auto&slot=slots_[index];
        if(slot.state!=SlotState::Available || slot.snapshot.subscription_generation()<=last_consumed_generation_)
            return fail(Failure::AlreadyConsumed);
        const auto now=clock_.now_us(clock_.context);const auto &s=slot.snapshot.estimator();
        if(!command.original_board_ingress_us || command.original_board_ingress_us<s.board_rx_us ||
            command.original_board_ingress_us>now)return fail(Failure::CommandTime);
        if(now<s.timestamp_sample_us || now-s.timestamp_sample_us>state_config_.sample_max_age_us)return fail(Failure::Stale);
        if(command.configuration_sha256!=execution_config_.configuration_payload_sha256 ||
            command.matlab_source_sha256!=execution_config_.matlab_extraction_source_sha256 ||
            command.generated_source_sha256!=execution_config_.kernel_source_sha256 ||
            command.wrapper_matlab_source_sha256!=execution_config_.wrapper_matlab_source_sha256)return fail(Failure::SourceConfiguration);
        // Direct decode into persistent scratch avoids a large per-tick stack.
        scratch_={};auto &in=scratch_.consumed;
        if(!gpenmpc_kernel_abi::decode(command.arguments.data(),command.arguments.size(),command.argument_sha256,
                                    in.kernel,decode_failure_))return fail(Failure::Abi);
        if(in.kernel.augmentation_state_generation!=s.generation || in.kernel.continuity_state_generation!=s.generation)
            return fail(Failure::Lineage);
        if(!slot.snapshot.bind_state(in))return fail(Failure::Source);
        in.reference=command.reference;in.outer=command.outer;
        in.control_tick_us=now; // ORIGINAL board prepare HRT, never HOST timestamp
        in.expected_reference_generation=command.reference.generation;in.expected_outer_generation=command.outer.generation;
        in.kernel_source_sha256=command.generated_source_sha256;
        scratch_.matlab_extraction_source_sha256=command.matlab_source_sha256;
        scratch_.wrapper_matlab_source_sha256=command.wrapper_matlab_source_sha256;
        // Existing executor boolean preconditions are evaluated only in this
        // explicitly named numerical operation. They are NOT observations or a
        // new authorization API. Prepared/Receipt authority remains false.
        in.armed=in.offboard=in.controller_selected=in.native_conflicting_publishers_disabled=true;
        slot.state=SlotState::Prepared;pending_=index;
        if(!executor_.prepare(scratch_,out.execution))return fail(Failure::Executor);
        out.numerically_prepared=true;out.snapshot_ticket=slot.ticket;return true;
    }
    // Preserves the existing exact token + output bits + generation ACK logic.
    // HOST tests use mock ACKs; a numerical receipt is NOT downstream consume.
    bool commitNumericalReceipt(const SnapshotTicket &ticket,const gpenmpc_rfly_execution::BackendRflyAck &ack,
                                NumericalReceipt &out) noexcept {
        out={};if(failure_!=Failure::None)return false;
        if(pending_==Capacity || slots_[pending_].ticket!=ticket)return fail(Failure::Ticket);
        if(!executor_.commit(ack,out.execution))return fail(Failure::Commit);
        auto&s=slots_[pending_];last_consumed_generation_=s.snapshot.subscription_generation();s.state=SlotState::Consumed;
        out.receipt_valid=true;out.snapshot_ticket=ticket;pending_=Capacity;return true;
    }
    void note_exception() noexcept {executor_.note_exception();fail(Failure::Exception);}
    Failure failure()const noexcept{return failure_;}
    gpenmpc_odometry::Failure source_failure()const noexcept{return adapter_.failure();}
    gpenmpc_rfly_execution::Error executor_error()const noexcept{return executor_.error();}
    gpenmpc_consumption::Fault consumption_failure()const noexcept{return executor_.consumption_fault();}
    gpenmpc_kernel_abi::DecodeFailure decode_failure()const noexcept{return decode_failure_;}
    std::uint64_t kernel_calls()const noexcept{return executor_.kernel_calls();}
    std::uint64_t committed_outputs()const noexcept{return executor_.committed_outputs();}
    constexpr static std::size_t capacity=Capacity;
private:
    std::size_t find(const SnapshotTicket&t)const noexcept {
        for(std::size_t j=0;j<Capacity;++j){
            if(slots_[j].state!=SlotState::Empty&&slots_[j].ticket==t)return j;
        }
        return Capacity;
    }
    bool fail(Failure e) noexcept {if(failure_==Failure::None)failure_=e;return false;}
    gpenmpc_odometry::Configuration state_config_{};gpenmpc_rfly_execution::Configuration execution_config_{};
    gpenmpc_rfly_execution::Clock clock_{};gpenmpc_odometry::AtomicOdometryAdapter adapter_;
    gpenmpc_rfly_execution::CanonicalRflyExecutor executor_;gpenmpc_portable::Array<Slot,Capacity>slots_{};
    gpenmpc_rfly_execution::ExecutionInput scratch_{};Failure failure_{Failure::None};
    gpenmpc_kernel_abi::DecodeFailure decode_failure_{gpenmpc_kernel_abi::DecodeFailure::None};
    std::size_t pending_{Capacity};std::uint64_t stored_sequence_{0},last_consumed_generation_{0};
};
} // namespace gpenmpc_rfly_state_execution
