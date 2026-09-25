#pragma once
#include "ClockObservationTap.hpp"
#include "../../px4_full_inner/px4_state_adapter/AtomicOdometryAdapter.hpp"

// Records endpoint association by exact source time. Receiver/sensor/estimator
// lineage, model atomicity, IMU intervals and rotor lag require separate tracking.
namespace gpenmpc_source_receipt {
using gpenmpc_consumption::Identity;
enum class Fault : std::uint32_t { None, Configuration, AccessConflict, OwnerDrift,
    MalformedTap, DuplicateHrt, HrtRegression, Overflow, InvalidSnapshot,
    SnapshotIdentity, MissingExactHrt, GyroNotUpdated, Stopped, RetirementOrder, RetiredHrt };
struct Owner {
    const void *receiver{};
    std::int32_t instance{-1}, channel{-1};
    std::uint8_t system{}, component{};
    Identity board_identity{};
};
struct Record {
    std::uint64_t original_receiver_hrt_us{}, original_wire_time_us{}, original_tap_event_ordinal{};
    std::uint32_t fields_updated{};
    std::int32_t receiver_instance{}, channel{};
    std::uint8_t system{}, component{}, sequence{}, payload_length{}, sensor_id{};
    std::uint8_t original_sensor52[52]{}; // exact original payload[8..59], zero-filled only by MAVLink truncation
    bool all_gyro_fields_updated{};
};
struct Receipt {
    Record original{};
    std::uint64_t snapshot_sample_us{}, snapshot_generation{};
    bool exact_unique_endpoint{};
    // Intentionally never upgraded by this component.
    bool receiver_to_estimator_lineage_proven{}, rotor_association_proven{}, control_authority{};
};
struct Retention {
    std::size_t retained{};
    std::uint64_t retired_through_original_hrt_us{}, retired_records{}, retired_pre_endpoint_records{};
};

// Fixed retention capacity. Retire through the latest successful exact lookup;
// track pre-endpoint records separately. Overflow and high-water faults latch.
// Bind/destruct while quiescent. Observation operations use nonblocking CAS
// and latch overlap as unavailable.
template<std::size_t Capacity> class ExactSourceReceiptLookup final {
    static_assert(Capacity > 0, "Caller must select an explicit bounded capacity");
    static_assert(__atomic_always_lock_free(sizeof(std::uint32_t), nullptr), "Actual lock-free 32-bit atomics required");
public:
    ExactSourceReceiptLookup() noexcept = default;
    ExactSourceReceiptLookup(const ExactSourceReceiptLookup&)=delete;
    ExactSourceReceiptLookup&operator=(const ExactSourceReceiptLookup&)=delete;
    ExactSourceReceiptLookup(ExactSourceReceiptLookup&&)=delete;
    ExactSourceReceiptLookup&operator=(ExactSourceReceiptLookup&&)=delete;
    bool bind_once_quiescent(bool enabled, const Owner &owner) noexcept {
        if (configured_) return fail(Fault::Configuration);
        configured_=true;
        if (!enabled) return true;
        const auto &id=owner.board_identity;
        // Preserve an explicitly bound wire tuple even if a vendor uses zero;
        // it is observational data, never a source of MAVLink authority.
        if (!owner.receiver || owner.instance<0 || owner.channel<0 ||
            !id.uid || !id.boot_generation || !id.system || !id.component) return fail(Fault::Configuration);
        owner_=owner; enabled_=true; return true;
    }
    Fault fault() const noexcept { return static_cast<Fault>(__atomic_load_n(&fault_,__ATOMIC_ACQUIRE)); }
    bool observe(const void *actual_receiver, const gpenmpc_clock_tap::ReceiverSample &tap) noexcept {
        Access access(*this); if (!access.held || fault()!=Fault::None || !enabled_) return false;
        if (actual_receiver!=owner_.receiver || tap.receiver_instance!=owner_.instance || tap.channel!=owner_.channel ||
            tap.system!=owner_.system || tap.component!=owner_.component) return fail(Fault::OwnerDrift);
        if (tap.message_id!=107 || !tap.original_receiver_hrt_us || tap.payload_length>65 || !tap.payload_length ||
            little(tap.original_payload65,8)!=tap.original_wire_time_us ||
            little(tap.original_payload65+60,4)!=tap.fields_updated || tap.original_payload65[64]!=tap.sensor_id)
            return fail(Fault::MalformedTap);
        // ReceiverSample is zero-initialized by the existing receiver_sample
        // helper; MAVLink2 may legitimately omit trailing zero payload bytes.
        for (std::size_t i=tap.payload_length;i<65;++i)
            if (tap.original_payload65[i]) return fail(Fault::MalformedTap);
        if (tap.original_receiver_hrt_us<=retired_through_) return fail(Fault::RetiredHrt);
        for (std::size_t i=0;i<count_;++i)
            if (records_[i].original_receiver_hrt_us==tap.original_receiver_hrt_us) return fail(Fault::DuplicateHrt);
        if (tap.original_receiver_hrt_us<last_observed_hrt_) return fail(Fault::HrtRegression);
        if (count_==Capacity) return fail(Fault::Overflow);
        auto &r=records_[count_];
        r.original_receiver_hrt_us=tap.original_receiver_hrt_us;r.original_wire_time_us=tap.original_wire_time_us;
        r.original_tap_event_ordinal=tap.event_ordinal;r.fields_updated=tap.fields_updated;
        r.receiver_instance=tap.receiver_instance;r.channel=tap.channel;r.system=tap.system;r.component=tap.component;
        r.sequence=tap.sequence;r.payload_length=tap.payload_length;r.sensor_id=tap.sensor_id;
        std::memcpy(r.original_sensor52,tap.original_payload65+8,sizeof r.original_sensor52);
        r.all_gyro_fields_updated=(tap.fields_updated & 0x38U)==0x38U; // exact production SensorSource::GYRO
        ++count_;last_observed_hrt_=tap.original_receiver_hrt_us;return fault()==Fault::None;
    }
    bool lookup(const gpenmpc_odometry::Snapshot &snapshot, Receipt &out) noexcept {
        out=Receipt{};
        Access access(*this); if (!access.held || fault()!=Fault::None || !enabled_) return false;
        if (!snapshot.valid()) return fail(Fault::InvalidSnapshot);
        const auto &state=snapshot.estimator();
        if (!(state.identity==owner_.board_identity)) return fail(Fault::SnapshotIdentity);
        const Record *match=nullptr;
        for (std::size_t i=0;i<count_;++i) if (records_[i].original_receiver_hrt_us==state.timestamp_sample_us) {
            if (match) return fail(Fault::DuplicateHrt);
            match=&records_[i];
        }
        if (!match) return fail(Fault::MissingExactHrt);
        if (!match->all_gyro_fields_updated) return fail(Fault::GyroNotUpdated);
        out.original=*match;out.snapshot_sample_us=state.timestamp_sample_us;out.snapshot_generation=state.generation;
        out.exact_unique_endpoint=true;
        if (fault()!=Fault::None) {out=Receipt{};return false;}
        last_matched_hrt_=state.timestamp_sample_us;
        return true;
    }
    bool retire_through(std::uint64_t original_endpoint_hrt_us) noexcept {
        Access access(*this);if (!access.held || fault()!=Fault::None || !enabled_) return false;
        if (!original_endpoint_hrt_us || original_endpoint_hrt_us!=last_matched_hrt_ || original_endpoint_hrt_us<=retired_through_)
            return fail(Fault::RetirementOrder);
        std::size_t retired=0;
        while (retired<count_ && records_[retired].original_receiver_hrt_us<=original_endpoint_hrt_us) ++retired;
        if (!retired || records_[retired-1].original_receiver_hrt_us!=original_endpoint_hrt_us ||
            retired_records_>UINT64_MAX-retired) return fail(Fault::RetirementOrder);
        // Reclaim only that explicit consumed prefix. No clock is sampled and
        // records newer than the consumed original endpoint retain all bits.
        for (std::size_t i=retired;i<count_;++i) records_[i-retired]=records_[i];
        count_-=retired;retired_through_=original_endpoint_hrt_us;last_matched_hrt_=0;
        retired_records_+=retired;retired_pre_endpoint_records_+=retired-1;
        return fault()==Fault::None;
    }
    bool retention(Retention &out) noexcept {
        out=Retention{};Access access(*this);if (!access.held) return false;
        out.retained=count_;out.retired_through_original_hrt_us=retired_through_;
        out.retired_records=retired_records_;out.retired_pre_endpoint_records=retired_pre_endpoint_records_;return true;
    }
    void stop() noexcept { (void)fail(Fault::Stopped); }
    // Historical copies only, still available after fault. This cannot return
    // an Exact receipt or authority. No pointer into mutable storage escapes.
    bool audit_copy(std::size_t index, Record &out) noexcept {
        out=Record{};Access access(*this);if (!access.held || index>=count_) return false;
        out=records_[index];return true;
    }
private:
    struct Access {
        ExactSourceReceiptLookup &self;bool held;
        explicit Access(ExactSourceReceiptLookup &s) noexcept : self(s),held(false) {
            std::uint32_t expected=0;
            held=__atomic_compare_exchange_n(&self.busy_,&expected,1,false,__ATOMIC_ACQUIRE,__ATOMIC_RELAXED);
            if (!held) (void)self.fail(Fault::AccessConflict);
        }
        ~Access() noexcept {if (held) __atomic_store_n(&self.busy_,0,__ATOMIC_RELEASE);}
        Access(const Access&)=delete;Access&operator=(const Access&)=delete;
    };
    static std::uint64_t little(const std::uint8_t *p,unsigned n) noexcept {
        std::uint64_t value=0;for (unsigned i=0;i<n;++i)value|=std::uint64_t(p[i])<<(8U*i);return value;
    }
    bool fail(Fault f) noexcept {
        std::uint32_t expected=0;
        (void)__atomic_compare_exchange_n(&fault_,&expected,static_cast<std::uint32_t>(f),false,__ATOMIC_RELEASE,__ATOMIC_RELAXED);
        return false;
    }
    Owner owner_{};Record records_[Capacity]{};std::size_t count_{};
    std::uint64_t last_observed_hrt_{},last_matched_hrt_{},retired_through_{},retired_records_{},retired_pre_endpoint_records_{};
    alignas(4) std::uint32_t busy_{},fault_{};
    bool configured_{},enabled_{};
};

// Call ONLY at the existing receiver tap site, passing that function's original
// `timestamp` sampled once before gyro/accel update. No additional HRT read.
// Not installed in any application by this independent header.
template<std::size_t Capacity,class Message,class Hil>
inline bool observe_original_receiver(ExactSourceReceiptLookup<Capacity> &lookup,const void *receiver,
        const Message &msg,const Hil &hil,std::uint64_t original_hrt,std::int32_t instance,
        std::int32_t channel,const void *original_payload) noexcept {
    return lookup.observe(receiver,gpenmpc_clock_tap::receiver_sample(msg,hil,original_hrt,instance,channel,original_payload));
}
} // namespace gpenmpc_source_receipt
