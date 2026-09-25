#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>

// Fixed diagnostic observation records.
namespace gpenmpc_clock_tap {
enum class PairScope : std::uint8_t { RuntimeOwnerSerializationUnproven, RetainedSequentialDllFixture };
struct DllGetterSample {
    std::uint64_t event_ordinal{}, accepted_generation{}, plant_session{};
    double hil_output30[30]{}, accepted_time_s{}, rotor_lag6_n[6]{};
    std::int32_t copied_length{};
    std::uint32_t getter_thread_id{};
    std::uint8_t observation_valid{}, model_failed{}, model_runtime_error{};
    PairScope pair_scope{PairScope::RuntimeOwnerSerializationUnproven};
};
struct ReceiverSample {
    std::uint64_t event_ordinal{}, original_wire_time_us{}, original_receiver_hrt_us{};
    std::uint32_t fields_updated{}, message_id{};
    std::int32_t receiver_instance{}, channel{};
    std::uint8_t system{}, component{}, sequence{}, payload_length{}, sensor_id{};
    std::uint8_t original_payload65[65]{};
};

// Single producer only. Configure/read only while its owner is quiescent.
// The owner enforces this lifecycle; concurrent consumers are unsupported.
template<class T, std::size_t Capacity> class PassiveBuffer final {
    static_assert(Capacity > 0, "Bounded diagnostic storage");
public:
    bool configure_once(bool enable) noexcept {
        if(configured_)return false;
        configured_=true;enabled_=enable;return true;
    }
    bool enabled()const noexcept{return enabled_;}
    bool observe(T sample)noexcept {
        if(!enabled_)return false;
        if(events_==UINT64_MAX){overflow_=true;enabled_=false;return false;}
        sample.event_ordinal=++events_;
        if(count_==Capacity){overflow_=true;++dropped_;return false;}
        records_[count_++]=sample;return true;
    }
    void stop()noexcept{enabled_=false;}
    std::size_t count()const noexcept{return count_;}
    std::uint64_t events()const noexcept{return events_;}
    std::uint64_t dropped()const noexcept{return dropped_;}
    bool overflow()const noexcept{return overflow_;}
    const T*at_quiescent(std::size_t i)const noexcept{return i<count_?&records_[i]:nullptr;}
private:
    T records_[Capacity]{};std::size_t count_{};
    std::uint64_t events_{},dropped_{};
    bool configured_{},enabled_{},overflow_{};
};

// The caller supplies ORIGINAL copied getter bytes and ORIGINAL generated fields.
// Runtime hook never upgrades pair_scope merely because times seem close.
inline DllGetterSample getter_sample(const double*copied,int length,
        std::uint64_t generation,std::uint64_t session,double accepted_time,
        const double*lag,bool valid,bool failed,bool runtime_error,std::uint32_t thread)noexcept {
    DllGetterSample s{};s.copied_length=length;
    if(length>0&&length<=30)std::memcpy(s.hil_output30,copied,sizeof(double)*static_cast<std::size_t>(length));
    s.accepted_generation=generation;s.plant_session=session;s.accepted_time_s=accepted_time;
    std::memcpy(s.rotor_lag6_n,lag,sizeof s.rotor_lag6_n);
    s.observation_valid=valid;s.model_failed=failed;s.model_runtime_error=runtime_error;s.getter_thread_id=thread;
    return s;
}
template<class Message,class Hil>inline ReceiverSample receiver_sample(
        const Message&msg,const Hil&hil,std::uint64_t original_hrt,
        std::int32_t instance,std::int32_t channel,const void*original_payload)noexcept {
    ReceiverSample s{};s.original_wire_time_us=hil.time_usec;s.original_receiver_hrt_us=original_hrt;
    s.fields_updated=hil.fields_updated;s.message_id=msg.msgid;s.receiver_instance=instance;s.channel=channel;
    s.system=msg.sysid;s.component=msg.compid;s.sequence=msg.seq;s.payload_length=msg.len;s.sensor_id=hil.id;
    const std::size_t n=msg.len<sizeof s.original_payload65?msg.len:sizeof s.original_payload65;
    std::memcpy(s.original_payload65,original_payload,n);return s;
}
enum class CandidateStatus : std::uint8_t { NoExactObservedTime, AmbiguousObservedTime,
    FailedHistoricalCandidate, CandidateCarrierUnverified };
struct Candidate {
    CandidateStatus status{CandidateStatus::NoExactObservedTime};
    std::size_t original_getter_index{};
    std::uint64_t original_generation{}, original_receiver_hrt_us{};
    bool control_authority{false};
    bool carrier_verified{false};
};
// Offline diagnostic search only. Retains a candidate's recorded generation;
// never divides HIL time, uses HOST RX, interpolates lag, or returns authority.
inline Candidate find_time_candidate(const DllGetterSample*records,std::size_t count,
                                    const ReceiverSample&rx)noexcept {
    Candidate out{};std::size_t matches=0;
    for(std::size_t k=0;k<count;++k){
        const auto&s=records[k];const double t=s.hil_output30[0];
        if(s.copied_length!=30||!std::isfinite(t)||t<0||t>=18446744073709551616.0)continue;
        const auto integer=static_cast<std::uint64_t>(t);
        const double back=static_cast<double>(integer);
        if(!(back<=t&&back>=t)||integer!=rx.original_wire_time_us)continue;
        ++matches;out.original_getter_index=k;out.original_generation=s.accepted_generation;
        out.original_receiver_hrt_us=rx.original_receiver_hrt_us;
        out.status=(s.model_failed||s.model_runtime_error||!s.observation_valid)
            ?CandidateStatus::FailedHistoricalCandidate:CandidateStatus::CandidateCarrierUnverified;
    }
    if(matches>1){out.status=CandidateStatus::AmbiguousObservedTime;out.original_generation=0;}
    return out;
}
} // namespace gpenmpc_clock_tap
