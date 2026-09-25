#pragma once
#include <cstdint>

namespace gpenmpc_rfly_stream {

struct HostHeartbeatBinding {
    std::uint64_t session_generation{},registration_hrt_us{};
    std::uint8_t host_system{},host_component{};
};
inline bool operator==(const HostHeartbeatBinding &a,const HostHeartbeatBinding &b)noexcept
{return a.session_generation==b.session_generation && a.registration_hrt_us==b.registration_hrt_us &&
        a.host_system==b.host_system && a.host_component==b.host_component;}
inline bool valid(const HostHeartbeatBinding &b)noexcept
{return b.session_generation && b.registration_hrt_us && b.host_system && b.host_component;}

enum class HeartbeatRecord:std::uint8_t {Recorded,IgnoredTuple,Duplicate,RejectedTime,Unbound,Exhausted};
enum class HeartbeatFreshness:std::uint8_t {Unbound,Missing,Fresh,Expired,InvalidClock,InvalidAge};
struct HostHeartbeatSnapshot {
    HostHeartbeatBinding binding{};
    std::uint64_t original_receiver_hrt_us{},receipt_generation{},original_valid_until_us{},max_age_us{};
    std::uint8_t source_system{},source_component{},mav_type{},autopilot{};
    HeartbeatFreshness freshness{HeartbeatFreshness::Unbound};
};

// Embed in the existing LinkLifetimeRegistry Entry. Every method, including
// snapshot/retire, requires that Entry's existing PI mutex. No own broker,
// lock, timer, thread or MAVLink parser is created. The enclosing Entry owns
// the exact lifecycle LinkToken; it must reject retired/reused pointers before
// calling this object. No naked Mavlink pointer is retained here.
//
// Records decoded HEARTBEAT arrival. Standard HEARTBEAT has no session nonce;
// the owner/replay ledger supplies binding. Validate solver progress,
// authentication and physical isolation separately.
class RegisteredHostHeartbeat final {
public:
    static constexpr std::uint8_t mav_type_gcs=6, mav_autopilot_invalid=8;
    bool bind(const HostHeartbeatBinding &binding)noexcept
    {
        if(bound_ || !valid(binding))return false;
        binding_=binding;bound_=true;return true;
    }
    HeartbeatRecord record(std::uint8_t system,std::uint8_t component,
        std::uint8_t type,std::uint8_t autopilot,std::uint64_t original_receiver_hrt_us)noexcept
    {
        if(!bound_)return HeartbeatRecord::Unbound;
        // Other vehicles/GCS/components must not update even the time watermark.
        if(system!=binding_.host_system || component!=binding_.host_component ||
           type!=mav_type_gcs || autopilot!=mav_autopilot_invalid)return HeartbeatRecord::IgnoredTuple;
        if(original_receiver_hrt_us<=binding_.registration_hrt_us || original_receiver_hrt_us<stamp_)
            return HeartbeatRecord::RejectedTime;
        if(original_receiver_hrt_us==stamp_)return HeartbeatRecord::Duplicate;
        if(generation_==UINT64_MAX)return HeartbeatRecord::Exhausted;
        stamp_=original_receiver_hrt_us;++generation_;return HeartbeatRecord::Recorded;
    }
    HostHeartbeatSnapshot snapshot(std::uint64_t now,std::uint64_t max_age_us)const noexcept
    {
        HostHeartbeatSnapshot out{};out.max_age_us=max_age_us;
        if(!bound_)return out;
        out.binding=binding_;out.original_receiver_hrt_us=stamp_;out.receipt_generation=generation_;
        if(!stamp_){out.freshness=HeartbeatFreshness::Missing;return out;}
        out.source_system=binding_.host_system;out.source_component=binding_.host_component;
        out.mav_type=mav_type_gcs;out.autopilot=mav_autopilot_invalid;
        if(!max_age_us || max_age_us>UINT64_MAX-stamp_){out.freshness=HeartbeatFreshness::InvalidAge;return out;}
        out.original_valid_until_us=stamp_+max_age_us;
        if(!now || now<stamp_)out.freshness=HeartbeatFreshness::InvalidClock;
        else out.freshness=now<=out.original_valid_until_us?HeartbeatFreshness::Fresh:HeartbeatFreshness::Expired;
        return out; // Preserve retained record fields.
    }
    void retire()noexcept {binding_={};stamp_=generation_=0;bound_=false;}
private:
    HostHeartbeatBinding binding_{};
    std::uint64_t stamp_{},generation_{};
    bool bound_{};
};

} // namespace gpenmpc_rfly_stream
