#pragma once
#include "../px4_stream/RegisteredHostHeartbeat.hpp"

namespace gpenmpc_rfly_px4 {

enum class PrestreamFact:std::uint8_t {Unknown,Pass,Fail};
struct PrestreamPrerequisites {
    // These are results of the actual Context/session/physical/atomic-source
    // checks. This policy does not perform those observations or grant them.
    PrestreamFact session{PrestreamFact::Unknown},physical{PrestreamFact::Unknown},atomic_source{PrestreamFact::Unknown};
    bool stop{},land{},fault{};
};
enum class PrestreamState:std::uint8_t {Dormant,Waiting,Enabled,Revoked};
enum class PrestreamAction:std::uint8_t {None,Waiting,EmitDirect,NoChange,Revoked};
enum class PrestreamReason:std::uint8_t {
    None,Configuration,Clock,Stop,Land,Fault,Prerequisites,HeartbeatMissing,HeartbeatInvalid,ReceiptChanged
};
struct DirectOffboardSuggestion {
    // Value suggestion for a real offboard_control_mode_s publication. It is
    // NOT a uORB publication, Commander mode switch, arm command or authority.
    std::uint64_t timestamp{};
    bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};
};
struct PrestreamDecision {
    PrestreamAction action{PrestreamAction::None};
    PrestreamReason reason{PrestreamReason::None};
    DirectOffboardSuggestion offboard{};
    std::uint64_t original_valid_until_us{},receipt_generation{};
};

// Single owning Context/task calls this policy. No callbacks, locks or I/O.
// A new original heartbeat creates at most one publication suggestion. It
// never substitutes poll time for the original receiver timestamp. Once any
// enabled prerequisite/liveness fails, stop/LAND/fault occurs, or time goes
// backwards, this object can never emit again. Session identity/native LAND
// recovery remain independent: no mutation/revoke of their objects occurs.
class DirectOffboardPrestream final {
public:
    bool start(const gpenmpc_rfly_stream::HostHeartbeatBinding &binding,std::uint64_t frozen_max_age_us)noexcept
    {
        if(state_!=PrestreamState::Dormant)return false; // never resets a running/revoked lifetime
        if(!gpenmpc_rfly_stream::valid(binding) || !frozen_max_age_us){
            revoke(PrestreamReason::Configuration);return false;
        }
        binding_=binding;max_age_=frozen_max_age_us;last_poll_=binding.registration_hrt_us;
        state_=PrestreamState::Waiting;return true;
    }
    PrestreamDecision poll(std::uint64_t now,const gpenmpc_rfly_stream::HostHeartbeatSnapshot &heartbeat,
        const PrestreamPrerequisites &p)noexcept
    {
        if(state_==PrestreamState::Revoked)return decision(PrestreamAction::Revoked);
        if(p.stop){revoke(PrestreamReason::Stop);return decision(PrestreamAction::Revoked);}
        if(p.land){revoke(PrestreamReason::Land);return decision(PrestreamAction::Revoked);}
        if(p.fault){revoke(PrestreamReason::Fault);return decision(PrestreamAction::Revoked);}
        if(state_==PrestreamState::Dormant)return {};
        if(!now || now<last_poll_){revoke(PrestreamReason::Clock);return decision(PrestreamAction::Revoked);}
        last_poll_=now;
        if(p.session!=PrestreamFact::Pass || p.physical!=PrestreamFact::Pass || p.atomic_source!=PrestreamFact::Pass)
            return unavailable(PrestreamReason::Prerequisites);
        using gpenmpc_rfly_stream::HeartbeatFreshness;
        if(heartbeat.freshness==HeartbeatFreshness::Missing || heartbeat.freshness==HeartbeatFreshness::Unbound)
            return unavailable(PrestreamReason::HeartbeatMissing);
        const auto stamp=heartbeat.original_receiver_hrt_us;
        if(!(heartbeat.binding==binding_) || heartbeat.freshness!=HeartbeatFreshness::Fresh ||
           heartbeat.max_age_us!=max_age_ || stamp<=binding_.registration_hrt_us || stamp>now ||
           max_age_>UINT64_MAX-stamp || heartbeat.original_valid_until_us!=stamp+max_age_ ||
           now>heartbeat.original_valid_until_us || !heartbeat.receipt_generation ||
           heartbeat.source_system!=binding_.host_system || heartbeat.source_component!=binding_.host_component ||
           heartbeat.mav_type!=gpenmpc_rfly_stream::RegisteredHostHeartbeat::mav_type_gcs ||
           heartbeat.autopilot!=gpenmpc_rfly_stream::RegisteredHostHeartbeat::mav_autopilot_invalid)
            return unavailable(PrestreamReason::HeartbeatInvalid);
        if(last_receipt_ && (heartbeat.receipt_generation<last_receipt_ || stamp<last_stamp_ ||
           ((heartbeat.receipt_generation==last_receipt_)!=(stamp==last_stamp_)))){
            revoke(PrestreamReason::ReceiptChanged);return decision(PrestreamAction::Revoked);
        }
        if(heartbeat.receipt_generation==last_receipt_)return decision(PrestreamAction::NoChange);
        last_receipt_=heartbeat.receipt_generation;last_stamp_=stamp;last_until_=heartbeat.original_valid_until_us;
        state_=PrestreamState::Enabled;reason_=PrestreamReason::None;
        auto out=decision(PrestreamAction::EmitDirect);out.offboard.timestamp=stamp;out.offboard.direct_actuator=true;
        return out;
    }
    void stop()noexcept {revoke(PrestreamReason::Stop);}
    PrestreamState state()const noexcept{return state_;}
    PrestreamReason first_reason()const noexcept{return reason_;}
private:
    void revoke(PrestreamReason why)noexcept
    {
        if(state_!=PrestreamState::Revoked){state_=PrestreamState::Revoked;reason_=why;}
    }
    PrestreamDecision unavailable(PrestreamReason why)noexcept
    {
        if(state_==PrestreamState::Enabled){revoke(why);return decision(PrestreamAction::Revoked);}
        PrestreamDecision out{};out.action=PrestreamAction::Waiting;out.reason=why;return out;
    }
    PrestreamDecision decision(PrestreamAction action)const noexcept
    {
        PrestreamDecision out{};out.action=action;out.reason=reason_;
        out.original_valid_until_us=last_until_;out.receipt_generation=last_receipt_;return out;
    }
    gpenmpc_rfly_stream::HostHeartbeatBinding binding_{};
    std::uint64_t max_age_{},last_poll_{},last_receipt_{},last_stamp_{},last_until_{};
    PrestreamState state_{PrestreamState::Dormant};
    PrestreamReason reason_{PrestreamReason::None};
};

} // namespace gpenmpc_rfly_px4
