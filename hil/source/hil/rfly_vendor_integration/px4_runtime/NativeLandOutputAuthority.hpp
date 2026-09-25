#pragma once

#include "BoardSafetyEvidence.hpp"
#include "InheritingMutex.hpp"
#include "NativeLandModeShape.hpp"
#include "../px4_stream/RflyStreamAuthority.hpp"
#include <cmath>

namespace gpenmpc_rfly_px4 {

enum class NativeLandFault : std::uint8_t {
    None, Configuration, Guard, IdentityMismatch, Clock, Mode, Request, Payload,
    Generation, Revoked
};
enum class NativeLandActivation : std::uint8_t {None,RequestedLand,ObservedLand,ObservedDisarmedZero};
struct NativeLandDiagnostics {
    NativeLandFault first_fault{NativeLandFault::None};
    std::uint64_t first_fault_us{0}, request_us{0}, original_deadline_us{0};
    std::uint64_t accepted{0}, disarmed_zero_accepted{0}, last_output_us{0};
    unsigned last_uorb_generation{0};
    bool selected{false}, observed_disarmed{false};
    NativeLandActivation activation{NativeLandActivation::None};
};

// A distinct safety-tail source, never CanonicalOutputAuthority's implicit
// fallback. The owning context must revoke and quiescently detach the direct
// source BEFORE binding this one through the existing single-owner registry.
// No arm/mode/parameter command is sent here. begin_native_land requires the
// original requested-LAND event and independently observed PX4 native ownership.
// This class does not equate a MAVLink send with CopterSim consumption/landing.
template<class Guard,class Clock>
class NativeLandOutputAuthority final : public gpenmpc_rfly_stream::Authority {
public:
    NativeLandOutputAuthority(Guard &guard,Clock &clock,
        const gpenmpc_consumption::Identity &identity,std::uint64_t frozen_max_age_us,
        std::uint64_t frozen_commander_max_age_us=0) noexcept:
        guard_(guard),clock_(clock),identity_(identity),max_age_(frozen_max_age_us),
        commander_max_age_(frozen_commander_max_age_us?frozen_commander_max_age_us:frozen_max_age_us)
    {
        if(!mutex_.ready() || !max_age_ || commander_max_age_<max_age_ ||
           !identity_.uid || !identity_.boot_generation ||
           !identity_.system || !identity_.component)
            diagnostics_.first_fault=NativeLandFault::Configuration;
    }

    bool begin_native_land(std::uint64_t original_request_us,std::uint64_t original_deadline_us,
        const vehicle_status_s &status,const vehicle_control_mode_s &mode) noexcept
    {return begin(NativeLandActivation::RequestedLand,original_request_us,original_deadline_us,status,mode);}

    // Safety supervisor already observed native LAND (including PX4 failsafe),
    // or actual disarm. Its original status/mode event is NOT called a request
    // or an accepted COMMAND_ACK. Disarmed selection can transmit only zeros.
    bool begin_observed_safety_state(std::uint64_t original_deadline_us,
        const vehicle_status_s &status,const vehicle_control_mode_s &mode) noexcept
    {
        const auto event=minimum(status.timestamp,mode.timestamp);
        const auto kind=disarmed_control_shape(status,mode)?NativeLandActivation::ObservedDisarmedZero:
            NativeLandActivation::ObservedLand;
        return begin(kind,event,original_deadline_us,status,mode);
    }

private:
    bool begin(NativeLandActivation activation,std::uint64_t original_request_us,std::uint64_t original_deadline_us,
        const vehicle_status_s &status,const vehicle_control_mode_s &mode) noexcept
    {
        if(!mutex_.lock())return false;
        const auto now=clock_.now();
        bool ok=observe(now);
        if(ok && (diagnostics_.selected || !original_request_us || original_request_us>now ||
            original_deadline_us<=original_request_us || now>original_deadline_us))
            ok=fault(NativeLandFault::Request,now);
        const bool disarmed=activation==NativeLandActivation::ObservedDisarmedZero;
        const bool shape=disarmed?(common(status,mode)&&disarmed_control_shape(status,mode)&&
            evidence_.disarmed_control==GuardFact::Pass):active_land(status,mode);
        if(ok && (!shape || status.timestamp<original_request_us ||
            mode.timestamp<original_request_us))ok=fault(NativeLandFault::Mode,now);
        if(ok){
            diagnostics_.request_us=original_request_us;
            diagnostics_.original_deadline_us=original_deadline_us;
            diagnostics_.selected=true;
            diagnostics_.activation=activation;diagnostics_.observed_disarmed=disarmed;
        }
        return finish(ok);
    }

public:

    gpenmpc_rfly_stream::Source choose(std::uint64_t now,const vehicle_status_s &status,
        const vehicle_control_mode_s &mode) noexcept override
    {
        if(!mutex_.lock())return gpenmpc_rfly_stream::Source::Unavailable;
        if(!diagnostics_.selected){(void)finish(false);return gpenmpc_rfly_stream::Source::Unavailable;}
        bool ok=observe(now);
        if(ok && now>diagnostics_.original_deadline_us)ok=fault(NativeLandFault::Clock,now);
        if(ok && !selected_shape(status,mode))ok=fault(NativeLandFault::Mode,now);
        if(ok && status.arming_state==vehicle_status_s::ARMING_STATE_DISARMED)diagnostics_.observed_disarmed=true;
        return finish(ok)?gpenmpc_rfly_stream::Source::NativeOutputsSim:gpenmpc_rfly_stream::Source::Unavailable;
    }

    bool accept(const gpenmpc_rfly_stream::Observation &sample,std::uint64_t now,
        gpenmpc_rfly_stream::OriginalValidity &original) noexcept override
    {
        original={};
        if(!mutex_.lock())return false;
        bool ok=diagnostics_.selected && observe(now);
        if(ok && (sample.source!=gpenmpc_rfly_stream::Source::NativeOutputsSim ||
            !selected_shape(sample.status,sample.control_mode)))ok=fault(NativeLandFault::Mode,now);
        // This uORB subscription is intentionally unread during direct control.
        // Its first update can therefore be the cached pre-LAND publication.
        // Discard that unselected history, without transmitting or renewing it,
        // and allow a genuinely new native publication within the ORIGINAL
        // tail deadline. After one accepted output, regressions remain fatal.
        if(ok && !diagnostics_.accepted && sample.output.timestamp &&
            sample.output.timestamp<diagnostics_.request_us &&
            now<=diagnostics_.original_deadline_us)return finish(false);
        // The tail retains the source timestamp and validates its own output.
        const auto expiry=guard_original_expiry(sample.output.timestamp,max_age_);
        if(ok && (!fresh(sample.output.timestamp) || sample.output.timestamp<diagnostics_.request_us ||
            !expiry || now>diagnostics_.original_deadline_us))ok=fault(NativeLandFault::Clock,now);
        if(ok && (!sample.output.noutputs || sample.output.noutputs>16))ok=fault(NativeLandFault::Payload,now);
        const bool disarmed=sample.status.arming_state==vehicle_status_s::ARMING_STATE_DISARMED;
        for(unsigned i=0;ok && i<16;++i){
            const float x=sample.output.output[i];
            if(!std::isfinite(x) || x< -1.f || x>1.f ||
                ((disarmed || i>=sample.output.noutputs) && (x<0.f || x>0.f)))
                ok=fault(NativeLandFault::Payload,now);
        }
        if(ok && diagnostics_.accepted){
            const unsigned delta=sample.output_uorb_generation-diagnostics_.last_uorb_generation;
            if(!delta || delta>~0u/2u || sample.output.timestamp<=diagnostics_.last_output_us)
                ok=fault(NativeLandFault::Generation,now);
        }
        if(ok){
            if(disarmed)diagnostics_.observed_disarmed=true;
            original.valid_until_us=minimum(expiry,minimum(diagnostics_.original_deadline_us,
                minimum(evidence_.identity_valid_until_us,
                    minimum(guard_original_expiry(sample.status.timestamp,commander_max_age_),
                        guard_original_expiry(sample.control_mode.timestamp,commander_max_age_)))));
            if(original.valid_until_us<evidence_.observation_us){original={};ok=fault(NativeLandFault::Clock,now);}
        }
        if(ok){
            ++diagnostics_.accepted;if(disarmed)++diagnostics_.disarmed_zero_accepted;
            diagnostics_.last_output_us=sample.output.timestamp;
            diagnostics_.last_uorb_generation=sample.output_uorb_generation;
        }
        if(!finish(ok)){original={};return false;}
        return true;
    }

    void revoke() noexcept
    {
        if(!mutex_.lock())return;
        (void)fault(NativeLandFault::Revoked,clock_.now());(void)finish(false);
    }
    bool diagnostic_snapshot(NativeLandDiagnostics &out) noexcept
    {if(!mutex_.lock())return false;out=diagnostics_;return finish(true);}

private:
    static std::uint64_t minimum(std::uint64_t a,std::uint64_t b) noexcept{return a<b?a:b;}
    bool finish(bool result) noexcept{return mutex_.unlock()&&result;}
    bool fault(NativeLandFault f,std::uint64_t now) noexcept
    {
        if(diagnostics_.first_fault==NativeLandFault::None){diagnostics_.first_fault=f;diagnostics_.first_fault_us=now;}
        diagnostics_.selected=false;return false;
    }
    bool observe(std::uint64_t now) noexcept
    {
        if(diagnostics_.first_fault!=NativeLandFault::None)return false;
        if(!now || now<last_now_ || !guard_.observe(evidence_) || !evidence_.require_identity_only() ||
            !evidence_.require_physical_facts() || evidence_.usb_transport!=GuardFact::Pass ||
            evidence_.hil_configuration!=GuardFact::Pass)return fault(NativeLandFault::Guard,now);
        if(!(evidence_.identity==identity_) || evidence_.boot_session!=GuardFact::Pass)
            return fault(NativeLandFault::IdentityMismatch,now);
        if(evidence_.observation_us<now || evidence_.identity_valid_until_us<evidence_.observation_us ||
            !fresh_commander(evidence_.status_timestamp_us) || !fresh_commander(evidence_.mode_timestamp_us) ||
            !fresh(evidence_.power_timestamp_us))return fault(NativeLandFault::Clock,now);
        last_now_=now;return true;
    }
    bool fresh(std::uint64_t stamp) const noexcept
    {return stamp && stamp<=evidence_.observation_us && evidence_.observation_us-stamp<=max_age_;}
    bool fresh_commander(std::uint64_t stamp) const noexcept
    {return stamp && stamp<=evidence_.observation_us && evidence_.observation_us-stamp<=commander_max_age_;}
    bool common(const vehicle_status_s &s,const vehicle_control_mode_s &m) const noexcept
    {
        return fresh_commander(s.timestamp)&&fresh_commander(m.timestamp) &&
            s.system_id==identity_.system && s.component_id==identity_.component &&
            s.vehicle_type==vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !s.is_vtol &&
            !s.in_transition_mode && !s.in_transition_to_fw &&
            s.hil_state==vehicle_status_s::HIL_STATE_ON && !m.flag_control_termination_enabled;
    }
    bool active_land(const vehicle_status_s &s,const vehicle_control_mode_s &m) const noexcept
    {
        // Shape comes from this target's commander/ModeUtil/control_mode.cpp,
        // not from receipt labels or a requested mode acknowledged elsewhere.
        return common(s,m) && native_land_mode_shape(s,m) && evidence_.native_land_mode==GuardFact::Pass;
    }
    bool selected_shape(const vehicle_status_s &s,const vehicle_control_mode_s &m) const noexcept
    {
        return (!diagnostics_.observed_disarmed && active_land(s,m)) || (common(s,m) && disarmed_control_shape(s,m) &&
            evidence_.disarmed_control==GuardFact::Pass);
    }
    Guard &guard_;Clock &clock_;
    const gpenmpc_consumption::Identity identity_;
    const std::uint64_t max_age_;
    const std::uint64_t commander_max_age_;
    InheritingMutex mutex_{};
    BoardSafetyEvidence evidence_{};
    NativeLandDiagnostics diagnostics_{};
    std::uint64_t last_now_{0};
};
} // namespace gpenmpc_rfly_px4
