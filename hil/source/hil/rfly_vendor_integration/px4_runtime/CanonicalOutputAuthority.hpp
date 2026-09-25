#pragma once

#include "ExecutionAuthority.hpp"
#include "BoardSafetyEvidence.hpp"
#include "../px4_stream/RflyStreamAuthority.hpp"
#include "InheritingMutex.hpp"
#include <cmath>
#include <cstring>

namespace gpenmpc_rfly_px4 {

enum class LeaseFault:std::uint8_t {None,Configuration,Guard,IdentityMismatch,Control,
    Time,TokenMismatch,Unconsumed,Payload,Generation,NoLease,Revoked};
struct LeaseDiagnostics {
    std::uint64_t validated{0},installed{0},confirmed{0},accepted{0},revoked{0};
    std::uint64_t first_fault_us{0},last_consumed_output_us{0},original_expiry_us{0};
    unsigned last_consumed_uorb_generation{0};
    LeaseFault first_fault{LeaseFault::None};
    bool consumed{false};
};

// Guard.observe() must obtain actual evidence, including its locally created
// session. It is called ONLY while the PI mutex is held: real uORB Subscription
// objects are not accessed concurrently by the control and MAVLink tasks.
// Clock is actual HRT in the target instantiation, an explicit mock in tests.
// This class does not infer physical isolation from host intent or USB power.
// It provides no automatic NativeOutputsSim fallback on any canonical fault.
template<class Guard,class Clock>
class CanonicalOutputAuthority final : public Authority,public gpenmpc_rfly_stream::Authority {
public:
    CanonicalOutputAuthority(Guard &guard,Clock &clock,const Identity &bound,
                             std::uint64_t frozen_telemetry_max_age_us,
                             std::uint64_t frozen_commander_max_age_us=0)noexcept:
        guard_(guard),clock_(clock),identity_(bound),telemetry_max_age_us_(frozen_telemetry_max_age_us),
        commander_max_age_us_(frozen_commander_max_age_us?frozen_commander_max_age_us:frozen_telemetry_max_age_us)
    {
        if(!mutex_.ready() || !telemetry_max_age_us_ || commander_max_age_us_<telemetry_max_age_us_ ||
           !identity_.uid || !identity_.boot_generation ||
           !identity_.system || !identity_.component){
            diagnostics_.first_fault=LeaseFault::Configuration;
        }
    }
    CanonicalOutputAuthority(const CanonicalOutputAuthority &)=delete;
    CanonicalOutputAuthority &operator=(const CanonicalOutputAuthority &)=delete;

    // An explicit selection operation after actual Direct/HIL ownership is
    // observed. It sends no mode/arm/parameter command. Stream polling while
    // still disarmed and unselected is normal and must not poison the owner.
    bool begin_direct()noexcept
    {
        if(!mutex_.lock())return false;
        const bool ok=observe(clock_.now(),true);
        if(ok)selected_=true;
        return finish(ok);
    }

    bool observed_identity(Identity &out)noexcept override
    {
        if(!mutex_.lock())return false;
        const bool ok=observe(clock_.now(),false);
        if(ok)out=evidence_.identity;
        return finish(ok);
    }

    bool validate(const Token &token,const vehicle_status_s &status,
                  const vehicle_control_mode_s &mode,const offboard_control_mode_s &offboard,
                  std::uint64_t now,std::uint64_t &original_expiry)noexcept override
    {
        original_expiry=0;
        if(!mutex_.lock())return false;
        bool ok=selected_ && observe(now,true);
        if(ok && (!same_control(status,mode) || !fresh(offboard.timestamp) || !offboard.direct_actuator ||
            offboard.position || offboard.velocity || offboard.acceleration || offboard.attitude ||
            offboard.body_rate || offboard.thrust_and_torque))
            ok=fault(LeaseFault::Control,now);
        if(ok && (!(token.identity==identity_) ||
            token.publication_path!=gpenmpc_consumption::PublicationPath::DirectCanonicalMotors ||
            !token.transaction || !token.output_generation || !token.sample_generation ||
            !token.timestamp_sample_us || !token.control_tick_us ||
            token.timestamp_sample_us>token.control_tick_us || token.control_tick_us>now ||
            token.reference_valid_until_us<now || token.outer_valid_until_us<now))
            ok=fault(LeaseFault::TokenMismatch,now);
        if(ok && (pending_validation_ || (lease_present_ && !diagnostics_.consumed)))
            ok=fault(LeaseFault::Unconsumed,now);
        if(ok && diagnostics_.installed &&
            (token.transaction<=last_transaction_ || token.output_generation<=last_output_generation_))
            ok=fault(LeaseFault::Generation,now);
        if(ok){
            validated_token_=token;
            validation_expiry_=minimum(evidence_.original_valid_until_us,
                minimum(token.reference_valid_until_us,token.outer_valid_until_us));
            pending_validation_=true;
            original_expiry=validation_expiry_;
            ++diagnostics_.validated;
        }
        return finish(ok);
    }

    bool install_lease(const Token &token,const actuator_outputs_s &output,
                       std::uint64_t original_expiry)noexcept override
    {
        if(!mutex_.lock())return false;
        const std::uint64_t now=clock_.now();
        bool ok=selected_ && observe(now,true);
        if(ok && (!pending_validation_ || !(token==validated_token_)))ok=fault(LeaseFault::TokenMismatch,now);
        if(ok && (!original_expiry || original_expiry>validation_expiry_ || now>original_expiry ||
            !output.timestamp || output.timestamp<token.control_tick_us || output.timestamp>now))
            ok=fault(LeaseFault::Time,now);
        if(ok && output.noutputs!=0)ok=fault(LeaseFault::Payload,now);
        for(unsigned i=0;ok && i<16;++i){
            const float value=output.output[i];
            if(!std::isfinite(value) || value<0.f || value>1.f || (i>=6 && value>0.f))
                ok=fault(LeaseFault::Payload,now);
        }
        if(ok){
            lease_=output;
            lease_expiry_=original_expiry;
            pending_validation_=false;
            lease_present_=true;
            confirmed_=false;
            last_transaction_=token.transaction;
            last_output_generation_=token.output_generation;
            ++diagnostics_.installed;
            diagnostics_.consumed=false;
            diagnostics_.original_expiry_us=original_expiry;
        }
        return finish(ok);
    }

    bool confirm_publication(const Token &token,std::uint64_t publication_receipt,
                             std::uint64_t commit_completed)noexcept override
    {
        if(!mutex_.lock())return false;
        const std::uint64_t now=clock_.now();
        bool ok=selected_ && observe(now,true);
        if(ok && (!lease_present_ || confirmed_ || !(token==validated_token_)))ok=fault(LeaseFault::TokenMismatch,now);
        if(ok && (publication_receipt<lease_.timestamp || commit_completed<publication_receipt ||
            now<commit_completed || now>lease_expiry_))ok=fault(LeaseFault::Time,now);
        if(ok){confirmed_=true;++diagnostics_.confirmed;}
        return finish(ok);
    }

    gpenmpc_rfly_stream::Source choose(std::uint64_t now,const vehicle_status_s &status,
                                      const vehicle_control_mode_s &mode)noexcept override
    {
        if(!mutex_.lock())return gpenmpc_rfly_stream::Source::Unavailable;
        if(!selected_){(void)finish(false);return gpenmpc_rfly_stream::Source::Unavailable;}
        bool ok=selected_ && observe(now,true);
        if(ok && !same_control(status,mode))ok=fault(LeaseFault::Control,now);
        if(ok && lease_present_ && !diagnostics_.consumed && now>lease_expiry_)
            ok=fault(LeaseFault::Time,now);
        const bool selected=ok && lease_present_ && confirmed_ && !diagnostics_.consumed;
        return finish(selected)?gpenmpc_rfly_stream::Source::RflyOutputs:gpenmpc_rfly_stream::Source::Unavailable;
    }

    bool accept(const gpenmpc_rfly_stream::Observation &sample,std::uint64_t now,
                gpenmpc_rfly_stream::OriginalValidity &validity)noexcept override
    {
        validity.valid_until_us=0;
        if(!mutex_.lock())return false;
        bool ok=observe(now,true);
        if(ok && (!lease_present_ || !confirmed_ || diagnostics_.consumed))ok=fault(LeaseFault::NoLease,now);
        if(ok && (sample.source!=gpenmpc_rfly_stream::Source::RflyOutputs ||
            !same_control(sample.status,sample.control_mode)))ok=fault(LeaseFault::Control,now);
        if(ok && (now>lease_expiry_ || sample.output.timestamp>now))ok=fault(LeaseFault::Time,now);
        if(ok && (sample.output.timestamp!=lease_.timestamp || sample.output.noutputs!=lease_.noutputs ||
            std::memcmp(sample.output.output,lease_.output,sizeof(lease_.output))!=0))
            ok=fault(LeaseFault::Payload,now);
        // Real Subscription generation may skip or wrap; it must advance by a
        // positive modular half-range. Do not invent an exact +1 cadence.
        if(ok && diagnostics_.accepted){
            const unsigned delta=sample.output_uorb_generation-diagnostics_.last_consumed_uorb_generation;
            if(!delta || delta>~0u/2u)ok=fault(LeaseFault::Generation,now);
        }
        if(ok){
            diagnostics_.consumed=true;
            ++diagnostics_.accepted;
            diagnostics_.last_consumed_output_us=sample.output.timestamp;
            diagnostics_.last_consumed_uorb_generation=sample.output_uorb_generation;
            validity.valid_until_us=minimum(lease_expiry_,evidence_.original_valid_until_us);
        }
        // Consumption means this stream accepted the exact uORB message. The
        // void MAVLink send API is NOT a wire/CopterSim acknowledgement.
        return finish(ok);
    }

    void revoke()noexcept override
    {
        if(!mutex_.lock())return;
        ++diagnostics_.revoked;
        (void)fault(LeaseFault::Revoked,clock_.now());
        (void)finish(false);
    }
    bool diagnostic_snapshot(LeaseDiagnostics &out)noexcept
    {
        if(!mutex_.lock())return false;
        out=diagnostics_;
        return finish(true);
    }

private:
    bool finish(bool result)noexcept{return mutex_.unlock() && result;}
    static std::uint64_t minimum(std::uint64_t a,std::uint64_t b)noexcept{return a<b?a:b;}
    bool fault(LeaseFault reason,std::uint64_t now)noexcept
    {
        if(diagnostics_.first_fault==LeaseFault::None){diagnostics_.first_fault=reason;diagnostics_.first_fault_us=now;}
        pending_validation_=false;lease_present_=false;selected_=false;confirmed_=false;
        return false;
    }
    bool observe(std::uint64_t now,bool active)noexcept
    {
        if(diagnostics_.first_fault!=LeaseFault::None)return false;
        if(!now || !guard_.observe(evidence_) || !evidence_.require_identity_only())return fault(LeaseFault::Guard,now);
        if(!(evidence_.identity==identity_) || evidence_.boot_session!=GuardFact::Pass)return fault(LeaseFault::IdentityMismatch,now);
        const auto bound_expiry=active?evidence_.original_valid_until_us:evidence_.identity_valid_until_us;
        if(!evidence_.observation_us || evidence_.observation_us<now ||
           bound_expiry<evidence_.observation_us)return fault(LeaseFault::Time,now);
        if(active && (!evidence_.require_active_direct() || !evidence_.require_physical_facts()))
            return fault(LeaseFault::Guard,now);
        return true;
    }
    bool same_control(const vehicle_status_s &status,const vehicle_control_mode_s &mode)const noexcept
    {
        // Independent subscriptions may legally see different fresh updates.
        // Match the actual ownership shape and each ORIGINAL age, not exact
        // callback arrival order or equality of asynchronously read timestamps.
        return fresh_commander(status.timestamp) && fresh_commander(mode.timestamp) &&
            status.hil_state==vehicle_status_s::HIL_STATE_ON &&
            status.arming_state==vehicle_status_s::ARMING_STATE_ARMED &&
            status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD &&
            mode.flag_armed && mode.flag_control_offboard_enabled &&
            !mode.flag_multicopter_position_control_enabled && !mode.flag_control_manual_enabled &&
            !mode.flag_control_auto_enabled && !mode.flag_control_altitude_enabled &&
            !mode.flag_control_climb_rate_enabled && !mode.flag_control_termination_enabled &&
            !mode.flag_control_position_enabled && !mode.flag_control_velocity_enabled &&
            !mode.flag_control_acceleration_enabled && !mode.flag_control_attitude_enabled &&
            !mode.flag_control_rates_enabled && !mode.flag_control_allocation_enabled;
    }
    bool fresh(std::uint64_t stamp)const noexcept
    {
        return stamp && stamp<=evidence_.observation_us && evidence_.observation_us-stamp<=telemetry_max_age_us_;
    }
    bool fresh_commander(std::uint64_t stamp)const noexcept
    {
        return stamp && stamp<=evidence_.observation_us &&
            evidence_.observation_us-stamp<=commander_max_age_us_;
    }
    Guard &guard_;
    Clock &clock_;
    const Identity identity_;
    const std::uint64_t telemetry_max_age_us_;
    const std::uint64_t commander_max_age_us_;
    InheritingMutex mutex_{};
    BoardSafetyEvidence evidence_{};
    Token validated_token_{};
    actuator_outputs_s lease_{};
    std::uint64_t validation_expiry_{0},lease_expiry_{0};
    std::uint64_t last_transaction_{0},last_output_generation_{0};
    bool pending_validation_{false},lease_present_{false},selected_{false},confirmed_{false};
    LeaseDiagnostics diagnostics_{};
};
}
