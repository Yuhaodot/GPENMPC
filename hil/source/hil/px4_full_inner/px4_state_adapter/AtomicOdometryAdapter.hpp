#pragma once
// Only actual generated topic storage is accepted. No reconstructed LP/ATT
// message, quaternion synthesis, time rebasing, scheduler or board authority.
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include "../consumption/ConsumptionBinding.hpp"
#include "../portable/CanonicalPortable.hpp"
#include <cstring>

namespace gpenmpc_odometry {
using gpenmpc_consumption::Identity;
using gpenmpc_consumption::Estimator;
enum class Coordinates : std::uint8_t { Unspecified, ExplicitTranslatedNed };
enum class Failure : std::uint8_t {
    None, Configuration, Identity, TopicInstance, Frames, Nonfinite, Quaternion,
    TimeOrder, Stale, Generation, SampleDelta, Reset, ReceiptRegression
};
struct Configuration {
    Identity identity{};
    // Must be ORB_ID(vehicle_odometry) and the selected instance in the real
    // integration, not merely any topic with the same message layout.
    const void *vehicle_odometry_topic{nullptr};
    std::uint8_t instance{0}, initial_reset_counter{0};
    Coordinates coordinates{Coordinates::Unspecified};
    gpenmpc_portable::Array<double,3> task_origin_ned_m{};
    std::uint64_t sample_max_age_us{0}; // explicit caller limit; no default gate
};
class AtomicOdometryAdapter;
class Snapshot final {
public:
    bool valid() const noexcept { return valid_; }
    bool board_authority() const noexcept { return false; }
    const vehicle_odometry_s &raw() const noexcept { return raw_; }
    const Estimator &estimator() const noexcept { return estimator_; }
    const gpenmpc_portable::Array<double,3> &task_origin_ned_m() const noexcept { return origin_; }
    std::uint32_t subscription_generation() const noexcept { return generation_; }
    std::uint64_t generation_delta() const noexcept { return generation_delta_; }
    std::uint64_t actual_sample_delta_us() const noexcept { return sample_delta_us_; }
    // Provenance supplied to, and checked by, the original ingest call. Not a
    // host-provided estimator selector or a permission to publish controls.
    const void *source_topic() const noexcept { return source_topic_; }
    std::uint8_t source_instance() const noexcept { return source_instance_; }
    // Control continuity only. Raw state/counters are never rewritten. This
    // can be set only after native EKF companion messages identify a yaw-only
    // correction with unchanged NED position/velocity reset epochs.
    bool heading_reset_from(std::uint8_t prior) const noexcept {
        return valid_&&heading_reset_verified_&&prior==heading_reset_prior_&&
            estimator_.reset_counter==static_cast<std::uint8_t>(prior+1);
    }
    // Bind the selected odometry state.
    // The existing executor must still verify its independently built x13.
    bool bind_state(gpenmpc_consumption::Input &destination) const noexcept {
        if (!valid_) return false;
        destination.state=estimator_; return true;
    }
private:
    friend class AtomicOdometryAdapter;
    bool valid_{false};
    vehicle_odometry_s raw_{};
    Estimator estimator_{};
    gpenmpc_portable::Array<double,3> origin_{};
    std::uint32_t generation_{0};
    std::uint64_t generation_delta_{0}, sample_delta_us_{0};
    const void *source_topic_{nullptr};
    std::uint8_t source_instance_{UINT8_MAX};
    bool heading_reset_verified_{false};std::uint8_t heading_reset_prior_{};
};

class AtomicOdometryAdapter final {
public:
    explicit AtomicOdometryAdapter(Configuration c) noexcept : config_(c), accepted_reset_(c.initial_reset_counter) {
        if (!c.identity.uid || !c.identity.boot_generation || !c.identity.system || !c.identity.component ||
            !c.vehicle_odometry_topic || !c.sample_max_age_us ||
            c.coordinates!=Coordinates::ExplicitTranslatedNed ||
            !gpenmpc_consumption::finite(c.task_origin_ned_m)) fail(Failure::Configuration);
    }
    bool ingest(const vehicle_odometry_s &message, std::uint32_t last_generation,
                std::uint64_t original_hrt_receipt_us, std::uint64_t validation_hrt_us,
                Identity observed_identity, const void *topic, std::uint8_t instance,
                Snapshot &out, bool enforce_control_interval=true,
                std::uint64_t maximum_control_interval_us=gpenmpc_consumption::canonical_dt_us,
                std::uint64_t previous_control_sample_us=0,
                const vehicle_attitude_s *attitude=nullptr,
                const vehicle_local_position_s *position=nullptr) noexcept {
        out=Snapshot{};
        if (failure_!=Failure::None) return false;
        // Preserve the complete original message for first-failure diagnosis,
        // including optional covariance NaNs, quality and original raw bits.
        std::memcpy(&last_observed_,&message,sizeof(message)); observed_=true;
        if (!(observed_identity==config_.identity)) return fail(Failure::Identity);
        if (topic!=config_.vehicle_odometry_topic || instance!=config_.instance) return fail(Failure::TopicInstance);
        if (message.pose_frame!=vehicle_odometry_s::POSE_FRAME_NED ||
            message.velocity_frame!=vehicle_odometry_s::VELOCITY_FRAME_NED) return fail(Failure::Frames);
        for (unsigned j=0;j<3;++j)
            if (!std::isfinite(message.position[j]) || !std::isfinite(message.velocity[j]) ||
                !std::isfinite(message.angular_velocity[j])) return fail(Failure::Nonfinite);
        double q2=0; for(float q:message.q) {if(!std::isfinite(q)) return fail(Failure::Nonfinite);q2+=double(q)*double(q);}
        // Same representation tolerance as CanonicalFullInnerExecutor;
        // the raw quaternion is never normalized here.
        if (std::abs(std::sqrt(q2)-1.0)>1e-6) return fail(Failure::Quaternion);
        if (!message.timestamp_sample || !message.timestamp || !original_hrt_receipt_us ||
            message.timestamp_sample>message.timestamp || message.timestamp>original_hrt_receipt_us ||
            original_hrt_receipt_us>validation_hrt_us) return fail(Failure::TimeOrder);
        if (validation_hrt_us-message.timestamp_sample>config_.sample_max_age_us) return fail(Failure::Stale);
        if (!last_generation || (has_previous_ && last_generation<=previous_generation_)) return fail(Failure::Generation);
        std::uint64_t dt=0;
        // A refreshed, not-yet-executed capture is an observation, not an
        // integration step. The actual Io supplies its last installed token
        // as the control anchor; never replace it with an intermediate read.
        if(previous_control_sample_us&&(!has_previous_||!enforce_control_interval||
           previous_control_sample_us>previous_sample_us_))return fail(Failure::SampleDelta);
        if (has_previous_) {
            // A disarmed observation is not an executed integration step.
            // Retain its real delta, but do not require 100-Hz control cadence
            // while no control is running. Freshness, order, identity/reset
            // and all state checks above remain identical. Active ingestion
            // defaults to 10 ms. A zero maximum explicitly disables only the
            // upper interval gate; the installed-control anchor, monotonic
            // ordering and actual elapsed dt are still preserved.
            const auto interval_from=previous_control_sample_us?previous_control_sample_us:previous_sample_us_;
            if (message.timestamp_sample<=previous_sample_us_ ||
                (enforce_control_interval&&maximum_control_interval_us&&
                 message.timestamp_sample-interval_from>maximum_control_interval_us)) return fail(Failure::SampleDelta);
            if (original_hrt_receipt_us<=previous_receipt_us_ || message.timestamp<previous_publication_us_)
                return fail(Failure::ReceiptRegression);
            dt=message.timestamp_sample-interval_from;
        }
        const auto companion_fresh=[&](std::uint64_t sample,std::uint64_t published){
            return sample&&sample<=published&&
                published<=validation_hrt_us&&validation_hrt_us-sample<=config_.sample_max_age_us;
        };
        const bool companions=attitude&&position&&
            companion_fresh(attitude->timestamp_sample,attitude->timestamp)&&
            companion_fresh(position->timestamp_sample,position->timestamp);
        const bool reset=message.reset_counter!=accepted_reset_;
        if(reset){
            if(!companions||!have_reset_basis_||message.reset_counter!=static_cast<std::uint8_t>(accepted_reset_+1)||
               position->xy_reset_counter!=xy_reset_||position->z_reset_counter!=z_reset_||
               position->vxy_reset_counter!=vxy_reset_||position->vz_reset_counter!=vz_reset_)return fail(Failure::Reset);
            const auto next_heading=static_cast<std::uint8_t>(heading_reset_+1);
            if((position->heading_reset_counter!=heading_reset_&&position->heading_reset_counter!=next_heading)||
               (attitude->quat_reset_counter!=heading_reset_&&attitude->quat_reset_counter!=next_heading))return fail(Failure::Reset);
            // EKF2 publishes attitude before update(), then local position and
            // odometry after it. A genuine one-cycle companion lag is not an
            // invalid state nor permission to reuse the pre-reset snapshot.
            // No capture/clock is committed while awaiting the next actual
            // uORB publication; its source must pass the freshness check.
            // Independent uORB copies can straddle either companion update,
            // not only attitude. A fresh known pre-reset companion or one
            // newer than this odometry sample defers capture, never control
            // on old state. No generation/receipt/reset basis is advanced here.
            if(position->heading_reset_counter==heading_reset_||attitude->quat_reset_counter==heading_reset_||
               attitude->timestamp_sample>message.timestamp_sample||position->timestamp_sample>message.timestamp_sample)return false;
            if(attitude->quat_reset_counter!=position->heading_reset_counter)return fail(Failure::Reset);
            double norm=0;for(float q:attitude->delta_q_reset){if(!std::isfinite(q))return fail(Failure::Reset);norm+=double(q)*double(q);}
            const auto *dq=attitude->delta_q_reset;
            const double yaw=2.0*std::atan2(double(dq[3]),double(dq[0]));
            const double difference=yaw-double(position->delta_heading);
            if(std::abs(std::sqrt(norm)-1.0)>1e-6||std::abs(double(dq[1]))>1e-6||std::abs(double(dq[2]))>1e-6||
               !std::isfinite(position->delta_heading)||
               std::abs(std::atan2(std::sin(difference),std::cos(difference)))>1e-6)return fail(Failure::Reset);
        }
        Snapshot candidate{};
        candidate.source_topic_=topic;candidate.source_instance_=instance;
        std::memcpy(&candidate.raw_,&message,sizeof(message));
        candidate.origin_=config_.task_origin_ned_m;
        candidate.generation_=last_generation;
        candidate.generation_delta_=has_previous_ ? std::uint64_t(last_generation)-previous_generation_ : 0;
        candidate.sample_delta_us_=dt;
        auto &s=candidate.estimator_;
        s.identity=observed_identity;s.generation=last_generation;
        s.timestamp_sample_us=message.timestamp_sample;s.publication_us=message.timestamp;s.board_rx_us=original_hrt_receipt_us;
        s.pose_frame=message.pose_frame;s.velocity_frame=message.velocity_frame;s.reset_counter=message.reset_counter;
        for (unsigned j=0;j<3;++j) {
            s.p[j]=double(message.position[j])-config_.task_origin_ned_m[j];
            s.v[j]=double(message.velocity[j]);s.body_rates[j]=double(message.angular_velocity[j]);
        }
        for(unsigned j=0;j<4;++j)s.q[j]=double(message.q[j]);
        if (!gpenmpc_consumption::finite(s.p)) return fail(Failure::Nonfinite);
        if(reset){heading_reset_prior_=accepted_reset_;accepted_reset_=message.reset_counter;heading_reset_verified_=true;}
        if(companions&&attitude->timestamp_sample<=message.timestamp_sample&&position->timestamp_sample<=message.timestamp_sample&&
           attitude->quat_reset_counter==position->heading_reset_counter&&(!have_reset_basis_||reset)){
            have_reset_basis_=true;heading_reset_=position->heading_reset_counter;
            xy_reset_=position->xy_reset_counter;z_reset_=position->z_reset_counter;
            vxy_reset_=position->vxy_reset_counter;vz_reset_=position->vz_reset_counter;
        }
        candidate.heading_reset_verified_=heading_reset_verified_;candidate.heading_reset_prior_=heading_reset_prior_;
        candidate.valid_=true;out=candidate;
        has_previous_=true;previous_generation_=last_generation;previous_sample_us_=message.timestamp_sample;
        previous_publication_us_=message.timestamp;previous_receipt_us_=original_hrt_receipt_us;
        return true;
    }
    Failure failure() const noexcept {return failure_;}
    const vehicle_odometry_s *last_observed() const noexcept {return observed_?&last_observed_:nullptr;}
    const Configuration &configuration() const noexcept {return config_;}
private:
    bool fail(Failure f) noexcept {if(failure_==Failure::None)failure_=f;return false;}
    Configuration config_{};Failure failure_{Failure::None};
    bool has_previous_{false},observed_{false};
    vehicle_odometry_s last_observed_{};
    std::uint32_t previous_generation_{0};
    std::uint64_t previous_sample_us_{0},previous_publication_us_{0},previous_receipt_us_{0};
    bool have_reset_basis_{false},heading_reset_verified_{false};
    std::uint8_t accepted_reset_{},heading_reset_prior_{},heading_reset_{},xy_reset_{},z_reset_{},vxy_reset_{},vz_reset_{};
};

enum class PollResult : std::uint8_t { NoUpdate, Accepted, Rejected };
// Single-owner subscription only. No separate copy()/updated() or counter
// synthesis. Actual uORB backend is NOT runtime validated by the host mock.
// Clock is the board HRT callable, invoked once immediately after update.
template<class Subscription, class HrtClock>
PollResult poll(Subscription &subscription,HrtClock hrt,Identity observed_identity,
                AtomicOdometryAdapter &adapter,Snapshot &out) noexcept {
    out=Snapshot{};
    if(adapter.failure()!=Failure::None)return PollResult::Rejected;
    vehicle_odometry_s message{};
    if(!subscription.update(&message))return PollResult::NoUpdate;
    const std::uint64_t receipt=hrt();
    const auto generation=subscription.get_last_generation();
    static_assert(sizeof(generation)==sizeof(std::uint32_t),"PX4 unsigned generation must remain raw uint32");
    return adapter.ingest(message,generation,receipt,receipt,observed_identity,
        subscription.get_topic(),subscription.get_instance(),out)?PollResult::Accepted:PollResult::Rejected;
}
} // namespace gpenmpc_odometry
