#include "Px4CanonicalIo.hpp"

namespace gpenmpc_rfly_px4 {
Px4CanonicalIo::Px4CanonicalIo(const gpenmpc_odometry::Configuration &source,
    const gpenmpc_rfly_execution::Configuration &execution,
    uint64_t telemetry_max_age_us,Authority *authority) noexcept:
    expected_identity_(execution.identity),limits_(execution.limits),
    telemetry_max_age_us_(telemetry_max_age_us),authority_(authority),
    executor_(source,execution,{clock,nullptr}),odometry_(ORB_ID(vehicle_odometry),source.instance)
{
    if(!telemetry_max_age_us_ || source.vehicle_odometry_topic!=ORB_ID(vehicle_odometry) ||
       executor_.failure()!=gpenmpc_rfly_state_execution::Failure::None)fail(Fault::Configuration);
    // No advertisement, nonzero output, auto-arm or mode change at construction.
}

Px4CanonicalIo::~Px4CanonicalIo(){stop();}

uint64_t Px4CanonicalIo::expiry(uint64_t origin,uint64_t age) noexcept
{
    return age && origin && age<=UINT64_MAX-origin?origin+age:0;
}

bool Px4CanonicalIo::fresh(uint64_t stamp,uint64_t now)const noexcept
{
    return stamp && now>=stamp && now-stamp<=telemetry_max_age_us_;
}

bool Px4CanonicalIo::fail(Fault reason) noexcept
{
    if(diagnostics_.fault==Fault::None){diagnostics_.fault=reason;diagnostics_.first_fault_us=clock(nullptr);}
    if(authority_)authority_->revoke();
    feedback_.revoke(); // preserve already committed raw, withdraw Fresh only
    pending_=false;
    observation_pending_=false;
    return false;
}

bool Px4CanonicalIo::identity_matches() noexcept
{
    if(!authority_)return fail(Fault::AuthorityUnavailable);
    Identity observed{};
    return authority_->observed_identity(observed) && observed==expected_identity_?true:fail(Fault::IdentityMismatch);
}

Capture Px4CanonicalIo::capture_next(Ticket &ticket) noexcept
{
    ticket={};++diagnostics_.capture_calls;
    if(diagnostics_.fault!=Fault::None)return Capture::Rejected;
    if(pending_){fail(Fault::Pending);return Capture::Rejected;}
    if(!identity_matches())return Capture::Rejected;
    if(!odometry_.update(&raw_))return Capture::NoUpdate;
    const uint64_t original_receipt=clock(nullptr);
    const uint32_t generation=odometry_.get_last_generation();
    ++diagnostics_.source_updates;diagnostics_.last_source_generation=generation;
    if(!executor_.capture(raw_,generation,original_receipt,expected_identity_,
                         odometry_.get_topic(),odometry_.get_instance(),pending_ticket_)){
        fail(Fault::Source);return Capture::Rejected;
    }
    pending_=true;++diagnostics_.captured;ticket=pending_ticket_;return Capture::Accepted;
}

const gpenmpc_odometry::Snapshot *Px4CanonicalIo::snapshot(const Ticket &ticket)const noexcept
{
    return diagnostics_.fault==Fault::None?executor_.snapshot(ticket):nullptr;
}

bool Px4CanonicalIo::refresh_disarmed() noexcept
{
    status_sub_.update(&status_);mode_sub_.update(&mode_);const auto now=clock(nullptr);
    if(!fresh(status_.timestamp,now)||!fresh(mode_.timestamp,now))return fail(Fault::Telemetry);
    if(status_.hil_state!=vehicle_status_s::HIL_STATE_ON||status_.arming_state!=vehicle_status_s::ARMING_STATE_DISARMED||mode_.flag_armed)
        return fail(Fault::ControlOwnership);
    return true;
}

Capture Px4CanonicalIo::capture_disarmed_observation(Ticket &ticket) noexcept
{
    ticket={};if(diagnostics_.fault!=Fault::None)return Capture::Rejected;
    if(diagnostics_.kernel_calls||diagnostics_.publish_attempts||diagnostics_.numerical_commits){fail(Fault::Command);return Capture::Rejected;}
    if(!identity_matches()||!refresh_disarmed())return Capture::Rejected;
    const auto result=capture_next(ticket);
    if(result!=Capture::Accepted)return result;
    observation_pending_=true;
    if(!refresh_disarmed()){ticket={};return Capture::Rejected;}
    return Capture::Accepted;
}

bool Px4CanonicalIo::release_disarmed_observation(const Ticket &ticket) noexcept
{
    if(diagnostics_.fault!=Fault::None)return false;
    if(!pending_||!observation_pending_||ticket!=pending_ticket_)return fail(Fault::Command);
    if(!identity_matches()||!refresh_disarmed())return false;
    if(!executor_.releaseObservation(ticket))return fail(Fault::Command);
    ++diagnostics_.disarmed_observations_released;pending_=false;observation_pending_=false;return true;
}

bool Px4CanonicalIo::refresh_control(uint64_t &now) noexcept
{
    status_sub_.update(&status_);mode_sub_.update(&mode_);offboard_sub_.update(&offboard_);
    now=clock(nullptr); // Original receipt after actual copies, not before them.
    if(!fresh(status_.timestamp,now)||!fresh(mode_.timestamp,now)||!fresh(offboard_.timestamp,now))
        return fail(Fault::Telemetry);
    // Validate Commander state; Authority separately validates exclusive ownership
    // and physical-output isolation.
    if(status_.hil_state!=vehicle_status_s::HIL_STATE_ON ||
       status_.arming_state!=vehicle_status_s::ARMING_STATE_ARMED ||
       status_.nav_state!=vehicle_status_s::NAVIGATION_STATE_OFFBOARD ||
       !mode_.flag_armed || !mode_.flag_control_offboard_enabled ||
       mode_.flag_multicopter_position_control_enabled || mode_.flag_control_manual_enabled ||
       mode_.flag_control_auto_enabled || mode_.flag_control_position_enabled ||
       mode_.flag_control_velocity_enabled || mode_.flag_control_altitude_enabled ||
       mode_.flag_control_climb_rate_enabled || mode_.flag_control_acceleration_enabled ||
       mode_.flag_control_attitude_enabled || mode_.flag_control_rates_enabled ||
       mode_.flag_control_allocation_enabled || mode_.flag_control_termination_enabled ||
       !offboard_.direct_actuator || offboard_.position || offboard_.velocity || offboard_.acceleration ||
       offboard_.attitude || offboard_.body_rate || offboard_.thrust_and_torque)
        return fail(Fault::ControlOwnership);
    return true;
}

bool Px4CanonicalIo::execute(const gpenmpc_rfly_slim::Command &command) noexcept
{
    receipt_={};
    if(diagnostics_.fault!=Fault::None)return false;
    if(observation_pending_)return fail(Fault::Command);
    if(feedback_.pending())return fail(Fault::Feedback); // before another kernel/publication
    if(!pending_ || command.snapshot_ticket!=pending_ticket_)return fail(Fault::Command);
    uint64_t control_receipt=0;
    if(!identity_matches() || !refresh_control(control_receipt))return false;
    if(!executor_.prepareNumericalOnly(command,prepared_)){
        diagnostics_.kernel_calls=executor_.kernel_calls();return fail(Fault::Command);
    }
    diagnostics_.kernel_calls=executor_.kernel_calls();
    const auto &prepared=prepared_.execution;const auto &token=prepared.token;
    uint64_t now=0;
    if(!identity_matches() || !refresh_control(now))return false;
    uint64_t authority_expiry=0;
    if(!authority_->validate(token,status_,mode_,offboard_,now,authority_expiry))return fail(Fault::ControlOwnership);
    const uint64_t sample_expiry=expiry(token.timestamp_sample_us,limits_.sample_max_age_us);
    const uint64_t tick_expiry=expiry(token.control_tick_us,limits_.transaction_max_wall_us);
    uint64_t valid_until=minimum(minimum(sample_expiry,tick_expiry),
        minimum(token.reference_valid_until_us,token.outer_valid_until_us));
    valid_until=minimum(valid_until,authority_expiry);
    // Carry the already-checked telemetry freshness through validate/publish/
    // commit; subsequent work cannot renew those original Commander stamps.
    valid_until=minimum(valid_until,expiry(status_.timestamp,telemetry_max_age_us_));
    valid_until=minimum(valid_until,expiry(mode_.timestamp,telemetry_max_age_us_));
    valid_until=minimum(valid_until,expiry(offboard_.timestamp,telemetry_max_age_us_));
    if(!valid_until || now>valid_until)return fail(Fault::Expired);
    output_message_={};
    // Official normalized-16 layout, no PWM quantization/reallocation. Preserve
    // producer completion time; the send path must not refresh it from its clock.
    output_message_.timestamp=prepared.kernel_completed_us;
    for(unsigned i=0;i<16;++i)output_message_.output[i]=prepared.rfly_controls16[i];
    if(!authority_->install_lease(token,output_message_,valid_until))return fail(Fault::Lease);
    diagnostics_.original_valid_until_us=valid_until;
    if(clock(nullptr)>valid_until)return fail(Fault::Expired);
    ++diagnostics_.publish_attempts;
    const bool published=output_.publish(output_message_); // actual uORB result
    const uint64_t publication_receipt=clock(nullptr);
    if(published){++diagnostics_.publish_succeeded;diagnostics_.last_publication_us=publication_receipt;}
    ack_={};ack_.token=token;ack_.publish_succeeded=published;
    // output_generation identifies the input transaction. The downstream stream
    // reads its uORB generation separately.
    ack_.output_generation=token.output_generation;ack_.publication_us=publication_receipt;
    ack_.published_control=prepared.rfly_controls16;ack_.encoding=prepared.encoding;
    ack_.generated_arm_source_sha256=prepared.generated_c_source_sha256;
    ack_.wrapper_matlab_source_sha256=prepared.wrapper_matlab_source_sha256;
    if(!published)return fail(Fault::Publish);
    if(!executor_.commitNumericalReceipt(pending_ticket_,ack_,receipt_))return fail(Fault::Commit);
    const uint64_t commit_completed=clock(nullptr);
    ++diagnostics_.numerical_commits;
    diagnostics_.last_commit_completed_us=commit_completed;
    // Even a successful orb_publish/commit can finish too late for the stream.
    // Revoke immediately; never count this as a usable renewed output.
    if(commit_completed>valid_until)return fail(Fault::Expired);
    if(!authority_->confirm_publication(token,publication_receipt,commit_completed))return fail(Fault::Lease);
    if(!feedback_.record_success(prepared_,receipt_,commit_completed,valid_until))return fail(Fault::Feedback);
    // Confirmation also consumes real wall time. Keep the already committed
    // raw audit record, but revoke Fresh if this same original deadline passed.
    if(clock(nullptr)>valid_until)return fail(Fault::Expired);
    pending_=false;return true;
}

FeedbackDisposition Px4CanonicalIo::take_committed_feedback(CommittedFeedback &out) noexcept
{
    return feedback_.take(out,clock(nullptr));
}

void Px4CanonicalIo::stop() noexcept
{
    fail(Fault::Stopped);
    // Publication destructor releases its actual uORB handle. Final Commander
    // disarm, native handoff and downstream output retirement remain mandatory.
}
} // namespace gpenmpc_rfly_px4
