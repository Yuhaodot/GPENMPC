#include "Px4CanonicalLocalIo.hpp"
#include <cstring>
#include <px4_platform_common/log.h>
namespace gpenmpc_rfly_px4 {
namespace {
gpenmpc_full_consumption::Hash words(const unsigned char*p)noexcept{
    gpenmpc_full_consumption::Hash out{};
    for(unsigned j=0;j<8;++j)out[j]=(std::uint32_t(p[4*j])<<24)|(std::uint32_t(p[4*j+1])<<16)|
        (std::uint32_t(p[4*j+2])<<8)|std::uint32_t(p[4*j+3]);
    return out;
}
gpenmpc_full_consumption::Hash reference_hash(const gpenmpc_full_inner_reference_candidate&r)noexcept{
    gpenmpc_consumption::CanonicalSha256 h;h.u32(0x52465032); // actual pending RFP2 fields, no padding
    for(unsigned j=0;j<32;++j)h.byte(r.reference_asset_sha256[j]);
    h.u32(r.leg_index);
    h.u64(r.source_timestamp_ns);h.u64(r.source_generation);h.u64(r.reference_generation);h.u64(r.outer_generation);
    h.u64(r.window_generation);h.u64(r.query_sequence);h.u64(r.candidate_generation);
    h.real(r.phase_before_s);h.real(r.phase_after_s);for(double v:r.actual_reference_pvaj)h.real(v);return h.finish();
}
gpenmpc_full_consumption::Hash retry_identity(const LocalCommand&c)noexcept{
    // Only a replacement window generation may change while the SAME source
    // waits for a missing segment. Never renew a query, outer event or expiry.
    gpenmpc_consumption::CanonicalSha256 h;h.u32(0x52465232);
    h.byte(c.operator_reference?1:0);
    const auto&q=c.reference_query;
    for(auto b:q.reference_asset_sha256)h.byte(b);
    h.u32(q.leg_index);
    h.u64(q.query_sequence);h.u64(q.source_timestamp_ns);h.u64(q.source_generation);
    h.u64(q.reference_generation);h.u64(q.outer_generation);
    h.real(q.progress_s);h.real(q.progress_rate);h.real(q.target_phase_acceleration);
    for(double v:q.target_outer_f)h.real(v);
    h.real(q.dt_s);
    const auto&i=c.input_context;gpenmpc_consumption::hash_identity(h,i.observed_session);
    h.words(i.task_sha256);h.words(i.configuration_sha256);h.words(i.reference_asset_sha256);
    h.u64(i.explicit_leg);h.byte(static_cast<std::uint8_t>(i.step_kind));
    gpenmpc_full_consumption::detail::hash_reference(h,c.reference_envelope);
    gpenmpc_full_consumption::detail::hash_outer(h,c.outer_envelope);
    const auto source=[&h](const gpenmpc_local_input::SnapshotKey&s){
        gpenmpc_consumption::hash_identity(h,s.identity);h.u64(s.sample_us);h.u64(s.publication_us);
        h.u64(s.original_receipt_us);h.u64(s.source_generation);h.u64(s.generation_delta);
        h.u64(s.sample_delta_us);h.byte(s.reset_counter);h.words(s.state_and_origin_sha256);
    };
    h.byte(c.rotor!=nullptr);if(c.rotor){source(c.rotor->source);h.byte(static_cast<std::uint8_t>(c.rotor->value_kind));
        const auto&r=c.rotor->original_observation;h.reals(r.observed_thrust_n);h.u64(r.dll_generation);h.u64(r.dll_session);
        h.u64(r.original_host_receive_ns);h.u64(r.original_board_ingress_us);h.real(r.original_sim_time_s);
        h.words(r.original_observation_sha);h.words(c.rotor->verified_association_receipt_sha256);}
    h.byte(c.payload!=nullptr);if(c.payload){source(c.payload->source);h.words(c.payload->task_sha256);
        h.words(c.payload->original_schedule_evidence_sha256);h.u64(c.payload->original_schedule_generation);h.real(c.payload->payload_kg);}
    h.byte(c.wind!=nullptr);if(c.wind){source(c.wind->source);h.words(c.wind->original_estimate_evidence_sha256);
        h.u64(c.wind->original_estimate_generation);h.reals(c.wind->estimate_xy_mps);}
    h.byte(c.initial_interval!=nullptr);if(c.initial_interval){h.words(c.initial_interval->configuration_sha256);
        h.words(c.initial_interval->original_configuration_receipt_sha256);h.u64(c.initial_interval->leg);h.real(c.initial_interval->configured_dt_s);}
    return h.finish();
}
}
Px4CanonicalLocalIo::Px4CanonicalLocalIo(const gpenmpc_odometry::Configuration&s,
    const gpenmpc_full_consumption::Configuration&c,std::uint64_t telemetry_age,Authority*a,
    std::uint64_t commander_age,std::uint64_t output_transport_age)noexcept:
    configuration_(c),telemetry_max_age_us_(telemetry_age),
    commander_telemetry_max_age_us_(commander_age?commander_age:telemetry_age),
    output_transport_max_age_us_(output_transport_age),
    authority_(a),source_(s),consumption_(c),
    odometry_(ORB_ID(vehicle_odometry),s.instance)
{
    gpenmpc_full_inner_build_identity actual{};gpenmpc_full_inner_identity(&actual);
    if(!telemetry_age||commander_telemetry_max_age_us_<telemetry_age||
       (output_transport_age&&(output_transport_age<c.limits.transaction_max_wall_us||output_transport_age>c.limits.sample_max_age_us))||
       s.vehicle_odometry_topic!=ORB_ID(vehicle_odometry)||!(s.identity==c.identity)||
       source_.failure()!=gpenmpc_odometry::Failure::None||consumption_.fault()!=gpenmpc_full_consumption::Fault::None||
       std::memcmp(&actual,&c.build,sizeof actual)||gpenmpc_full_inner_storage_bytes()>full_owner_capacity||
       gpenmpc_full_inner_storage_alignment()>alignof(Px4CanonicalLocalIo)||
       gpenmpc_full_inner_construct(full_storage_,sizeof full_storage_,&c.numerical,&full_owner_)!=RFI_OK)
        fail(LocalFault::Configuration);
    // No publisher advertisement, mode request, arm or output at construction.
}
Px4CanonicalLocalIo::~Px4CanonicalLocalIo(){stop();}
std::uint64_t Px4CanonicalLocalIo::expiry(std::uint64_t t,std::uint64_t age)noexcept{return t&&age&&age<=UINT64_MAX-t?t+age:0;}
bool Px4CanonicalLocalIo::fresh(std::uint64_t t,std::uint64_t now)const noexcept{return t&&now>=t&&now-t<=telemetry_max_age_us_;}
bool Px4CanonicalLocalIo::fresh_commander(std::uint64_t t,std::uint64_t now)const noexcept
{return t&&now>=t&&now-t<=commander_telemetry_max_age_us_;}
bool Px4CanonicalLocalIo::fail(LocalFault f)noexcept{
    const bool first=diagnostics_.first_fault==LocalFault::None;
    if(first){diagnostics_.first_fault=f;diagnostics_.first_fault_us=clock();}
    if(authority_)authority_->revoke();
    // Report the existing first-error state after revoking control, not only
    // after successful context destruction (Closing may await external zero).
    // No extra work or admission condition is introduced on a healthy tick.
    if(first&&f!=LocalFault::Stopped){
        PX4_ERR("LOCAL_IO fault=%u us=%llu source=%u consume=%u",unsigned(f),
            (unsigned long long)diagnostics_.first_fault_us,unsigned(diagnostics_.source_adapter_fault),unsigned(consumption_.fault()));
        PX4_ERR("LOCAL_IO kernel=%llu publish=%llu/%llu commits=%llu installs=%llu",
            (unsigned long long)diagnostics_.kernel_calls,(unsigned long long)diagnostics_.publish_succeeded,
            (unsigned long long)diagnostics_.publish_attempts,(unsigned long long)diagnostics_.consumption_commits,
            (unsigned long long)diagnostics_.joint_installs);
        PX4_ERR("LOCAL_IO sample=%llu previous=%llu start=%llu",
            (unsigned long long)(f==LocalFault::Source?diagnostics_.source_failure_sample_us:snapshot_.estimator().timestamp_sample_us),
            (unsigned long long)diagnostics_.source_failure_previous_sample_us,(unsigned long long)original_execute_start_us_);
        PX4_ERR("LOCAL_IO published=%llu installed=%llu",(unsigned long long)diagnostics_.original_publication_us,
            (unsigned long long)diagnostics_.original_commit_completed_us);
    }
    consumption_.revoke();
    if(full_owner_)gpenmpc_full_inner_retire(full_owner_);
    pending_=false;disarmed_observation_=false;feedback_.fresh_at_read=false;return false;
}
bool Px4CanonicalLocalIo::identity()noexcept{
    Identity actual{};
    return authority_&&authority_->observed_identity(actual)&&actual==configuration_.identity?true:fail(LocalFault::IdentityMismatch);
}
LocalCapture Px4CanonicalLocalIo::capture_next(LocalTicket&t,bool disarmed_observation,bool replace_unexecuted)noexcept{
    if(!replace_unexecuted)t={};
    if(diagnostics_.first_fault!=LocalFault::None)return LocalCapture::Rejected;
    const bool replace=pending_&&replace_unexecuted&&!disarmed_observation&&!disarmed_observation_&&
        !execute_started_;
    if((pending_&&!replace)||feedback_pending_){fail(LocalFault::Pending);return LocalCapture::Rejected;}
    if(!identity())return LocalCapture::Rejected;
    if(!odometry_.update(&raw_))return LocalCapture::NoUpdate;
    const auto received=clock();const auto generation=odometry_.get_last_generation();++diagnostics_.source_updates;
    // Copy the selected source first. A new odometry reset must not be
    // compared to companion storage sampled before that source was read.
    attitude_sub_.update(&attitude_);position_sub_.update(&position_);
    const auto validated=clock();
    if(disarmed_observation&&!refresh_disarmed())return LocalCapture::Rejected;
    // The first control uses ExplicitConfiguredLegInitial. Later controls use
    // commit-to-sample dt and source checks. Autonomous intervals are bounded
    // at 50 ms; manual continuity follows the operator-command lifetime.
    const bool subsequent_control=!disarmed_observation&&diagnostics_.joint_installs>0;
    const auto previous_control=subsequent_control?installed_numerics_.token.lease_envelope.timestamp_sample_us:0;
    // Manual mode retains the last installed sample as the integration anchor,
    // without turning a delayed update into a session-ending interval fault.
    const auto maximum_interval=configuration_.operator_reference?0U:50000U;
    if(!source_.ingest(raw_,generation,received,validated,configuration_.identity,odometry_.get_topic(),odometry_.get_instance(),snapshot_,subsequent_control,maximum_interval,previous_control,&attitude_,&position_)){
        if(source_.failure()==gpenmpc_odometry::Failure::None)return LocalCapture::NoUpdate;
        diagnostics_.source_adapter_fault=source_.failure();
        diagnostics_.source_failure_sample_us=raw_.timestamp_sample;
        diagnostics_.source_failure_previous_sample_us=previous_control?previous_control:
            (retained_snapshot_.valid()?retained_snapshot_.estimator().timestamp_sample_us:0);
        diagnostics_.source_failure_received_us=received;
        fail(LocalFault::Source);return LocalCapture::Rejected;
    }
    if(diagnostics_.captured==UINT64_MAX){fail(LocalFault::Source);return LocalCapture::Rejected;}
    retained_snapshot_=snapshot_; // Retain the validated capture.
    pending_ticket_={++diagnostics_.captured,generation,raw_.timestamp_sample};pending_=true;
    execute_started_=false;original_execute_start_us_=0;diagnostics_.awaiting_window=false;t=pending_ticket_;return LocalCapture::Accepted;
}
const gpenmpc_odometry::Snapshot*Px4CanonicalLocalIo::snapshot(const LocalTicket&t)const noexcept{
    return diagnostics_.first_fault==LocalFault::None&&t==pending_ticket_&&snapshot_.valid()?&snapshot_:nullptr;
}
const gpenmpc_odometry::Snapshot*Px4CanonicalLocalIo::retained_latest_snapshot()const noexcept{
    return retained_snapshot_.valid()?&retained_snapshot_:nullptr;
}
bool Px4CanonicalLocalIo::refresh_disarmed()noexcept{
    status_sub_.update(&status_);mode_sub_.update(&mode_);const auto now=clock();
    if(!fresh_commander(status_.timestamp,now)||!fresh_commander(mode_.timestamp,now))return fail(LocalFault::Telemetry);
    return status_.hil_state==vehicle_status_s::HIL_STATE_ON&&status_.arming_state==vehicle_status_s::ARMING_STATE_DISARMED&&
        !mode_.flag_armed?true:fail(LocalFault::Ownership);
}
LocalCapture Px4CanonicalLocalIo::capture_disarmed(LocalTicket&t)noexcept{
    t={};if(diagnostics_.first_fault!=LocalFault::None)return LocalCapture::Rejected;
    if(diagnostics_.kernel_calls||diagnostics_.publish_attempts){fail(LocalFault::Ownership);return LocalCapture::Rejected;}
    if(!identity()||!refresh_disarmed())return LocalCapture::Rejected;
    const auto result=capture_next(t,true);if(result!=LocalCapture::Accepted)return result;
    disarmed_observation_=true;
    if(!refresh_disarmed()){t={};return LocalCapture::Rejected;}return result;
}
bool Px4CanonicalLocalIo::release_disarmed(const LocalTicket&t)noexcept{
    if(diagnostics_.first_fault!=LocalFault::None)return false;
    if(!pending_||!disarmed_observation_||!(t==pending_ticket_))return fail(LocalFault::Pending);
    if(!identity()||!refresh_disarmed())return false;
    pending_=false;disarmed_observation_=false;++diagnostics_.observations_released;return true;
}
bool Px4CanonicalLocalIo::load_window(const gpenmpc_full_inner_window&w,void*scratch,std::size_t bytes)noexcept{
    if(diagnostics_.first_fault!=LocalFault::None)return false;
    if(pending_){
        // A genuine query window miss did not prepare reference/numeric state.
        // Refill may complete this SAME source only within its ORIGINAL age
        // and control-start bound. It never calls capture or refreshes time.
        if(!diagnostics_.awaiting_window||!execute_started_||feedback_pending_)return fail(LocalFault::Pending);
        const auto deadline=minimum(expiry(snapshot_.estimator().timestamp_sample_us,configuration_.limits.sample_max_age_us),
            expiry(original_execute_start_us_,configuration_.limits.transaction_max_wall_us));
        if(!deadline||clock()>deadline)return fail(LocalFault::Expired);
    }
    if(gpenmpc_full_inner_load_window(full_owner_,&w,scratch,bytes)!=RFI_OK)return fail(LocalFault::Reference);
    if(pending_){const auto deadline=minimum(expiry(snapshot_.estimator().timestamp_sample_us,configuration_.limits.sample_max_age_us),
            expiry(original_execute_start_us_,configuration_.limits.transaction_max_wall_us));
        if(!deadline||clock()>deadline)return fail(LocalFault::Expired);}
    return true;
}
bool Px4CanonicalLocalIo::fill_gp(const std::uint64_t tags[2],const double r[18])noexcept{
    if(diagnostics_.first_fault!=LocalFault::None)return false;
    if(pending_)return fail(LocalFault::Pending);
    if(gpenmpc_full_inner_fill_gp(full_owner_,tags,r)!=RFI_OK)return fail(LocalFault::Numeric);
    ++diagnostics_.gp_fills;return true; // never invokes a control tick
}
bool Px4CanonicalLocalIo::refresh_control(std::uint64_t&now)noexcept{
    status_sub_.update(&status_);mode_sub_.update(&mode_);offboard_sub_.update(&offboard_);now=clock();
    if(!fresh_commander(status_.timestamp,now)||!fresh_commander(mode_.timestamp,now)||
       !fresh(offboard_.timestamp,now))return fail(LocalFault::Telemetry);
    if(status_.hil_state!=vehicle_status_s::HIL_STATE_ON||status_.arming_state!=vehicle_status_s::ARMING_STATE_ARMED||
       status_.nav_state!=vehicle_status_s::NAVIGATION_STATE_OFFBOARD||!mode_.flag_armed||!mode_.flag_control_offboard_enabled||
       mode_.flag_multicopter_position_control_enabled||mode_.flag_control_manual_enabled||mode_.flag_control_auto_enabled||
       mode_.flag_control_position_enabled||mode_.flag_control_velocity_enabled||mode_.flag_control_altitude_enabled||
       mode_.flag_control_climb_rate_enabled||mode_.flag_control_acceleration_enabled||mode_.flag_control_attitude_enabled||
       mode_.flag_control_rates_enabled||mode_.flag_control_allocation_enabled||mode_.flag_control_termination_enabled||
       !offboard_.direct_actuator||offboard_.position||offboard_.velocity||offboard_.acceleration||offboard_.attitude||
       offboard_.body_rate||offboard_.thrust_and_torque)return fail(LocalFault::Ownership);
    return true;
}
bool Px4CanonicalLocalIo::execute(const LocalCommand&c)noexcept{
    if(diagnostics_.first_fault!=LocalFault::None)return false;
    if(c.operator_reference!=configuration_.operator_reference||
       c.input_context.operator_reference!=configuration_.operator_reference)return fail(LocalFault::Configuration);
    if(!pending_||disarmed_observation_||feedback_pending_||!(c.ticket==pending_ticket_))return fail(LocalFault::Pending);
    std::uint64_t now_before=0;if(!identity()||!refresh_control(now_before))return false;
    if(execute_started_&&!diagnostics_.awaiting_window)return fail(LocalFault::Pending);
    if(!execute_started_){original_execute_start_us_=now_before;original_retry_identity_=retry_identity(c);execute_started_=true;}
    else if(!(original_retry_identity_==retry_identity(c)))return fail(LocalFault::Reference);
    const auto start=original_execute_start_us_;
    const auto original_deadline=minimum(expiry(snapshot_.estimator().timestamp_sample_us,configuration_.limits.sample_max_age_us),
        expiry(start,configuration_.limits.transaction_max_wall_us));
    if(!original_deadline||now_before>original_deadline)return fail(LocalFault::Expired);
    if(gpenmpc_full_inner_copy_state(full_owner_,&prior_state_)!=RFI_OK)return fail(LocalFault::Numeric);
    evidence_={};evidence_.original_prior_committed_state_sha256=words(prior_state_.committed_state_sha256);
    evidence_.original_loaded_window_sha256=words(prior_state_.loaded_window_sha256);
    evidence_.original_reference_query=c.reference_query;
    ++diagnostics_.reference_prepares;
    const auto reference_result=c.operator_reference?
        gpenmpc_full_inner_prepare_operator_reference(full_owner_,&c.reference_query,&reference_candidate_):
        gpenmpc_full_inner_prepare_reference(full_owner_,&c.reference_query,&reference_candidate_);
    if(reference_result==RFI_REFERENCE_WINDOW_MISS){
        ++diagnostics_.reference_window_misses;diagnostics_.awaiting_window=true;return false;
    }
    diagnostics_.awaiting_window=false;
    if(reference_result!=RFI_OK)return fail(LocalFault::Reference);
    const auto&r=reference_candidate_;
    bound_reference_={};if(!gpenmpc_local_input::snapshot_key(snapshot_,bound_reference_.source))return fail(LocalFault::Source);
    bound_reference_.task_sha256=words(configuration_.numerical.task_sha256);
    bound_reference_.configuration_sha256=words(configuration_.numerical.configuration_sha256);
    bound_reference_.asset_sha256=words(r.reference_asset_sha256);bound_reference_.candidate_evidence_sha256=reference_hash(r);
    auto&v=bound_reference_.candidate;v.leg=r.leg_index;v.window_generation=r.window_generation;
    v.original_source_generation=r.source_generation;v.candidate_token=r.candidate_generation;
    v.phase_before_s=r.phase_before_s;v.phase_after_s=r.phase_after_s;
    for(unsigned j=0;j<3;++j){v.position_m[j]=r.actual_reference_pvaj[j];v.velocity_mps[j]=r.actual_reference_pvaj[j+3];
        v.acceleration_mps2[j]=r.actual_reference_pvaj[j+6];v.jerk_mps3[j]=r.actual_reference_pvaj[j+9];}
    if(!gpenmpc_local_input::build(snapshot_,c.input_context,c.rotor,&bound_reference_,c.payload,c.wind,c.initial_interval,assembled_))return fail(LocalFault::Input);
    std::uint64_t tags[2]={assembled_.tags2[0],assembled_.tags2[1]};
    ++diagnostics_.kernel_prepare_attempts;
    const auto numeric_result=gpenmpc_full_inner_prepare_numeric(full_owner_,assembled_.input36,tags,&r,&candidate_);
    gpenmpc_full_inner_diagnostics actual_numeric{};
    if(gpenmpc_full_inner_diagnose(full_owner_,&actual_numeric)!=RFI_OK)return fail(LocalFault::Numeric);
    diagnostics_.kernel_calls=actual_numeric.kernel_calls;
    if(numeric_result!=RFI_OK)return fail(LocalFault::Numeric);
    const auto completed=clock();
    // Metadata is original accepted input timing; only coordinates are derived
    // from the ACTUAL local reference, with the original canonical z flip.
    reference_envelope_=c.reference_envelope;
    for(unsigned j=0;j<3;++j){const double s=j==2?-1.0:1.0;reference_envelope_.p[j]=s*r.actual_reference_pvaj[j];
        reference_envelope_.v[j]=s*r.actual_reference_pvaj[j+3];reference_envelope_.a[j]=s*r.actual_reference_pvaj[j+6];}
    if(!consumption_.begin(snapshot_,assembled_,candidate_,evidence_,reference_envelope_,c.outer_envelope,start,completed,token_))return fail(LocalFault::Consumption);
    feedback_={};feedback_.token=token_;
    std::memcpy(feedback_.request19,candidate_.request19,sizeof candidate_.request19);
    std::memcpy(feedback_.prepared_control16,candidate_.control16,sizeof candidate_.control16);
    feedback_pending_=true; // raw evidence retained even if authority/publish later rejects
    std::uint64_t now=0,authority_expiry=0;
    if(!identity()||!refresh_control(now))return false;
    const auto&t=token_.lease_envelope;
    if(!authority_->validate(t,status_,mode_,offboard_,now,authority_expiry))return fail(LocalFault::Ownership);
    auto valid_until=minimum(token_.original_valid_until_us,authority_expiry);
    valid_until=minimum(valid_until,expiry(status_.timestamp,commander_telemetry_max_age_us_));
    valid_until=minimum(valid_until,expiry(mode_.timestamp,commander_telemetry_max_age_us_));
    valid_until=minimum(valid_until,expiry(offboard_.timestamp,telemetry_max_age_us_));
    if(!valid_until||now>valid_until)return fail(LocalFault::Expired);
    // Completed output crosses an asynchronous 200-Hz MAVLink stream.
    // Source, outer and reference ages are checked separately from compute time.
    auto transport_until=valid_until;
    if(output_transport_max_age_us_){
        transport_until=minimum(expiry(start,output_transport_max_age_us_),authority_expiry);
        transport_until=minimum(transport_until,expiry(snapshot_.estimator().timestamp_sample_us,configuration_.limits.sample_max_age_us));
        transport_until=minimum(transport_until,minimum(t.reference_valid_until_us,t.outer_valid_until_us));
        transport_until=minimum(transport_until,expiry(t.reference_board_rx_us,configuration_.limits.reference_max_age_us));
        transport_until=minimum(transport_until,expiry(t.outer_board_rx_us,configuration_.limits.outer_max_age_us));
        transport_until=minimum(transport_until,expiry(status_.timestamp,commander_telemetry_max_age_us_));
        transport_until=minimum(transport_until,expiry(mode_.timestamp,commander_telemetry_max_age_us_));
        transport_until=minimum(transport_until,expiry(offboard_.timestamp,telemetry_max_age_us_));
        if(!transport_until||now>transport_until)return fail(LocalFault::Expired);
    }
    output_message_={};output_message_.timestamp=completed;
    std::memcpy(output_message_.output,candidate_.control16,sizeof candidate_.control16);
    if(!authority_->install_lease(t,output_message_,transport_until))return fail(LocalFault::Lease);
    diagnostics_.original_valid_until_us=transport_until;
    feedback_.original_valid_until_us=valid_until;
    if(clock()>valid_until)return fail(LocalFault::Expired);
    ++diagnostics_.publish_attempts;feedback_.publication_attempted=true;
    const bool published=output_.publish(output_message_);const auto published_at=clock();
    feedback_.publication_succeeded=published;feedback_.original_publication_us=published_at;
    if(published){++diagnostics_.publish_succeeded;diagnostics_.original_publication_us=published_at;
        std::memcpy(feedback_.actual_control16,output_message_.output,sizeof feedback_.actual_control16);}
    if(!published){consumption_.note_publication_failure(token_,published_at);install_actual_publication(published_at,false,false);return fail(LocalFault::Publish);}
    if(!consumption_.commit(token_,output_message_.output,published_at,clock(),consumption_receipt_)){
        install_actual_publication(published_at,true,false);return fail(LocalFault::Consumption);
    }
    ++diagnostics_.consumption_commits;
    feedback_.consumption_committed=true;
    if(!install_actual_publication(published_at,true,true))return fail(LocalFault::Install);
    const auto installed_at=clock();diagnostics_.original_commit_completed_us=installed_at;
    feedback_.numerical_reference_installed=true;feedback_.original_commit_completed_us=installed_at;
    // Reuse persistent scratch: never put the full state on the tick stack.
    // Publication/install remain recorded even if this subsequent copy fails.
    if(gpenmpc_full_inner_copy_state(full_owner_,&prior_state_)!=RFI_OK||
       !prior_state_.numeric_installed||!prior_state_.reference_committed)return fail(LocalFault::Install);
    feedback_.committed_reference=prior_state_.reference;feedback_.reference_state_copied=true;
    // Retain joint-install completion time when copying state after the deadline.
    if(installed_at>valid_until||clock()>valid_until)return fail(LocalFault::Expired);
    if(!authority_->confirm_publication(t,published_at,installed_at))return fail(LocalFault::Lease);
    feedback_.authority_confirmed=true;
    // Direct SE(3) control updates vehicle_thrust_setpoint for the land detector.
    // The mean committed rotor command equals total thrust divided by maximum
    // six-rotor thrust. After LAND handoff, native control owns this topic.
    vehicle_thrust_setpoint_s thrust{};
    thrust.timestamp=clock();
    thrust.timestamp_sample=snapshot_.estimator().timestamp_sample_us;
    float normalized_total=0.0F;
    for(unsigned rotor=0;rotor<6;++rotor)normalized_total+=output_message_.output[rotor];
    thrust.xyz[2]=-normalized_total/6.0F; // body FRD upward thrust, [-1,0]
    if(!thrust_status_.publish(thrust))return fail(LocalFault::Publish);
    if(clock()>valid_until)return fail(LocalFault::Expired);
    pending_=false;return true;
}
bool Px4CanonicalLocalIo::install_actual_publication(std::uint64_t published_at,bool actual_published,bool consumption_ok)noexcept{
    backend_={};numeric_receipt_={};reference_receipt_={};const auto&t=token_.lease_envelope;
    backend_.source_timestamp_ns=candidate_.original_tags2[0];backend_.source_generation=candidate_.original_tags2[1];
    backend_.output_generation=t.output_generation;backend_.original_publication_us=published_at;
    std::memcpy(backend_.actual_control16,output_message_.output,sizeof backend_.actual_control16);
    // Actual publication success is counted independently of later validation.
    backend_.output_published=actual_published;
    numeric_receipt_.source_timestamp_ns=backend_.source_timestamp_ns;numeric_receipt_.source_generation=backend_.source_generation;
    numeric_receipt_.output_generation=backend_.output_generation;numeric_receipt_.original_publication_us=published_at;
    std::memcpy(numeric_receipt_.actual_control16,backend_.actual_control16,sizeof backend_.actual_control16);
    numeric_receipt_.publication_succeeded=backend_.output_published;numeric_receipt_.numerical_commit_succeeded=consumption_ok;
    reference_receipt_.source_timestamp_ns=backend_.source_timestamp_ns;reference_receipt_.source_generation=backend_.source_generation;
    reference_receipt_.output_generation=backend_.output_generation;reference_receipt_.original_publication_us=published_at;
    reference_receipt_.candidate_generation=candidate_.reference_candidate_generation;
    reference_receipt_.query_sequence=candidate_.reference_query_sequence;reference_receipt_.window_generation=candidate_.reference_window_generation;
    reference_receipt_.reference_generation=candidate_.reference_generation;reference_receipt_.outer_generation=candidate_.outer_generation;
    std::memcpy(reference_receipt_.actual_reference_pvaj,candidate_.consumed_reference_pvaj,sizeof reference_receipt_.actual_reference_pvaj);
    reference_receipt_.publication_succeeded=backend_.output_published;reference_receipt_.numerical_commit_succeeded=consumption_ok;
    const bool installed=gpenmpc_full_inner_commit(full_owner_,&backend_,&numeric_receipt_,&reference_receipt_)==RFI_OK;
    if(installed){
        ++diagnostics_.joint_installs;
        // Retain exactly the candidate whose actual publication was installed.
        // A later uncommitted/failed candidate cannot overwrite this slot.
        installed_numerics_.token=token_;
        std::memcpy(installed_numerics_.actual_input36,assembled_.input36,sizeof assembled_.input36);
        std::memcpy(installed_numerics_.actual_kernel61,candidate_.kernel61,sizeof candidate_.kernel61);
        std::memcpy(installed_numerics_.actual_request19,candidate_.request19,sizeof candidate_.request19);
        std::memcpy(installed_numerics_.actual_published_control16,output_message_.output,sizeof installed_numerics_.actual_published_control16);
        installed_numerics_.joint_install_count=diagnostics_.joint_installs;
        installed_numerics_.original_publication_us=published_at;installed_numerics_.present=true;
    }
    return installed;
}
bool Px4CanonicalLocalIo::take_execution_feedback(LocalExecutionFeedback&out)noexcept{
    out={};if(!feedback_pending_)return false;out=feedback_;feedback_pending_=false;
    out.fresh_at_read=diagnostics_.first_fault==LocalFault::None&&out.publication_succeeded&&out.consumption_committed&&
        out.numerical_reference_installed&&out.reference_state_copied&&out.authority_confirmed&&clock()<=out.original_valid_until_us;return true;
}
bool Px4CanonicalLocalIo::numerical_diagnostics(gpenmpc_full_inner_diagnostics&out)const noexcept{
    return full_owner_&&gpenmpc_full_inner_diagnose(full_owner_,&out)==RFI_OK;
}
bool Px4CanonicalLocalIo::copy_local_numerical_state(LocalNumericalObservation&out)const noexcept{
    out={};
    if(!full_owner_||gpenmpc_full_inner_copy_state(full_owner_,&out.state)!=RFI_OK||
       gpenmpc_full_inner_diagnose(full_owner_,&out.actual_diagnostics)!=RFI_OK)return false;
    out.configuration=configuration_.numerical;out.build=configuration_.build;out.last_installed=installed_numerics_;
    out.actual_joint_installs=diagnostics_.joint_installs;out.actual_gp_fills=diagnostics_.gp_fills;
    out.io_first_fault=static_cast<std::uint32_t>(diagnostics_.first_fault);
    out.capture_pending=pending_;out.execution_started=execute_started_;out.unread_execution_feedback=feedback_pending_;out.copied=true;
    const auto&t=out.last_installed.token.lease_envelope;
    out.installed_evidence_matches_state=out.last_installed.present&&out.state.numeric_installed&&out.state.reference_committed&&
        t.timestamp_sample_us<=UINT64_MAX/1000&&t.timestamp_sample_us*1000==out.state.original_installed_tags2[0]&&
        t.sample_generation==out.state.original_installed_tags2[1]&&t.output_generation==out.state.reference.output_generation&&
        out.last_installed.original_publication_us==out.state.reference.publication_us&&
        out.last_installed.joint_install_count==out.actual_diagnostics.joint_installs;
    const bool unusable=diagnostics_.first_fault!=LocalFault::None||pending_||
        out.actual_diagnostics.abi_failure||out.actual_diagnostics.numeric_failure||out.actual_diagnostics.reference_failure||
        out.actual_diagnostics.joint_failure||out.actual_diagnostics.partial_installs;
    if(unusable)out.kind=LocalStateKind::HistoricalUnusable;
    else if(!out.state.numeric_installed&&!out.state.reference_committed&&!out.actual_diagnostics.numeric_installs)
        out.kind=LocalStateKind::InitialUncommitted;
    else if(!out.installed_evidence_matches_state)out.kind=LocalStateKind::HistoricalUnusable;
    else if(!out.state.prediction_required)out.kind=LocalStateKind::CommittedNoGp;
    else out.kind=out.state.prediction_ready?LocalStateKind::CommittedGpReady:LocalStateKind::CommittedAwaitingGp;
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
    if(gpenmpc_full_inner_copy_closed_evidence(full_owner_,&out.closed_gp)!=RFI_OK)return false;
    const bool closed_bound=out.closed_gp.exported&&out.closed_gp.installed&&out.installed_evidence_matches_state&&
        std::memcmp(out.closed_gp.original_installed_tags2,out.state.original_installed_tags2,sizeof out.state.original_installed_tags2)==0&&
        out.closed_gp.original_publication_us==out.state.reference.publication_us;
    const double available=out.closed_gp.values5[0]; // Discrete binary64 availability flag.
    out.latest_closed_gp_evidence_available=closed_bound&&out.kind!=LocalStateKind::HistoricalUnusable&&available<=1.0&&available>=1.0;
#endif
    return true; // copied historical data, never a permission/availability ACK
}
void Px4CanonicalLocalIo::stop()noexcept{fail(LocalFault::Stopped);}
}
