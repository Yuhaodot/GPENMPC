#pragma once
// Assembles the numerical input from separately validated observations.
#include "CanonicalSnapshotStateMapping.hpp"
#include "BoardLocalInnerSchedule.hpp"
#include "CanonicalOperatorReference.hpp"
namespace gpenmpc_local_input {
using Hash=gpenmpc_local_schedule::Hash;
using Identity=gpenmpc_consumption::Identity;
// Held-observation age margin. Estimator age, numerical dt and GP deadlines
// are checked separately; retained source timestamps are preserved.
constexpr std::uint64_t maximum_held_input_age_us=100000;
enum class Failure:std::uint8_t {None,Snapshot,Configuration,IdentityMismatch,SourceBinding,MissingRotor,Rotor,MissingReference,Reference,
    MissingPayload,Payload,MissingWind,Wind,MissingInitialInterval,Interval,TimestampOverflow,Mapping};
enum class IntervalBasis:std::uint8_t {None,MeasuredSnapshotDelta,ExplicitConfiguredLegInitial};
enum class StepKind:std::uint8_t {Unspecified,FirstOfExplicitLeg,SubsequentObservedSource};
enum class RotorValueKind:std::uint8_t {Unspecified,OriginalPlantLagState,ActuatorCommand};
struct SnapshotKey {
    Identity identity{};
    std::uint64_t sample_us{},publication_us{},original_receipt_us{},source_generation{},generation_delta{},sample_delta_us{};
    std::uint8_t reset_counter{};
    Hash state_and_origin_sha256{};
};
inline bool snapshot_key(const gpenmpc_odometry::Snapshot&s,SnapshotKey&out)noexcept{
    out={};if(!s.valid())return false;const auto&e=s.estimator();
    out.identity=e.identity;out.sample_us=e.timestamp_sample_us;out.publication_us=e.publication_us;out.original_receipt_us=e.board_rx_us;
    out.source_generation=e.generation;out.generation_delta=s.generation_delta();out.sample_delta_us=s.actual_sample_delta_us();out.reset_counter=e.reset_counter;
    gpenmpc_consumption::CanonicalSha256 h;h.u32(0x49425331); // IBS1 source semantic key, not an authorization token
    h.reals(s.task_origin_ned_m());h.reals(e.p);h.reals(e.v);h.reals(e.q);h.reals(e.body_rates);h.byte(e.pose_frame);h.byte(e.velocity_frame);
    out.state_and_origin_sha256=h.finish();return true;
}
struct Context {
    Identity observed_session{};
    Hash task_sha256{},configuration_sha256{},reference_asset_sha256{};
    std::uint64_t explicit_leg{};
    StepKind step_kind{StepKind::Unspecified};
    bool allow_valid_held_inputs{};
    bool operator_reference{};
};
struct BoundRotorLag {
    SnapshotKey source{};
    RotorValueKind value_kind{RotorValueKind::Unspecified};
    gpenmpc_local_schedule::RotorLag original_observation{};
    Hash verified_association_receipt_sha256{}; // independent original sim-clock/board-source association, supplied by its owner
};
struct BoundReference {
    SnapshotKey source{};
    gpenmpc_local_schedule::ReferenceCandidate candidate{};
    Hash task_sha256{},configuration_sha256{},asset_sha256{},candidate_evidence_sha256{};
};
struct BoundPayload {
    SnapshotKey source{};Hash task_sha256{},original_schedule_evidence_sha256{};
    std::uint64_t original_schedule_generation{};
    double payload_kg{};
};
struct BoundWind {
    SnapshotKey source{};Hash original_estimate_evidence_sha256{};
    std::uint64_t original_estimate_generation{};
    gpenmpc_portable::Array<double,2> estimate_xy_mps{};
};
struct ExplicitInitialInterval {
    Hash configuration_sha256{},original_configuration_receipt_sha256{};
    std::uint64_t leg{};double configured_dt_s{};
};
struct Result {
    double input36[36]{};unsigned long long tags2[2]{};
    bool assembled{};
    Failure failure{Failure::None};IntervalBasis interval_basis{IntervalBasis::None};
    SnapshotKey source{};
    std::uint64_t explicit_leg{},rotor_generation{},rotor_session{},reference_candidate_token{},reference_window_generation{},
        payload_generation{},wind_generation{};
    Hash rotor_association_receipt_sha256{},initial_interval_configuration_receipt_sha256{};
    // Numerical assembly leaves source, clock and output validation to their owners.
    bool external_clock_association_proven_here{},board_authority{};
};
inline bool empty(const Hash&h)noexcept{for(auto w:h)if(w)return false;return true;}
inline bool same_source(const SnapshotKey&a,const SnapshotKey&b)noexcept{
    return a.identity==b.identity&&a.sample_us==b.sample_us&&a.publication_us==b.publication_us&&
        a.original_receipt_us==b.original_receipt_us&&a.source_generation==b.source_generation&&a.reset_counter==b.reset_counter&&
        a.generation_delta==b.generation_delta&&a.sample_delta_us==b.sample_delta_us&&a.state_and_origin_sha256==b.state_and_origin_sha256;
}
inline bool reject(Result&out,Failure f)noexcept{out.failure=f;return false;}
inline bool build(const gpenmpc_odometry::Snapshot&snapshot,const Context&context,const BoundRotorLag*rotor,
    const BoundReference*reference,const BoundPayload*payload,const BoundWind*wind,const ExplicitInitialInterval*initial,Result&out)noexcept{
    out={};for(double&v:out.input36)v=gpenmpc_portable::Ieee754<double>::quiet_NaN();
    if(!snapshot_key(snapshot,out.source))return reject(out,Failure::Snapshot);
    if(!context.explicit_leg||context.explicit_leg>(1ULL<<53)||empty(context.task_sha256)||empty(context.configuration_sha256)||
       empty(context.reference_asset_sha256)||context.step_kind==StepKind::Unspecified)return reject(out,Failure::Configuration);
    if(!(out.source.identity==context.observed_session))return reject(out,Failure::IdentityMismatch);
    if(!rotor)return reject(out,Failure::MissingRotor);
    if(!reference)return reject(out,Failure::MissingReference);
    if(!payload)return reject(out,Failure::MissingPayload);
    if(!wind)return reject(out,Failure::MissingWind);
    const auto usable=[&](const SnapshotKey&old){
        if(same_source(out.source,old))return true;
        return context.allow_valid_held_inputs&&old.identity==out.source.identity&&
            (old.reset_counter==out.source.reset_counter||snapshot.heading_reset_from(old.reset_counter))&&
            old.source_generation<=out.source.source_generation&&old.sample_us&&old.sample_us<=out.source.sample_us&&
            out.source.sample_us-old.sample_us<=maximum_held_input_age_us;
    };
    if(!usable(rotor->source)||!same_source(out.source,reference->source)||!usable(payload->source)||!usable(wind->source))
        return reject(out,Failure::SourceBinding);
    const auto&r=rotor->original_observation;
    if(rotor->value_kind!=RotorValueKind::OriginalPlantLagState||!r.dll_generation||!r.dll_session||!r.original_host_receive_ns||!r.original_board_ingress_us||
       !std::isfinite(r.original_sim_time_s)||r.original_sim_time_s<0||empty(r.original_observation_sha)||empty(rotor->verified_association_receipt_sha256)||
       !gpenmpc_consumption::finite(r.observed_thrust_n))return reject(out,Failure::Rotor);
    for(double v:r.observed_thrust_n)if(v<0)return reject(out,Failure::Rotor);
    const auto&ref=reference->candidate;
    if(reference->task_sha256!=context.task_sha256||reference->configuration_sha256!=context.configuration_sha256||reference->asset_sha256!=context.reference_asset_sha256||
       empty(reference->candidate_evidence_sha256)||ref.leg!=context.explicit_leg||!ref.window_generation||!ref.candidate_token||ref.original_source_generation!=out.source.source_generation||
       !std::isfinite(ref.phase_before_s)||!std::isfinite(ref.phase_after_s)||ref.phase_after_s<ref.phase_before_s||
       !gpenmpc_consumption::finite(ref.position_m)||!gpenmpc_consumption::finite(ref.velocity_mps)||!gpenmpc_consumption::finite(ref.acceleration_mps2)||!gpenmpc_consumption::finite(ref.jerk_mps3))
        return reject(out,Failure::Reference);
    if(payload->task_sha256!=context.task_sha256||!payload->original_schedule_generation||empty(payload->original_schedule_evidence_sha256)||
       !std::isfinite(payload->payload_kg)||payload->payload_kg<0)return reject(out,Failure::Payload);
    if(!wind->original_estimate_generation||empty(wind->original_estimate_evidence_sha256)||!gpenmpc_consumption::finite(wind->estimate_xy_mps))return reject(out,Failure::Wind);
    double dt=0;
    if(context.step_kind==StepKind::FirstOfExplicitLeg){
        if(!initial)return reject(out,Failure::MissingInitialInterval);
        if(initial->leg!=context.explicit_leg||initial->configuration_sha256!=context.configuration_sha256||empty(initial->original_configuration_receipt_sha256))
            return reject(out,Failure::Interval);
        dt=initial->configured_dt_s;out.interval_basis=IntervalBasis::ExplicitConfiguredLegInitial;
        out.initial_interval_configuration_receipt_sha256=initial->original_configuration_receipt_sha256;
    }else if(context.step_kind==StepKind::SubsequentObservedSource){
        if(initial||!out.source.sample_delta_us)return reject(out,Failure::Interval);
        dt=static_cast<double>(out.source.sample_delta_us)*1e-6;out.interval_basis=IntervalBasis::MeasuredSnapshotDelta;
    }else return reject(out,Failure::Configuration);
    // Existing canonical adapter/numeric domain, NOT a measured hardware
    // limit or justification of a sampling-frequency/transport guarantee.
    if(!gpenmpc_operator_reference::valid_control_interval_s(dt,context.operator_reference))return reject(out,Failure::Interval);
    if(out.source.sample_us>UINT64_MAX/1000)return reject(out,Failure::TimestampOverflow);
    gpenmpc_portable::Array<double,13>x13{};
    if(!gpenmpc_snapshot_mapping::state13(snapshot,x13))return reject(out,Failure::Mapping);
    for(unsigned j=0;j<13;++j)out.input36[j]=x13[j];
    for(unsigned j=0;j<6;++j)out.input36[13+j]=r.observed_thrust_n[j];
    for(unsigned j=0;j<3;++j){out.input36[19+j]=ref.position_m[j];out.input36[22+j]=ref.velocity_mps[j];
        out.input36[25+j]=ref.acceleration_mps2[j];out.input36[28+j]=ref.jerk_mps3[j];}
    out.input36[31]=payload->payload_kg;out.input36[32]=wind->estimate_xy_mps[0];out.input36[33]=wind->estimate_xy_mps[1];
    out.input36[34]=dt;out.input36[35]=static_cast<double>(context.explicit_leg);
    out.tags2[0]=out.source.sample_us*1000;out.tags2[1]=out.source.source_generation;
    out.explicit_leg=context.explicit_leg;out.rotor_generation=r.dll_generation;out.rotor_session=r.dll_session;
    out.reference_candidate_token=ref.candidate_token;out.reference_window_generation=ref.window_generation;
    out.payload_generation=payload->original_schedule_generation;out.wind_generation=wind->original_estimate_generation;
    out.rotor_association_receipt_sha256=rotor->verified_association_receipt_sha256;
    out.assembled=true;return true;
}
} // namespace gpenmpc_local_input
