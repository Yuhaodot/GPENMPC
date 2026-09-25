#pragma once
// Numerical reference state.
#include <cmath>
#include <cstdint>
#include <cstring>
#include "gpenmpcNative_queryCanonicalReferenceWindow.h"
#include "gpenmpcNative_canonicalReferenceTransitionFromJet.h"
#include "CanonicalOperatorReference.hpp"

namespace gpenmpc_reference_math {
enum class Failure:std::uint8_t {None,Configuration,Window,Identity,Ordering,Input,PendingCandidate,Query,Numerics,Publication};
enum class PrepareResult:std::uint8_t {Candidate,WindowMiss,Rejected};
struct Configuration {
    unsigned char reference_asset_sha256[32]{};
    std::uint32_t leg_index{};
    double initial_phase_acceleration{},initial_outer_i[3]{},jerk_limit_mps3{};
};
struct Input {
    struct53_T query{};
    std::uint64_t source_timestamp_ns{},source_generation{},reference_generation{},outer_generation{};
    double progress_rate{},target_phase_acceleration{},target_outer_f[3]{},dt_s{};
};
struct InstalledState {
    struct52_T query{};
    double query_progress_s{},progress_rate{},phase_acceleration{},outer_i[3]{};
    std::uint64_t source_timestamp_ns{},source_generation{},reference_generation{},outer_generation{},output_generation{},publication_us{};
};
struct Candidate {
    struct52_T next_query{};
    struct54_T query_receipt{};
    struct55_T transition{};
    double jet[12]{};
    Input input{};
    std::uint64_t candidate_generation{};
};
struct PublicationReceipt {
    std::uint64_t source_timestamp_ns{},source_generation{},candidate_generation{},query_sequence{},window_generation{},
        reference_generation{},outer_generation{},output_generation{},original_publication_us{};
    double actual_reference_pvaj[12]{};
    bool publication_succeeded{},numerical_commit_succeeded{};
};
class CanonicalReferenceStateStore final {
public:
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
    using Workspace=e_gpenmpcNative_canonicalLocalIn;
#else
    using Workspace=f_gpenmpcNative_canonicalLocalIn;
#endif
    CanonicalReferenceStateStore(Workspace &workspace,const Configuration &configuration)noexcept
        :workspace_(&workspace),configuration_(configuration){
        bool nonzero=false;for(unsigned char b:configuration.reference_asset_sha256)nonzero|=b!=0;
        if(!nonzero||configuration.leg_index<1||configuration.leg_index>5||
           !std::isfinite(configuration.initial_phase_acceleration)||!finite(configuration.initial_outer_i,3)||
           !std::isfinite(configuration.jerk_limit_mps3)||configuration.jerk_limit_mps3<=0){fail(Failure::Configuration);return;}
        std::memcpy(installed_.query.reference_asset_sha256,configuration.reference_asset_sha256,32);
        installed_.query.leg_index=configuration.leg_index;
        installed_.phase_acceleration=configuration.initial_phase_acceleration;
        std::memcpy(installed_.outer_i,configuration.initial_outer_i,sizeof installed_.outer_i);
    }
    CanonicalReferenceStateStore(const CanonicalReferenceStateStore&)=delete;
    CanonicalReferenceStateStore&operator=(const CanonicalReferenceStateStore&)=delete;
    // Incoming bytes are already bound to the task asset by the owner; an
    // asset label/source id is not authentication. Validate before copying.
    // Refills preserve all installed scientific state and query generation.
    bool load_window(const struct51_T &window)noexcept{
        if(failure_!=Failure::None)return false;
        if(prepared_)return fail(Failure::PendingCandidate);
        ++window_attempts_;
        if(std::memcmp(window.reference_asset_sha256,configuration_.reference_asset_sha256,32)||
           window.leg_index!=configuration_.leg_index)return fail(Failure::Identity);
        if(window_ready_&&(window.window_generation<=window_.window_generation||!same_binding(window_,window)))
            return fail(Failure::Window);
        if(!window.window_generation||window.schema!=1||window.capacity!=256||window.row_count<2||window.row_count>256||
           !window.source_first_row||static_cast<std::uint64_t>(window.source_first_row)+window.row_count-1>window.source_total_rows||
           !std::isfinite(window.time_s[0])||installed_.query.last_accepted_sequence==UINT64_MAX)return fail(Failure::Window);
        // Use the actual generated validator, without committing its scratch
        // next state. Probe strictly inside the first original interval:
        // (first+25)-25 need not equal first in binary64. This is ONLY a
        // structural validation query; the actual requested q is never
        // moved, rounded or clamped by this wrapper.
        probe_state_=installed_.query;probe_state_.window_generation=window.window_generation;
        probe_request_={};std::memcpy(probe_request_.reference_asset_sha256,configuration_.reference_asset_sha256,32);
        probe_request_.leg_index=configuration_.leg_index;probe_request_.window_generation=window.window_generation;
        probe_request_.query_sequence=installed_.query.last_accepted_sequence+1;
        probe_request_.progress_s=window.time_s[0]*0.5+window.time_s[1]*0.5+(window.binding_mode==1?25.0:0.0);
        gpenmpcNative_queryCanonicalReferenceWindow(workspace_,&window,&probe_state_,&probe_request_,&probe_next_,probe_jet_,&probe_receipt_);
        ++validation_queries_;
        if(!probe_receipt_.accepted||!finite(probe_jet_,12))return fail(Failure::Window);
        if(window_ready_&&!same_overlap(window_,window))return fail(Failure::Window);
        std::memcpy(&window_,&window,sizeof window_);window_ready_=true;++windows_loaded_;return true;
    }
    PrepareResult prepare(const Input &input,bool operator_velocity=false)noexcept{
        if(failure_!=Failure::None)return PrepareResult::Rejected;
        if(prepared_){fail(Failure::PendingCandidate);return PrepareResult::Rejected;}
        if(!window_ready_)return PrepareResult::WindowMiss;
        const unsigned mode=operator_velocity?2U:1U;
        if(reference_mode_&&reference_mode_!=mode){fail(Failure::Configuration);return PrepareResult::Rejected;}
        if(std::memcmp(input.query.reference_asset_sha256,configuration_.reference_asset_sha256,32)||
           input.query.leg_index!=configuration_.leg_index||input.query.window_generation!=window_.window_generation){
            fail(Failure::Identity);return PrepareResult::Rejected;}
        if(!input.source_timestamp_ns||!input.source_generation||!input.reference_generation||!input.outer_generation||
           input.query.query_sequence<=installed_.query.last_accepted_sequence||
           (committed_&&(input.source_timestamp_ns<=installed_.source_timestamp_ns||input.source_generation<=installed_.source_generation||
            input.reference_generation<installed_.reference_generation||input.outer_generation<installed_.outer_generation))){
            fail(Failure::Ordering);return PrepareResult::Rejected;}
        if(!std::isfinite(input.query.progress_s)||!std::isfinite(input.progress_rate)||
           !std::isfinite(input.target_phase_acceleration)||!finite(input.target_outer_f,3)){
            fail(Failure::Input);return PrepareResult::Rejected;}
        // FromJet uses fraction zero for nonpositive or nonfinite dt.
        // The owner validates source time and age.
        candidate_={};candidate_.input=input;candidate_.next_query=installed_.query;
        candidate_.next_query.window_generation=window_.window_generation;
        if(operator_velocity){
            // USB mode consumes world-frame velocity and uses target[0] as yaw rate.
            // Joint publication installs the reference jet.
            if(std::fabs(input.progress_rate-1.0)>0.0||std::fabs(input.target_phase_acceleration)>gpenmpc_operator_reference::yaw_rate_rad_s){fail(Failure::Input);return PrepareResult::Rejected;}
            const double* prior=committed_?operator_jet_:window_.ground_jet;
            if(!gpenmpc_operator_reference::advance(prior,input.target_outer_f,input.dt_s,candidate_.jet)){
                fail(Failure::Input);return PrepareResult::Rejected;
            }
            probe_next_=candidate_.next_query;
            probe_next_.last_accepted_sequence=input.query.query_sequence;
            candidate_.query_receipt.accepted=true;
            candidate_.query_receipt.query_progress_s=input.query.progress_s;
            candidate_.query_receipt.query_sequence=input.query.query_sequence;
            // This jet is generated by the operator-reference path.
            candidate_.query_receipt.reason=uint8_t(6);
        }else{
            gpenmpcNative_queryCanonicalReferenceWindow(workspace_,&window_,&candidate_.next_query,&input.query,
                &probe_next_,candidate_.jet,&candidate_.query_receipt);++query_calls_;
        }
        if(!candidate_.query_receipt.accepted){
            if(candidate_.query_receipt.reason==5){++window_misses_;return PrepareResult::WindowMiss;}
            fail(Failure::Query);return PrepareResult::Rejected;
        }
        candidate_.next_query=probe_next_;
        const double zero[3]{};
        gpenmpcNative_canonicalReferenceTransitionFromJet(candidate_.jet,input.progress_rate,installed_.phase_acceleration,
            operator_velocity?0.0:input.target_phase_acceleration,installed_.outer_i,operator_velocity?zero:input.target_outer_f,input.dt_s,configuration_.jerk_limit_mps3,&candidate_.transition);
        ++transition_calls_;
        if(!valid_transition(candidate_.transition)||candidate_generation_==UINT64_MAX){fail(Failure::Numerics);return PrepareResult::Rejected;}
        candidate_.candidate_generation=++candidate_generation_;reference_mode_=mode;prepared_=true;return PrepareResult::Candidate;
    }
    const Candidate *candidate()const noexcept{return prepared_&&failure_==Failure::None?&candidate_:nullptr;}
    // Read-only precheck for the single owner's joint install. This neither
    // increments attempts nor rejects/consumes the pending candidate.
    bool validate_publication(const PublicationReceipt &receipt)const noexcept{
        return failure_==Failure::None&&prepared_&&receipt.publication_succeeded&&receipt.numerical_commit_succeeded&&
           receipt.source_timestamp_ns==candidate_.input.source_timestamp_ns&&receipt.source_generation==candidate_.input.source_generation&&
           receipt.candidate_generation==candidate_.candidate_generation&&receipt.query_sequence==candidate_.input.query.query_sequence&&
           receipt.window_generation==candidate_.input.query.window_generation&&receipt.reference_generation==candidate_.input.reference_generation&&
           receipt.outer_generation==candidate_.input.outer_generation&&receipt.output_generation&&receipt.output_generation>installed_.output_generation&&
           receipt.original_publication_us&&receipt.original_publication_us<=UINT64_MAX/1000&&
           receipt.original_publication_us*1000>=receipt.source_timestamp_ns&&same_reference(receipt.actual_reference_pvaj,candidate_.transition.reference);
    }
    // A failed joint publication must not leave an independently installable
    // candidate. The actual backend action remains recorded by its owner.
    void reject_joint_publication()noexcept{(void)fail(Failure::Publication);}
    bool install_after_publication(const PublicationReceipt &receipt)noexcept{
        if(failure_!=Failure::None)return false;
        ++publication_attempts_;
        if(!validate_publication(receipt))return fail(Failure::Publication);
        installed_.query=candidate_.next_query;installed_.query_progress_s=candidate_.input.query.progress_s;
        installed_.progress_rate=candidate_.input.progress_rate;installed_.phase_acceleration=candidate_.transition.phase_acceleration_s_inv;
        std::memcpy(installed_.outer_i,candidate_.transition.outer_correction_i_mps2,sizeof installed_.outer_i);
        installed_.source_timestamp_ns=receipt.source_timestamp_ns;installed_.source_generation=receipt.source_generation;
        installed_.reference_generation=receipt.reference_generation;installed_.outer_generation=receipt.outer_generation;
        installed_.output_generation=receipt.output_generation;installed_.publication_us=receipt.original_publication_us;
        if(reference_mode_==2)std::memcpy(operator_jet_,candidate_.jet,sizeof operator_jet_);
        prepared_=false;committed_=true;++installed_count_;return true;
    }
    // Raw state remains readable after failure.
    const InstalledState &installed_state()const noexcept{return installed_;}
    const struct54_T &last_query_receipt()const noexcept{return candidate_.query_receipt;}
    std::uint64_t loaded_window_generation()const noexcept{return window_ready_?window_.window_generation:0;}
    bool committed()const noexcept{return committed_;}
    Failure failure()const noexcept{return failure_;}
    std::uint64_t query_calls()const noexcept{return query_calls_;}
    std::uint64_t transition_calls()const noexcept{return transition_calls_;}
    std::uint64_t validation_queries()const noexcept{return validation_queries_;}
    std::uint64_t window_misses()const noexcept{return window_misses_;}
    std::uint64_t windows_loaded()const noexcept{return windows_loaded_;}
    std::uint64_t installed_count()const noexcept{return installed_count_;}
    std::uint64_t publication_attempts()const noexcept{return publication_attempts_;}
private:
    static bool finite(const double *p,unsigned n)noexcept{for(unsigned j=0;j<n;++j)if(!std::isfinite(p[j]))return false;return true;}
    static bool bits(const void*a,const void*b,std::size_t n)noexcept{return std::memcmp(a,b,n)==0;}
    static bool same_reference(const double*a,const struct56_T&r)noexcept{
        return bits(a,r.position_m,24)&&bits(a+3,r.velocity_mps,24)&&bits(a+6,r.acceleration_mps2,24)&&bits(a+9,r.jerk_mps3,24);}
    static bool valid_transition(const struct55_T&t)noexcept{
        return finite(t.reference.position_m,3)&&finite(t.reference.velocity_mps,3)&&finite(t.reference.acceleration_mps2,3)&&
            finite(t.reference.jerk_mps3,3)&&std::isfinite(t.phase_acceleration_s_inv)&&std::isfinite(t.phase_jerk_s_inv2)&&
            finite(t.outer_correction_i_mps2,3)&&finite(t.outer_correction_jerk_i_mps3,3)&&std::isfinite(t.fraction)&&
            finite(t.frame_i_from_f,9)&&finite(t.reference_frame_i_from_f,9)&&std::isfinite(t.reference_curvature)&&std::isfinite(t.reference_signed_yaw_rate);}
    static bool same_binding(const struct51_T&a,const struct51_T&b)noexcept{
        return a.schema==b.schema&&a.capacity==b.capacity&&a.source_total_rows==b.source_total_rows&&a.binding_mode==b.binding_mode&&
            bits(&a.nominal_duration_s,&b.nominal_duration_s,8)&&bits(&a.total_duration_s,&b.total_duration_s,8)&&
            bits(&a.prefix_duration_s,&b.prefix_duration_s,8)&&bits(&a.relaunch_duration_s,&b.relaunch_duration_s,8)&&
            bits(&a.vertical_frame_offset_ned_m,&b.vertical_frame_offset_ned_m,8)&&bits(a.prefix_coefficients,b.prefix_coefficients,sizeof a.prefix_coefficients)&&
            bits(a.ground_jet,b.ground_jet,sizeof a.ground_jet)&&bits(a.rest_jet,b.rest_jet,sizeof a.rest_jet)&&
            bits(a.relaunch_offset_ned_m,b.relaunch_offset_ned_m,sizeof a.relaunch_offset_ned_m);}
    static bool same_overlap(const struct51_T&a,const struct51_T&b)noexcept{
        // No adjacency/source-rate assumption is added. If original rows
        // overlap, their values must be byte-identical; full asset integrity
        // still belongs to the external loader, not these shared rows.
        for(unsigned j=0;j<b.row_count;++j){const std::uint64_t row=static_cast<std::uint64_t>(b.source_first_row)+j;
            if(row<a.source_first_row||row>=static_cast<std::uint64_t>(a.source_first_row)+a.row_count)continue;
            const unsigned old=static_cast<unsigned>(row-a.source_first_row);
            if(!bits(a.time_s+old,b.time_s+j,8))return false;
            for(unsigned k=0;k<12;++k)if(!bits(a.nominal_jet+old+256*k,b.nominal_jet+j+256*k,8))return false;}
        return true;
    }
    bool fail(Failure f)noexcept{if(failure_==Failure::None)failure_=f;prepared_=false;return false;}
    Workspace *const workspace_;
    const Configuration configuration_;
    struct51_T window_{};Candidate candidate_{};InstalledState installed_{};
    struct52_T probe_state_{},probe_next_{};struct53_T probe_request_{};struct54_T probe_receipt_{};double probe_jet_[12]{};
    std::uint64_t candidate_generation_{},query_calls_{},transition_calls_{},validation_queries_{},window_attempts_{},
        window_misses_{},windows_loaded_{},installed_count_{},publication_attempts_{};
    Failure failure_{Failure::None};bool window_ready_{},prepared_{},committed_{};
    unsigned reference_mode_{};double operator_jet_[12]{};
};
} // namespace gpenmpc_reference_math
