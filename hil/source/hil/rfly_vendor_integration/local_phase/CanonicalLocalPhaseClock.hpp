#pragma once
// Single-owner numerical phase state. No clock read, source construction,
// publisher, permission, scheduler, reset, or HOST phase override. Its caller
// must obtain installation/publication records from the actual LocalIo.
#include "CanonicalLocalPhase.h"
#include "../CanonicalFullInnerConsumption.hpp"

namespace gpenmpc_local_phase {
enum class Fault:std::uint8_t {None,Configuration,Pending,Snapshot,Interval,
    SourceOrder,Outer,Overflow,Installation,Retired};
struct Configuration {
    gpenmpc_consumption::Identity identity{};
    gpenmpc_local_input::Hash configuration_sha256{};
    std::uint8_t reference_asset_sha256[32]{};
    std::uint32_t leg_index{};
    // Original per-leg task duration and canonical phase-rate configuration.
    // First phase=0/rate=1, as in the original MATLAB adapter.
    double duration_s{},rate_min{},rate_max{};
};
struct Diagnostics {
    Fault first_fault{Fault::None};
    std::uint64_t queries{},window_retries{},installs{};
    std::uint64_t last_source_timestamp_ns{},last_source_generation{},
        last_output_generation{},last_publication_us{};
    double phase_s{},phase_rate{1.0};
};
class CanonicalLocalPhaseClock final {
public:
    explicit CanonicalLocalPhaseClock(const Configuration&c)noexcept:config_(c){
        bool asset=false;for(auto v:c.reference_asset_sha256)asset=asset||v!=0;
        if(!c.identity.uid||!c.identity.boot_generation||!c.identity.system||!c.identity.component||
           !asset||!c.leg_index||gpenmpc_local_input::empty(c.configuration_sha256)||
           !std::isfinite(c.duration_s)||c.duration_s<=0||!std::isfinite(c.rate_min)||
           !std::isfinite(c.rate_max)||c.rate_min<=0||c.rate_min>1||c.rate_max<1)
            fail(Fault::Configuration);
    }
    CanonicalLocalPhaseClock(const CanonicalLocalPhaseClock&)=delete;
    CanonicalLocalPhaseClock&operator=(const CanonicalLocalPhaseClock&)=delete;
    bool begin(const gpenmpc_odometry::Snapshot&s,const gpenmpc_consumption::OuterCommand&o,
        const double target4[4],std::uint64_t window_generation,
        const gpenmpc_local_input::ExplicitInitialInterval*initial,
        gpenmpc_full_inner_reference_input&out,bool operator_reference=false)noexcept {
        out={};if(d_.first_fault!=Fault::None)return false;
        if(pending_)return fail(Fault::Pending);
        if(!s.valid()||!(s.estimator().identity==config_.identity))return fail(Fault::Snapshot);
        const auto&e=s.estimator();
        if(!e.timestamp_sample_us||e.timestamp_sample_us>UINT64_MAX/1000||
           d_.installs==UINT64_MAX)return fail(Fault::Overflow);
        if(!target4||!window_generation||!o.generation||!(o.identity==config_.identity)||
           !o.based_on_sample_generation||o.based_on_sample_generation>e.generation||
           !o.based_on_timestamp_sample_us||o.based_on_timestamp_sample_us>e.timestamp_sample_us)
            return fail(Fault::Outer);
        gpenmpc_consumption::CanonicalSha256 h;
        for(unsigned j=0;j<4;++j){if(!std::isfinite(target4[j]))return fail(Fault::Outer);h.real(target4[j]);}
        if(h.finish()!=o.payload_sha256)return fail(Fault::Outer);
        if(d_.installs&&(o.generation<last_outer_generation_||
           (o.generation==last_outer_generation_&&o.payload_sha256!=last_outer_sha_)))return fail(Fault::Outer);
        double dt=0;
        if(!d_.installs){
            if(!initial||initial->leg!=config_.leg_index||initial->configuration_sha256!=config_.configuration_sha256||
               gpenmpc_local_input::empty(initial->original_configuration_receipt_sha256))return fail(Fault::Interval);
            dt=initial->configured_dt_s;
        }else{
            if(initial||e.generation<=d_.last_source_generation||e.timestamp_sample_us*1000<=d_.last_source_timestamp_ns||
               !s.actual_sample_delta_us()||
               (e.timestamp_sample_us-d_.last_source_timestamp_ns/1000)!=s.actual_sample_delta_us())return fail(Fault::SourceOrder);
            dt=static_cast<double>(s.actual_sample_delta_us())*1e-6;
        }
        // Validate the deployment input interval.
        if(!gpenmpc_operator_reference::valid_control_interval_s(dt,operator_reference))return fail(Fault::Interval);
        q_={};std::memcpy(q_.reference_asset_sha256,config_.reference_asset_sha256,32);
        q_.leg_index=config_.leg_index;q_.window_generation=window_generation;
        q_.query_sequence=d_.installs+1;q_.source_timestamp_ns=e.timestamp_sample_us*1000;
        q_.source_generation=e.generation;
        // A new actual local reference query, not an outer-solver generation.
        q_.reference_generation=q_.query_sequence;q_.outer_generation=o.generation;
        q_.progress_s=d_.phase_s;q_.progress_rate=d_.phase_rate;q_.dt_s=dt;
        q_.target_phase_acceleration=target4[0];
        for(unsigned j=0;j<3;++j)q_.target_outer_f[j]=target4[j+1];
        pending_outer_sha_=o.payload_sha256;pending_=true;++d_.queries;out=q_;return true;
    }
    // Only after actual ABI WINDOW_MISS and a validated refill. It cannot
    // change the source, phase, dt, outer target, or original runtime deadline.
    bool retry_window(std::uint64_t generation,gpenmpc_full_inner_reference_input&out)noexcept{
        out={};if(d_.first_fault!=Fault::None)return false;
        if(!pending_||generation<=q_.window_generation)return fail(Fault::Pending);
        q_.window_generation=generation;++d_.window_retries;out=q_;return true;
    }
    bool install(const gpenmpc_full_inner_reference_state&s,
                 const gpenmpc_full_inner_backend&publication)noexcept{
        if(d_.first_fault!=Fault::None)return false;
        if(!pending_||!publication.output_published||
           std::memcmp(s.reference_asset_sha256,config_.reference_asset_sha256,32)!=0||
           s.leg_index!=q_.leg_index||s.window_generation!=q_.window_generation||
           s.last_accepted_sequence!=q_.query_sequence||s.source_timestamp_ns!=q_.source_timestamp_ns||
           s.source_generation!=q_.source_generation||s.reference_generation!=q_.reference_generation||
           s.outer_generation!=q_.outer_generation||!same(s.query_progress_s,q_.progress_s)||
           !same(s.progress_rate,q_.progress_rate)||!std::isfinite(s.phase_acceleration)||
           publication.source_timestamp_ns!=q_.source_timestamp_ns||publication.source_generation!=q_.source_generation||
           !s.output_generation||s.output_generation!=publication.output_generation||s.output_generation<=d_.last_output_generation||
           !s.publication_us||s.publication_us!=publication.original_publication_us||
           s.publication_us<q_.source_timestamp_ns/1000||s.publication_us<=d_.last_publication_us)
            return fail(Fault::Installation);
        for(auto v:s.outer_i)if(!std::isfinite(v))return fail(Fault::Installation);
        const double inputs[7]={q_.progress_s,q_.progress_rate,s.phase_acceleration,q_.dt_s,
            config_.duration_s,config_.rate_min,config_.rate_max};double next[2]{};
        gpenmpcNative_canonicalLocalPhaseAdvance(inputs,next);
        if(!std::isfinite(next[0])||!std::isfinite(next[1])||next[0]<d_.phase_s||next[0]>config_.duration_s||
           next[1]<config_.rate_min||next[1]>config_.rate_max)return fail(Fault::Installation);
        d_.phase_s=next[0];d_.phase_rate=next[1];++d_.installs;
        d_.last_source_timestamp_ns=q_.source_timestamp_ns;d_.last_source_generation=q_.source_generation;
        d_.last_output_generation=s.output_generation;d_.last_publication_us=s.publication_us;
        last_outer_generation_=q_.outer_generation;last_outer_sha_=pending_outer_sha_;pending_=false;return true;
    }
    void retire()noexcept{fail(Fault::Retired);}
    const Diagnostics&diagnostics()const noexcept{return d_;}
    bool pending()const noexcept{return pending_;}
private:
    static bool same(double a,double b)noexcept{return std::memcmp(&a,&b,sizeof a)==0;}
    bool fail(Fault f)noexcept{if(d_.first_fault==Fault::None)d_.first_fault=f;return false;}
    const Configuration config_;
    Diagnostics d_{};gpenmpc_full_inner_reference_input q_{};bool pending_{};
    std::uint64_t last_outer_generation_{};
    gpenmpc_local_input::Hash last_outer_sha_{},pending_outer_sha_{};
};
}
