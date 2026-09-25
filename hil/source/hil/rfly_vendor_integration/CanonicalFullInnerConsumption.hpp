#pragma once
// RFL2 stateful numerical consumption receipt. The caller supplies the
// publication event; the receipt binds its identity, values and timestamps.
#include "CanonicalLocalInnerInputBuilder.hpp"
#include "full_inner_abi/CanonicalFullInnerAbi.h"
#include <cstring>

namespace gpenmpc_full_consumption {
using Hash=gpenmpc_portable::Array<std::uint32_t,8>;
using Identity=gpenmpc_consumption::Identity;
using Limits=gpenmpc_consumption::Limits;
using Reference=gpenmpc_consumption::Reference;
using OuterCommand=gpenmpc_consumption::OuterCommand;
constexpr std::uint32_t domain_rfl2=0x52464c32U;
enum class Fault:std::uint8_t {None,Configuration,Snapshot,IdentityMismatch,InputBinding,Interval,ReferenceMismatch,Outer,
    SourceTime,Stale,DuplicateSource,SourceRegression,Reset,TickRegression,ReferenceRegression,ReferenceMutation,
    OuterRegression,OuterMutation,HashMismatch,Candidate,Pending,TokenMismatch,PublicationFailed,PublicationMismatch,
    Deadline,CounterOverflow,Exception,Revoked};
struct Configuration {
    Identity identity{};
    Limits limits{};
    gpenmpc_full_inner_configuration numerical{};
    gpenmpc_full_inner_build_identity build{};
    bool operator_reference{};
};
struct NumericalEvidence {
    // Copy the real ABI state BEFORE prepare; GP completion may legitimately
    // change its open pending prediction since the preceding publication.
    Hash original_prior_committed_state_sha256{},original_loaded_window_sha256{};
    gpenmpc_full_inner_reference_input original_reference_query{};
};
struct StatefulToken {
    gpenmpc_consumption::Token lease_envelope{};
    std::uint32_t domain{domain_rfl2};
    Hash prior_committed_state_sha256{},full_inner_input_sha256{},loaded_window_sha256{},candidate_sha256{};
    Hash task_sha256{},configuration_sha256{},reference_asset_sha256{},facade_source_sha256{},private_archive_sha256{};
    Hash rotor_association_receipt_sha256{};
    std::uint64_t explicit_leg{},reference_candidate_generation{},reference_query_sequence{},reference_window_generation{};
    std::uint64_t rotor_generation{},rotor_session{},payload_generation{},wind_generation{};
    std::uint64_t kernel_completed_us{},original_valid_until_us{};
    bool board_authority{}; // always false, not accepted as an admission flag
};
bool operator==(const StatefulToken&,const StatefulToken&)noexcept;
struct Receipt {
    bool valid{},board_authority{},full_numeric_reference_state_installed{};
    StatefulToken token{};
    std::uint64_t original_publication_us{},original_validation_us{};
    float actual_control16[16]{};
};
struct Diagnostics {
    std::uint64_t begun_transactions{},commit_attempts{},reported_publications{},failed_publication_reports{},committed_outputs{};
    std::uint64_t last_publication_us{},last_validation_us{};
    StatefulToken last_reported_token{};
    float last_reported_control16[16]{};
    Fault first_fault{Fault::None};
};
class CanonicalFullInnerConsumption final {
public:
    explicit CanonicalFullInnerConsumption(const Configuration&)noexcept;
    CanonicalFullInnerConsumption(const CanonicalFullInnerConsumption&)=delete;
    CanonicalFullInnerConsumption&operator=(const CanonicalFullInnerConsumption&)=delete;
    // The reference envelope maps canonical p/v/a into TASK-NED by negating z.
    // The query, loaded window and prior state determine the input digest.
    bool begin(const gpenmpc_odometry::Snapshot&,const gpenmpc_local_input::Result&,
        const gpenmpc_full_inner_candidate&,const NumericalEvidence&,
        const Reference&,const OuterCommand&,std::uint64_t original_start_us,
        std::uint64_t kernel_completed_us,StatefulToken&)noexcept;
    // Call after the publisher reports success. This validates the supplied
    // event; the owner then completes the full ABI joint-state installation.
    bool commit(const StatefulToken&,const float actual_published16[16],
        std::uint64_t original_publication_us,std::uint64_t validation_us,Receipt&)noexcept;
    bool note_publication_failure(const StatefulToken&,std::uint64_t original_attempt_us)noexcept;
    void note_exception()noexcept;
    void revoke()noexcept;
    Fault fault()const noexcept{return diagnostics_.first_fault;}
    const Diagnostics&diagnostics()const noexcept{return diagnostics_;}
    std::uint64_t committed_outputs()const noexcept{return diagnostics_.committed_outputs;}
private:
    bool fail(Fault)noexcept;
    Configuration config_{};Diagnostics diagnostics_{};
    bool pending_{},have_last_{};
    StatefulToken pending_token_{};
    float pending_control16_[16]{};
    gpenmpc_consumption::Estimator pending_state_{},last_state_{};
    Reference pending_reference_{},last_reference_{};
    OuterCommand pending_outer_{},last_outer_{};
    std::uint64_t last_start_us_{};
};

namespace detail {
using Sha=gpenmpc_consumption::CanonicalSha256;
inline Hash words(const std::uint8_t b[32])noexcept{
    Hash h{};for(unsigned i=0;i<8;++i)for(unsigned j=0;j<4;++j)h[i]=(h[i]<<8)|b[4*i+j];return h;
}
inline bool empty(const Hash&h)noexcept{return gpenmpc_local_input::empty(h);}
inline void bytes(Sha&h,const std::uint8_t*b,unsigned n)noexcept{for(unsigned i=0;i<n;++i)h.byte(b[i]);}
template<std::size_t N>inline void domain(Sha&h,const char(&s)[N])noexcept{for(char c:s)h.byte(static_cast<std::uint8_t>(c));}
inline void reals(Sha&h,const double*p,unsigned n)noexcept{for(unsigned i=0;i<n;++i)h.real(p[i]);}
inline bool finite(const double*p,unsigned n)noexcept{for(unsigned i=0;i<n;++i)if(!std::isfinite(p[i]))return false;return true;}
inline bool bits(double a,double b)noexcept{return std::memcmp(&a,&b,sizeof a)==0;}
inline std::uint64_t expiry(std::uint64_t t,std::uint64_t age)noexcept{return age>UINT64_MAX-t?UINT64_MAX:t+age;}
inline void lower(std::uint64_t&v,std::uint64_t b)noexcept{if(b<v)v=b;}
inline void hash_reference(Sha&h,const Reference&r)noexcept{
    gpenmpc_consumption::hash_identity(h,r.identity);h.u64(r.generation);h.u64(r.outer_generation);h.u64(r.timestamp_us);
    h.u64(r.board_rx_us);h.u64(r.valid_until_us);h.reals(r.p);h.reals(r.v);h.reals(r.a);h.real(r.yaw);h.real(r.yaw_rate);
}
inline void hash_outer(Sha&h,const OuterCommand&o)noexcept{
    gpenmpc_consumption::hash_identity(h,o.identity);h.u64(o.generation);h.u64(o.based_on_sample_generation);
    h.u64(o.based_on_timestamp_sample_us);h.u64(o.board_rx_us);h.u64(o.valid_until_us);h.words(o.payload_sha256);
}
inline bool same_reference(const Reference&a,const Reference&b)noexcept{
    Sha x,y;hash_reference(x,a);hash_reference(y,b);return x.finish()==y.finish();
}
inline bool same_outer(const OuterCommand&a,const OuterCommand&b)noexcept{
    Sha x,y;hash_outer(x,a);hash_outer(y,b);return x.finish()==y.finish();
}
// Existing RCT1 outer payload: SHA256 of four original binary64 values in
// big-endian order, no added domain bytes (RflyContextWire.hpp binding).
inline Hash outer_payload_sha(const gpenmpc_full_inner_reference_input&q)noexcept{
    Sha h;h.real(q.target_phase_acceleration);reals(h,q.target_outer_f,3);return h.finish();
}
// Independent implementation of the public facade's numerical-input domain.
// Byte order and the terminating NUL are intentional; no POD padding is read.
inline Hash numerical_input_sha(const Configuration&c,const gpenmpc_local_input::Result&in,const NumericalEvidence&e)noexcept{
    Sha h;domain(h,"GPENMPC_FULL_INNER_INPUT36_STATE64_PENDING70_REFERENCE_V1");
    bytes(h,c.build.generated_source_set_sha256,32);bytes(h,c.build.facade_source_sha256,32);
    const auto&n=c.numerical;bytes(h,n.task_sha256,32);bytes(h,n.configuration_sha256,32);bytes(h,n.reference_asset_sha256,32);
    h.u32(n.leg_index);h.real(n.initial_phase_acceleration);reals(h,n.initial_outer_i,3);h.real(n.jerk_limit_mps3);
    h.words(e.original_loaded_window_sha256);h.words(e.original_prior_committed_state_sha256);
    reals(h,in.input36,36);h.u64(in.tags2[0]);h.u64(in.tags2[1]);const auto&q=e.original_reference_query;
    bytes(h,q.reference_asset_sha256,32);h.u32(q.leg_index);h.u64(q.window_generation);h.u64(q.query_sequence);
    h.u64(q.source_timestamp_ns);h.u64(q.source_generation);h.u64(q.reference_generation);h.u64(q.outer_generation);
    h.real(q.progress_s);h.real(q.progress_rate);h.real(q.target_phase_acceleration);reals(h,q.target_outer_f,3);h.real(q.dt_s);
    return h.finish();
}
inline Hash candidate_sha(const gpenmpc_full_inner_candidate&c)noexcept{
    Sha h;domain(h,"GPENMPC_RFL2_ACTUAL_FULL_LOCAL_CANDIDATE_V1");
    reals(h,c.post_state64,64);reals(h,c.kernel61,61);reals(h,c.scaffold70,70);reals(h,c.request19,19);reals(h,c.consumed_reference_pvaj,12);
    h.u64(c.original_tags2[0]);h.u64(c.original_tags2[1]);h.u64(c.reference_candidate_generation);h.u64(c.reference_query_sequence);
    h.u64(c.reference_window_generation);h.u64(c.reference_generation);h.u64(c.outer_generation);
    for(float f:c.control16){std::uint32_t b{};std::memcpy(&b,&f,sizeof b);h.u32(b);}
    bytes(h,c.prior_committed_state_sha256,32);bytes(h,c.full_inner_input_sha256,32);h.byte(c.control_authority);return h.finish();
}
inline void hash_metadata(Sha&h,const StatefulToken&t)noexcept{
    h.u32(t.domain);h.words(t.prior_committed_state_sha256);h.words(t.full_inner_input_sha256);h.words(t.loaded_window_sha256);
    h.words(t.candidate_sha256);h.words(t.task_sha256);h.words(t.configuration_sha256);h.words(t.reference_asset_sha256);
    h.words(t.facade_source_sha256);h.words(t.private_archive_sha256);h.words(t.rotor_association_receipt_sha256);
    h.u64(t.explicit_leg);h.u64(t.reference_candidate_generation);h.u64(t.reference_query_sequence);h.u64(t.reference_window_generation);
    h.u64(t.rotor_generation);h.u64(t.rotor_session);h.u64(t.payload_generation);h.u64(t.wind_generation);
    h.u64(t.kernel_completed_us);h.u64(t.original_valid_until_us);h.byte(t.board_authority);
}
inline Hash full_sha(const Configuration&c,const StatefulToken&t,const gpenmpc_local_input::Result&in,
    const Reference&r,const OuterCommand&o)noexcept{
    Sha h;h.u32(domain_rfl2);domain(h,"GPENMPC_STATEFUL_FULL_LOCAL_LEASE_V1");hash_metadata(h,t);
    const auto&l=t.lease_envelope;gpenmpc_consumption::hash_identity(h,l.identity);h.byte(static_cast<std::uint8_t>(l.publication_path));
    h.u64(l.transaction);h.u64(l.output_generation);h.u64(l.sample_generation);h.u64(l.timestamp_sample_us);h.u64(l.source_generation_delta);
    h.u64(l.state_publication_us);h.u64(l.state_board_rx_us);h.u64(l.control_tick_us);h.u64(l.sample_delta_us);h.u64(l.actual_tick_delta_us);
    h.words(l.kernel_source_sha256);h.words(in.source.state_and_origin_sha256);h.byte(in.source.reset_counter);
    h.byte(static_cast<std::uint8_t>(in.interval_basis));h.words(in.initial_interval_configuration_receipt_sha256);
    reals(h,in.input36,36);h.u64(in.tags2[0]);h.u64(in.tags2[1]);hash_reference(h,r);hash_outer(h,o);
    h.u64(c.limits.sample_max_age_us);h.u64(c.limits.reference_max_age_us);h.u64(c.limits.outer_max_age_us);h.u64(c.limits.transaction_max_wall_us);
    return h.finish();
}
} // namespace detail

inline bool operator==(const StatefulToken&a,const StatefulToken&b)noexcept{
    detail::Sha x,y;detail::hash_metadata(x,a);detail::hash_metadata(y,b);
    return a.lease_envelope==b.lease_envelope&&x.finish()==y.finish();
}
inline bool CanonicalFullInnerConsumption::fail(Fault f)noexcept{
    if(diagnostics_.first_fault==Fault::None)diagnostics_.first_fault=f;
    return false;
}
inline CanonicalFullInnerConsumption::CanonicalFullInnerConsumption(const Configuration&c)noexcept:config_(c){
    const auto&n=c.numerical;const auto&l=c.limits;
    if(!c.identity.uid||!c.identity.boot_generation||!c.identity.system||!c.identity.component||!l.sample_max_age_us||
       !l.reference_max_age_us||!l.outer_max_age_us||!l.transaction_max_wall_us||
       l.publication_path!=gpenmpc_consumption::PublicationPath::DirectCanonicalMotors||
       n.abi_version!=GPENMPC_FULL_INNER_ABI_VERSION||!n.leg_index||!std::isfinite(n.initial_phase_acceleration)||
       !detail::finite(n.initial_outer_i,3)||!std::isfinite(n.jerk_limit_mps3)||n.jerk_limit_mps3<0||
       detail::empty(detail::words(n.task_sha256))||detail::empty(detail::words(n.configuration_sha256))||
       detail::empty(detail::words(n.reference_asset_sha256))||detail::empty(detail::words(c.build.generated_source_set_sha256))||
       detail::empty(detail::words(c.build.facade_source_sha256))||detail::empty(detail::words(c.build.private_archive_sha256)))fail(Fault::Configuration);
}
inline bool CanonicalFullInnerConsumption::begin(const gpenmpc_odometry::Snapshot&snapshot,const gpenmpc_local_input::Result&in,
    const gpenmpc_full_inner_candidate&candidate,const NumericalEvidence&e,const Reference&r,const OuterCommand&o,
    std::uint64_t start,std::uint64_t completed,StatefulToken&out)noexcept{
    out={};if(fault()!=Fault::None)return false;if(pending_)return fail(Fault::Pending);
    gpenmpc_local_input::SnapshotKey key{};if(!gpenmpc_local_input::snapshot_key(snapshot,key))return fail(Fault::Snapshot);
    const auto&s=snapshot.estimator();const auto&q=e.original_reference_query;
    if(!(s.identity==config_.identity)||!(r.identity==config_.identity)||!(o.identity==config_.identity))return fail(Fault::IdentityMismatch);
    if(!in.assembled||in.failure!=gpenmpc_local_input::Failure::None||in.board_authority||in.external_clock_association_proven_here||
       !gpenmpc_local_input::same_source(key,in.source)||!detail::finite(in.input36,36)||in.explicit_leg!=config_.numerical.leg_index||
       !detail::bits(in.input36[35],static_cast<double>(in.explicit_leg))||key.sample_us>UINT64_MAX/1000||
       in.tags2[0]!=key.sample_us*1000||in.tags2[1]!=key.source_generation||!in.rotor_generation||!in.rotor_session||
       !in.payload_generation||!in.wind_generation||detail::empty(in.rotor_association_receipt_sha256))return fail(Fault::InputBinding);
    gpenmpc_portable::Array<double,13>x{};if(!gpenmpc_snapshot_mapping::state13(snapshot,x))return fail(Fault::Snapshot);
    for(unsigned i=0;i<13;++i)if(!detail::bits(x[i],in.input36[i]))return fail(Fault::InputBinding);
    for(unsigned i=13;i<19;++i)if(in.input36[i]<0)return fail(Fault::InputBinding);
    if(in.input36[31]<0)return fail(Fault::InputBinding);
    if(!gpenmpc_operator_reference::valid_control_interval_s(in.input36[34],config_.operator_reference)||
       !detail::bits(in.input36[34],q.dt_s))return fail(Fault::Interval);
    if(!have_last_){
        if(in.interval_basis!=gpenmpc_local_input::IntervalBasis::ExplicitConfiguredLegInitial||
           detail::empty(in.initial_interval_configuration_receipt_sha256))return fail(Fault::Interval);
    }else{
        if(s.generation==last_state_.generation||s.timestamp_sample_us==last_state_.timestamp_sample_us)return fail(Fault::DuplicateSource);
        if(s.generation<last_state_.generation||s.timestamp_sample_us<last_state_.timestamp_sample_us||
           s.board_rx_us<=last_state_.board_rx_us||s.publication_us<last_state_.publication_us)return fail(Fault::SourceRegression);
        if(s.reset_counter!=last_state_.reset_counter&&!snapshot.heading_reset_from(last_state_.reset_counter))return fail(Fault::Reset);
        const auto delta=s.timestamp_sample_us-last_state_.timestamp_sample_us;
        if((!config_.operator_reference&&delta>50000U)||key.sample_delta_us!=delta||
           in.interval_basis!=gpenmpc_local_input::IntervalBasis::MeasuredSnapshotDelta||
           !detail::bits(in.input36[34],static_cast<double>(delta)*1e-6))return fail(Fault::Interval);
        if(start<=last_start_us_||start<diagnostics_.last_validation_us)return fail(Fault::TickRegression);
        if(r.generation<last_reference_.generation)return fail(Fault::ReferenceRegression);
        if(r.generation==last_reference_.generation){if(!detail::same_reference(r,last_reference_))return fail(Fault::ReferenceMutation);}
        else if(r.board_rx_us<=last_reference_.board_rx_us||r.timestamp_us<=last_reference_.timestamp_us)return fail(Fault::ReferenceRegression);
        if(o.generation<last_outer_.generation)return fail(Fault::OuterRegression);
        if(o.generation==last_outer_.generation){if(!detail::same_outer(o,last_outer_))return fail(Fault::OuterMutation);}
        else if(o.board_rx_us<=last_outer_.board_rx_us)return fail(Fault::OuterRegression);
        if(q.query_sequence<=pending_token_.reference_query_sequence||
           candidate.reference_candidate_generation<=pending_token_.reference_candidate_generation)return fail(Fault::ReferenceRegression);
    }
    if(!r.generation||!r.outer_generation||r.outer_generation!=o.generation||!gpenmpc_consumption::finite(r.p)||
       !gpenmpc_consumption::finite(r.v)||!gpenmpc_consumption::finite(r.a)||!std::isfinite(r.yaw)||!std::isfinite(r.yaw_rate))return fail(Fault::ReferenceMismatch);
    // Explicit coordinate-only envelope conversion. Never changes the actual
    // canonical pvaj passed to C; task origin was applied once in state13.
    for(unsigned j=0;j<3;++j){const double sign=j==2?-1.0:1.0;
        if(!detail::bits(r.p[j],sign*in.input36[19+j])||!detail::bits(r.v[j],sign*in.input36[22+j])||
           !detail::bits(r.a[j],sign*in.input36[25+j]))return fail(Fault::ReferenceMismatch);
    }
    if(!o.generation||!o.based_on_sample_generation||!o.based_on_timestamp_sample_us||detail::empty(o.payload_sha256)||
       o.based_on_sample_generation>s.generation||o.based_on_timestamp_sample_us>s.timestamp_sample_us||
       (o.based_on_sample_generation==s.generation&&o.based_on_timestamp_sample_us!=s.timestamp_sample_us)||
       o.payload_sha256!=detail::outer_payload_sha(q))return fail(Fault::Outer);
    if(!start||!s.timestamp_sample_us||!s.publication_us||!s.board_rx_us||!r.timestamp_us||!r.board_rx_us||!o.board_rx_us||
       s.timestamp_sample_us>s.publication_us||s.publication_us>s.board_rx_us||s.board_rx_us>start||r.timestamp_us>r.board_rx_us||
       r.board_rx_us>start||o.board_rx_us>r.board_rx_us||o.based_on_timestamp_sample_us>o.board_rx_us||
       r.valid_until_us<r.board_rx_us||o.valid_until_us<o.board_rx_us)return fail(Fault::SourceTime);
    std::uint64_t valid=detail::expiry(s.timestamp_sample_us,config_.limits.sample_max_age_us);
    detail::lower(valid,detail::expiry(r.board_rx_us,config_.limits.reference_max_age_us));detail::lower(valid,r.valid_until_us);
    detail::lower(valid,detail::expiry(o.board_rx_us,config_.limits.outer_max_age_us));detail::lower(valid,o.valid_until_us);
    detail::lower(valid,detail::expiry(start,config_.limits.transaction_max_wall_us));
    if(start>valid)return fail(Fault::Stale);
    if(completed<start||completed>valid)return fail(Fault::Deadline);
    if(candidate.control_authority||candidate.original_tags2[0]!=in.tags2[0]||candidate.original_tags2[1]!=in.tags2[1]||
       !candidate.reference_candidate_generation||candidate.reference_candidate_generation!=in.reference_candidate_token||
       candidate.reference_window_generation!=in.reference_window_generation||!candidate.reference_window_generation||
       candidate.reference_query_sequence!=q.query_sequence||!q.query_sequence||
       candidate.reference_generation!=r.generation||candidate.outer_generation!=o.generation||
       q.source_timestamp_ns!=in.tags2[0]||q.source_generation!=in.tags2[1]||q.reference_generation!=r.generation||q.outer_generation!=o.generation||
       q.window_generation!=in.reference_window_generation||q.leg_index!=in.explicit_leg||
       detail::words(q.reference_asset_sha256)!=detail::words(config_.numerical.reference_asset_sha256)||
       !std::isfinite(q.progress_s)||!std::isfinite(q.progress_rate)||!std::isfinite(q.target_phase_acceleration)||
       !detail::finite(q.target_outer_f,3)||!detail::finite(candidate.kernel61,61))return fail(Fault::Candidate);
    for(unsigned i=0;i<12;++i)if(!detail::bits(candidate.consumed_reference_pvaj[i],in.input36[19+i]))return fail(Fault::Candidate);
    for(float v:candidate.control16)if(!std::isfinite(v)||v<-1||v>1)return fail(Fault::Candidate);
    // NaNs in the original closed-GP scaffold/request are legal placeholders
    // when prediction_required=false. Hash their actual bits, never zero them.
    if(detail::empty(e.original_prior_committed_state_sha256)||detail::empty(e.original_loaded_window_sha256)||
       detail::words(candidate.prior_committed_state_sha256)!=e.original_prior_committed_state_sha256||
       detail::words(candidate.full_inner_input_sha256)!=detail::numerical_input_sha(config_,in,e))return fail(Fault::HashMismatch);
    if(diagnostics_.begun_transactions==UINT64_MAX||diagnostics_.committed_outputs==UINT64_MAX)return fail(Fault::CounterOverflow);
    StatefulToken t{};auto&l=t.lease_envelope;l.identity=config_.identity;l.publication_path=config_.limits.publication_path;
    l.transaction=diagnostics_.begun_transactions+1;l.output_generation=diagnostics_.committed_outputs+1;
    l.sample_generation=s.generation;l.timestamp_sample_us=s.timestamp_sample_us;l.source_generation_delta=key.generation_delta;
    l.state_publication_us=s.publication_us;l.state_board_rx_us=s.board_rx_us;l.control_tick_us=start;
    l.sample_delta_us=key.sample_delta_us;l.actual_tick_delta_us=have_last_?start-last_start_us_:0;
    l.reference_generation=r.generation;l.reference_timestamp_us=r.timestamp_us;l.reference_board_rx_us=r.board_rx_us;l.reference_valid_until_us=r.valid_until_us;
    l.outer_generation=o.generation;l.outer_board_rx_us=o.board_rx_us;l.outer_valid_until_us=o.valid_until_us;
    l.outer_based_on_sample_generation=o.based_on_sample_generation;l.outer_based_on_timestamp_sample_us=o.based_on_timestamp_sample_us;
    l.outer_payload_sha256=o.payload_sha256;l.kernel_source_sha256=detail::words(config_.build.generated_source_set_sha256);
    t.prior_committed_state_sha256=e.original_prior_committed_state_sha256;t.loaded_window_sha256=e.original_loaded_window_sha256;
    t.full_inner_input_sha256=detail::words(candidate.full_inner_input_sha256);t.candidate_sha256=detail::candidate_sha(candidate);
    t.task_sha256=detail::words(config_.numerical.task_sha256);t.configuration_sha256=detail::words(config_.numerical.configuration_sha256);
    t.reference_asset_sha256=detail::words(config_.numerical.reference_asset_sha256);t.facade_source_sha256=detail::words(config_.build.facade_source_sha256);
    t.private_archive_sha256=detail::words(config_.build.private_archive_sha256);t.rotor_association_receipt_sha256=in.rotor_association_receipt_sha256;
    t.explicit_leg=in.explicit_leg;t.reference_candidate_generation=candidate.reference_candidate_generation;
    t.reference_query_sequence=q.query_sequence;t.reference_window_generation=in.reference_window_generation;
    t.rotor_generation=in.rotor_generation;t.rotor_session=in.rotor_session;t.payload_generation=in.payload_generation;t.wind_generation=in.wind_generation;
    t.kernel_completed_us=completed;t.original_valid_until_us=valid;l.full_input_sha256=detail::full_sha(config_,t,in,r,o);
    // kernel_argument_sha256 deliberately stays ZERO: never legacy RAK1/101.
    pending_token_=t;std::memcpy(pending_control16_,candidate.control16,sizeof pending_control16_);
    pending_state_=s;pending_reference_=r;pending_outer_=o;pending_=true;++diagnostics_.begun_transactions;out=t;return true;
}
inline bool CanonicalFullInnerConsumption::commit(const StatefulToken&t,const float actual[16],std::uint64_t published,
    std::uint64_t validation,Receipt&out)noexcept{
    out={};
    // Record reported side effects BEFORE validation and never roll them back.
    if(diagnostics_.commit_attempts==UINT64_MAX||diagnostics_.reported_publications==UINT64_MAX)return fail(Fault::CounterOverflow);
    ++diagnostics_.commit_attempts;++diagnostics_.reported_publications;diagnostics_.last_reported_token=t;
    const auto previous_publication=diagnostics_.last_publication_us;
    diagnostics_.last_publication_us=published;diagnostics_.last_validation_us=validation;
    if(actual)std::memcpy(diagnostics_.last_reported_control16,actual,sizeof diagnostics_.last_reported_control16);
    if(fault()!=Fault::None)return false;
    if(!pending_||!(t==pending_token_))return fail(Fault::TokenMismatch);
    if(!actual||std::memcmp(actual,pending_control16_,sizeof pending_control16_))return fail(Fault::PublicationMismatch);
    if(published<t.kernel_completed_us||validation<published||validation>t.original_valid_until_us||
       (have_last_&&published<=previous_publication))return fail(Fault::Deadline);
    if(diagnostics_.committed_outputs==UINT64_MAX)return fail(Fault::CounterOverflow);
    ++diagnostics_.committed_outputs;last_state_=pending_state_;last_reference_=pending_reference_;last_outer_=pending_outer_;
    last_start_us_=t.lease_envelope.control_tick_us;have_last_=true;pending_=false;
    out.valid=true;out.token=t;out.original_publication_us=published;out.original_validation_us=validation;
    std::memcpy(out.actual_control16,actual,sizeof out.actual_control16);return true;
}
inline bool CanonicalFullInnerConsumption::note_publication_failure(const StatefulToken&t,std::uint64_t attempted)noexcept{
    if(diagnostics_.failed_publication_reports==UINT64_MAX)return fail(Fault::CounterOverflow);
    ++diagnostics_.failed_publication_reports;diagnostics_.last_reported_token=t;diagnostics_.last_publication_us=attempted;
    return fail(Fault::PublicationFailed);
}
inline void CanonicalFullInnerConsumption::note_exception()noexcept{fail(Fault::Exception);}
inline void CanonicalFullInnerConsumption::revoke()noexcept{fail(Fault::Revoked);}
} // namespace gpenmpc_full_consumption
