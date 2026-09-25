#pragma once
// Single-task numerical handoff of the installed pending query.
// Streams copy bytes through their quiescent outbox.
#include "CanonicalLocalExecutionCycle.hpp"
#include "../px4_wire/CanonicalLocalGpWire.hpp"
namespace gpenmpc_rfly_px4 {
enum class LocalGpFault:std::uint8_t {None,Pending,Feedback,Numeric,Wire,Clock,Stopped};
enum class LocalGpBegin:std::uint8_t {Empty,NotRequired,RequestReady,Rejected};
struct LocalGpDiagnostics {
    LocalGpFault first_fault{LocalGpFault::None};
    std::uint64_t actual_feedback_reads{},requests{},reply_attempts{},fill_attempts{},fills{},
        deadline_unavailable{},busy_unavailable{},late_replies_not_installed{},
        last_original_arrival_us{},last_processing_us{},
        retained_reply_original_arrival_us{},retained_reply_processing_us{};
};
class CanonicalLocalGpPending final {
public:
    explicit CanonicalLocalGpPending(CanonicalLocalExecutionCycle&cycle)noexcept:cycle_(cycle){}
    CanonicalLocalGpPending(const CanonicalLocalGpPending&)=delete;
    CanonicalLocalGpPending&operator=(const CanonicalLocalGpPending&)=delete;
    LocalGpBegin observe_actual_commit()noexcept{
        if(d_.first_fault!=LocalGpFault::None)return LocalGpBegin::Rejected;
        if(pending_&&!numerical_closed_){fail(LocalGpFault::Pending);return LocalGpBegin::Rejected;}
        LocalCycleFeedback actual{};
        if(!cycle_.take_feedback(actual))return LocalGpBegin::Empty;
        retained_=actual;++d_.actual_feedback_reads;
        if(!actual.original_cycle_read_us||actual.original_cycle_read_us<d_.last_processing_us){
            fail(LocalGpFault::Clock);return LocalGpBegin::Rejected;
        }
        const auto&a=actual.actual;const auto&t=a.token;const auto&e=t.lease_envelope;
        if(!actual.present||!actual.usable_at_read||!actual.phase_installed||(!cycle_.runtime_state_only()&&!actual.source_receipt_retired)||
           !a.publication_attempted||!a.publication_succeeded||!a.consumption_committed||
           !a.numerical_reference_installed||!a.reference_state_copied||!a.authority_confirmed||
           e.timestamp_sample_us>UINT64_MAX/1000||
           a.committed_reference.source_timestamp_ns!=e.timestamp_sample_us*1000||
           a.committed_reference.source_generation!=e.sample_generation||
           a.committed_reference.output_generation!=e.output_generation){fail(LocalGpFault::Feedback);return LocalGpBegin::Rejected;}
        gpenmpc_full_inner_diagnostics numerical{};
        if(!cycle_.numerical_diagnostics(numerical)||numerical.abi_failure||numerical.numeric_failure||
           numerical.reference_failure||numerical.joint_failure||numerical.partial_installs||numerical.control_authority){
            fail(LocalGpFault::Numeric);return LocalGpBegin::Rejected;}
        if(!numerical.prediction_required){
            if(!(a.request19[0]>=0.0&&a.request19[0]<=0.0)){fail(LocalGpFault::Feedback);return LocalGpBegin::Rejected;}
            return LocalGpBegin::NotRequired;
        }
        if(numerical.prediction_ready){fail(LocalGpFault::Pending);return LocalGpBegin::Rejected;}
        if(cycle_.component_initialization()){
            // SE(3)-only input uses the hard-invalid, zero-trust learning path.
            double unavailable[18]{};unavailable[14]=1.0;
            const std::uint64_t tags[2]={e.timestamp_sample_us*1000,e.sample_generation};
            if(!cycle_.fill_gp(tags,unavailable)){fail(LocalGpFault::Numeric);return LocalGpBegin::Rejected;}
            return LocalGpBegin::NotRequired;
        }
        if(pending_){
            // While HOST inference is pending, use the unavailable-prediction path.
            if(!fill_unavailable(e.timestamp_sample_us*1000,e.sample_generation))return LocalGpBegin::Rejected;
            ++d_.busy_unavailable;return LocalGpBegin::NotRequired;
        }
        request_={};request_.identity=e.identity;request_.source_timestamp_ns=e.timestamp_sample_us*1000;
        request_.source_generation=e.sample_generation;request_.output_generation=e.output_generation;
        request_.original_publication_us=a.original_publication_us;
        request_.publication_valid_until_us=a.original_valid_until_us;
        request_.configuration_sha256=t.configuration_sha256;
        request_.gp_model_sha256=gpenmpc_local_gp_wire::canonical_model();
        std::memcpy(request_.request19,a.request19,sizeof request_.request19);
        if(!gpenmpc_local_gp_wire::encode(request_,bytes_)){fail(LocalGpFault::Wire);return LocalGpBegin::Rejected;}
        request_read_us_=actual.original_cycle_read_us;
        numerical_closed_=false;pending_=true;++d_.requests;return LocalGpBegin::RequestReady;
    }
    const gpenmpc_local_gp_wire::RequestBytes*pending_bytes()const noexcept{
        return pending_&&d_.first_fault==LocalGpFault::None?&bytes_:nullptr;
    }
    // Historical bytes remain inspectable after fill/fault/stop; neither is
    // a live outbox or a freshness/permission grant.
    const gpenmpc_local_gp_wire::RequestBytes*retained_request_bytes()const noexcept{
        return d_.requests?&bytes_:nullptr;
    }
    const gpenmpc_local_gp_wire::ReplyBytes*retained_reply_bytes()const noexcept{
        return have_reply_bytes_?&reply_bytes_:nullptr;
    }
    // The per-link dispatcher validates assembly, sequence and owner before
    // passing a complete reply. LocalIo and Phase validate source dt and age.
    bool accept_reply(const gpenmpc_local_gp_wire::ReplyBytes&b,
        std::uint64_t original_arrival_us,std::uint64_t processing_us)noexcept{
        if(d_.first_fault!=LocalGpFault::None)return false;
        ++d_.reply_attempts;
        reply_bytes_=b;have_reply_bytes_=true;
        d_.retained_reply_original_arrival_us=original_arrival_us;d_.retained_reply_processing_us=processing_us;
        if(!pending_)return fail(LocalGpFault::Pending);
        if(!original_arrival_us||original_arrival_us<request_.original_publication_us||
           original_arrival_us<request_read_us_||
           processing_us<original_arrival_us||processing_us<d_.last_processing_us||
           original_arrival_us<d_.last_original_arrival_us)return fail(LocalGpFault::Clock);
        d_.last_original_arrival_us=original_arrival_us;d_.last_processing_us=processing_us;
        if(!gpenmpc_local_gp_wire::decode(b,reply_)||!gpenmpc_local_gp_wire::matches(request_,reply_))return fail(LocalGpFault::Wire);
        if(cycle_.runtime_state_only()&&!service_control_deadline(processing_us))return false;
        if(numerical_closed_){
            ++d_.late_replies_not_installed;pending_=false;return true;
        }
        const std::uint64_t tags[2]={reply_.source_timestamp_ns,reply_.source_generation};
        ++d_.fill_attempts;
        if(!cycle_.fill_gp(tags,reply_.result18))return fail(LocalGpFault::Numeric);
        ++d_.fills;pending_=false;return true; // no capture, reference or control step
    }
    void stop()noexcept{fail(LocalGpFault::Stopped);}
    bool retire_unanswered(std::uint64_t generation,std::uint64_t now)noexcept{
        if(!cycle_.runtime_state_only()||!pending_||generation!=request_.output_generation||
           !service_control_deadline(now)||!numerical_closed_)return fail(LocalGpFault::Pending);
        // The original unavailable result was installed once at 10 ms.
        // Transport retirement grants no prediction and performs no fill.
        pending_=false;return true;
    }
    bool pending()const noexcept{return pending_;}
    bool numerical_pending()const noexcept{return pending_&&!numerical_closed_;}
    bool service_control_deadline(std::uint64_t now)noexcept{
        if(!numerical_pending())return d_.first_fault==LocalGpFault::None;
        if(now<request_.original_publication_us)return fail(LocalGpFault::Clock);
        // The next control opportunity defines the numerical dependency deadline.
        // Retain the wire request until reply or transport timeout.
        if(now-request_.original_publication_us<gpenmpc_consumption::canonical_dt_us)return true;
        if(!fill_unavailable(request_.source_timestamp_ns,request_.source_generation))return false;
        numerical_closed_=true;++d_.deadline_unavailable;return true;
    }
    const LocalCycleFeedback&retained_actual_feedback()const noexcept{return retained_;}
    const LocalGpDiagnostics&diagnostics()const noexcept{return d_;}
private:
    bool fill_unavailable(std::uint64_t timestamp,std::uint64_t generation)noexcept{
        double unavailable[18]{};unavailable[14]=1.0;
        const std::uint64_t tags[2]={timestamp,generation};
        return cycle_.fill_gp(tags,unavailable)||fail(LocalGpFault::Numeric);
    }
    bool fail(LocalGpFault f)noexcept{if(d_.first_fault==LocalGpFault::None)d_.first_fault=f;cycle_.stop();return false;}
    CanonicalLocalExecutionCycle&cycle_;
    gpenmpc_local_gp_wire::Request request_{};gpenmpc_local_gp_wire::Reply reply_{};
    gpenmpc_local_gp_wire::RequestBytes bytes_{};gpenmpc_local_gp_wire::ReplyBytes reply_bytes_{};
    LocalCycleFeedback retained_{};
    LocalGpDiagnostics d_{};bool pending_{},have_reply_bytes_{},numerical_closed_{};
    std::uint64_t request_read_us_{};
};
} // namespace gpenmpc_rfly_px4
