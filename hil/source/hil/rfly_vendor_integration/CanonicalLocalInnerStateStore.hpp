#pragma once
// Fixed storage for the generated canonical numerical state. The scheduler
// validates source health, timing and actuator authority before entry.
// Generated C calls are serialized and initialization is process-owned.
// Candidate state is installed only with a matching publication/commit receipt.
#include <cmath>
#include <cstdint>
#include <cstring>
#include "CanonicalOperatorReference.hpp"
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditStep.h"
#elif defined(GPENMPC_CANONICAL_CLOSED_EVIDENCE)
#include "gpenmpcNative_canonicalLocalInnerWithEvidenceFirst.h"
#include "gpenmpcNative_canonicalLocalInnerWithEvidenceStep.h"
#else // Retained numerical fixtures.
#include "gpenmpcNative_canonicalLocalInnerFixedFirst.h"
#include "gpenmpcNative_canonicalLocalInnerFixedStep.h"
#endif

namespace gpenmpc_local_math {
enum class Failure:std::uint8_t {None,Input,Ordering,PendingCandidate,MissingPrediction,Numerics,Publication,Prediction};
struct PublicationReceipt {
    std::uint64_t source_timestamp_ns{},source_generation{},output_generation{},original_publication_us{};
    float actual_control16[16]{};
    bool publication_succeeded{},numerical_commit_succeeded{};
};
struct Candidate {
    double post_state64[64]{},kernel61[61]{},scaffold70[70]{},request19[19]{};
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
    double closed5[5]{}; // generated CURRENT closed evidence, never scaffold70
#endif
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
    double learning12[12]{};
#endif
    // Exact reference actually consumed by this numerical call, for joint
    // reference/state installation. No extra reference reconstruction.
    double consumed_reference_pvaj[12]{};
    unsigned long long original_tags2[2]{};
    float control16[16]{};
};
class CanonicalLocalInnerStateStore final {
public:
#ifdef GPENMPC_CANONICAL_EXPLICIT_WORKSPACE
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
    using Workspace=e_gpenmpcNative_canonicalLocalIn;
#else
    using Workspace=f_gpenmpcNative_canonicalLocalIn;
#endif
    // Caller allocates one resident workspace, never a per-tick stack local.
    // The same owner's reference query and inner math are serialized; the
    // generated workspace contains unions and must not be used concurrently.
    explicit CanonicalLocalInnerStateStore(Workspace&w)noexcept:workspace_(&w){}
#else
    CanonicalLocalInnerStateStore()noexcept=default;
#endif
    CanonicalLocalInnerStateStore(const CanonicalLocalInnerStateStore&)=delete;
    CanonicalLocalInnerStateStore&operator=(const CanonicalLocalInnerStateStore&)=delete;
    // One instance is one explicitly initialized leg owner. No implicit reset
    // on stale source, failure, GP arrival or ground pause exists here.
    bool prepare(const double input36[36],const unsigned long long tags2[2],bool operator_reference=false)noexcept {
        if(failure_!=Failure::None)return false;
        if(prepared_)return fail(Failure::PendingCandidate);
        if(!input36||!tags2||!finite(input36,36)||!tags2[0]||!tags2[1]||
           !gpenmpc_operator_reference::valid_control_interval_s(input36[34],operator_reference)||input36[31]<0||
           input36[35]<1||!equal(std::floor(input36[35]),input36[35]))return fail(Failure::Input);
        double q2=0;for(unsigned j=6;j<10;++j)q2+=input36[j]*input36[j];
        if(!std::isfinite(q2)||q2<=0)return fail(Failure::Input);
        for(unsigned j=13;j<19;++j)if(input36[j]<0)return fail(Failure::Input);
        if(installed_&&(tags2[0]<=tags_[0]||tags2[1]<=tags_[1]||!equal(input36[35],state_[63])))return fail(Failure::Ordering);
        if(prediction_required_&&!prediction_ready_)return fail(Failure::MissingPrediction);
        candidate_={};
        if(!installed_)
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
            gpenmpcNative_canonicalLocalInnerWithAuditFirst(
#elif defined(GPENMPC_CANONICAL_CLOSED_EVIDENCE)
            gpenmpcNative_canonicalLocalInnerWithEvidenceFirst(
#else
            gpenmpcNative_canonicalLocalInnerFixedFirst(
#endif
#ifdef GPENMPC_CANONICAL_EXPLICIT_WORKSPACE
            workspace_,
#endif
            input36,tags2,candidate_.post_state64,
            candidate_.kernel61,candidate_.scaffold70,candidate_.request19
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
            ,candidate_.closed5
#endif
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
            ,candidate_.learning12
#endif
            );
        else
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
            gpenmpcNative_canonicalLocalInnerWithAuditStep(
#elif defined(GPENMPC_CANONICAL_CLOSED_EVIDENCE)
            gpenmpcNative_canonicalLocalInnerWithEvidenceStep(
#else
            gpenmpcNative_canonicalLocalInnerFixedStep(
#endif
#ifdef GPENMPC_CANONICAL_EXPLICIT_WORKSPACE
            workspace_,
#endif
            state_,tags_,input36,tags2,pending_,tags_,
            candidate_.post_state64,candidate_.kernel61,candidate_.scaffold70,candidate_.request19
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
            ,candidate_.closed5
#endif
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
            ,candidate_.learning12
#endif
            );
        ++kernel_calls_;
        // Pending diagnostics may contain NaN when unavailable; preserve that state.
        const bool required=equal(candidate_.request19[0],1);
        if(!finite(candidate_.post_state64,64)||!finite(candidate_.kernel61,61)||
           (!required&&!equal(candidate_.request19[0],0))||
           (required&&!finite(candidate_.request19,19))||!equal(candidate_.scaffold70[1],0)||
           !equal(candidate_.scaffold70[44],0)||!equal(candidate_.scaffold70[45],0))return fail(Failure::Numerics);
        // The original first-sample early return emits required=false and
        // explicit NaN feature/scale placeholders. They are not GP inputs.
        const unsigned map[6]={4,0,3,5,1,2};const double upper=32.145727009134916;
        for(unsigned j=0;j<6;++j){
            const double force=candidate_.kernel61[4+j];
            if(force<0||force>upper)return fail(Failure::Numerics);
            candidate_.control16[map[j]]=static_cast<float>(force/upper);
        }
        std::memcpy(candidate_.original_tags2,tags2,sizeof candidate_.original_tags2);
        std::memcpy(candidate_.consumed_reference_pvaj,input36+19,sizeof candidate_.consumed_reference_pvaj);
        prepared_=true;return true;
    }
    const Candidate *candidate()const noexcept{return prepared_&&failure_==Failure::None?&candidate_:nullptr;}
    bool validate_publication(const PublicationReceipt&r)const noexcept {
        return failure_==Failure::None&&prepared_&&r.publication_succeeded&&r.numerical_commit_succeeded&&
            r.source_timestamp_ns==candidate_.original_tags2[0]&&r.source_generation==candidate_.original_tags2[1]&&
            r.output_generation>last_output_generation_&&r.original_publication_us&&
            r.original_publication_us<=UINT64_MAX/1000&&r.original_publication_us*1000>=r.source_timestamp_ns&&
            std::memcmp(r.actual_control16,candidate_.control16,sizeof r.actual_control16)==0;
    }
    void reject_joint_publication()noexcept{(void)fail(Failure::Publication);}
    bool install_after_publication(const PublicationReceipt&r)noexcept {
        if(failure_!=Failure::None)return false;
        ++install_attempts_;
        if(!validate_publication(r))return fail(Failure::Publication);
        std::memcpy(state_,candidate_.post_state64,sizeof state_);
        std::memcpy(pending_,candidate_.scaffold70,sizeof pending_);
        std::memcpy(tags_,candidate_.original_tags2,sizeof tags_);
        std::memcpy(request_,candidate_.request19,sizeof request_);
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
        std::memcpy(closed_,candidate_.closed5,sizeof closed_);
#endif
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
        std::memcpy(learning_,candidate_.learning12,sizeof learning_);
#endif
        prediction_required_=!equal(request_[0],0);prediction_ready_=!prediction_required_;
        last_output_generation_=r.output_generation;installed_=true;prepared_=false;++installed_count_;return true;
    }
    // This numerical receipt is already request/model/expiry-bound by the
    // production scheduler.
    // An explicit hard-invalid original GP result is retained, unlike a
    // missing/busy/rejected GP transport result which must never call this.
    bool fill_open_prediction(const unsigned long long tags2[2],const double result18[18])noexcept {
        if(failure_!=Failure::None)return false;
        if(!installed_||prepared_||!prediction_required_||prediction_ready_||!tags2||!result18||
           std::memcmp(tags2,tags_,sizeof tags_)!=0||!equal(pending_[1],0)||!equal(pending_[44],0)||!equal(pending_[45],0))
            return fail(Failure::Prediction);
        pending_[1]=1;std::memcpy(pending_+9,request_+1,17*sizeof(double));
        std::memcpy(pending_+35,result18,3*sizeof(double));std::memcpy(pending_+41,result18+6,3*sizeof(double));
        pending_[5]=result18[13];pending_[7]=result18[12];
        pending_[8]=::fmax(::fmax(result18[9],result18[10]),result18[11]);
        pending_[4]=static_cast<double>(!equal(result18[14],0));
        for(unsigned j=0;j<3;++j)pending_[38+j]=(request_[18]*pending_[5])*pending_[35+j];
        prediction_ready_=true;++prediction_fills_;return true; // no kernel/observer/phase call
    }
    const double *installed_state64()const noexcept{return installed_?state_:nullptr;}
    const double *open_pending70()const noexcept{return installed_?pending_:nullptr;}
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
    const double *installed_closed5()const noexcept{return installed_?closed_:nullptr;}
#endif
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
    const double *installed_learning12()const noexcept{return installed_?learning_:nullptr;}
#endif
    const double *gp_request19()const noexcept{return installed_&&prediction_required_?request_:nullptr;}
    bool prediction_required()const noexcept{return prediction_required_;}
    bool prediction_ready()const noexcept{return prediction_ready_;}
    Failure failure()const noexcept{return failure_;}
    std::uint64_t kernel_calls()const noexcept{return kernel_calls_;}
    std::uint64_t install_attempts()const noexcept{return install_attempts_;}
    std::uint64_t installed_count()const noexcept{return installed_count_;}
    std::uint64_t prediction_fills()const noexcept{return prediction_fills_;}
private:
    static bool equal(double a,double b)noexcept{return a<=b&&a>=b;}
    static bool finite(const double*p,unsigned n)noexcept{for(unsigned j=0;j<n;++j)if(!std::isfinite(p[j]))return false;return true;}
    bool fail(Failure f)noexcept{failure_=f;prepared_=false;return false;}
    Candidate candidate_{};double state_[64]{},pending_[70]{},request_[19]{};unsigned long long tags_[2]{};
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
    double closed_[5]{};
#endif
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
    double learning_[12]{};
#endif
#ifdef GPENMPC_CANONICAL_EXPLICIT_WORKSPACE
    Workspace*const workspace_;
#endif
    std::uint64_t last_output_generation_{},kernel_calls_{},install_attempts_{},installed_count_{},prediction_fills_{};
    Failure failure_{Failure::None};bool installed_{},prepared_{},prediction_required_{},prediction_ready_{};
};
} // namespace gpenmpc_local_math
