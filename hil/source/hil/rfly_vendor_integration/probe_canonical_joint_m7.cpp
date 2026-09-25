// Cortex-M7 compile probe. The caller owns input, refill, receipt and output
// storage and serializes the resident SD between reference and inner calls.
#if !defined(GPENMPC_CANONICAL_EXPLICIT_WORKSPACE) || GPENMPC_CANONICAL_EXPLICIT_WORKSPACE != 1
#error Require the actual combined generated SD ABI
#endif
#include "CanonicalJointStateInstaller.hpp"
using Numeric=gpenmpc_local_math::CanonicalLocalInnerStateStore;
using Reference=gpenmpc_reference_math::CanonicalReferenceStateStore;
using Joint=gpenmpc_joint_math::CanonicalJointStateInstaller;
struct JointOwnerLayout {
    Numeric::Workspace sd;
    Numeric numeric;
    Reference reference;
    Joint joint;
    explicit JointOwnerLayout(const gpenmpc_reference_math::Configuration&c)noexcept:
        sd{},numeric(sd),reference(sd,c),joint(numeric,reference){}
};
// Measurement-only symbols, not allocated runtime owners. The runner excludes
// these .rodata markers when reporting wrapper code/data footprint.
extern "C" {
__attribute__((used)) const unsigned char gpenmpc_size_numeric[sizeof(Numeric)]={};
__attribute__((used)) const unsigned char gpenmpc_size_reference[sizeof(Reference)]={};
__attribute__((used)) const unsigned char gpenmpc_size_joint[sizeof(Joint)]={};
__attribute__((used)) const unsigned char gpenmpc_size_sd[sizeof(Numeric::Workspace)]={};
__attribute__((used)) const unsigned char gpenmpc_size_owner[sizeof(JointOwnerLayout)]={};
__attribute__((used)) const unsigned char gpenmpc_size_window_already_in_reference[sizeof(struct51_T)]={};
__attribute__((used)) const unsigned char gpenmpc_size_numeric_candidate_already_in_store[sizeof(gpenmpc_local_math::Candidate)]={};
__attribute__((used)) const unsigned char gpenmpc_size_ref_candidate_already_in_store[sizeof(gpenmpc_reference_math::Candidate)]={};
__attribute__((used)) const unsigned char gpenmpc_size_numeric_receipt[sizeof(gpenmpc_local_math::PublicationReceipt)]={};
__attribute__((used)) const unsigned char gpenmpc_size_reference_receipt[sizeof(gpenmpc_reference_math::PublicationReceipt)]={};
__attribute__((used)) const unsigned char gpenmpc_size_backend_event[sizeof(gpenmpc_joint_math::BackendPublication)]={};

bool gpenmpc_joint_probe_load_window(Reference*r,const struct51_T*w)noexcept {
    return r&&w&&r->load_window(*w);
}
bool gpenmpc_joint_probe_prepare(Reference*r,Numeric*n,const gpenmpc_reference_math::Input*i,
                               double caller_input36[36])noexcept {
    if(!r||!n||!i||!caller_input36||r->prepare(*i)!=gpenmpc_reference_math::PrepareResult::Candidate)return false;
    const auto*c=r->candidate();if(!c)return false;
    const auto&jet=c->transition.reference;
    std::memcpy(caller_input36+19,jet.position_m,24);
    std::memcpy(caller_input36+22,jet.velocity_mps,24);
    std::memcpy(caller_input36+25,jet.acceleration_mps2,24);
    std::memcpy(caller_input36+28,jet.jerk_mps3,24);
    const unsigned long long tags[2]={i->source_timestamp_ns,i->source_generation};
    return n->prepare(caller_input36,tags);
}
bool gpenmpc_joint_probe_validate(const Joint*j,const gpenmpc_joint_math::BackendPublication*e,
                                const gpenmpc_local_math::PublicationReceipt*n,const gpenmpc_reference_math::PublicationReceipt*r)noexcept {
    return j&&e&&n&&r&&j->validate_publication(*e,*n,*r);
}
bool gpenmpc_joint_probe_install(Joint*j,const gpenmpc_joint_math::BackendPublication*e,
                               const gpenmpc_local_math::PublicationReceipt*n,const gpenmpc_reference_math::PublicationReceipt*r)noexcept {
    return j&&e&&n&&r&&j->install_after_publication(*e,*n,*r);
}
bool gpenmpc_joint_probe_prediction(Numeric*n,const unsigned long long tags2[2],const double result18[18])noexcept {
    return n&&n->fill_open_prediction(tags2,result18);
}
// Composite call path only: the passed backend receipt is NOT produced here.
// A real publisher and its original timing/health checks remain external.
bool gpenmpc_joint_probe_compose(Reference*r,Numeric*n,Joint*j,const gpenmpc_reference_math::Input*i,
                               double input36[36],const gpenmpc_joint_math::BackendPublication*e,
                               const gpenmpc_local_math::PublicationReceipt*nr,const gpenmpc_reference_math::PublicationReceipt*rr)noexcept {
    return gpenmpc_joint_probe_prepare(r,n,i,input36)&&gpenmpc_joint_probe_install(j,e,nr,rr);
}
}
