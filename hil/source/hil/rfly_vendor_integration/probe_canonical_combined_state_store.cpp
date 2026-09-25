// Combined actual target smoke; all resident state/workspace remain caller-owned.
// These sizeof symbols are measurement-only objects, never runtime task slots.
#if !defined(GPENMPC_CANONICAL_EXPLICIT_WORKSPACE) || GPENMPC_CANONICAL_EXPLICIT_WORKSPACE != 1
#error This probe requires the combined explicit-workspace ABI
#endif
#include "CanonicalLocalInnerStateStore.hpp"
#include "gpenmpcNative_queryCanonicalReferenceWindow.h"
#include "gpenmpcNative_canonicalReferenceTransitionFromJet.h"
using Store=gpenmpc_local_math::CanonicalLocalInnerStateStore;
using Receipt=gpenmpc_local_math::PublicationReceipt;
extern "C" {
__attribute__((used)) const unsigned char gpenmpc_size_local_store[sizeof(Store)]={};
__attribute__((used)) const unsigned char gpenmpc_size_local_candidate[sizeof(gpenmpc_local_math::Candidate)]={};
__attribute__((used)) const unsigned char gpenmpc_size_local_receipt[sizeof(Receipt)]={};
__attribute__((used)) const unsigned char gpenmpc_size_combined_workspace[sizeof(Store::Workspace)]={};
__attribute__((used)) const unsigned char gpenmpc_size_reference_window[sizeof(struct51_T)]={};
__attribute__((used)) const unsigned char gpenmpc_size_reference_state[sizeof(struct52_T)]={};
__attribute__((used)) const unsigned char gpenmpc_size_reference_request[sizeof(struct53_T)]={};
__attribute__((used)) const unsigned char gpenmpc_size_reference_receipt[sizeof(struct54_T)]={};
bool gpenmpc_probe_local_store_prepare(Store*s,const double input36[36],const unsigned long long tags2[2]) noexcept {
    return s&&s->prepare(input36,tags2);
}
bool gpenmpc_probe_local_store_install(Store*s,const Receipt*r) noexcept {
    return s&&r&&s->install_after_publication(*r);
}
bool gpenmpc_probe_local_store_prediction(Store*s,const unsigned long long tags2[2],const double result18[18]) noexcept {
    return s&&s->fill_open_prediction(tags2,result18);
}
}
