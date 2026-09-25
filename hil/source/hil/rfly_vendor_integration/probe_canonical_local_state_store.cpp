// Actual target header smoke only. The store is caller-owned, never placed
// on this probe's stack or initialized as a shadow runtime singleton.
#include "CanonicalLocalInnerStateStore.hpp"
using Store=gpenmpc_local_math::CanonicalLocalInnerStateStore;
using Receipt=gpenmpc_local_math::PublicationReceipt;
extern "C" {
__attribute__((used)) const unsigned char gpenmpc_size_local_store[sizeof(Store)]={};
__attribute__((used)) const unsigned char gpenmpc_size_local_candidate[sizeof(gpenmpc_local_math::Candidate)]={};
__attribute__((used)) const unsigned char gpenmpc_size_local_receipt[sizeof(Receipt)]={};
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
