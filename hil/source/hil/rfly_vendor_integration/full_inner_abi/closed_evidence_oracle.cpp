// Host ABI commit boundary test calling generated C directly.

#include "../CanonicalCombinedSymbolNamespace.h"
#include "CanonicalFullInnerAbi.h"
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditStep.h"
#else
#include "gpenmpcNative_canonicalLocalInnerWithEvidenceFirst.h"
#include "gpenmpcNative_canonicalLocalInnerWithEvidenceStep.h"
#endif
extern "C" void rfi_direct_closed_oracle(const gpenmpc_full_inner_state *before,
    const double input[36],const uint64_t tags[2],double closed[5],double learning[12]) {
    static e_gpenmpcNative_canonicalLocalIn workspace{};
    double s[64],y[61],pending[70],request[19];
    const unsigned long long t[2]={tags[0],tags[1]},p[2]={before->original_installed_tags2[0],before->original_installed_tags2[1]};
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
    if(!before->numeric_installed)
        gpenmpcNative_canonicalLocalInnerWithAuditFirst(&workspace,input,t,s,y,pending,request,closed,learning);
    else gpenmpcNative_canonicalLocalInnerWithAuditStep(&workspace,before->state64,p,input,t,before->pending70,p,s,y,pending,request,closed,learning);
#else
    (void)learning;
    if(!before->numeric_installed)
        gpenmpcNative_canonicalLocalInnerWithEvidenceFirst(&workspace,input,t,s,y,pending,request,closed);
    else gpenmpcNative_canonicalLocalInnerWithEvidenceStep(&workspace,before->state64,p,input,t,before->pending70,p,s,y,pending,request,closed);
#endif
}
