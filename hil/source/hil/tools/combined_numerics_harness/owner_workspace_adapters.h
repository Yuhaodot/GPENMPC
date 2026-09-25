#ifndef GPENMPC_COMBINED_OWNER_WORKSPACE_TEST_ADAPTERS_H
#define GPENMPC_COMBINED_OWNER_WORKSPACE_TEST_ADAPTERS_H
/* Test ABI adapters forward to the combined C entry points using
 * one serial owner and its spill workspace. */
#include "gpenmpcNative_canonicalLocalInnerFixedFirst.h"
#include "gpenmpcNative_canonicalLocalInnerFixedStep.h"
#include "gpenmpcNative_queryCanonicalReferenceWindow.h"
#include "gpenmpcNative_canonicalReferenceTransitionFromJet.h"
static f_gpenmpcNative_canonicalLocalIn owner_workspace;
static inline void owner_first(const double x[36],const unsigned long long tags[2],
    double state[64],double kernel[61],double pending[70],double request[19]) {
  gpenmpcNative_canonicalLocalInnerFixedFirst(&owner_workspace,x,tags,state,kernel,pending,request);
}
static inline void owner_step(const double old[64],const unsigned long long old_tags[2],
    const double x[36],const unsigned long long tags[2],const double pending[70],
    const unsigned long long pending_tags[2],double state[64],double kernel[61],double scaffold[70],double request[19]) {
  gpenmpcNative_canonicalLocalInnerFixedStep(&owner_workspace,old,old_tags,x,tags,pending,pending_tags,state,kernel,scaffold,request);
}
static inline void owner_query(const struct51_T *window,const struct52_T *state,const struct53_T *request,
    struct52_T *next,double jet[12],struct54_T *receipt) {
  gpenmpcNative_queryCanonicalReferenceWindow(&owner_workspace,window,state,request,next,jet,receipt);
}
#define gpenmpcNative_canonicalLocalInnerFixedFirst owner_first
#define gpenmpcNative_canonicalLocalInnerFixedStep owner_step
#define gpenmpcNative_queryCanonicalReferenceWindow owner_query
#endif
