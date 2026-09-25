#define GPENMPC_GP256_EXPORTS
#include "canonical_gp_standalone_api.h"
#include "gpenmpcNative_canonicalSparseGpFixedInput.h"
#include <math.h>
#include <stdatomic.h>
#include <string.h>

static atomic_flag owner=ATOMIC_FLAG_INIT;
static _Atomic uint64_t attempts=0,successes=0,busy_rejections=0,invalid_arguments=0;

static void invalidate(double *output)
{
    if(output)for(unsigned i=0;i<18;++i)output[i]=NAN;
}

int gpenmpc_gp256_predict(const double input17[17],double output18[18])
{
    atomic_fetch_add_explicit(&attempts,1,memory_order_relaxed);
    if(!input17 || !output18){
        invalidate(output18);
        atomic_fetch_add_explicit(&invalid_arguments,1,memory_order_relaxed);
        return GPENMPC_GP256_ARGUMENT;
    }
    if(atomic_flag_test_and_set_explicit(&owner,memory_order_acquire)){
        invalidate(output18);
        atomic_fetch_add_explicit(&busy_rejections,1,memory_order_relaxed);
        return GPENMPC_GP256_BUSY;
    }
    /* Copy inputs locally to support input/output aliasing.
     * The predictor defines NaN/Inf feature handling. */
    double input[17];memcpy(input,input17,sizeof(input));
    gpenmpcNative_canonicalSparseGpFixedInput(input,output18);
    atomic_fetch_add_explicit(&successes,1,memory_order_relaxed);
    atomic_flag_clear_explicit(&owner,memory_order_release);
    return GPENMPC_GP256_OK;
}

int gpenmpc_gp256_stats(GPENMPCGp256Stats *output)
{
    if(!output)return GPENMPC_GP256_ARGUMENT;
    /* Counters are individually atomic. Join or stop all workers before
     * reading a coherent terminal snapshot. */
    output->attempts=atomic_load_explicit(&attempts,memory_order_relaxed);
    output->successes=atomic_load_explicit(&successes,memory_order_relaxed);
    output->busy_rejections=atomic_load_explicit(&busy_rejections,memory_order_relaxed);
    output->invalid_arguments=atomic_load_explicit(&invalid_arguments,memory_order_relaxed);
    return GPENMPC_GP256_OK;
}
