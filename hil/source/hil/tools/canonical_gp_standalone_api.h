#ifndef GPENMPC_CANONICAL_GP_STANDALONE_API_H
#define GPENMPC_CANONICAL_GP_STANDALONE_API_H
#include <stdint.h>
#if defined(GPENMPC_GP256_EXPORTS)
#define GPENMPC_GP256_API __declspec(dllexport)
#else
#define GPENMPC_GP256_API
#endif
#ifdef __cplusplus
extern "C" {
#endif
enum { GPENMPC_GP256_OK=0, GPENMPC_GP256_BUSY=1, GPENMPC_GP256_ARGUMENT=2 };
typedef struct {
    uint64_t attempts, successes, busy_rejections, invalid_arguments;
} GPENMPCGp256Stats;
/* Use output18 only on OK. BUSY returns immediately with NaN output.
 * The caller owns causal association, age checks and method fallback.
 * Generated static scratch is accessed serially. */
GPENMPC_GP256_API int gpenmpc_gp256_predict(const double input17[17],double output18[18]);
GPENMPC_GP256_API int gpenmpc_gp256_stats(GPENMPCGp256Stats *output);
#ifdef __cplusplus
}
#endif
#endif
