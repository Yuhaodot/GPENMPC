#ifndef GPENMPC_SYMBOL_COEXISTENCE_PROBE_H
#define GPENMPC_SYMBOL_COEXISTENCE_PROBE_H
// Host-test POD shared without generated Coder headers.
#include <stdint.h>
typedef struct {
    uint64_t nan_bits,inf_bits,minus_inf_bits,pow_nan_bits;
    uint32_t nan_float_bits;
    uintptr_t nan_address,pow_function_address;
    double finite_pow;
    int nan_recognized,inf_recognized,old_generated_rejected_output_is_zero;
} GPENMPCCoexistenceProbe;
#ifdef __cplusplus
extern "C" {
#endif
void gpenmpc_old_symbol_probe(GPENMPCCoexistenceProbe *out);
void gpenmpc_new_symbol_probe(GPENMPCCoexistenceProbe *out);
#ifdef __cplusplus
}
#endif
#endif
