// Forced-include namespace maps only this TU's generated runtime symbols.
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_rtwutil.h"
#include "symbol_coexistence_probe.h"
#include <string.h>
void gpenmpc_new_symbol_probe(GPENMPCCoexistenceProbe *out){
    memset(out,0,sizeof(*out));
    memcpy(&out->nan_bits,&rtNaN,8);memcpy(&out->nan_float_bits,&rtNaNF,4);
    const double positive=rtGetInf(),negative=rtGetMinusInf();
    memcpy(&out->inf_bits,&positive,8);memcpy(&out->minus_inf_bits,&negative,8);
    out->nan_address=(uintptr_t)&rtNaN;out->pow_function_address=(uintptr_t)&rt_powd_snf;
    out->nan_recognized=rtIsNaN(rtNaN);out->inf_recognized=rtIsInf(positive)&&rtIsInfF(rtGetInfF())&&rtIsInfF(rtGetMinusInfF());
    out->finite_pow=rt_powd_snf(2.0,3.0);const double p=rt_powd_snf(rtNaN,2.0);memcpy(&out->pow_nan_bits,&p,8);
}
