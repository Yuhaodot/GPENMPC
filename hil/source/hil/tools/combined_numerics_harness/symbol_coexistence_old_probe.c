// Use the Simulink runtime namespace.
#include "GPENMPC_Rfly_Canonical_Controller.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "symbol_coexistence_probe.h"
#include <string.h>
// Actual independently compiled parent generated rtrpdc_cg.c definition.
extern double rt_powd_snf(double,double);
void gpenmpc_old_symbol_probe(GPENMPCCoexistenceProbe *out){
    memset(out,0,sizeof(*out));
    memcpy(&out->nan_bits,&rtNaN,8);memcpy(&out->nan_float_bits,&rtNaNF,4);
    const double positive=rtGetInf(),negative=rtGetMinusInf();
    memcpy(&out->inf_bits,&positive,8);memcpy(&out->minus_inf_bits,&negative,8);
    out->nan_address=(uintptr_t)&rtNaN;out->pow_function_address=(uintptr_t)&rt_powd_snf;
    out->nan_recognized=rtIsNaN(rtNaN);out->inf_recognized=rtIsInf(positive)&&rtIsInfF(rtGetInfF())&&rtIsInfF(rtGetMinusInfF());
    out->finite_pow=rt_powd_snf(2.0,3.0);const double p=rt_powd_snf(rtNaN,2.0);memcpy(&out->pow_nan_bits,&p,8);
    memset(&GPENMPC_Rfly_Canonical_Control_U,0,sizeof(GPENMPC_Rfly_Canonical_Control_U));
    GPENMPC_Rfly_Canonical_Controller_step();
    int zero=!GPENMPC_Rfly_Canonical_Control_Y.OutputValid;
    for(unsigned j=0;j<16;++j)zero=zero&&(GPENMPC_Rfly_Canonical_Control_Y.Controls16[j]==0);
    out->old_generated_rejected_output_is_zero=zero;
}
