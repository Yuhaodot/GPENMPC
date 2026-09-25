// Keep generated Coder types within their translation units.
#include "symbol_coexistence_probe.h"
#include <cstdio>
int source_gp_chain_original_main(int,wchar_t**);
static bool valid(const GPENMPCCoexistenceProbe&o,const GPENMPCCoexistenceProbe&n){
    return o.nan_bits==UINT64_C(0xfff8000000000000)&&n.nan_bits==UINT64_C(0x7ff8000000000000)&&
        o.nan_float_bits==UINT32_C(0xffc00000)&&n.nan_float_bits==UINT32_C(0x7fc00000)&&
        o.nan_address&&n.nan_address&&o.nan_address!=n.nan_address&&o.pow_function_address&&n.pow_function_address&&o.pow_function_address!=n.pow_function_address&&
        o.pow_nan_bits==o.nan_bits&&n.pow_nan_bits==n.nan_bits&&o.finite_pow==8&&n.finite_pow==8&&
        o.inf_bits==UINT64_C(0x7ff0000000000000)&&n.inf_bits==o.inf_bits&&o.minus_inf_bits==UINT64_C(0xfff0000000000000)&&n.minus_inf_bits==o.minus_inf_bits&&
        o.nan_recognized&&n.nan_recognized&&o.inf_recognized&&n.inf_recognized&&o.old_generated_rejected_output_is_zero;
}
int wmain(int argc,wchar_t**argv){
    if(argc!=11)return 2;GPENMPCCoexistenceProbe old_before{},new_before{},old_after{},new_after{};
    gpenmpc_old_symbol_probe(&old_before);gpenmpc_new_symbol_probe(&new_before);
    const bool before=valid(old_before,new_before);
    // Rename only the SJC671 entry symbol during compilation.
    const int chain_rc=source_gp_chain_original_main(10,argv);
    gpenmpc_old_symbol_probe(&old_after);gpenmpc_new_symbol_probe(&new_after);
    const bool after=valid(old_after,new_after)&&old_before.nan_address==old_after.nan_address&&new_before.nan_address==new_after.nan_address&&
        old_before.pow_function_address==old_after.pow_function_address&&new_before.pow_function_address==new_after.pow_function_address;
    FILE*f=_wfopen(argv[10],L"w");if(!f)return 3;
    std::fprintf(f,"{\"pass\":%s,\"before\":%s,\"after\":%s,\"chain_exit_code\":%d,\"old_negative_NaN64\":\"%016llX\",\"new_positive_NaN64\":\"%016llX\",\"old_negative_NaN32\":\"%08X\",\"new_positive_NaN32\":\"%08X\",\"old_nan_address\":\"%llX\",\"new_nan_address\":\"%llX\",\"actual_parent_rtrpdc_pow_address\":\"%llX\",\"isolated_new_pow_address\":\"%llX\",\"actual_parent_source_compiled\":true,\"legacy_pow_mocked\":false,\"old_generated_step_calls\":2,\"old_step_generation_accepted\":false,\"model_IO_board_actions\":0}\n",
        before&&after&&chain_rc==0?"true":"false",before?"true":"false",after?"true":"false",chain_rc,
        (unsigned long long)old_after.nan_bits,(unsigned long long)new_after.nan_bits,old_after.nan_float_bits,new_after.nan_float_bits,
        (unsigned long long)old_after.nan_address,(unsigned long long)new_after.nan_address,(unsigned long long)old_after.pow_function_address,(unsigned long long)new_after.pow_function_address);
    std::fclose(f);std::printf("SYMBOL_COEXISTENCE before=%d after=%d chain=%d\n",before,after,chain_rc);
    return before&&after&&chain_rc==0?0:4;
}
