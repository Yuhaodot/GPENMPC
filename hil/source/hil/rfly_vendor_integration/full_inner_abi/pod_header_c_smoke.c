#include "CanonicalFullInnerAbi.h"
/* C compiler sees no generated typedef_struct*_T or rt* namespace macro. */
#if defined(typedef_struct_T) || defined(RTW_HEADER_rtwtypes_h_) || defined(rtNaN)
#error Coder types or symbol aliases leaked into the POD boundary
#endif
int gpenmpc_full_inner_c_client(gpenmpc_full_inner_owner *owner,gpenmpc_full_inner_diagnostics *out)
{
    return gpenmpc_full_inner_diagnose(owner,out);
}
