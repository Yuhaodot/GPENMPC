/* Compile generated rtwtypes separately from the MEX host typedefs.
 * The bridge copies binary64 inputs and generated outputs. */
#include "GPENMPC_Rfly_Canonical_Controller.h"
#include <string.h>

int gpenmpc_generated_closure_call(const double *arguments, double *result61,
                                  float *controls16)
{
    _Static_assert(sizeof(real_T) == sizeof(double), "binary64 generated input");
    _Static_assert(sizeof(real32_T) == sizeof(float), "binary32 generated output");
    memcpy(GPENMPC_Rfly_Canonical_Control_U.KernelArguments101, arguments, 101 * sizeof(double));
    GPENMPC_Rfly_Canonical_Control_U.ContinuityEnabled = 1;
    GPENMPC_Rfly_Canonical_Control_U.InputGenerationAccepted = 1;
    GPENMPC_Rfly_Canonical_Controller_step();
    memcpy(result61, GPENMPC_Rfly_Canonical_Control_Y.FullKernel61, 61 * sizeof(double));
    memcpy(controls16, GPENMPC_Rfly_Canonical_Control_Y.Controls16, 16 * sizeof(float));
    return GPENMPC_Rfly_Canonical_Control_Y.OutputValid ? 1 : 0;
}
