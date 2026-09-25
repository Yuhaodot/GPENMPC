// Compiled with the actual PWMOut.cpp command, including its exact PARAM_PREFIX
// and MODULE_NAME, so the included real class definition is not macro-relabelled.
#include <drivers/pwm_out/PWMOut.hpp>

// Do not instantiate a private default (-1) task slot in this adapter. Resolve
// the real driver's ModuleBase storage at final link; an absent driver must not
// become synthetic evidence of "stopped" merely because this TU compiled.
extern template class ModuleBase<PWMOut>;

extern "C" bool gpenmpc_readonly_pwm_out_running() noexcept
{
    // Actual public ModuleBase<PWMOut> query. This is a point observation, not
    // proof of physical isolation or exclusion of a future driver start.
    return PWMOut::is_running();
}
