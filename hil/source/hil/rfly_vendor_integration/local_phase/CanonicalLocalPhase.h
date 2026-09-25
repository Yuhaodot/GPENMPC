#ifndef GPENMPC_CANONICAL_LOCAL_PHASE_H
#define GPENMPC_CANONICAL_LOCAL_PHASE_H

/* Plain C ABI for the separately compiled MATLAB-generated TU.
 * Do not include its rtwtypes.h beside another generated library's types.
 * input7 = phase, rate, actual accepted transition acceleration, actual dt,
 *          duration, configured rate minimum, configured rate maximum.
 * next2 = next phase, next rate. No state installation, clock or authority.
 * The owner validates inputs and installs only after its matching receipt.
 */
#ifdef __cplusplus
extern "C" {
#endif
void gpenmpcNative_canonicalLocalPhaseAdvance(const double input7[7],
                                           double next2[2]);
#ifdef __cplusplus
}
#endif
#endif
