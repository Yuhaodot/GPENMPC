#include "../px4_runtime/CanonicalLocalExecutionCycle.hpp"
// Size-only object, never linked into an application or executed.
extern "C" {
unsigned char gpenmpc_size_cycle[sizeof(gpenmpc_rfly_px4::CanonicalLocalExecutionCycle)];
unsigned char gpenmpc_size_cycle_feedback[sizeof(gpenmpc_rfly_px4::LocalCycleFeedback)];
unsigned char gpenmpc_size_phase_clock[sizeof(gpenmpc_local_phase::CanonicalLocalPhaseClock)];
unsigned char gpenmpc_size_local_io[sizeof(gpenmpc_rfly_px4::Px4CanonicalLocalIo)];
unsigned char gpenmpc_size_endpoint_reader[sizeof(gpenmpc_hil_endpoint_reader::Px4OriginalHilReceiptReader)];
unsigned char gpenmpc_size_retained_snapshot[sizeof(gpenmpc_odometry::Snapshot)];
}
