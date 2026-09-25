#include "LegacyTrajectoryReadOnly.hpp"
#ifndef GPENMPC_LEGACY_EXECUTOR_NOT_LINKED
#include <modules/gpenmpc_trajectory_exec/GPENMPCTrajectoryExec.hpp>
#endif

namespace gpenmpc_rfly_px4 {
bool legacy_trajectory_stopped_with_module_lock() noexcept
{
#ifdef GPENMPC_LEGACY_EXECUTOR_NOT_LINKED
    // CMake excludes the legacy executable. Builtin and symbol checks confirm
    // the selected module set; legacy parameter definitions remain available.
    return true;
#else
    // Real public ModuleBase state, not gpenmpc_exec_status freshness.
    // Actual task_spawn and exit_and_cleanup clear _object before setting
    // _task_id=-1 under this SAME module mutex. get_instance is protected;
    // neither a stand-in class nor a protected-access bypass is used here.
    return !GPENMPCTrajectoryExec::is_running();
#endif
}
} // namespace gpenmpc_rfly_px4
