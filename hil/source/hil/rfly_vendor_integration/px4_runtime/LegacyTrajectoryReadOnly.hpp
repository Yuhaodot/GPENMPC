#pragma once

namespace gpenmpc_rfly_px4 {

// Caller MUST already hold the real px4_modules_mutex, as ModuleBase start
// does. No recursive command, stopping action, lock, telemetry approximation or
// cached "stopped" assertion occurs here. The same lock must remain held until
// the canonical reservation is bound; the legacy task_spawn overlay checks the
// reservation under that same module-lifecycle lock.
bool legacy_trajectory_stopped_with_module_lock() noexcept;

} // namespace gpenmpc_rfly_px4
