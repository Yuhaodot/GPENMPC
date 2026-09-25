#pragma once

#include "Px4CanonicalIo.hpp"
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>

namespace gpenmpc_rfly_px4 {

// Supplied by the application, not CLI UID/permission booleans. No scheduling
// defaults: sleep cadence is NOT a replacement for original source dt checks.
struct ModuleConfiguration {
    gpenmpc_odometry::Configuration source{};
    gpenmpc_rfly_execution::Configuration execution{};
    Authority *authority{nullptr};
    std::uint64_t telemetry_max_age_us{0};
    std::uint32_t poll_period_us{0};
    int task_priority{0};
    int task_stack_bytes{0};
};

enum class ModuleAcquire : std::uint8_t {Ready, Unavailable, Rejected};
enum class ModulePoll : std::uint8_t {Idle, Progress, Fault};
enum class ModulePhase : std::uint8_t {Stopped, Starting, Running, Closing};
enum class ModuleStopReason : std::uint8_t {
    None, Requested, ContextFault, IoFault, IdentityMismatch, SleepFailure
};
enum class ModuleDetach : std::uint8_t {Pending, Unavailable, Detached};
enum class ModulePlantDisposition : std::uint8_t {
    Unknown, NativeLandingRequested, NativeLandingObserved, DisarmedObserved
};
struct ModuleCloseResult {
    ModuleDetach route{ModuleDetach::Unavailable};
    ModulePlantDisposition plant{ModulePlantDisposition::Unknown};
    // Evidence only: never substituted for a control/source timestamp.
    std::uint64_t original_observation_us{0};
};

class ModuleContext {
public:
    virtual ~ModuleContext()=default;
    // Called once per start, while PX4's module lifecycle mutex is held.
    // Must be bounded and must not call a ModuleBase command recursively.
    // Ready requires the actual independently observed identity/physical
    // evidence and frozen session/configuration. Missing evidence -> Unavailable.
    // This phase must NOT register a live stream route or produce an output;
    // failed acquire must leave no resources requiring asynchronous close.
    virtual ModuleAcquire acquire(ModuleConfiguration &out) noexcept=0;
    // Synchronous rollback before task, poll, route or publication starts.
    // Called under px4_modules_mutex after a post-acquire start failure.
    virtual bool abort_acquire() noexcept=0;
    // Called only by the dedicated task. A production implementation drives
    // real ingress/capture/snapshot/execute on this exact persistent Io object.
    // Every method must return; no blocking device command or arm/mode write.
    virtual ModulePoll poll(Px4CanonicalIo &io, std::uint64_t now_us) noexcept=0;
    // First revoke/unbind the shared router, then await in-flight callbacks.
    // Detached means callback quiescence, NOT plant stopping. The context and
    // Authority MUST remain alive through configure(nullptr) after task exit;
    // in particular close() must not delete Io's referenced Authority.
    // Pending/Unavailable keeps this module and Io alive, without another poll.
    // Native-mode/failsafe evidence is a separate typed result, never inferred
    // from stopping a publisher. No automatic mode/arm/parameter command here.
    virtual ModuleCloseResult close(ModuleStopReason reason, std::uint64_t now_us) noexcept=0;
};

class GPENMPCRflyCanonicalModule final : public ModuleBase<GPENMPCRflyCanonicalModule> {
public:
    // Application-owned context must remain alive until configure(nullptr)
    // succeeds after task exit. There is no permission/configuration CLI.
    static bool configure(ModuleContext *context) noexcept;
    static int main(int argc, char *argv[]);
    static int task_spawn(int argc, char *argv[]);
    static GPENMPCRflyCanonicalModule *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason=nullptr);
    int print_status() override;
    void run() override;
    ~GPENMPCRflyCanonicalModule() override=default;

private:
    GPENMPCRflyCanonicalModule(ModuleContext &context, const ModuleConfiguration &configuration) noexcept;
    static int cooperative_stop();
    static int persistent_status();
    static bool valid_configuration(const ModuleConfiguration &configuration) noexcept;
    static ModuleContext *configured_context_;
    static ModuleConfiguration frozen_configuration_;
    // Atomic byte diagnostics avoid unsynchronized status reads of live 64-bit
    // Io counters on Cortex-M7; actual publication counts remain in Io receipts.
    static px4::atomic<std::uint8_t> phase_;
    static px4::atomic<std::uint8_t> stop_reason_;
    static px4::atomic<std::uint8_t> detach_;
    static px4::atomic<std::uint8_t> plant_;

    ModuleContext &context_;
    const ModuleConfiguration configuration_;
    // Allocated as part of this heap-owned module, never on a 2240-byte WorkItem
    // or caller tick stack. Exactly one task owns every Io method invocation.
    Px4CanonicalIo io_;
};

} // namespace gpenmpc_rfly_px4

extern "C" __EXPORT int gpenmpc_rfly_canonical_main(int argc, char *argv[]);
