#include "GPENMPCRflyCanonicalLocalModule.hpp"
#include <cerrno>
#include <px4_platform_common/posix.h>

namespace gpenmpc_rfly_local_px4 {
ModuleContext *GPENMPCRflyCanonicalLocalModule::configured_context_{nullptr};
ModuleConfiguration GPENMPCRflyCanonicalLocalModule::frozen_configuration_{};
px4::atomic<std::uint8_t> GPENMPCRflyCanonicalLocalModule::phase_{static_cast<std::uint8_t>(ModulePhase::Stopped)};
px4::atomic<std::uint8_t> GPENMPCRflyCanonicalLocalModule::stop_reason_{static_cast<std::uint8_t>(ModuleStopReason::None)};
px4::atomic<std::uint8_t> GPENMPCRflyCanonicalLocalModule::detach_{static_cast<std::uint8_t>(ModuleDetach::Unavailable)};
px4::atomic<std::uint8_t> GPENMPCRflyCanonicalLocalModule::plant_{static_cast<std::uint8_t>(ModulePlantDisposition::Unknown)};
px4::atomic<std::uint8_t> GPENMPCRflyCanonicalLocalModule::task_created_{0};

GPENMPCRflyCanonicalLocalModule::GPENMPCRflyCanonicalLocalModule(ModuleContext &context,
    const ModuleConfiguration &configuration) noexcept:
    context_(context), configuration_(configuration),
    io_(configuration.source, configuration.execution,
        configuration.telemetry_max_age_us, configuration.authority,
        configuration.commander_telemetry_max_age_us,20000)
{}

bool GPENMPCRflyCanonicalLocalModule::configure(ModuleContext *context) noexcept
{
    if(pthread_mutex_lock(&px4_modules_mutex)!=0)return false;
    const bool stopped=!is_running() && _object.load()==nullptr;
    if(stopped)configured_context_=context;
    const bool unlocked=pthread_mutex_unlock(&px4_modules_mutex)==0;
    return stopped && unlocked;
}

bool GPENMPCRflyCanonicalLocalModule::valid_configuration(const ModuleConfiguration &configuration) noexcept
{
    // The application supplies a dedicated task stack larger than 2240 bytes
    // and validates its high-water usage on target.
    return configuration.authority && configuration.telemetry_max_age_us &&
        configuration.commander_telemetry_max_age_us>=configuration.telemetry_max_age_us &&
        configuration.poll_period_us && configuration.task_stack_bytes>2240 &&
        configuration.task_priority>=SCHED_PRIORITY_MIN &&
        configuration.task_priority<=SCHED_PRIORITY_MAX &&
        configuration.source.identity==configuration.execution.identity;
}

int GPENMPCRflyCanonicalLocalModule::task_spawn(int argc, char *argv[])
{
    task_created_.store(0);
    // ModuleBase::start_command_base serializes this with configure/stop/status.
    if(argc!=1 || !argv || !argv[0] || std::strcmp(argv[0],"start")!=0)return -EINVAL;
    if(!configured_context_){PX4_ERR("application context unavailable");return -ENODEV;}
    frozen_configuration_={};
    const auto acquired=configured_context_->acquire(frozen_configuration_);
    if(acquired!=ModuleAcquire::Ready){PX4_ERR("session/evidence unavailable or rejected (%u)",
        static_cast<unsigned>(acquired));return -EACCES;}
    if(!valid_configuration(frozen_configuration_)){
        (void)configured_context_->abort_acquire();
        PX4_ERR("invalid explicit task/configuration resources");return -EINVAL;
    }
    // PX4's -fno-exceptions NuttX runtime exposes its checked plain-new API,
    // not std::nothrow (absent from this target's actual C++ header).
    auto *candidate=new GPENMPCRflyCanonicalLocalModule(*configured_context_,frozen_configuration_);
    if(!candidate){(void)configured_context_->abort_acquire();return -ENOMEM;}
    if(candidate->io_.diagnostics().first_fault!=LocalFault::None){delete candidate;(void)configured_context_->abort_acquire();return -EINVAL;}
    stop_reason_.store(static_cast<std::uint8_t>(ModuleStopReason::None));
    detach_.store(static_cast<std::uint8_t>(ModuleDetach::Unavailable));
    plant_.store(static_cast<std::uint8_t>(ModulePlantDisposition::Unknown));
    phase_.store(static_cast<std::uint8_t>(ModulePhase::Starting));
    // One real ModuleBase slot, installed before spawn so a stop arriving
    // during startup can request cooperative exit without inventing a slot.
    _object.store(candidate);
    _task_id=px4_task_spawn_cmd("gpenmpc_rfly_canonical_local",SCHED_DEFAULT,
        frozen_configuration_.task_priority,frozen_configuration_.task_stack_bytes,
        &run_trampoline,argv);
    if(_task_id<0){
        const int spawn_errno=errno;
        _task_id=-1;_object.store(nullptr);delete candidate;
        (void)configured_context_->abort_acquire();
        phase_.store(static_cast<std::uint8_t>(ModulePhase::Stopped));
        return spawn_errno?-spawn_errno:-EIO;
    }
    task_created_.store(1);
    return 0;
}

GPENMPCRflyCanonicalLocalModule *GPENMPCRflyCanonicalLocalModule::instantiate(int argc, char *argv[])
{
    // The actual ModuleBase trampoline owns run/exit_and_cleanup. The unique
    // heap object was preallocated before spawn, not placed on the task stack.
    (void)argc;(void)argv;
    return _object.load();
}

void GPENMPCRflyCanonicalLocalModule::run()
{
    ModuleStopReason reason=ModuleStopReason::None;
    Identity observed{};
    if(!configuration_.authority->observed_identity(observed) ||
       !(observed==configuration_.execution.identity))reason=ModuleStopReason::IdentityMismatch;
    phase_.store(static_cast<std::uint8_t>(ModulePhase::Running));
    std::uint64_t previous_end=0,previous_work=0,maximum_work=0;
    while(reason==ModuleStopReason::None && !should_exit()){
        const auto begin=hrt_absolute_time();
        const ModulePoll result=context_.poll(io_,begin);
        const auto end=hrt_absolute_time();
        const auto work=end-begin;if(work>maximum_work)maximum_work=work;
        if(io_.diagnostics().first_fault!=LocalFault::None)reason=ModuleStopReason::IoFault;
        else if(result==ModulePoll::Fault)reason=ModuleStopReason::ContextFault;
        if(reason!=ModuleStopReason::None){
            // Existing first-error NSH path only. Distinguish actual task
            // execution cost from wake-up delay without changing scheduling
            // or the original sample/compute/output expiry checks.
            PX4_ERR("LOCAL_SCHED gap=%llu prior_work=%llu fault_work=%llu max_work=%llu",
                (unsigned long long)(previous_end?begin-previous_end:0),
                (unsigned long long)previous_work,(unsigned long long)work,(unsigned long long)maximum_work);
        }
        previous_end=end;previous_work=work;
        if(reason!=ModuleStopReason::None || should_exit())break;
        if(px4_usleep(configuration_.poll_period_us)!=0 && errno!=EINTR)
            reason=ModuleStopReason::SleepFailure;
    }
    if(reason==ModuleStopReason::None)reason=ModuleStopReason::Requested;
    stop_reason_.store(static_cast<std::uint8_t>(reason));
    phase_.store(static_cast<std::uint8_t>(ModulePhase::Closing));
    // No further poll/capture/execute. Revoke immediately, and retain referenced
    // objects until the router has rejected new users and drained old callbacks.
    configuration_.authority->revoke();
    for(;;){
        const ModuleCloseResult result=context_.close(reason,hrt_absolute_time());
        detach_.store(static_cast<std::uint8_t>(result.route));
        plant_.store(static_cast<std::uint8_t>(result.plant));
        if(result.route==ModuleDetach::Detached)break;
        // Keep referenced objects alive until stream callbacks detach.
        // Pending and unavailable states remain visible in status.
        (void)px4_usleep(configuration_.poll_period_us);
    }
    io_.stop();
    phase_.store(static_cast<std::uint8_t>(ModulePhase::Stopped));
    // ModuleBase then destroys Io/module and clears the original task slot.
    // Parent application may release context/authority only after configure(null).
}

int GPENMPCRflyCanonicalLocalModule::cooperative_stop()
{
    if(pthread_mutex_lock(&px4_modules_mutex)!=0)return -EIO;
    auto *instance=_object.load();
    if(instance)instance->request_stop();
    const bool running=is_running();
    (void)pthread_mutex_unlock(&px4_modules_mutex);
    if(running)PX4_INFO("stop requested; status reports route closure, not plant stopping");
    else PX4_INFO("task not running; physical/plant stopping is not established");
    return 0; // Cooperative stop requested.
}

int GPENMPCRflyCanonicalLocalModule::persistent_status()
{
    if(pthread_mutex_lock(&px4_modules_mutex)!=0)return -EIO;
    PX4_INFO("task=%s phase=%u reason=%u route=%u plant_evidence=%u",
        is_running()?"running":"stopped",static_cast<unsigned>(phase_.load()),
        static_cast<unsigned>(stop_reason_.load()),static_cast<unsigned>(detach_.load()),
        static_cast<unsigned>(plant_.load()));
    (void)pthread_mutex_unlock(&px4_modules_mutex);
    return 0;
}

int GPENMPCRflyCanonicalLocalModule::print_status()
{
    // Do not recursively take px4_modules_mutex if a C++ caller uses Base status.
    PX4_INFO("phase=%u reason=%u route=%u plant_evidence=%u",
        static_cast<unsigned>(phase_.load()),static_cast<unsigned>(stop_reason_.load()),
        static_cast<unsigned>(detach_.load()),static_cast<unsigned>(plant_.load()));
    return 0;
}

int GPENMPCRflyCanonicalLocalModule::main(int argc, char *argv[])
{
    if(argc!=2 || !argv || !argv[1])return print_usage("exactly one command required");
    if(std::strcmp(argv[1],"stop")==0)return cooperative_stop();
    if(std::strcmp(argv[1],"status")==0)return persistent_status();
    // Avoid ModuleBase's timeout -> forced task deletion stop branch entirely.
    if(std::strcmp(argv[1],"start")==0)return start_command_base(argc-1,argv+1);
    if(std::strcmp(argv[1],"help")==0 || std::strcmp(argv[1],"-h")==0)return print_usage();
    return print_usage("unrecognized command");
}

int GPENMPCRflyCanonicalLocalModule::custom_command(int argc, char *argv[])
{
    (void)argc;(void)argv;return print_usage("unrecognized command");
}

int GPENMPCRflyCanonicalLocalModule::print_usage(const char *reason)
{
    if(reason)PX4_WARN("%s",reason);
    PRINT_MODULE_USAGE_NAME("gpenmpc_rfly_canonical_local","controller");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start","start the configured session");
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop","request cooperative stop");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status","show task, route and plant status");
    return reason?-EINVAL:0;
}
} // namespace gpenmpc_rfly_local_px4

extern "C" __EXPORT int gpenmpc_rfly_canonical_local_main(int argc, char *argv[])
{
    return gpenmpc_rfly_local_px4::GPENMPCRflyCanonicalLocalModule::main(argc,argv);
}
