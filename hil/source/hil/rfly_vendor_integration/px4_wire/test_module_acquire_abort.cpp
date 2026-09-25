#define GPENMPC_CONTEXT_TEST_HELPERS_ONLY
#include "test_registered_context.cpp"
#include <cstdarg>
static bool fail_next_module_allocation=false;
static unsigned simulated_spawn_calls=0,unexpected_task_actions=0;
extern "C" void *__real__Znwm(std::size_t);
extern "C" void *__wrap__Znwm(std::size_t bytes)
{
    if(fail_next_module_allocation){fail_next_module_allocation=false;return nullptr;}
    return __real__Znwm(bytes);
}
// Explicit platform-effect substitutes only; the production Module and real
// ModuleBase/header/module.cpp lifecycle mutex are compiled, not reshaped.
extern "C" px4_task_t px4_task_spawn_cmd(const char *,int,int,int,px4_main_t,char *const[])
{++simulated_spawn_calls;errno=EAGAIN;return -1;}
extern "C" int px4_task_delete(px4_task_t){++unexpected_task_actions;return -1;}
extern "C" void px4_task_exit(int){++unexpected_task_actions;std::terminate();}
extern "C" void px4_log_modulename(int,const char *,const char *,...){ }
extern "C" void px4_log_raw(int,const char *,...){ }
class AbortContext final:public px::ModuleContext {
public:
    AbortContext(px::RegisteredCanonicalModuleContext &context,unsigned site):context_(context),site_(site){}
    px::ModuleAcquire acquire(px::ModuleConfiguration &out)noexcept override
    {
        const auto result=context_.acquire(out);++acquires;
        if(result==px::ModuleAcquire::Ready){
            if(site_==0)out.task_stack_bytes=2240;
            if(site_==1)fail_next_module_allocation=true;
            if(site_==2)out.source.vehicle_odometry_topic=nullptr;
        }
        return result;
    }
    bool abort_acquire()noexcept override{++aborts;return context_.abort_acquire();}
    px::ModulePoll poll(px::Px4CanonicalIo &,std::uint64_t)noexcept override{++unexpected_task_actions;return px::ModulePoll::Fault;}
    px::ModuleCloseResult close(px::ModuleStopReason,std::uint64_t)noexcept override{++unexpected_task_actions;return {};}
    unsigned acquires{},aborts{};
private:px::RegisteredCanonicalModuleContext &context_;unsigned site_;
};
int main(int argc,char **argv){try{
    if(argc!=3)return 2;const auto f=read_kernel(argv[1]);(void)read_state(argv[2]);
    for(unsigned site=0;site<4;++site){
        context_bus_start();ContextFixture env;auto c=context_config(f[0],env.echo);c.module.task_priority=SCHED_PRIORITY_ATTITUDE_CONTROL;
        px::RegisteredCanonicalModuleContext context(env.guard,env.echo,c);AbortContext adapter(context,site);
        check(px::GPENMPCRflyCanonicalModule::configure(&adapter),"real stopped ModuleBase slot accepts explicit test Context");
        const auto before_spawn=simulated_spawn_calls;char name[]="gpenmpc_rfly_canonical",start[]="start";char *args[]{name,start,nullptr};
        const auto result=px::GPENMPCRflyCanonicalModule::main(2,args);
        const int expected=site==1?-ENOMEM:(site==3?-EAGAIN:-EINVAL);
        check(result==expected,"actual Module start reaches declared resource/allocation/Io/spawn failure");
        check(adapter.acquires==1&&adapter.aborts==1,"actual failure branch invokes synchronous abortAcquire exactly once");
        check(simulated_spawn_calls==before_spawn+(site==3?1:0),"only last case attempts mock platform spawn; no task executes");
        check(context.storage_releasable()&&env.links.native_offboard_allowed()==rs::LinkAccess::Read,"actual failure cleanup leaves no canonical reservation");
        check(!px::GPENMPCRflyCanonicalModule::is_running()&&px::GPENMPCRflyCanonicalModule::configure(nullptr),"actual ModuleBase returns to stopped/configuration detached");
    }
    check(unexpected_task_actions==0,"no task trampoline, forced delete, polling or PX4 task ran");
    std::cout<<"{\"checks\":"<<checks<<",\"failed\":"<<failures<<",\"real_modulebase_sync_start\":true,\"real_production_module_cpp\":true,\"spawn_effect\":\"MOCK_FAILURE_ONLY\",\"allocation_effect\":\"ONE_EXPLICIT_HOST_NEW_FAILURE\",\"platform_tasks_executed\":0}\n";
    return failures?1:0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
