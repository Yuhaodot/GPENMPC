#define GPENMPC_CONTEXT_TEST_HELPERS_ONLY
#include "test_registered_context.cpp"
#include "../px4_stream/NativeOffboardBorrow.hpp"

static void reserved_acquire_negatives(const Fixture&f)
{
    for(unsigned mutation=0;mutation<4;++mutation){
        context_bus_start();ContextFixture env;auto c=context_config(f,env.echo);
        simulated_legacy_running=mutation==0;simulated_recovery_configuration_bad=mutation==1;
        if(mutation==2)c.transport.receiver_instance=ORB_MULTI_MAX_INSTANCES;
        rs::NativeBorrowToken entered{};
        if(mutation==3)check(env.links.enter_native_offboard(env.link.pointer,entered)==rs::LinkAccess::Read,"native handler genuinely entered before acquire");
        px::RegisteredCanonicalModuleContext context(env.guard,env.echo,c);px::ModuleConfiguration out{};
        check(context.acquire(out)!=px::ModuleAcquire::Ready&&!out.authority,"legacy running, actual recovery configuration fail, unsupported HB channel, or in-flight native handler rejects acquire");
        if(mutation==3)check(env.links.leave_native_offboard(entered)==rs::LinkAccess::Read,"entered native handler leaves");
        check(context.abort_acquire()&&context.storage_releasable()&&env.links.native_offboard_allowed()==rs::LinkAccess::Read,"failed acquire synchronous cleanup has no OCM reservation leak");
    }
    // The real Module task_spawn calls this same no-task/no-Io release hook at
    // invalid resources, allocation failure, Io construction fault and spawn
    // failure. This loop tests the production hook, not an executed scheduler.
    for(unsigned failure_site=0;failure_site<4;++failure_site){
        context_bus_start();ContextFixture env;px::RegisteredCanonicalModuleContext context(env.guard,env.echo,context_config(f,env.echo));px::ModuleConfiguration out{};
        check(context.acquire(out)==px::ModuleAcquire::Ready&&!context.storage_releasable(),"acquire reserves existing link before simulated spawn failure");
        check(context.abort_acquire()&&context.storage_releasable()&&env.links.native_offboard_allowed()==rs::LinkAccess::Read,"real synchronous abort releases no-task reservation at each simulated failure site");
        check(context.acquire(out)!=px::ModuleAcquire::Ready,"aborted context cannot restart same prestream lifetime");
    }
}
static void heartbeat_waits_for_actual_source(const Fixture&f,const Raw&state)
{
    context_bus_start();ContextFixture env;px::RegisteredCanonicalModuleContext context(env.guard,env.echo,context_config(f,env.echo));px::ModuleConfiguration out{};
    check(context.acquire(out)==px::ModuleAcquire::Ready,"source-event flow acquire");
    px::Px4CanonicalIo io(out.source,out.execution,out.telemetry_max_age_us,out.authority);
    const auto first_hb=actual_test_hrt+1;context_heartbeat(env,first_hb);actual_test_hrt+=2;
    check(context.poll(io,actual_test_hrt)==px::ModulePoll::Idle&&context.diagnostics_after_stop().offboard_publications==0,"HB before any actual source produces zero OCM");
    std::uint64_t sample=1001700;actual_test_hrt=sample+300;context_telemetry(actual_test_hrt-10,0);
    auto odometry=raw(state,sample);gpenmpc_test_topic_publish(ORB_ID(vehicle_odometry),&odometry);
    check(context.poll(io,actual_test_hrt)==px::ModulePoll::Progress&&context.diagnostics_after_stop().offboard_publications==1,"later actual fresh atomic capture permits one OCM");
    check(context.diagnostics_after_stop().last_offboard_original_hrt_us==first_hb&&context.diagnostics_after_stop().last_offboard_original_valid_until_us==first_hb+100000,"deferred OCM retains HB original HRT and original100ms expiry");
    offboard_control_mode_s o{};std::uint32_t g=0;gpenmpc_test_topic_copy(ORB_ID(offboard_control_mode),&o,g,0);
    check(o.timestamp==first_hb&&o.direct_actuator&&!o.position&&!o.velocity&&!o.acceleration&&!o.attitude&&!o.body_rate&&!o.thrust_and_torque,"actual production uORB publication is direct-only and never arms");
    (void)registered_snapshot(env.echo);
    actual_test_hrt=sample+6000;context_telemetry(actual_test_hrt-10,0);
    check(context.poll(io,actual_test_hrt)==px::ModulePoll::Idle&&context.diagnostics_after_stop().offboard_publications==1,"normal source sixth millisecond does not invent a persistent5ms gate");
    const auto second_hb=actual_test_hrt;context_heartbeat(env,second_hb);
    check(context.poll(io,actual_test_hrt)==px::ModulePoll::Idle&&context.diagnostics_after_stop().offboard_publications==1,"new HB waits for next fresh actual source without replacing original HB timestamp");
    sample+=10000;actual_test_hrt=sample+300;context_telemetry(actual_test_hrt-10,0);odometry=raw(state,sample);gpenmpc_test_topic_publish(ORB_ID(vehicle_odometry),&odometry);
    check(context.poll(io,actual_test_hrt)==px::ModulePoll::Progress&&context.diagnostics_after_stop().offboard_publications==2&&context.diagnostics_after_stop().last_offboard_original_hrt_us==second_hb,"new HB receives exactly one fresh-source-backed publication");
    (void)registered_snapshot(env.echo);
    for(unsigned i=0;i<5;++i){sample+=10000;actual_test_hrt=sample+300;context_telemetry(actual_test_hrt-10,0);odometry=raw(state,sample);gpenmpc_test_topic_publish(ORB_ID(vehicle_odometry),&odometry);
        check(context.poll(io,actual_test_hrt)==px::ModulePoll::Progress&&context.diagnostics_after_stop().offboard_publications==2,"many new captures without a new HB create zero OCM credit");(void)registered_snapshot(env.echo);}
    check(io.diagnostics().kernel_calls==0&&io.diagnostics().publish_attempts==0&&io.diagnostics().numerical_commits==0,"all preparation observations remain zero kernel/output/commit");
    check(!context.abort_acquire(),"synchronous no-task abort cannot release actual attached Io");
    actual_test_hrt=second_hb+100001;context_telemetry(actual_test_hrt-10,0);
    check(context.poll(io,actual_test_hrt)==px::ModulePoll::Fault&&context.diagnostics_after_stop().offboard_publications==2,"original HB expiry stops OCM permanently");
    px::SessionGuardEvidence evidence{};check(env.guard.evidence_snapshot(evidence)&&evidence.first_fault==px::SessionFault::None,"HB liveness failure does not poison registered session identity");
    check(env.links.native_offboard_allowed()==rs::LinkAccess::Rejected,"first fault retains global native handler reservation");
    check(context.close(px::ModuleStopReason::ContextFault,actual_test_hrt).route==px::ModuleDetach::Detached&&env.links.native_offboard_allowed()==rs::LinkAccess::Read,"actual disarmed no-control close releases reservation only after detach");
}
static void missing_and_late_source(const Fixture&f,const Raw&state)
{
    for(unsigned late=0;late<2;++late){
        context_bus_start();ContextFixture env;px::RegisteredCanonicalModuleContext context(env.guard,env.echo,context_config(f,env.echo));px::ModuleConfiguration out{};
        check(context.acquire(out)==px::ModuleAcquire::Ready,"missing/late source acquire");px::Px4CanonicalIo io(out.source,out.execution,out.telemetry_max_age_us,out.authority);
        const auto hb=actual_test_hrt+1;context_heartbeat(env,hb);actual_test_hrt+=2;
        check(context.poll(io,actual_test_hrt)==px::ModulePoll::Idle,"HB alone cannot synthesize a source");
        actual_test_hrt=hb+100001;context_telemetry(actual_test_hrt-10,0);
        if(late){auto odometry=raw(state,actual_test_hrt-300);gpenmpc_test_topic_publish(ORB_ID(vehicle_odometry),&odometry);}
        check(context.poll(io,actual_test_hrt)==px::ModulePoll::Fault&&context.diagnostics_after_stop().offboard_publications==0,"absent source or first actual capture later than original HB expiry cannot emit");
        check(io.diagnostics().kernel_calls==0&&io.diagnostics().publish_attempts==0,"missing/late source never executes control");
        check(context.close(px::ModuleStopReason::ContextFault,actual_test_hrt).route==px::ModuleDetach::Detached,"missing/late source disarmed cleanup closes bounded lifetime");
    }
}
static void actual_ocm_publication_crosses_source_admission(const Fixture&f,const Raw&state)
{
    context_bus_start();ContextFixture env;px::RegisteredCanonicalModuleContext context(env.guard,env.echo,context_config(f,env.echo));px::ModuleConfiguration out{};
    check(context.acquire(out)==px::ModuleAcquire::Ready,"crossing source-admission acquire");px::Px4CanonicalIo io(out.source,out.execution,out.telemetry_max_age_us,out.authority);
    const auto hb=actual_test_hrt+1;context_heartbeat(env,hb);actual_test_hrt+=2;
    auto odometry=raw(state,1000000);gpenmpc_test_topic_publish(ORB_ID(vehicle_odometry),&odometry);
    const auto before_generation=bus_generation[3];simulated_ocm_publish_delay_us=5000;
    check(context.poll(io,actual_test_hrt)==px::ModulePoll::Fault,"explicit mock OCM publication latency crosses source's original5ms admission");
    const auto &d=context.diagnostics_after_stop();
    check(d.offboard_publication_attempts==1&&d.offboard_publications==1&&bus_generation[3]==before_generation+1,"already occurred actual publish attempt/success are retained, not zeroed");
    check(d.last_offboard_original_hrt_us==hb&&d.last_offboard_original_valid_until_us==hb+100000&&actual_test_hrt<hb+100000,"source admission failure does not rewrite original100ms HB lifetime");
    check(io.diagnostics().kernel_calls==0&&io.diagnostics().publish_attempts==0&&io.diagnostics().numerical_commits==0,"OCM signal is not actuator control/arm/kernel commit");
    simulated_ocm_publish_delay_us=0;context_heartbeat(env,actual_test_hrt+1);++actual_test_hrt;
    check(context.poll(io,actual_test_hrt)==px::ModulePoll::Fault&&d.offboard_publications==1,"post-publication source expiry permanently revokes this Context");
    check(context.close(px::ModuleStopReason::ContextFault,actual_test_hrt).route==px::ModuleDetach::Detached,"crossing-source disarmed no-actuator cleanup detaches");
}
int main(int argc,char**argv){try{if(argc!=3)return 2;const auto f=read_kernel(argv[1]);const auto r=read_state(argv[2]);
    acquire_negative_tests(f[0]);partial_bind_tests(f[0]);dual_owner_test(f[0],r[0]);
    live_shape_test(f[0],r[0],false);live_shape_test(f[0],r[0],true);
    reserved_acquire_negatives(f[0]);heartbeat_waits_for_actual_source(f[0],r[0]);missing_and_late_source(f[0],r[0]);actual_ocm_publication_crosses_source_admission(f[0],r[0]);
    std::cout<<"{\"checks\":"<<checks<<",\"failed\":"<<failures<<",\"actual_context_cpp\":true,\"actual_kernel_steps_positive\":2,\"module_scheduler_executed\":false,\"scope\":\"PRODUCTION_CONTEXT_REGISTRY_IO_PUMP_ACTUAL_MAVLINK_PACK_PARSE_MOCK_RAWGUARD_UORB_HRT_LEGACY_STATE\"}\n";
    return failures?1:0;}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
