#include <limits>
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#pragma GCC diagnostic ignored "-Wunused-function"
#define GPENMPC_PUMP_TEST_HELPERS_ONLY
#include "test_exchange_pump.cpp"
#include "../px4_runtime/RegisteredCanonicalModuleContext.hpp"
#include <uORB/topics/system_power.h>
namespace rs=gpenmpc_rfly_stream;
const orb_metadata __orb_vehicle_land_detected{6,sizeof(vehicle_land_detected_s),1};
const orb_metadata __orb_system_power{7,sizeof(system_power_s),1};
bool gpenmpc_test_topic_copy(const orb_metadata*m,void*out,std::uint32_t&generation,std::uint8_t instance)noexcept{
    if(instance||bus[m->id].empty())return false;const auto&r=bus[m->id].back();generation=r.generation;std::memcpy(out,r.bytes.data(),m->size);return true;
}
static bool wrong_observed_uid=false;
static bool simulated_legacy_running=false,simulated_recovery_configuration_bad=false;
class ForeignOutputOwner final:public rs::Authority {
public:rs::Source choose(std::uint64_t,const vehicle_status_s&,const vehicle_control_mode_s&)noexcept override{return rs::Source::Unavailable;}
    bool accept(const rs::Observation&,std::uint64_t,rs::OriginalValidity&out)noexcept override{out={};return false;}
};
static ForeignOutputOwner foreign_output;static unsigned inject_busy_after_observations=0;static rs::LinkToken foreign_link{};static rs::Registration foreign_registration{};
namespace gpenmpc_rfly_px4 {
std::uint64_t Px4SessionClock::now()const noexcept{return hrt_absolute_time();}
bool legacy_trajectory_stopped_with_module_lock()noexcept{return !simulated_legacy_running;}
bool gpenmpc_context_simulated_board_evidence(BoardSafetyEvidence&e)noexcept{
    if(inject_busy_after_observations&&!--inject_busy_after_observations)
        (void)rs::shared_output_registry()->bind(foreign_link,foreign_output,foreign_registration);
    e={};vehicle_status_s s{};vehicle_control_mode_s m{};offboard_control_mode_s o{};system_power_s p{};std::uint32_t gen=0;
    if(!gpenmpc_test_topic_copy(ORB_ID(vehicle_status),&s,gen,0)||!gpenmpc_test_topic_copy(ORB_ID(vehicle_control_mode),&m,gen,0)||
       !gpenmpc_test_topic_copy(ORB_ID(offboard_control_mode),&o,gen,0)||!gpenmpc_test_topic_copy(ORB_ID(system_power),&p,gen,0))return false;
    e.identity.uid=0x1122334455667788ULL+(wrong_observed_uid?1:0);e.identity.system=s.system_id;e.identity.component=s.component_id;
    e.uid=e.mavlink_identity=e.usb_transport=e.hil_configuration=GuardFact::Pass;
    e.native_recovery_configuration=simulated_recovery_configuration_bad?GuardFact::Fail:GuardFact::Pass;
    e.pwm_functions_zero=e.pwm_out_stopped=e.io_driver_stopped=e.dshot_stopped=e.usb_power_observed=GuardFact::Pass;
    e.checked_pwm_functions=16;e.raw_usb_connected=e.raw_usb_valid=1;e.raw_servo_valid=1;
    e.status_timestamp_us=s.timestamp;e.mode_timestamp_us=m.timestamp;e.offboard_timestamp_us=o.timestamp;e.power_timestamp_us=p.timestamp;
    e.observation_us=hrt_absolute_time();e.original_valid_until_us=e.identity_valid_until_us=guard_original_expiry(s.timestamp,100000);
    e.telemetry_fresh=e.observation_us>=s.timestamp&&e.observation_us<=e.original_valid_until_us?GuardFact::Pass:GuardFact::Fail;
    e.disarmed_control=disarmed_control_shape(s,m)?GuardFact::Pass:GuardFact::Fail;e.native_land_mode=native_land_mode_shape(s,m)?GuardFact::Pass:GuardFact::Fail;
    e.active_direct_mode=s.arming_state==vehicle_status_s::ARMING_STATE_ARMED&&s.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD&&m.flag_armed&&m.flag_control_offboard_enabled&&o.direct_actuator?GuardFact::Pass:GuardFact::Fail;
    e.native_controllers_disabled=!m.flag_control_auto_enabled&&!m.flag_control_position_enabled&&!m.flag_control_attitude_enabled&&!m.flag_control_rates_enabled&&!m.flag_control_allocation_enabled?GuardFact::Pass:GuardFact::Fail;
    e.reason=GuardReason::BootSessionUnknown;return true;
}
}
void context_telemetry(std::uint64_t stamp,unsigned phase){vehicle_status_s s{};vehicle_control_mode_s m{};offboard_control_mode_s o{};system_power_s p{};
    s.timestamp=m.timestamp=o.timestamp=p.timestamp=stamp;s.system_id=s.component_id=1;s.vehicle_type=vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;s.hil_state=vehicle_status_s::HIL_STATE_ON;
    s.arming_state=phase?vehicle_status_s::ARMING_STATE_ARMED:vehicle_status_s::ARMING_STATE_DISARMED;m.flag_armed=phase!=0;
    if(phase==1){s.nav_state=vehicle_status_s::NAVIGATION_STATE_OFFBOARD;m.flag_control_offboard_enabled=true;o.direct_actuator=true;}
    if(phase==2){s.nav_state=vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;m.flag_control_auto_enabled=m.flag_multicopter_position_control_enabled=m.flag_control_position_enabled=true;
        m.flag_control_velocity_enabled=m.flag_control_altitude_enabled=m.flag_control_climb_rate_enabled=m.flag_control_attitude_enabled=m.flag_control_rates_enabled=m.flag_control_allocation_enabled=true;}
    p.usb_connected=p.usb_valid=1;gpenmpc_test_topic_publish(ORB_ID(vehicle_status),&s);gpenmpc_test_topic_publish(ORB_ID(vehicle_control_mode),&m);
    if(bus[3].empty())gpenmpc_test_topic_publish(ORB_ID(offboard_control_mode),&o);gpenmpc_test_topic_publish(ORB_ID(system_power),&p);
}
struct ContextFixture {
    int link_object{};rs::LinkToken link{};px::ContextHostRawGuard raw_guard{};px::ContextHostClock clock{};
    px::ExecutionSessionLedger ledger{};rs::LinkLifetimeRegistry &links{*rs::link_lifetime_registry()};px::SessionEcho echo{};
    px::RegisteredContextGuard guard;
    ContextFixture(bool enrolled=true):guard(raw_guard,clock,ledger,links,make_link(),{0x1122334455667788ULL,1,1},gpenmpc_rfly_execution::kCanonicalConfigurationSha){
        if(enrolled){check(guard.register_session({1,2},echo)&&guard.confirm_echo(echo),"real session register and exact echo confirm over mock raw hardware");
            px::SessionPhysicalDeclaration d{};d.source=px::PhysicalDeclarationSource::OperatorUsbIsolationDeclaration;
            d.physical_setup_record_sha256[0]=1;d.exact_session_sha256=px::execution_session_digest(echo);d.usb_only=d.props_removed=d.no_actuator_propulsion_power=px::HumanIsolationClaim::Declared;
            check(guard.bind_physical_declaration(d),"explicit simulated attributable physical declaration bound by real session guard");}
        else {echo.link=link;echo.observed_identity={0x1122334455667788ULL,1,1};echo.process_session_generation=1;echo.configuration_sha256=gpenmpc_rfly_execution::kCanonicalConfigurationSha;}
    }
    ~ContextFixture(){(void)links.retire(link);}
    rs::LinkToken make_link(){check(links.register_constructed(&link_object,link)==rs::LinkRegistration::Registered&&links.activate(&link_object)==rs::LinkAccess::Read,"real link lifecycle registry over opaque HOST test link");return link;}
};
px::RegisteredContextConfiguration context_config(const Fixture&f,const px::SessionEcho&e){px::RegisteredContextConfiguration c{};
    c.module.source=source_config();c.module.execution=execution_config(f);const Identity observed{e.observed_identity.uid,e.process_session_generation,e.observed_identity.system,e.observed_identity.component};
    c.module.source.identity=c.module.execution.identity=observed;c.module.source.vehicle_odometry_topic=ORB_ID(vehicle_odometry);
    c.module.telemetry_max_age_us=100000;c.module.poll_period_us=1000;c.module.task_priority=100;c.module.task_stack_bytes=8192;
    c.transport={42,191,1,1,3,5000};c.retained_anchor_capacity=64;c.native_land_tail_max_us=50000;return c;}
void context_bus_start(){clear_bus();wrong_observed_uid=false;simulated_legacy_running=false;simulated_recovery_configuration_bad=false;inject_busy_after_observations=0;foreign_registration={};actual_test_hrt=1000300;context_telemetry(1000200,0);}
void context_heartbeat(const ContextFixture&e,std::uint64_t stamp){
    mavlink_message_t message{},buffer{},parsed{};mavlink_status_t tx{},parser{},status{};
    mavlink_msg_heartbeat_pack_status(42,191,&tx,&message,MAV_TYPE_GCS,MAV_AUTOPILOT_INVALID,0,0,MAV_STATE_ACTIVE);
    std::uint8_t wire[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(wire,&message);unsigned good=0;
    for(unsigned j=0;j<n;++j)good+=mavlink_frame_char_buffer(&buffer,&parser,wire[j],&parsed,&status)==MAVLINK_FRAMING_OK;
    mavlink_heartbeat_t heartbeat{};mavlink_msg_heartbeat_decode(&parsed,&heartbeat);
    check(good==1&&e.links.record_heartbeat(e.link.pointer,parsed.sysid,parsed.compid,heartbeat.type,heartbeat.autopilot,stamp)==rs::HeartbeatRecord::Recorded,"actual generated HB pack/parse original receiver event enters existing registry");
}
sw::Bytes registered_snapshot(const px::SessionEcho&e){sw::Bytes out{};unsigned offset=0;static int view;
    for(unsigned j=0;j<3;++j){px::SnapshotFragment f{};if(rs::snapshot_route_registry()->take(e.link,&view,f,actual_test_hrt)!=px::ExportTake::Fragment)throw std::runtime_error("registered RSP missing");
        const unsigned n=f.fragment.length-9;std::memcpy(out.data()+offset,f.fragment.payload+9,n);offset+=n;}return out;}
void enqueue_context_command(const Fixture&f,const SnapshotTicket&ticket,const SnapshotTicket&outer_ticket,std::uint64_t sample){
    cw::Context context{};context.configuration_sha256=old::bytes_of(gpenmpc_rfly_execution::kCanonicalConfigurationSha);context.reference_generation=context.outer_generation=1;
    context.reference_source_ticket=ticket;context.outer_source_ticket=outer_ticket;context.reference_time={9010000000ULL,9010100000ULL,9410000000ULL};
    context.outer_time={9000000000ULL,9000050000ULL,9400000000ULL};context.outer_payload={0.125,0.01,-0.02,0.03};
    for(unsigned j=0;j<3;++j){const double sign=j==2?-1:1;context.reference_ned[j]=sign*f.k.refP[j];context.reference_ned[j+3]=sign*f.k.refV[j];context.reference_ned[j+6]=sign*f.k.refA[j];}
    cw::Bytes context_bytes{};cw::encode(context,context_bytes);old::Metadata md{};md.command_generation=2;md.reference_generation=md.outer_generation=1;md.snapshot_ticket=ticket;md.configuration_sha256=context.configuration_sha256;
    auto k=f.k;k.augmentation_state_generation=k.continuity_state_generation=2;const auto abi=gpenmpc_rfly_slim::encode(k);t::SlimMessage numerical{};t::encode_slim(md,abi.data(),abi.size(),numerical);
    ingress::Receiver receiver;for(unsigned j=0;j<7;++j){old::Fragment part{};if(j<3)cw::fragment(context_bytes,j,part);else t::fragment_slim(numerical,j-3,part);
        mavlink_message_t message{},buffer{},parsed{};mavlink_status_t tx{},parser{},status{};mavlink_msg_tunnel_pack_status(42,191,&tx,&message,1,1,ingress::payload_type,part.length,part.payload);
        std::uint8_t wire[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(wire,&message);unsigned good=0;
        for(unsigned b=0;b<n;++b)good+=mavlink_frame_char_buffer(&buffer,&parser,wire[b],&parsed,&status)==MAVLINK_FRAMING_OK;check(good==1,"Context actual generated pack and byte parser");receiver.receive(parsed,sample+500+100*j,1,1,3);}
    receiver.drain(sample+1150,[&](const ingress::Fields&fields){gpenmpc_full_inner_ingress_s m{};ingress::copy_to_topic(fields,receiver.counters(),m);return gpenmpc_test_topic_publish(ORB_ID(gpenmpc_full_inner_ingress),&m);});
}
bool consume_selected(const px::SessionEcho&e,rs::Source expected,float value,unsigned generation){auto*registry=rs::shared_output_registry();static int view;
    vehicle_status_s s{};vehicle_control_mode_s m{};std::uint32_t g=0;gpenmpc_test_topic_copy(ORB_ID(vehicle_status),&s,g,0);gpenmpc_test_topic_copy(ORB_ID(vehicle_control_mode),&m,g,0);
    rs::Registration selection{};if(registry->choose(e.link,&view,actual_test_hrt,s,m,selection)!=expected)return false;
    rs::Observation observation{};observation.source=expected;observation.status=s;observation.control_mode=m;observation.output_uorb_generation=generation;
    observation.status_uorb_generation=bus_generation[1];observation.mode_uorb_generation=bus_generation[2];
    if(expected==rs::Source::RflyOutputs){std::uint32_t ignored=0;gpenmpc_test_topic_copy(ORB_ID(actuator_outputs_rfly),&observation.output,ignored,0);observation.output_uorb_generation=ignored;}
    else{observation.output.timestamp=actual_test_hrt;observation.output.noutputs=16;for(float&v:observation.output.output)v=value;}
    rs::OriginalValidity validity{};return registry->accept(e.link,&view,selection,observation,actual_test_hrt,validity)&&validity.valid_until_us>=actual_test_hrt;
}
void acquire_negative_tests(const Fixture&f){
    for(unsigned kind=0;kind<3;++kind){context_bus_start();ContextFixture env(kind!=0);auto echo=env.echo;
        if(kind==1)++echo.observed_identity.uid;auto configuration=context_config(f,echo);px::RegisteredCanonicalModuleContext context(env.guard,echo,configuration);
        px::ModuleConfiguration out{};if(kind==2)wrong_observed_uid=true;
        check(context.acquire(out)!=px::ModuleAcquire::Ready&&!out.authority,"unregistered/asserted wrong identity/observed identity drift cannot acquire");
        rs::RegistryDiagnostics d{};rs::shared_output_registry()->diagnostics(d);check(!d.bound&&context.storage_releasable(),"failed acquire binds no routes or module storage");}
    context_bus_start();ContextFixture env;auto configuration=context_config(f,env.echo);px::RegisteredCanonicalModuleContext context(env.guard,env.echo,configuration);px::ModuleConfiguration out{};
    check(context.acquire(out)==px::ModuleAcquire::Ready,"valid actual session composition acquires");rs::RegistryDiagnostics d{};rs::shared_output_registry()->diagnostics(d);check(!d.bound,"acquire creates no route/publication");
    check(context.acquire(out)==px::ModuleAcquire::Rejected,"duplicate acquire cannot create second owner");
    check(context.abort_acquire()&&context.storage_releasable(),"acquired but unspawned owner synchronously releases reservation");
}
void partial_bind_tests(const Fixture&f){for(unsigned kind=0;kind<2;++kind){context_bus_start();ContextFixture env;auto c=context_config(f,env.echo);
    px::RegisteredCanonicalModuleContext context(env.guard,env.echo,c);px::ModuleConfiguration out{};check(context.acquire(out)==px::ModuleAcquire::Ready,"partial-bind acquisition");
    px::Px4CanonicalIo io(out.source,out.execution,out.telemetry_max_age_us,out.authority);px::SnapshotOutbox snapshot;px::CommittedFeedbackOutbox feedback;
    rs::SnapshotRegistration sr{};rs::FeedbackRegistration fr{};
    if(kind==0)check(rs::snapshot_route_registry()->bind(env.echo.link,snapshot,sr)==rs::SnapshotBind::Bound,"foreign snapshot owner installed");
    else check(rs::committed_feedback_route_registry()->bind(env.echo.link,feedback,fr)==rs::FeedbackBind::Bound,"foreign feedback owner installed");
    check(context.poll(io,actual_test_hrt)==px::ModulePoll::Fault,"partial later registry bind rejects");
    check(context.close(px::ModuleStopReason::ContextFault,actual_test_hrt).route==px::ModuleDetach::Detached&&context.storage_releasable(),"partial bind cleanup releases only owned routes without artificial zero publication");
    if(kind==0)check(rs::snapshot_route_registry()->unbind(sr)==rs::SnapshotDetach::Detached,"foreign snapshot owner preserved through cleanup");
    else check(rs::committed_feedback_route_registry()->unbind(fr)==rs::FeedbackDetach::Detached,"foreign feedback owner preserved through cleanup");
}}
void dual_owner_test(const Fixture&f,const Raw&state){context_bus_start();ContextFixture first,second;
    px::RegisteredCanonicalModuleContext a(first.guard,first.echo,context_config(f,first.echo)),b(second.guard,second.echo,context_config(f,second.echo));px::ModuleConfiguration ca{},cb{};
    check(a.acquire(ca)==px::ModuleAcquire::Ready&&b.acquire(cb)!=px::ModuleAcquire::Ready&&!cb.authority,"second context cannot acquire global OCM reservation");
    px::Px4CanonicalIo ia(ca.source,ca.execution,ca.telemetry_max_age_us,ca.authority);
    auto odometry=raw(state);gpenmpc_test_topic_publish(ORB_ID(vehicle_odometry),&odometry);
    check(a.poll(ia,actual_test_hrt)==px::ModulePoll::Progress,"first actual reserved context captures observation");
    check(b.abort_acquire()&&b.storage_releasable(),"rejected second owner cleanup cannot release first reservation");
    rs::RegistryDiagnostics d{};rs::shared_output_registry()->diagnostics(d);check(d.bound,"first owner remains registered after second cleanup");
    check(a.close(px::ModuleStopReason::Requested,actual_test_hrt).route==px::ModuleDetach::Detached,"first disarmed observation owner detaches without publication");
}
void live_shape_test(const Fixture&f,const Raw&state,bool deadline){context_bus_start();ContextFixture env;auto c=context_config(f,env.echo);
    px::RegisteredCanonicalModuleContext context(env.guard,env.echo,c);px::ModuleConfiguration out{};check(context.acquire(out)==px::ModuleAcquire::Ready,"Context flow acquires real registered Guard composition");
    context_heartbeat(env,actual_test_hrt+1);actual_test_hrt+=2;
    px::Px4CanonicalIo io(out.source,out.execution,out.telemetry_max_age_us,out.authority);auto odometry=raw(state,1000000);gpenmpc_test_topic_publish(ORB_ID(vehicle_odometry),&odometry);
    check(context.poll(io,actual_test_hrt)==px::ModulePoll::Progress&&io.diagnostics().disarmed_observations_released==1&&io.diagnostics().kernel_calls==0,"registered disarmed observation performs no numerical control");
    const auto observed=registered_snapshot(env.echo);SnapshotTicket outer_ticket{};std::memcpy(outer_ticket.data(),observed.data()+4,32);
    const std::uint64_t sample=1010000;actual_test_hrt=sample+300;context_telemetry(sample+200,1);odometry=raw(state,sample);gpenmpc_test_topic_publish(ORB_ID(vehicle_odometry),&odometry);
    check(context.poll(io,actual_test_hrt)==px::ModulePoll::Progress,"registered direct entry captures fresh actual source");
    const auto captured=registered_snapshot(env.echo);SnapshotTicket ticket{};std::memcpy(ticket.data(),captured.data()+4,32);enqueue_context_command(f,ticket,outer_ticket,sample);
    actual_test_hrt=sample+1500;check(context.poll(io,actual_test_hrt)==px::ModulePoll::Progress&&io.diagnostics().kernel_calls==1&&io.diagnostics().publish_succeeded==1,"Context to real Guard authority Io Pump actual Simulink step and simulated uORB publish");
    for(unsigned j=0;j<61;++j)check(std::abs(GPENMPC_Rfly_Canonical_Control_Y.FullKernel61[j]-f.expected[j])<=1e-10,"registered execution actual61 numerical unchanged");
    check(consume_selected(env.echo,rs::Source::RflyOutputs,0.f,1),"single actual output registry accepts canonical original payload");
    actual_test_hrt+=100;context_telemetry(actual_test_hrt-10,2);foreign_link=env.echo.link;if(!deadline)inject_busy_after_observations=1;
    const auto closed=context.close(px::ModuleStopReason::Requested,actual_test_hrt);
    check(closed.route==px::ModuleDetach::Pending&&closed.plant==px::ModulePlantDisposition::NativeLandingObserved&&!context.storage_releasable(),"explicit externally observed native LAND retains module storage");
    const auto stopped_ocm=context.diagnostics_after_stop().offboard_publications;
    context_heartbeat(env,actual_test_hrt+1);++actual_test_hrt;
    check(context.poll(io,actual_test_hrt)==px::ModulePoll::Fault&&context.diagnostics_after_stop().offboard_publications==stopped_ocm&&env.links.native_offboard_allowed()==rs::LinkAccess::Rejected,
        "new actual HB after native LAND/close cannot revive OCM or release global reservation");
    check(context.diagnostics_after_stop().interrupted_raw_feedback_retained&&!context.diagnostics_after_stop().failed_raw_feedback_retained&&
        context.interrupted_raw_feedback_after_stop().disposition==px::FeedbackDisposition::Revoked&&
        std::memcmp(context.interrupted_raw_feedback_after_stop().actual61.data(),GPENMPC_Rfly_Canonical_Control_Y.FullKernel61,488)==0,
        "unconsumed RFC1 actual61 retained separately after registry quiescence, no fake downstream ACK");
    if(!deadline){const auto original_deadline=context.diagnostics_after_stop().original_land_deadline_us;
        check(foreign_registration.generation!=0&&rs::shared_output_registry()->unbind(foreign_registration)==rs::Detach::Detached,"native begin succeeded but foreign bind busy is retained and released");
        check(context.close(px::ModuleStopReason::Requested,actual_test_hrt).route==px::ModuleDetach::Pending&&context.diagnostics_after_stop().original_land_deadline_us==original_deadline,"native bind retry neither repeats begin nor renews deadline");}
    check(consume_selected(env.echo,rs::Source::NativeOutputsSim,.1f,1),"native safety tail selected separately without canonical fallback");
    if(deadline){actual_test_hrt=context.diagnostics_after_stop().original_land_deadline_us+1;context_telemetry(actual_test_hrt-10,2);
        check(context.close(px::ModuleStopReason::Requested,actual_test_hrt).route!=px::ModuleDetach::Detached&&!context.storage_releasable()&&context.diagnostics_after_stop().first_fault==px::RegisteredContextFault::NativeTailDeadline,"original LAND deadline passed: owner retired but storage retained, no forced delete");}
    actual_test_hrt+=100;context_telemetry(actual_test_hrt-10,0);vehicle_land_detected_s landed{};landed.timestamp=actual_test_hrt-10;landed.landed=true;gpenmpc_test_topic_publish(ORB_ID(vehicle_land_detected),&landed);
    if(!deadline){inject_busy_after_observations=3;foreign_registration={};}
    check(context.close(px::ModuleStopReason::Requested,actual_test_hrt).route==px::ModuleDetach::Pending,"actual disarm selects distinct zero-only route before detach");
    if(!deadline){check(foreign_registration.generation!=0&&rs::shared_output_registry()->unbind(foreign_registration)==rs::Detach::Detached,"zero begin succeeded but bind busy preserves its first activation");
        check(context.close(px::ModuleStopReason::Requested,actual_test_hrt).route==px::ModuleDetach::Pending,"zero bind retry succeeds without repeated begin or renewed expiry");}
    check(consume_selected(env.echo,rs::Source::NativeOutputsSim,0.f,1),"zero-only route accepts actual simulated original native zero");
    check(context.close(px::ModuleStopReason::Requested,actual_test_hrt).route==px::ModuleDetach::Detached&&context.storage_releasable()&&context.diagnostics_after_stop().virtual_zero_stream_accepted&&!context.diagnostics_after_stop().plant_cache_zero_proven,"quiescent routes detach with stream acceptance and pending plant confirmation");
}
#ifndef GPENMPC_CONTEXT_TEST_HELPERS_ONLY
int main(int argc,char**argv){try{if(argc!=3)return 2;const auto fixture=read_kernel(argv[1]);const auto state=read_state(argv[2]);
    acquire_negative_tests(fixture[0]);partial_bind_tests(fixture[0]);dual_owner_test(fixture[0],state[0]);live_shape_test(fixture[0],state[0],false);live_shape_test(fixture[0],state[0],true);
    std::cout<<"{\"checks\":"<<checks<<",\"failed\":"<<failures<<",\"actual_context_cpp\":true,\"actual_kernel_steps_positive\":2,\"actual_native_plant_zero_proven\":false,\"scope\":\"REAL_SESSION_LEDGER_REGISTRY_CONTEXT_IO_PUMP_WITH_MOCK_RAWGUARD_UORB_HRT\"}\n";return failures?1:0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
#endif
