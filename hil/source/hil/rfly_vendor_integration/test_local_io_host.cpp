// Actual Px4CanonicalLocalIo.cpp + facade/private74 + original GP DLL.
// Broker, HRT, Authority and source/context/rotor provenance are HOST mocks.
// The ABI-call shim forwards calls and permits explicit
// failure injection at the joint-install boundary. No socket/model/device.
#define wmain retained_abi_test_main
#include "full_inner_abi/test_full_inner_abi.cpp"
#undef wmain
#include "px4_runtime/Px4CanonicalLocalIo.hpp"
#include <memory>
#include <limits>
namespace li=gpenmpc_rfly_px4;namespace fc=gpenmpc_full_consumption;namespace ib=gpenmpc_local_input;namespace od=gpenmpc_odometry;
const orb_metadata __orb_vehicle_odometry{0,sizeof(vehicle_odometry_s),1};
const orb_metadata __orb_vehicle_status{1,sizeof(vehicle_status_s),1};
const orb_metadata __orb_vehicle_control_mode{2,sizeof(vehicle_control_mode_s),1};
const orb_metadata __orb_offboard_control_mode{3,sizeof(offboard_control_mode_s),1};
const orb_metadata __orb_actuator_outputs_rfly{4,sizeof(actuator_outputs_s),1};
static vehicle_odometry_s raw_fixture[60],bus_source{};
static vehicle_status_s bus_status{};static vehicle_control_mode_s bus_mode{};static offboard_control_mode_s bus_offboard{};
static uint32_t bus_generation[4]{};static uint64_t clock_us{},clock_calls{},publish_calls{},abi_calls{};
static unsigned injection{},timeline_n{};static char timeline[256]{};static actuator_outputs_s actual_output{};
static bool publish_success=true;static fc::Hash fixture_hash{};
static gpenmpc_full_inner_reference_state actual_copied_reference{};
static bool same_reference_state(const gpenmpc_full_inner_reference_state&a,const gpenmpc_full_inner_reference_state&b){
    return same(a.reference_asset_sha256,b.reference_asset_sha256,32)&&a.leg_index==b.leg_index&&a.window_generation==b.window_generation&&
        a.last_accepted_sequence==b.last_accepted_sequence&&same(&a.query_progress_s,&b.query_progress_s,8)&&same(&a.progress_rate,&b.progress_rate,8)&&
        same(&a.phase_acceleration,&b.phase_acceleration,8)&&same(a.outer_i,b.outer_i,24)&&a.source_timestamp_ns==b.source_timestamp_ns&&
        a.source_generation==b.source_generation&&a.reference_generation==b.reference_generation&&a.outer_generation==b.outer_generation&&
        a.output_generation==b.output_generation&&a.publication_us==b.publication_us;
}
static li::Px4CanonicalLocalIo*active_io{};
static void event(char c){if(timeline_n<sizeof timeline)timeline[timeline_n++]=c;}
extern "C" uint64_t hrt_absolute_time()noexcept{++clock_calls;return clock_us++;}
bool gpenmpc_test_topic_copy(const orb_metadata*m,void*out,uint32_t&gen,uint8_t instance)noexcept{
    if(!m||m->id>3||instance||!bus_generation[m->id])return false;
    const void*p=m->id==0?static_cast<void*>(&bus_source):m->id==1?static_cast<void*>(&bus_status):m->id==2?static_cast<void*>(&bus_mode):static_cast<void*>(&bus_offboard);
    std::memcpy(out,p,m->size);gen=bus_generation[m->id];return true;
}
bool gpenmpc_test_topic_update(const orb_metadata*m,void*out,uint32_t&gen,uint8_t instance)noexcept{
    return m&&m->id<4&&gen!=bus_generation[m->id]&&gpenmpc_test_topic_copy(m,out,gen,instance);
}
bool gpenmpc_test_topic_publish(const orb_metadata*m,const void*data)noexcept{
    ++publish_calls;event('P');check(m==ORB_ID(actuator_outputs_rfly),"only canonical16 output topic publication requested");
    std::memcpy(&actual_output,data,sizeof actual_output);
    if(injection==1)clock_us+=6000; // actual successful call returns after original expiry
    if(injection==2){auto*p=const_cast<actuator_outputs_s*>(static_cast<const actuator_outputs_s*>(data));p->output[0]=std::nextafter(p->output[0],1.0f);}
    return publish_success;
}
extern "C" int gpenmpc_test_call_full_inner_commit(gpenmpc_full_inner_owner*owner,const gpenmpc_full_inner_backend*b,
    const gpenmpc_full_inner_numeric_receipt*n,const gpenmpc_full_inner_reference_receipt*r){
    ++abi_calls;event('C');int result=0;
    if(injection==3){auto rejected=*n;rejected.numerical_commit_succeeded=0;result=gpenmpc_full_inner_commit(owner,b,&rejected,r);}
    else result=gpenmpc_full_inner_commit(owner,b,n,r);
    if(injection==4)clock_us+=6000; // real install completed; original expiry now lost
    return result;
}
extern "C" int gpenmpc_test_call_full_inner_copy_state(const gpenmpc_full_inner_owner*owner,gpenmpc_full_inner_state*out){
    const int result=gpenmpc_full_inner_copy_state(owner,out);
    if(result==RFI_OK&&out->numeric_installed&&out->reference_committed){
        actual_copied_reference=out->reference;
        if(injection==6)return RFI_ARGUMENT; // explicit read-return failure after a real copy
        if(injection==7)clock_us+=6000; // real original state read, late return
    }
    return result;
}
static fc::Identity identity(){return {0x1122334455667788ULL,42,1,1};}
struct MockAuthority:li::Authority{
    bool available=true,validation_ok=true,lease_ok=true,confirmation_ok=true,revoked=false;
    uint64_t deadline{},validations{},leases{},confirmations{},revocations{},installed_expiry{};
    li::Token token{};actuator_outputs_s output{};
    bool observed_identity(li::Identity&out)noexcept override{out=identity();return available&&!revoked;}
    bool validate(const li::Token&t,const vehicle_status_s&,const vehicle_control_mode_s&,const offboard_control_mode_s&,uint64_t,uint64_t&until)noexcept override{
        ++validations;event('V');token=t;until=deadline;return validation_ok;}
    bool install_lease(const li::Token&t,const actuator_outputs_s&v,uint64_t until)noexcept override{
        ++leases;event('L');token=t;output=v;installed_expiry=until;check(until<=deadline,"lease expiry never extends mocked authority original bound");return lease_ok;}
    bool confirm_publication(const li::Token&t,uint64_t pub,uint64_t installed)noexcept override{
        ++confirmations;event('F');check(t==token&&pub<=installed&&active_io&&active_io->diagnostics().consumption_commits>0&&active_io->diagnostics().joint_installs>0,
            "actual consumption and real joint install precede authority confirmation");
        if(injection==5)clock_us+=6000;return confirmation_ok;}
    void revoke()noexcept override{revoked=true;++revocations;event('R');}
};
static fc::Configuration config_consumption(){fc::Configuration c{};c.identity=identity();c.numerical=config;gpenmpc_full_inner_identity(&c.build);
    c.limits={5000,100000,400000,5000,gpenmpc_consumption::PublicationPath::DirectCanonicalMotors};return c;}
static od::Configuration config_source(){od::Configuration c{};c.identity=identity();c.vehicle_odometry_topic=ORB_ID(vehicle_odometry);
    c.coordinates=od::Coordinates::ExplicitTranslatedNed;c.sample_max_age_us=5000;return c;}
static void reset_bus(){std::memset(bus_generation,0,sizeof bus_generation);clock_us=1;clock_calls=publish_calls=abi_calls=0;injection=0;timeline_n=0;publish_success=true;active_io=nullptr;}
static void source(unsigned index,bool disarmed=false){bus_source=raw_fixture[index];bus_generation[0]=static_cast<uint32_t>(original[index].tags[1]);
    bus_status={};bus_mode={};bus_offboard={};const auto stamp=bus_source.timestamp_sample+150;
    bus_status.timestamp=bus_mode.timestamp=bus_offboard.timestamp=stamp;
    bus_status.hil_state=vehicle_status_s::HIL_STATE_ON;bus_status.nav_state=vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
    bus_status.arming_state=disarmed?vehicle_status_s::ARMING_STATE_DISARMED:vehicle_status_s::ARMING_STATE_ARMED;
    bus_mode.flag_armed=!disarmed;bus_mode.flag_control_offboard_enabled=!disarmed;bus_offboard.direct_actuator=true;
    bus_generation[1]=bus_generation[2]=bus_generation[3]=index+1;clock_us=bus_source.timestamp_sample+300;
}
struct Command {li::LocalCommand value{};ib::BoundRotorLag rotor{};ib::BoundPayload payload{};ib::BoundWind wind{};ib::ExplicitInitialInterval initial{};};
static bool command(li::Px4CanonicalLocalIo&io,const li::LocalTicket&t,unsigned i,Command&out){
    const auto*s=io.snapshot(t);if(!s)return false;auto&c=out.value;c.ticket=t;auto&cx=c.input_context;
    cx.observed_session=identity();cx.explicit_leg=config.leg_index;cx.task_sha256=fc::detail::words(config.task_sha256);
    cx.configuration_sha256=fc::detail::words(config.configuration_sha256);cx.reference_asset_sha256=fc::detail::words(config.reference_asset_sha256);
    cx.step_kind=i?ib::StepKind::SubsequentObservedSource:ib::StepKind::FirstOfExplicitLeg;
    ib::SnapshotKey key{};ib::snapshot_key(*s,key);out.rotor.source=out.payload.source=out.wind.source=key;
    out.rotor.value_kind=ib::RotorValueKind::OriginalPlantLagState;out.rotor.verified_association_receipt_sha256=fixture_hash;
    auto&lag=out.rotor.original_observation;for(unsigned j=0;j<6;++j)lag.observed_thrust_n[j]=original[i].input[13+j];
    lag.dll_generation=i+1;lag.dll_session=11;lag.original_host_receive_ns=7000000000ULL+9000000ULL*i;
    lag.original_board_ingress_us=s->estimator().board_rx_us;lag.original_sim_time_s=1+.009*i;lag.original_observation_sha=fixture_hash;
    out.payload.task_sha256=cx.task_sha256;out.payload.original_schedule_evidence_sha256=fixture_hash;out.payload.original_schedule_generation=i+1;out.payload.payload_kg=original[i].input[31];
    out.wind.original_estimate_evidence_sha256=fixture_hash;out.wind.original_estimate_generation=i+1;out.wind.estimate_xy_mps={original[i].input[32],original[i].input[33]};
    out.initial.configuration_sha256=cx.configuration_sha256;out.initial.original_configuration_receipt_sha256=fixture_hash;out.initial.leg=cx.explicit_leg;out.initial.configured_dt_s=rows[i].args[9];
    c.rotor=&out.rotor;c.payload=&out.payload;c.wind=&out.wind;c.initial_interval=i?nullptr:&out.initial;
    c.reference_query=reference_input(i);c.reference_query.reference_generation=i+1;c.reference_query.outer_generation=i+1;
    const auto time=s->estimator().timestamp_sample_us;auto&r=c.reference_envelope;r.identity=identity();r.generation=i+1;r.outer_generation=i+1;
    r.timestamp_us=time+210;r.board_rx_us=time+250;r.valid_until_us=time+100000;
    for(unsigned j=0;j<3;++j){const double sign=j==2?-1:1;r.p[j]=sign*original[i].input[19+j];r.v[j]=sign*original[i].input[22+j];r.a[j]=sign*original[i].input[25+j];}
    auto&o=c.outer_envelope;o.identity=identity();o.generation=i+1;o.based_on_sample_generation=t.source_generation;
    o.based_on_timestamp_sample_us=time;o.board_rx_us=time+220;o.valid_until_us=time+400000;o.payload_sha256=fc::detail::outer_payload_sha(c.reference_query);return true;
}
struct TestOwner{
    MockAuthority authority;std::unique_ptr<li::Px4CanonicalLocalIo>io;std::vector<uint64_t>scratch;
    explicit TestOwner(bool load=true,bool no_authority=false,uint64_t commander_age=0,uint64_t transport_age=0):scratch((gpenmpc_full_inner_window_scratch_bytes()+7)/8){
        reset_bus();auto sc=config_source();auto cc=config_consumption();
        if(transport_age){sc.sample_max_age_us=cc.limits.sample_max_age_us=50000;cc.limits.transaction_max_wall_us=4000;}
        io.reset(new li::Px4CanonicalLocalIo(sc,cc,100000,
            no_authority?nullptr:&authority,commander_age,transport_age));active_io=io.get();
        if(load)check(io->load_window(windows[rows[0].ids[0]-1],scratch.data(),scratch.size()*8),"startup real reference window load");}
    bool capture(unsigned i,Command&c){source(i);authority.deadline=raw_fixture[i].timestamp_sample+5000;li::LocalTicket t{};
        return io->capture_next(t)==li::LocalCapture::Accepted&&command(*io,t,i,c);}
    bool load(){return io->load_window(windows[rows[0].ids[0]-1],scratch.data(),scratch.size()*8);}
};
static bool load_private(const wchar_t*path){FILE*f=_wfopen(path,L"rb");if(!f)return false;char magic[4];uint32_t n{},bytes{};
    bool ok=read(f,magic,4)&&same(magic,"SJC1",4)&&read(f,&n,4)&&n==60&&read(f,&bytes,4)&&bytes==sizeof(vehicle_odometry_s);
    for(unsigned i=0;i<60&&ok;++i){Original row{};ok=read(f,&raw_fixture[i],bytes)&&read(f,&row,sizeof row)&&same(&row,&original[i],sizeof row);}std::fclose(f);return ok;}
static bool parse_hash(const wchar_t*s){if(std::wcslen(s)!=64)return false;for(unsigned i=0;i<64;++i){unsigned v=16;const auto c=s[i];
    if(c>=L'0'&&c<=L'9')v=c-L'0';if(c>=L'A'&&c<=L'F')v=c-L'A'+10;if(v>15)return false;fixture_hash[i/8]=(fixture_hash[i/8]<<4)|v;}return true;}
int wmain(int argc,wchar_t**argv){if((argc!=5&&argc!=6)||!load(argv[1],argv[2])||!load_private(argv[2])||!parse_hash(argv[4]))return 2;
    if(argc==6){
      if(!std::wcscmp(argv[5],L"TRANSPORT_ONLY")){
        {TestOwner o(true,false,0,20000);Command c{};check(o.capture(0,c),"original source and command captured");
          o.authority.deadline=raw_fixture[0].timestamp_sample+50000;
          check(o.io->execute(c.value),"real generated control and joint install within original four ms");
          li::LocalExecutionFeedback f{};check(o.io->take_execution_feedback(f)&&f.fresh_at_read,"original feedback validity retained");
          check(o.authority.installed_expiry==o.authority.token.control_tick_us+20000&&
            f.original_valid_until_us==o.authority.token.control_tick_us+4000,"20ms output transport distinct from 4ms compute and feedback");
          check(o.authority.installed_expiry<=raw_fixture[0].timestamp_sample+50000,"source validity not renewed");}
        for(const unsigned kind:{1u,4u,5u,7u}){TestOwner o(true,false,0,20000);Command c{};check(o.capture(0,c),"late-call capture");
          o.authority.deadline=raw_fixture[0].timestamp_sample+50000;injection=kind;
          check(!o.io->execute(c.value)&&o.authority.revoked,"late publication/install/confirmation/copy still reject despite transport budget");}
        {TestOwner o(true,false,0,20000);Command c{};check(o.capture(0,c),"short outer capture");
          o.authority.deadline=raw_fixture[0].timestamp_sample+50000;c.value.outer_envelope.valid_until_us=clock_us+2000;
          check(o.io->execute(c.value)&&o.authority.installed_expiry==c.value.outer_envelope.valid_until_us,"transport cannot extend original outer expiry");}
        std::printf("TRANSPORT_ONLY checks=%u failed=%u generated_math=ACTUAL source_and_transport=MOCK COM=0\n",checks,failed);return failed?1:0;
      }
      if(!std::wcscmp(argv[5],L"CADENCE_ONLY")){
        {TestOwner o;source(0);li::LocalTicket t{};
          check(o.io->capture_next(t)==li::LocalCapture::Accepted,"unexecuted first source capture");
          const auto old=t;
          check(o.io->capture_next(t,false,true)==li::LocalCapture::NoUpdate&&t==old&&o.io->snapshot(t),"no new source retains pending ticket without executing");
          source(1);const auto stamp=bus_source.timestamp_sample;
          check(o.io->capture_next(t,false,true)==li::LocalCapture::Accepted&&t.sample_us==stamp&&!(t==old)&&!o.io->snapshot(old),"fresh first source replaces only unexecuted original snapshot");
          check(o.io->diagnostics().kernel_calls==0&&publish_calls==0,"first-source refresh performs no math or publication");}
        const auto raw1=raw_fixture[1];const auto old1=original[1];const auto row1=rows[1];
        for(const uint64_t dt:{10000ULL,15444ULL,20000ULL,20001ULL}){
          raw_fixture[1]=raw1;original[1]=old1;rows[1]=row1;
          raw_fixture[1].timestamp_sample=raw_fixture[0].timestamp_sample+dt;
          raw_fixture[1].timestamp=raw_fixture[1].timestamp_sample+100;
          original[1].tags[0]=raw_fixture[1].timestamp_sample*1000;
          rows[1].args[9]=double(dt)*1e-6;original[1].input[34]=rows[1].args[9];
          TestOwner o;Command first{};check(o.capture(0,first)&&o.io->execute(first.value),"first actual generated kernel commits in mock broker");
          li::LocalExecutionFeedback f{};check(o.io->take_execution_feedback(f),"first feedback consumed");
          Command next{};const bool captured=o.capture(1,next);
          if(dt>20000){check(!captured&&o.io->diagnostics().source_adapter_fault==od::Failure::SampleDelta&&o.io->diagnostics().joint_installs==1,"20ms plus one remains rejected before second execution");continue;}
          check(captured,"bounded actual next source captured");
          const bool executed=captured&&o.io->execute(next.value);
          if(!executed)std::fprintf(stderr,"CADENCE dt=%llu fault=%u consumption=%u\n",(unsigned long long)dt,unsigned(o.io->diagnostics().first_fault),unsigned(o.io->consumption_diagnostics().first_fault));
          check(executed&&o.io->diagnostics().joint_installs==2,"real unchanged generated math accepts bounded actual delta and commits twice");
          for(unsigned j=0;j<6;++j)check(std::isfinite(actual_output.output[j])&&actual_output.output[j]>=0&&actual_output.output[j]<=1,"finite bounded virtual rotor result");
          check(o.io->capture_next(next.value.ticket,false,true)==li::LocalCapture::Rejected&&o.io->diagnostics().first_fault==li::LocalFault::Pending,"refresh flag cannot bypass unconsumed execution feedback");
        }
        raw_fixture[1]=raw1;original[1]=old1;rows[1]=row1;
        {od::AtomicOdometryAdapter a(config_source());od::Snapshot s{};
          auto raw=raw_fixture[0];check(a.ingest(raw,1,raw.timestamp_sample+300,raw.timestamp_sample+300,identity(),ORB_ID(vehicle_odometry),0,s),"legacy source initial sample");
          raw.timestamp_sample+=15444;raw.timestamp=raw.timestamp_sample+100;
          check(!a.ingest(raw,2,raw.timestamp_sample+300,raw.timestamp_sample+300,identity(),ORB_ID(vehicle_odometry),0,s)&&a.failure()==od::Failure::SampleDelta,"legacy default 10ms unchanged");}
        std::printf("CADENCE_ONLY checks=%u failed=%u source_publication=MOCK generated_math=ACTUAL COM=0\n",checks,failed);return failed?1:0;
      }
      if(std::wcscmp(argv[5],L"CAPTURE_ONLY"))return 2;
      for(unsigned fault=0;fault<4;++fault){
        TestOwner o;source(0,true);li::LocalTicket t{};
        check(o.io->capture_disarmed(t)==li::LocalCapture::Accepted&&o.io->release_disarmed(t),"original disarmed observation before first control");
        const auto previous=bus_source.timestamp_sample;source(1);
        bus_source.timestamp_sample=previous+12500;bus_source.timestamp=bus_source.timestamp_sample+100;
        clock_us=bus_source.timestamp_sample+300;
        if(fault==1)clock_us=bus_source.timestamp_sample+5001;
        if(fault==2)++bus_source.reset_counter;
        if(fault==3)bus_source.timestamp=bus_source.timestamp_sample-1;
        const auto result=o.io->capture_next(t);
        if(!fault){
          check(result==li::LocalCapture::Accepted,"first actual control capture is not timed from a prior disarmed integration");
          check(o.io->snapshot(t)&&o.io->snapshot(t)->actual_sample_delta_us()==12500,"original 12.5ms observation delta retained without retimestamping");
        }else{
          const od::Failure expected=fault==1?od::Failure::Stale:fault==2?od::Failure::Reset:od::Failure::TimeOrder;
          check(result==li::LocalCapture::Rejected&&o.io->diagnostics().source_adapter_fault==expected,"first-control real source fault remains rejected with exact reason");
        }
        check(o.io->diagnostics().kernel_calls==0&&publish_calls==0,"capture-only performs no control or publication");
      }
      std::printf("CAPTURE_ONLY checks=%u failed=%u kernel_calls=0 publication_calls=0\n",checks,failed);return failed?1:0;
    }
    HMODULE dll=LoadLibraryExW(argv[3],nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);if(!dll)return 3;
    using Predict=int(*)(const double*,double*);const auto predict=reinterpret_cast<Predict>(GetProcAddress(dll,"gpenmpc_gp256_predict"));if(!predict)return 3;
    unsigned completed=0,gps=0;
    {TestOwner o;for(unsigned i=0;i<60;++i){Command c{};check(o.capture(i,c),"actual LocalIo captures sole original private source");timeline_n=0;
        const bool ok=o.io->execute(c.value);if(!ok)std::fprintf(stderr,"LocalIo row %u fault %u consumption %u\n",i,static_cast<unsigned>(o.io->diagnostics().first_fault),static_cast<unsigned>(o.io->consumption_diagnostics().first_fault));
        check(ok,"actual LocalIo source-to-publication-to-install-to-confirm");if(!ok)break;
        check(timeline_n==5&&same(timeline,"VLPCF",5),"observable validate lease publish actualABIcommit confirm order");
        li::LocalExecutionFeedback f{},empty{};check(o.io->take_execution_feedback(f)&&f.fresh_at_read&&!o.io->take_execution_feedback(empty),"fresh feedback is one-shot");
        check(same(f.request19,original[i].request,152)&&same(f.actual_control16,original[i].controls,64),"actual feedback request19 and published16 match retained numerical oracle");
        const auto&r=f.committed_reference;
        check(f.reference_state_copied&&same_reference_state(r,actual_copied_reference)&&same(r.reference_asset_sha256,config.reference_asset_sha256,32)&&r.leg_index==config.leg_index&&
            r.source_timestamp_ns==original[i].tags[0]&&r.source_generation==original[i].tags[1]&&r.reference_generation==i+1&&r.outer_generation==i+1&&r.output_generation==i+1&&
            r.window_generation==c.value.reference_query.window_generation&&r.last_accepted_sequence==rows[i].seq&&r.publication_us==f.original_publication_us,
            "feedback contains actual copied committed reference original identities/publication");
        check(same(&r.query_progress_s,&rows[i].phase,8)&&same(&r.progress_rate,&rows[i].args[0],8)&&same(&r.phase_acceleration,&original[i].transition[12],8)&&
            same(r.outer_i,original[i].transition+14,24)&&f.original_publication_us<=f.original_commit_completed_us,
            "committed phase/rate/actual bounded acceleration/outer values exact; original completion not relabeled");
        check(same(actual_output.output,original[i].controls,64)&&actual_output.noutputs==0,"actual official16 bits exact; vendor noutputs zero remains disclosed");
        check(o.io->diagnostics().kernel_calls==i+1&&o.io->diagnostics().joint_installs==i+1&&publish_calls==i+1,"one actual C and publication/install per source");
        gpenmpc_full_inner_diagnostics d{};check(o.io->numerical_diagnostics(d)&&d.joint_installs==i+1,"underlying real stores agree with Io counts");
        if(d.prediction_required){double gp[18];check(predict(f.request19+1,gp)==0&&same(gp,original[i].gp,144),"actual original GP consumes actual Io request not recorded predictor input");
            const uint64_t tags[2]={f.token.lease_envelope.timestamp_sample_us*1000,f.token.lease_envelope.sample_generation};
            const auto before=publish_calls;check(o.io->fill_gp(tags,gp)&&publish_calls==before,"GP ingress only fills pending; never executes control");++gps;}
        ++completed;}
        check(completed==60&&gps==59&&o.authority.confirmations==60&&clock_calls>60&&abi_calls==60,"complete actual LocalIo60/private74/GP59 closure");}
    for(unsigned mode=0;mode<10;++mode){TestOwner o;Command c{};check(o.capture(0,c),"failure boundary actual source capture");
        if(mode==0)publish_success=false;if(mode>=1&&mode<=5)injection=mode;
        if(mode==6)o.authority.validation_ok=false;if(mode==7)o.authority.lease_ok=false;if(mode==8)o.authority.confirmation_ok=false;
        if(mode==9)o.authority.deadline=clock_us-1;
        check(!o.io->execute(c.value)&&o.io->diagnostics().first_fault!=li::LocalFault::None&&o.authority.revoked,"publish/commit/expiry/authority failures sticky revoke");
        const auto&d=o.io->diagnostics();gpenmpc_full_inner_diagnostics n{};o.io->numerical_diagnostics(n);
        if(mode==0)check(d.publish_attempts==1&&d.publish_succeeded==0&&d.consumption_commits==0&&d.joint_installs==0&&o.authority.confirmations==0,"publisher failure has no success/install/confirmation");
        if(mode==1||mode==2)check(d.publish_succeeded==1&&d.consumption_commits==0&&d.joint_installs==0&&n.reported_publications==1,"already published event preserved after numeric validation failure");
        if(mode==3)check(d.publish_succeeded==1&&d.consumption_commits==1&&d.joint_installs==0&&n.reported_publications==1,"real joint receipt rejection preserves preceding side effects");
        if(mode==4)check(d.joint_installs==1&&o.authority.confirmations==0,"post-install expiry records real state install without authority confirmation");
        if(mode==5)check(d.joint_installs==1&&o.authority.confirmations==1,"post-confirm expiry records real confirmation then revoke");
        if(mode==6||mode==7||mode==9)check(d.publish_attempts==0&&publish_calls==0,"invalid authority or original expiry blocks publication");
        if(mode==8)check(d.joint_installs==1&&d.publish_succeeded==1,"failed confirmation does not erase actual publication or install");
        li::LocalExecutionFeedback f{};check(o.io->take_execution_feedback(f)&&!f.fresh_at_read,"failed execution raw feedback not mislabeled fresh");
        const auto calls=publish_calls;check(!o.io->execute(c.value)&&publish_calls==calls,"same failed source cannot automatically resume");
    }
    for(unsigned mode:{6U,7U}){TestOwner o;Command c{};check(o.capture(0,c),"read-copy boundary actual source capture");injection=mode;
        check(!o.io->execute(c.value)&&o.authority.revoked&&o.authority.confirmations==0,"failed or late read-copy cannot attempt authority confirmation");
        const auto&d=o.io->diagnostics();li::LocalExecutionFeedback f{};
        check(d.publish_succeeded==1&&d.consumption_commits==1&&d.joint_installs==1&&o.io->take_execution_feedback(f)&&f.numerical_reference_installed&&!f.fresh_at_read&&
            f.original_commit_completed_us>=f.original_publication_us,"read-copy failure preserves actual publication and joint installation evidence");
        check(f.reference_state_copied==(mode==7)&&d.first_fault==(mode==7?li::LocalFault::Expired:li::LocalFault::Install),"copy disposition and original first fault accurately retained");}
    {TestOwner o(false);Command c{};check(o.capture(0,c),"same-source missing window capture");const auto source_time=c.value.ticket.sample_us;
        check(!o.io->execute(c.value)&&o.io->diagnostics().awaiting_window&&o.io->diagnostics().first_fault==li::LocalFault::None&&publish_calls==0,"real window miss is nonfault and zero inner/publish");
        check(o.load()&&o.io->execute(c.value)&&o.io->diagnostics().captured==1&&o.io->diagnostics().kernel_calls==1,"bounded refill reuses same source with no additional capture");
        li::LocalExecutionFeedback f{};o.io->take_execution_feedback(f);check(f.token.lease_envelope.timestamp_sample_us==source_time&&f.original_valid_until_us<=source_time+5000,"refill never renews source deadline");}
    for(unsigned k=0;k<9;++k){TestOwner o(false);Command c{};o.capture(0,c);o.io->execute(c.value);
        if(k==0)clock_us=c.value.ticket.sample_us+5001;
        const bool loaded=o.load();if(k==0){check(!loaded&&publish_calls==0,"late real window refill permanently refuses original stale source");continue;}
        check(loaded,"negative retry window loaded before original deadline");
        if(k==1)c.value.reference_query.progress_s+=.001;if(k==2)c.value.reference_envelope.valid_until_us++;
        if(k==3)c.rotor.original_observation.observed_thrust_n[0]+=.1;if(k==4)c.payload.payload_kg+=.1;
        if(k==5)c.wind.estimate_xy_mps[0]+=.1;if(k==6)c.initial.configured_dt_s=.008;
        if(k==7)c.rotor.verified_association_receipt_sha256[0]^=1;if(k==8)c.value.wind=nullptr;
        check(!o.io->execute(c.value)&&publish_calls==0&&o.io->diagnostics().kernel_calls==0,"same-source retry rejects replacement original command or bound observation");}
    for(unsigned k=0;k<10;++k){TestOwner o;Command c{};o.capture(0,c);
        if(k==0)c.value.ticket.capture++;if(k==1)c.value.reference_query.source_generation++;if(k==2)c.value.reference_query.dt_s=.008;
        if(k==3)c.value.outer_envelope.payload_sha256[0]^=1;if(k==4)c.value.reference_envelope.valid_until_us=clock_us-1;
        if(k==5)c.value.outer_envelope.valid_until_us=clock_us-1;if(k==6)c.value.input_context.configuration_sha256[0]^=1;
        if(k==7)c.value.rotor=nullptr;if(k==8)bus_mode.flag_control_attitude_enabled=true;
        if(k==9)clock_us=c.value.ticket.sample_us+5001;
        check(!o.io->execute(c.value)&&publish_calls==0,"wrong source dt context mode missing rotor or stale command never publishes");}
    {TestOwner o(true,true);source(0);li::LocalTicket t{};check(o.io->capture_next(t)==li::LocalCapture::Rejected&&publish_calls==0,"missing Authority cannot capture/execute");}
    {TestOwner o;source(0,true);li::LocalTicket t{};check(o.io->capture_disarmed(t)==li::LocalCapture::Accepted&&o.io->release_disarmed(t)&&publish_calls==0&&o.io->diagnostics().kernel_calls==0,"real disarmed observation lifecycle advances no numerical state");}
    {TestOwner o(true,false,600000);source(0,true);const auto now=clock_us;
        bus_status.timestamp=bus_mode.timestamp=now-500000;li::LocalTicket t{};
        check(o.io->capture_disarmed(t)==li::LocalCapture::Accepted&&o.io->release_disarmed(t),
            "500ms Commander status and mode remain fresh under explicit 600ms local cadence");}
    {TestOwner o(true,false,600000);Command c{};check(o.capture(0,c),"active split-age positive source capture");
        const auto now=clock_us;bus_status.timestamp=bus_mode.timestamp=now-500000;
        check(o.io->execute(c.value)&&publish_calls==1,
            "active control accepts 500ms Commander status and mode when fast Offboard and source remain fresh");
        li::LocalExecutionFeedback f{};check(o.io->take_execution_feedback(f)&&
            f.original_valid_until_us<=bus_offboard.timestamp+100000&&
            f.original_valid_until_us<=c.value.ticket.sample_us+5000,
            "active lease remains bounded by fast Offboard/source expiry rather than 600ms Commander cadence");}
    {TestOwner o(true,false,600000);source(0,true);const auto now=clock_us;
        bus_status.timestamp=now-600001;bus_mode.timestamp=now;li::LocalTicket t{};
        check(o.io->capture_disarmed(t)==li::LocalCapture::Rejected&&
            o.io->diagnostics().first_fault==li::LocalFault::Telemetry,
            "Commander status beyond 600ms fails closed");}
    {TestOwner o(true,false,600000);Command c{};check(o.capture(0,c),"active split-age source capture");
        const auto now=clock_us;bus_status.timestamp=bus_mode.timestamp=now-500000;
        bus_offboard.timestamp=now-100001;
        check(!o.io->execute(c.value)&&o.io->diagnostics().first_fault==li::LocalFault::Telemetry&&publish_calls==0,
            "100ms offboard age remains unchanged while Commander uses 600ms");}
    {TestOwner o;Command c{};o.capture(0,c);o.io->stop();const auto prior=publish_calls;check(!o.io->execute(c.value)&&!o.load()&&!o.io->fill_gp(original[0].tags,original[0].gp)&&publish_calls==prior,"retirement blocks prepare/refill/GP replay without control");}
    for(unsigned mode=0;mode<3;++mode){TestOwner o;Command c{};bool ready=true;
        for(unsigned i=0;i<2;++i){ready=ready&&o.capture(i,c)&&o.io->execute(c.value);li::LocalExecutionFeedback f{};o.io->take_execution_feedback(f);}
        check(ready,"actual second-source GP-required state prepared");const auto old_calls=publish_calls;uint64_t tags[2]={original[1].tags[0],original[1].tags[1]};double gp[18];predict(original[1].request+1,gp);
        if(mode==0)tags[1]++;if(mode==1)tags[0]--;if(mode==2)o.io->stop();
        check(!o.io->fill_gp(tags,gp)&&publish_calls==old_calls,"wrong GP source tags or late retired reply cannot trigger next control");}
    FreeLibrary(dll);std::printf("{\"checks\":%u,\"failed\":%u,\"actual_LocalIo_rows\":%u,\"actual_GP_calls\":%u,\"real_private74\":true,\"HRT_uORB_Authority_publication_provenance_MOCK\":true,\"ABI_shim_failure_injection_disclosed\":true,\"board_runtime\":false,\"COM\":0}\n",checks,failed,completed,gps);return failed?1:0;
}
