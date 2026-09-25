// Actual Px4CanonicalLocalIo.cpp + facade/private74 + original GP DLL.
// Broker, HRT, Authority and source/context/rotor provenance are HOST mocks.
// The ABI-call shim forwards calls and permits explicit
// failure injection at the joint-install boundary. No socket/model/device.
#define wmain retained_abi_test_main
#include "full_inner_abi/test_full_inner_abi.cpp"
#undef wmain
#include "px4_runtime/CanonicalLocalExecutionCycle.hpp"
#include "local_source_ingress/Px4OriginalHilReceipt.hpp"
#include <common/mavlink.h>
#include <deque>
#include <memory>
#include <limits>
namespace rr=gpenmpc_hil_endpoint_reader;namespace pc=gpenmpc_local_phase;
namespace li=gpenmpc_rfly_px4;namespace fc=gpenmpc_full_consumption;namespace ib=gpenmpc_local_input;namespace od=gpenmpc_odometry;
const orb_metadata __orb_vehicle_odometry{0,sizeof(vehicle_odometry_s),1};
const orb_metadata __orb_vehicle_status{1,sizeof(vehicle_status_s),1};
const orb_metadata __orb_vehicle_control_mode{2,sizeof(vehicle_control_mode_s),1};
const orb_metadata __orb_offboard_control_mode{3,sizeof(offboard_control_mode_s),1};
const orb_metadata __orb_actuator_outputs_rfly{4,sizeof(actuator_outputs_s),1};
const orb_metadata __orb_gpenmpc_original_hil_receipt{5,sizeof(gpenmpc_original_hil_receipt_s),16};
const orb_metadata __orb_vehicle_attitude{13,sizeof(vehicle_attitude_s),1};
const orb_metadata __orb_vehicle_local_position{14,sizeof(vehicle_local_position_s),1};
struct Queued { gpenmpc_original_hil_receipt_s value; uint32_t generation; };
static std::deque<Queued> receipt_queue;static uint32_t receipt_generation;
static Queued delayed_receipt{};static bool release_receipt_on_odometry_copy{};
static rr::Px4OriginalHilReceiptReader*active_reader{};
static pc::CanonicalLocalPhaseClock*active_phase{};
static bool phase_delay_injected{};
static int fixture_link,fixture_receiver;static unsigned actual_parses{};
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
extern "C" uint64_t hrt_absolute_time()noexcept{
    if(injection==10&&active_phase&&active_phase->diagnostics().installs&&!phase_delay_injected){clock_us+=6000;phase_delay_injected=true;}
    ++clock_calls;return clock_us++;
}
bool gpenmpc_test_topic_copy(const orb_metadata*m,void*out,uint32_t&gen,uint8_t instance)noexcept{
    if(!m||m->id>3||instance||!bus_generation[m->id])return false;
    if(m->id==0&&release_receipt_on_odometry_copy){
        receipt_queue.push_back(delayed_receipt);release_receipt_on_odometry_copy=false;
    }
    const void*p=m->id==0?static_cast<void*>(&bus_source):m->id==1?static_cast<void*>(&bus_status):m->id==2?static_cast<void*>(&bus_mode):static_cast<void*>(&bus_offboard);
    std::memcpy(out,p,m->size);gen=bus_generation[m->id];return true;
}
bool gpenmpc_test_topic_update(const orb_metadata*m,void*out,uint32_t&gen,uint8_t instance)noexcept{
    if(m==ORB_ID(gpenmpc_original_hil_receipt)&&!instance&&!receipt_queue.empty()){
        const auto q=receipt_queue.front();receipt_queue.pop_front();std::memcpy(out,&q.value,sizeof q.value);gen=q.generation;return true;}
    return m&&m->id<4&&gen!=bus_generation[m->id]&&gpenmpc_test_topic_copy(m,out,gen,instance);
}
bool gpenmpc_test_topic_publish(const orb_metadata*m,const void*data)noexcept{
    if(m==ORB_ID(gpenmpc_original_hil_receipt)){
        if(receipt_queue.size()>=16)return false;
        Queued q{};std::memcpy(&q.value,data,sizeof q.value);q.generation=++receipt_generation;receipt_queue.push_back(q);return true;}
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
        if(injection==8)out->reference.source_generation++;
        if(injection==7)clock_us+=6000; // real original state read, late return
    }
    return result;
}
static fc::Identity identity(){return {0x1122334455667788ULL,42,1,1};}
struct MockAuthority:li::Authority{
    bool available=true,validation_ok=true,lease_ok=true,confirmation_ok=true,revoked=false;
    uint64_t deadline{},validations{},leases{},confirmations{},revocations{};
    li::Token token{};actuator_outputs_s output{};
    bool observed_identity(li::Identity&out)noexcept override{out=identity();return available&&!revoked;}
    bool validate(const li::Token&t,const vehicle_status_s&,const vehicle_control_mode_s&,const offboard_control_mode_s&,uint64_t,uint64_t&until)noexcept override{
        ++validations;event('V');token=t;until=deadline;return validation_ok;}
    bool install_lease(const li::Token&t,const actuator_outputs_s&v,uint64_t until)noexcept override{
        ++leases;event('L');token=t;output=v;check(until<=deadline,"lease expiry never extends mocked authority original bound");return lease_ok;}
    bool confirm_publication(const li::Token&t,uint64_t pub,uint64_t installed)noexcept override{
        ++confirmations;event('F');check(t==token&&pub<=installed&&active_io&&active_io->diagnostics().consumption_commits>0&&active_io->diagnostics().joint_installs>0,
            "actual consumption and real joint install precede authority confirmation");
        if(injection==5)clock_us+=6000;
        if(injection==9&&active_reader)active_reader->stop();return confirmation_ok;}
    void revoke()noexcept override{revoked=true;++revocations;event('R');}
};
static fc::Configuration config_consumption(){fc::Configuration c{};c.identity=identity();c.numerical=config;gpenmpc_full_inner_identity(&c.build);
    c.limits={5000,100000,400000,5000,gpenmpc_consumption::PublicationPath::DirectCanonicalMotors};return c;}
static od::Configuration config_source(){od::Configuration c{};c.identity=identity();c.vehicle_odometry_topic=ORB_ID(vehicle_odometry);
    c.coordinates=od::Coordinates::ExplicitTranslatedNed;c.sample_max_age_us=5000;return c;}
static void reset_bus(){std::memset(bus_generation,0,sizeof bus_generation);clock_us=1;clock_calls=publish_calls=abi_calls=0;injection=0;timeline_n=0;publish_success=true;active_io=nullptr;active_reader=nullptr;active_phase=nullptr;phase_delay_injected=false;receipt_queue.clear();receipt_generation=100;delayed_receipt={};release_receipt_on_odometry_copy=false;}
static void source(unsigned index,bool disarmed=false){bus_source=raw_fixture[index];bus_generation[0]=static_cast<uint32_t>(original[index].tags[1]);
    bus_status={};bus_mode={};bus_offboard={};const auto stamp=bus_source.timestamp_sample+150;
    bus_status.timestamp=bus_mode.timestamp=bus_offboard.timestamp=stamp;
    bus_status.hil_state=vehicle_status_s::HIL_STATE_ON;bus_status.nav_state=vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
    bus_status.arming_state=disarmed?vehicle_status_s::ARMING_STATE_DISARMED:vehicle_status_s::ARMING_STATE_ARMED;
    bus_mode.flag_armed=!disarmed;bus_mode.flag_control_offboard_enabled=!disarmed;bus_offboard.direct_actuator=true;
    bus_generation[1]=bus_generation[2]=bus_generation[3]=index+1;clock_us=bus_source.timestamp_sample+300;
}
struct Command {li::LocalCommand value{};ib::BoundRotorLag rotor{};ib::BoundPayload payload{};ib::BoundWind wind{};ib::ExplicitInitialInterval initial{};};
static bool command(const od::Snapshot&snapshot,unsigned i,Command&out){
    const auto*s=&snapshot;li::LocalTicket t{};t.source_generation=s->estimator().generation;t.sample_us=s->estimator().timestamp_sample_us;auto&c=out.value;c.ticket=t;auto&cx=c.input_context;
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
static bool load_private(const wchar_t*path){FILE*f=_wfopen(path,L"rb");if(!f)return false;char magic[4];uint32_t n{},bytes{};
    bool ok=read(f,magic,4)&&same(magic,"SJC1",4)&&read(f,&n,4)&&n==60&&read(f,&bytes,4)&&bytes==sizeof(vehicle_odometry_s);
    for(unsigned i=0;i<60&&ok;++i){Original row{};ok=read(f,&raw_fixture[i],bytes)&&read(f,&row,sizeof row)&&same(&row,&original[i],sizeof row);}std::fclose(f);return ok;}
static bool parse_hash(const wchar_t*s){if(std::wcslen(s)!=64)return false;for(unsigned i=0;i<64;++i){unsigned v=16;const auto c=s[i];
    if(c>=L'0'&&c<=L'9')v=c-L'0';if(c>=L'A'&&c<=L'F')v=c-L'A'+10;if(v>15)return false;fixture_hash[i/8]=(fixture_hash[i/8]<<4)|v;}return true;}

static rr::Configuration reader_config(){rr::Configuration c{};c.expected_registered_link=&fixture_link;c.expected_receiver_instance=2;c.expected_channel=3;
    c.expected_noui_system=255;c.expected_noui_component=0;c.independently_observed_board_identity=identity();return c;}
static pc::Configuration phase_config(){pc::Configuration c{};c.identity=identity();c.configuration_sha256=fc::detail::words(config.configuration_sha256);
    std::memcpy(c.reference_asset_sha256,config.reference_asset_sha256,32);c.leg_index=config.leg_index;
    c.duration_s=windows[rows[0].ids[0]-1].total_duration_s;c.rate_min=.70;c.rate_max=1.08;return c;}
static li::LocalCycleConfiguration cycle_config(){li::LocalCycleConfiguration c{};auto&x=c.context;x.observed_session=identity();
    x.explicit_leg=config.leg_index;x.task_sha256=fc::detail::words(config.task_sha256);x.configuration_sha256=fc::detail::words(config.configuration_sha256);
    x.reference_asset_sha256=fc::detail::words(config.reference_asset_sha256);c.initial_interval.configuration_sha256=x.configuration_sha256;
    c.initial_interval.original_configuration_receipt_sha256=fixture_hash;c.initial_interval.leg=x.explicit_leg;
    c.initial_interval.configured_dt_s=rows[0].args[9];c.reference_max_age_us=100000;return c;}
static bool emit_receipt(gpenmpc_hil_endpoint::Px4OriginalHilReceipt&p,uint64_t timestamp){
    mavlink_message_t sent{},parsed{},rx{};mavlink_status_t status{},reported{};
    mavlink_msg_hil_sensor_pack(255,0,&sent,UINT64_C(1788693896516000),1.25f,-2.5f,3.75f,4.25f,-5.5f,6.75f,7,8,9,10,11,12,13,0x1fffU,0);
    uint8_t wire[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(wire,&sent);unsigned accepted=0;
    for(unsigned j=0;j<n;++j)if(mavlink_frame_char_buffer(&rx,&status,wire[j],&parsed,&reported)==MAVLINK_FRAMING_OK)++accepted;
    if(accepted!=1||parsed.msgid!=MAVLINK_MSG_ID_HIL_SENSOR)return false;
    ++actual_parses;mavlink_hil_sensor_t hil{};mavlink_msg_hil_sensor_decode(&parsed,&hil);
    return p.record(parsed,hil,timestamp,&fixture_receiver,&fixture_link,2,3,_MAV_PAYLOAD(&parsed),true,true);
}
struct StepInputs {Command backing{};li::LocalCycleInputs value{};};
struct CycleOwner {
    MockAuthority authority;gpenmpc_hil_endpoint::Px4OriginalHilReceipt producer;
    std::unique_ptr<li::Px4CanonicalLocalIo>io;std::unique_ptr<rr::Px4OriginalHilReceiptReader>reader;
    std::unique_ptr<pc::CanonicalLocalPhaseClock>phase;std::unique_ptr<li::CanonicalLocalExecutionCycle>cycle;std::vector<uint64_t>scratch;
    explicit CycleOwner(bool window=true,bool no_authority=false,const od::Configuration*source_override=nullptr,bool component=false,bool runtime_state=false):scratch((gpenmpc_full_inner_window_scratch_bytes()+7)/8){
        reset_bus();io.reset(new li::Px4CanonicalLocalIo(source_override?*source_override:config_source(),config_consumption(),100000,no_authority?nullptr:&authority));
        active_io=io.get();reader.reset(new rr::Px4OriginalHilReceiptReader(reader_config()));active_reader=reader.get();
        phase.reset(new pc::CanonicalLocalPhaseClock(phase_config()));active_phase=phase.get();
        auto cc=cycle_config();cc.component_initialization=component;cc.context.allow_valid_held_inputs=component;
        cc.runtime_state_only=runtime_state;
        cycle.reset(new li::CanonicalLocalExecutionCycle(*io,*phase,*reader,cc));
        if(window)check(load_window(false),"actual initial resident window loaded");
    }
    bool load_window(bool next){auto w=windows[rows[0].ids[0]-1];if(next)++w.window_generation;
        return io->load_window(w,scratch.data(),scratch.size()*8);}
    bool capture(unsigned i,StepInputs&out,bool receipt=true){
        source(i);authority.deadline=raw_fixture[i].timestamp_sample+5000;
        if(receipt&&!emit_receipt(producer,bus_source.timestamp_sample))return false;
        if(cycle->capture()!=li::LocalCapture::Accepted||!cycle->snapshot()||!command(*cycle->snapshot(),i,out.backing))return false;
        out.value.rotor=&out.backing.rotor;out.value.payload=&out.backing.payload;out.value.wind=&out.backing.wind;
        out.value.outer=out.backing.value.outer_envelope;out.value.target4[0]=rows[i].args[2];
        for(unsigned j=0;j<3;++j)out.value.target4[j+1]=rows[i].args[6+j];
        out.value.loaded_window_generation=windows[rows[0].ids[0]-1].window_generation;return true;
    }
};
static bool fill_actual(CycleOwner&o,const li::LocalCycleFeedback&f,int(*predict)(const double*,double*),unsigned&calls){
    gpenmpc_full_inner_diagnostics d{};if(!o.cycle->numerical_diagnostics(d))return false;if(!d.prediction_required)return true;
    double gp[18];if(predict(f.actual.request19+1,gp)!=0)return false;
    const uint64_t tags[2]={f.actual.token.lease_envelope.timestamp_sample_us*1000,f.actual.token.lease_envelope.sample_generation};
    ++calls;return o.cycle->fill_gp(tags,gp);
}
static li::LocalCapture observe(CycleOwner&o,unsigned i,bool receipt=true){
    source(i,true);o.authority.deadline=raw_fixture[i].timestamp_sample+5000;
    if(receipt&&!emit_receipt(o.producer,bus_source.timestamp_sample))return li::LocalCapture::Rejected;
    return o.cycle->capture_disarmed();
}
static bool no_math_or_publication(CycleOwner&o){
    gpenmpc_full_inner_diagnostics d{};
    return o.io->numerical_diagnostics(d)&&!d.kernel_calls&&!d.numeric_installs&&!d.reference_installs&&!d.prediction_fills&&!d.joint_attempts&&
        !d.reported_publications&&!o.io->diagnostics().reference_prepares&&!o.io->diagnostics().kernel_prepare_attempts&&!o.io->diagnostics().gp_fills&&
        !o.phase->diagnostics().queries&&!o.phase->diagnostics().installs&&!publish_calls&&!abi_calls;
}
#ifndef GPENMPC_CYCLE_TEST_MAIN
#define GPENMPC_CYCLE_TEST_MAIN wmain
#endif
int GPENMPC_CYCLE_TEST_MAIN(int argc,wchar_t**argv){
    if(argc!=5||!load(argv[1],argv[2])||!load_private(argv[2])||!parse_hash(argv[4]))return 2;
    HMODULE dll=LoadLibraryExW(argv[3],nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);if(!dll)return 3;
    using Predict=int(*)(const double*,double*);const auto predict=reinterpret_cast<Predict>(GetProcAddress(dll,"gpenmpc_gp256_predict"));if(!predict)return 3;
    unsigned completed=0,gps=0,raw_after_failure=0;
    {CycleOwner o;
        for(unsigned j=0;j<2000;++j)check(o.cycle->capture()==li::LocalCapture::NoUpdate&&o.phase->diagnostics().installs==0&&publish_calls==0,"no source cannot advance phase or execute");
        for(unsigned i=0;i<60;++i){StepInputs in{};const auto before=o.phase->diagnostics();
            check(o.capture(i,in),"actual cycle capture reader endpoint private source");
            const auto*ep=o.cycle->original_endpoint();
            check(ep&&ep->endpoint.exact_unique_endpoint&&!ep->receiver_to_estimator_lineage_proven&&!ep->dll_association_proven&&!ep->control_authority,
                "real endpoint remains endpoint-only, no authority or rotor lineage inferred");
            check(o.phase->diagnostics().installs==i&&same(&before.phase_s,&o.phase->diagnostics().phase_s,8),"capture performs no phase advancement");
            timeline_n=0;const auto result=o.cycle->execute(in.value);
            if(result!=li::LocalCycleExecute::Committed)std::fprintf(stderr,"row%u cycle%u io%u phase%u reader%u\n",i,unsigned(o.cycle->diagnostics().first_fault),unsigned(o.io->diagnostics().first_fault),unsigned(o.phase->diagnostics().first_fault),unsigned(o.reader->fault()));
            check(result==li::LocalCycleExecute::Committed,"actual cycle complete C publication joint phase and original receipt retirement");
            if(result!=li::LocalCycleExecute::Committed)break;
            li::LocalCycleFeedback f{},empty{};check(o.cycle->take_feedback(f)&&!o.cycle->take_feedback(empty),"cycle feedback one shot");
            const auto&r=f.actual.committed_reference;const auto&t=f.actual.token.lease_envelope;
            check(f.present&&f.usable_at_read&&f.actual.fresh_at_read&&f.phase_installed&&f.source_receipt_retired&&f.actual.reference_state_copied,
                "historical IO read and current cycle readiness separate and true after complete path");
            check(f.original_cycle_read_us>=f.actual.original_commit_completed_us&&f.original_cycle_read_us<=f.actual.original_valid_until_us,
                "actual cycle read uses original immutable deadline");
            check(same_reference_state(r,actual_copied_reference)&&r.source_generation==original[i].tags[1]&&r.source_timestamp_ns==original[i].tags[0]&&
                r.reference_generation==i+1&&r.outer_generation==i+1&&r.publication_us==f.actual.original_publication_us,"actual committed reference source and publication copied exactly");
            check(same(f.actual.actual_control16,actual_output.output,64)&&same(f.actual.prepared_control16,actual_output.output,64)&&actual_output.noutputs==0,
                "actual prepared and mock-published official16 bit exact, vendor zero noutputs not relabeled");
            const double x[7]={before.phase_s,before.phase_rate,r.phase_acceleration,original[i].input[34],phase_config().duration_s,.70,1.08};
            double expected[2];gpenmpcNative_canonicalLocalPhaseAdvance(x,expected);
            check(same(&o.phase->diagnostics().phase_s,expected,8)&&same(&o.phase->diagnostics().phase_rate,expected+1,8)&&
                same(&r.query_progress_s,&before.phase_s,8)&&same(&r.progress_rate,&before.phase_rate,8),"phase advanced by real generated C from actual committed bounded transition, not target command");
            check(t.sample_generation==original[i].tags[1]&&t.timestamp_sample_us*1000==original[i].tags[0]&&
                timeline_n==5&&same(timeline,"VLPCF",5),"original source tags and actual validate lease publish joint confirm order");
            rr::Diagnostics d{};check(o.reader->diagnostics(d)&&d.retained==0&&d.retired_records==i+1&&d.unknown_prebaseline,
                "actual reader exact endpoint retired only after real publication/phase install");
            check(o.io->diagnostics().kernel_calls==i+1&&o.io->diagnostics().joint_installs==i+1&&publish_calls==i+1&&
                o.phase->diagnostics().installs==i+1&&o.cycle->diagnostics().commits==i+1,"all five real completion denominators agree");
            const auto after=o.phase->diagnostics();const auto pubs=publish_calls;
            check(fill_actual(o,f,predict,gps)&&pubs==publish_calls&&same(&after.phase_s,&o.phase->diagnostics().phase_s,8)&&
                same(&after.phase_rate,&o.phase->diagnostics().phase_rate,8),"actual GP return fills pending only; no control or phase advance");
            check(o.cycle->capture()==li::LocalCapture::NoUpdate&&o.phase->diagnostics().installs==i+1,"polling without new source cannot advance");
            ++completed;
        }
        check(completed==60&&gps==59&&abi_calls==60&&clock_calls>60&&o.authority.confirmations==60,"real cycle60 C/reference/reader and actualGP59 chain");
    }
    {CycleOwner o(false);StepInputs in{};check(o.capture(0,in),"real window miss capture");
        check(o.cycle->execute(in.value)==li::LocalCycleExecute::NeedReferenceWindow&&o.phase->diagnostics().installs==0&&publish_calls==0&&
            o.io->diagnostics().kernel_calls==0,"window miss leaves phase and inner/publication untouched");
        const auto original_creation=o.cycle->diagnostics().original_reference_created_us;
        check(o.load_window(true)&&o.phase->diagnostics().installs==0,"real resident refill alone cannot advance");
        ++in.value.loaded_window_generation;
        check(o.cycle->execute(in.value)==li::LocalCycleExecute::Committed&&o.phase->diagnostics().queries==1&&o.phase->diagnostics().window_retries==1&&
            o.io->diagnostics().kernel_calls==1&&o.cycle->diagnostics().original_reference_created_us==original_creation,"same source retry changes window only, one real step and no reference time renewal");
    }
    for(unsigned kind=0;kind<8;++kind){CycleOwner o(false);StepInputs in{};o.capture(0,in);
        check(o.cycle->execute(in.value)==li::LocalCycleExecute::NeedReferenceWindow&&o.load_window(true),"retry negative has real miss then actual refill");
        ++in.value.loaded_window_generation;
        if(kind==0)in.value.target4[0]+=.001;
        if(kind==1)in.value.outer.valid_until_us++;
        if(kind==2)in.backing.rotor.original_observation.observed_thrust_n[0]+=.1;
        if(kind==3)in.backing.payload.payload_kg+=.1;
        if(kind==4)in.backing.wind.estimate_xy_mps[0]+=.1;
        if(kind==5)in.value.wind=nullptr;
        if(kind==6)in.value.loaded_window_generation--;
        if(kind==7)clock_us=raw_fixture[0].timestamp_sample+5001;
        check(o.cycle->execute(in.value)==li::LocalCycleExecute::Rejected&&publish_calls==0&&o.phase->diagnostics().installs==0&&
            o.cycle->diagnostics().first_fault!=li::LocalCycleFault::None,"wrong retry target outer rotor payload wind generation or deadline never publishes");
    }
    for(unsigned mode=0;mode<11;++mode){CycleOwner o;StepInputs in{};check(o.capture(0,in),"fault injection actual cycle source");
        if(mode==0)publish_success=false;else injection=mode;
        const auto result=o.cycle->execute(in.value);
        check(result==li::LocalCycleExecute::Rejected&&o.authority.revoked,"publication joint-copy phase or endpoint-retirement fault rejects");
        li::LocalCycleFeedback f{};check(o.cycle->take_feedback(f)&&!f.usable_at_read,"raw unsuccessful cycle never currently usable");
        check(o.cycle->diagnostics().commits==0,"rejected transaction not counted as complete cycle");
        if(mode>=1){check(f.actual.publication_succeeded&&publish_calls==1,"performed successful publication retained after downstream failure");++raw_after_failure;}
        if(mode==8)check(f.actual.fresh_at_read&&f.actual.numerical_reference_installed&&!f.phase_installed&&!f.source_receipt_retired&&
            o.phase->diagnostics().installs==0,"wrong committed-reference copy rejects phase with actual numerical installation retained");
        if(mode==9)check(f.actual.fresh_at_read&&f.actual.numerical_reference_installed&&f.phase_installed&&!f.source_receipt_retired&&
            o.phase->diagnostics().installs==1,"reader retirement failure preserves phase and publication without false rollback");
        if(mode==10)check(f.actual.fresh_at_read&&f.phase_installed&&f.source_receipt_retired&&phase_delay_injected&&
            o.cycle->diagnostics().first_fault==li::LocalCycleFault::Clock,"late completion retains actual phase and source retirement but refuses completed-cycle success");
        const auto first=o.cycle->diagnostics().first_fault;const auto pubs=publish_calls;
        check(o.cycle->execute(in.value)==li::LocalCycleExecute::Rejected&&o.cycle->capture()==li::LocalCapture::Rejected&&
            o.cycle->diagnostics().first_fault==first&&publish_calls==pubs,"first fault and actual call counts survive retries");
    }
    for(unsigned mode=0;mode<5;++mode){CycleOwner o;StepInputs in{};o.capture(0,in);
        check(o.cycle->execute(in.value)==li::LocalCycleExecute::Committed,"completed cycle before lifecycle/read-time negative");
        if(mode==0)o.cycle->stop();if(mode==1)clock_us=raw_fixture[0].timestamp_sample+5001;
        if(mode==2)o.reader->stop();if(mode==3)o.phase->retire();
        if(mode==4)o.io->stop();
        li::LocalCycleFeedback f{};check(o.cycle->take_feedback(f)&&!f.usable_at_read&&f.actual.fresh_at_read&&f.actual.publication_succeeded&&
            f.phase_installed&&f.source_receipt_retired,"late or revoked reading preserves original historical flags but denies current use");
    }
    for(unsigned mode=0;mode<4;++mode){CycleOwner o;StepInputs in{};
        if(mode==0){check(!o.capture(0,in,false)&&publish_calls==0,"missing exact original receipt refuses private source");continue;}
        if(mode==1){source(0);emit_receipt(o.producer,bus_source.timestamp_sample);receipt_queue.front().value.link_address++;
            check(o.cycle->capture()==li::LocalCapture::Rejected&&publish_calls==0,"competing receiver-link record cannot enter cycle");continue;}
        o.capture(0,in);if(mode==2)in.value.outer.valid_until_us=clock_us-1;if(mode==3)clock_us=raw_fixture[0].timestamp_sample+5001;
        check(o.cycle->execute(in.value)==li::LocalCycleExecute::Rejected&&publish_calls==0&&o.phase->diagnostics().installs==0,"expired outer or original source cannot run control");
    }
    {CycleOwner o(true,true);StepInputs in{};check(!o.capture(0,in)&&publish_calls==0,"missing real Authority stays refused");}
    {CycleOwner o;StepInputs in{};o.capture(0,in);o.cycle->stop();const auto n=publish_calls;
        check(o.cycle->execute(in.value)==li::LocalCycleExecute::Rejected&&!o.load_window(false)&&
            !o.io->fill_gp(original[0].tags,original[0].gp)&&publish_calls==n,"retirement prevents source window and GP revival");}
    {CycleOwner o;li::LocalCycleFeedback f{};
        for(unsigned i=0;i<2;++i){StepInputs in{};check(o.capture(i,in)&&o.cycle->execute(in.value)==li::LocalCycleExecute::Committed&&o.cycle->take_feedback(f),"GP missing-next-source setup");}
        gpenmpc_full_inner_diagnostics d{};o.io->numerical_diagnostics(d);check(d.prediction_required,"actual second output creates required pending GP");
        StepInputs next{};check(o.capture(2,next)&&o.cycle->execute(next.value)==li::LocalCycleExecute::Rejected&&
            publish_calls==2&&o.phase->diagnostics().installs==2,"unfilled prior prediction refuses next control and phase advance");
        double gp[18];predict(f.actual.request19+1,gp);const uint64_t tags[2]={f.actual.token.lease_envelope.timestamp_sample_us*1000,f.actual.token.lease_envelope.sample_generation};
        check(!o.io->fill_gp(tags,gp)&&publish_calls==2,"late GP cannot revive rejected cycle");
    }
    for(unsigned kind=0;kind<4;++kind){CycleOwner o;li::LocalCycleFeedback f{};
        for(unsigned i=0;i<2;++i){StepInputs in{};check(o.capture(i,in)&&o.cycle->execute(in.value)==li::LocalCycleExecute::Committed&&o.cycle->take_feedback(f),"same-owner GP method negative setup");}
        gpenmpc_full_inner_diagnostics before{},after{};const auto clock_before=clock_calls;
        check(o.cycle->numerical_diagnostics(before)&&clock_calls==clock_before&&before.prediction_required,"same-owner numerical diagnostics uses no new HRT");
        uint64_t tags[2]={f.actual.token.lease_envelope.timestamp_sample_us*1000,f.actual.token.lease_envelope.sample_generation};double gp[18];predict(f.actual.request19+1,gp);
        if(kind==0)tags[0]++;if(kind==1)tags[1]++;
        if(kind==2){StepInputs in{};check(o.capture(2,in),"new private source pending before late GP reply");}
        if(kind==3)o.cycle->stop();
        check(!o.cycle->fill_gp(tags,gp)&&o.cycle->numerical_diagnostics(after)&&after.prediction_fills==before.prediction_fills&&
            o.phase->diagnostics().installs==2&&publish_calls==2,"wrong source tags, pending source or stopped cycle never fills another/late owner or executes");
        const auto fills=after.prediction_fills;
        check(!o.cycle->fill_gp(original[1].tags,gp)&&o.cycle->numerical_diagnostics(after)&&after.prediction_fills==fills,"rejected GP method is first-fault closed");
    }
    unsigned observations=0;
    {CycleOwner o;
        source(0,true);o.authority.deadline=raw_fixture[0].timestamp_sample+5000;
        check(emit_receipt(o.producer,bus_source.timestamp_sample)&&receipt_queue.size()==1,
            "delayed exact receipt fixture prepared without board or approximate time");
        delayed_receipt=receipt_queue.front();receipt_queue.pop_front();release_receipt_on_odometry_copy=true;
        check(o.cycle->capture_disarmed()==li::LocalCapture::Accepted,
            "bounded post-capture drain accepts exact receipt published during odometry capture");
        rr::Diagnostics d{};check(o.reader->diagnostics(d)&&d.accepted_records==1&&d.successful_lookups==1&&
            o.cycle->diagnostics().first_fault==li::LocalCycleFault::None&&no_math_or_publication(o),
            "post-capture drain preserves exact lookup and performs no control or publication");
        check(o.cycle->release_disarmed()&&no_math_or_publication(o),
            "delayed exact startup observation retires through the unchanged disarmed release path");
    }
    {CycleOwner o;
        check(!o.io->retained_latest_snapshot(),"no observed source cannot fabricate a retained latest snapshot");
        for(unsigned i=0;i<3;++i){
            check(observe(o,i)==li::LocalCapture::Accepted,"actual disarmed capture through original reader and Io");
            od::Snapshot exported{};rr::Receipt endpoint{};const auto clocks=clock_calls;
            check(o.cycle->copy_disarmed_observation(exported,endpoint)&&clock_calls==clocks,"observation export copies original evidence without clock resampling");
            const auto*actual=o.cycle->snapshot();const auto*retained=o.io->retained_latest_snapshot();
            check(actual&&retained&&exported.valid()&&same(&exported.raw(),&raw_fixture[i],sizeof(vehicle_odometry_s))&&
                same(&retained->raw(),&exported.raw(),sizeof(vehicle_odometry_s))&&exported.estimator().board_rx_us==actual->estimator().board_rx_us&&
                exported.estimator().timestamp_sample_us==raw_fixture[i].timestamp_sample&&exported.estimator().publication_us==raw_fixture[i].timestamp&&
                exported.subscription_generation()==original[i].tags[1]&&exported.source_topic()==ORB_ID(vehicle_odometry)&&exported.source_instance()==0,
                "export retains actual private raw bits, exact source topic/generation and original board time");
            check(endpoint.endpoint.exact_unique_endpoint&&endpoint.original.topic.timestamp==raw_fixture[i].timestamp_sample&&
                !endpoint.receiver_to_estimator_lineage_proven&&!endpoint.dll_association_proven&&!endpoint.control_authority&&!exported.board_authority(),
                "disarmed observation does not upgrade endpoint or copied snapshot to permission/lineage");
            check(no_math_or_publication(o),"disarmed capture/export performs zero reference GP or inner state operation");
            const auto original_rx=exported.estimator().board_rx_us;
            // Release validates identity and disarmed status and retains source time.
            if(i==0)clock_us=raw_fixture[i].timestamp_sample+6000;
            check(o.cycle->release_disarmed()&&no_math_or_publication(o),"actual release retires observation only, even without renewing stale source");
            rr::Diagnostics d{};check(o.reader->diagnostics(d)&&d.retained==0&&d.retired_records==i+1&&
                o.io->diagnostics().observations_released==i+1&&o.cycle->diagnostics().observation_io_releases==i+1&&
                o.cycle->diagnostics().observation_endpoint_retirements==i+1,"actual Io release precedes exact reader retirement with separate counters");
            const auto calls=clock_calls;retained=o.io->retained_latest_snapshot();
            check(retained&&retained->estimator().board_rx_us==original_rx&&retained->estimator().timestamp_sample_us==raw_fixture[i].timestamp_sample&&
                clock_calls==calls&&!o.cycle->snapshot()&&!o.cycle->original_endpoint()&&
                !o.cycle->copy_disarmed_observation(exported,endpoint)&&!exported.valid(),"released latest remains historical; active export closed without retimestamping");
            check(o.cycle->capture_disarmed()==li::LocalCapture::NoUpdate&&no_math_or_publication(o),"disarmed poll with no new source advances no scientific state");
            ++observations;
        }
        StepInputs flight{};check(o.capture(3,flight)&&o.cycle->execute(flight.value)==li::LocalCycleExecute::Committed,
            "new genuine armed source can perform first control after three disarmed observations");
        li::LocalCycleFeedback f{};check(o.cycle->take_feedback(f)&&f.usable_at_read&&f.actual.token.lease_envelope.sample_generation==original[3].tags[1]&&
            f.actual.token.lease_envelope.timestamp_sample_us*1000==original[3].tags[0]&&f.actual.committed_reference.reference_generation==1&&
            o.phase->diagnostics().queries==1&&o.phase->diagnostics().installs==1&&o.io->diagnostics().kernel_calls==1,
            "disarmed source counts do not become reference/inner generations or pre-advance phase");
        check(observe(o,4)==li::LocalCapture::Rejected&&publish_calls==1,"post-control ground observation requires new lifetime, not silent numerical reuse");
    }
    for(unsigned mode=0;mode<10;++mode){CycleOwner o;od::Snapshot exported{};rr::Receipt endpoint{};
        if(mode==0){source(0);emit_receipt(o.producer,bus_source.timestamp_sample);
            check(o.cycle->capture_disarmed()==li::LocalCapture::Rejected&&no_math_or_publication(o),"armed source cannot enter disarmed preparation");continue;}
        if(mode==1){o.authority.available=false;check(observe(o,0)==li::LocalCapture::Rejected&&no_math_or_publication(o),"unobserved identity cannot capture disarmed source");continue;}
        if(mode==2){check(observe(o,0,false)==li::LocalCapture::Rejected&&no_math_or_publication(o),"missing original endpoint cannot export disarmed source");continue;}
        check(observe(o,0)==li::LocalCapture::Accepted&&o.cycle->copy_disarmed_observation(exported,endpoint),"release negative retains actual original observation");
        const auto original_rx=exported.estimator().board_rx_us;
        if(mode==3){bus_status.arming_state=vehicle_status_s::ARMING_STATE_ARMED;bus_mode.flag_armed=true;++bus_generation[1];++bus_generation[2];}
        if(mode==4)o.authority.available=false;
        if(mode==5)clock_us+=100001;
        if(mode==6)o.reader->stop();
        if(mode==7){StepInputs control{};check(o.cycle->execute(control.value)==li::LocalCycleExecute::Rejected&&no_math_or_publication(o),
            "disarmed ticket cannot trigger even a reference query");continue;}
        if(mode==8){check(o.cycle->release_disarmed(),"duplicate-release setup actually retired once");}
        if(mode==9){o.cycle->stop();}
        check(!o.cycle->release_disarmed()&&no_math_or_publication(o)&&!o.cycle->copy_disarmed_observation(exported,endpoint),
            "armed identity stale stopped duplicate or endpoint retirement failure refuses export/release reuse");
        const auto*retained=o.io->retained_latest_snapshot();check(retained&&retained->estimator().board_rx_us==original_rx&&
            same(&retained->raw(),&raw_fixture[0],sizeof(vehicle_odometry_s)),"fault preserves genuine historical latest snapshot without new validity claim");
        if(mode==6)check(o.io->diagnostics().observations_released==1&&o.cycle->diagnostics().observation_io_releases==1&&
            o.cycle->diagnostics().observation_endpoint_retirements==0,"failed endpoint retirement does not erase actual successful Io release");
    }
    {CycleOwner o;check(observe(o,0)==li::LocalCapture::Accepted&&o.cycle->release_disarmed(),"valid historical source before rejected later ingest");
        source(1,true);bus_source.q[0]=std::numeric_limits<float>::quiet_NaN();emit_receipt(o.producer,bus_source.timestamp_sample);
        check(o.cycle->capture_disarmed()==li::LocalCapture::Rejected,"later malformed raw source rejected");
        const auto*s=o.io->retained_latest_snapshot();check(s&&same(&s->raw(),&raw_fixture[0],sizeof(vehicle_odometry_s))&&no_math_or_publication(o),
            "malformed ingest preserves the latest validated snapshot");}
    check(observations==3,"three real disarmed export/release transactions completed before first flight control");
    {CycleOwner o;check(observe(o,0)==li::LocalCapture::Accepted,"disarmed pending GP gate setup");
        check(!o.cycle->fill_gp(original[0].tags,original[0].gp)&&no_math_or_publication(o)&&
            o.cycle->diagnostics().first_fault==li::LocalCycleFault::Pending,"pending disarmed observation cannot accept GP state or run control");}
    FreeLibrary(dll);
    std::printf("{\"checks\":%u,\"failed\":%u,\"actual_cycle_rows\":%u,\"actual_GP_calls\":%u,\"actual_MAVLink_pack_parse\":%u,\"raw_postpublication_failures_retained\":%u,\"disarmed_observation_export_release\":%u,\"sizeof_cycle\":%zu,\"added_persistent_latest_snapshot_bytes\":%zu,\"real_private74_and_phase_C\":true,\"uORB_HRT_Authority_provenance_MOCK\":true,\"ABI_fault_injection_shims_disclosed\":true,\"COM\":0}\n",
        checks,failed,completed,gps,actual_parses,raw_after_failure,observations,sizeof(li::CanonicalLocalExecutionCycle),sizeof(od::Snapshot));
    return failed?1:0;
}
