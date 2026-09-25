#include "px4_runtime/CanonicalLocalExchangeCore.hpp"
#include <cstdlib>
#define gpenmpc_test_topic_copy retained_topic_copy
#define gpenmpc_test_topic_update retained_topic_update
#define gpenmpc_test_topic_publish retained_topic_publish
#define GPENMPC_GP_PENDING_TEST_MAIN retained_pending_main
#include "test_local_gp_pending.cpp"
#undef GPENMPC_GP_PENDING_TEST_MAIN
#undef gpenmpc_test_topic_copy
#undef gpenmpc_test_topic_update
#undef gpenmpc_test_topic_publish
#include <parameters/param.h>
#undef param_get
namespace ss=gpenmpc_selected_source;
#define META(n,id,q) const orb_metadata __orb_##n{id,sizeof(n##_s),q}
META(sensor_gyro,6,8);META(sensor_accel,7,8);META(vehicle_imu,8,1);
META(sensor_selection,9,1);META(sensors_status_imu,10,1);META(sensor_combined,11,1);META(gpenmpc_full_inner_ingress,12,gpenmpc_full_inner_ingress_s::ORB_QUEUE_LENGTH);
META(vehicle_thrust_setpoint,15,1);
struct ExtraRecord{std::vector<unsigned char>bytes;uint32_t generation;};
struct ExtraTopic{std::deque<ExtraRecord>queue;uint32_t generation=100;};
static ExtraTopic extra[16][4];static int32_t params[3]={1,0,0};static unsigned core_wire_packets{},task_calls{};
extern "C" param_t param_find_no_notification(const char*n){const char*names[]={"SENS_IMU_MODE","EKF2_MULTI_IMU","EKF2_MULTI_MAG"};for(unsigned i=0;i<3;++i)if(!std::strcmp(n,names[i]))return i;return PARAM_INVALID;}
extern "C" param_type_t param_type(param_t){return PARAM_TYPE_INT32;}
extern "C" int param_get(param_t p,void*out){if(p>=3)return -1;std::memcpy(out,&params[p],4);return 0;}
bool gpenmpc_test_topic_advertised(const orb_metadata*m,uint8_t i)noexcept{return m&&m->id>=6&&m->id<16&&i<4&&!extra[m->id][i].queue.empty();}
bool gpenmpc_test_topic_updated(const orb_metadata*m,uint32_t g,uint8_t i)noexcept{return gpenmpc_test_topic_advertised(m,i)&&extra[m->id][i].queue.back().generation>g;}
bool gpenmpc_test_topic_copy(const orb_metadata*m,void*out,uint32_t&g,uint8_t i)noexcept{
    if(m&&m->id<6)return retained_topic_copy(m,out,g,i);
    if(!gpenmpc_test_topic_advertised(m,i))return false;const auto&r=extra[m->id][i].queue.back();std::memcpy(out,r.bytes.data(),m->size);g=r.generation;return true;
}
bool gpenmpc_test_topic_update(const orb_metadata*m,void*out,uint32_t&g,uint8_t i)noexcept{
    if(m&&m->id<6)return retained_topic_update(m,out,g,i);
    if(!m||m->id>=16||i>=4)return false;for(const auto&r:extra[m->id][i].queue)if(r.generation>g){std::memcpy(out,r.bytes.data(),m->size);g=r.generation;return true;}return false;
}
static void extra_publish(const orb_metadata*m,const void*p,uint8_t i=0){auto&t=extra[m->id][i];if(t.queue.size()>=m->queue)t.queue.pop_front();const auto*b=static_cast<const unsigned char*>(p);t.queue.push_back({std::vector<unsigned char>(b,b+m->size),++t.generation});}
bool gpenmpc_test_topic_publish(const orb_metadata*m,const void*p)noexcept{if(m&&m->id>=6){extra_publish(m,p);return true;}return retained_topic_publish(m,p);}
static void extra_clear(){for(auto&g:extra)for(auto&t:g)t=ExtraTopic{};params[0]=1;params[1]=params[2]=0;task_calls=0;}
static bool original_selected(CycleOwner&o,unsigned i,bool disarmed=false){
    source(i,disarmed);o.authority.deadline=bus_source.timestamp_sample+5000;const auto t=bus_source.timestamp_sample;
    mavlink_message_t msg{};mavlink_msg_hil_sensor_pack(255,0,&msg,123456,1,2,3,4,5,6,7,8,9,10,11,12,13,8191,0);
    mavlink_hil_sensor_t h{};mavlink_msg_hil_sensor_decode(&msg,&h);
    if(!o.producer.record(msg,h,t,&fixture_receiver,&fixture_link,2,3,_MAV_PAYLOAD(&msg),true,true,0,0,1310988,1310988))return false;
    sensor_gyro_s g{};sensor_accel_s a{};vehicle_imu_s v{};sensor_selection_s s{};sensors_status_imu_s st{};sensor_combined_s c{};
    g.timestamp=t+10;g.timestamp_sample=t;g.device_id=1310988;g.x=4;g.y=5;g.z=6;g.samples=1;
    a.timestamp=t+20;a.timestamp_sample=t;a.device_id=1310988;a.x=1;a.y=2;a.z=3;a.samples=1;
    v.timestamp=t+30;v.timestamp_sample=t;v.accel_device_id=v.gyro_device_id=1310988;v.delta_angle_dt=v.delta_velocity_dt=5000;
    for(unsigned k=0;k<3;++k){v.delta_angle[k]=.007f*(k+1);v.delta_velocity[k]=.011f*(k+1);}
    s.timestamp=t+40;s.accel_device_id=s.gyro_device_id=1310988;st.timestamp=t+50;st.accel_device_id_primary=st.gyro_device_id_primary=1310988;st.accel_device_ids[0]=st.gyro_device_ids[0]=1310988;
    c.timestamp=t;c.accelerometer_integral_dt=c.gyro_integral_dt=5000;for(unsigned k=0;k<3;++k){c.gyro_rad[k]=v.delta_angle[k]*(1.e6f/float(v.delta_angle_dt));c.accelerometer_m_s2[k]=v.delta_velocity[k]*(1.e6f/float(v.delta_velocity_dt));}
    extra_publish(ORB_ID(sensor_gyro),&g);extra_publish(ORB_ID(sensor_accel),&a);extra_publish(ORB_ID(vehicle_imu),&v);
    extra_publish(ORB_ID(sensor_selection),&s);extra_publish(ORB_ID(sensors_status_imu),&st);extra_publish(ORB_ID(sensor_combined),&c);return true;
}
struct TaskOwner {
    unsigned index{},bad{},read_delay_us{};bool hold_previous_source{},runtime_state_only{};ib::SnapshotKey previous_input{};
    StepInputs step{};gpenmpc_full_inner_window window{};std::vector<uint64_t>scratch;
    gpenmpc_local_window_wire::Assembler*receiver{};
    TaskOwner():window(windows[rows[0].ids[0]-1]),scratch((gpenmpc_full_inner_window_scratch_bytes()+7)/8){}
    static li::LocalTaskRead get_window(void*p,li::LocalTaskView&out)noexcept{
        auto&s=*static_cast<TaskOwner*>(p);out.window=s.receiver?s.receiver->ready_window(hrt_absolute_time()):&s.window;
        out.window_scratch=s.scratch.data();out.window_scratch_bytes=s.scratch.size()*8;
        return out.window?li::LocalTaskRead::Ready:li::LocalTaskRead::Missing;
    }
    static li::LocalTaskRead read(void*p,const od::Snapshot&s,const ss::Receipt&r,li::LocalTaskView&out)noexcept{
        auto&self=*static_cast<TaskOwner*>(p);++task_calls;
        check(self.runtime_state_only||(r.unique_selected_endpoint_observed&&r.snapshot_sample_us==s.estimator().timestamp_sample_us),"TaskInput receives actual private source and actual selected endpoint receipt or explicit runtime source");
        if(self.bad==1)return li::LocalTaskRead::Missing;
        clock_us+=self.read_delay_us; // Explicit HOST fixture work; production timestamps are never edited.
        if(!command(s,self.index,self.step.backing))return li::LocalTaskRead::Invalid;
        if(self.hold_previous_source&&self.previous_input.sample_us){
            self.step.backing.rotor.source=self.step.backing.payload.source=self.step.backing.wind.source=self.previous_input;
        }else ib::snapshot_key(s,self.previous_input);
        auto&v=self.step.value;v.rotor=&self.step.backing.rotor;v.payload=&self.step.backing.payload;v.wind=&self.step.backing.wind;v.outer=self.step.backing.value.outer_envelope;
        v.target4[0]=rows[self.index].args[2];for(unsigned k=0;k<3;++k)v.target4[k+1]=rows[self.index].args[6+k];v.loaded_window_generation=self.window.window_generation;
        if(self.bad==2)++self.step.backing.rotor.source.source_generation;if(self.bad==3)v.wind=nullptr;if(self.bad==4)++v.outer.payload_sha256[0];
        out.inputs=v;out.window=&self.window;out.window_scratch=self.scratch.data();out.window_scratch_bytes=self.scratch.size()*8;return li::LocalTaskRead::Ready;
    }
};
static li::LocalExchangeConfiguration exchange_config(){li::LocalExchangeConfiguration c{};c.identity=identity();c.transport={255,190,1,1,2,2000};c.gp_request_transport_max_age_us=c.snapshot_transport_max_age_us=2000;
    // This numerical fixture disables snapshot exports; the dedicated
    // live-default tests below exercise active snapshot observation.
    c.export_active_snapshots=false;c.export_committed_state=false;return c;}
static bool drain_outbox(li::CanonicalLocalExchangeCore&core,const void*consumer,gw::RequestBytes*out){
    unsigned pieces=0;std::size_t offset=0;gw::RequestBytes request{};
    for(;;){li::LocalWireFragment f{};auto r=core.outbox().take(consumer,f,hrt_absolute_time());if(r==li::LocalWireTake::Empty)break;
        if(r!=li::LocalWireTake::Fragment)return false;++pieces;const auto n=f.fragment.length-9;
        if(out){if(f.original.schema!=gw::request_schema||offset+n>request.size())return false;std::memcpy(request.data()+offset,f.fragment.payload+9,n);offset+=n;}}
    if(out){if(pieces!=3||offset!=request.size())return false;*out=request;}return pieces!=0;
}
static bool same_observed_state(const gpenmpc_full_inner_state&a,const gpenmpc_full_inner_state&b){
    return a.numeric_installed==b.numeric_installed&&a.reference_committed==b.reference_committed&&
        a.prediction_required==b.prediction_required&&a.prediction_ready==b.prediction_ready&&
        same(a.state64,b.state64,sizeof a.state64)&&same(a.pending70,b.pending70,sizeof a.pending70)&&
        same(a.original_installed_tags2,b.original_installed_tags2,sizeof a.original_installed_tags2)&&
        same_reference_state(a.reference,b.reference)&&same(a.committed_state_sha256,b.committed_state_sha256,32)&&
        same(a.loaded_window_sha256,b.loaded_window_sha256,32);
}
static li::LocalNumericalObservation observe_state(CycleOwner&o){
    li::LocalNumericalObservation first{},second{};gpenmpc_full_inner_diagnostics before{},after{};
    const auto calls=clock_calls,time=clock_us,pubs=publish_calls;const auto phase=o.phase->diagnostics();
    check(o.io->numerical_diagnostics(before)&&o.io->copy_local_numerical_state(first)&&o.io->copy_local_numerical_state(second)&&o.io->numerical_diagnostics(after),"actual owner readonly copies return original state and diagnostics");
    check(clock_calls==calls&&clock_us==time&&publish_calls==pubs&&o.phase->diagnostics().installs==phase.installs,"readonly copy takes no new HRT publishes nothing and never advances phase");
    check(same_observed_state(first.state,second.state)&&same(&before,&after,sizeof before)&&first.kind==second.kind&&
        first.actual_joint_installs==second.actual_joint_installs&&first.actual_gp_fills==second.actual_gp_fills&&
        first.capture_pending==second.capture_pending&&first.unread_execution_feedback==second.unread_execution_feedback,"two reads retain every state/hash/counter and feedback freshness unchanged");
    check(first.copied&&!first.control_authority&&!first.source_freshness_granted,"read-only observation leaves control and freshness flags clear");
#ifndef GPENMPC_CANONICAL_CLOSED_EVIDENCE
    check(!first.latest_closed_gp_evidence_available,"legacy build cannot invent closed GP output");
#else
    if(first.latest_closed_gp_evidence_available)check(first.closed_gp.exported&&first.closed_gp.installed&&first.installed_evidence_matches_state,
        "closed availability requires actual generated closed output and same installation");
#endif
    return first;
}
#ifndef GPENMPC_CORE_TEST_MAIN
#define GPENMPC_CORE_TEST_MAIN wmain
#endif
int GPENMPC_CORE_TEST_MAIN(int argc,wchar_t**argv){
    if((argc!=5&&argc!=6)||!load(argv[1],argv[2])||!load_private(argv[2])||!parse_hash(argv[4]))return 2;
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
    if(argc!=6)return 2;FILE*closed_file=_wfopen(argv[5],L"wb");if(!closed_file)return 3;unsigned closed_rows=0;
#endif
    const auto c=gw::canonical_configuration();for(unsigned j=0;j<32;++j)config.configuration_sha256[j]=uint8_t(c[j/4]>>(24-8*(j%4)));
    HMODULE dll=LoadLibraryExW(argv[3],nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);if(!dll)return 3;
    using Predict=int(*)(const double*,double*);const auto predict=reinterpret_cast<Predict>(GetProcAddress(dll,"gpenmpc_gp256_predict"));if(!predict)return 3;
    unsigned main_steps=0,main_gp=0;int consumer{};
    if(std::getenv("GPENMPC_GP_TIMEOUT_ONLY")){
        CycleOwner o(true,false,nullptr,false,true);li::CanonicalLocalGpPending pending(*o.cycle);
        check(two_commits(o,pending),"actual C full SE3 produces original GP dependency");
        const auto request=*pending.pending_bytes();gw::ReplyBytes reply{};
        check(actual_reply(pending,predict,reply),"actual canonical GP256 computes reply without early fill");
        const auto ready=pending.retained_actual_feedback().original_cycle_read_us;
        auto cf=exchange_config().transport;cf.max_assembly_us=200000;
        const auto later=ready+200001;
        gpenmpc_local_ingress::CanonicalLocalIngress legacy(cf),rx(cf,nullptr,nullptr,true);
        check(legacy.expect_gp_reply(request,ready,ready)&&!legacy.tick(later),"legacy unanswered timeout remains fail closed");
        check(rx.expect_gp_reply(request,ready,ready)&&rx.tick(later)&&!rx.failed()&&!rx.ready(),
            "runtime unanswered transport timeout does not poison independent control channel");
        const auto generation=rx.take_unanswered_generation();
        check(generation!=0&&rx.take_unanswered_generation()==0&&pending.retire_unanswered(generation,later)
            &&!pending.pending()&&pending.diagnostics().fills==0&&pending.diagnostics().deadline_unavailable==1&&publish_calls==2,
            "retirement follows the unavailable numerical path");
        gw::Request next{};check(gw::decode(request,next),"original request decode");
        next.source_timestamp_ns+=300000000;next.source_generation++;next.output_generation++;
        next.original_publication_us+=300000;next.publication_valid_until_us+=300000;
        gw::RequestBytes next_bytes{};gw::Reply next_reply{};gw::ReplyBytes next_reply_bytes{};
        check(gw::encode(next,next_bytes)&&gw::decode(reply,next_reply),"explicit HOST next-request fixture");
        next_reply.source_timestamp_ns=next.source_timestamp_ns;next_reply.source_generation=next.source_generation;
        next_reply.output_generation=next.output_generation;
        next_reply.original_request_sha256=gw::digest(next_bytes.data(),next_bytes.size());
        check(gw::encode(next_reply,next_reply_bytes),"next fixture reply retains original numeric values and new exact tags");
        const auto fresh=next.original_publication_us+1;
        check(rx.expect_gp_reply(next_bytes,fresh,fresh),"new genuine request slot available after retirement");
        std::uint64_t seq=0;
        auto arrival=[&](const gw::ReplyBytes&body,unsigned j,std::uint64_t when){
            gw::Fragment frag{};check(gw::fragment(body,j,frag),"original reply fragment");
            gpenmpc_argument_transport::Arrival a{};auto&v=a.fields;
            v.timestamp=when;v.source_system=255;v.source_component=190;v.target_system=1;v.target_component=1;
            v.receiver_instance=2;v.payload_type=42002;v.payload_length=frag.length;v.wire_payload_length=frag.length+5;
            v.reception_sequence=++seq;a.uorb_generation=seq;std::memcpy(v.payload,frag.payload,128);return a;
        };
        for(unsigned j=0;j<3;++j)check(rx.receive(arrival(reply,j,fresh+j+1),fresh+j+1)&&!rx.ready(),
            "exact retired reply is checked but cannot become GP numerical readiness");
        for(unsigned j=0;j<3;++j)check(rx.receive(arrival(next_reply_bytes,j,fresh+j+10),fresh+j+10),"fresh reply accepted by same dispatcher");
        gpenmpc_local_ingress::Completed out{};
        check(rx.take_gp_reply(out,fresh+14)&&out.bytes==next_reply_bytes,"only current exact reply is delivered");
        for(unsigned bad=0;bad<3;++bad){
            gpenmpc_local_ingress::CanonicalLocalIngress negative(cf,nullptr,nullptr,true);seq=0;
            check(negative.expect_gp_reply(request,ready,ready),"negative original expectation");
            if(bad==0){check(negative.receive(arrival(reply,0,ready+1),ready+1)&&!negative.tick(later+1),
                "partial real reply timeout remains fatal");continue;}
            check(negative.tick(later)&&negative.take_unanswered_generation()==generation,"negative retired request");
            bool accepted=true;
            for(unsigned j=0;j<3&&accepted;++j){auto a=arrival(reply,j,later+j+1);
                if(bad==1&&j==2)a.fields.payload[20]^=1;
                if(bad==2&&j==0)a.fields.payload[8]^=64;
                accepted=negative.receive(a,later+j+1);
            }
            check(!accepted&&negative.failed()&&!negative.ready(),"bad retired body or unknown generation cannot be discarded as valid");
        }
        for(unsigned variant=0;variant<3;++variant){
            gpenmpc_local_ingress::CanonicalLocalIngress crossing(cf,nullptr,nullptr,true);seq=0;
            check(crossing.expect_gp_reply(request,ready,ready),"original request for crossing-boundary reply");
            const auto first=ready+199500;
            check(crossing.receive(arrival(reply,0,first),first)&&crossing.tick(later)&&
                !crossing.failed()&&!crossing.ready()&&crossing.received_fragments()==1&&
                crossing.take_unanswered_generation()==0&&crossing.audit_fragment(0)->fields.timestamp==first,
                "late prefix preserved; numerical retirement cannot free a still-draining request");
            if(variant==1){check(!crossing.tick(first+cf.max_assembly_us+1)&&
                crossing.diagnostics().first_fault==gpenmpc_local_ingress::Fault::Expired,
                "missing late suffix expires from its original first byte, not an infinite wait");continue;}
            check(crossing.receive(arrival(reply,1,later+200),later+200),"same original second fragment resumes after request retirement");
            auto last=arrival(reply,2,later+500);if(variant==2)last.fields.payload[20]^=1;
            const bool accepted=crossing.receive(last,later+500);
            if(variant==2){check(!accepted&&crossing.failed()&&!crossing.ready(),"corrupt late suffix cannot hide behind history classification");continue;}
            gpenmpc_local_ingress::Completed unused{};
            check(accepted&&!crossing.failed()&&!crossing.ready()&&crossing.received_fragments()==0&&
                crossing.take_unanswered_generation()==generation&&crossing.take_unanswered_generation()==0&&
                !crossing.take_gp_reply(unused,later+501)&&!unused.complete,
                "CRC-complete late reply retires once and is never exposed to numerical fill");
            check(crossing.expect_gp_reply(next_bytes,fresh,fresh),"only complete checked late drain releases the next existing GP slot");
        }
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
        check(std::fclose(closed_file)==0,"close unused closed-history fixture");
#endif
        FreeLibrary(dll);std::printf("{\"checks\":%u,\"failed\":%u,\"actual_core_commits\":2,\"actual_GP\":1,\"COM\":0,\"board\":0}\n",checks,failed);
        return failed?1:0;
    }
    {extra_clear();CycleOwner o(false,false,nullptr,true);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;
        li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},exchange_config());
        check(original_selected(o,0),"component cadence actual source fixture");const auto first_start=clock_us;
        check(core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==1,"component first actual generated-C commit is immediate");
        task.index=1;check(original_selected(o,1),"component next actual source retains its original timestamp");
        clock_us=first_start+gpenmpc_consumption::canonical_dt_us-1;
        check(core.poll(false)==li::LocalExchangePoll::Idle&&core.diagnostics().commits==1&&task_calls==1,
            "component services poll without early second capture or control computation");
        clock_us=o.authority.token.control_tick_us+gpenmpc_consumption::canonical_dt_us;
        check(core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==2&&task_calls==2,
            "component next10ms tick captures actual latest state and commits unchanged SE3");
        check(!core.diagnostics().gp_requests_enqueued&&!core.pending_after_stop().pending(),"component does not wait for disabled outer GP");
        clock_us+=100;check(core.poll(false)==li::LocalExchangePoll::Idle&&core.diagnostics().commits==2,
            "component never fabricates a catch-up step or repeats an old state");
    }
    if(std::getenv("GPENMPC_COMPONENT_CADENCE_ONLY")){
        {extra_clear();CycleOwner o(false,false,nullptr,true,true);
            ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;
            task.runtime_state_only=true;task.hold_previous_source=true;
            auto cfg=exchange_config();cfg.export_active_snapshots=true;cfg.snapshot_transport_max_age_us=50000;
            li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},cfg);
            unsigned step=0;uint64_t second_export=0;
            for(unsigned index:{0U,3U,6U,9U}){
                task.index=index;check(original_selected(o,index),"RC backpressure uses actual retained source fixture");
                const auto outcome=core.poll(false);++step;
                check(outcome==li::LocalExchangePoll::Progress&&core.diagnostics().commits==step&&publish_calls==step,
                    "waiting for returned HOST input never gates fresh full-SE3 control with still-valid inputs");
                if(step==1||step==2){
                    check(core.diagnostics().snapshots_enqueued==step&&drain_outbox(core,&consumer,nullptr),
                        "initial snapshot and first acknowledged refresh are exported normally");
                    if(step==2)second_export=clock_us;
                }else if(step==3){
                    check(clock_us-second_export<40000&&core.diagnostics().snapshots_enqueued==2&&!core.outbox().pending(),
                        "unanswered snapshot does not create another backlog at the next 20ms slot");
                }else{
                    check(clock_us-second_export>=40000&&core.diagnostics().snapshots_enqueued==3&&drain_outbox(core,&consumer,nullptr),
                        "expired unsent HOST candidate cannot deadlock refresh: new actual snapshot retries after40ms");
                }
                check(task.previous_input.sample_us==raw_fixture[0].timestamp_sample,
                    "producer pacing never renews the held input source timestamp");
            }
        }
        if(std::getenv("GPENMPC_RC_PACING_ONLY")){
            std::printf("{\"checks\":%u,\"failed\":%u,\"rc_pacing_only\":true,\"COM\":0,\"board\":0}\n",checks,failed);
            return failed?1:0;
        }
        {extra_clear();CycleOwner o(false,false,nullptr,true);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;
            task.read_delay_us=3000;
            li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},exchange_config());
            check(original_selected(o,0),"execution-clock delayed input uses original source fixture");const auto capture_clock=clock_us;
            check(core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==1,
                "delayed but still fresh input performs the actual first generated-C control");
            const auto actual_start=o.authority.token.control_tick_us;
            task.read_delay_us=0;task.index=1;check(original_selected(o,1),"execution-clock next actual source");
            clock_us=capture_clock+gpenmpc_consumption::canonical_dt_us;
            check(clock_us>actual_start&&clock_us-actual_start<gpenmpc_consumption::canonical_dt_us,
                "old capture anchor would release another tick less than10ms after real execution");
            check(core.poll(false)==li::LocalExchangePoll::Idle&&core.diagnostics().commits==1&&publish_calls==1,
                "new execution anchor prevents catch-up publication without relaxing ownership or source age");
            clock_us=actual_start+gpenmpc_consumption::canonical_dt_us;
            check(core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==2&&publish_calls==2,
                "due execution consumes next original fresh source");
            check(o.authority.token.timestamp_sample_us==raw_fixture[1].timestamp_sample&&
                !core.diagnostics().gp_requests_enqueued,"source remains original and component does not invent GP calls");
        }
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
        for(unsigned held_roundtrip=0;held_roundtrip<2;++held_roundtrip){extra_clear();CycleOwner o(false,false,nullptr,true);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;
            auto cfg=exchange_config();cfg.export_active_snapshots=true;cfg.export_committed_state=true;
            cfg.snapshot_transport_max_age_us=50000; // Component transport age budget.
            li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},cfg);
            check(original_selected(o,0),"component record-priority original source");const auto initial=clock_us;
            check(core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==1&&
                core.diagnostics().committed_states_enqueued==0,"first real commit retains record behind numerical state");
            check(drain_outbox(core,&consumer,nullptr),"original RLS drained without claiming a control acknowledgement");
            clock_us=initial+500;
            check(core.poll(false)==li::LocalExchangePoll::Idle&&!core.outbox().pending(),
                "record waits for a newer actual RLS rather than stealing its slot");
            task.index=3;task.hold_previous_source=held_roundtrip!=0;
            check(original_selected(o,3),"newer source supplied by the original fixture");
            if(clock_us<initial+20001)clock_us=initial+20001;
            check(core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==2&&
                core.diagnostics().snapshots_enqueued==2&&core.diagnostics().committed_states_enqueued==0,
                "newer numerical state precedes retained history without blocking actual SE3 control");
            check(drain_outbox(core,&consumer,nullptr),"newer original RLS fully consumed first");
            ++clock_us;
            if(held_roundtrip){
                check(core.poll(false)==li::LocalExchangePoll::Idle&&
                    core.diagnostics().committed_states_enqueued==0&&!core.outbox().pending(),
                    "RLS send completion alone cannot release history ahead of its input round trip");
                task.index=4;task.hold_previous_source=false;
                const bool fresh_input_source=original_selected(o,4);
                // Fixture rows are 9 ms apart; poll when the actual 10-ms
                // control schedule is due, with this source still fresh.
                clock_us+=2000;
                const auto fresh_result=core.poll(false);
                check(fresh_input_source&&fresh_result==li::LocalExchangePoll::Progress&&
                    core.diagnostics().commits==3,
                    "new complete input supports actual SE3 before the retained history is sent");
            }
            check(core.poll(false)==li::LocalExchangePoll::Idle&&core.diagnostics().committed_states_enqueued==1,
                "same buffer publishes unchanged RLC only after newer input actually supports control");
            gpenmpc_local_committed_wire::Bytes b{};unsigned offset=0;
            if(held_roundtrip)clock_us=initial+cfg.snapshot_transport_max_age_us+1000;
            while(core.outbox().pending()){li::LocalWireFragment fragment{};
                check(core.outbox().take(&consumer,fragment,hrt_absolute_time())==li::LocalWireTake::Fragment&&
                    fragment.original.schema==gpenmpc_local_committed_wire::schema,"retained RLC fragments keep original schema and expiry");
                if(held_roundtrip)check(fragment.original.component_history_only&&
                    clock_us>fragment.original.original_transport_valid_until_us&&
                    fragment.original.original_transport_valid_until_us==fragment.original.original_anchor_us+50000,
                    "late component history keeps original 50-ms screening timestamp without granting freshness");
                const unsigned n=fragment.fragment.length-9;if(offset+n>b.size())return 4;
                std::memcpy(b.data()+offset,fragment.fragment.payload+9,n);offset+=n;}
            gpenmpc_local_committed_wire::Observation observed{};
            check(offset==b.size()&&gpenmpc_local_committed_wire::decode(b,observed)&&observed.joint_installs==1&&
                observed.source_timestamp_ns==raw_fixture[0].timestamp_sample*1000ULL,
                "queued first actual record preserved, not replaced or retimestamped by second commit");
            if(held_roundtrip){
                auto wire_cfg=li::LocalWireConfiguration{};wire_cfg.expected_session=identity();
                wire_cfg.producer_identity=&task;wire_cfg.snapshot_transport_max_age_us=50000;
                wire_cfg.gp_request_transport_max_age_us=50000;wire_cfg.target_system=255;wire_cfg.target_component=190;
                li::CanonicalLocalWireOutbox numerical(wire_cfg);li::LocalWireFragment fragment{};
                check(numerical.publish_committed(b,&task)==li::LocalWirePublish::Published&&
                    numerical.take(&consumer,fragment,clock_us)==li::LocalWireTake::Expired,
                    "same late committed bytes in full-method mode still expire; no numerical dependency bypass");
            }
            check(!core.pending_after_stop().pending()&&!core.diagnostics().gp_requests_enqueued,
                "stopped component retains no pending GP request");
        }
#endif
        // live consumer timestamp was captured before a producer
        // could publish a newer observation. Use the actual mock HRT at take,
        // preserving the original source, deadline and true-future rejection.
        for(unsigned variant=0;variant<4;++variant){
            extra_clear();CycleOwner o(false,false,nullptr,true);
            ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;task.bad=1;
            auto cfg=exchange_config();cfg.export_active_snapshots=true;
            li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},cfg);
            check(original_selected(o,0)&&core.poll(false)==li::LocalExchangePoll::Idle,
                "consumer clock fixture publishes real source observation without control");
            const auto anchor=raw_fixture[0].timestamp_sample;
            li::LocalWireFragment f{};
            clock_us=variant==2?anchor-1:variant==3?anchor+cfg.snapshot_transport_max_age_us+1:anchor+300;
            const auto taken=core.outbox().take(&consumer,f,variant==0?anchor-1:0);
            if(variant==1)check(taken==li::LocalWireTake::Fragment&&f.original.original_anchor_us==anchor&&
                f.original.original_transport_valid_until_us==anchor+cfg.snapshot_transport_max_age_us&&
                core.outbox().fault()==li::LocalWireFault::None,
                "post-acquire actual HRT accepts new source without rewriting source or expiry");
            else check(taken==(variant==3?li::LocalWireTake::Expired:li::LocalWireTake::Unavailable)&&
                core.outbox().fault()==(variant==3?li::LocalWireFault::Expired:li::LocalWireFault::Clock),
                "old caller time reproduces false future while real future and real expiry remain rejected");
        }
        {extra_clear();CycleOwner o(false,false,nullptr,true);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;
            li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},exchange_config());
            check(original_selected(o,0)&&core.poll(false)==li::LocalExchangePoll::Progress,"pending refresh begins after a real first commit");
            task.bad=1;
            for(unsigned index:{1U,2U}){
                task.index=index;check(original_selected(o,index),"pending refresh receives real later source fixture");
                // Fixture sample 1 arrives before the next scheduled tick.
                // Advance only mock processing HRT, never the source timestamp.
                if(index==1)clock_us=o.authority.token.control_tick_us+gpenmpc_consumption::canonical_dt_us;
                check(core.poll(false)==li::LocalExchangePoll::Idle&&core.diagnostics().commits==1&&publish_calls==1,
                    "missing input refreshes only observation and never computes control");
                check(o.cycle->snapshot()&&o.cycle->snapshot()->estimator().timestamp_sample_us==raw_fixture[index].timestamp_sample&&
                    o.cycle->snapshot()->actual_sample_delta_us()==raw_fixture[index].timestamp_sample-raw_fixture[0].timestamp_sample,
                    "pending observation retains real source and dt from last executed source not intermediate capture");
            }
            task.bad=0;
            const auto pending_result=core.poll(false);
            std::printf("pending-refresh result=%u core=%u cycle=%u io=%u sample=%llu delta=%llu phase=%llu\n",unsigned(pending_result),unsigned(core.diagnostics().first_fault),unsigned(o.cycle->diagnostics().first_fault),unsigned(o.io->diagnostics().first_fault),static_cast<unsigned long long>(raw_fixture[2].timestamp_sample),static_cast<unsigned long long>(raw_fixture[2].timestamp_sample-raw_fixture[0].timestamp_sample),static_cast<unsigned long long>(o.phase->diagnostics().installs));
            check(pending_result==li::LocalExchangePoll::Progress&&core.diagnostics().commits==2&&o.phase->diagnostics().installs==2,
                "valid input executes latest pending state without requiring another uORB message");
            li::LocalNumericalObservation actual{};check(o.io->copy_local_numerical_state(actual)&&
                actual.last_installed.token.lease_envelope.timestamp_sample_us==raw_fixture[2].timestamp_sample&&
                actual.last_installed.actual_input36[34]==double(raw_fixture[2].timestamp_sample-raw_fixture[0].timestamp_sample)*1e-6,
                "actual unchanged C kernel receives full commit-to-commit measured dt");
        }
        {const auto raw2=raw_fixture[2];const auto old2=original[2];
            extra_clear();CycleOwner o(false,false,nullptr,true);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;
            li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},exchange_config());
            check(original_selected(o,0)&&core.poll(false)==li::LocalExchangePoll::Progress,"pending interval negative begins with actual commit");
            task.bad=1;task.index=1;check(original_selected(o,1),"intermediate raw source before due tick");
            clock_us=o.authority.token.control_tick_us+gpenmpc_consumption::canonical_dt_us;
            check(core.poll(false)==li::LocalExchangePoll::Idle&&o.cycle->snapshot()&&
                o.cycle->snapshot()->estimator().timestamp_sample_us==raw_fixture[1].timestamp_sample,
                "intermediate unexecuted observation remains uncommitted");
            raw_fixture[2].timestamp_sample=raw_fixture[0].timestamp_sample+50001;
            raw_fixture[2].timestamp=raw_fixture[2].timestamp_sample+100;original[2].tags[0]=raw_fixture[2].timestamp_sample*1000;
            task.index=2;check(original_selected(o,2),"overdue sample remains raw in broker");
            check(core.poll(false)==li::LocalExchangePoll::Fault&&o.io->diagnostics().source_adapter_fault==od::Failure::SampleDelta&&
                core.diagnostics().commits==1&&publish_calls==1,"intermediate captures cannot conceal a real greater-than-50ms control gap");
            raw_fixture[2]=raw2;original[2]=old2;
        }
        const auto raw1=raw_fixture[1];const auto old1=original[1];const auto row1=rows[1];
        // Check source intervals around the deployment bound.
        for(const uint64_t dt:{20000ULL,27083ULL,34082ULL,37705ULL,40000ULL,40001ULL,40142ULL,44578ULL,50000ULL,50001ULL}){
            raw_fixture[1]=raw1;original[1]=old1;rows[1]=row1;
            raw_fixture[1].timestamp_sample=raw_fixture[0].timestamp_sample+dt;
            raw_fixture[1].timestamp=raw_fixture[1].timestamp_sample+100;
            original[1].tags[0]=raw_fixture[1].timestamp_sample*1000;
            rows[1].args[9]=double(dt)*1e-6;original[1].input[34]=rows[1].args[9];
            extra_clear();CycleOwner o(false,false,nullptr,true);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;
            li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},exchange_config());
            check(original_selected(o,0)&&core.poll(false)==li::LocalExchangePoll::Progress,"jitter fixture initial actual component commit");
            task.index=1;check(original_selected(o,1),"jitter actual source timestamp retained in mock broker");
            const auto result=core.poll(false);
            if(dt>50000){check(result==li::LocalExchangePoll::Fault&&o.io->diagnostics().source_adapter_fault==od::Failure::SampleDelta&&core.diagnostics().commits==1,"50ms plus one rejected before next control");}
            else{
                check(result==li::LocalExchangePoll::Progress&&core.diagnostics().commits==2,"actual unchanged SE3 and phase accept bounded measured jitter");
                check(o.phase->diagnostics().installs==2&&o.io->retained_latest_snapshot()->actual_sample_delta_us()==dt,"original measured dt is not clamped or retimestamped");
                for(unsigned j=0;j<6;++j)check(std::isfinite(actual_output.output[j])&&actual_output.output[j]>=0&&actual_output.output[j]<=1,"actual rotor outputs finite within absolute limits");
            }
        }
        raw_fixture[1]=raw1;original[1]=old1;rows[1]=row1;
        // Heading-reset fixture: attitude can precede the EKF2 companion update
        // by one source sample; delta_heading=-0.013024 rad.
        const auto yaw_raw1=raw_fixture[1],yaw_raw2=raw_fixture[2];
        for(unsigned fault=0;fault<11;++fault){
            raw_fixture[1]=yaw_raw1;raw_fixture[2]=yaw_raw2;
            extra_clear();CycleOwner o(false,false,nullptr,true);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;
            task.hold_previous_source=true;
            li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},exchange_config());
            const auto companions=[&](unsigned i,unsigned att_count,unsigned heading_count,bool corrupt){
                vehicle_attitude_s a{};vehicle_local_position_s p{};
                a.timestamp_sample=p.timestamp_sample=bus_source.timestamp_sample;
                a.timestamp=p.timestamp=bus_source.timestamp;
                a.quat_reset_counter=att_count;p.heading_reset_counter=heading_count;
                a.delta_q_reset[0]=float(std::cos(-.013024/2));a.delta_q_reset[3]=float(std::sin(-.013024/2));p.delta_heading=-.013024f;
                if(corrupt){
                    if(fault==1)++p.xy_reset_counter;
                    if(fault==2)++p.vz_reset_counter;
                    if(fault==3)a.delta_q_reset[1]=.1f;
                    if(fault==4)a.timestamp_sample=p.timestamp_sample=bus_source.timestamp_sample-6000;
                    if(fault==5)p.delta_heading=.2f;
                    if(fault==7)a.delta_q_reset[0]=std::numeric_limits<float>::quiet_NaN();
                    if(fault==10){a.timestamp_sample=p.timestamp_sample=bus_source.timestamp_sample+1;a.timestamp=p.timestamp=bus_source.timestamp;}
                }
                if(i!=99){extra_publish(ORB_ID(vehicle_attitude),&a);extra_publish(ORB_ID(vehicle_local_position),&p);}
            };
            check(original_selected(o,0),"yaw-reset original pre-reset input");companions(0,0,0,false);
            check(core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==1,"yaw-reset pre-state actual C commit");
            raw_fixture[1].reset_counter=fault==6?2:1;
            task.index=1;check(original_selected(o,1),"yaw-reset raw source increment");
            companions(1,(fault==0||fault==8)?0:1,(fault==8||fault==9)?0:1,true);
            clock_us=o.authority.token.control_tick_us+gpenmpc_consumption::canonical_dt_us;
            const auto outcome=core.poll(false);
            if(fault>0&&fault<8){
                check(outcome==li::LocalExchangePoll::Fault&&o.io->diagnostics().source_adapter_fault==od::Failure::Reset&&
                    core.diagnostics().commits==1,"position velocity tilt stale mismatched unknown or nonfinite reset rejects before publication");
            }else{
                check(outcome==li::LocalExchangePoll::Idle&&core.diagnostics().commits==1&&task_calls==1,
                    "either companion lag or newer independent copy performs no old-state control or timestamp renewal");
                raw_fixture[2].reset_counter=1;task.index=2;check(original_selected(o,2),"yaw-reset next original uORB source");companions(2,1,1,false);
                check(core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==2,
                    "verified native yaw reset continues unchanged full SE3 with still-valid held inputs");
                const auto*s=o.io->retained_latest_snapshot();
                check(s&&s->heading_reset_from(0)&&s->raw().reset_counter==1&&
                    s->raw().timestamp_sample==raw_fixture[2].timestamp_sample&&
                    s->actual_sample_delta_us()==raw_fixture[2].timestamp_sample-raw_fixture[0].timestamp_sample,
                    "raw reset epoch and actual measured control gap retained without state rewriting");
            }
        }
        raw_fixture[1]=yaw_raw1;raw_fixture[2]=yaw_raw2;
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
        std::fclose(closed_file);
#endif
        FreeLibrary(dll);std::printf("{\"checks\":%u,\"failed\":%u,\"component_cadence_only\":true,\"COM\":0,\"board\":0}\n",checks,failed);return failed?1:0;
    }
    {extra_clear();CycleOwner o(false);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;task.bad=1;
        auto cfg=exchange_config();cfg.export_active_snapshots=true;
        li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},cfg);
        check(original_selected(o,0),"active export original source fixture");
        check(core.poll(false)==li::LocalExchangePoll::Idle&&core.diagnostics().snapshots_enqueued==1&&
            !core.diagnostics().commits&&!publish_calls,"active RLS is exported before absent host data, not after a circular wait");
        check(drain_outbox(core,&consumer,nullptr),"same real outbox exports active private source");
        check(core.poll(false)==li::LocalExchangePoll::Idle&&core.diagnostics().snapshots_enqueued==1,
            "waiting for real input neither duplicates source nor renews its time");
        task.bad=0;check(core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==1,
            "same retained source commits only after actual input becomes available");}
    {extra_clear();CycleOwner o(false);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;
        namespace ww=gpenmpc_local_window_wire;ww::Configuration wc{};wc.expected.identity=identity();wc.max_assembly_us=1000000;
        wc.expected.leg_index=task.window.leg_index;
        for(unsigned j=0;j<8;++j){wc.expected.execution_session_sha256[j]=j+1;
            const auto*t=config.task_sha256+4*j;wc.expected.task_sha256[j]=uint32_t(t[0])<<24|uint32_t(t[1])<<16|uint32_t(t[2])<<8|t[3];
            wc.expected.configuration_sha256[j]=gw::canonical_configuration()[j];
            const auto*p=task.window.reference_asset_sha256+4*j;wc.expected.reference_asset_sha256[j]=uint32_t(p[0])<<24|uint32_t(p[1])<<16|uint32_t(p[2])<<8|p[3];}
        ww::Assembler receiver(wc);task.receiver=&receiver;auto cfg=exchange_config();cfg.transport.max_assembly_us=wc.max_assembly_us;
        li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window,&receiver},cfg);
        ww::Encoder encoder(wc.expected,task.window);gpenmpc_ingress::Receiver rx;check(encoder.valid(),"actual original RWW1 encoder accepts registered window");
        for(unsigned i=0;i<ww::fragment_count;++i){ww::Fragment f{};check(encoder.fragment(i,f),"bounded original window fragment");
            mavlink_message_t msg{},parsed{},parser{};mavlink_status_t status{},reported{};
            mavlink_msg_tunnel_pack(255,190,&msg,1,1,42002,f.length,f.payload);uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
            const auto n=mavlink_msg_to_send_buffer(bytes,&msg);unsigned count=0;
            for(unsigned j=0;j<n;++j)if(mavlink_frame_char_buffer(&parser,&status,bytes[j],&parsed,&reported)==MAVLINK_FRAMING_OK)++count;
            check(count==1&&rx.receive(parsed,hrt_absolute_time(),1,1,2)==gpenmpc_ingress::Result::Accepted,
                "actual CRC parsed window traverses production receiver");
            check(rx.drain(1,[&](const gpenmpc_ingress::Fields&v){gpenmpc_full_inner_ingress_s t{};gpenmpc_ingress::copy_to_topic(v,rx.counters(),t);return gpenmpc_test_topic_publish(ORB_ID(gpenmpc_full_inner_ingress),&t);}),"window same actual queue");
            if((i+1)%16==0||i+1==ww::fragment_count)
                check(core.poll(false)==li::LocalExchangePoll::Idle,"one actual prearm16 burst before Core poll never executes inner");
        }
        check(core.diagnostics().window_loads==1&&receiver.diagnostics().released_windows==1&&
            core.ingress_after_stop().diagnostics().envelopes_admitted==244&&!publish_calls,
            "one real Core subscription loads then releases assembled 244-fragment window");
        check(original_selected(o,0)&&core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==1,
            "released receive buffer may be missing while installed reference still executes actual C");
        check(core.diagnostics().window_loads==1,"installed generation is not reloaded by a missing window callback");}
    {extra_clear();CycleOwner o(false);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;
        auto cfg=exchange_config();
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
        cfg.export_committed_state=true;
#endif
        li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},cfg);gpenmpc_ingress::Receiver rx;
        const auto initial=observe_state(o);check(initial.kind==li::LocalStateKind::InitialUncommitted&&!initial.state.numeric_installed&&
            !initial.state.reference_committed&&!initial.last_installed.present&&!initial.actual_joint_installs,"unexecuted First is explicitly uncommitted storage not initialized observer");
        for(unsigned i=0;i<60;++i){task.index=i;check(original_selected(o,i),"actual original source receipt plus stock-topic endpoint fixture");const auto result=core.poll(false);
            if(result!=li::LocalExchangePoll::Progress){
                std::fprintf(stderr,"row%u core%u cycle%u selected%u io%u commits%llu\n",i,unsigned(core.diagnostics().first_fault),unsigned(o.cycle->diagnostics().first_fault),unsigned(selected.diagnostics().first_reason),unsigned(o.io->diagnostics().first_fault),static_cast<unsigned long long>(core.diagnostics().commits));
                li::LocalNumericalObservation diagnostic{};o.io->copy_local_numerical_state(diagnostic);
                std::fprintf(stderr,"closed_export%u installed%u learning%u closed5",unsigned(diagnostic.closed_gp.exported),unsigned(diagnostic.closed_gp.installed),unsigned(diagnostic.closed_gp.learning_exported));
                for(double v:diagnostic.closed_gp.values5)std::fprintf(stderr," %.17g",v);
                std::fprintf(stderr," learning12");for(double v:diagnostic.closed_gp.learning12)std::fprintf(stderr," %.17g",v);std::fprintf(stderr,"\n");
            }
            check(result==li::LocalExchangePoll::Progress&&core.diagnostics().commits==i+1,"actual Core drives capture selectedjoin realwindow realC and joint publication");if(result!=li::LocalExchangePoll::Progress)break;++main_steps;
            const auto installed=observe_state(o);check(installed.installed_evidence_matches_state&&installed.actual_joint_installs==i+1&&
                installed.last_installed.joint_install_count==i+1&&installed.last_installed.present&&
                installed.kind==(i?li::LocalStateKind::CommittedAwaitingGp:li::LocalStateKind::CommittedNoGp),"actual installed output token matches numeric/reference state; next open prediction clearly separated");
            check(same(installed.last_installed.actual_published_control16,actual_output.output,sizeof actual_output.output)&&
                installed.last_installed.original_publication_us==installed.state.reference.publication_us&&
                installed.state.original_installed_tags2[0]==bus_source.timestamp_sample*1000,"retained actual control bits publication HRT and original source tags belong to same installation");
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
            gw::RequestBytes priority_request{};
            if(i){
                check(drain_outbox(core,&consumer,&priority_request),
                    "actual GP query is available before any same-commit diagnostic fragment");
                check(core.diagnostics().gp_requests_enqueued==i&&core.diagnostics().committed_states_enqueued==i&&
                    core.pending_after_stop().pending()&&publish_calls==i+1,
                    "GP-first scheduling neither drops pending evidence nor fabricates a GP completion");
                check(core.poll(false)==li::LocalExchangePoll::Idle&&core.diagnostics().committed_states_enqueued==i+1,
                    "same outbox promotes retained closed evidence without another capture or publication");
            }
            namespace cw=gpenmpc_local_committed_wire;cw::Bytes bytes{};unsigned count=0,offset=0;
            while(core.outbox().pending()){li::LocalWireFragment f{};
                check(core.outbox().take(&consumer,f,hrt_absolute_time())==li::LocalWireTake::Fragment&&f.original.schema==cw::schema,
                    "actual Core sends RLC2 from its committed owner before the next GP fill");
                const unsigned n=f.fragment.length-9;if(offset+n>bytes.size())return 4;
                std::memcpy(bytes.data()+offset,f.fragment.payload+9,n);offset+=n;++count;}
            cw::Observation observed{};check(count==cw::fragment_count&&offset==cw::message_bytes&&cw::decode(bytes,observed),"actual fragments reassemble original closed-state digest");
            check(same(observed.state64,installed.state.state64,sizeof observed.state64)&&
                same(observed.closed5,installed.closed_gp.values5,sizeof observed.closed5)&&
                same(observed.kernel61,installed.last_installed.actual_kernel61,sizeof observed.kernel61)&&
                same(observed.published_control16,actual_output.output,sizeof observed.published_control16),"RLC state64 closed5 kernel61 control16 original bits unchanged");
            check(installed.closed_gp.learning_exported&&same(observed.learning12,installed.closed_gp.learning12,sizeof observed.learning12),
                "actual installed innovation and per-axis learning bits survive the production outbox");
            if(i==1){auto invalid=observed;invalid.closed_available=1;invalid.closed5[0]=1;invalid.closed5[2]=1;
                invalid.learning12[0]=std::numeric_limits<double>::quiet_NaN();cw::Bytes rejected{};
                check(!cw::encode(invalid,rejected),"claimed available innovation rejects NaN");
                auto unavailable=invalid;unavailable.closed_available=0;unavailable.closed5[0]=0;
                check(cw::encode(unavailable,rejected),"original unavailable prediction keeps observed-label flag and NaN as raw evidence");}
            check(observed.source_timestamp_ns==installed.state.original_installed_tags2[0]&&
                observed.source_generation==installed.state.original_installed_tags2[1]&&
                observed.output_generation==installed.state.reference.output_generation&&
                observed.original_publication_us==installed.state.reference.publication_us&&
                observed.installed_phase2[0]==o.phase->diagnostics().phase_s,"closed transport preserves original source/output/phase, not HOST time");
            check(std::fwrite(bytes.data(),1,bytes.size(),closed_file)==bytes.size(),"persist actual RLC fixture");++closed_rows;
            check(core.poll(false)==li::LocalExchangePoll::Idle,"closed evidence drain leaves original GP pending without a new inner publication");
#endif
            if(i){gw::RequestBytes request{};
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
                request=priority_request;
#else
                check(drain_outbox(core,&consumer,&request),"actual outbox three fragments retain actual installed GP query");
#endif
                gw::Request q{};check(gw::decode(request,q),"actual outgoing request checksum");gw::Reply reply{};reply.identity=q.identity;reply.source_timestamp_ns=q.source_timestamp_ns;reply.source_generation=q.source_generation;reply.output_generation=q.output_generation;reply.original_request_sha256=gw::digest(request.data(),request.size());reply.gp_model_sha256=q.gp_model_sha256;
                check(same(installed.last_installed.actual_request19,q.request19,sizeof q.request19),"readonly retained request19 equals actual outgoing request not recomputed HOST inner");
                check(predict(q.request19+1,reply.result18)==0,"actual unchanged GP256 DLL");gw::ReplyBytes b{};check(gw::encode(reply,b),"actual numerical reply encode");
                for(unsigned k=0;k<3;++k){gw::Fragment f{};gw::fragment(b,k,f);mavlink_message_t m{},got{},parser{};mavlink_status_t st{},reported{};
                    mavlink_msg_tunnel_pack(255,190,&m,1,1,42002,f.length,f.payload);uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(bytes,&m);unsigned count=0;
                    for(unsigned j=0;j<n;++j)if(mavlink_frame_char_buffer(&parser,&st,bytes[j],&got,&reported)==MAVLINK_FRAMING_OK)++count;
                    const auto original=hrt_absolute_time();check(count==1&&rx.receive(got,original,1,1,2)==gpenmpc_ingress::Result::Accepted,"actual MAVLink CRC receiver preserves its own original HRT");++core_wire_packets;
                    check(rx.drain(1,[&](const gpenmpc_ingress::Fields&v){gpenmpc_full_inner_ingress_s t{};gpenmpc_ingress::copy_to_topic(v,rx.counters(),t);return gpenmpc_test_topic_publish(ORB_ID(gpenmpc_full_inner_ingress),&t);}),"actual ingress producer to generated-capacity mockuORB queue");}
                check(core.poll(false)==li::LocalExchangePoll::Idle&&core.diagnostics().gp_replies_filled==i&&publish_calls==i+1,"actual subscription/central ingress/Pending fill without another publication");++main_gp;
                const auto filled=observe_state(o);check(filled.kind==li::LocalStateKind::CommittedGpReady&&filled.actual_gp_fills==i&&
                    filled.installed_evidence_matches_state&&filled.actual_joint_installs==installed.actual_joint_installs&&
                    same(filled.state.state64,installed.state.state64,sizeof filled.state.state64)&&
                    same(filled.state.original_installed_tags2,installed.state.original_installed_tags2,sizeof filled.state.original_installed_tags2)&&
                    same_reference_state(filled.state.reference,installed.state.reference),"GP fill updates original pending slot without changing installed observer/reference/source/output");
            }
        }
        check(main_steps==60&&main_gp==59&&task_calls==60&&core.diagnostics().window_loads==1&&core_wire_packets==177,"60 complete core steps59 realGP177 packets single installed originalwindow");core.stop();check(core.poll(false)==li::LocalExchangePoll::Fault&&publish_calls==60,"stop latches and preserves occurred outputs");}
    for(unsigned bad=0;bad<5;++bad){extra_clear();CycleOwner o(false);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;task.bad=bad;
        li::LocalTaskInputPort port=bad?li::LocalTaskInputPort{&task,&TaskOwner::read,&TaskOwner::get_window}:li::LocalTaskInputPort{};li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,port,exchange_config());original_selected(o,0);
        const auto r=core.poll(false);if(bad<2)check(r==li::LocalExchangePoll::Idle&&!publish_calls&&!o.phase->diagnostics().installs&&core.diagnostics().missing_task_inputs==1,"absent or missing TaskInput never fabricates value or permission");
        else check(r==li::LocalExchangePoll::Fault&&!publish_calls&&!o.phase->diagnostics().installs,"wrong source missingwind wrongouter cannot execute or install");
        const auto state=observe_state(o);check(!state.last_installed.present&&!state.state.numeric_installed&&
            state.kind==(bad?li::LocalStateKind::HistoricalUnusable:li::LocalStateKind::InitialUncommitted),"missing captured input or fault cannot export a fictitious committed initial state");}
    {extra_clear();CycleOwner o(false);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},exchange_config());original_selected(o,0);publish_success=false;
        check(core.poll(false)==li::LocalExchangePoll::Fault&&publish_calls==1&&!o.phase->diagnostics().installs&&!core.diagnostics().commits,"failed real publication retains attempt and never advances local phase");
        const auto state=observe_state(o);check(state.kind==li::LocalStateKind::HistoricalUnusable&&!state.last_installed.present&&!state.actual_joint_installs,"failed publication retained diagnostics never masquerade as installed data");}
    {extra_clear();CycleOwner o(false);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},exchange_config());original_selected(o,0);injection=4;
        check(core.poll(false)==li::LocalExchangePoll::Fault&&publish_calls==1&&o.io->diagnostics().joint_installs==1&&!o.phase->diagnostics().installs,"post-install deadline failure retains occurred numeric installation and publication without phase success");
        const auto state=observe_state(o);check(state.kind==li::LocalStateKind::HistoricalUnusable&&state.last_installed.present&&state.installed_evidence_matches_state&&state.actual_joint_installs==1,"late failed candidate retains actually installed state as historical unusable not rolled back");}
    {extra_clear();CycleOwner o(false);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;task.bad=1;
        auto cfg=exchange_config();cfg.export_active_snapshots=true;
        li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},cfg);
        uint64_t last_export=0;unsigned exports=0,suppressed=0;
        for(unsigned i=0;i<5;++i){original_selected(o,i,true);const auto observed_clock=clock_us;
            const bool due=!last_export||observed_clock-last_export>=20000;
            check(core.poll(true)==li::LocalExchangePoll::Progress,"disarmed capture released every original sample without control");
            if(due){check(drain_outbox(core,&consumer,nullptr),"due20ms disarmed RLS retains original source bytes");last_export=observed_clock;++exports;}
            else{check(!core.outbox().pending(),"intermediate disarmed observation does not queue another four-fragment RLS");++suppressed;}
            check(core.diagnostics().snapshots_enqueued==exports,"actual observation exports follow existing20ms cadence");
        }
        check(exports>0&&suppressed>0&&!publish_calls&&!o.phase->diagnostics().installs&&!core.diagnostics().gp_requests_enqueued,
            "full-method disarmed observation has no GP or inner/control authority");
        original_selected(o,5);check(clock_us-last_export<20000,"arm transition occurs before next observation is due");
        check(core.poll(false)==li::LocalExchangePoll::Idle&&core.diagnostics().snapshots_enqueued==exports+1&&
            drain_outbox(core,&consumer,nullptr)&&!core.diagnostics().commits,
            "first full-method active source exports immediately but still requires exact real input");}
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
    {extra_clear();CycleOwner o(false,false,nullptr,false,true);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;
        task.runtime_state_only=true;
        auto cfg=exchange_config();cfg.export_active_snapshots=true;cfg.export_committed_state=true;cfg.snapshot_transport_max_age_us=50000;
        li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},cfg);
        check(original_selected(o,0,true)&&core.poll(true)==li::LocalExchangePoll::Progress&&core.outbox().numerical_pending(),
            "runtime observed RLS remains genuinely unsent in the existing outbox");
        task.index=1;check(original_selected(o,1),"runtime new active input retains actual newer source");
        check(core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==1&&publish_calls==1,
            "pending observation does not prevent first actual full SE3 control with valid input");
        check(core.diagnostics().committed_states_enqueued==0&&core.diagnostics().snapshots_enqueued==1,
            "unexported active source does not produce an unpaired RLC or invented observation");
        task.index=2;check(original_selected(o,2),"runtime subsequent actual source available");clock_us+=2000;
        check(core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==2&&publish_calls==2,
            "pending observation also cannot block next due100Hz numerical step");
        check(core.pending_after_stop().pending()&&!core.diagnostics().gp_replies_filled,
            "full method still generates a real GP request and cannot fabricate inference completion");
        check(drain_outbox(core,&consumer,nullptr),"RLS drains after control");
    }
    {extra_clear();CycleOwner o(false,false,nullptr,false,true);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner task;
        task.runtime_state_only=true;
        auto cfg=exchange_config();cfg.export_active_snapshots=true;cfg.export_committed_state=true;cfg.snapshot_transport_max_age_us=50000;
        li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},cfg);
        check(original_selected(o,0)&&core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==1,
            "full runtime actual first SE3 commit does not wait for history");
        check(core.diagnostics().committed_states_enqueued==0&&drain_outbox(core,&consumer,nullptr),
            "first original RLS drains while immutable RLC remains behind the real input roundtrip");
        clock_us+=500;
        check(core.poll(false)==li::LocalExchangePoll::Idle&&core.diagnostics().commits==1&&core.diagnostics().committed_states_enqueued==0,
            "an idle poll with no new actual input does not release pending history");
        task.index=1;check(original_selected(o,1),"fresh original slow input and source");clock_us+=2000;
        check(core.poll(false)==li::LocalExchangePoll::Progress&&core.diagnostics().commits==2&&core.diagnostics().committed_states_enqueued==1,
            "new input supports actual second full SE3 commit then releases original first history");
        check(core.pending_after_stop().pending()&&core.diagnostics().gp_replies_filled==0&&core.outbox().history_pending(),
            "second actual commit retains a real GP dependency separately from immutable first history");
        core.stop();
    }
    for(unsigned late=0;late<2;++late){
        CycleOwner o(true,false,nullptr,false,true);li::CanonicalLocalGpPending pending(*o.cycle);
        check(two_commits(o,pending),"runtime full mode creates actual GP query after committed SE3");
        gw::ReplyBytes reply{};check(actual_reply(pending,predict,reply),"same original GP256 computes real pending prediction");
        const auto publication=pending.retained_actual_feedback().actual.original_publication_us;
        const auto arrival=publication+(late?11000:1000);clock_us=arrival+1;
        check(pending.accept_reply(reply,arrival,clock_us)&&!pending.pending(),"real reply remains matched to original source and request");
        check(pending.diagnostics().fills==(late?0:1)&&pending.diagnostics().deadline_unavailable==(late?1:0)&&
            pending.diagnostics().late_replies_not_installed==(late?1:0)&&publish_calls==2,
            "late inference withdraws learning, timely inference fills, neither fabricates another control");
    }
    {CycleOwner o(true,false,nullptr,false,true);li::CanonicalLocalGpPending pending(*o.cycle);
        check(two_commits(o,pending),"busy HOST inference starts from actual two-commit query");
        gw::ReplyBytes reply{};check(actual_reply(pending,predict,reply),"busy query computes original GP reply without installing early");
        clock_us=pending.retained_actual_feedback().actual.original_publication_us+10000;
        check(pending.service_control_deadline(clock_us)&&pending.pending()&&!pending.numerical_pending(),
            "numerical deadline leaves original wire request in flight");
        const auto deadline_clock=clock_us;StepInputs next{};
        const bool captured=o.capture(2,next);
        // Original 9-ms sample already exists when the 10-ms task is due.
        // Advance only explicit mock processing HRT, not its source time.
        clock_us=std::max(clock_us,deadline_clock)+1;
        check(captured&&o.cycle->execute(next.value)==li::LocalCycleExecute::Committed,
            "next actual source executes full SE3 while old HOST inference remains pending");
        check(pending.observe_actual_commit()==li::LocalGpBegin::NotRequired&&pending.diagnostics().busy_unavailable==1,
            "new interval has explicit unavailable learning without duplicate HOST request");
        ++clock_us;
        check(pending.accept_reply(reply,clock_us,clock_us)&&!pending.pending()&&pending.diagnostics().fills==0
            &&pending.diagnostics().late_replies_not_installed==1&&publish_calls==3,
            "old matched reply retires only its wire request and cannot alter new numerical interval");
    }
    check(closed_rows==60&&std::fclose(closed_file)==0,"60 actual committed-state rows retained");
#endif
    FreeLibrary(dll);std::printf("{\"checks\":%u,\"failed\":%u,\"actual_core_commits\":%u,\"actual_GP\":%u,\"actual_MAVLink_reply_packets\":%u,\"core_bytes\":%zu,\"COM\":0,\"board\":0}\n",checks,failed,main_steps,main_gp,core_wire_packets,sizeof(li::CanonicalLocalExchangeCore));return failed?1:0;
}
