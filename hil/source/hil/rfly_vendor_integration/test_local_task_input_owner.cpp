#include "px4_runtime/CanonicalLocalTaskInputOwner.hpp"
#define GPENMPC_CORE_TEST_MAIN exchange_core_test_entry
#include "test_local_exchange_core.cpp"
#undef GPENMPC_CORE_TEST_MAIN
namespace tw=gpenmpc_local_task_wire;
static gpenmpc_rfly_px4::LocalTaskInputConfiguration input_config(const gpenmpc_full_inner_window&w){
    gpenmpc_rfly_px4::LocalTaskInputConfiguration c{};c.window.max_assembly_us=1000000;
    c.input_assembly_max_us=2000;c.outer_max_age_us=300000;
    auto&e=c.window.expected;e.identity=identity();e.leg_index=w.leg_index;
    e.configuration_sha256=gw::canonical_configuration();
    e.task_sha256=gpenmpc_full_consumption::detail::words(config.task_sha256);
    e.reference_asset_sha256=gpenmpc_full_consumption::detail::words(w.reference_asset_sha256);
    for(unsigned j=0;j<8;++j)e.execution_session_sha256[j]=j+1;return c;
}
static bool ingress_frame(gpenmpc_ingress::Receiver&rx,li::CanonicalLocalExchangeCore&core,
    const uint8_t*p,uint8_t length,li::LocalExchangePoll&result,bool disarmed=false){
    mavlink_message_t m{},got{},parser{};mavlink_status_t status{},reported{};
    mavlink_msg_tunnel_pack(255,190,&m,1,1,42002,length,p);uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
    const auto n=mavlink_msg_to_send_buffer(bytes,&m);unsigned decoded=0;
    for(unsigned j=0;j<n;++j)if(mavlink_frame_char_buffer(&parser,&status,bytes[j],&got,&reported)==MAVLINK_FRAMING_OK)++decoded;
    if(decoded!=1||rx.receive(got,hrt_absolute_time(),1,1,2)!=gpenmpc_ingress::Result::Accepted)return false;
    if(!rx.drain(1,[&](const gpenmpc_ingress::Fields&v){gpenmpc_full_inner_ingress_s t{};
        gpenmpc_ingress::copy_to_topic(v,rx.counters(),t);return gpenmpc_test_topic_publish(ORB_ID(gpenmpc_full_inner_ingress),&t);}))return false;
    result=core.poll(disarmed);return true;
}
static bool get_rls(li::CanonicalLocalExchangeCore&core,void*consumer,gpenmpc_local_snapshot_wire::Bytes&bytes){
    unsigned at=0,pieces=0;
    while(core.outbox().pending()){li::LocalWireFragment f{};
        if(core.outbox().take(consumer,f,hrt_absolute_time())!=li::LocalWireTake::Fragment||f.original.schema!=10)return false;
        const auto n=f.fragment.length-9;if(at+n>bytes.size())return false;
        std::memcpy(bytes.data()+at,f.fragment.payload+9,n);at+=n;++pieces;}
    return pieces==4&&at==bytes.size();
}
static tw::Message message(CycleOwner&o,unsigned i,const gpenmpc_rfly_px4::LocalTaskInputConfiguration&c){
    tw::Message m{};const auto*s=o.cycle->snapshot();if(!s)s=o.io->retained_latest_snapshot();if(!s)return m;StepInputs b{};
    if(!command(*s,i,b.backing))return m;const auto&r=b.backing.rotor;const auto&rv=r.original_observation;
    m.session_sha=c.window.expected.execution_session_sha256;m.configuration_sha=c.window.expected.configuration_sha256;
    m.task_sha=c.window.expected.task_sha256;m.reference_sha=c.window.expected.reference_asset_sha256;m.leg=c.window.expected.leg_index;
    m.source=r.source;if(o.cycle->original_endpoint())std::memcpy(m.original_sensor52,o.cycle->original_endpoint()->endpoint.original.original_sensor52,52);
    m.rotor_generation=rv.dll_generation;m.rotor_session=rv.dll_session;m.rotor_host_receive_ns=rv.original_host_receive_ns;
    m.rotor_sim_time_s=rv.original_sim_time_s;std::memcpy(m.rotor_n,rv.observed_thrust_n.data(),sizeof m.rotor_n);
    m.rotor_observation_sha=rv.original_observation_sha;m.rotor_association_sha=r.verified_association_receipt_sha256;
    m.payload_generation=b.backing.payload.original_schedule_generation;m.payload_evidence_sha=b.backing.payload.original_schedule_evidence_sha256;
    m.payload_kg=b.backing.payload.payload_kg;m.wind_generation=b.backing.wind.original_estimate_generation;
    m.wind_evidence_sha=b.backing.wind.original_estimate_evidence_sha256;
    std::memcpy(m.estimated_wind_xy,b.backing.wind.estimate_xy_mps.data(),sizeof m.estimated_wind_xy);
    m.outer_generation=i+1;m.outer_source_generation=m.source.source_generation;m.outer_sample_us=m.source.sample_us;
    m.outer_original_host_source_rx_ns=1000000000ULL+i*10000000ULL;
    m.outer_creation_ns=m.outer_original_host_source_rx_ns+1000;m.outer_expiry_ns=m.outer_original_host_source_rx_ns+300000000ULL;
    m.outer_target4[0]=rows[i].args[2];for(unsigned j=0;j<3;++j)m.outer_target4[j+1]=rows[i].args[6+j];return m;
}
static int operator_receiver_only(){
    // 's first nonzero yaw must pass the REAL six-fragment
    // receiver/observe/read path, not just the operator-reference helper.
    struct Case {double target[4];bool component,operator_mode,accepted;};
    const Case cases[]={
        {{0,0,0,0},true,true,true},
        {{-.0034734917733089568,0,0,0},true,true,true},
        {{.0034734917733089568,0,0,0},true,true,true},
        {{-1,-3,3,1},true,true,true},
        {{1,3,-3,-1},true,true,true},
        {{-1.000001,0,0,0},true,true,false},
        {{1.000001,0,0,0},true,true,false},
        {{0,3.000001,0,0},true,true,false},
        {{0,0,-3.000001,0},true,true,false},
        {{0,0,0,1.000001},true,true,false},
        {{0,0,0,0},true,false,true},
        {{.0034734917733089568,0,0,0},true,false,false},
        {{0,0,0,0},false,true,false}
    };
    for(const auto&test:cases){
        extra_clear();CycleOwner o(false);TaskOwner originals;
        auto cfg=input_config(originals.window);cfg.runtime_state_only=true;
        cfg.component_initialization=test.component;cfg.operator_reference=test.operator_mode;
        li::CanonicalLocalTaskInputOwner owner(cfg);auto port=owner.port();ss::Receipt receipt{};
        check(observe(o,0)==li::LocalCapture::Accepted,"original private snapshot for receiver regression");
        check(port.observe_source(port.owner,*o.cycle->snapshot(),receipt,true)==li::LocalTaskRead::Ready,"retain real source anchor");
        auto m=message(o,0,cfg);std::memcpy(m.outer_target4,test.target,sizeof test.target);
        tw::Bytes bytes{};bool delivered=tw::encode(m,bytes);
        for(unsigned j=0;j<tw::fragment_count&&delivered;++j){
            gpenmpc_argument_transport::Fragment f{};delivered=tw::fragment(bytes,j,f);
            gpenmpc_argument_transport::Arrival a{};a.fields.timestamp=clock_us;
            a.fields.payload_length=f.length;std::memcpy(a.fields.payload,f.payload,f.length);
            delivered=delivered&&owner.receive(a,clock_us);
        }
        check(delivered&&owner.diagnostics().messages==1,"real six-fragment input decoded and accepted structurally");
        const auto state=port.observe_source(port.owner,*o.cycle->snapshot(),receipt,false);
        if(test.accepted){
            check(state==li::LocalTaskRead::Ready&&owner.diagnostics().bindings==1&&
                owner.diagnostics().first_fault==li::LocalTaskInputFault::None,"valid RC yaw/XYZ input installed");
            li::LocalTaskView view{};
            check(port.read(port.owner,*o.cycle->snapshot(),receipt,view)==li::LocalTaskRead::Missing&&
                view.inputs.rotor&&same(view.inputs.target4,test.target,sizeof test.target),
                "read preserves all four command values; missing route window does not fabricate control");
        }else check(state==li::LocalTaskRead::Invalid&&!owner.diagnostics().bindings&&
            owner.diagnostics().first_fault==li::LocalTaskInputFault::OuterMutation,
            "out-of-range or wrong-mode input rejected before numerical installation");
        check(no_math_or_publication(o),"receiver-only fixture has no numerical control or hardware publication");
        o.cycle->release_disarmed();
    }
    std::printf("{\"scope\":\"ACTUAL_OPERATOR_INPUT_RECEIVER\",\"cases\":13,\"checks\":%u,\"failed\":%u,\"COM\":0,\"board\":0}\n",checks,failed);
    return failed?1:0;
}
#ifndef GPENMPC_TASK_INPUT_TEST_MAIN
#define GPENMPC_TASK_INPUT_TEST_MAIN wmain
#endif
int GPENMPC_TASK_INPUT_TEST_MAIN(int argc,wchar_t**argv){
    const bool receiver_only=argc==7&&std::wcscmp(argv[6],L"--operator-receiver-only")==0;
    if((argc!=6&&!receiver_only)||!load(argv[1],argv[2])||!load_private(argv[2])||!parse_hash(argv[4]))return 2;
    const auto c=gw::canonical_configuration();for(unsigned j=0;j<32;++j)config.configuration_sha256[j]=uint8_t(c[j/4]>>(24-8*(j%4)));
    if(receiver_only)return operator_receiver_only();
    HMODULE dll=LoadLibraryExW(argv[3],nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);if(!dll)return 3;
    using Predict=int(*)(const double*,double*);const auto predict=reinterpret_cast<Predict>(GetProcAddress(dll,"gpenmpc_gp256_predict"));if(!predict)return 3;
    FILE*out=_wfopen(argv[5],L"wb");if(!out)return 3;unsigned commits=0,matched=0;
    for(unsigned bad=0;bad<8;++bad){
        extra_clear();CycleOwner o(false);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner originals;
        auto cfg=input_config(originals.window);li::CanonicalLocalTaskInputOwner owner(cfg);auto xc=exchange_config();
        xc.export_active_snapshots=true;xc.transport.max_assembly_us=cfg.window.max_assembly_us;
        li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,owner.port(),xc);gpenmpc_ingress::Receiver rx;int consumer{};
        gpenmpc_local_window_wire::Encoder encoder(cfg.window.expected,originals.window);li::LocalExchangePoll last{};
        bool loaded=encoder.valid();
        for(unsigned j=0;j<244&&loaded;++j){gpenmpc_local_window_wire::Fragment f{};loaded=encoder.fragment(j,f)&&ingress_frame(rx,core,f.payload,f.length,last)&&last==li::LocalExchangePoll::Idle;}
        check(loaded&&core.diagnostics().window_loads==1,"real data owner shares Core's 244-fragment window receiver and scratch");
        tw::Message first{};
        const unsigned steps=bad==0?3:(bad==5?2:1);
        for(unsigned i=0;i<steps;++i){
            check(original_selected(o,i)&&core.poll(false)==li::LocalExchangePoll::Idle&&owner.diagnostics().bindings==i,
                "first source waits for actual task packet, never substitutes command for rotor lag");
            // After the first row bindings naturally remain nonzero.
            gpenmpc_local_snapshot_wire::Bytes rls{};check(get_rls(core,&consumer,rls),"actual active RLS source exported on same outbox");
            auto m=message(o,i,cfg);if(!i)first=m;
            if(bad==1)m.original_sensor52[0]^=1;
            if(bad==2){m.outer_sample_us=1;}
            if(bad==3)++m.source.publication_us;
            if(bad==4)++m.configuration_sha[0];
            if(bad==5&&i){m.outer_generation=first.outer_generation;m.outer_source_generation=first.outer_source_generation;m.outer_sample_us=first.outer_sample_us;
                m.outer_original_host_source_rx_ns=first.outer_original_host_source_rx_ns;m.outer_creation_ns=first.outer_creation_ns;m.outer_expiry_ns=first.outer_expiry_ns;
                std::memcpy(m.outer_target4,first.outer_target4,sizeof m.outer_target4);m.outer_target4[0]+=0.01;}
            tw::Bytes bytes{};tw::Message decoded{};check(tw::encode(m,bytes)&&tw::decode(bytes,decoded),"RLI647 numerical codec unchanged binary64/uint64 values");
            check(same(decoded.rotor_n,m.rotor_n,sizeof m.rotor_n)&&same(decoded.outer_target4,m.outer_target4,sizeof m.outer_target4)&&
                gpenmpc_local_input::same_source(decoded.source,m.source),"source rotor lag and held outer bits roundtrip");
            bool delivered=true;
            for(unsigned j=0;j<tw::fragment_count&&delivered;++j){gpenmpc_argument_transport::Fragment f{};
                delivered=tw::fragment(bytes,j,f);if(bad==6&&!j)f.payload[0]++;
                delivered=delivered&&ingress_frame(rx,core,f.payload,f.length,last);
                if(bad==7){clock_us+=3000;last=core.poll(false);break;}
                if(last==li::LocalExchangePoll::Fault)break;
            }
            const bool rejected=bad&&!(bad==5&&!i);
            if(rejected){check(delivered&&last==li::LocalExchangePoll::Fault&&core.diagnostics().commits==i,
                "invalid input/anchor/held-target/framing/timeout fails before any extra inner publication");break;}
            check(delivered&&last==li::LocalExchangePoll::Progress&&core.diagnostics().commits==i+1&&owner.diagnostics().bindings==i+1,
                "real owner data enters same Core, actual C inner and unique joint publication");
            if(!bad){++commits;check(std::fwrite(rls.data(),1,rls.size(),out)==rls.size()&&std::fwrite(bytes.data(),1,bytes.size(),out)==bytes.size(),"persist matched actual RLS/RLI bytes");++matched;}
            if(i){gw::RequestBytes request{};check(drain_outbox(core,&consumer,&request),"real next GP request remains independent of task ingress");
                gw::Request q{};gw::decode(request,q);gw::Reply reply{};reply.identity=q.identity;reply.source_timestamp_ns=q.source_timestamp_ns;
                reply.source_generation=q.source_generation;reply.output_generation=q.output_generation;reply.original_request_sha256=gw::digest(request.data(),request.size());reply.gp_model_sha256=q.gp_model_sha256;
                check(predict(q.request19+1,reply.result18)==0,"actual original GP DLL remains unchanged");gw::ReplyBytes b{};gw::encode(reply,b);
                for(unsigned j=0;j<3;++j){gw::Fragment f{};gw::fragment(b,j,f);if(!ingress_frame(rx,core,f.payload,f.length,last))return 6;}
                check(last==li::LocalExchangePoll::Idle&&core.diagnostics().gp_replies_filled==i,"same ingress accepts real GP reply without extra execution");}
        }
        core.stop();check(core.poll(false)==li::LocalExchangePoll::Fault,"stop keeps task receiver permanently retired");
    }
    // Disarmed preloading fixture: retain an observed
    // source for initial outer preparation without executing a control step.
    {
        extra_clear();CycleOwner o(false);ss::Px4SelectedSourceReader selected(*o.reader,identity());TaskOwner originals;
        source(0,true);bus_generation[0]=0; // fresh disarmed telemetry, no odometry update yet
        clock_us=bus_source.timestamp_sample-10000;bus_status.timestamp=clock_us;bus_mode.timestamp=clock_us;
        auto cfg=input_config(originals.window);li::CanonicalLocalTaskInputOwner owner(cfg);auto xc=exchange_config();
        xc.export_active_snapshots=true;xc.transport.max_assembly_us=cfg.window.max_assembly_us;
        li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,owner.port(),xc);gpenmpc_ingress::Receiver rx;int consumer{};
        gpenmpc_local_window_wire::Encoder encoder(cfg.window.expected,originals.window);li::LocalExchangePoll last{};
        bool loaded=encoder.valid();
        for(unsigned j=0;j<244&&loaded;++j){gpenmpc_local_window_wire::Fragment f{};
            loaded=encoder.fragment(j,f)&&ingress_frame(rx,core,f.payload,f.length,last,true)&&last==li::LocalExchangePoll::Idle;}
        check(loaded&&core.diagnostics().window_loads==1&&core.diagnostics().commits==0,
            "disarmed original window load has no control execution or phase publication");
        const auto prior_us=clock_us;const bool observed=original_selected(o,0,true);
        last=core.poll(true);
        if(last!=li::LocalExchangePoll::Progress)std::fprintf(stderr,"BOOTSTRAP_DIAG prior=%llu now=%llu core=%u io=%u cycle=%u input=%u\n",
            (unsigned long long)prior_us,(unsigned long long)clock_us,unsigned(core.diagnostics().first_fault),
            unsigned(o.io->diagnostics().first_fault),unsigned(o.cycle->diagnostics().first_fault),unsigned(owner.diagnostics().first_fault));
        check(observed&&last==li::LocalExchangePoll::Progress,
            "original disarmed observation follows actual selected source path");
        gpenmpc_local_snapshot_wire::Bytes rls{};check(get_rls(core,&consumer,rls),"actual disarmed RLS exported");
        gpenmpc_local_input::SnapshotKey first{};const auto*retained=o.io->retained_latest_snapshot();
        check(retained&&gpenmpc_local_input::snapshot_key(*retained,first),"retired observation remains readable");
        check(owner.diagnostics().sources_retained==1&&owner.diagnostics().outer_installs==0&&owner.diagnostics().bindings==0&&core.diagnostics().commits==0,
            "disarmed source anchor only; no input consumption outer install or numeric commit");
        check(original_selected(o,1)&&core.poll(false)==li::LocalExchangePoll::Idle,"next active source still waits for exact inputs");
        check(get_rls(core,&consumer,rls),"active controller RLS state");
        auto m=message(o,1,cfg);m.outer_source_generation=first.source_generation;m.outer_sample_us=first.sample_us;
        m.outer_original_host_source_rx_ns=1000000000ULL;m.outer_creation_ns=1000001000ULL;m.outer_expiry_ns=1300000000ULL;
        tw::Bytes bytes{};check(tw::encode(m,bytes),"fresh held outer references real disarmed anchor without renewing origin");
        bool delivered=true;
        for(unsigned j=0;j<tw::fragment_count&&delivered;++j){gpenmpc_argument_transport::Fragment f{};
            delivered=tw::fragment(bytes,j,f)&&ingress_frame(rx,core,f.payload,f.length,last);}
        check(delivered&&last==li::LocalExchangePoll::Progress&&core.diagnostics().commits==1&&owner.diagnostics().outer_installs==1,
            "first actual inner commit accepts retained disarmed source of initial outer solve");
        core.stop();
    }
    // Check the source, held-input and assembly deadlines.
    unsigned expired_index=1;
    while(expired_index<60&&raw_fixture[expired_index].timestamp_sample-raw_fixture[0].timestamp_sample<=
            gpenmpc_local_input::maximum_held_input_age_us)++expired_index;
    check(expired_index<60&&raw_fixture[9].timestamp_sample-raw_fixture[0].timestamp_sample>50000&&
        raw_fixture[9].timestamp_sample-raw_fixture[0].timestamp_sample<=100000,
        "fixture observations separate the 50-ms and 100-ms held-input boundaries");
    if(expired_index>=60)return 7;
    for(unsigned mode=0;mode<6;++mode){
        extra_clear();CycleOwner o(false);TaskOwner originals;
        auto cfg=input_config(originals.window);cfg.runtime_state_only=true;cfg.component_initialization=true;
        li::CanonicalLocalTaskInputOwner owner(cfg);auto port=owner.port();ss::Receipt receipt{};
        check(observe(o,0)==li::LocalCapture::Accepted,"actual first private disarmed snapshot");
        check(port.observe_source(port.owner,*o.cycle->snapshot(),receipt,true)==li::LocalTaskRead::Ready,"retain first observed source");
        auto m=message(o,0,cfg);for(auto&v:m.outer_target4)v=0;
        const auto newer_index=(mode==1||mode==2)?expired_index:9;
        if(mode){
            check(o.cycle->release_disarmed()&&observe(o,newer_index)==li::LocalCapture::Accepted,"newer private observation, original clocks unchanged");
        }
        if(mode==3)++m.source.publication_us; // an unknown exact source
        if(mode==4){m.source.sample_us=o.cycle->snapshot()->estimator().timestamp_sample_us+1;
            m.source.publication_us=m.source.sample_us+1;m.source.original_receipt_us=m.source.sample_us+2;}
        const auto deliver=[&](const tw::Message&input){
            tw::Bytes bytes{};if(!tw::encode(input,bytes))return false;
            for(unsigned j=0;j<tw::fragment_count;++j){gpenmpc_argument_transport::Fragment f{};
                if(!tw::fragment(bytes,j,f))return false;
                gpenmpc_argument_transport::Arrival a{};a.fields.timestamp=clock_us;
                a.fields.payload_length=f.length;std::memcpy(a.fields.payload,f.payload,f.length);
                if(!owner.receive(a,clock_us))return false;
            }return true;
        };
        check(deliver(m),"all six task fragments, original complete message");
        const auto state=port.observe_source(port.owner,*o.cycle->snapshot(),receipt,false);
        if(mode==3||mode==4){
            check(state==li::LocalTaskRead::Invalid&&owner.diagnostics().first_fault==li::LocalTaskInputFault::Source&&
                !owner.diagnostics().bindings&&!owner.diagnostics().outer_installs,"unknown and future source remain rejected");
        }else if(mode==1||mode==2){
            check(state==li::LocalTaskRead::Ready&&owner.diagnostics().first_fault==li::LocalTaskInputFault::None&&
                owner.diagnostics().missing_inputs==1&&!owner.diagnostics().bindings&&!owner.diagnostics().outer_installs,
                "aged known observation retired without installation or new timestamp");
            li::LocalTaskView unavailable{};
            check(port.read(port.owner,*o.cycle->snapshot(),receipt,unavailable)==li::LocalTaskRead::Missing&&
                !unavailable.inputs.rotor&&!unavailable.inputs.payload&&!unavailable.inputs.wind,
                "retired stale candidate provides no numerical input or control permission");
            auto fresh=message(o,newer_index,cfg);for(auto&v:fresh.outer_target4)v=0;
            check(deliver(fresh)&&port.observe_source(port.owner,*o.cycle->snapshot(),receipt,true)==li::LocalTaskRead::Ready&&
                owner.diagnostics().bindings==1&&owner.diagnostics().last_source==fresh.source.source_generation,
                "next fresh input accepted after retired observation, no session restart");
        }else {
            check(state==li::LocalTaskRead::Ready&&owner.diagnostics().bindings==1,"valid original held input installs within selected margin");
            li::LocalTaskView available{};
            // This focused observation fixture deliberately has no route
            // window, so read reports Missing after assembling valid inputs.
            check(port.read(port.owner,*o.cycle->snapshot(),receipt,available)==li::LocalTaskRead::Missing&&
                available.inputs.rotor&&available.inputs.rotor->source.sample_us==m.source.sample_us,
                "held input retains original timestamp, never substitutes a fresh source timestamp");
        }
        check(no_math_or_publication(o),"observation test creates no control publication");
        o.cycle->release_disarmed();
    }
    // fast native observations are not all HOST reply anchors.
    // The actual Core/export/receiver path must outlive 64 captures inside
    // 600 ms without enlarging its fixed pool or changing any source time.
    for(unsigned bad=0;bad<3;++bad){
        extra_clear();CycleOwner o(false,false,nullptr,false,true);TaskOwner originals;
        auto cfg=input_config(originals.window);cfg.runtime_state_only=true;
        cfg.input_assembly_max_us=50000;cfg.outer_max_age_us=600000;
        li::CanonicalLocalTaskInputOwner owner(cfg);ss::Px4SelectedSourceReader selected(*o.reader,identity());
        auto xc=exchange_config();xc.export_active_snapshots=true;xc.snapshot_transport_max_age_us=50000;
        li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,owner.port(),xc);
        int consumer{};tw::Message first{},exported{},latest{};bool observed=true;
        const auto base=raw_fixture[0].timestamp_sample;
        const auto generation=static_cast<uint32_t>(original[0].tags[1]);
        for(unsigned i=0;i<160&&observed;++i){
            // Host broker timing fixture.
            source(0,true);const auto sample=base+2000*i;
            bus_source.timestamp_sample=sample;bus_source.timestamp=sample+100;
            bus_status.timestamp=bus_mode.timestamp=bus_offboard.timestamp=sample+150;
            for(auto&g:bus_generation)g=generation+i;clock_us=sample+300;
            observed=core.poll(true)==li::LocalExchangePoll::Progress;
            latest=message(o,0,cfg);
            if(observed&&core.outbox().numerical_pending()){
                gpenmpc_local_snapshot_wire::Bytes rls{};
                observed=get_rls(core,&consumer,rls);exported=latest;if(!i)first=latest;
            }
        }
        check(observed&&owner.diagnostics().first_fault==li::LocalTaskInputFault::None&&
            core.diagnostics().snapshots_enqueued==16&&owner.diagnostics().sources_retained==16&&
            latest.source.source_generation>exported.source.source_generation,
            "160 fast observations retain only 16 actual RLS anchors; fixed64 pool and clocks unchanged");
        auto m=bad==1?latest:exported;
        if(bad==2){m.outer_source_generation=first.outer_source_generation;m.outer_sample_us=first.outer_sample_us;
            m.outer_original_host_source_rx_ns=first.outer_original_host_source_rx_ns;
            m.outer_creation_ns=first.outer_creation_ns;m.outer_expiry_ns=first.outer_expiry_ns;}
        tw::Bytes bytes{};bool delivered=observed&&tw::encode(m,bytes);gpenmpc_ingress::Receiver rx;
        li::LocalExchangePoll last{};
        for(unsigned j=0;j<tw::fragment_count&&delivered;++j){gpenmpc_argument_transport::Fragment f{};
            delivered=tw::fragment(bytes,j,f)&&ingress_frame(rx,core,f.payload,f.length,last,true);}
        if(!bad)check(delivered&&last!=li::LocalExchangePoll::Fault&&owner.diagnostics().bindings==1&&
            owner.diagnostics().outer_installs==1,"actual exported input still installs through receive with no new capture");
        else check(delivered&&last==li::LocalExchangePoll::Fault&&!owner.diagnostics().bindings&&
            owner.diagnostics().first_fault==(bad==1?li::LocalTaskInputFault::Source:li::LocalTaskInputFault::OuterExpired),
            "unexported source and original expired outer command cannot acquire authority");
        check(core.diagnostics().commits==0&&no_math_or_publication(o),"prearm anchor grants no control, GP or publication authority");
    }
    // Retain complete input until the next uORB observation. Exercise ingress
    // in each poll mode while preserving source and packet timestamps.
    for(unsigned mode=0;mode<5;++mode){
        extra_clear();CycleOwner o(false);TaskOwner originals;
        auto cfg=input_config(originals.window);cfg.input_assembly_max_us=50000;
        cfg.runtime_state_only=true;cfg.component_initialization=mode!=4;
        li::CanonicalLocalTaskInputOwner owner(cfg);auto port=owner.port();ss::Receipt receipt{};
        check(observe(o,0)==li::LocalCapture::Accepted,"expiry test uses original private source");
        check(port.observe_source(port.owner,*o.cycle->snapshot(),receipt,true)==li::LocalTaskRead::Ready,"source anchored before reception");
        auto m=message(o,0,cfg);for(auto&v:m.outer_target4)v=0;
        if(mode==3)++m.source.publication_us;
        tw::Bytes bytes{};check(tw::encode(m,bytes),"expiry test retains complete source bytes");
        const auto first=clock_us;bool delivered=true;
        for(unsigned j=0;j<(mode==2?1:tw::fragment_count);++j){
            gpenmpc_argument_transport::Fragment f{};delivered=delivered&&tw::fragment(bytes,j,f);
            gpenmpc_argument_transport::Arrival a{};a.fields.timestamp=clock_us;a.fields.payload_length=f.length;
            std::memcpy(a.fields.payload,f.payload,f.length);delivered=delivered&&owner.receive(a,clock_us);
        }
        check(delivered,"original packet or deliberately incomplete first fragment retained");
        auto xc=exchange_config();gpenmpc_local_ingress::CanonicalLocalIngress ingress(xc.transport,nullptr,&owner);
        clock_us=first+50000;check(ingress.tick(clock_us,mode!=1),"original 50-ms boundary unchanged");
        clock_us=first+50001;const bool accepted=ingress.tick(clock_us,mode!=1);
        if(mode==0||mode==4){
            check(accepted&&!ingress.failed()&&owner.diagnostics().missing_inputs==1&&
                !owner.diagnostics().bindings&&!owner.diagnostics().outer_installs&&
                same(owner.retained_raw_message().data(),bytes.data(),bytes.size()),
                "complete known disarmed input retired without new uORB sample, install or timestamp renewal");
            check(ingress.tick(clock_us+1000,true)&&owner.diagnostics().missing_inputs==1,
                "retirement is once only and reception remains live");
        }else check(!accepted&&ingress.failed()&&owner.diagnostics().missing_inputs==0&&
            owner.diagnostics().first_fault==(mode==3?li::LocalTaskInputFault::Source:li::LocalTaskInputFault::Expired),
            "active, partial and unknown source expiration stay fail-closed");
        check(no_math_or_publication(o),"expiry retirement never executes control");
        o.cycle->release_disarmed();
    }
    // Install input for an exported, released observation before the next capture.
    for(unsigned mode=0;mode<4;++mode){
        extra_clear();CycleOwner o(false,false,nullptr,mode!=3,true);TaskOwner originals;
        auto cfg=input_config(originals.window);cfg.input_assembly_max_us=50000;
        cfg.runtime_state_only=true;cfg.component_initialization=mode!=3;
        li::CanonicalLocalTaskInputOwner owner(cfg);auto port=owner.port();ss::Receipt receipt{};
        check(observe(o,0)==li::LocalCapture::Accepted,"component ingress test captures actual private source");
        check(port.observe_source(port.owner,*o.cycle->snapshot(),receipt,true)==li::LocalTaskRead::Ready,"component source retained before its host response");
        auto m=message(o,0,cfg);for(auto&v:m.outer_target4)v=0;
        if(mode==2||mode==3)m.outer_target4[0]=.1;
        tw::Bytes bytes{};check(tw::encode(m,bytes)&&o.cycle->release_disarmed(),"original message retained after observation release");
        ss::Px4SelectedSourceReader selected(*o.reader,identity());auto xc=exchange_config();
        li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,owner.port(),xc);gpenmpc_ingress::Receiver rx;
        li::LocalExchangePoll last{};bool delivered=true;const auto first=clock_us;
        for(unsigned j=0;j<(mode==1?1:tw::fragment_count)&&delivered;++j){
            gpenmpc_argument_transport::Fragment f{};
            delivered=tw::fragment(bytes,j,f)&&ingress_frame(rx,core,f.payload,f.length,last,true);
        }
        if(mode==0||mode==3){
            check(delivered&&owner.diagnostics().bindings==1&&owner.diagnostics().outer_installs==1&&
                core.diagnostics().commits==0,"complete input installs without a new uORB capture or control commit; full mode preserves nonzero outer command");
            clock_us=first+50001;last=core.poll(true);
            check(last!=li::LocalExchangePoll::Fault&&owner.diagnostics().bindings==1&&
                owner.diagnostics().missing_inputs==0,"consumed input cannot expire in the assembly slot while capture has NoUpdate");
        }else if(mode==1){
            clock_us=first+50001;last=core.poll(true);
            check(delivered&&last==li::LocalExchangePoll::Fault&&owner.diagnostics().first_fault==li::LocalTaskInputFault::Expired&&
                owner.diagnostics().bindings==0,"partial input still expires at original 50 ms without installation");
        }else check(delivered&&last==li::LocalExchangePoll::Fault&&owner.diagnostics().first_fault==li::LocalTaskInputFault::OuterMutation&&
            owner.diagnostics().bindings==0,"non-nominal component command remains rejected before installation");
        check(no_math_or_publication(o),"input reception does not execute stale state or publish any control");
    }
    // interrupt one partial component-history body with the real
    // generated source snapshot. Same codecs and fragment order, no replay
    // timestamps renewed and no second stream or numerical authority.
    {extra_clear();CycleOwner o(false,false,nullptr,true);TaskOwner task;
        ss::Px4SelectedSourceReader selected(*o.reader,identity());auto xc=exchange_config();
        xc.export_active_snapshots=true;xc.snapshot_transport_max_age_us=50000;
        li::CanonicalLocalExchangeCore core(*o.io,*o.cycle,selected,{&task,&TaskOwner::read,&TaskOwner::get_window},xc);
        int consumer{};gpenmpc_local_snapshot_wire::Bytes sb{};
        check(original_selected(o,0)&&core.poll(false)==li::LocalExchangePoll::Progress&&get_rls(core,&consumer,sb),
            "priority fixture retains actual generated-C commit and original snapshot");
        li::LocalNumericalObservation numerical{};
        gpenmpc_local_committed_wire::Observation co{};gpenmpc_local_committed_wire::Bytes cb{};
        check(o.io->copy_local_numerical_state(numerical)&&
            gpenmpc_local_committed_wire::from_actual(numerical,o.cycle->committed_phase_observation(),co)&&
            gpenmpc_local_committed_wire::encode(co,cb),"priority history is actual closed record, not synthetic authority");
        li::LocalWireConfiguration wc{};wc.expected_session=identity();wc.producer_identity=&task;
        wc.snapshot_transport_max_age_us=wc.gp_request_transport_max_age_us=50000;wc.target_system=255;wc.target_component=190;
        li::CanonicalLocalWireOutbox box(wc);li::LocalWireFragment f{};
        check(box.publish_committed(cb,&task,true)==li::LocalWirePublish::Published&&
            box.history_pending()&&!box.numerical_pending(),"history no longer consumes numerical slot");
        check(box.take(&consumer,f,clock_us)==li::LocalWireTake::Fragment&&(f.fragment.payload[0]&15)==0&&
            f.original.schema==14&&f.original.yield_after_fragment,"component history retains one-fragment scheduling");
        gpenmpc_local_committed_wire::Bytes recovered{};std::memcpy(recovered.data(),f.fragment.payload+9,f.fragment.length-9);
        check(box.publish_snapshot(sb,&task)==li::LocalWirePublish::Published&&box.numerical_pending(),
            "fresh RLS accepted while history is partially sent");
        check(box.publish_committed(cb,&task,true)==li::LocalWirePublish::Busy&&box.fault()==li::LocalWireFault::None,
            "second history cannot overwrite pending record");
        gpenmpc_local_snapshot_wire::Bytes recovered_source{};
        for(unsigned j=0;j<4;++j){
            check(box.take(&consumer,f,clock_us)==li::LocalWireTake::Fragment&&f.original.schema==10&&
                (f.fragment.payload[0]&15)==j,"numerical fragments preempt pending history in original order");
            std::memcpy(recovered_source.data()+j*119,f.fragment.payload+9,f.fragment.length-9);
        }
        check(recovered_source==sb&&!box.numerical_pending()&&box.history_pending(),"priority source bytes unchanged");
        for(unsigned j=1;j<13;++j){
            check(box.take(&consumer,f,clock_us)==li::LocalWireTake::Fragment&&f.original.schema==14&&
                (f.fragment.payload[0]&15)==j,"same history resumes without missing or duplicate fragment");
            std::memcpy(recovered.data()+j*119,f.fragment.payload+9,f.fragment.length-9);
        }
        check(recovered==cb&&!box.pending(),"entire original history survives priority interleave");
        li::CanonicalLocalWireOutbox timely(wc);
        check(timely.publish_committed(cb,&task,true,true)==li::LocalWirePublish::Published,
            "full outer observation shares existing history lane");
        recovered={};
        for(unsigned j=0;j<13;++j){
            check(timely.take(&consumer,f,clock_us)==li::LocalWireTake::Fragment&&
                f.original.schema==14&&(f.fragment.payload[0]&15)==j&&
                f.original.component_history_only&&
                f.original.yield_after_fragment==((j+1)%gpenmpc_local_snapshot_wire::fragment_count==0)&&
                f.original.original_anchor_us==co.original_publication_us&&
                f.original.original_transport_valid_until_us==co.original_publication_us+50000,
                "full observation yields at RLS-sized boundaries without rewriting original times");
            std::memcpy(recovered.data()+j*119,f.fragment.payload+9,f.fragment.length-9);
            if(j==0){
                check(timely.publish_snapshot(sb,&task)==li::LocalWirePublish::Published,
                    "numerical source can preempt partial full observation");
                for(unsigned k=0;k<4;++k)
                    check(timely.take(&consumer,f,clock_us)==li::LocalWireTake::Fragment&&
                        f.original.schema==10&&(f.fragment.payload[0]&15)==k,
                        "full observation retains per-fragment numerical priority");
            }
        }
        check(recovered==cb&&!timely.pending(),"full observation payload is byte-identical and complete");
        li::CanonicalLocalWireOutbox expired(wc);
        check(expired.publish_committed(cb,&task,true)==li::LocalWirePublish::Published&&
            expired.publish_snapshot(sb,&task)==li::LocalWirePublish::Published&&
            expired.take(&consumer,f,raw_fixture[0].timestamp_sample+50001)==li::LocalWireTake::Expired,
            "priority never bypasses original numerical source expiry");
    }
    check(commits==3&&matched==3&&std::fclose(out)==0,"three prior input-owner/C steps and matched records plus one new bootstrap commit");FreeLibrary(dll);
    std::printf("{\"checks\":%u,\"failed\":%u,\"actual_commits\":%u,\"owner_bytes\":%zu,\"COM\":0,\"board\":0}\n",checks,failed,commits,sizeof(li::CanonicalLocalTaskInputOwner));return failed?1:0;
}
