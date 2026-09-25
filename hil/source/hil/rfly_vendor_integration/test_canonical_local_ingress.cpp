#define GPENMPC_GP_PENDING_TEST_MAIN retained_pending_main
#include "test_local_gp_pending.cpp"
#undef GPENMPC_GP_PENDING_TEST_MAIN
#include "px4_wire/CanonicalLocalIngress.hpp"
#include <uORB/topics/gpenmpc_full_inner_ingress.h>
namespace ing=gpenmpc_local_ingress;
static unsigned ingress_packets{},main_ingress_packets{},mock_topic_overflows{};
static ing::Configuration ingress_config(){ing::Configuration c{};c.source_system=255;c.source_component=190;c.target_system=1;c.target_component=1;c.receiver_instance=2;
    c.max_assembly_us=2000;return c;} // Test assembly budget.
struct Peer {
    gpenmpc_ingress::Receiver receiver;std::uint32_t generation=100;
    std::deque<ing::Arrival>queue;
    bool push(const gw::Fragment&f,uint64_t original,bool drain=true){
        mavlink_message_t sent{},got{},rx{};mavlink_status_t state{},reported{};
        mavlink_msg_tunnel_pack(255,190,&sent,1,1,42002,f.length,f.payload);
        uint8_t wire[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(wire,&sent);unsigned frames=0;
        for(unsigned i=0;i<n;++i)if(mavlink_frame_char_buffer(&rx,&state,wire[i],&got,&reported)==MAVLINK_FRAMING_OK)++frames;
        if(frames!=1)return false;++ingress_packets;
        const auto result=receiver.receive(got,original,1,1,2);
        if(drain)flush();return result==gpenmpc_ingress::Result::Accepted;
    }
    bool flush(bool success=true){return receiver.drain(1,[&](const gpenmpc_ingress::Fields&f){
        if(!success)return false;
        gpenmpc_full_inner_ingress_s topic{};gpenmpc_ingress::copy_to_topic(f,receiver.counters(),topic);
        if(queue.size()==gpenmpc_full_inner_ingress_s::ORB_QUEUE_LENGTH){queue.pop_front();++mock_topic_overflows;}
        queue.push_back(gpenmpc_argument_transport::from_topic(topic,++generation));return true;
    });}
    bool next(const gw::ReplyBytes&b,unsigned index,uint64_t original,ing::Arrival&out){gw::Fragment f{};if(!gw::fragment(b,index,f)||!push(f,original)||queue.empty())return false;out=queue.front();queue.pop_front();return true;}
};
static bool expected(ing::CanonicalLocalIngress&in,const li::CanonicalLocalGpPending&p,uint64_t now){
    const auto*b=p.pending_bytes();return b&&in.expect_gp_reply(*b,p.retained_actual_feedback().original_cycle_read_us,now);
}
static bool receive_all(ing::CanonicalLocalIngress&in,Peer&peer,const gw::ReplyBytes&b,uint64_t start,uint64_t now){
    for(unsigned k=0;k<3;++k){ing::Arrival a{};if(!peer.next(b,k,start+k,a)||!in.receive(a,now))return false;}return true;
}
int wmain(int argc,wchar_t**argv){
    if(argc!=5||!load(argv[1],argv[2])||!load_private(argv[2])||!parse_hash(argv[4]))return 2;
    const auto cfg=gw::canonical_configuration();for(unsigned j=0;j<32;++j)config.configuration_sha256[j]=static_cast<uint8_t>(cfg[j/4]>>(24-8*(j%4)));
    HMODULE dll=LoadLibraryExW(argv[3],nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);if(!dll)return 3;
    using Predict=int(*)(const double*,double*);auto predict=reinterpret_cast<Predict>(GetProcAddress(dll,"gpenmpc_gp256_predict"));if(!predict)return 3;
    unsigned main_fills=0;
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);ing::CanonicalLocalIngress dispatcher(ingress_config());Peer peer;
        for(unsigned i=0;i<60;++i){StepInputs step{};check(o.capture(i,step)&&o.cycle->execute(step.value)==li::LocalCycleExecute::Committed,"actual source/Cycle/fullC commit before GP request");
            const auto begin=p.observe_actual_commit();check(begin==(i?li::LocalGpBegin::RequestReady:li::LocalGpBegin::NotRequired),"actual Pending request, no synthetic host request");
            if(!i)continue;const auto ready=p.retained_actual_feedback().original_cycle_read_us;gw::ReplyBytes raw{};
            check(expected(dispatcher,p,ready+1)&&actual_reply(p,predict,raw),"expect exact real Pending bytes and compute actual GP256");
            const auto pubs=publish_calls;check(receive_all(dispatcher,peer,raw,ready+20,ready+30),"actual MAVLink/Receiver/generatedtopic/Arrival to central dispatcher");
            ing::Completed complete{};check(dispatcher.take_gp_reply(complete,ready+31)&&complete.complete&&same(complete.bytes.data(),raw.data(),raw.size())&&
                complete.first_original_arrival_us==ready+20&&complete.last_original_arrival_us==ready+22&&complete.completed_processing_us==ready+30,
                "complete exact286 bytes with distinct original first/last/processing HRT");
            for(unsigned k=0;k<3;++k)check(complete.fragments[k].fields.timestamp==ready+20+k&&complete.fragments[k].fields.reception_sequence==uint64_t((i-1)*3+k+1)&&
                complete.fragments[k].uorb_generation==uint64_t(100+(i-1)*3+k+1)&&complete.fragments[k].fields.payload_type==42002,
                "original receiver/uORB sequence and per-fragment HRT never synthesized by dispatcher");
            check(!complete.control_authority&&!complete.source_freshness_granted&&p.accept_reply(complete.bytes,complete.last_original_arrival_us,ready+32)&&p.diagnostics().fills==i&&publish_calls==pubs,
                "existing actual Pending independently matches and fills without control/source action");
            clock_us=ready+33;++main_fills;
        }
        main_ingress_packets=ingress_packets;check(main_fills==59&&main_ingress_packets==177&&dispatcher.diagnostics().envelopes_admitted==177&&dispatcher.diagnostics().taken==59,
            "one nonresetting dispatcher across59 actual pending replies");}
    for(unsigned kind=0;kind<31;++kind){CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"negative actual Cycle/Pending setup");
        ing::CanonicalLocalIngress dispatcher(ingress_config());Peer peer;const auto ready=p.retained_actual_feedback().original_cycle_read_us;gw::ReplyBytes raw{};
        check(expected(dispatcher,p,ready+1)&&actual_reply(p,predict,raw),"negative starts from genuine pending and actualGP reply");
        if(kind==26){gw::Reply wrong{};gw::decode(raw,wrong);wrong.original_request_sha256[0]^=1;check(gw::encode(wrong,raw),"well-formed checksum but wrong request binding");}
        bool accepted=true;ing::Arrival failing{};
        for(unsigned k=0;k<3&&accepted;++k){ing::Arrival a{};check(peer.next(raw,k,ready+20+k,a),"negative uses actual frame/parser/Receiver/topic before explicit fault injection");
            const bool second=(kind==13||kind==14||kind==17||kind==19||kind==30),last=(kind==23||kind==24);
            if(k==(last?2U:second?1U:0U)){
                if(kind==0)++a.fields.source_system;if(kind==1)++a.fields.source_component;if(kind==2)++a.fields.target_system;if(kind==3)++a.fields.target_component;
                if(kind==4)++a.fields.receiver_instance;if(kind==5)a.fields.payload_type=200;
                if(kind==6)a.ingress_first_fault=1;if(kind==7)a.ingress_last_fault=1;if(kind==8)a.ingress_first_fault_hrt=1;if(kind==9)a.ingress_last_fault_hrt=1;
                if(kind==10)a.ingress_rejected_total=1;if(kind==11)a.ingress_queue_overflows=1;if(kind==12)a.ingress_publication_failures=1;
                if(kind==13)++a.fields.reception_sequence;if(kind==14)++a.uorb_generation;if(kind==15)a.fields.timestamp=0;if(kind==16)a.fields.timestamp=ready+100;
                if(kind==17)a.fields.timestamp=ready+19;if(kind==18)a.fields.payload[0]=0x80;
                if(kind==19)a.fields.payload[0]=0x90;if(kind==20)a.fields.payload[0]=0x92;if(kind==21)a.fields.payload[8]^=1;
                if(kind==22)--a.fields.payload_length;if(kind==23)a.fields.payload[127]=1;if(kind==24)a.fields.wire_payload_length=133;
                if(kind==25)a.fields.payload[18]^=1;if(kind==27)a.uorb_generation=0;if(kind==28)a.uorb_generation=UINT64_MAX;
                if(kind==29)a.fields.timestamp=ready-1;if(kind==30)--a.fields.reception_sequence;
            }
            accepted=dispatcher.receive(a,ready+30);if(!accepted)failing=a;
        }
        check(!accepted&&dispatcher.failed()&&!dispatcher.ready()&&p.diagnostics().fill_attempts==0&&p.diagnostics().fills==0,"source/sequence/fault/time/schema/length/digest mismatch cannot reach numerical fill");
        const auto*bad=dispatcher.first_fault_arrival();check(bad&&bad->fields.timestamp==failing.fields.timestamp&&same(bad->fields.payload,failing.fields.payload,128),"exact first rejected fragment retained even after checksum fault");
        const auto fault=dispatcher.diagnostics().first_fault;const auto count=dispatcher.received_fragments();
        check(!dispatcher.tick(ready+40)&&!expected(dispatcher,p,ready+41)&&!dispatcher.receive(failing,ready+42)&&dispatcher.diagnostics().first_fault==fault&&dispatcher.received_fragments()==count,
            "first fault permanent, no new expectation or silent sequence reset");
        if(count)check(dispatcher.audit_fragment(0)&&dispatcher.retained_reply()&&dispatcher.retained_request(),"accepted partial fragment and exact expected bytes remain auditable");
    }
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"timeout real pending");ing::CanonicalLocalIngress in(ingress_config());Peer peer;gw::ReplyBytes raw{};
        const auto t=p.retained_actual_feedback().original_cycle_read_us;check(expected(in,p,t+1)&&actual_reply(p,predict,raw),"timeout expectation");ing::Arrival a{};peer.next(raw,0,t+20,a);check(in.receive(a,t+30),"timeout retains partial original");
        check(!in.tick(t+2001)&&in.diagnostics().first_fault==ing::Fault::Expired&&!in.first_fault_arrival()&&in.audit_fragment(0),"missing two fragments expires explicit original-ready budget; no fictitious failure packet");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"ready timeout actual pending");ing::CanonicalLocalIngress in(ingress_config());Peer peer;gw::ReplyBytes raw{};
        const auto t=p.retained_actual_feedback().original_cycle_read_us;expected(in,p,t+1);actual_reply(p,predict,raw);check(receive_all(in,peer,raw,t+20,t+30),"ready before timeout");ing::Completed out{};
        check(!in.take_gp_reply(out,t+2001)&&in.diagnostics().first_fault==ing::Fault::Expired&&!out.complete&&in.received_fragments()==3,"assembled but unconsumed reply also expires without erasing raw");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"expiry is engineering parameter not fixed five-ms");auto c=ingress_config();c.max_assembly_us=10000;ing::CanonicalLocalIngress in(c);Peer peer;gw::ReplyBytes raw{};
        const auto t=p.retained_actual_feedback().original_cycle_read_us;expected(in,p,t+1);actual_reply(p,predict,raw);check(receive_all(in,peer,raw,t+6000,t+6002),"same original packet after6ms accepted under explicit10ms transport config, no lease renewal");
        ing::Completed out{};check(in.take_gp_reply(out,t+6003)&&!out.control_authority&&!out.source_freshness_granted,"transport acceptance still grants nothing");}
    for(unsigned kind=0;kind<2;++kind){CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);two_commits(o,p);ing::CanonicalLocalIngress in(ingress_config());Peer peer;gw::ReplyBytes raw{};
        const auto t=p.retained_actual_feedback().original_cycle_read_us;expected(in,p,t+1);actual_reply(p,predict,raw);gw::Fragment f{};gw::fragment(raw,0,f);
        if(kind==0){for(unsigned k=0;k<9;++k)peer.push(f,t+20+k,false);peer.flush();}
        else{peer.push(f,t+20,false);check(!peer.flush(false),"actual Receiver publisher callback fails");peer.flush();}
        check(!peer.queue.empty()&&!in.receive(peer.queue.front(),t+50)&&in.diagnostics().first_fault==ing::Fault::IngressFault&&p.diagnostics().fills==0,
            "actual receiver queue8 overflow/publication failure watermarks propagate into generated topic and reject");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);two_commits(o,p);ing::CanonicalLocalIngress in(ingress_config());Peer peer;gw::ReplyBytes raw{};
        const auto t=p.retained_actual_feedback().original_cycle_read_us;expected(in,p,t+1);actual_reply(p,predict,raw);ing::Arrival first{};peer.next(raw,0,t+20,first);check(in.receive(first,t+30),"real queue overwrite baseline admitted");
        gw::Fragment f{};gw::fragment(raw,1,f);for(unsigned k=0;k<=gpenmpc_full_inner_ingress_s::ORB_QUEUE_LENGTH;++k)peer.push(f,t+40+k);
        check(peer.queue.size()==gpenmpc_full_inner_ingress_s::ORB_QUEUE_LENGTH&&!in.receive(peer.queue.front(),t+100)&&in.diagnostics().first_fault==ing::Fault::SequenceGap,"actual generated-capacity mock uORB overwrite cannot be repaired by replacing sequence");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);two_commits(o,p);ing::CanonicalLocalIngress in(ingress_config());Peer peer;gw::ReplyBytes raw{};
        const auto t=p.retained_actual_feedback().original_cycle_read_us;expected(in,p,t+1);actual_reply(p,predict,raw);receive_all(in,peer,raw,t+20,t+30);ing::Completed out{};check(in.take_gp_reply(out,t+31),"replay completed reply taken once");
        check(!expected(in,p,t+32)&&in.diagnostics().first_fault==ing::Fault::Replay,"same real pending bytes cannot start a second transport transaction");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);two_commits(o,p);ing::CanonicalLocalIngress in(ingress_config());Peer peer;gw::ReplyBytes raw{};
        const auto t=p.retained_actual_feedback().original_cycle_read_us;expected(in,p,t+1);actual_reply(p,predict,raw);receive_all(in,peer,raw,t+20,t+30);ing::Completed out{};in.take_gp_reply(out,t+31);
        ing::Arrival repeated{};check(peer.next(raw,0,t+40,repeated)&&!in.receive(repeated,t+50)&&in.diagnostics().first_fault==ing::Fault::NoExpectation&&in.diagnostics().taken==1,
            "real retransmission with new receiver sequence cannot fill a consumed transport again");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);two_commits(o,p);ing::CanonicalLocalIngress in(ingress_config());Peer peer;gw::ReplyBytes raw{};
        const auto t=p.retained_actual_feedback().original_cycle_read_us;expected(in,p,t+1);actual_reply(p,predict,raw);ing::Arrival a{};peer.next(raw,0,t+20,a);
        a.uorb_generation=UINT32_MAX;check(in.receive(a,t+30),"explicit counter-exhaustion negative fixture records actual-width maximum");peer.next(raw,1,t+21,a);
        check(!in.receive(a,t+31)&&in.diagnostics().first_fault==ing::Fault::CounterOverflow,"uORB counter exhaustion cannot wrap into a new baseline");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);two_commits(o,p);ing::CanonicalLocalIngress in(ingress_config());Peer peer;gw::ReplyBytes raw{};
        const auto t=p.retained_actual_feedback().original_cycle_read_us;expected(in,p,t+1);actual_reply(p,predict,raw);ing::Arrival a{};peer.next(raw,0,t+20,a);in.receive(a,t+30);in.stop(t+31);
        check(in.diagnostics().first_fault==ing::Fault::Stopped&&in.audit_fragment(0)&&!in.first_fault_arrival()&&!in.tick(t+32)&&!expected(in,p,t+33),"explicit stop retains raw but does not invent a failure packet or permit reset");}
    FreeLibrary(dll);std::printf("{\"checks\":%u,\"failed\":%u,\"main_actual_Cycle_commits\":60,\"main_GP_fills\":%u,\"main_actual_receiver_packets\":%u,\"all_actual_receiver_packets\":%u,\"uORB_queue_overflows_MOCK\":%u,\"dispatcher_bytes\":%zu,\"Completed_bytes\":%zu,\"actual_clock_uORB_authority_MOCK\":true,\"COM\":0,\"board\":0}\n",checks,failed,main_fills,main_ingress_packets,ingress_packets,mock_topic_overflows,sizeof(ing::CanonicalLocalIngress),sizeof(ing::Completed));return failed?1:0;
}
