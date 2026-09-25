// Same real Cycle/Io/private74/phase C and real GP as the retained fixture.
// Only broker, raw sensor events/HRT, Authority and rotor provenance are MOCK.
#define GPENMPC_GP_PENDING_TEST_MAIN retained_gp_pending_fixture_main
#include "test_local_gp_pending.cpp"
#undef GPENMPC_GP_PENDING_TEST_MAIN
#include "px4_wire/CanonicalLocalSnapshotWire.hpp"
namespace sw=gpenmpc_local_snapshot_wire;
static unsigned snapshot_packets{};
// Bounded test receiver: verifies codec ordering, tuple and actual MAVLink CRC.
// Host-test dispatcher.
static bool snapshot_roundtrip(const sw::Bytes&bytes,sw::Bytes&joined,unsigned inject=0){
    joined={};unsigned offset=0;
    for(unsigned k=0;k<4;++k){const unsigned index=inject==1?(k==0?1:k==1?0:k):inject==2&&k==1?0:k;
        sw::Fragment f{};if(!sw::fragment(bytes,index,f))return false;
        mavlink_message_t sent{},got{},rx{};mavlink_status_t status{},reported{};
        mavlink_msg_tunnel_pack(inject==4?200:1,1,&sent,255,190,inject==5?200:42002,f.length,f.payload);
        uint8_t wire[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(wire,&sent);
        if(inject==3&&k==0)wire[n-1]^=1;
        unsigned ok=0,bad=0;for(unsigned j=0;j<n;++j){const auto result=mavlink_frame_char_buffer(&rx,&status,wire[j],&got,&reported);
            if(result==MAVLINK_FRAMING_OK)++ok;if(result==MAVLINK_FRAMING_BAD_CRC)++bad;}
        if(inject==3&&k==0)check(bad==1&&ok==0,"actual corrupted MAVLink frame yields BAD_CRC, not parse_char guess");
        if(ok!=1||bad||got.msgid!=MAVLINK_MSG_ID_TUNNEL)return false;
        ++snapshot_packets;mavlink_tunnel_t t{};mavlink_msg_tunnel_decode(&got,&t);
        const unsigned want=k==3?34:128;
        if(got.sysid!=1||got.compid!=1||t.target_system!=255||t.target_component!=190||t.payload_type!=42002||
            t.payload_length!=want||t.payload[0]!=(0xa0u|k)||std::memcmp(t.payload+1,bytes.data()+46,8))return false;
        const unsigned count=t.payload_length-9;if(offset+count>bytes.size())return false;
        std::memcpy(joined.data()+offset,t.payload+9,count);offset+=count;
    }
    return offset==bytes.size()&&same(bytes.data(),joined.data(),bytes.size());
}
static bool verify_snapshot(const od::Snapshot&s,const rr::Receipt&r,sw::Bytes&bytes){
    sw::Observation from{},decoded{};sw::Bytes arrived{};const auto clocks=clock_calls;
    check(sw::from_actual(s,r,from)&&sw::encode_from_actual(s,r,bytes),"RLS1 only from actual private source plus same Reader endpoint");
    check(snapshot_roundtrip(bytes,arrived)&&sw::decode(arrived,decoded),"four actual board-to-HOST TUNNEL42002 frames retain all382 bytes");
    gpenmpc_portable::Array<double,13> state{};ib::SnapshotKey key{};
    check(gpenmpc_snapshot_mapping::state13(s,state)&&ib::snapshot_key(s,key)&&ib::same_source(key,decoded.source)&&
        ib::same_source(key,r.original_snapshot_key),"original complete semantic source key is computed by actual Reader and survives codec");
    check(same(state.data(),decoded.canonical_state13,104)&&same(s.task_origin_ned_m().data(),decoded.task_origin_ned_m,24)&&
        decoded.odometry_instance==s.source_instance(),"all13 actual binary64 state values and translated origin exact");
    const auto&e=decoded.endpoint;const auto&t=r.original.topic;bool raw=true;
    for(unsigned i=0;i<65;++i)raw=raw&&sw::payload_byte(e,i)==t.original_payload[i];
    check(raw&&e.original_receiver_hrt_us==t.timestamp&&e.original_wire_time_us==t.wire_time_usec&&
        e.original_tap_event_ordinal==t.original_event_sequence&&e.fields_updated==t.fields_updated&&
        e.receiver_instance==t.receiver_instance&&e.channel==t.channel&&e.system==t.system_id&&e.component==t.component_id&&
        e.sequence==t.mavlink_sequence&&e.payload_length==t.payload_length&&e.sensor_id==t.sensor_id,
        "all65 original HIL payload bytes and original endpoint metadata exact, no timestamp mapping");
    check(decoded.gyro_instance==t.gyro_topic_instance&&decoded.accel_instance==t.accel_topic_instance&&
        decoded.gyro_device_id==t.gyro_device_id&&decoded.accel_device_id==t.accel_device_id&&
        decoded.gyro_update_called==t.gyro_update_called&&decoded.accel_update_called==t.accel_update_called&&
        decoded.original_subscription_generation==r.original.original_subscription_generation,
        "actual sensor instance/device fields, call evidence and subscription generation exact");
    check(clock_calls==clocks&&!r.receiver_to_estimator_lineage_proven&&!r.dll_association_proven&&!r.control_authority,
        "encoding observes no new HRT and grants neither freshness nor full lineage/association/authority");
    return sw::valid(decoded);
}
static bool known_sensor_receipt(gpenmpc_hil_endpoint::Px4OriginalHilReceipt&p,uint64_t stamp){
    mavlink_message_t sent{},got{},rx{};mavlink_status_t status{},reported{};
    mavlink_msg_hil_sensor_pack(255,0,&sent,UINT64_C(1788693896516000),1.25f,-2.5f,3.75f,4.25f,-5.5f,6.75f,7,8,9,10,11,12,13,0x1fffU,7);
    uint8_t wire[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(wire,&sent);unsigned accepted=0;
    for(unsigned j=0;j<n;++j)if(mavlink_frame_char_buffer(&rx,&status,wire[j],&got,&reported)==MAVLINK_FRAMING_OK)++accepted;
    if(accepted!=1)return false;++actual_parses;mavlink_hil_sensor_t hil{};mavlink_msg_hil_sensor_decode(&got,&hil);
    return p.record(got,hil,stamp,&fixture_receiver,&fixture_link,2,3,_MAV_PAYLOAD(&got),true,true,2,1,4211,4312);
}
static void pairing_negatives(const od::Snapshot&s,const rr::Receipt&r){
    // Negative Snapshots still come from the genuine private ingest factory,
    // never from a deserialized HOST Observation or an authority boolean.
    for(unsigned kind=0;kind<7;++kind){auto cfg=config_source();auto raw=s.raw();
        if(kind==0)raw.position[0]+=.01f;if(kind==1)cfg.task_origin_ned_m[0]=.25;
        if(kind==2){raw.reset_counter++;cfg.initial_reset_counter=raw.reset_counter;}
        if(kind==3)cfg.identity.uid++;if(kind==4)cfg.identity.boot_generation++;
        static int wrong_topic;if(kind==5)cfg.vehicle_odometry_topic=&wrong_topic;
        if(kind==6)cfg.instance=1;
        od::AtomicOdometryAdapter adapter(cfg);od::Snapshot other{};
        check(adapter.ingest(raw,s.subscription_generation(),s.estimator().board_rx_us,s.estimator().board_rx_us+1,
            cfg.identity,cfg.vehicle_odometry_topic,cfg.instance,other),"negative alternate source made only by actual private factory");
        sw::Observation out{};check(!sw::from_actual(other,r,out),"same generation/time but different state origin reset identity topic or instance cannot reuse actual receipt");
    }
    for(unsigned kind=0;kind<13;++kind){auto bad=r;sw::Observation out{};
        if(kind==0)bad.original.topic.timestamp++;if(kind==1)bad.original.topic.prior_publication_failures=1;
        if(kind==2)bad.original.topic.original_payload[0]^=1;if(kind==3)bad.original.topic.original_payload[60]^=1;
        if(kind==4)bad.original.topic.original_payload[64]^=1;if(kind==5)bad.original.topic.original_payload[8]^=1;
        if(kind==6)bad.original.topic.payload_length=1;if(kind==7)bad.original_snapshot_key.state_and_origin_sha256[0]^=1;
        if(kind==8)bad.original_source_topic=nullptr;if(kind==9)bad.original_source_instance++;
        if(kind==10)bad.endpoint.exact_unique_endpoint=false;if(kind==11)bad.original.topic.gyro_update_called=false;
        if(kind==12){bad.original.topic.payload_length=1;bad.endpoint.original.payload_length=1;}
        check(!sw::from_actual(s,bad,out),"altered receipt raw bytes metadata sourcekey call provenance or nonzero truncated tail refused");
    }
    sw::Bytes original_bytes{};check(sw::encode_from_actual(s,r,original_bytes),"negative codec seed from actual source");
    for(unsigned offset:{0u,107u,200u,381u}){auto b=original_bytes;b[offset]^=1;sw::Observation out{};
        check(!sw::decode(b,out),"magic or payload checksum corruption rejected");}
    for(unsigned kind=1;kind<=5;++kind){sw::Bytes joined{};check(!snapshot_roundtrip(original_bytes,joined,kind),
        "test bounded receiver refuses out of order duplicate CRC sender or payload-type errors");}
}
int wmain(int argc,wchar_t**argv){
    if(argc!=7||!load(argv[1],argv[2])||!load_private(argv[2])||!parse_hash(argv[4]))return 2;
    const auto frozen=gw::canonical_configuration();for(unsigned j=0;j<32;++j)config.configuration_sha256[j]=uint8_t(frozen[j/4]>>(24-8*(j%4)));
    HMODULE dll=LoadLibraryExW(argv[3],nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);if(!dll)return 3;
    const auto predict=reinterpret_cast<int(*)(const double*,double*)>(GetProcAddress(dll,"gpenmpc_gp256_predict"));if(!predict)return 3;
    FILE*rls=_wfopen(argv[5],L"wb");FILE*pairs=_wfopen(argv[6],L"wb");if(!rls||!pairs)return 4;
    unsigned completed=0,queries=0,disarmed=0,main_packets=0;
    {CycleOwner o;li::CanonicalLocalGpPending pending(*o.cycle);
        for(unsigned i=0;i<60;++i){StepInputs in{};if(!o.capture(i,in)){check(false,"actual capture");break;}
            sw::Bytes bytes{};check(verify_snapshot(*o.cycle->snapshot(),*o.cycle->original_endpoint(),bytes),"actual armed snapshot encoded");
            check(std::fwrite(bytes.data(),1,bytes.size(),rls)==bytes.size(),"write original actual encoded RLS bytes from this same run");
            check(o.cycle->execute(in.value)==li::LocalCycleExecute::Committed,"actual numeric publication and joint/phase installation");
            const auto phase=o.phase->diagnostics();const auto pubs=publish_calls;
            const auto begin=pending.observe_actual_commit();check(begin==(i?li::LocalGpBegin::RequestReady:li::LocalGpBegin::NotRequired),"real committed pending query lifecycle");
            if(begin==li::LocalGpBegin::Rejected)break;
            if(i){gw::ReplyBytes reply{};check(actual_reply(pending,predict,reply),"actual original GP over bidirectional42002 codec");
                const auto*request=pending.pending_bytes();check(request&&std::fwrite(request->data(),1,request->size(),pairs)==request->size()&&
                    std::fwrite(reply.data(),1,reply.size(),pairs)==reply.size(),"save actual310 request and286 predictor reply, no oracle injection");
                const auto arrival=pending.retained_actual_feedback().original_cycle_read_us+20;
                check(pending.accept_reply(reply,arrival,arrival+10),"only matching GP reply fills same owner");clock_us=arrival+11;++queries;
            }
            check(o.phase->diagnostics().installs==phase.installs&&same(&o.phase->diagnostics().phase_s,&phase.phase_s,8)&&pubs==publish_calls,
                "snapshot export and GP reply do not advance committed phase or add controls");++completed;
        }
        main_packets=snapshot_packets;check(completed==60&&queries==59&&gp_queries==59&&publish_calls==60&&gp_packets==354&&main_packets==240,
            "one sixty-source actual-C chain,59 actualGP calls,240 RLS and354 GP actual MAVLink frames");
    }
    auto cfg=config_source();cfg.task_origin_ned_m={.5,-.25,.125};
    {CycleOwner o(true,false,&cfg);
        for(unsigned i=0;i<3;++i){source(i,true);check(known_sensor_receipt(o.producer,bus_source.timestamp_sample)&&
                o.cycle->capture_disarmed()==li::LocalCapture::Accepted,"actual disarmed capture with nonzero origin and actual producer sensor fields");
            od::Snapshot s{};rr::Receipt r{};check(o.cycle->copy_disarmed_observation(s,r),"actual disarmed export");sw::Bytes b{};
            check(verify_snapshot(s,r,b)&&r.original.topic.payload_length==65&&r.original.topic.gyro_device_id==4211,
                "actual full65 payload and known sensor fields survive disarmed encoding");
            check(o.cycle->release_disarmed()&&no_math_or_publication(o),"disarmed observation and release install no scientific state");++disarmed;
        }
    }
    {CycleOwner o;check(observe(o,0)==li::LocalCapture::Accepted,"original snapshot negative setup");
        pairing_negatives(*o.cycle->snapshot(),*o.cycle->original_endpoint());}
    std::fclose(rls);std::fclose(pairs);FreeLibrary(dll);
    std::printf("{\"checks\":%u,\"failed\":%u,\"actual_cycle_rows\":%u,\"actual_GP_calls\":%u,\"disarmed_rows\":%u,\"RLS_records\":%u,\"RLS_bytes\":%u,\"GP_pair_records\":%u,\"GP_pair_bytes\":%u,\"main_RLS_packets\":%u,\"main_GP_packets\":%u,\"Observation_size\":%zu,\"Receipt_size\":%zu,\"schema\":10,\"payload_type\":42002,\"source_HRT_uORB_Authority_MOCK\":true,\"COM\":0}\n",
        checks,failed,completed,queries,disarmed,completed,completed*382,queries,queries*596,main_packets,gp_packets,sizeof(sw::Observation),sizeof(rr::Receipt));
    return failed?1:0;
}
