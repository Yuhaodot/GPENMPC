#include "Px4OriginalHilReceiptReader.hpp"
#include "Px4OriginalHilReceipt.hpp"
#include <common/mavlink.h>
#include <cstdio>
#include <cstring>
#include <deque>
#include <limits>
namespace rr=gpenmpc_hil_endpoint_reader;namespace od=gpenmpc_odometry;
const orb_metadata __orb_gpenmpc_original_hil_receipt{0,sizeof(gpenmpc_original_hil_receipt_s),16};
static unsigned checks{},failed{},actual_parses{},publications{},overwritten{},hrt_reads{};
static void check(bool v,const char*name){++checks;if(!v){++failed;std::fprintf(stderr,"FAIL %s\n",name);}}
static bool same(const void*a,const void*b,std::size_t n){return std::memcmp(a,b,n)==0;}
struct Queued {gpenmpc_original_hil_receipt_s message;uint32_t generation;};
static std::deque<Queued>queue;static uint32_t generation=100;static rr::Px4OriginalHilReceiptReader*reenter{};
static gpenmpc_original_hil_receipt_s last_published{};
bool gpenmpc_test_topic_publish(const orb_metadata*m,const void*p)noexcept{
    ++publications;if(m!=ORB_ID(gpenmpc_original_hil_receipt))return false;
    std::memcpy(&last_published,p,sizeof last_published);
    if(queue.size()==16){queue.pop_front();++overwritten;}
    queue.push_back({last_published,++generation});return true;
}
bool gpenmpc_test_topic_copy(const orb_metadata*,void*,uint32_t&,uint8_t)noexcept{return false;}
bool gpenmpc_test_topic_update(const orb_metadata*m,void*out,uint32_t&g,uint8_t instance)noexcept{
    if(m!=ORB_ID(gpenmpc_original_hil_receipt)||instance||queue.empty())return false;
    if(reenter){rr::Receipt r{};auto*p=reenter;reenter=nullptr;check(!p->lookup(od::Snapshot{},r),"nested reader access unavailable, not blocking");}
    const auto q=queue.front();queue.pop_front();std::memcpy(out,&q.message,sizeof q.message);g=q.generation;return true;
}
extern "C" uint64_t hrt_absolute_time()noexcept{++hrt_reads;return 0;}
static int link,receiver,foreign_link,foreign_receiver,topic;
static constexpr uint64_t original_hrt=UINT64_C(9007199254741993);
static rr::Configuration configuration(){rr::Configuration c{};c.expected_registered_link=&link;c.expected_receiver_instance=2;c.expected_channel=3;
    c.expected_noui_system=255;c.expected_noui_component=0;c.independently_observed_board_identity={0x1122334455667788ULL,42,1,1};return c;}
static od::Configuration source_configuration(){od::Configuration c{};c.identity=configuration().independently_observed_board_identity;c.vehicle_odometry_topic=&topic;
    c.coordinates=od::Coordinates::ExplicitTranslatedNed;c.sample_max_age_us=5000;return c;}
static bool capture(od::AtomicOdometryAdapter&a,uint64_t stamp,uint32_t gen,od::Snapshot&s){vehicle_odometry_s raw{};
    raw.timestamp_sample=stamp;raw.timestamp=stamp+100;raw.pose_frame=raw.POSE_FRAME_NED;raw.velocity_frame=raw.VELOCITY_FRAME_NED;raw.q[0]=1;
    return a.ingest(raw,gen,stamp+200,stamp+300,a.configuration().identity,&topic,0,s);}
static od::Snapshot snapshot(uint64_t stamp){od::AtomicOdometryAdapter a(source_configuration());od::Snapshot s{};check(capture(a,stamp,1,s),"private Snapshot from actual adapter");return s;}
static mavlink_message_t seed{};static mavlink_hil_sensor_t seed_hil{};
static bool actual_packet(bool partial=false){mavlink_message_t sent{},parsed{},rx{};mavlink_status_t status{},reported{};
    mavlink_msg_hil_sensor_pack(255,0,&sent,UINT64_C(18446744073709550000),1.25f,-2.5f,3.75f,4.25f,-5.5f,6.75f,7,8,9,10,11,12,13,partial?0x3fU:0x1fffU,partial?0:7);
    uint8_t wire[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(wire,&sent);unsigned accepted=0;
    for(unsigned i=0;i<n;++i){const auto result=mavlink_frame_char_buffer(&rx,&status,wire[i],&parsed,&reported);if(result==MAVLINK_FRAMING_OK)++accepted;}
    if(accepted!=1||parsed.msgid!=MAVLINK_MSG_ID_HIL_SENSOR)return false;
    seed=parsed;mavlink_msg_hil_sensor_decode(&seed,&seed_hil);++actual_parses;return true;
}
static bool emit(gpenmpc_hil_endpoint::Px4OriginalHilReceipt&p,uint64_t stamp,bool gyro=true,bool accel=true){
    return p.record(seed,seed_hil,stamp,&receiver,&link,2,3,_MAV_PAYLOAD(&seed),gyro,accel);}
static void clean(){queue.clear();generation=100;reenter=nullptr;}
int main(){check(actual_packet(),"real MAVLink pack CRC parse HIL_SENSOR source");
    {clean();rr::Px4OriginalHilReceiptReader reader(configuration());rr::Diagnostics d{};
        for(unsigned i=0;i<2000;++i)check(reader.drain()==rr::Drain::NoUpdate,"no input does not fabricate record or time");
        check(reader.diagnostics(d)&&!d.observed_baseline&&d.unknown_prebaseline&&!d.retained&&!d.successful_lookups,"startup remains no observed receiver");
        rr::Receipt receipt{};check(!reader.lookup(snapshot(original_hrt),receipt)&&!receipt.endpoint.exact_unique_endpoint,"empty startup cannot invent exact receipt");}
    unsigned continuous=0;
    {clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt producer;
        for(unsigned i=0;i<100;++i)check(emit(producer,original_hrt+i*1000),"actual producer pre-reader history");
        rr::Px4OriginalHilReceiptReader reader(configuration());rr::Diagnostics d{};
        check(reader.drain()==rr::Drain::Observed&&reader.diagnostics(d)&&d.accepted_records==16&&d.first_original_subscription_generation==185&&d.first_original_event_sequence==85&&d.unknown_prebaseline,
            "actual first retained sequence/generation form explicit unknown-prehistory baseline, not assumed one");
        rr::Receipt receipt{};check(reader.lookup(snapshot(original_hrt+99000),receipt)&&reader.retire_through(original_hrt+99000),"explicit endpoint retires only accepted prefix");
        od::AtomicOdometryAdapter adapter(source_configuration());
        for(unsigned i=100;i<600;++i){const auto stamp=original_hrt+i*1000;od::Snapshot s{};check(emit(producer,stamp)&&reader.drain()==rr::Drain::Observed,"continuous original receiver observation through bounded queue");
            check(capture(adapter,stamp,i-99,s)&&reader.lookup(s,receipt),"actual private source exact original endpoint match");
            check(receipt.original.topic.timestamp==stamp&&receipt.original.topic.wire_time_usec==seed_hil.time_usec&&receipt.endpoint.original.original_receiver_hrt_us==stamp&&
                same(receipt.endpoint.original.original_sensor52,_MAV_PAYLOAD(&seed)+8,52)&&receipt.original.topic.gyro_update_called&&receipt.original.topic.accel_update_called&&
                !receipt.control_authority&&!receipt.dll_association_proven&&!receipt.receiver_to_estimator_lineage_proven&&!receipt.endpoint.rotor_association_proven,
                "payload52 uint64 original HRT/wire/calls preserved without clock or lineage proof");
            check(reader.retire_through(stamp),"consumed exact endpoint permits bounded reuse without high-water reset");++continuous;}
        check(reader.diagnostics(d)&&d.retained==0&&d.accepted_records==516&&d.retired_records==516&&d.first_original_event_sequence==85,"continuous bounded history counts include pre-endpoint records");}
    for(unsigned kind=0;kind<17;++kind){clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());
        emit(p,original_hrt);reader.drain();emit(p,original_hrt+1000);auto&m=queue.front().message;
        if(kind==0)queue.front().generation++;
        if(kind==1)m.original_event_sequence++;
        if(kind==2)m.receiver_address=reinterpret_cast<uintptr_t>(&foreign_receiver);
        if(kind==3)m.link_address=reinterpret_cast<uintptr_t>(&foreign_link);
        if(kind==4)m.receiver_instance++;
        if(kind==5)m.channel++;
        if(kind==6)m.system_id--;
        if(kind==7)m.component_id++;
        if(kind==8)m.prior_publication_failures=1;
        if(kind==9)m.timestamp=original_hrt;
        if(kind==10)m.timestamp=original_hrt-1;
        if(kind==11)m.wire_time_usec--;
        if(kind==12)m.original_payload[60]^=1;
        if(kind==13)m.sensor_id++;
        if(kind==14)m.payload_length=0;
        if(kind==15){m.payload_length=61;m.original_payload[64]=7;}
        if(kind==16){m.fields_updated=0;m.gyro_update_called=true;}
        check(reader.drain()==rr::Drain::Unavailable&&reader.fault()!=rr::Fault::None,"gap competing owner publication fault malformed or HRT replay rejects");
        rr::Diagnostics d{};rr::Original raw{};check(reader.diagnostics(d)&&d.observed_topic_records==2&&d.accepted_records==1&&reader.audit_copy(0,raw)&&raw.topic.timestamp==original_hrt,"fault retains original accepted history plus rejected raw topic");
        const auto first=reader.fault();check(emit(p,original_hrt+2000)&&reader.drain()==rr::Drain::Unavailable&&reader.fault()==first,"new valid records cannot rebaseline a failed reader");}
    {clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());emit(p,original_hrt,false,true);
        rr::Receipt r{};check(reader.drain()==rr::Drain::Observed&&!reader.lookup(snapshot(original_hrt),r)&&reader.fault()==rr::Fault::ActualGyroNotCalled,"gyro fields cannot fabricate an actual gyro call");}
    {clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());emit(p,original_hrt,true,false);rr::Receipt r{};
        check(reader.drain()==rr::Drain::Observed&&reader.lookup(snapshot(original_hrt),r)&&!r.original.topic.accel_update_called,"actual accel-call false remains false; no invented both-sensor lineage");}
    {clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());emit(p,original_hrt);reader.drain();
        for(unsigned i=1;i<=17;++i)emit(p,original_hrt+i*1000);
        check(reader.drain()==rr::Drain::Unavailable&&reader.fault()==rr::Fault::TopicGap,"actual mock queue16 overwrite creates detected public generation gap");}
    {clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());
        for(unsigned round=0;round<2;++round){for(unsigned i=0;i<16;++i)emit(p,original_hrt+(round*16+i)*1000);check(reader.drain()==rr::Drain::Observed,"exact16 bounded drain");}
        emit(p,original_hrt+32000);check(reader.drain()==rr::Drain::Unavailable&&reader.fault()==rr::Fault::RetentionOverflow,"unconsumed32 capacity cannot evict or overwrite oldest");}
    for(unsigned kind=0;kind<2;++kind){clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());emit(p,original_hrt);
        if(kind==0)queue.front().generation=UINT32_MAX;else queue.front().message.original_event_sequence=UINT64_MAX;
        check(reader.drain()==rr::Drain::Observed,"actual counter maximum can be retained as baseline");emit(p,original_hrt+1000);
        check(reader.drain()==rr::Drain::Unavailable&&reader.fault()==rr::Fault::CounterOverflow,"counter wrap cannot resume original provenance");}
    for(int delta:{-1,1}){clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());emit(p,original_hrt);reader.drain();rr::Receipt r{};
        check(!reader.lookup(snapshot(delta<0?original_hrt-1:original_hrt+1),r)&&reader.fault()==rr::Fault::Lookup,"no nearest endpoint or prebaseline lookup allowed");}
    for(unsigned kind=0;kind<4;++kind){clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());emit(p,original_hrt);reader.drain();
        auto c=source_configuration();if(kind==0)c.identity.uid++;if(kind==1)c.identity.boot_generation++;if(kind==2)c.identity.system++;if(kind==3)c.identity.component++;
        od::AtomicOdometryAdapter adapter(c);od::Snapshot s{};rr::Receipt r{};rr::Diagnostics d{};
        check(capture(adapter,original_hrt,1,s)&&!reader.lookup(s,r)&&reader.diagnostics(d)&&d.lookup_fault==gpenmpc_source_receipt::Fault::SnapshotIdentity,
            "independently valid Snapshot with changed board identity cannot bind original receiver endpoint");}
    {clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());emit(p,original_hrt);reader.drain();rr::Receipt r{};rr::Diagnostics d{};
        check(!reader.lookup(od::Snapshot{},r)&&reader.diagnostics(d)&&d.lookup_fault==gpenmpc_source_receipt::Fault::InvalidSnapshot,"default public Snapshot cannot impersonate private source");}
    {clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());emit(p,original_hrt);reader.drain();rr::Receipt r{};
        check(reader.lookup(snapshot(original_hrt),r)&&reader.retire_through(original_hrt)&&!reader.retire_through(original_hrt),"duplicate retirement cannot clear high-water");}
    {clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());emit(p,original_hrt);reader.drain();rr::Receipt r{};
        check(reader.lookup(snapshot(original_hrt),r)&&reader.retire_through(original_hrt),"retired replay setup");emit(p,original_hrt);
        check(reader.drain()==rr::Drain::Unavailable,"old HRT cannot reenter after actual prefix retirement");}
    {clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());emit(p,original_hrt);reenter=&reader;
        check(reader.drain()==rr::Drain::Unavailable&&reader.fault()==rr::Fault::AccessConflict,"reentrant drain/lookup fails without waiting or claiming atomicity");}
    {clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());emit(p,original_hrt);reader.drain();reader.stop();rr::Receipt r{};
        check(reader.drain()==rr::Drain::Unavailable&&!reader.lookup(snapshot(original_hrt),r)&&reader.fault()==rr::Fault::Stopped,"stop remains permanent and cannot create new baseline");}
    check(actual_packet(true)&&seed.len==61,"actual MAVLink2 legitimate zero-tail short payload");
    {clean();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader reader(configuration());emit(p,original_hrt);rr::Receipt r{};
        check(reader.drain()==rr::Drain::Observed&&reader.lookup(snapshot(original_hrt),r)&&r.original.topic.payload_length==61&&r.original.topic.sensor_id==0,"short original payload remains exact no fake wire65");}
    check(hrt_reads==0&&continuous==500,"reader and producer never sample fresh HRT;500 exact source endpoints");
    std::printf("{\"checks\":%u,\"failed\":%u,\"actual_MAVLink_pack_parse\":%u,\"actual_producer_mock_publications\":%u,\"continuous_exact_private_endpoints\":%u,\"mock_queue_overwrites\":%u,\"observer_HRT_reads\":%u,\"reader_bytes\":%zu,\"unknown_prebaseline\":true,\"full_EKF_lineage_proven\":false,\"DLL_association_proven\":false,\"uORB_HRT_source_MOCK\":true,\"COM\":0}\n",checks,failed,actual_parses,publications,continuous,overwritten,hrt_reads,sizeof(rr::Px4OriginalHilReceiptReader));return failed?1:0;
}
