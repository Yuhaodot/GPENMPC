#include "Px4SelectedSourceReader.hpp"
#include "Px4OriginalHilReceipt.hpp"
#include <common/mavlink.h>
#include <parameters/param.h>
#undef param_get
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>
#include <limits>
namespace ss=gpenmpc_selected_source;namespace rr=gpenmpc_hil_endpoint_reader;namespace od=gpenmpc_odometry;
#define META(n,id,q) const orb_metadata __orb_##n{id,sizeof(n##_s),q}
META(gpenmpc_original_hil_receipt,0,16);META(sensor_gyro,1,8);META(sensor_accel,2,8);META(vehicle_imu,3,1);
META(sensor_selection,4,1);META(sensors_status_imu,5,1);META(sensor_combined,6,1);META(vehicle_odometry,7,1);
static unsigned checks{},failed{},packets{},param_reads{},param_writes{},hrt_reads{},queue_losses{};
static void check(bool v,const char*n){++checks;if(!v){++failed;std::fprintf(stderr,"FAIL %s\n",n);}}
struct Q {std::vector<unsigned char>bytes;uint32_t generation;};
struct Topic {std::deque<Q>q;uint32_t generation=100;bool advertised{};} topics[8][ORB_MULTI_MAX_INSTANCES];
static int32_t actual_params[3]={1,0,0};static int missing_param=-1,bad_type=-1,bad_read=-1;
extern "C" param_t param_find_no_notification(const char*n){const char*names[]={"SENS_IMU_MODE","EKF2_MULTI_IMU","EKF2_MULTI_MAG"};for(unsigned i=0;i<3;++i)if(!std::strcmp(n,names[i]))return static_cast<int>(i)==missing_param?PARAM_INVALID:static_cast<param_t>(i);return PARAM_INVALID;}
extern "C" param_type_t param_type(param_t p){return static_cast<int>(p)==bad_type?PARAM_TYPE_FLOAT:PARAM_TYPE_INT32;}
extern "C" int param_get(param_t p,void*out){++param_reads;if(static_cast<int>(p)==bad_read)return -1;if(p>=3)return -1;std::memcpy(out,&actual_params[p],4);return 0;}
extern "C" int param_set(param_t,const void*){++param_writes;return -1;}
extern "C" uint64_t hrt_absolute_time()noexcept{++hrt_reads;return 0;}
bool gpenmpc_test_topic_advertised(const orb_metadata*m,uint8_t i)noexcept{return m&&topics[m->id][i].advertised;}
bool gpenmpc_test_topic_updated(const orb_metadata*m,uint32_t g,uint8_t i)noexcept{return m&&!topics[m->id][i].q.empty()&&topics[m->id][i].q.back().generation>g;}
bool gpenmpc_test_topic_copy(const orb_metadata*,void*,uint32_t&,uint8_t)noexcept{return false;}
bool gpenmpc_test_topic_update(const orb_metadata*m,void*out,uint32_t&g,uint8_t i)noexcept{
    if(!m)return false;
    auto&q=topics[m->id][i].q;
    for(const auto&r:q){if(r.generation>g){std::memcpy(out,r.bytes.data(),m->size);g=r.generation;return true;}}
    return false;
}
static void emit(const orb_metadata*m,const void*p,uint8_t i=0){auto&t=topics[m->id][i];t.advertised=true;
    if(t.q.size()==m->queue){t.q.pop_front();++queue_losses;}const auto*b=static_cast<const unsigned char*>(p);t.q.push_back({std::vector<unsigned char>(b,b+m->size),++t.generation});}
bool gpenmpc_test_topic_publish(const orb_metadata*m,const void*p)noexcept{emit(m,p);return true;}
static void clear(){for(auto&group:topics)for(auto&t:group)t=Topic{};actual_params[0]=1;actual_params[1]=actual_params[2]=0;missing_param=bad_type=bad_read=-1;}
static int link,receiver,foreign_topic;
static constexpr uint64_t T=UINT64_C(9007199254741993);static constexpr uint32_t ID=1310988;
static rr::Configuration configuration(){rr::Configuration c{};c.expected_registered_link=&link;c.expected_receiver_instance=2;c.expected_channel=3;c.expected_noui_system=255;c.expected_noui_component=0;c.independently_observed_board_identity={0x1122334455667788ULL,42,1,1};return c;}
static od::Configuration source_config(uint8_t i=0,const void*topic=ORB_ID(vehicle_odometry)){od::Configuration c{};c.identity=configuration().independently_observed_board_identity;c.vehicle_odometry_topic=topic;c.instance=i;c.coordinates=od::Coordinates::ExplicitTranslatedNed;c.sample_max_age_us=5000;return c;}
static od::Snapshot snapshot(uint64_t t,uint8_t i=0,const void*topic=ORB_ID(vehicle_odometry)){
    od::AtomicOdometryAdapter a(source_config(i,topic));vehicle_odometry_s v{};v.timestamp_sample=t;v.timestamp=t+800;v.q[0]=1;v.pose_frame=v.POSE_FRAME_NED;v.velocity_frame=v.VELOCITY_FRAME_NED;od::Snapshot s{};
    check(a.ingest(v,101,t+900,t+1000,a.configuration().identity,topic,i,s),"private Snapshot actual factory");
    check(s.source_topic()==topic&&s.source_instance()==i,"Snapshot exact original topic/instance getters");return s;
}
static void original(gpenmpc_hil_endpoint::Px4OriginalHilReceipt&p,uint64_t t,int gi=0,int ai=0,uint32_t gd=ID,uint32_t ad=ID){
    mavlink_message_t sent{},parsed{},rx{};mavlink_status_t status{},reported{};
    mavlink_msg_hil_sensor_pack(255,0,&sent,123456,1,2,3,4,5,6,7,8,9,10,11,12,13,8191,224);
    uint8_t wire[MAVLINK_MAX_PACKET_LEN]{};auto n=mavlink_msg_to_send_buffer(wire,&sent);unsigned accepted=0;
    for(unsigned i=0;i<n;++i)if(mavlink_frame_char_buffer(&rx,&status,wire[i],&parsed,&reported)==MAVLINK_FRAMING_OK)++accepted;
    check(accepted==1,"actual MAVLink pack wire CRC parse");++packets;mavlink_hil_sensor_t hil{};mavlink_msg_hil_sensor_decode(&parsed,&hil);
    check(p.record(parsed,hil,t,&receiver,&link,2,3,_MAV_PAYLOAD(&parsed),true,true,static_cast<int16_t>(gi),static_cast<int16_t>(ai),gd,ad),"actual observer with explicit mocked post-update getter values");
}
struct Chain {sensor_gyro_s g{};sensor_accel_s a{};vehicle_imu_s v{};sensor_selection_s select{};sensors_status_imu_s status{};sensor_combined_s c{};};
static Chain chain(uint64_t t){Chain x{};x.g.timestamp=t+10;x.g.timestamp_sample=t;x.g.device_id=ID;x.g.x=4;x.g.y=5;x.g.z=6;x.g.samples=1;
    x.a.timestamp=t+20;x.a.timestamp_sample=t;x.a.device_id=ID;x.a.x=1;x.a.y=2;x.a.z=3;x.a.samples=1;
    x.v.timestamp=t+100;x.v.timestamp_sample=t;x.v.accel_device_id=x.v.gyro_device_id=ID;x.v.delta_angle_dt=x.v.delta_velocity_dt=5000;
    for(unsigned i=0;i<3;++i){x.v.delta_angle[i]=0.007f*static_cast<float>(i+1);x.v.delta_velocity[i]=0.011f*static_cast<float>(i+1);}
    x.select.timestamp=t+200;x.select.accel_device_id=x.select.gyro_device_id=ID;
    x.status.timestamp=t+300;x.status.accel_device_id_primary=x.status.gyro_device_id_primary=ID;x.status.accel_device_ids[0]=x.status.gyro_device_ids[0]=ID;
    x.status.accel_healthy[0]=x.status.gyro_healthy[0]=true;x.status.accel_priority[0]=x.status.gyro_priority[0]=50;
    x.c.timestamp=t;x.c.accelerometer_integral_dt=x.v.delta_velocity_dt;x.c.gyro_integral_dt=x.v.delta_angle_dt;
    for(unsigned i=0;i<3;++i){x.c.gyro_rad[i]=x.v.delta_angle[i]*(1.e6f/static_cast<float>(x.v.delta_angle_dt));x.c.accelerometer_m_s2[i]=x.v.delta_velocity[i]*(1.e6f/static_cast<float>(x.v.delta_velocity_dt));}return x;
}
static void publish(const Chain&x,int omit=-1){if(omit!=1)emit(ORB_ID(sensor_gyro),&x.g);if(omit!=2)emit(ORB_ID(sensor_accel),&x.a);if(omit!=3)emit(ORB_ID(vehicle_imu),&x.v);
    if(omit!=4)emit(ORB_ID(sensor_selection),&x.select);
    if(omit!=5)emit(ORB_ID(sensors_status_imu),&x.status);
    if(omit!=6)emit(ORB_ID(sensor_combined),&x.c);}
int main(){
    {clear();rr::Px4OriginalHilReceiptReader original_reader(configuration());ss::Px4SelectedSourceReader reader(original_reader,configuration().independently_observed_board_identity);ss::Receipt out{};
        for(unsigned i=0;i<32;++i)check(reader.drain()==ss::Result::Missing&&reader.lookup(snapshot(T),out)==ss::Result::Missing&&!out.unique_selected_endpoint_observed,"missing data leaves the selected endpoint unavailable");}
    {clear();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);ss::Receipt out{};
        for(unsigned i=0;i<60;++i){const auto t=T+static_cast<uint64_t>(i)*9000;original(p,t);publish(chain(t));check(r.drain()==rr::Drain::Observed,"original reader actual producer endpoint drain");reader.drain();
            check(reader.lookup(snapshot(t),out)==ss::Result::Matched,"exact selected chain matched");
            check(out.snapshot_sample_us==t&&out.original.original.topic.timestamp==t&&out.gyro.value.timestamp_sample==t&&out.imu.value.timestamp_sample==t&&out.combined.value.timestamp==t&&out.gyro.generation==101+i,"original uint64 endpoint/generation preserved above flintmax");
            check(out.unique_selected_endpoint_observed&&!out.full_integration_history_proven&&!out.full_ekf_history_proven&&!out.dll_association_proven&&!out.control_authority,"scope stays selection endpoint, not full EKF or authority");
            check(reader.retire_through(t)&&r.retire_through(t),"bounded independent retirement not identity reset");}
        check(reader.diagnostics().matched==60&&reader.diagnostics().generation_gaps==0,"60 measured endpoint joins without invented intermediate data");}
    for(unsigned which=0;which<9;++which){clear();if(which<3)missing_param=static_cast<int>(which);else if(which<6)bad_type=static_cast<int>(which-3);else bad_read=static_cast<int>(which-6);
        rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);check(reader.drain()==ss::Result::Unsupported,"actual missing/type/read param failure refuses");}
    for(unsigned which=0;which<3;++which){clear();actual_params[which]=which?1:0;rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);check(reader.drain()==ss::Result::Unsupported,"no coercion of unsupported actual multi config");}
    for(int omit=1;omit<=6;++omit){clear();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);ss::Receipt out{};original(p,T);r.drain();publish(chain(T),omit);reader.drain();
        check(reader.lookup(snapshot(T),out)==ss::Result::Missing&&!out.unique_selected_endpoint_observed,"missing exact stage returns Missing not inferred lineage");}
    for(unsigned which=0;which<19;++which){clear();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);ss::Receipt out{};
        original(p,T,which==0?-1:0,which==1?1:0,which==2?0:ID);r.drain();auto x=chain(T);
        if(which==3)x.g.device_id++;
        if(which==4)x.a.device_id++;
        if(which==5)x.v.gyro_device_id++;
        if(which==6)x.select.gyro_device_id++;
        if(which==7)x.status.accel_device_id_primary++;
        if(which==8)x.status.gyro_device_ids[1]=ID;
        if(which==9)x.status.gyro_healthy[0]=false;
        if(which==10)x.c.gyro_rad[0]+=0.01f;
        if(which==11)x.c.accelerometer_timestamp_relative=-1;
        if(which==12)x.c.gyro_integral_dt++;
        if(which==13)x.c.accel_calibration_count++;
        if(which==14)x.g.x=std::numeric_limits<float>::quiet_NaN();
        if(which==15)x.v.delta_angle[0]=std::numeric_limits<float>::infinity();
        if(which==16)x.c.gyro_rad[0]=std::numeric_limits<float>::quiet_NaN();
        publish(x);if(which==17)emit(ORB_ID(sensor_gyro),&x.g,1);if(which==18)topics[1][1].advertised=true;reader.drain();
        const auto result=reader.lookup(snapshot(T),out);
        if(which==9)check(result==ss::Result::Matched&&!out.status.value.gyro_healthy[0],"actual selected unhealthy flag retained without new health threshold");
        else check(result==(which==18?ss::Result::Missing:ss::Result::Invalid)&&!out.unique_selected_endpoint_observed,"wrong source/mapping/selection/float bytes cannot match");}
    for(unsigned which=0;which<3;++which){clear();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);ss::Receipt out{};original(p,T);r.drain();publish(chain(T));reader.drain();
        const auto s=which==0?od::Snapshot{}:(which==1?snapshot(T,1):snapshot(T,0,&foreign_topic));check(reader.lookup(s,out)==ss::Result::Invalid,"default/other actual topic/instance source rejects");}
    for(unsigned id=1;id<=6;++id){clear();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);original(p,T);r.drain();publish(chain(T));reader.drain();
        auto x=chain(T+9000);topics[id][0].generation++;publish(x);check(reader.drain()==ss::Result::Missing&&reader.diagnostics().first_reason==ss::Reason::IncompleteHistory,"known generation gap is sticky Missing history, not nearest endpoint");check(!reader.retire_through(T),"retire cannot erase missing history");}
    for(unsigned id=1;id<=6;++id){clear();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);original(p,T);r.drain();publish(chain(T));reader.drain();
        auto x=chain(T+9000);if(id==1)x.g.timestamp_sample=T;if(id==2)x.a.timestamp_sample=T;if(id==3)x.v.timestamp_sample=T;if(id==4)x.select.timestamp=T+200;if(id==5)x.status.timestamp=T+300;if(id==6)x.c.timestamp=T;
        publish(x);check(reader.drain()==ss::Result::Invalid&&reader.diagnostics().first_reason==ss::Reason::Regression,"duplicate original sample or metadata timestamp rejects without renewing age");}
    {clear();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);ss::Receipt out{};
        for(unsigned i=0;i<30;++i){const auto t=T+static_cast<uint64_t>(i)*10000;original(p,t);publish(chain(t),i?4:-1);r.drain();reader.drain();
            check(reader.lookup(snapshot(t),out)==ss::Result::Matched&&out.selection.value.timestamp==T+200&&out.selection.generation==101,"stock change-only sensor_selection does not acquire artificial fresh HRT or a freshness gate");
            check(reader.retire_through(t)&&r.retire_through(t),"metadata baseline survives bounded retirement");}
        actual_params[0]=0;check(reader.lookup(snapshot(T+300000),out)==ss::Result::Unsupported&&!out.unique_selected_endpoint_observed,"runtime parameter change refuses after previous successful endpoint");}
    {clear();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);ss::Receipt out{};
        original(p,T);publish(chain(T));r.drain();reader.drain();
        // Real stock queue 1 behavior: two distinct publications occur before
        // this reader drains again.
        auto a=chain(T+5000),b=chain(T+10000);emit(ORB_ID(vehicle_imu),&a.v);emit(ORB_ID(vehicle_imu),&b.v);
        check(reader.drain()==ss::Result::Missing&&reader.diagnostics().generation_gaps==1,"actual queue1 overwrite yields Missing history");
        check(reader.lookup(snapshot(T),out)==ss::Result::Missing&&!out.unique_selected_endpoint_observed,"an earlier matching endpoint cannot clear missing-history latch");}
    {clear();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);ss::Receipt out{};
        original(p,T);auto x=chain(T);x.status.timestamp=T+1001;publish(x);r.drain();reader.drain();
        check(reader.lookup(snapshot(T),out)==ss::Result::Missing,"future status outside original estimator publication bracket cannot support current source");}
    {clear();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);ss::Receipt out{};
        sensors_status_imu_s startup{};startup.timestamp=T-100;emit(ORB_ID(sensors_status_imu),&startup);reader.drain();
        check(reader.diagnostics().first_reason==ss::Reason::None,"timestamped zero-selection startup remains non-fatal");
        original(p,T);publish(chain(T));r.drain();reader.drain();check(reader.lookup(snapshot(T),out)==ss::Result::Matched,"startup status followed by actual selected endpoint can match");}
    for(unsigned changed=0;changed<2;++changed){clear();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);ss::Receipt out{};
        original(p,T);auto x=chain(T);publish(x);r.drain();reader.drain();
        if(changed==0){x.select.timestamp=T+600;x.select.gyro_device_id++;emit(ORB_ID(sensor_selection),&x.select);}
        else{x.status.timestamp=T+600;x.status.gyro_device_ids[1]=ID;emit(ORB_ID(sensors_status_imu),&x.status);}
        reader.drain();check(reader.lookup(snapshot(T),out)==ss::Result::Invalid&&!out.unique_selected_endpoint_observed,"later conflicting metadata inside estimator bracket cannot be cherry-picked away");}
    {clear();gpenmpc_hil_endpoint::Px4OriginalHilReceipt p;rr::Px4OriginalHilReceiptReader r(configuration());ss::Px4SelectedSourceReader reader(r,configuration().independently_observed_board_identity);for(unsigned i=0;i<9;++i){publish(chain(T+i*9000));reader.drain();}check(reader.diagnostics().first_reason==ss::Reason::RetentionFull,"unconsumed retention never silently evicts oldest");}
    check(!param_writes&&!hrt_reads,"zero param writes and zero HRT sampling");
    std::printf("{\"checks\":%u,\"failed\":%u,\"actual_MAVLink_packets\":%u,\"actual_private_endpoints\":60,\"param_get_calls_mock_actual_API\":%u,\"reader_bytes\":%zu,\"queue1_and_queue8_overwrites\":%u,\"params_uORB_HRT_MOCK\":true,\"full_EKF_history_proven\":false,\"COM\":0,\"board\":0}\n",checks,failed,packets,param_reads,sizeof(ss::Px4SelectedSourceReader),queue_losses);return failed?1:0;
}
