#include "RflyHilPacket.hpp"
#include <cstdio>
#include <cstring>
#include <limits>

static unsigned checks=0,failures=0;
static void check(bool ok,const char *name){++checks;if(!ok){++failures;std::fprintf(stderr,"FAIL %s\n",name);}}
int main(){
    using namespace gpenmpc_rfly_packet;
    actuator_outputs_s raw{};
    raw.timestamp=UINT64_C(9007199254741111); // Do not transit through double.
    raw.noutputs=0; // Actual official writer behavior.
    for(unsigned i=0;i<6;++i)raw.output[i]=float(i)/5.0f;
    vehicle_status_s status{};
    status.hil_state=vehicle_status_s::HIL_STATE_ON;
    status.arming_state=vehicle_status_s::ARMING_STATE_ARMED;
    status.nav_state=vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
    vehicle_control_mode_s mode{};
    mode.flag_control_auto_enabled=true;mode.flag_control_attitude_enabled=true;
    auto encode_now=[&](){return encode(raw,status,mode,true,raw.timestamp+50,raw.timestamp+100);};
    auto good=encode_now();
    check(good.accepted&&good.raw_noutputs==0,"official fixed16 with raw noutputs0");
    check(good.packet.flags==123,"vendor normalized flag");
    check(good.packet.time_usec==raw.timestamp,"original uint64 source time");
    check((good.packet.mode&MAV_MODE_FLAG_SAFETY_ARMED)!=0,"observed armed bit not a command");
    for(unsigned i=0;i<16;++i)check(std::memcmp(&raw.output[i],&good.packet.controls[i],4)==0,"no float remap");
    mavlink_message_t message{},decoded{};
    mavlink_msg_hil_actuator_controls_encode(1,1,&message,&good.packet);
    std::uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
    const auto n=mavlink_msg_to_send_buffer(bytes,&message);
    mavlink_status_t parse_status{};unsigned decoded_count=0;
    for(unsigned i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,bytes[i],&decoded,&parse_status))++decoded_count;
    check(decoded_count==1&&decoded.msgid==MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS,"actual generated MAVLink encode/parser");
    mavlink_hil_actuator_controls_t unpacked{};
    mavlink_msg_hil_actuator_controls_decode(&decoded,&unpacked);
    check(unpacked.time_usec==raw.timestamp&&unpacked.flags==123&&unpacked.mode==good.packet.mode,"roundtrip metadata");
    check(std::memcmp(unpacked.controls,raw.output,sizeof(raw.output))==0,"roundtrip16 byte exact");
    check(encode(raw,status,mode,false,raw.timestamp+50,raw.timestamp+100).error==Error::NotSelected,"cannot self-select");
    check(encode(raw,status,mode,true,raw.timestamp-1,raw.timestamp+100).error==Error::InvalidTime,"future timestamp");
    check(encode(raw,status,mode,true,raw.timestamp+101,raw.timestamp+100).error==Error::Expired,"original expiry");
    check(encode(raw,status,mode,true,raw.timestamp+100,raw.timestamp+100).accepted,"expiry equality");
    status.hil_state=vehicle_status_s::HIL_STATE_OFF;
    check(encode_now().error==Error::NotHil,"no non-HIL output");
    status.hil_state=vehicle_status_s::HIL_STATE_ON;
    status.arming_state=vehicle_status_s::ARMING_STATE_DISARMED;
    check((encode_now().packet.mode&MAV_MODE_FLAG_SAFETY_ARMED)==0,"disarmed status clears MAV_MODE_FLAG_SAFETY_ARMED");
    for(unsigned i=0;i<16;++i){
        const float save=raw.output[i];
        raw.output[i]=std::numeric_limits<float>::quiet_NaN();
        auto r=encode_now();check(r.error==Error::Nonfinite,"NaN reject");
        check(r.packet.flags==0&&r.packet.time_usec==0,"rejected packet not transmissible");
        raw.output[i]=std::numeric_limits<float>::infinity();check(encode_now().error==Error::Nonfinite,"Inf reject");
        raw.output[i]=1.001f;check(encode_now().error==Error::Range,"over one not clipped");
        raw.output[i]=-0.001f;check(encode_now().error==Error::Range,"negative thrust not clipped");
        if(i>=6){raw.output[i]=0.2f;check(encode_now().error==Error::UnexpectedUnusedControl,"spare output forbidden");}
        raw.output[i]=save;
    }
    std::printf("{\"status\":\"%s\",\"checks\":%u,\"failures\":%u,\"actual_generated_mavlink_packet\":1,\"raw_noutputs\":0,\"controls\":16,\"wire_bytes\":%u,\"socket_open\":0,\"uorb_broker_run\":false,\"stream_installed\":false}\n",failures?"FAIL":"PASS_HOST_PACKET_BOUNDARY",checks,failures,unsigned(n));
    return failures?1:0;
}
