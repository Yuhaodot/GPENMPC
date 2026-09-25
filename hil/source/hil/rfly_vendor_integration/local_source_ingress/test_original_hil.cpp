#include <common/mavlink.h>
#include "Px4OriginalHilReceipt.hpp"
#include <cstdio>
#include <cstring>
#include <type_traits>

const orb_metadata mock_original_hil_metadata{};
namespace original_hil_mock {
bool next_publish_result=true;unsigned attempts=0;
gpenmpc_original_hil_receipt_s last_attempt{};
}
static unsigned checks=0,failed=0;
static void check(bool value,const char*label){++checks;if(!value){++failed;std::fprintf(stderr,"FAIL %s\n",label);}}
static bool parsed_message(mavlink_message_t&parsed,std::uint64_t wire,std::uint32_t fields,std::uint8_t id,bool all_zero=false){
    mavlink_message_t sent{};mavlink_status_t state{};std::uint8_t buffer[MAVLINK_MAX_PACKET_LEN]{};
    const float v=all_zero?0.0f:1.25f;
    mavlink_msg_hil_sensor_pack(255,0,&sent,wire,v,-v,3*v,4*v,-5*v,6*v,7*v,8*v,9*v,10*v,11*v,12*v,13*v,fields,id);
    const auto n=mavlink_msg_to_send_buffer(buffer,&sent);unsigned accepted=0;
    for(unsigned i=0;i<n;++i)accepted+=mavlink_parse_char(MAVLINK_COMM_1,buffer[i],&parsed,&state)==MAVLINK_FRAMING_OK;
    return accepted==1&&parsed.msgid==MAVLINK_MSG_ID_HIL_SENSOR;
}
int main(){
    using Observer=gpenmpc_hil_endpoint::Px4OriginalHilReceipt;
    static_assert(!std::is_copy_constructible<Observer>::value,"one receiver owns each observer");
    static_assert(!std::is_move_constructible<Observer>::value,"observer cannot move between receivers");
    check(gpenmpc_original_hil_receipt_s::ORB_QUEUE_LENGTH==16,"actual generated engineering queue length 16");
    Observer first,second;int receiver1=0,receiver2=0,link=0;
    const std::uint64_t hrt=9007199254740993ULL,wire=18446744073709550000ULL;
    mavlink_message_t msg{};mavlink_hil_sensor_t hil{};
    check(parsed_message(msg,wire,0x3f,7),"real MAVLink pack serialize parse CRC accepted");
    mavlink_msg_hil_sensor_decode(&msg,&hil);const auto msg_before=msg;const auto hil_before=hil;
    check(first.record(msg,hil,hrt,&receiver1,&link,2,3,_MAV_PAYLOAD(&msg),true,true),"first actual record to mock publisher");
    const auto initial=original_hil_mock::last_attempt;
    check(initial.timestamp==hrt&&initial.wire_time_usec==wire,"separate original uint64 times above 2^53 unchanged");
    check(initial.receiver_address==reinterpret_cast<std::uintptr_t>(&receiver1)&&initial.link_address==reinterpret_cast<std::uintptr_t>(&link)&&initial.receiver_instance==2&&initial.channel==3,"original owner addresses instance channel");
    check(initial.system_id==255&&initial.component_id==0&&initial.mavlink_sequence==msg.seq&&initial.payload_length==msg.len&&initial.sensor_id==7,"original decoded header retained including legitimate component zero");
    check(initial.fields_updated==0x3f&&initial.gyro_update_called&&initial.accel_update_called,"actual called flags not inferred or altered");
    check(initial.gyro_topic_instance==-1&&initial.accel_topic_instance==-1&&initial.gyro_device_id==0&&initial.accel_device_id==0,
        "legacy endpoint-only caller remains explicitly unknown selected source");
    check(std::memcmp(initial.original_payload,_MAV_PAYLOAD(&msg),msg.len)==0,"original payload bytes exact");
    check(std::memcmp(&msg,&msg_before,sizeof msg)==0&&std::memcmp(&hil,&hil_before,sizeof hil)==0,"record does not change decoded sensor or MAVLink message");
    check(first.attempted_after_quiescence()==1&&first.failed_after_quiescence()==0&&!first.overflow_after_quiescence(),"first counters");
    original_hil_mock::next_publish_result=false;
    check(!first.record(msg,hil,hrt+1,&receiver1,&link,2,3,_MAV_PAYLOAD(&msg),false,true),"publication false is returned");
    check(first.attempted_after_quiescence()==2&&first.failed_after_quiescence()==1&&!original_hil_mock::last_attempt.gyro_update_called,"failed attempt counted actual flags preserved");
    original_hil_mock::next_publish_result=true;
    check(first.record(msg,hil,hrt+2,&receiver1,&link,2,3,_MAV_PAYLOAD(&msg),true,false),"later observed publication is not fault recovery authority");
    check(original_hil_mock::last_attempt.original_event_sequence==3&&original_hil_mock::last_attempt.prior_publication_failures==1&&!original_hil_mock::last_attempt.accel_update_called,"sticky prior publication failure travels in next record");
    check(second.record(msg,hil,hrt+3,&receiver2,&link,4,5,_MAV_PAYLOAD(&msg),false,false)&&second.attempted_after_quiescence()==1&&original_hil_mock::last_attempt.original_event_sequence==1,"receiver-local sequences do not share mutable counter");
    check(parsed_message(msg,23,1,0,true),"real short MAVLink2 zero-tail packet parsed");
    mavlink_msg_hil_sensor_decode(&msg,&hil);
    check(msg.len<65,"actual serialized payload is partial rather than synthetic truncation");
    check(first.record(msg,hil,hrt+4,&receiver1,&link,2,3,_MAV_PAYLOAD(&msg),false,true),"partial payload observer record");
    const auto partial=original_hil_mock::last_attempt;
    check(partial.payload_length==msg.len&&partial.sensor_id==0&&partial.fields_updated==1,"partial decoded values original");
    check(std::memcmp(partial.original_payload,_MAV_PAYLOAD(&msg),msg.len)==0,"partial original prefix exact");
    bool zero=true;for(unsigned i=msg.len;i<65;++i)zero&=partial.original_payload[i]==0;
    check(zero,"uncopied payload tail deterministic zero not previous record data");
    check(first.attempted_after_quiescence()==4&&first.failed_after_quiescence()==1,"failures never erase or rewind attempt sequence");
    for(unsigned i=0;i<64;++i){
        check(first.record(msg,hil,hrt+5+i,&receiver1,&link,2,3,_MAV_PAYLOAD(&msg),false,true),"continuous observer publish invocation");
        check(original_hil_mock::last_attempt.original_event_sequence==5+i&&original_hil_mock::last_attempt.prior_publication_failures==1,"continuous original sequence and sticky failures");
    }
    Observer metadata;
    for(int instance=0;instance<4;++instance){
        check(metadata.record(msg,hil,hrt+100+instance,&receiver1,&link,2,3,_MAV_PAYLOAD(&msg),true,true,
            static_cast<std::int16_t>(instance),static_cast<std::int16_t>(3-instance),1310988u,1310988u),
            "actual distinct publisher instances can carry identical simulation device ID");
        const auto &m=original_hil_mock::last_attempt;
        check(m.gyro_topic_instance==instance&&m.accel_topic_instance==3-instance&&m.gyro_device_id==1310988u&&m.accel_device_id==1310988u,
            "publisher instance and device IDs retained independently without inferring selection");
        check(m.timestamp==hrt+100+instance&&std::memcmp(m.original_payload,partial.original_payload,65)==0,
            "publisher metadata does not replace original HRT or sensor payload");
    }
    check(metadata.record(msg,hil,hrt+110,&receiver1,&link,2,3,_MAV_PAYLOAD(&msg),false,false,2,3,1310988u,1310988u),
        "uncalled sensor record remains observable");
    check(original_hil_mock::last_attempt.gyro_topic_instance==-1&&original_hil_mock::last_attempt.accel_topic_instance==-1&&
          original_hil_mock::last_attempt.gyro_device_id==0&&original_hil_mock::last_attempt.accel_device_id==0,
        "no current update cannot inherit a previous publisher identity");
    std::printf("{\"checks\":%u,\"failed\":%u,\"actual_MAVLink_pack_parse\":true,\"actual_generated_uORB_schema\":true,\"uORB_mock\":true,\"control_authority\":false,\"sensor_calls_by_test\":0,\"hrt_clock_reads_by_observer\":0,\"partial_payload_length\":%u,\"attempts_first\":%llu,\"prior_publish_failures\":%llu,\"COM\":0,\"board\":0}\n",checks,failed,unsigned(msg.len),static_cast<unsigned long long>(first.attempted_after_quiescence()),static_cast<unsigned long long>(first.failed_after_quiescence()));
    return failed?1:0;
}
