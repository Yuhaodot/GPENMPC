#include "GPENMPCFullInnerIngress.hpp"
#include <uORB/topics/gpenmpc_full_inner_ingress.h>
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace gpenmpc_ingress;
static unsigned checks{}, failures{};
struct CheckResult{bool passed;const char *name;};
static std::vector<CheckResult> results;
static void check(bool ok,const char *name){++checks;results.push_back({ok,name});if(!ok){++failures;std::printf("FAIL %s\n",name);}}
struct Parsed{mavlink_message_t frame{};unsigned accepted{},bad_crc{},bad_signature{};};
static Parsed parse(const std::vector<std::uint8_t> &bytes){
    mavlink_message_t buffer{};mavlink_status_t state{},reported{};Parsed out{};
    for(auto byte:bytes){const auto r=mavlink_frame_char_buffer(&buffer,&state,byte,&out.frame,&reported);
        out.accepted+=r==MAVLINK_FRAMING_OK;out.bad_crc+=r==MAVLINK_FRAMING_BAD_CRC;out.bad_signature+=r==MAVLINK_FRAMING_BAD_SIGNATURE;}
    return out;
}
static std::vector<std::uint8_t> bytes(const mavlink_message_t &frame){
    std::array<std::uint8_t,MAVLINK_MAX_PACKET_LEN> out{};
    const auto count=mavlink_msg_to_send_buffer(out.data(),&frame);
    return {out.begin(),out.begin()+count};
}
static mavlink_message_t tunnel(std::uint8_t length=128,std::uint16_t type=payload_type,
    std::uint8_t target_system=1,std::uint8_t target_component=1,std::uint8_t source_system=42,std::uint8_t source_component=191,
    bool zeros=false){
    std::array<std::uint8_t,128> body{};
    if(!zeros)for(std::size_t i=0;i<body.size();++i)body[i]=static_cast<std::uint8_t>((i*37+19)&255);
    mavlink_message_t out{};mavlink_status_t state{};
    mavlink_msg_tunnel_pack_status(source_system,source_component,&state,&out,target_system,target_component,type,length,body.data());
    return out;
}
static Result receive_wire(Receiver &receiver,const mavlink_message_t &source,std::uint64_t time=123456789){
    const auto decoded=parse(bytes(source));check(decoded.accepted==1&&decoded.bad_crc==0,"real generated parser accepts packed frame");
    return receiver.receive(decoded.frame,time,1,1,3);
}
// Actual PX4-generated topic type; this still does not run the uORB broker.
int main(int argc,char **argv){
    static_assert(gpenmpc_full_inner_ingress_s::ORB_QUEUE_LENGTH == queue_capacity,"actual uORB queue constant");
    static_assert(sizeof(gpenmpc_full_inner_ingress_s::payload) == payload_capacity,"actual uORB payload bound");
    Receiver receiver;
    const auto original=tunnel();const auto serialized=bytes(original);
    check(serialized[0]==MAVLINK_STX&&serialized[1]==133,"MAVLink2 wire header length");
    check(serialized[5]==42&&serialized[6]==191,"real wire source IDs");
    check(serialized[7]==0x81&&serialized[8]==1&&serialized[9]==0,"real wire TUNNEL message ID 385");
    check(serialized[10]==static_cast<std::uint8_t>(payload_type)&&serialized[11]==static_cast<std::uint8_t>(payload_type>>8),"real wire payload type little endian");
    check(serialized[12]==1&&serialized[13]==1&&serialized[14]==128,"real wire target and meaningful length");
    check(receive_wire(receiver,original)==Result::Accepted,"128-byte ingress accepted");
    const auto first=*receiver.front();
    check(first.timestamp==123456789&&first.source_system==42&&first.source_component==191&&first.receiver_instance==3,"source and original HRT preserved");
    check(first.payload[0]==19&&first.payload[127]==static_cast<std::uint8_t>((127*37+19)&255),"128-byte content preserved");
    gpenmpc_full_inner_ingress_s topic{};copy_to_topic(first,receiver.counters(),topic);
    check(topic.timestamp==first.timestamp&&topic.reception_sequence==first.reception_sequence&&topic.receiver_instance==3
        &&topic.source_system==42&&topic.source_component==191&&topic.target_system==1&&topic.target_component==1
        &&topic.mavlink_sequence==first.mavlink_sequence&&topic.wire_payload_length==133&&topic.payload_type==payload_type
        &&topic.payload_length==128&&std::equal(first.payload,first.payload+payload_capacity,topic.payload),"all topic fields copied");
    check(!receiver.drain(123456790,[](const Fields &){return false;})&&receiver.pending()==1&&receiver.counters().publication_failures==1,"failed publication retains exact queue item");
    check(receiver.front()->timestamp==first.timestamp&&receiver.front()->reception_sequence==first.reception_sequence,"retry does not retimestamp or renumber");
    check(receiver.drain(123456800,[&](const Fields &in){copy_to_topic(in,receiver.counters(),topic);return true;})&&receiver.pending()==0&&receiver.counters().published==1,"successful publication commits once");
    check(topic.ingress_first_fault==static_cast<std::uint8_t>(Result::PublicationFailure)&&topic.ingress_first_fault_hrt==123456790
        &&topic.ingress_publication_failures==1&&topic.timestamp==123456789,"successful retry atomically retains latched publication-failure evidence and original receive HRT");
    check(receiver.drain(123456801,[](const Fields &){return false;})&&receiver.counters().publication_failures==1,"empty drain invokes no publisher");
    for(unsigned len=1;len<=128;++len){
        Receiver x;check(receive_wire(x,tunnel(static_cast<std::uint8_t>(len)))==Result::Accepted,"all meaningful lengths 1..128 accepted");
        bool equal=x.front()->payload_length==len;
        for(std::size_t i=0;i<128;++i)equal&=x.front()->payload[i]==(i<len?static_cast<std::uint8_t>((i*37+19)&255):0);
        check(equal,"used bytes preserved and unused bytes sanitized");
    }
    Receiver zero;const auto trimmed=tunnel(128,payload_type,1,1,42,191,true);
    check(trimmed.len==5,"generated MAVLink2 trims all-zero payload to header");
    check(receive_wire(zero,trimmed)==Result::Accepted&&zero.front()->payload[127]==0,"zero-trimmed wire restores 128 zeros");
    struct Negative{mavlink_message_t frame;Result expected;const char *name;};
    const Negative negatives[]={
        {tunnel(0),Result::BadPayloadLength,"zero application length"},
        {tunnel(129),Result::BadPayloadLength,"length129 rejected"},
        {tunnel(255),Result::BadPayloadLength,"length255 rejected"},
        {tunnel(128,42001),Result::OtherPayload,"legacy RTA1 stays separate"},
        {tunnel(128,0),Result::OtherPayload,"unknown type stays outside new ingress"},
        {tunnel(128,65535),Result::OtherPayload,"other experimental type stays outside"},
        {tunnel(128,payload_type,2,1),Result::WrongTarget,"wrong target system"},
        {tunnel(128,payload_type,1,2),Result::WrongTarget,"wrong target component"},
        {tunnel(128,payload_type,0,1),Result::WrongTarget,"system broadcast refused only for new channel"},
        {tunnel(128,payload_type,1,0),Result::WrongTarget,"component broadcast refused only for new channel"},
        {tunnel(128,payload_type,1,1,0,191),Result::BadSource,"zero source system"},
        {tunnel(128,payload_type,1,1,42,0),Result::BadSource,"zero source component"}
    };
    for(const auto &negative:negatives){Receiver x;check(receive_wire(x,negative.frame)==negative.expected,negative.name);check(x.pending()==0,"rejected frame creates no queue item");}
    check(belongs_to_channel(original)&&!belongs_to_channel(tunnel(128,42001))&&!belongs_to_channel(tunnel(128,0)),"new dispatch preserves legacy and unknown payload channels");
    Receiver alt;check(receive_wire(alt,tunnel(1,payload_type,1,1,254,255))==Result::Accepted&&alt.front()->source_system==254&&alt.front()->source_component==255,"nonzero IDs are preserved routing labels, not authentication");
    Receiver clock;check(receive_wire(clock,tunnel(),0)==Result::BadTimestamp,"zero HRT rejected");
    const auto decoded=parse(bytes(original));Receiver local;
    check(local.receive(decoded.frame,10,0,1,0)==Result::BadLocalRoute,"local system must be exact nonzero ID");
    check(local.receive(decoded.frame,10,1,0,0)==Result::BadLocalRoute,"local component must be exact nonzero ID");
    mavlink_message_t heartbeat{};mavlink_msg_heartbeat_pack(42,191,&heartbeat,MAV_TYPE_GENERIC,MAV_AUTOPILOT_GENERIC,0,0,MAV_STATE_STANDBY);
    check(receive_wire(local,heartbeat)==Result::NotTunnel,"other real MAVLink message rejected from ingress");
    auto corrupt=serialized;corrupt[15]^=0x20;const auto corrupt_result=parse(corrupt);
    check(corrupt_result.accepted==0&&corrupt_result.bad_crc==1,"real generated parser rejects corrupt CRC frame before helper");
    auto truncated=serialized;truncated.pop_back();check(parse(truncated).accepted==0,"real parser refuses incomplete wire frame");
    auto short_frame=original;short_frame.len=4;
    mavlink_status_t short_status{};
    mavlink_finalize_message_buffer(&short_frame,42,191,&short_status,4,4,MAVLINK_MSG_ID_TUNNEL_CRC);
    const auto short_decoded=parse(bytes(short_frame));
    check(short_decoded.accepted==0 || local.receive(short_decoded.frame,10,1,1,0)==Result::BadWireLength,"short CRC-valid frame cannot enter ingress");
    auto long_frame=original;reinterpret_cast<std::uint8_t*>(long_frame.payload64)[133]=17;
    mavlink_status_t long_status{};
    mavlink_finalize_message_buffer(&long_frame,42,191,&long_status,134,134,MAVLINK_MSG_ID_TUNNEL_CRC);
    const auto long_decoded=parse(bytes(long_frame));
    check(long_decoded.accepted==0 || local.receive(long_decoded.frame,10,1,1,0)==Result::BadWireLength,"oversize CRC-valid frame cannot enter ingress");
    // Defensive malformed in-memory inputs supplement (not replace) real-wire tests.
    auto wrong_version=decoded.frame;wrong_version.magic=MAVLINK_STX_MAVLINK1;
    check(local.receive(wrong_version,10,1,1,0)==Result::BadVersion,"invalid in-memory MAVLink version refused");
    auto no_type=decoded.frame;no_type.len=1;
    check(local.receive(no_type,10,1,1,0)==Result::BadWireLength,"missing payload type refused");
    check(!belongs_to_channel(no_type),"unidentifiable short legacy frame remains on original dispatcher path");
    Receiver queue;
    for(unsigned i=0;i<queue_capacity;++i)check(receive_wire(queue,tunnel(),100+i)==Result::Accepted,"bounded FIFO accepts available slot");
    check(receive_wire(queue,tunnel(),999)==Result::QueueFull&&queue.pending()==queue_capacity,"full FIFO rejects without overwriting");
    check(queue.front()->timestamp==100&&queue.front()->reception_sequence==1,"queue overflow retains oldest packet");
    unsigned published=0;bool order=true;
    check(queue.drain(1000,[&](const Fields &item){copy_to_topic(item,queue.counters(),topic);order&=item.timestamp==100+published&&item.reception_sequence==1+published;++published;return true;})&&order&&published==queue_capacity,"real helper FIFO drains ordered fields");
    check(topic.ingress_first_fault==static_cast<std::uint8_t>(Result::QueueFull)&&topic.ingress_queue_overflows==1
        &&topic.ingress_first_fault_hrt==999&&topic.ingress_rejected_total==1,"overflow fault watermark travels atomically even on older queued publications");
    check(receive_wire(queue,tunnel(),1000)==Result::Accepted&&queue.front()->reception_sequence==10,"queue-full gap remains visible downstream");
    check(queue.counters().outcomes[static_cast<std::size_t>(Result::QueueFull)]==1&&queue.counters().last_rejected_hrt==999,"rejection reason and original HRT trace retained");
    check(receive_wire(queue,tunnel(128,payload_type,2,1),1001)==Result::WrongTarget
        &&queue.counters().first_fault==Result::QueueFull&&queue.counters().first_fault_hrt==999
        &&queue.counters().last_fault==Result::WrongTarget&&queue.counters().last_fault_hrt==1001,"first fault immutable and last fault updated");
    // Sequence numbers on the MAVLink transport are not estimator generations.
    Receiver wrap;auto wrapped=decoded.frame;wrapped.seq=255;
    check(wrap.receive(wrapped,2000,1,1,3)==Result::Accepted,"transport seq255 accepted");
    wrapped.seq=0;check(wrap.receive(wrapped,2000,1,1,3)==Result::Accepted&&wrap.pending()==2,"transport wrap/same-HRT do not fabricate sample identity");
    std::printf("{\"checks\":%u,\"failures\":%u,\"payload_type\":%u,\"queue_capacity\":%zu,\"real_generated_mavlink\":true,\"real_generated_uorb_type\":true,\"uorb_topic_size\":%zu,\"uorb_broker_executed\":false,\"board_actions\":0}\n",checks,failures,payload_type,queue_capacity,sizeof(topic));
    if(argc>1){
        FILE *file=std::fopen(argv[1],"wb");if(!file){std::fprintf(stderr,"Cannot write test result\n");return 2;}
        std::fprintf(file,"{\"checks\":%u,\"failures\":%u,\"payload_type\":%u,\"queue_capacity\":%zu,\"source_ids_are_authentication\":false,\"real_generated_mavlink\":true,\"real_generated_uorb_type\":true,\"uorb_topic_size\":%zu,\"uorb_broker_executed\":false,\"wire_delivery_proven\":false,\"board_actions\":0,\"cases\":[",checks,failures,payload_type,queue_capacity,sizeof(topic));
        for(std::size_t i=0;i<results.size();++i)std::fprintf(file,"%s{\"name\":\"%s\",\"passed\":%s}",i?",":"",results[i].name,results[i].passed?"true":"false");
        std::fprintf(file,"]}\n");std::fclose(file);
    }
    return failures?1:0;
}
