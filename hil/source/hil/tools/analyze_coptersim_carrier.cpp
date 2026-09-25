// Replay retained NoUI channels and reconstruct complete MAVLink frames.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <map>
#include <vector>
#include <mavlink/common/mavlink.h>
struct Stream{mavlink_message_t rx{};mavlink_status_t status{};std::map<std::uint32_t,std::uint64_t> counts;};
struct Frame{std::vector<unsigned char> wire;unsigned source_channel;std::uint32_t id;unsigned char payload[128];};
struct Command{unsigned channel;std::uint64_t qpc;unsigned sys,comp; mavlink_command_long_t value;};
int wmain(int argc,wchar_t**argv){
    if(argc!=3)return 2;FILE*f=_wfopen(argv[1],L"rb");if(!f)return 3;
    Stream streams[5];std::vector<Frame> originals;std::vector<Command> commands;
    std::uint64_t all_control_messages=0,nonzero_control_values=0,nonfinite_control_values=0;
    std::uint64_t chunks=0,bytes=0,bad=0,decoded=0,full_exact=0,payload_only=0,unmatched=0,trailing=0;
    std::map<unsigned,unsigned> exact_by_channel;bool complete=true;
    for(;;){std::uint64_t qpc=0;std::uint32_t channel=0,n=0;const auto got=fread(&qpc,1,8,f);if(!got)break;
        if(got!=8||fread(&channel,4,1,f)!=1||fread(&n,4,1,f)!=1||channel>4||n>8*1024*1024){complete=false;break;}
        std::vector<unsigned char>data(n);if(fread(data.data(),1,n,f)!=n){complete=false;break;}
        ++chunks;bytes+=n;auto&s=streams[channel];
        for(auto b:data){mavlink_message_t parsed{};mavlink_status_t status{};
            const auto result=mavlink_frame_char_buffer(&s.rx,&s.status,b,&parsed,&status);
            if(result==MAVLINK_FRAMING_BAD_CRC||result==MAVLINK_FRAMING_BAD_SIGNATURE){++bad;continue;}
            if(result!=MAVLINK_FRAMING_OK)continue;++decoded;++s.counts[parsed.msgid];
            if(parsed.msgid==MAVLINK_MSG_ID_COMMAND_LONG){mavlink_command_long_t c{};mavlink_msg_command_long_decode(&parsed,&c);commands.push_back({channel,qpc,parsed.sysid,parsed.compid,c});}
            if(parsed.msgid==MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS){mavlink_hil_actuator_controls_t c{};mavlink_msg_hil_actuator_controls_decode(&parsed,&c);++all_control_messages;for(const auto v:c.controls){if(!std::isfinite(v))++nonfinite_control_values;else if(v!=0)++nonzero_control_values;}}
            if(parsed.msgid!=MAVLINK_MSG_ID_TUNNEL)continue;
            mavlink_tunnel_t tunnel{};mavlink_msg_tunnel_decode(&parsed,&tunnel);
            if(tunnel.payload_type!=42002||tunnel.payload_length!=128)continue;
            unsigned char wire[MAVLINK_MAX_PACKET_LEN];const auto length=mavlink_msg_to_send_buffer(wire,&parsed);
            if(channel==2||channel==3){Frame frame{std::vector<unsigned char>(wire,wire+length),channel,parsed.msgid,{}};std::memcpy(frame.payload,tunnel.payload,128);originals.push_back(frame);continue;}
            bool found=false;for(const auto&o:originals){if(std::memcmp(o.payload,tunnel.payload,128))continue;found=true;
                if(o.wire.size()==length&&std::memcmp(o.wire.data(),wire,length)==0){++full_exact;++exact_by_channel[channel];}else ++payload_only;break;}
            if(!found)++unmatched;
        }
    }fclose(f);
    for(auto&s:streams)if(s.status.parse_state!=MAVLINK_PARSE_STATE_IDLE&&s.status.parse_state!=MAVLINK_PARSE_STATE_UNINIT)++trailing;
    f=_wfopen(argv[2],L"wb");if(!f)return 4;
    fprintf(f,"{\"scope\":\"Offline software-carrier packet replay\",\"raw_chunks\":%llu,\"raw_bytes\":%llu,\"complete_raw_container\":%s,\"decoded_frames\":%llu,\"bad_crc_or_signature\":%llu,\"streams_with_partial_frame\":%llu,\"recorded_tunnel_sends\":%zu,\"received_tunnel_full_wire_bit_exact\":%llu,\"received_payload_only_not_wire_exact\":%llu,\"received_tunnel_unmatched\":%llu,\"wire_exact_host_api_receive\":%u,\"wire_exact_fake_px4_udp_receive\":%u,\"channel_message_counts\":[",
        static_cast<unsigned long long>(chunks),static_cast<unsigned long long>(bytes),complete?"true":"false",static_cast<unsigned long long>(decoded),static_cast<unsigned long long>(bad),static_cast<unsigned long long>(trailing),originals.size(),static_cast<unsigned long long>(full_exact),static_cast<unsigned long long>(payload_only),static_cast<unsigned long long>(unmatched),exact_by_channel[1],exact_by_channel[4]);
    bool first=true;for(unsigned channel=0;channel<5;++channel)for(const auto&entry:streams[channel].counts){if(!first)fputc(',',f);first=false;fprintf(f,"{\"channel\":%u,\"message_id\":%u,\"count\":%llu}",channel,entry.first,static_cast<unsigned long long>(entry.second));}
    fprintf(f,"],\"observed_command_long\":[");first=true;for(const auto&r:commands){if(!first)fputc(',',f);first=false;const auto&c=r.value;fprintf(f,"{\"channel\":%u,\"qpc\":%llu,\"source_system\":%u,\"source_component\":%u,\"command\":%u,\"target_system\":%u,\"target_component\":%u,\"confirmation\":%u,\"params\":[%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g]}",r.channel,static_cast<unsigned long long>(r.qpc),r.sys,r.comp,c.command,c.target_system,c.target_component,c.confirmation,c.param1,c.param2,c.param3,c.param4,c.param5,c.param6,c.param7);}
    fprintf(f,"],\"all_channels_hil_actuator_messages\":%llu,\"nonzero_control_values\":%llu,\"nonfinite_control_values\":%llu,\"COM_open\":0,\"simulator_launches\":0,\"board_actions\":0}\n",static_cast<unsigned long long>(all_control_messages),static_cast<unsigned long long>(nonzero_control_values),static_cast<unsigned long long>(nonfinite_control_values));fclose(f);return complete&&bad==0&&trailing==0?0:5;
}
