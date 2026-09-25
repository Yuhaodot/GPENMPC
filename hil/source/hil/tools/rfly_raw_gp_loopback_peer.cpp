// Serve retained GP fixtures on loopback.
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "../rfly_vendor_integration/px4_wire/CanonicalLocalGpWire.hpp"
#include "../rfly_vendor_integration/application_integration/build_fmuv6c/mavlink/common/mavlink.h"

namespace gw = gpenmpc_local_gp_wire;
struct Entry { std::uint8_t direction{}; std::uint32_t length{}; std::int64_t qpc{}; unsigned char bytes[300]{}; };
struct FrameEntry {
    std::uint32_t datagram_index{}, frame_index{}, offset{}, length{};
    std::uint8_t direction{}; std::int64_t qpc{};
};
struct Round { std::int64_t first{}, last{}; };
static bool exists(const std::wstring& p) { return GetFileAttributesW(p.c_str()) != INVALID_FILE_ATTRIBUTES; }
static bool text_file(const std::wstring& p, const char* text) {
    if (exists(p)) return false;
    FILE* f = _wfopen(p.c_str(), L"wb"); if (!f) return false;
    const auto n = std::strlen(text); const bool ok = std::fwrite(text, 1, n, f) == n;
    return std::fclose(f) == 0 && ok;
}
int wmain(int argc, wchar_t** argv) {
    if (argc < 3 || argc > 5) return 2;
    const bool event_wait = argc >= 4 && std::wcscmp(argv[3], L"select") == 0;
    if (argc >= 4 && !event_wait && std::wcscmp(argv[3], L"sleep") != 0) return 2;
    const bool reply_pairs = argc == 5 && std::wcscmp(argv[4], L"reply_pairs") == 0;
    if (argc == 5 && !reply_pairs) return 2;
    const std::wstring directory(argv[2]);
    const auto attr = GetFileAttributesW(directory.c_str());
    if (attr == INVALID_FILE_ATTRIBUTES || !(attr & FILE_ATTRIBUTE_DIRECTORY)) return 2;
    for (const auto* name : {L"READY.json", L"STOP.txt", L"PEER_RAW.bin", L"PEER_RESULT.json", L"ROUND_TRIPS.csv"})
        if (exists(directory + L"\\" + name)) return 2;
    if (reply_pairs && exists(directory + L"\\PEER_DATAGRAM_FRAMES.csv")) return 2;
    std::vector<unsigned char> fixture(596 * 59 + 1);
    FILE* f = _wfopen(argv[1], L"rb"); if (!f) return 2;
    const auto n = std::fread(fixture.data(), 1, fixture.size(), f);
    const bool read_ok = std::ferror(f) == 0; std::fclose(f);
    if (!read_ok || n != 596 * 59) return 2;
    fixture.resize(n);
    gw::Request queries[59]{};
    for (unsigned k = 0; k < 59; ++k) {
        gw::RequestBytes b{}; gw::ReplyBytes r{}; gw::Reply reply{};
        std::memcpy(b.data(), fixture.data() + 596*k, b.size());
        std::memcpy(r.data(), fixture.data() + 596*k + b.size(), r.size());
        if (!gw::decode(b, queries[k]) || !gw::decode(r, reply) || !gw::matches(queries[k], reply)) return 2;
    }
    std::vector<Entry> ledger; ledger.reserve(1000);
    std::vector<FrameEntry> frame_ledger;
    if (reply_pairs) frame_ledger.reserve(2000);
    Round rounds[59]{}; gw::ReplyBytes reply_bytes{};
    LARGE_INTEGER frequency{}, begin{}, tick{};
    QueryPerformanceFrequency(&frequency); QueryPerformanceCounter(&begin);
    WSADATA wsa{}; int error = WSAStartup(MAKEWORD(2,2), &wsa);
    const bool wsa_ok = error == 0; SOCKET sock = INVALID_SOCKET;
    bool bound = false, closed = false, stopped = false, timeout = false, started = false;
    unsigned sent = 0, received = 0, completed = 0, launched = 0, fragments = 0, fragment_index = 0, unexpected = 0;
    unsigned received_frames = 0, reply_pair_datagrams = 0;
    std::uint8_t expected_sequence = 0;
    sockaddr_in address{}, target{};
    address.sin_family = target.sin_family = AF_INET;
    address.sin_addr.s_addr = target.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(62321); target.sin_port = htons(62322);
    if (!error) { sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); if(sock == INVALID_SOCKET) error=WSAGetLastError(); }
    if (!error) {
        BOOL exclusive=TRUE;
        if (setsockopt(sock,SOL_SOCKET,SO_EXCLUSIVEADDRUSE,reinterpret_cast<const char*>(&exclusive),sizeof(exclusive))) error=WSAGetLastError();
    }
    if (!error) { bound = bind(sock,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0; if(!bound)error=WSAGetLastError(); }
    if (!error) { u_long mode=1; if(ioctlsocket(sock,FIONBIO,&mode))error=WSAGetLastError(); }
    if (!error && !text_file(directory+L"\\READY.json","{\"ready\":true,\"loopback_only\":true}\n")) error=ERROR_WRITE_FAULT;
    const auto record = [&](std::uint8_t direction,const unsigned char* bytes,int count) {
        if(count<1||count>300||ledger.size()>=1000){error=ERROR_BUFFER_OVERFLOW;return;}
        Entry e{};e.direction=direction;e.length=static_cast<std::uint32_t>(count);
        QueryPerformanceCounter(&tick);e.qpc=tick.QuadPart;std::memcpy(e.bytes,bytes,static_cast<std::size_t>(count));ledger.push_back(e);
        if(reply_pairs&&direction==1){
            FrameEntry frame{};frame.datagram_index=static_cast<std::uint32_t>(ledger.size());
            frame.direction=direction;frame.frame_index=1;frame.length=e.length;frame.qpc=e.qpc;
            frame_ledger.push_back(frame); // Requests remain one original frame/datagram.
        }
    };
    const auto send_query = [&]() {
        if(completed>=59||error)return;
        gw::RequestBytes b{};std::memcpy(b.data(),fixture.data()+596*completed,b.size());
        QueryPerformanceCounter(&tick);rounds[completed].first=tick.QuadPart;
        ++launched;fragment_index=0;reply_bytes={};
        for(unsigned j=0;j<3&&!error;++j){
            gw::Fragment frag{};if(!gw::fragment(b,j,frag)){error=ERROR_INVALID_DATA;break;}
            mavlink_message_t msg{};unsigned char bytes[MAVLINK_MAX_PACKET_LEN]{};
            mavlink_msg_tunnel_pack(queries[completed].identity.system,queries[completed].identity.component,
                &msg,255,190,42002,frag.length,frag.payload);
            const auto len=mavlink_msg_to_send_buffer(bytes,&msg);
            record(1,bytes,len);if(error)break;
            const int wrote=sendto(sock,reinterpret_cast<const char*>(bytes),len,0,reinterpret_cast<sockaddr*>(&target),sizeof(target));
            if(wrote!=len){error=wrote==SOCKET_ERROR?WSAGetLastError():ERROR_WRITE_FAULT;break;}++sent;
        }
    };
    while(!error){
        QueryPerformanceCounter(&tick);
        if(static_cast<double>(tick.QuadPart-begin.QuadPart)/frequency.QuadPart>=30){timeout=true;break;}
        if(exists(directory+L"\\STOP.txt")){stopped=true;break;}
        unsigned char bytes[301]{};sockaddr_in source{};int size=sizeof(source);
        const int count=recvfrom(sock,reinterpret_cast<char*>(bytes),sizeof(bytes),0,reinterpret_cast<sockaddr*>(&source),&size);
        if(count==SOCKET_ERROR){
            const int e=WSAGetLastError();
            if(e==WSAEWOULDBLOCK){
                // Use select to wake on packet arrival without Sleep(1) quantization.
                // The timeout checks STOP and the wall-clock bound.
                if(event_wait){
                    fd_set readable;FD_ZERO(&readable);FD_SET(sock,&readable);
                    timeval limit{0,10000};
                    if(select(0,&readable,nullptr,nullptr,&limit)==SOCKET_ERROR){error=WSAGetLastError();break;}
                }else Sleep(1);
                continue;
            }
            error=e;break;
        }
        ++received;record(0,bytes,count);if(error)break;
        if(source.sin_addr.s_addr!=htonl(INADDR_LOOPBACK)||source.sin_port!=htons(62322)||count<12||count>300){++unexpected;error=ERROR_INVALID_DATA;break;}
        // Validate complete datagram boundaries before dispatch.
        const auto datagram_index=static_cast<std::uint32_t>(ledger.size());
        const auto datagram_qpc=ledger.back().qpc;
        unsigned frame_offsets[2]{},frame_lengths[2]{},frame_count=0,position=0;
        while(position<static_cast<unsigned>(count)){
            if(frame_count>=(reply_pairs?2u:1u)||static_cast<unsigned>(count)-position<12||
               bytes[position]!=MAVLINK_STX||bytes[position+2]!=0||bytes[position+3]!=0){
                ++unexpected;error=ERROR_INVALID_DATA;break;
            }
            const unsigned length=unsigned(bytes[position+1])+12;
            if(length>static_cast<unsigned>(count)-position){++unexpected;error=ERROR_INVALID_DATA;break;}
            frame_offsets[frame_count]=position;frame_lengths[frame_count]=length;++frame_count;position+=length;
        }
        if(error)break;
        // Require readiness and next-query replies to arrive as separate protocol events.
        if(frame_count==2&&(!started||fragment_index==2)){++unexpected;error=ERROR_INVALID_DATA;break;}
        for(unsigned frame_index=0;frame_index<frame_count&&!error;++frame_index){
            const unsigned frame_offset=frame_offsets[frame_index],frame_length=frame_lengths[frame_index];
            mavlink_message_t parser{},msg{};mavlink_status_t parse{},status{};unsigned decoded=0;
            for(unsigned j=0;j<frame_length;++j){
                const auto state=mavlink_frame_char_buffer(&parser,&parse,bytes[frame_offset+j],&msg,&status);
                if(state==MAVLINK_FRAMING_BAD_CRC||state==MAVLINK_FRAMING_BAD_SIGNATURE||status.packet_rx_drop_count){
                    ++unexpected;error=ERROR_INVALID_DATA;break;
                }
                if(state==MAVLINK_FRAMING_OK){
                    ++decoded;
                    if(j+1!=frame_length){++unexpected;error=ERROR_INVALID_DATA;break;}
                }
            }
            if(error)break;
            if(decoded!=1||parse.parse_state!=MAVLINK_PARSE_STATE_IDLE||msg.magic!=MAVLINK_STX||
               msg.sysid!=255||msg.compid!=190||(reply_pairs&&msg.seq!=expected_sequence)){
                ++unexpected;error=ERROR_INVALID_DATA;break;
            }
            ++received_frames;++expected_sequence;
            if(reply_pairs){
                FrameEntry frame{};frame.datagram_index=datagram_index;frame.direction=0;
                frame.frame_index=frame_index+1;frame.offset=frame_offset;frame.length=frame_length;frame.qpc=datagram_qpc;
                frame_ledger.push_back(frame); // Exact datagram tick, never a fabricated per-frame arrival.
            }
            // Dispatch each completed parser event immediately.
            if(!started){
                mavlink_heartbeat_t hb{};
                if(msg.msgid!=MAVLINK_MSG_ID_HEARTBEAT){++unexpected;error=ERROR_INVALID_DATA;break;}
                mavlink_msg_heartbeat_decode(&msg,&hb);
                if(hb.base_mode&MAV_MODE_FLAG_SAFETY_ARMED){++unexpected;error=ERROR_INVALID_DATA;break;}
                started=true;send_query();continue;
            }
            if(completed>=59||msg.msgid!=MAVLINK_MSG_ID_TUNNEL){++unexpected;error=ERROR_INVALID_DATA;break;}
            mavlink_tunnel_t tunnel{};mavlink_msg_tunnel_decode(&msg,&tunnel);
            const auto& q=queries[completed];
            std::uint64_t gen=0;for(unsigned j=1;j<9;++j)gen=(gen<<8)|tunnel.payload[j];
            const unsigned offset=119*fragment_index;
            const unsigned payload_size=286-offset<119?286-offset:119;
            if(tunnel.target_system!=q.identity.system||tunnel.target_component!=q.identity.component||
               tunnel.payload_type!=42002||tunnel.payload[0]!=(0x90|fragment_index)||
               tunnel.payload_length!=payload_size+9||gen!=q.output_generation){++unexpected;error=ERROR_INVALID_DATA;break;}
            std::memcpy(reply_bytes.data()+offset,tunnel.payload+9,payload_size);++fragments;++fragment_index;
            if(fragment_index==3){
                gw::Reply reply{};
                if(!gw::decode(reply_bytes,reply)||!gw::matches(q,reply)||
                   std::memcmp(reply_bytes.data(),fixture.data()+596*completed+310,286)!=0){++unexpected;error=ERROR_INVALID_DATA;break;}
                QueryPerformanceCounter(&tick);rounds[completed].last=tick.QuadPart;++completed;send_query();
            }
        }
        if(!error&&frame_count==2)++reply_pair_datagrams;
    }
    if(sock!=INVALID_SOCKET){closed=closesocket(sock)==0;if(!closed&&!error)error=WSAGetLastError();}
    if(wsa_ok&&WSACleanup()&&!error)error=WSAGetLastError();
    bool raw_ok=true;
    f=_wfopen((directory+L"\\PEER_RAW.bin").c_str(),L"wb");
    if(!f)raw_ok=false;else{
        for(const auto& e:ledger){
            raw_ok=raw_ok&&std::fwrite(&e.direction,1,1,f)==1;
            raw_ok=raw_ok&&std::fwrite(&e.length,4,1,f)==1;
            raw_ok=raw_ok&&std::fwrite(&e.qpc,8,1,f)==1;
            raw_ok=raw_ok&&std::fwrite(e.bytes,1,e.length,f)==e.length;
        }raw_ok=std::fclose(f)==0&&raw_ok;
    }
    f=_wfopen((directory+L"\\ROUND_TRIPS.csv").c_str(),L"wb");
    if(!f)raw_ok=false;else{
        std::fprintf(f,"query,first_send_qpc,reply_qpc,elapsed_ms\n");
        for(unsigned k=0;k<launched;++k)std::fprintf(f,"%u,%lld,%lld,%.9f\n",k+1,
            static_cast<long long>(rounds[k].first),static_cast<long long>(rounds[k].last),
            rounds[k].last?1000.0*(rounds[k].last-rounds[k].first)/frequency.QuadPart:-1.0);
        raw_ok=std::fclose(f)==0&&raw_ok;
    }
    if(reply_pairs){
        f=_wfopen((directory+L"\\PEER_DATAGRAM_FRAMES.csv").c_str(),L"wb");
        if(!f)raw_ok=false;else{
            bool frames_ok=std::fprintf(f,"datagram_index,direction,frame_index,offset,length,qpc\n")>=0;
            for(const auto& frame:frame_ledger)
                frames_ok=std::fprintf(f,"%u,%u,%u,%u,%u,%lld\n",frame.datagram_index,unsigned(frame.direction),
                    frame.frame_index,frame.offset,frame.length,static_cast<long long>(frame.qpc))>=0&&frames_ok;
            raw_ok=std::fclose(f)==0&&frames_ok&&raw_ok;
        }
    }
    if(!raw_ok&&!error)error=ERROR_WRITE_FAULT;
    char result[1800]{};QueryPerformanceCounter(&tick);
    std::snprintf(result,sizeof(result),
        "{\"scope\":\"RETAINED_C_GP_FIXTURE_NATIVE_TRANSPORT_NO_LIVE_AUTHORITY\",\"peer_wait_strategy\":\"%s\","
        "\"transport_proof_scope\":\"HOST_ONLY_NO_COPTERSIM_UDP_COM_PROOF\",\"reply_pairs_enabled\":%s,"
        "\"reply_pair_datagrams\":%u,\"received_frame_count\":%u,"
        "\"queries_sent\":%u,\"queries_completed\":%u,\"received_count\":%u,\"sent_count\":%u,\"reply_fragments\":%u,"
        "\"unexpected_count\":%u,\"all_replies_exact_fixture\":%s,\"bind_ok\":%s,\"close_ok\":%s,\"error_code\":%d,"
        "\"stop_requested\":%s,\"hard_timeout\":%s,\"qpc_frequency\":%lld,\"elapsed_s\":%.9f,"
        "\"fixture_rows\":59,\"hard_wall_bound_s\":30,\"raw_written\":%s,\"COM\":0,\"board\":0,\"controller\":0,\"plant\":0}\n",
        event_wait?"SOCKET_READABLE_SELECT":"LEGACY_SLEEP_1MS",reply_pairs?"true":"false",reply_pair_datagrams,received_frames,
        launched,completed,received,sent,fragments,unexpected,completed==59?"true":"false",bound?"true":"false",closed?"true":"false",error,
        stopped?"true":"false",timeout?"true":"false",static_cast<long long>(frequency.QuadPart),
        static_cast<double>(tick.QuadPart-begin.QuadPart)/frequency.QuadPart,raw_ok?"true":"false");
    const bool result_ok=text_file(directory+L"\\PEER_RESULT.json",result);
    return !error&&completed==59&&stopped&&!timeout&&closed&&result_ok?0:1;
}
