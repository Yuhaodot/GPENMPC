// Run the NoUI software-mode probe with a loopback peer and zero actuator input.
// Record automatic simulator commands and contain NoUI in the owned Job.
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <iphlpapi.h>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <mavlink/common/mavlink.h>
#include "../rfly_vendor_integration/clock_tap_overlay/DllSharedGetterTap.hpp"
#include "../rfly_vendor_integration/clock_tap_overlay/DllGetterRing.hpp"
#include "../rfly_vendor_integration/clock_tap_overlay/DllStepSnapshotSharedStatus.hpp"

struct Raw { std::uint64_t qpc; std::uint32_t channel; std::vector<unsigned char> bytes; };
static std::vector<Raw> raw;
static std::size_t raw_bytes=0;
static LARGE_INTEGER frequency;
static std::uint64_t now(){LARGE_INTEGER q;QueryPerformanceCounter(&q);return static_cast<std::uint64_t>(q.QuadPart);}
static double seconds(std::uint64_t t){return double(now()-t)/double(frequency.QuadPart);}
static bool record(unsigned channel,const unsigned char*p,std::size_t n){
    if(raw_bytes+n>8U*1024U*1024U)return false;
    raw.push_back({now(),channel,std::vector<unsigned char>(p,p+n)});raw_bytes+=n;return true;
}
static sockaddr_in address(unsigned port){sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(static_cast<u_short>(port));return a;}
static bool exclusive_bind(SOCKET s,unsigned port){BOOL one=TRUE;setsockopt(s,SOL_SOCKET,SO_EXCLUSIVEADDRUSE,reinterpret_cast<const char*>(&one),sizeof(one));auto a=address(port);return bind(s,reinterpret_cast<sockaddr*>(&a),sizeof(a))==0;}
static bool owner_listening(unsigned port,DWORD pid){
    ULONG size=0;GetExtendedTcpTable(nullptr,&size,FALSE,AF_INET,TCP_TABLE_OWNER_PID_LISTENER,0);
    std::vector<unsigned char>b(size);if(GetExtendedTcpTable(b.data(),&size,FALSE,AF_INET,TCP_TABLE_OWNER_PID_LISTENER,0)!=NO_ERROR)return false;
    auto*t=reinterpret_cast<MIB_TCPTABLE_OWNER_PID*>(b.data());
    for(DWORD i=0;i<t->dwNumEntries;++i)if(ntohs(static_cast<u_short>(t->table[i].dwLocalPort))==port&&t->table[i].dwOwningPid==pid)return true;
    return false;
}
static bool transmit(SOCKET s,const mavlink_message_t&m,bool udp,unsigned port){
    unsigned char b[MAVLINK_MAX_PACKET_LEN];const auto n=mavlink_msg_to_send_buffer(b,&m);int sent=0;
    if(udp){auto a=address(port);sent=sendto(s,reinterpret_cast<const char*>(b),n,0,reinterpret_cast<sockaddr*>(&a),sizeof(a));}
    else {while(sent<n){const auto k=send(s,reinterpret_cast<const char*>(b+sent),n-sent,0);if(k<=0)return false;sent+=k;}}
    return sent==n&&record(udp?3:2,b,n);
}
int wmain(int argc,wchar_t**argv){
    if(argc!=4&&argc!=5&&argc!=6)return 2; // optional independent RDR1 diagnostic ABI
    const std::wstring exe=argv[1],runtime=argv[2],out=argv[3];
    const std::wstring model=argc>=5?argv[4]:L"0";
    if(model!=L"0"&&model!=L"GPENMPC_M600_Diagnostic")return 2;
    const bool ring_requested=argc==6;
    const bool snapshot_requested=argc==6&&std::wstring(argv[5])==L"RDR1_SSS1";
#ifdef GPENMPC_EXTERNAL_RDR_MATLAB_CONSUMER
    const bool external_consumer=true;
    if(!snapshot_requested)return 2;
#else
    const bool external_consumer=false;
#endif
    if(ring_requested&&((std::wstring(argv[5])!=L"RDR1"&&!snapshot_requested)||model!=L"GPENMPC_M600_Diagnostic"))return 2;
    constexpr unsigned id=48,tcp_port=4560+id-1,udp_port=20100+2*(id-1);
    QueryPerformanceFrequency(&frequency);WSADATA ws{};if(WSAStartup(MAKEWORD(2,2),&ws))return 3;
    SOCKET tcp=INVALID_SOCKET,udp=INVALID_SOCKET,px4_udp=INVALID_SOCKET;HANDLE job=nullptr;PROCESS_INFORMATION pi{};HANDLE log=INVALID_HANDLE_VALUE;
    constexpr unsigned px4_udp_port=14580+id-1;
    bool connected=false,terminated=false,released=false,port_released=false;unsigned tcp_tunnel=0,udp_tunnel=0,control_udp_tunnel=0,control_udp_ping=0,hil=0,gps=0,bad=0,zero_inputs=0;
    unsigned vendor_mode_requests=0,vendor_arm_requests=0,vendor_capability_requests=0,vendor_other_commands=0;
    HANDLE section_handle=nullptr;gpenmpc_clock_tap::SharedSection*section=nullptr;
    HANDLE ring_handle=nullptr;gpenmpc_clock_ring::Section*ring=nullptr;gpenmpc_clock_ring::Consumer ring_consumer;
    HANDLE snapshot_handle=nullptr;gpenmpc_snapshot_status::Section*snapshot=nullptr;
    std::uint64_t ring_nonce=0,snapshot_runtime_first_fault=0,snapshot_runtime_status_errors=0;
    unsigned snapshot_runtime_reads=0;
    std::vector<gpenmpc_clock_ring::Sample> ring_records;std::vector<std::uint64_t> ring_read_qpc;
    constexpr std::size_t maximum_retained_getters=8192; // three-second diagnostic storage only
    if(ring_requested){ring_records.reserve(maximum_retained_getters);ring_read_qpc.reserve(maximum_retained_getters);}
    mavlink_message_t rx_messages[3]{};mavlink_status_t rx_status[3]{};
    // parse_char hides CRC errors; validate the actual framing API first.
    unsigned crc_negative=0,crc_positive=0;
    {mavlink_message_t m{},rx{},parsed{};mavlink_status_t s{},reported{};unsigned char bytes[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_ping_pack(255,190,&m,123,42,48,1);auto n=mavlink_msg_to_send_buffer(bytes,&m);bytes[n-1]^=1;
        for(unsigned k=0;k<n;++k)crc_negative+=mavlink_frame_char_buffer(&rx,&s,bytes[k],&parsed,&reported)==MAVLINK_FRAMING_BAD_CRC;
        bytes[n-1]^=1;rx={};s={};for(unsigned k=0;k<n;++k)crc_positive+=mavlink_frame_char_buffer(&rx,&s,bytes[k],&parsed,&reported)==MAVLINK_FRAMING_OK;
        if(crc_negative!=1||crc_positive!=1)return 9;
    }
    std::uint64_t first_hil=0,last_hil=0;bool strictly_monotone=true;DWORD child_exit=STILL_ACTIVE;
    const char*failure=nullptr;
    auto drain_ring=[&](){
        if(snapshot){gpenmpc_snapshot_status::View v{};
            if(!gpenmpc_snapshot_status::read_atomic(*snapshot,v))return false;
            ++snapshot_runtime_reads;
            if(v.counters[1]&&!snapshot_runtime_first_fault)snapshot_runtime_first_fault=v.counters[1];
            snapshot_runtime_status_errors|=static_cast<std::uint64_t>(v.sticky_errors);
        }
        if(!ring||external_consumer)return true;
        for(unsigned n=0;n<gpenmpc_clock_ring::capacity;++n){gpenmpc_clock_ring::Sample sample{};
            const auto r=ring_consumer.take(sample);if(r==gpenmpc_clock_ring::Read::Empty)return true;
            if(r!=gpenmpc_clock_ring::Read::Record)return false;
            if(ring_records.size()==maximum_retained_getters)return false;
            ring_records.push_back(sample);ring_read_qpc.push_back(now());
        }
        return true;
    };
    unsigned char to_board[128],to_host[128];for(unsigned i=0;i<128;++i){to_board[i]=static_cast<unsigned char>(i^0x5b);to_host[i]=static_cast<unsigned char>(i^0xa7);}
    struct TimeRow{std::uint64_t qpc,wire;unsigned seq;};std::vector<TimeRow> times;
    do {
        SetEnvironmentVariableW(gpenmpc_clock_tap::section_environment,nullptr);
        if(!external_consumer){SetEnvironmentVariableW(gpenmpc_clock_ring::environment,nullptr);
            SetEnvironmentVariableW(gpenmpc_snapshot_status::environment,nullptr);}
        if(external_consumer){
            wchar_t name[192]{},sn[192]{};
            const auto n=GetEnvironmentVariableW(gpenmpc_clock_ring::environment,name,192);
            const auto m=GetEnvironmentVariableW(gpenmpc_snapshot_status::environment,sn,192);
            if(!gpenmpc_snapshot_status::valid_name(name,n,gpenmpc_clock_ring::prefix,sizeof(gpenmpc_clock_ring::prefix)/sizeof(wchar_t)-1)||
               !gpenmpc_snapshot_status::valid_name(sn,m,gpenmpc_snapshot_status::prefix,sizeof(gpenmpc_snapshot_status::prefix)/sizeof(wchar_t)-1)){
                failure="EXTERNAL_READER_NAME";break;}
            ring_handle=OpenFileMappingW(FILE_MAP_ALL_ACCESS,FALSE,name);snapshot_handle=OpenFileMappingW(FILE_MAP_ALL_ACCESS,FALSE,sn);
            if(ring_handle)ring=static_cast<gpenmpc_clock_ring::Section*>(MapViewOfFile(ring_handle,FILE_MAP_ALL_ACCESS,0,0,gpenmpc_clock_ring::bytes));
            if(snapshot_handle)snapshot=static_cast<gpenmpc_snapshot_status::Section*>(MapViewOfFile(snapshot_handle,FILE_MAP_ALL_ACCESS,0,0,gpenmpc_snapshot_status::bytes));
            if(!ring||!snapshot||!gpenmpc_clock_ring::shape(ring->header)||!gpenmpc_snapshot_status::shape(*snapshot)||
               ring->header.peer_nonce!=snapshot->peer_nonce||gpenmpc_clock_ring::load(&ring->header.consumer_state)!=gpenmpc_clock_ring::ConsumerActive||
               ring->header.consumer_pid==GetCurrentProcessId()||!ring->header.consumer_pid||ring->header.producer_pid||snapshot->producer_pid){failure="EXTERNAL_READER_BINDING";break;}
            ring_nonce=ring->header.peer_nonce;
        }else if(ring_requested){
            const auto nonce=now();const auto name=std::wstring(gpenmpc_clock_ring::prefix)+std::to_wstring(GetCurrentProcessId())+L"_"+std::to_wstring(nonce);
            ring_nonce=nonce;
            ring_handle=CreateFileMappingW(INVALID_HANDLE_VALUE,nullptr,PAGE_READWRITE,0,static_cast<DWORD>(gpenmpc_clock_ring::bytes),name.c_str());
            if(!ring_handle||GetLastError()==ERROR_ALREADY_EXISTS){failure="NEW_RING_NOT_UNIQUE";break;}
            ring=static_cast<gpenmpc_clock_ring::Section*>(MapViewOfFile(ring_handle,FILE_MAP_ALL_ACCESS,0,0,gpenmpc_clock_ring::bytes));
            if(!ring||!gpenmpc_clock_ring::initialize_for_peer(*ring,nonce)||!ring_consumer.claim(*ring)||
                !SetEnvironmentVariableW(gpenmpc_clock_ring::environment,name.c_str())){failure="NEW_RING_INITIALIZATION";break;}
            if(snapshot_requested){
                const auto status_name=std::wstring(gpenmpc_snapshot_status::prefix)+std::to_wstring(GetCurrentProcessId())+L"_"+std::to_wstring(nonce);
                snapshot_handle=CreateFileMappingW(INVALID_HANDLE_VALUE,nullptr,PAGE_READWRITE,0,static_cast<DWORD>(gpenmpc_snapshot_status::bytes),status_name.c_str());
                if(!snapshot_handle||GetLastError()==ERROR_ALREADY_EXISTS){failure="NEW_SNAPSHOT_STATUS_NOT_UNIQUE";break;}
                snapshot=static_cast<gpenmpc_snapshot_status::Section*>(MapViewOfFile(snapshot_handle,FILE_MAP_ALL_ACCESS,0,0,gpenmpc_snapshot_status::bytes));
                if(!snapshot||!gpenmpc_snapshot_status::initialize_for_peer(*snapshot,nonce)||
                   !SetEnvironmentVariableW(gpenmpc_snapshot_status::environment,status_name.c_str())){failure="NEW_SNAPSHOT_STATUS_INITIALIZATION";break;}
            }
        }else if(model==L"GPENMPC_M600_Diagnostic"){
            const auto nonce=now();const auto name=std::wstring(gpenmpc_clock_tap::section_prefix)+std::to_wstring(GetCurrentProcessId())+L"_"+std::to_wstring(nonce);
            section_handle=CreateFileMappingW(INVALID_HANDLE_VALUE,nullptr,PAGE_READWRITE,0,static_cast<DWORD>(gpenmpc_clock_tap::section_bytes),name.c_str());
            if(!section_handle||GetLastError()==ERROR_ALREADY_EXISTS){failure="NEW_DIAGNOSTIC_SECTION_NOT_UNIQUE";break;}
            section=static_cast<gpenmpc_clock_tap::SharedSection*>(MapViewOfFile(section_handle,FILE_MAP_ALL_ACCESS,0,0,gpenmpc_clock_tap::section_bytes));
            if(!section||!gpenmpc_clock_tap::initialize_shared_for_peer(*section,nonce)||!SetEnvironmentVariableW(gpenmpc_clock_tap::section_environment,name.c_str())){failure="NEW_DIAGNOSTIC_SECTION_INITIALIZATION";break;}
        }
        SOCKET check=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
        if(check==INVALID_SOCKET||!exclusive_bind(check,tcp_port)){if(check!=INVALID_SOCKET)closesocket(check);failure="TCP_PORT_OCCUPIED_NO_CHILD";break;}closesocket(check);
        check=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
        if(check==INVALID_SOCKET||!exclusive_bind(check,udp_port)){if(check!=INVALID_SOCKET)closesocket(check);failure="UDP_INPUT_OCCUPIED_NO_CHILD";break;}closesocket(check);
        udp=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);if(udp==INVALID_SOCKET||!exclusive_bind(udp,udp_port+1)){failure="UDP_OUTPUT_OCCUPIED_NO_CHILD";break;}
        px4_udp=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);if(px4_udp==INVALID_SOCKET||!exclusive_bind(px4_udp,px4_udp_port)){failure="SOFTWARE_PX4_MAVLINK_UDP_OCCUPIED_NO_CHILD";break;}
        u_long nonblock=1;ioctlsocket(udp,FIONBIO,&nonblock);
        ioctlsocket(px4_udp,FIONBIO,&nonblock);
        job=CreateJobObjectW(nullptr,nullptr);JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};limits.BasicLimitInformation.LimitFlags=JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
        if(!job||!SetInformationJobObject(job,JobObjectExtendedLimitInformation,&limits,sizeof(limits))){failure="JOB_CONFIGURATION";break;}
        SECURITY_ATTRIBUTES sa{sizeof(sa),nullptr,TRUE};
        log=CreateFileW((out+L"\\COPTER_STDIO.raw.log").c_str(),GENERIC_WRITE,FILE_SHARE_READ,&sa,CREATE_NEW,FILE_ATTRIBUTE_NORMAL,nullptr);
        if(log==INVALID_HANDLE_VALUE){failure="LOG_CREATE";break;}
        STARTUPINFOW si{};si.cb=sizeof(si);si.dwFlags=STARTF_USESTDHANDLES|STARTF_USESHOWWINDOW;si.wShowWindow=SW_HIDE;si.hStdOutput=log;si.hStdError=log;
        std::wstring command=L"\""+exe+L"\" 1 48 -1 "+model+L" 1 Grasslands 127.0.0.1 0 0 0 1 2";
        std::vector<wchar_t> mutable_command(command.begin(),command.end());mutable_command.push_back(0);
        if(!CreateProcessW(exe.c_str(),mutable_command.data(),nullptr,nullptr,TRUE,CREATE_SUSPENDED|CREATE_NO_WINDOW|NORMAL_PRIORITY_CLASS,nullptr,runtime.c_str(),&si,&pi)){failure="CREATE_PROCESS";break;}
        if(!AssignProcessToJobObject(job,pi.hProcess)){failure="JOB_ASSIGN";TerminateProcess(pi.hProcess,4);break;}
        if(ResumeThread(pi.hThread)==static_cast<DWORD>(-1)){failure="RESUME";break;}
        const auto waiting=now();while(seconds(waiting)<10.0&&!owner_listening(tcp_port,pi.dwProcessId)){if(WaitForSingleObject(pi.hProcess,0)==WAIT_OBJECT_0)break;Sleep(20);}
        if(!owner_listening(tcp_port,pi.dwProcessId)){failure="OWNED_TCP_LISTENER_NOT_OBSERVED";break;}
        tcp=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);auto a=address(tcp_port);
        if(tcp==INVALID_SOCKET||connect(tcp,reinterpret_cast<sockaddr*>(&a),sizeof(a))){failure="LOOPBACK_CONNECT";break;}connected=true;
        DWORD timeout=100;setsockopt(tcp,SOL_SOCKET,SO_SNDTIMEO,reinterpret_cast<char*>(&timeout),sizeof(timeout));ioctlsocket(tcp,FIONBIO,&nonblock);
        mavlink_message_t message{};mavlink_msg_heartbeat_pack(id,1,&message,MAV_TYPE_HEXAROTOR,MAV_AUTOPILOT_PX4,MAV_MODE_FLAG_HIL_ENABLED,0,MAV_STATE_STANDBY);
        if(!transmit(tcp,message,false,0)){failure="INITIAL_DISARMED_HEARTBEAT_SEND";break;}
        const auto started=now();double next_zero=0.0;bool probes_sent=false;
        while(seconds(started)<3.0&&!failure){
            if(!drain_ring()){failure="RING_CONSUMER_OR_RETAINED_CAPTURE_FAILED";break;}
            const double elapsed=seconds(started);
            if(elapsed>=next_zero){float zeros[16]{};mavlink_msg_hil_actuator_controls_pack(id,1,&message,static_cast<std::uint64_t>(elapsed*1e6),zeros,0,0);
                if(!transmit(tcp,message,false,0)){failure="ZERO_INPUT_SEND";break;}++zero_inputs;next_zero+=.01;}
            if(!probes_sent&&elapsed>=.5){
                mavlink_msg_tunnel_pack(255,190,&message,id,1,42002,128,to_board);if(!transmit(udp,message,true,udp_port)){failure="HOST_TUNNEL_SEND";break;}
                mavlink_msg_ping_pack(255,190,&message,123456,424242,id,1);if(!transmit(udp,message,true,udp_port)){failure="HOST_BENIGN_PING_SEND";break;}
                mavlink_msg_tunnel_pack(id,1,&message,255,190,42002,128,to_host);if(!transmit(tcp,message,false,0)){failure="FAKE_PX4_TUNNEL_SEND";break;}probes_sent=true;
            }
            for(unsigned channel=0;channel<3&&!failure;++channel){
                unsigned char data[8192];const SOCKET s=channel==2?px4_udp:(channel?udp:tcp);
                for(unsigned burst=0;burst<64;++burst){const int n=recv(s,reinterpret_cast<char*>(data),sizeof(data),0);
                    if(n==SOCKET_ERROR){if(WSAGetLastError()!=WSAEWOULDBLOCK)failure="RECEIVE_ERROR";break;}
                    if(n==0){if(!channel)failure="TCP_EOF";break;}
                    if(!record(channel==2?4:channel,data,static_cast<std::size_t>(n))){failure="BOUNDED_CAPTURE_OVERFLOW";break;}
                    mavlink_status_t status{};mavlink_message_t parsed{};
                    for(int k=0;k<n;++k){const auto r=mavlink_frame_char_buffer(&rx_messages[channel],&rx_status[channel],data[k],&parsed,&status);
                        if(r==MAVLINK_FRAMING_BAD_CRC||r==MAVLINK_FRAMING_BAD_SIGNATURE){++bad;continue;}if(r!=MAVLINK_FRAMING_OK)continue;
                        if(!channel&&parsed.msgid==MAVLINK_MSG_ID_HIL_SENSOR){mavlink_hil_sensor_t h{};mavlink_msg_hil_sensor_decode(&parsed,&h);
                            if(hil&&h.time_usec<=last_hil)strictly_monotone=false;if(!hil)first_hil=h.time_usec;last_hil=h.time_usec;++hil;times.push_back({now(),h.time_usec,parsed.seq});}
                        if(!channel&&parsed.msgid==MAVLINK_MSG_ID_HIL_GPS)++gps;
                        if(channel==2&&parsed.msgid==MAVLINK_MSG_ID_PING){mavlink_ping_t p{};mavlink_msg_ping_decode(&parsed,&p);if(p.seq==424242&&p.time_usec==123456&&p.target_system==id&&p.target_component==1)++control_udp_ping;}
                        if(channel==2&&parsed.msgid==MAVLINK_MSG_ID_COMMAND_LONG){mavlink_command_long_t c{};mavlink_msg_command_long_decode(&parsed,&c);
                            if(c.command==MAV_CMD_DO_SET_MODE)++vendor_mode_requests;
                            else if(c.command==MAV_CMD_COMPONENT_ARM_DISARM)++vendor_arm_requests;
                            else if(c.command==MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES)++vendor_capability_requests;
                            else ++vendor_other_commands;}
                        if(parsed.msgid==MAVLINK_MSG_ID_TUNNEL){mavlink_tunnel_t t{};mavlink_msg_tunnel_decode(&parsed,&t);
                            if(t.payload_type==42002&&t.payload_length==128){if(!channel&&std::memcmp(t.payload,to_board,128)==0)++tcp_tunnel;if(channel==1&&std::memcmp(t.payload,to_host,128)==0)++udp_tunnel;if(channel==2&&std::memcmp(t.payload,to_board,128)==0)++control_udp_tunnel;}}
                    }
                }
            }
            if(WaitForSingleObject(pi.hProcess,0)==WAIT_OBJECT_0)failure="OWNED_CHILD_EXITED";
            Sleep(1);
        }
    }while(false);
    if(tcp!=INVALID_SOCKET){shutdown(tcp,SD_BOTH);closesocket(tcp);}if(udp!=INVALID_SOCKET)closesocket(udp);if(px4_udp!=INVALID_SOCKET)closesocket(px4_udp);
    if(pi.hProcess){
        // Keep consuming observations during the bounded normal-exit grace;
        // a blind 500ms wait would itself fill the 256-slot diagnostic ring.
        const auto grace=now();
        while(WaitForSingleObject(pi.hProcess,0)!=WAIT_OBJECT_0&&seconds(grace)<.5){
            if(!drain_ring()&&!failure)failure="EXIT_GRACE_RING_DRAIN_FAILED";Sleep(1);
        }
        if(WaitForSingleObject(pi.hProcess,0)!=WAIT_OBJECT_0){terminated=true;TerminateJobObject(job,0);}
        released=WaitForSingleObject(pi.hProcess,5000)==WAIT_OBJECT_0;GetExitCodeProcess(pi.hProcess,&child_exit);CloseHandle(pi.hThread);CloseHandle(pi.hProcess);
    }else released=true;
    if(job)CloseHandle(job);if(log!=INVALID_HANDLE_VALUE){FlushFileBuffers(log);CloseHandle(log);}
    if(ring&&released&&!drain_ring()&&!failure)failure="FINAL_RING_DRAIN_FAILED";
    if(ring&&released&&external_consumer){
        const auto final_drain=now();
        while(gpenmpc_clock_ring::load64(&ring->header.published_write)!=gpenmpc_clock_ring::load64(&ring->header.consumed_read)&&seconds(final_drain)<1.0)Sleep(1);
    }
    SOCKET a=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);SOCKET b=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    port_released=exclusive_bind(a,udp_port)&&exclusive_bind(b,udp_port+1);closesocket(a);closesocket(b);
    a=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);port_released=exclusive_bind(a,px4_udp_port)&&port_released;closesocket(a);
    FILE*f=_wfopen((out+L"\\RAW_STREAMS.bin").c_str(),L"wb");if(!f)return 5;
    for(const auto&r:raw){std::uint32_t n=static_cast<std::uint32_t>(r.bytes.size());fwrite(&r.qpc,8,1,f);fwrite(&r.channel,4,1,f);fwrite(&n,4,1,f);fwrite(r.bytes.data(),1,n,f);}fclose(f);
    f=_wfopen((out+L"\\HIL_WIRE_TIME.csv").c_str(),L"wb");if(!f)return 6;fprintf(f,"host_qpc,wire_time_usec,wire_seq\n");for(const auto&r:times)fprintf(f,"%llu,%llu,%u\n",static_cast<unsigned long long>(r.qpc),static_cast<unsigned long long>(r.wire),r.seq);fclose(f);
    if(ring&&released){
        auto&h=ring->header;const auto errors_before=gpenmpc_clock_ring::load(&h.sticky_errors);
        const auto written=gpenmpc_clock_ring::load64(&h.published_write),consumed=gpenmpc_clock_ring::load64(&h.consumed_read);
        const auto events=gpenmpc_clock_ring::load64(&h.events),drops=gpenmpc_clock_ring::load64(&h.dropped);
        const auto producer_state=gpenmpc_clock_ring::load(&h.state),writer_busy=gpenmpc_clock_ring::load(&h.writer_busy);
        const bool owner_matches=h.producer_pid==pi.dwProcessId;
        const bool capture_complete=owner_matches&&errors_before==0&&writer_busy==0&&written>0&&written==consumed&&
            (external_consumer||consumed==static_cast<LONG64>(ring_records.size()))&&events==written&&drops==0;
        f=_wfopen((out+L"\\GETTER_RING_RECORDS.bin").c_str(),L"wb");if(!f)return 14;
        fwrite(ring_records.data(),sizeof(gpenmpc_clock_ring::Sample),ring_records.size(),f);fclose(f);
        f=_wfopen((out+L"\\GETTER_RING_READ_QPC.bin").c_str(),L"wb");if(!f)return 15;
        fwrite(ring_read_qpc.data(),8,ring_read_qpc.size(),f);fclose(f);
        if(!external_consumer)ring_consumer.retire();const auto errors_after=gpenmpc_clock_ring::load(&h.sticky_errors);
        f=_wfopen((out+L"\\GETTER_RING_SECTION_FINAL.bin").c_str(),L"wb");if(!f)return 16;fwrite(ring,1,sizeof(*ring),f);fclose(f);
        f=_wfopen((out+L"\\GETTER_RING_OBSERVATION.json").c_str(),L"wb");if(!f)return 17;
        fprintf(f,"{\"scope\":\"Bounded diagnostic ring capture\",\"complete_capture\":%s,\"producer_matches\":%s,\"producer_state\":%ld,\"writer_busy_after_owned_child_exit\":%ld,\"events\":%lld,\"published\":%lld,\"consumed\":%lld,\"retained\":%zu,\"dropped\":%lld,\"errors_before_consumer_retirement\":%ld,\"errors_after_normal_consumer_retirement\":%ld,\"record_bytes\":%zu,\"engineering_ring_capacity\":%u,\"engineering_capture_capacity\":%zu,\"model_step_getter_atomicity_proven\":false,\"COM_open\":0,\"board_actions\":0}\n",capture_complete?"true":"false",owner_matches?"true":"false",producer_state,writer_busy,events,written,consumed,ring_records.size(),drops,errors_before,errors_after,sizeof(gpenmpc_clock_ring::Sample),gpenmpc_clock_ring::capacity,maximum_retained_getters);fclose(f);
        if(!capture_complete&&!failure)failure="RING_CAPTURE_INCOMPLETE";
    }
    if(snapshot&&released){
        gpenmpc_snapshot_status::View v{};const bool shape=gpenmpc_snapshot_status::read_atomic(*snapshot,v);
        const bool owner=shape&&v.producer_pid==pi.dwProcessId&&v.peer_nonce==ring_nonce&&ring&&ring->header.producer_pid==pi.dwProcessId;
        const bool getter_accounting=owner&&v.counters[5]==v.counters[6]+v.counters[7]+v.counters[8]&&
            v.counters[6]==static_cast<std::uint64_t>(gpenmpc_clock_ring::load64(&ring->header.events));
        const bool mirrors_complete=owner&&v.mirror_calls==v.mirror_completions&&v.mirror_contentions==0&&v.sticky_errors==0;
        // Official loading/configuration may initialize repeatedly BEFORE any
        // original step/getter. The DLL checks that condition at every entry;
        // first_snapshot_fault remains sticky after any run-time reinitialize.
        const bool observed_complete=owner&&getter_accounting&&mirrors_complete&&v.counters[0]==1&&v.counters[2]>=1&&
            v.counters[3]>0&&v.counters[3]==v.counters[4]&&v.counters[5]>0&&v.counters[8]==0&&v.counters[9]==0&&
            snapshot_runtime_first_fault==0&&snapshot_runtime_status_errors==0&&v.counters[1]==0;
        f=_wfopen((out+L"\\STEP_SNAPSHOT_STATUS_FINAL.bin").c_str(),L"wb");if(!f)return 18;fwrite(snapshot,1,sizeof(*snapshot),f);fclose(f);
        f=_wfopen((out+L"\\STEP_SNAPSHOT_OBSERVATION.json").c_str(),L"wb");if(!f)return 19;
        fprintf(f,"{\"scope\":\"Official NoUI step/getter observer with an independent total-getter count. Owned-process exit establishes quiescence.\",\"owned_process_exit_established\":%s,\"owner_nonce_shape_match\":%s,\"observation_complete\":%s,\"total_getter_accounting_complete\":%s,\"all_status_mirrors_completed\":%s,\"status_state\":%ld,\"runtime_status_reads\":%u,\"runtime_first_snapshot_fault\":%llu,\"runtime_status_errors\":%llu,\"mirror_calls\":%llu,\"mirror_completions\":%llu,\"mirror_contentions\":%llu,\"sticky_status_errors\":%ld,\"counters\":[",
            released?"true":"false",owner?"true":"false",observed_complete?"true":"false",getter_accounting?"true":"false",mirrors_complete?"true":"false",v.state,snapshot_runtime_reads,
            static_cast<unsigned long long>(snapshot_runtime_first_fault),static_cast<unsigned long long>(snapshot_runtime_status_errors),static_cast<unsigned long long>(v.mirror_calls),static_cast<unsigned long long>(v.mirror_completions),static_cast<unsigned long long>(v.mirror_contentions),v.sticky_errors);
        for(unsigned k=0;k<gpenmpc_snapshot_status::counter_count;++k)fprintf(f,"%s%llu",k?",":"",static_cast<unsigned long long>(v.counters[k]));
        fprintf(f,"],\"counter_order\":[\"version\",\"first_snapshot_fault\",\"initializations\",\"step_entries\",\"completed_snapshots\",\"getter_entries\",\"matches\",\"before_first_step\",\"rejected\",\"local_tap_contention\"],\"COM_open\":0,\"PX4_process\":0,\"board_actions\":0,\"true_receiver_HRT_association_proven\":false}\n");fclose(f);
        if(!observed_complete&&!failure)failure="STEP_SNAPSHOT_OBSERVATION_INCOMPLETE";
    }
    if(snapshot)UnmapViewOfFile(snapshot);if(snapshot_handle)CloseHandle(snapshot_handle);
    if(ring)UnmapViewOfFile(ring);if(ring_handle)CloseHandle(ring_handle);
    LONG tap_count=0,tap_state=0,tap_errors=0;LONG64 tap_events=0,tap_drops=0;bool tap_owner=false;unsigned exact_integer_time_matches=0;
    if(section&&released){
        tap_count=gpenmpc_clock_tap::load_long(&section->header.published_count);tap_state=gpenmpc_clock_tap::load_long(&section->header.state);
        tap_errors=gpenmpc_clock_tap::load_long(&section->header.sticky_errors);tap_events=gpenmpc_clock_tap::load_long64(&section->header.events);tap_drops=gpenmpc_clock_tap::load_long64(&section->header.dropped);tap_owner=section->header.producer_pid==pi.dwProcessId;
        f=_wfopen((out+L"\\DLL_SHARED_SECTION.bin").c_str(),L"wb");if(!f)return 10;fwrite(section,1,sizeof(*section),f);fclose(f);
        f=_wfopen((out+L"\\ACTUAL_DLL_GETTER.csv").c_str(),L"wb");if(!f)return 11;
        fprintf(f,"ordinal,generation,session,original_hil_double_usec,accepted_time_s,lag0,lag1,lag2,lag3,lag4,lag5,thread,valid,failed,runtime_error,pair_scope\n");
        if(tap_count<0||tap_count>32){if(!failure)failure="DIAGNOSTIC_COUNT_INVALID";}else for(LONG k=0;k<tap_count;++k){const auto&r=section->records[k];
            fprintf(f,"%llu,%llu,%llu,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%u,%u,%u,%u,%u\n",static_cast<unsigned long long>(r.event_ordinal),static_cast<unsigned long long>(r.accepted_generation),static_cast<unsigned long long>(r.plant_session),r.hil_output30[0],r.accepted_time_s,r.rotor_lag6_n[0],r.rotor_lag6_n[1],r.rotor_lag6_n[2],r.rotor_lag6_n[3],r.rotor_lag6_n[4],r.rotor_lag6_n[5],r.getter_thread_id,r.observation_valid,r.model_failed,r.model_runtime_error,static_cast<unsigned>(r.pair_scope));
            for(const auto&t:times){const double wire=static_cast<double>(t.wire);if(wire<=r.hil_output30[0]&&wire>=r.hil_output30[0])++exact_integer_time_matches;}
        }fclose(f);
    }
    if(section)UnmapViewOfFile(section);if(section_handle)CloseHandle(section_handle);
    f=_wfopen((out+L"\\GETTER_WIRE_OBSERVATION.json").c_str(),L"wb");if(!f)return 12;
    fprintf(f,"{\"diagnostic_section_requested\":%s,\"producer_pid_matches_owned_copter\":%s,\"published_records\":%ld,\"getter_events\":%lld,\"dropped_no_overwrite\":%lld,\"final_section_state\":%ld,\"sticky_diagnostic_errors\":%ld,\"exact_integer_getter_wire_pairs\":%u,\"CopterSim_getter_step_atomicity_proven\":false,\"board_receiver_clock_association_proven\":false,\"control_authority\":false}\n",model!=L"0"&&!ring_requested?"true":"false",tap_owner?"true":"false",tap_count,tap_events,tap_drops,tap_state,tap_errors,exact_integer_time_matches);fclose(f);
    f=_wfopen((out+L"\\VENDOR_FAKE_PEER_ACTIONS.json").c_str(),L"wb");if(!f)return 13;
    fprintf(f,"{\"peer_explicit_arm_mode_requests\":0,\"vendor_to_fake_peer_mode_requests\":%u,\"vendor_to_fake_peer_arm_disarm_requests\":%u,\"vendor_to_fake_peer_capability_requests\":%u,\"vendor_to_fake_peer_other_commands\":%u,\"physical_board_commands\":0,\"COM_open\":0,\"scope\":\"Official NoUI traffic to an isolated simulated UDP peer\"}\n",vendor_mode_requests,vendor_arm_requests,vendor_capability_requests,vendor_other_commands);fclose(f);
    f=_wfopen((out+L"\\NUMERICAL_RESULT.json").c_str(),L"wb");if(!f)return 7;
    fprintf(f,"{\"scope\":\"Official NoUI loopback TCP and PX4-UDP software carrier with a simulated disarmed peer; model identity is recorded in the parent receipt\",\"failure\":%s%s%s,\"owned_pid\":%lu,\"child_exit\":%lu,\"tcp_connected\":%s,\"host_to_tcp_exact_tunnel\":%u,\"tcp_to_host_exact_tunnel\":%u,\"host_to_px4_udp_exact_tunnel\":%u,\"host_to_px4_udp_exact_ping\":%u,\"crc_negative_control\":%u,\"crc_positive_control\":%u,\"hil_sensor_messages\":%u,\"hil_gps_messages\":%u,\"bad_frames\":%u,\"first_hil_time_usec\":%llu,\"last_hil_time_usec\":%llu,\"hil_time_strictly_increasing\":%s,\"zero_actuator_inputs_sent\":%u,\"owned_job_termination_used\":%s,\"owned_process_released\":%s,\"udp_ports_released\":%s,\"raw_capture_bytes\":%zu,\"arm_mode_commands\":0,\"nonzero_actuator_inputs\":0,\"COM_requested\":0,\"PX4_processes_launched\":0,\"custom_model_clock_conversion_proven\":false}\n",
        failure?"\"":"",failure?failure:"null",failure?"\"":"",pi.dwProcessId,child_exit,connected?"true":"false",tcp_tunnel,udp_tunnel,control_udp_tunnel,control_udp_ping,crc_negative,crc_positive,hil,gps,bad,static_cast<unsigned long long>(first_hil),static_cast<unsigned long long>(last_hil),strictly_monotone?"true":"false",zero_inputs,terminated?"true":"false",released?"true":"false",port_released?"true":"false",raw_bytes);fclose(f);WSACleanup();
    return failure||!released||!port_released?8:0;
}
