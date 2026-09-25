// Single-owner loopback UDP transport.
// GPENMPC_UDP_CONTINUOUS_GP enables continuous receive and GP256 replies.
// open(cfg): scope/allow_loopback/local_host/remote_host/local_port/remote_port.
// send(h,uint8[1..300]) returns a receipt; receive(h,1..64) returns datagram records.
// Retain runtime errors and the receive batch before throwing; only status/close remain available.
// close(h) is idempotent; status persists until the next open.
// Convert QPC to uint64 ns with quotient/remainder arithmetic.
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#ifdef GPENMPC_UDP_CONTINUOUS_GP
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include "canonical_gp_standalone_api.h"
#include "../rfly_vendor_integration/px4_wire/CanonicalLocalGpWire.hpp"
#include "../host_runtime/native_include/mavlink/common/mavlink.h"
#endif
#ifndef GPENMPC_UDP_TRANSPORT_NO_SOCKET_TEST
#include "mex.h"
#else
#include <cstdio>
#endif

// Initialize the address buffer and check getnameinfo's return value.
// NI_NUMERICHOST disables DNS lookup.
static int sdk_name_error=0;
static int checked_sdk_getnameinfo(const sockaddr* a,socklen_t n,char* host,
                                  DWORD hn,char* service,DWORD sn,int flags) {
    if(host&&hn)std::memset(host,0,hn);
    const int rc=::getnameinfo(a,n,host,hn,service,sn,flags);
    if(rc!=0)sdk_name_error=rc;
    return rc;
}
#define getnameinfo checked_sdk_getnameinfo
#include "gpenmpc_rfly_udp_sdk_bound.hpp"
#undef getnameinfo

namespace rtudp {
using U64=std::uint64_t;
constexpr unsigned MAX_BYTES=300,DIAGNOSTIC_BYTES=301,MAX_BATCH=64;
constexpr const char* SCOPE="HOST_ONLY_LOOPBACK";
constexpr const char* COPTERSIM_SCOPE="COPTERSIM_EXISTING_MAVLINK";
constexpr const char* SDK_SHA="BD96B77CA8F771BA28E1B37967D36BCE02C8F350822B5D930260E322B7E5352D";
struct Error:std::runtime_error { std::string code; Error(const char* c,const char* m):std::runtime_error(m),code(c){} };
struct Config { std::string scope,local_host,remote_host;bool allow_loopback{};unsigned local_port{},remote_port{}; };
bool valid_port(unsigned p){return p>=62200&&p<=62399;}
bool valid_config(const Config& c){
    if(!c.allow_loopback||c.local_host!="127.0.0.1"||c.remote_host!="127.0.0.1")return false;
    if(c.scope==SCOPE)return valid_port(c.local_port)&&valid_port(c.remote_port)&&c.local_port!=c.remote_port;
    // Existing CopterSim socket is the remote peer, NEVER a second local owner.
    return c.scope==COPTERSIM_SCOPE&&c.local_port==14550&&c.remote_port==18570;
}
bool qpc_ns(U64 ticks,U64 f,U64& result) {
    constexpr U64 billion=1000000000ULL,top=std::numeric_limits<U64>::max();
    if(!ticks||!f)return false;
    const U64 sec=ticks/f,rem=ticks%f;
    if(sec>top/billion||rem>top/billion)return false;
    const U64 whole=sec*billion,frac=rem*billion/f;
    if(whole>top-frac)return false;
    result=whole+frac;return true;
}
struct Stamp {U64 ticks{},ns{};};
struct SendRecord {bool attempted{},ok{},bytes_complete{};std::array<unsigned char,DIAGNOSTIC_BYTES> bytes{};
    unsigned stored{};U64 requested{};int sent{-1},socket_error{};Stamp before{},after{};std::string error;};
#ifdef GPENMPC_UDP_CONTINUOUS_GP
struct GpCompleted {
    gpenmpc_local_gp_wire::RequestBytes request{};
    gpenmpc_local_gp_wire::ReplyBytes reply{};
    double result[18]{};Stamp processing{};
    U64 rx[3]{};unsigned lengths[3]{};
    std::array<unsigned char,MAX_BYTES> frames[3]{};
    SendRecord sends[3]{};unsigned sent{};
};
#endif
struct ReceiveRecord {bool attempted{},ok{},bytes_complete{},source_matched{},would_block{};
    std::array<unsigned char,DIAGNOSTIC_BYTES> bytes{};unsigned stored{};int received{-1},socket_error{};
    std::string source,error;unsigned port{};Stamp before{},after{};
#ifdef GPENMPC_UDP_CONTINUOUS_GP
    std::shared_ptr<GpCompleted> gp;
    bool gp_history_only{};
    // Parse a complete TUNNEL datagram with the SDK; keep other frames on the MATLAB path.
    bool tunnel_valid{};
    std::array<unsigned char,MAVLINK_MSG_ID_TUNNEL_LEN> tunnel_payload{};
#endif
};
struct ReceiveBatch {unsigned requested{},attempts{},valid{};bool cap_reached{},returned{},ok{};
    std::array<ReceiveRecord,MAX_BATCH> records{};};
struct CloseRecord {bool attempted{},already_closed{},returned{},ok{};int socket_error{};Stamp before{},after{};std::string error;};
struct Owner {
    RflySdkRawMethods sdk{};Config config{};U64 handle{},frequency{},last_ns{};
    U64 send_attempts{},receive_attempts{},sent_count{},received_count{},close_calls{};
    bool opened{},closed{true},failed{},wsa_started{},bind_ok{},exclusive{},nonblocking{};
    int recv_buffer_bytes{},send_buffer_bytes{},open_socket_error{};std::string failure_code,first_error;
    SendRecord last_send{};ReceiveBatch last_receive{};CloseRecord last_close{};
    void latch(const char* code,const char* message) {if(!failed){failed=true;failure_code=code;first_error=message;}}
    [[noreturn]] void bad(const char* code,const char* message){latch(code,message);throw Error(code,message);}
    Stamp stamp() {
        LARGE_INTEGER q{};Stamp s{};
        if(!QueryPerformanceCounter(&q)||q.QuadPart<=0||!qpc_ns(U64(q.QuadPart),frequency,s.ns)||s.ns<last_ns)
            bad("Clock","QPC is unavailable, overflowing, or regressed; no substitute clock.");
        s.ticks=U64(q.QuadPart);last_ns=s.ns;return s;
    }
    void healthy(){if(!opened||closed||failed)bad("State","Transport is not an open healthy owner; only status/close are allowed.");}
    void check_handle(U64 h){if(!handle||h!=handle)bad("Handle","Exact current uint64 owner handle required.");}
    void release_noexcept() noexcept {
        if(sdk.sock!=INVALID_SOCKET){::closesocket(sdk.sock);sdk.sock=INVALID_SOCKET;}
        if(wsa_started){::WSACleanup();wsa_started=false;}
        opened=false;closed=true;
    }
    void open(const Config& c,U64 token) {
        if(opened||wsa_started)bad("SingleOwner","An existing owner must be closed before open.");
        if(!valid_config(c))throw Error("Scope","Only explicit HOST test ports or COPTERSIM_EXISTING_MAVLINK local14550/remote18570 on 127.0.0.1 are allowed.");
        *this=Owner{};config=c;handle=token;closed=false;
        try {
            LARGE_INTEGER f{};
            if(!QueryPerformanceFrequency(&f)||f.QuadPart<=0)bad("Clock","QPC frequency unavailable.");
            frequency=U64(f.QuadPart);stamp();
            WSADATA wd{};const int wr=WSAStartup(MAKEWORD(2,2),&wd);
            if(wr!=0){open_socket_error=wr;bad("Startup","WSAStartup failed; no fallback.");}
            wsa_started=true;
            if(wd.wVersion!=MAKEWORD(2,2))bad("Startup","Winsock 2.2 was not established.");
            sdk.sock=::socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
            if(sdk.sock==INVALID_SOCKET)bad("Socket","UDP socket creation failed.");
            BOOL ex=TRUE;
            if(setsockopt(sdk.sock,SOL_SOCKET,SO_EXCLUSIVEADDRUSE,reinterpret_cast<const char*>(&ex),sizeof(ex))!=0)
                bad("Exclusive","SO_EXCLUSIVEADDRUSE failed; socket sharing is forbidden.");
            int exn=sizeof(ex);ex=FALSE;
            if(getsockopt(sdk.sock,SOL_SOCKET,SO_EXCLUSIVEADDRUSE,reinterpret_cast<char*>(&ex),&exn)!=0||!ex)
                bad("Exclusive","Exclusive binding option was not confirmed.");
            exclusive=true;
            sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(static_cast<u_short>(c.local_port));
            a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
            if(::bind(sdk.sock,reinterpret_cast<const sockaddr*>(&a),sizeof(a))!=0)bad("Bind","Exclusive loopback bind failed.");
            bind_ok=true;
            u_long one=1;if(ioctlsocket(sdk.sock,FIONBIO,&one)!=0)bad("Nonblocking","Nonblocking setup failed.");
            nonblocking=true;sdk.hasSetNoblock=true; // Skip the original unchecked setup branch.
            int n=sizeof(int);
            if(getsockopt(sdk.sock,SOL_SOCKET,SO_RCVBUF,reinterpret_cast<char*>(&recv_buffer_bytes),&n)!=0)
                bad("SocketStatus","Cannot observe actual receive buffer size.");
            n=sizeof(int);
            if(getsockopt(sdk.sock,SOL_SOCKET,SO_SNDBUF,reinterpret_cast<char*>(&send_buffer_bytes),&n)!=0)
                bad("SocketStatus","Cannot observe actual send buffer size.");
            opened=true;closed=false;
        } catch(...) {
            if(!open_socket_error)open_socket_error=WSAGetLastError();
            // Retain the close result, including cleanup failures.
            try{close();}catch(...){}throw;}
    }
    void prepare_send(const unsigned char* p,U64 n,bool complete_type=true) {
        healthy();last_send=SendRecord{};last_send.requested=n;
        last_send.stored=complete_type?static_cast<unsigned>(std::min<U64>(n,DIAGNOSTIC_BYTES)):0;
        if(last_send.stored)std::memcpy(last_send.bytes.data(),p,last_send.stored);
        last_send.bytes_complete=complete_type&&n<=DIAGNOSTIC_BYTES;
        if(!complete_type||!n||n>MAX_BYTES){last_send.error="SendShape";bad("SendShape","Send requires a real full uint8 vector with 1..300 bytes; no datagram sent.");}
    }
    void finish_send(int n,int socket_error) {
        last_send.sent=n;last_send.socket_error=socket_error;
        if(n<0){last_send.error="SendSocket";bad("SendSocket","Actual SDK SendTo failed; inspect retained last_send.");}
        if(U64(n)!=last_send.requested){last_send.error="PartialSend";bad("PartialSend","Actual send length differs from requested bytes; no retry.");}
        last_send.ok=true;++sent_count;
    }
    void send_prepared() {
        last_send.before=stamp();last_send.attempted=true;++send_attempts;
        int error=0;WSASetLastError(0);
        const int n=sdk.SendTo(reinterpret_cast<const char*>(last_send.bytes.data()),last_send.stored,
            "127.0.0.1",static_cast<std::uint16_t>(config.remote_port),
            [&error](int,std::string){error=WSAGetLastError();});
        if(n<0&&!error)error=WSAGetLastError();
        last_send.sent=n;last_send.socket_error=error; // Persist outcome even if post-clock fails.
        try {last_send.after=stamp();}catch(...){last_send.error="Clock";throw;}
        finish_send(n,error);
    }
    // Share classification with standalone negative tests.
    void finish_receive(ReceiveRecord& r) {
        if(r.received<0){
            if(r.socket_error==WSAEWOULDBLOCK){r.would_block=true;return;}
            r.error=r.socket_error==WSAEMSGSIZE?"OversizeReceive":"ReceiveSocket";
            bad(r.error.c_str(),"Actual SDK RecvNoblock failed or datagram exceeds the bound; retained batch includes the fault.");
        }
        if(r.received==0||r.received>int(MAX_BYTES)){
            r.error="ReceiveLength";bad("ReceiveLength","Zero-length or >300-byte datagram rejected; no parsing or fallback.");}
        if(!r.source_matched){r.error="ReceiveSource";bad("ReceiveSource","Datagram source differs from the sole configured loopback peer.");}
        r.ok=true;++received_count;++last_receive.valid;
    }
    void receive(unsigned count) {
        healthy();last_receive=ReceiveBatch{};last_receive.requested=count;
        if(!count||count>MAX_BATCH)bad("ReceiveCount","Receive max_count must be an integer in 1..64.");
        for(unsigned k=0;k<count;++k){
            auto& r=last_receive.records[k];char buf[DIAGNOSTIC_BYTES+1]{};std::string ip;int port=0;
            r.before=stamp();r.attempted=true;++receive_attempts;++last_receive.attempts;
            WSASetLastError(0);sdk_name_error=0;
            const int n=sdk.RecvNoblock(buf,ip,port,DIAGNOSTIC_BYTES);
            const int error=n<0?WSAGetLastError():sdk_name_error;
            r.received=n;r.socket_error=error;r.source=ip;r.port=static_cast<unsigned>(port);
            r.source_matched=(ip=="127.0.0.1"&&port==static_cast<int>(config.remote_port));
            r.stored=n>=0?static_cast<unsigned>(n):(error==WSAEMSGSIZE?DIAGNOSTIC_BYTES:0);
            r.bytes_complete=n>=0; // WSAEMSGSIZE means original datagram length is UNKNOWN.
            if(r.stored)std::memcpy(r.bytes.data(),buf,r.stored);
            try{r.after=stamp();}catch(...){r.error="Clock";throw;}
            if(n>=0&&error){r.error="ReceiveAddress";bad("ReceiveAddress","Actual numeric source address conversion failed.");}
            finish_receive(r);
            if(r.would_block){last_receive.returned=true;last_receive.ok=true;return;}
        }
        last_receive.cap_reached=true;last_receive.returned=true;last_receive.ok=true;
        // A full receive batch indicates the caller capacity was reached.
    }
    void close() {
        ++close_calls;last_close=CloseRecord{};
        last_close.already_closed=closed&&!wsa_started&&sdk.sock==INVALID_SOCKET;
        if(last_close.already_closed){last_close.returned=true;last_close.ok=true;return;}
        // Continue cleanup after a clock fault and retain the close result.
        try{last_close.before=stamp();}catch(...){last_close.error="Clock";}
        last_close.attempted=true;int error=0;
        if(sdk.sock!=INVALID_SOCKET){
            if(::closesocket(sdk.sock)==0)sdk.sock=INVALID_SOCKET;
            else error=WSAGetLastError();
        }
        if(sdk.sock==INVALID_SOCKET&&wsa_started){
            if(WSACleanup()==0)wsa_started=false;else if(!error)error=WSAGetLastError();
        }
        last_close.socket_error=error;opened=(sdk.sock!=INVALID_SOCKET);closed=!opened&&!wsa_started;
        try{last_close.after=stamp();}catch(...){last_close.error="Clock";}
        last_close.returned=true;last_close.ok=closed&&error==0&&last_close.error.empty();
        if(error||!closed){last_close.error="CloseSocket";bad("CloseSocket","Actual cleanup failed; status retained and close may be retried.");}
    }
};
#ifndef GPENMPC_UDP_TRANSPORT_NO_SOCKET_TEST
static Owner owner;static U64 generation=0;
#endif
#if defined(GPENMPC_UDP_CONTINUOUS_GP) && !defined(GPENMPC_UDP_TRANSPORT_NO_SOCKET_TEST)
// Use one socket and GP library on the receive thread.
// Queue live state for the caller and retain processed GP history until close.
// Share one transmit sequence under the mutex.
namespace gw=gpenmpc_local_gp_wire;
std::mutex owner_mutex;
struct ContinuousGp {
    std::thread worker;std::atomic<bool> stop{false};bool active{};U64 registered_handle{};
    std::condition_variable space_available;
    std::deque<ReceiveRecord> queued;
    std::deque<ReceiveRecord> deferred_history;
    U64 history_limit{};
    U64 uid{},boot{},confirmed{},last_output{},last_source{},last_sample{};
    bool prediction_enabled{true};
    unsigned char ids[4]{};mavlink_status_t tx_status{};
    U64 calls{},completed{},replies{},worker_receives{};
    GpCompleted partial{};unsigned next{};U64 partial_generation{};
    std::shared_ptr<GpCompleted> last_result;
    static bool parse(const unsigned char*b,unsigned n,mavlink_message_t&m) {
        if(n<12||b[0]!=253||b[2]!=0||unsigned(b[1])+12!=n)return false;
        mavlink_message_t buffer{};mavlink_status_t parser{},status{};unsigned valid=0;
        for(unsigned k=0;k<n;++k)valid=mavlink_frame_char_buffer(&buffer,&parser,b[k],&m,&status);
        return valid==MAVLINK_FRAMING_OK;
    }
    void resequence() {
        mavlink_message_t m{};
        if(!parse(owner.last_send.bytes.data(),owner.last_send.stored,m))
            owner.bad("GpTxFrame","Continuous owner requires one CRC-valid unsigned MAVLink2 frame.");
        if(m.sysid!=ids[2]||m.compid!=ids[3])owner.bad("GpTxSource","Original sender changed.");
        const auto*entry=mavlink_get_msg_entry(m.msgid);
        if(!entry)owner.bad("GpTxFrame","Unknown original outgoing message.");
        const auto n=owner.last_send.stored;
        mavlink_finalize_message_buffer(&m,m.sysid,m.compid,&tx_status,entry->min_msg_len,m.len,entry->crc_extra);
        unsigned char wire[MAX_BYTES]{};const auto size=mavlink_msg_to_send_buffer(wire,&m);
        if(size!=n)owner.bad("GpTxFrame","Sequence assignment changed payload/frame length.");
        // Update sequence and CRC only.
        for(unsigned k=0;k<n;++k)if(k!=4&&k<n-2&&wire[k]!=owner.last_send.bytes[k])
            owner.bad("GpTxFrame","Sequence assignment changed scientific or command payload.");
        std::memcpy(owner.last_send.bytes.data(),wire,n);
    }
    void consume(ReceiveRecord&r) {
        const auto*b=r.bytes.data();unsigned offset=0;
        while(offset+12<=r.stored){
            if(b[offset]!=253||b[offset+2]!=0)break;
            const unsigned n=unsigned(b[offset+1])+12;if(offset+n>r.stored)break;
            mavlink_message_t m{};if(!parse(b+offset,n,m))break;
            if(m.msgid==MAVLINK_MSG_ID_TUNNEL){
                if(offset==0&&n==r.stored&&m.len<=r.tunnel_payload.size()){
                    std::memcpy(r.tunnel_payload.data(),_MAV_PAYLOAD(&m),m.len);
                    r.tunnel_valid=true;
                }
                mavlink_tunnel_t t{};mavlink_msg_tunnel_decode(&m,&t);
                if(t.payload_type==42002&&(t.payload[0]>>4)==gw::request_schema){
                    if(!prediction_enabled)owner.bad("ReceiveOnlyGpRequest","GP request is not enabled in this RC receive-only session.");
                    if(m.sysid!=ids[0]||m.compid!=ids[1]||t.target_system!=ids[2]||t.target_component!=ids[3])
                        owner.bad("GpSource","Original GP address mismatch.");
                    const unsigned index=t.payload[0]&15;const unsigned count=index<3?(index==2?81:128):0;
                    U64 gen=0;for(unsigned k=1;k<9;++k)gen=(gen<<8)|t.payload[k];
                    if(!gen||!count||t.payload_length!=count||index!=next||(next&&gen!=partial_generation))
                        owner.bad("GpOrder","Original three-fragment order/shape rejected.");
                    for(unsigned k=count;k<128;++k)if(t.payload[k])
                        owner.bad("GpOrder","Original GP padding is not zero.");
                    if(!index){partial=GpCompleted{};partial_generation=gen;}
                    if(r.after.ns<confirmed||(next&&(r.after.ns<partial.rx[index-1]||r.after.ns-partial.rx[0]>50000000ULL)))
                        owner.bad("GpClock","Original receive/50ms assembly bound rejected.");
                    partial.rx[index]=r.after.ns;partial.lengths[index]=n;
                    std::memcpy(partial.frames[index].data(),b+offset,n);
                    std::memcpy(partial.request.data()+119*index,t.payload+9,count-9);++next;
                    if(next==3){
                        gw::Request q{};
                        if(!gw::decode(partial.request,q)||q.output_generation!=gen||q.identity.uid!=uid||q.identity.boot_generation!=boot||
                           q.identity.system!=ids[0]||q.identity.component!=ids[1]||q.output_generation<=last_output||
                           q.source_generation<=last_source||q.source_timestamp_ns<=last_sample)
                            owner.bad("GpIdentity","Original body/model/source/replay identity rejected before inference.");
                        auto done=std::make_shared<GpCompleted>(partial);r.gp=done;last_result=done;
                        done->processing=owner.stamp();
                        if(done->processing.ns<done->rx[0]||done->processing.ns-done->rx[0]>50000000ULL)
                            owner.bad("GpClock","No prediction from an expired retained receive.");
                        ++calls;
                        if(gpenmpc_gp256_predict(q.request19+1,done->result)!=GPENMPC_GP256_OK)
                            owner.bad("GpNumeric","The original GP256 call failed.");
                        ++completed;gw::Reply answer{};answer.identity=q.identity;answer.source_timestamp_ns=q.source_timestamp_ns;
                        answer.source_generation=q.source_generation;answer.output_generation=q.output_generation;
                        answer.original_request_sha256=gw::digest(done->request.data(),done->request.size());
                        answer.gp_model_sha256=q.gp_model_sha256;std::memcpy(answer.result18,done->result,sizeof(done->result));
                        if(!gw::encode(answer,done->reply))owner.bad("GpNumeric","Original GP reply encoding rejected.");
                        last_output=q.output_generation;last_source=q.source_generation;last_sample=q.source_timestamp_ns;
                        for(unsigned part=0;part<3;++part){
                            const auto now=owner.stamp();if(now.ns-done->rx[0]>200000000ULL)
                                owner.bad("GpSendAge","Original 200ms transport age exceeded; no retimestamp.");
                            mavlink_tunnel_t tx{};tx.target_system=ids[0];tx.target_component=ids[1];tx.payload_type=42002;
                            const unsigned bytes=std::min<unsigned>(119,gw::reply_bytes-119*part);tx.payload_length=9+bytes;
                            tx.payload[0]=static_cast<unsigned char>(144+part);
                            std::memcpy(tx.payload+1,done->request.data()+38,8);
                            std::memcpy(tx.payload+9,done->reply.data()+119*part,bytes);
                            mavlink_message_t message{};
                            mavlink_msg_tunnel_encode_status(ids[2],ids[3],&tx_status,&message,&tx);
                            unsigned char wire[MAX_BYTES]{};const auto length=mavlink_msg_to_send_buffer(wire,&message);
                            owner.prepare_send(wire,length);owner.send_prepared();done->sends[part]=owner.last_send;++done->sent;
                            if(owner.last_send.after.ns-done->rx[0]>200000000ULL)owner.bad("GpSendAge","Original send returned late.");
                        }
                        ++replies;next=0;partial=GpCompleted{};
                    }
                    // Defer MATLAB history decoding only for a complete GP datagram that passed
                    // CRC, source and ordering checks. Preserve its raw bytes.
                    r.gp_history_only=offset==0&&n==r.stored;
                }
            }
            offset+=n;
        }
    }
    void start(U64 u,U64 boot_id,U64 original_confirmed,const unsigned char*addresses,U64 raw_history_limit=0,bool receive_only=false) {
        if(active||worker.joinable()||registered_handle==owner.handle||!u||!boot_id||!original_confirmed)owner.bad("GpStart","One exact registered GP owner required.");
        registered_handle=owner.handle;prediction_enabled=!receive_only;
        history_limit=raw_history_limit;deferred_history.clear();
        uid=u;boot=boot_id;confirmed=original_confirmed;std::memcpy(ids,addresses,4);
        last_output=last_source=last_sample=calls=completed=replies=worker_receives=0;next=0;partial={};queued.clear();tx_status={};last_result.reset();
        if(owner.last_send.ok&&owner.last_send.stored>=12&&owner.last_send.bytes[0]==253)
            tx_status.current_tx_seq=static_cast<unsigned char>(owner.last_send.bytes[4]+1);
        stop=false;active=true;
        worker=std::thread([this]{
            while(!stop.load()){
                fd_set fds;FD_ZERO(&fds);FD_SET(owner.sdk.sock,&fds);timeval wait{0,10000};
                const int ready=::select(0,&fds,nullptr,nullptr,&wait);
                if(stop.load())break;
                std::unique_lock<std::mutex> lock(owner_mutex);
                try{
                    if(ready<0)owner.bad("GpReceiveWait","Existing socket readiness failed.");
                    if(!ready)continue;
                    // Apply backpressure when the caller batch is full, preserving queued records.
                    if(queued.size()>=MAX_BATCH){
                        space_available.wait(lock,[this]{return stop.load()||queued.size()<MAX_BATCH;});
                        continue;
                    }
                    owner.receive(MAX_BATCH-static_cast<unsigned>(queued.size()));++worker_receives;
                    // Save original records before GP send can change last_send.
                    for(unsigned k=0;k<owner.last_receive.valid;++k){
                        queued.push_back(owner.last_receive.records[k]);consume(queued.back());
                        if(history_limit&&queued.back().gp_history_only){
                            // Defer MATLAB processing only for fully validated native GP records.
                            // Retain the current record and latch the session on error.
                            if(deferred_history.size()>=history_limit)
                                owner.bad("GpHistoryBound","Existing run raw-record budget exhausted; nothing evicted.");
                            deferred_history.push_back(std::move(queued.back()));queued.pop_back();
                        }
                    }
                }catch(const std::exception&e){owner.latch("GpContinuous",e.what());stop=true;}
                 catch(...){owner.latch("GpContinuous","Unexpected native GP execution error.");stop=true;}
            }
        });
    }
    void drain(unsigned count) {
        owner.healthy();if(!count||count>MAX_BATCH)owner.bad("ReceiveCount","Existing receive bound is 1..64.");
        owner.last_receive={};auto&b=owner.last_receive;b.requested=count;b.ok=b.returned=true;
        while(b.valid<count&&!queued.empty()){b.records[b.valid++]=queued.front();queued.pop_front();}
        b.attempts=b.valid;b.cap_reached=b.valid==count;
        space_available.notify_one();
    }
    void join(std::unique_lock<std::mutex>&lock) {
        stop=true;space_available.notify_one();lock.unlock();if(worker.joinable())worker.join();lock.lock();active=false;
    }
} continuous_gp;
#endif
}

#ifndef GPENMPC_UDP_TRANSPORT_NO_SOCKET_TEST
namespace {
using namespace rtudp;
bool mex_locked=false,exit_registered=false;
mxArray* u64(U64 n){auto*a=mxCreateNumericMatrix(1,1,mxUINT64_CLASS,mxREAL);*static_cast<U64*>(mxGetData(a))=n;return a;}
mxArray* i32(int n){auto*a=mxCreateNumericMatrix(1,1,mxINT32_CLASS,mxREAL);*static_cast<std::int32_t*>(mxGetData(a))=n;return a;}
mxArray* u32(unsigned n){auto*a=mxCreateNumericMatrix(1,1,mxUINT32_CLASS,mxREAL);*static_cast<std::uint32_t*>(mxGetData(a))=n;return a;}
mxArray* u16(unsigned n){auto*a=mxCreateNumericMatrix(1,1,mxUINT16_CLASS,mxREAL);*static_cast<std::uint16_t*>(mxGetData(a))=static_cast<std::uint16_t>(n);return a;}
void put(mxArray*a,const char*n,mxArray*v,mwIndex index=0){mxSetField(a,index,n,v);}
mxArray* bytes(const unsigned char*p,unsigned n){auto*a=mxCreateNumericMatrix(n,1,mxUINT8_CLASS,mxREAL);if(n)std::memcpy(mxGetData(a),p,n);return a;}
std::string text(const mxArray*a){if(!a||!mxIsChar(a)||mxGetM(a)>1)throw Error("Arguments","Character row vectors required.");
    char*p=mxArrayToString(a);if(!p)throw Error("Arguments","Character conversion failed.");std::string s(p);mxFree(p);return s;}
unsigned number(const mxArray*a){if(!a||!mxIsNumeric(a)||mxIsComplex(a)||mxIsSparse(a)||mxGetNumberOfElements(a)!=1)
    throw Error("Arguments","One real integer scalar required.");
    const double n=mxGetScalar(a);if(!std::isfinite(n)||n<0||n>65535||n!=std::floor(n))throw Error("Arguments","Integer scalar out of range.");return static_cast<unsigned>(n);}
U64 handle(const mxArray*a){
    if(!a||mxGetClassID(a)!=mxUINT64_CLASS||mxIsComplex(a)||mxGetNumberOfElements(a)!=1){
        throw Error("Handle","A scalar uint64 owner handle is required.");
    }
    return *static_cast<const U64*>(mxGetData(a));
}
Config config(const mxArray*a){const char* names[]={"scope","allow_loopback","local_host","remote_host","local_port","remote_port"};
    if(!a||!mxIsStruct(a)||mxGetNumberOfElements(a)!=1||mxGetNumberOfFields(a)!=6)throw Error("Scope","Exactly the six explicit HOST loopback config fields are required.");
    for(const auto*n:names)if(!mxGetField(a,0,n))throw Error("Scope","Missing explicit HOST-only config field.");
    auto*allow=mxGetField(a,0,"allow_loopback");if(!mxIsLogicalScalarTrue(allow))throw Error("Scope","allow_loopback must be scalar logical true.");
    return {text(mxGetField(a,0,"scope")),text(mxGetField(a,0,"local_host")),text(mxGetField(a,0,"remote_host")),true,
        number(mxGetField(a,0,"local_port")),number(mxGetField(a,0,"remote_port"))};}
mxArray* send_record(const SendRecord&r,U64 frequency=owner.frequency){const char*fields[]={"attempted","ok","bytes","bytes_complete","bytes_requested","bytes_sent",
    "submit_ns","return_ns","submit_qpc","return_qpc","qpc_frequency","socket_error","error_code"};
    auto*a=mxCreateStructMatrix(1,1,13,fields);put(a,"attempted",mxCreateLogicalScalar(r.attempted));put(a,"ok",mxCreateLogicalScalar(r.ok));
    put(a,"bytes",bytes(r.bytes.data(),r.stored));put(a,"bytes_complete",mxCreateLogicalScalar(r.bytes_complete));
    put(a,"bytes_requested",u64(r.requested));put(a,"bytes_sent",i32(r.sent));put(a,"submit_ns",u64(r.before.ns));put(a,"return_ns",u64(r.after.ns));
    put(a,"submit_qpc",u64(r.before.ticks));put(a,"return_qpc",u64(r.after.ticks));put(a,"qpc_frequency",u64(frequency));
    put(a,"socket_error",i32(r.socket_error));put(a,"error_code",mxCreateString(r.error.c_str()));return a;}
#ifdef GPENMPC_UDP_CONTINUOUS_GP
mxArray* gp_record(const std::shared_ptr<GpCompleted>&p,U64 frequency=owner.frequency){
    if(!p)return mxCreateDoubleMatrix(0,0,mxREAL);
    const char*names[]={"request_bytes","original_host_receive_ns","fragment_rx_ns","raw_frames","reply_bytes","result18","processing_ns","send"};
    auto*a=mxCreateStructMatrix(1,1,8,names);const auto&r=*p;
    put(a,"request_bytes",bytes(r.request.data(),r.request.size()));put(a,"reply_bytes",bytes(r.reply.data(),r.reply.size()));
    put(a,"original_host_receive_ns",u64(r.rx[0]));put(a,"processing_ns",u64(r.processing.ns));
    auto*rx=mxCreateNumericMatrix(3,1,mxUINT64_CLASS,mxREAL);std::memcpy(mxGetData(rx),r.rx,sizeof(r.rx));put(a,"fragment_rx_ns",rx);
    auto*frames=mxCreateCellMatrix(3,1);for(unsigned k=0;k<3;++k)mxSetCell(frames,k,bytes(r.frames[k].data(),r.lengths[k]));put(a,"raw_frames",frames);
    auto*y=mxCreateDoubleMatrix(1,18,mxREAL);std::memcpy(mxGetDoubles(y),r.result,sizeof(r.result));put(a,"result18",y);
    const char*sn[]={"messages_send_returned","submitted_ns","returned_ns","native","same_existing_transport_owner","board_receipt_proven","control_authority","status"};
    auto*s=mxCreateStructMatrix(1,1,8,sn);put(s,"messages_send_returned",mxCreateDoubleScalar(r.sent));
    auto*sub=mxCreateNumericMatrix(3,1,mxUINT64_CLASS,mxREAL);auto*end=mxCreateNumericMatrix(3,1,mxUINT64_CLASS,mxREAL);auto*n=mxCreateCellMatrix(1,3);
    for(unsigned k=0;k<3;++k){static_cast<U64*>(mxGetData(sub))[k]=r.sends[k].before.ns;static_cast<U64*>(mxGetData(end))[k]=r.sends[k].after.ns;mxSetCell(n,k,send_record(r.sends[k],frequency));}
    put(s,"submitted_ns",sub);put(s,"returned_ns",end);put(s,"native",n);put(s,"same_existing_transport_owner",mxCreateLogicalScalar(true));
    put(s,"board_receipt_proven",mxCreateLogicalScalar(false));put(s,"control_authority",mxCreateLogicalScalar(false));
    put(s,"status",mxCreateString("CONTINUOUS_SAME_OWNER_ORIGINAL_GP_REPLY"));put(a,"send",s);return a;
}
#endif
mxArray* receive_records(const ReceiveBatch&b,bool successes_only,U64 frequency=owner.frequency){
    const char*fields[]={"attempted","ok","bytes","bytes_complete","requested_capacity","received_bytes","source_ip","source_port",
        "source_matched","submit_ns","dequeue_ns","submit_qpc","dequeue_qpc","qpc_frequency","socket_error","error_code","empty_would_block"
#ifdef GPENMPC_UDP_CONTINUOUS_GP
        ,"native_gp","native_gp_history_only","native_tunnel_payload"
#endif
    };
    auto*a=mxCreateStructMatrix(successes_only?b.valid:b.attempts,1,sizeof(fields)/sizeof(*fields),fields);mwIndex at=0;
    for(unsigned j=0;j<b.attempts;++j){const auto&r=b.records[j];if(successes_only&&!r.ok)continue;
        put(a,"attempted",mxCreateLogicalScalar(r.attempted),at);put(a,"ok",mxCreateLogicalScalar(r.ok),at);put(a,"bytes",bytes(r.bytes.data(),r.stored),at);
        put(a,"bytes_complete",mxCreateLogicalScalar(r.bytes_complete),at);put(a,"requested_capacity",u32(DIAGNOSTIC_BYTES),at);
        put(a,"received_bytes",i32(r.received),at);put(a,"source_ip",mxCreateString(r.source.c_str()),at);put(a,"source_port",u16(r.port),at);
        put(a,"source_matched",mxCreateLogicalScalar(r.source_matched),at);put(a,"submit_ns",u64(r.before.ns),at);put(a,"dequeue_ns",u64(r.after.ns),at);
        put(a,"submit_qpc",u64(r.before.ticks),at);put(a,"dequeue_qpc",u64(r.after.ticks),at);put(a,"qpc_frequency",u64(frequency),at);
        put(a,"socket_error",i32(r.socket_error),at);put(a,"error_code",mxCreateString(r.error.c_str()),at);
        put(a,"empty_would_block",mxCreateLogicalScalar(r.would_block),at);
#ifdef GPENMPC_UDP_CONTINUOUS_GP
        put(a,"native_gp",gp_record(r.gp,frequency),at);
        put(a,"native_gp_history_only",mxCreateLogicalScalar(r.gp_history_only),at);
        put(a,"native_tunnel_payload",bytes(r.tunnel_payload.data(),r.tunnel_valid?r.tunnel_payload.size():0),at);
#endif
        ++at;}return a;}
mxArray* batch_record(){const auto&b=owner.last_receive;const char*fields[]={"requested_max_count","attempt_count","valid_count","cap_reached","returned","ok","records"};
    auto*a=mxCreateStructMatrix(1,1,7,fields);put(a,"requested_max_count",u32(b.requested));put(a,"attempt_count",u32(b.attempts));put(a,"valid_count",u32(b.valid));
    put(a,"cap_reached",mxCreateLogicalScalar(b.cap_reached));put(a,"returned",mxCreateLogicalScalar(b.returned));put(a,"ok",mxCreateLogicalScalar(b.ok));put(a,"records",receive_records(b,false));return a;}
mxArray* close_record(){const auto&r=owner.last_close;const char*fields[]={"attempted","already_closed","returned","ok","submit_ns","return_ns","socket_error","error_code"};
    auto*a=mxCreateStructMatrix(1,1,8,fields);put(a,"attempted",mxCreateLogicalScalar(r.attempted));put(a,"already_closed",mxCreateLogicalScalar(r.already_closed));
    put(a,"returned",mxCreateLogicalScalar(r.returned));put(a,"ok",mxCreateLogicalScalar(r.ok));put(a,"submit_ns",u64(r.before.ns));put(a,"return_ns",u64(r.after.ns));
    put(a,"socket_error",i32(r.socket_error));put(a,"error_code",mxCreateString(r.error.c_str()));return a;}
mxArray* status(){const char*fields[]={"handle","scope","open","closed","failed","failure_code","first_error","local_host","local_port","remote_host","remote_port",
    "qpc_frequency","sent_count","received_count","send_attempt_count","receive_attempt_count","last_send","last_receive","last_close","sdk_source_sha256",
    "limit_provenance","wsa_started","bind_ok","exclusive_address_use","nonblocking","os_receive_buffer_bytes","os_send_buffer_bytes","close_calls","receive_clock_scope","send_receipt_scope","open_socket_error"
#ifdef GPENMPC_UDP_CONTINUOUS_GP
    ,"continuous_gp"
#endif
    };
    auto*a=mxCreateStructMatrix(1,1,sizeof(fields)/sizeof(*fields),fields);put(a,"handle",u64(owner.handle));
    put(a,"scope",mxCreateString(owner.config.scope.empty()?SCOPE:owner.config.scope.c_str()));
    put(a,"open",mxCreateLogicalScalar(owner.opened));put(a,"closed",mxCreateLogicalScalar(owner.closed));put(a,"failed",mxCreateLogicalScalar(owner.failed));
    put(a,"failure_code",mxCreateString(owner.failure_code.c_str()));put(a,"first_error",mxCreateString(owner.first_error.c_str()));
    put(a,"local_host",mxCreateString(owner.config.local_host.c_str()));put(a,"remote_host",mxCreateString(owner.config.remote_host.c_str()));
    put(a,"local_port",u16(owner.config.local_port));put(a,"remote_port",u16(owner.config.remote_port));put(a,"qpc_frequency",u64(owner.frequency));
    put(a,"sent_count",u64(owner.sent_count));put(a,"received_count",u64(owner.received_count));put(a,"send_attempt_count",u64(owner.send_attempts));put(a,"receive_attempt_count",u64(owner.receive_attempts));
    put(a,"last_send",send_record(owner.last_send));put(a,"last_receive",batch_record());put(a,"last_close",close_record());
    put(a,"sdk_source_sha256",mxCreateString(SDK_SHA));put(a,"limit_provenance",mxCreateString("300B_HOST_ADAPTER_ENGINEERING_CAP_ALIGNED_WITH_TESTED_RAW_NOT_UDP_LIMIT;64_DEQUEUE_CALL_CAP_NOT_OS_QUEUE_DEPTH"));
    put(a,"wsa_started",mxCreateLogicalScalar(owner.wsa_started));put(a,"bind_ok",mxCreateLogicalScalar(owner.bind_ok));put(a,"exclusive_address_use",mxCreateLogicalScalar(owner.exclusive));
    put(a,"nonblocking",mxCreateLogicalScalar(owner.nonblocking));put(a,"os_receive_buffer_bytes",i32(owner.recv_buffer_bytes));put(a,"os_send_buffer_bytes",i32(owner.send_buffer_bytes));
    put(a,"close_calls",u64(owner.close_calls));put(a,"receive_clock_scope",mxCreateString("HOST_QPC_DEQUEUE_CALL_INTERVAL"));
    put(a,"send_receipt_scope",mxCreateString("SENDTO_RETURN_LENGTH_AND_CALL_QPC"));put(a,"open_socket_error",i32(owner.open_socket_error));
#ifdef GPENMPC_UDP_CONTINUOUS_GP
    const char*names[]={"active","queries","completed","replies","worker_receives","queued_datagrams","last_result","deferred_history_count","deferred_history_chunks","prediction_enabled"};
    auto*g=mxCreateStructMatrix(1,1,10,names);put(g,"active",mxCreateLogicalScalar(continuous_gp.active));
    put(g,"prediction_enabled",mxCreateLogicalScalar(continuous_gp.prediction_enabled));
    put(g,"queries",u64(continuous_gp.calls));put(g,"completed",u64(continuous_gp.completed));put(g,"replies",u64(continuous_gp.replies));
    put(g,"worker_receives",u64(continuous_gp.worker_receives));put(g,"queued_datagrams",u64(continuous_gp.queued.size()));put(g,"last_result",gp_record(continuous_gp.last_result));
    put(g,"deferred_history_count",u64(continuous_gp.deferred_history.size()));
    const auto history_size=owner.closed&&continuous_gp.registered_handle==owner.handle?continuous_gp.deferred_history.size():0;
    auto*chunks=mxCreateCellMatrix((history_size+MAX_BATCH-1)/MAX_BATCH,1);
    for(std::size_t offset=0;offset<history_size;offset+=MAX_BATCH){
        ReceiveBatch batch{};batch.valid=std::min<std::size_t>(MAX_BATCH,history_size-offset);
        batch.attempts=batch.valid;
        for(unsigned k=0;k<batch.valid;++k)batch.records[k]=continuous_gp.deferred_history[offset+k];
        mxSetCell(chunks,offset/MAX_BATCH,receive_records(batch,true));
    }
    put(g,"deferred_history_chunks",chunks);put(a,"continuous_gp",g);
#endif
    return a;}
void at_exit(){
#ifdef GPENMPC_UDP_CONTINUOUS_GP
    std::unique_lock<std::mutex> lock(owner_mutex);continuous_gp.join(lock);
#endif
    try{owner.close();}catch(...){owner.release_noexcept();}}
}
extern "C" void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
#ifdef GPENMPC_UDP_CONTINUOUS_GP
    std::unique_lock<std::mutex> lock(owner_mutex);
#endif
    try{
        if(nrhs<1||nlhs!=1)throw rtudp::Error("Arity","Exactly one output is required for every operation; no unrecorded side effect.");
        const auto command=text(prhs[0]);
        if(command=="status"){
            if(nrhs==2)owner.check_handle(handle(prhs[1]));else if(nrhs!=1)throw Error("Arity","status accepts zero or one handle.");
            plhs[0]=status();return;
        }
        if(command=="open"){
            if(nrhs!=2)throw Error("Arity","open requires the explicit config.");
            if(!exit_registered){mexAtExit(at_exit);exit_registered=true;}
            const auto c=config(prhs[1]);if(generation==std::numeric_limits<U64>::max())throw Error("Handle","Handle generation exhausted.");
            owner.open(c,++generation);mexLock();mex_locked=true;plhs[0]=u64(owner.handle);return;
        }
        if(nrhs<2)throw Error("Arity","Operation requires the exact uint64 owner handle.");
        owner.check_handle(handle(prhs[1]));
#ifdef GPENMPC_UDP_CONTINUOUS_GP
        if(command=="gp_start"){
            if((nrhs!=4&&nrhs!=5&&nrhs!=6)||!mxIsUint64(prhs[2])||mxGetNumberOfElements(prhs[2])!=3||!mxIsUint8(prhs[3])||mxGetNumberOfElements(prhs[3])!=4)
                throw Error("GpStart","Exact UID/boot/original confirmation and source/target addresses required.");
            owner.healthy();const auto*r=static_cast<const U64*>(mxGetData(prhs[2]));const auto*ids=static_cast<const unsigned char*>(mxGetData(prhs[3]));
            for(unsigned k=0;k<4;++k)if(!ids[k])throw Error("GpStart","Missing source/target identity.");
            const auto bound=nrhs>=5?handle(prhs[4]):0;
            if(nrhs==6&&(!mxIsLogical(prhs[5])||mxGetNumberOfElements(prhs[5])!=1))throw Error("GpStart","Receive-only selection must be one explicit logical.");
            const bool receive_only=nrhs==6&&mxIsLogicalScalarTrue(prhs[5]);
            if(bound>250000)throw Error("GpHistoryBound","Cannot exceed the existing selected run raw-record limit.");
            continuous_gp.start(r[0],r[1],r[2],ids,bound,receive_only);plhs[0]=status();return;
        }
        if(command=="gp_stop"){
            if(nrhs!=2)throw Error("GpStop","Stop only the current same-owner GP thread.");
            continuous_gp.join(lock);plhs[0]=status();return;
        }
#endif
        if(command=="close"){
            if(nrhs!=2)throw Error("Arity","close takes only the handle.");
#ifdef GPENMPC_UDP_CONTINUOUS_GP
            continuous_gp.join(lock);
#endif
            owner.close();
            if(owner.closed&&mex_locked){mexUnlock();mex_locked=false;}plhs[0]=close_record();return;
        }
        if(command=="send"){
            if(nrhs!=3)throw Error("Arity","send requires one original uint8 vector.");const auto*b=prhs[2];
            const bool shape=mxGetClassID(b)==mxUINT8_CLASS&&!mxIsComplex(b)&&!mxIsSparse(b)&&mxGetNumberOfDimensions(b)==2&&(mxGetM(b)==1||mxGetN(b)==1);
            owner.prepare_send(shape?static_cast<const unsigned char*>(mxGetData(b)):nullptr,mxGetNumberOfElements(b),shape);
#ifdef GPENMPC_UDP_CONTINUOUS_GP
            if(continuous_gp.registered_handle==owner.handle)continuous_gp.resequence();
#endif
            owner.send_prepared();plhs[0]=send_record(owner.last_send);return;
        }
        if(command=="receive"){
            if(nrhs!=3)throw Error("Arity","receive requires explicit max_count 1..64.");
#ifdef GPENMPC_UDP_CONTINUOUS_GP
            if(continuous_gp.active||(!continuous_gp.queued.empty()&&continuous_gp.registered_handle==owner.handle))continuous_gp.drain(number(prhs[2]));else
#endif
            owner.receive(number(prhs[2]));
            // Transfer the batch under the socket lock, then release it before MATLAB allocation.
            // Copy immutable records and frequency for conversion.
            const ReceiveBatch batch=owner.last_receive;const U64 frequency=owner.frequency;
#ifdef GPENMPC_UDP_CONTINUOUS_GP
            lock.unlock();
#endif
            plhs[0]=receive_records(batch,true,frequency);return;
        }
        throw Error("Command","Unknown operation; no fallback.");
    }catch(const rtudp::Error&e){
#ifdef GPENMPC_UDP_CONTINUOUS_GP
        if(!lock.owns_lock())lock.lock();
#endif
        if((owner.opened||owner.wsa_started)&&!mex_locked){mexLock();mex_locked=true;}
        if(owner.opened||owner.wsa_started)owner.latch(e.code.c_str(),e.what());
        const std::string id="gpenmpcNative:UdpTransport"+e.code;
#ifdef GPENMPC_UDP_CONTINUOUS_GP
        lock.unlock(); // mexErrMsg long-jumps: do not leave the shared mutex locked.
#endif
        mexErrMsgIdAndTxt(id.c_str(),"%s",e.what());
    }catch(const std::exception&e){
#ifdef GPENMPC_UDP_CONTINUOUS_GP
        if(!lock.owns_lock())lock.lock();
#endif
        if((owner.opened||owner.wsa_started)&&!mex_locked){mexLock();mex_locked=true;}
        owner.latch("Unexpected",e.what());
#ifdef GPENMPC_UDP_CONTINUOUS_GP
        lock.unlock();
#endif
        mexErrMsgIdAndTxt("gpenmpcNative:UdpTransportUnexpected","%s",e.what());
    }catch(...){
#ifdef GPENMPC_UDP_CONTINUOUS_GP
        if(!lock.owns_lock())lock.lock();
#endif
        if((owner.opened||owner.wsa_started)&&!mex_locked){mexLock();mex_locked=true;}
        owner.latch("Unknown","Unknown native exception; only status/close remain available.");
#ifdef GPENMPC_UDP_CONTINUOUS_GP
        lock.unlock();
#endif
        mexErrMsgIdAndTxt("gpenmpcNative:UdpTransportUnknown","Unknown native exception; status retained.");}
}
#else
// This executable exercises only pure guards/classification. It NEVER invokes
// Owner::open/send_prepared/receive, WSAStartup, socket, bind, sendto or recvfrom.
int main(){using namespace rtudp;unsigned checks=0,pass=0;auto check=[&](bool ok){++checks;if(ok)++pass;};
#ifdef GPENMPC_UDP_TRANSPORT_ENDPOINT_GUARD_TEST
    Config c{SCOPE,"127.0.0.1","127.0.0.1",true,62322,62321};check(valid_config(c));
    c.scope=COPTERSIM_SCOPE;c.local_port=14550;c.remote_port=18570;check(valid_config(c));
    auto b=c;b.local_port=14551;check(!valid_config(b));
    b=c;b.remote_host="192.168.1.1";check(!valid_config(b));
    b=c;b.allow_loopback=false;check(!valid_config(b));
    b=c;b.scope=SCOPE;check(!valid_config(b));
    std::printf("{\"scope\":\"PURE_ENDPOINT_GUARDS_NO_SOCKET\",\"pass\":%s,\"checks\":%u,\"passed\":%u,\"socket_calls\":0,\"MATLAB_runs\":0,\"sdk_source_sha256\":\"%s\"}\n",pass==checks?"true":"false",checks,pass,SDK_SHA);
    return pass==checks?0:1;
#else
    Config c{SCOPE,"127.0.0.1","127.0.0.1",true,62322,62321};check(valid_config(c));
    for(unsigned p:{14550U,18570U,62199U,62400U}){auto bad=c;bad.local_port=p;check(!valid_config(bad));}
    auto b=c;b.remote_host="192.168.1.1";check(!valid_config(b));b=c;b.local_host="0.0.0.0";check(!valid_config(b));
    b=c;b.allow_loopback=false;check(!valid_config(b));b=c;b.scope="LIVE";check(!valid_config(b));b=c;b.remote_port=b.local_port;check(!valid_config(b));
    U64 ns=0;check(qpc_ns(123456789,10000000,ns)&&ns==12345678900ULL);check(!qpc_ns(1,0,ns));check(!qpc_ns(std::numeric_limits<U64>::max(),1,ns));
    unsigned char raw[301]{};raw[0]=253;raw[299]=42;
    Owner s;s.opened=true;s.closed=false;s.prepare_send(raw,300);check(s.last_send.bytes_complete&&s.last_send.stored==300&&s.last_send.bytes[299]==42);
    s.last_send.attempted=true;s.finish_send(300,0);check(s.last_send.ok&&s.sent_count==1);
    try{s.finish_send(299,0);}catch(const Error&){}check(s.failed&&s.last_send.sent==299&&s.failure_code=="PartialSend");
    const auto original=s.failure_code;try{s.healthy();}catch(const Error&){}check(s.failure_code==original);
    Owner oversize;oversize.opened=true;oversize.closed=false;try{oversize.prepare_send(raw,301);}catch(const Error&){}
    check(oversize.failed&&!oversize.last_send.attempted&&oversize.last_send.requested==301&&oversize.last_send.stored==301);
    Owner r;ReceiveRecord rr;rr.received=-1;rr.socket_error=WSAEWOULDBLOCK;r.finish_receive(rr);check(rr.would_block&&!r.failed);
    rr={};rr.received=300;rr.source_matched=true;r.finish_receive(rr);check(rr.ok&&r.received_count==1);
    auto&tail=r.last_receive.records[1];tail.received=-1;tail.socket_error=WSAEMSGSIZE;tail.stored=301;tail.bytes[0]=253;
    try{r.finish_receive(tail);}catch(const Error&){}check(r.failed&&tail.error=="OversizeReceive"&&tail.bytes[0]==253&&r.received_count==1);
    Owner other;ReceiveRecord wrong;wrong.received=21;try{other.finish_receive(wrong);}catch(const Error&){}check(other.failed&&wrong.error=="ReceiveSource");
    Owner unknown;ReceiveRecord er;er.received=-1;er.socket_error=12345;try{unknown.finish_receive(er);}catch(const Error&){}check(unknown.failed&&er.socket_error==12345);
    Owner zero;ReceiveRecord zr;zr.received=0;try{zero.finish_receive(zr);}catch(const Error&){}check(zero.failed&&zr.error=="ReceiveLength");
    Owner closed;closed.close();check(closed.last_close.already_closed&&closed.last_close.ok&&closed.closed);
    check(sizeof(SOCKET)==sizeof(void*));check(MAX_BATCH==64&&MAX_BYTES==300);check(std::strlen(SDK_SHA)==64);
    std::printf("{\"scope\":\"HOST_PURE_NO_SOCKET_NEGATIVES\",\"pass\":%s,\"checks\":%u,\"passed\":%u,\"socket_calls\":0,\"MATLAB_runs\":0}\n",pass==checks?"true":"false",checks,pass);
    return pass==checks?0:1;
#endif
}
#endif
