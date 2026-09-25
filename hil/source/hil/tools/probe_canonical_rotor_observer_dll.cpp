// Test the rotor-observer DLL with localhost IO.
// Use two sequential model lifetimes and the delivery-DLL start/frame contract.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <windows.h>
#include <iphlpapi.h>
#include <bcrypt.h>
#include <array>
#include <vector>
#include <string>
#include <stdexcept>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <algorithm>
#include "../host_runtime/copter_observer/rotor_observer.hpp"

namespace {
using namespace gpenmpc_rotor_observer;
constexpr char expected_sha[]="B817CA67429871A64F2368B7D161B91F4E2E032607B1D76E73AE12D4255D33C1";
constexpr std::uint64_t session=26090501;
constexpr unsigned normal_ticks=121;
struct Frame {std::array<double,60> v{};std::array<double,30> s{},g{};std::array<double,32> d{};};
static_assert(sizeof(Frame)==152*sizeof(double));
struct File {
    FILE *f=nullptr;
    explicit File(const wchar_t *p){if(GetFileAttributesW(p)!=INVALID_FILE_ATTRIBUTES)throw std::runtime_error("output_exists");
        f=_wfopen(p,L"wb");if(!f)throw std::runtime_error("output_open");}
    ~File(){if(f)std::fclose(f);}
    void write(const void *p,std::size_t n){if(std::fwrite(p,1,n,f)!=n)throw std::runtime_error("raw_write");}
};
template<class T>T symbol(HMODULE h,const char *n){auto p=GetProcAddress(h,n);if(!p)throw std::runtime_error(n);return reinterpret_cast<T>(p);}
unsigned lifetimes=0,concurrent=0,max_concurrent=0;
struct Module {
    HMODULE h=nullptr;bool initialized=false;
    void(*init)()=nullptr;void(*destroy)()=nullptr;void(*step)()=nullptr;
    void(*pos)(const double*,const double*)=nullptr;void(*gps)(const double*)=nullptr;
    void(*pwm)(const double*)=nullptr;void(*terrain)(double*)=nullptr;void(*env)(const double*)=nullptr;
    void(*vehicle)(double*,int)=nullptr;void(*sensor)(double*,int)=nullptr;void(*gps_out)(double*,int)=nullptr;
    void(*diag)(double*)=nullptr;void(*status)(std::uint64_t*)=nullptr;double(*period)()=nullptr;
    explicit Module(const wchar_t *p){if(concurrent)throw std::runtime_error("second_model_forbidden");
        h=LoadLibraryW(p);if(!h)throw std::runtime_error("LoadLibraryW");++lifetimes;++concurrent;max_concurrent=std::max(max_concurrent,concurrent);
        try {init=symbol<void(*)()>(h,"DllReInitModel");destroy=symbol<void(*)()>(h,"DllDestroyModel");step=symbol<void(*)()>(h,"Dllstep");
            pos=symbol<void(*)(const double*,const double*)>(h,"DllInitPosAngState");gps=symbol<void(*)(const double*)>(h,"DllInitGpsPos");
            pwm=symbol<void(*)(const double*)>(h,"DllInputPWMs");terrain=symbol<void(*)(double*)>(h,"DllTerrainIn15d");
            env=symbol<void(*)(const double*)>(h,"DllInputDoubCtrls");vehicle=symbol<void(*)(double*,int)>(h,"DlloutVehileInfo60d");
            sensor=symbol<void(*)(double*,int)>(h,"DlloutHILSensor30d");gps_out=symbol<void(*)(double*,int)>(h,"DlloutHILGPS30d");
            diag=symbol<void(*)(double*)>(h,"DllOutCopterData");status=symbol<void(*)(std::uint64_t*)>(h,"DllRotorObserverStatus");
            period=symbol<double(*)()>(h,"DllGetStep0");symbol<void(*)()>(h,"?DllCreatModel@@YAXXZ")();
        }catch(...){FreeLibrary(h);h=nullptr;--concurrent;throw;}}
    ~Module(){if(h){if(initialized)destroy();FreeLibrary(h);--concurrent;}}
    Module(const Module&)=delete;Module&operator=(const Module&)=delete;
    void start(){
        const double xyz[3]={},rpy[3]={},origin[3]={40.1540302,116.2593683,50},empty[28]={},u[16]={};double ground[15]={};
        env(empty);pos(xyz,rpy);gps(origin);init();initialized=true;env(empty);pwm(u);terrain(ground);
        if(std::abs(period()-.001)>1e-12)throw std::runtime_error("parent_1ms_DLL_period");
    }
    Frame output(){Frame o;vehicle(o.v.data(),60);sensor(o.s.data(),30);gps_out(o.g.data(),30);diag(o.d.data());return o;}
    std::array<std::uint64_t,11> counts(){std::array<std::uint64_t,11>s{};status(s.data());return s;}
    void finish(){if(initialized){destroy();initialized=false;}}
};
unsigned owned_udp(){DWORD n=0;DWORD rc=GetExtendedUdpTable(nullptr,&n,FALSE,AF_INET,UDP_TABLE_OWNER_PID,0);
    if(rc!=ERROR_INSUFFICIENT_BUFFER&&rc!=NO_ERROR)throw std::runtime_error("udp_table_size");
    std::vector<std::uint8_t>b(n);rc=GetExtendedUdpTable(b.data(),&n,FALSE,AF_INET,UDP_TABLE_OWNER_PID,0);
    if(rc!=NO_ERROR)throw std::runtime_error("udp_table_read");
    const auto *t=reinterpret_cast<const MIB_UDPTABLE_OWNER_PID*>(b.data());unsigned count=0;
    for(DWORD i=0;i<t->dwNumEntries;++i)if(t->table[i].dwOwningPid==GetCurrentProcessId())++count;return count;
}
struct Receiver {
    bool wsa=false;SOCKET s=INVALID_SOCKET;unsigned port=0;
    Receiver(){WSADATA d{};if(WSAStartup(MAKEWORD(2,2),&d)!=0)throw std::runtime_error("test_WSAStartup");wsa=true;
        try{s=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);if(s==INVALID_SOCKET)throw std::runtime_error("receiver_socket");
            sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(0x7f000001U);
            if(bind(s,reinterpret_cast<sockaddr*>(&a),sizeof(a))!=0)throw std::runtime_error("localhost_bind");
            int len=sizeof(a);if(getsockname(s,reinterpret_cast<sockaddr*>(&a),&len)!=0)throw std::runtime_error("getsockname");port=ntohs(a.sin_port);
            u_long nonblocking=1;if(ioctlsocket(s,FIONBIO,&nonblocking)!=0)throw std::runtime_error("receiver_nonblocking");
        }catch(...){close();throw;}}
    ~Receiver(){close();}
    void close(){if(s!=INVALID_SOCKET){closesocket(s);s=INVALID_SOCKET;}if(wsa){WSACleanup();wsa=false;}}
    std::vector<Packet> drain(bool wait){
        if(wait){fd_set f;FD_ZERO(&f);FD_SET(s,&f);timeval t{0,100000};int rc=select(0,&f,nullptr,nullptr,&t);if(rc<=0)throw std::runtime_error("localhost_packet_wait_100ms");}
        std::vector<Packet> packets;
        for(unsigned count=0;count<8;++count){std::array<std::uint8_t,129>b{};sockaddr_in a{};int len=sizeof(a);
            int n=recvfrom(s,reinterpret_cast<char*>(b.data()),static_cast<int>(b.size()),0,reinterpret_cast<sockaddr*>(&a),&len);
            if(n==SOCKET_ERROR){if(WSAGetLastError()==WSAEWOULDBLOCK)return packets;throw std::runtime_error("localhost_recv");}
            if(n!=128||a.sin_addr.s_addr!=htonl(0x7f000001U))throw std::runtime_error("packet_length_source");
            Packet p{};std::copy_n(b.begin(),128,p.begin());packets.push_back(p);}
        throw std::runtime_error("receiver_queue_bound");
    }
};
void env(const char *k,const char *v){if(!SetEnvironmentVariableA(k,v))throw std::runtime_error("test_environment");}
const char *keys[]={"GPENMPC_ROTOR_OBSERVER_ENABLE","GPENMPC_ROTOR_OBSERVER_IP","GPENMPC_ROTOR_OBSERVER_PORT","GPENMPC_ROTOR_OBSERVER_SESSION","GPENMPC_ROTOR_OBSERVER_DLL_SHA256"};
void clear_env(){for(const auto*k:keys)env(k,nullptr);}
struct EnvGuard{~EnvGuard(){for(const auto*k:keys)SetEnvironmentVariableA(k,nullptr);}};
std::array<std::uint8_t,32> digest(const wchar_t *path){
    FILE *f=_wfopen(path,L"rb");if(!f)throw std::runtime_error("dll_hash_open");
    BCRYPT_ALG_HANDLE a=nullptr;BCRYPT_HASH_HANDLE h=nullptr;std::array<std::uint8_t,32>out{};
    try{if(BCryptOpenAlgorithmProvider(&a,BCRYPT_SHA256_ALGORITHM,nullptr,0)<0)throw std::runtime_error("SHA_provider");
        if(BCryptCreateHash(a,&h,nullptr,0,nullptr,0,0)<0)throw std::runtime_error("SHA_create");
        std::array<std::uint8_t,16384>b{};std::size_t n=0;while((n=std::fread(b.data(),1,b.size(),f))!=0)
            if(BCryptHashData(h,b.data(),static_cast<ULONG>(n),0)<0)throw std::runtime_error("SHA_update");
        if(std::ferror(f)||BCryptFinishHash(h,out.data(),32,0)<0)throw std::runtime_error("SHA_finish");
    }catch(...){if(h)BCryptDestroyHash(h);if(a)BCryptCloseAlgorithmProvider(a,0);std::fclose(f);throw;}
    BCryptDestroyHash(h);BCryptCloseAlgorithmProvider(a,0);std::fclose(f);return out;
}
std::string hex(const std::array<std::uint8_t,32>& b){char out[65]{};for(unsigned k=0;k<32;++k)std::sprintf(out+2*k,"%02X",b[k]);return out;}
std::uint64_t integer(const Packet&p,std::size_t at,std::size_t n){std::uint64_t v=0;for(std::size_t k=0;k<n;++k)v|=std::uint64_t(p[at+k])<<(8*k);return v;}
double real(const Packet&p,std::size_t at){auto bits=integer(p,at,8);double v;std::memcpy(&v,&bits,8);return v;}
void input(Module&a,unsigned tick,bool bad){
    if(tick==0){a.step();return;} // Initialize at t=0.
    double f[28]={};f[0]=2;f[1]=1+(tick-1)/10;f[2]=tick*.001;f[4]=2.21;f[7]=2;
    f[21]=1;f[22]=double(session);f[23]=f[2];f[24]=15;f[25]=1;f[26]=1;
    double u[16]={};for(unsigned k=0;k<6;++k)u[k]=.1+.01*k;if(bad)u[0]=-.1;
    a.env(f);a.pwm(u);a.step();
}
bool finite(const Frame&f){const auto*p=reinterpret_cast<const double*>(&f);for(unsigned k=0;k<152;++k)if(!std::isfinite(p[k]))return false;return true;}
std::string escape(const std::string&s){std::string r;for(char c:s){if(c=='"'||c=='\\')r+='\\';if(c=='\n')r+="\\n";else r+=c;}return r;}
}
int wmain(int argc,wchar_t**argv){
    if(argc!=6)return 2;unsigned checks=0,passed=0,rx_count=0,repeat_ticks=0;std::string failure;
    std::array<std::uint64_t,11> disabled{},enabled{},closed{};std::vector<std::string>names;
    unsigned receiver_port=0;bool output_equal=true,finite_outputs=true,packet_match=true,lag_match=true;
    double max_lag_error=0;std::array<std::uint8_t,32>actual{};
    auto check=[&](bool ok,const char*n){++checks;names.emplace_back(n);if(ok)++passed;else throw std::runtime_error(n);};
    try{
        File result(argv[2]),csv(argv[3]),packets(argv[4]),outputs(argv[5]);
        try{
            EnvGuard eg;clear_env();actual=digest(argv[1]);check(hex(actual)==expected_sha,"independent_loader_exact_DLL_SHA");
            check(owned_udp()==0,"test_process_starts_with_zero_UDP");
            std::fprintf(csv.f,"lifetime,tick,dll_time,core_time,failed,delivery_session,delivery_status,observer_failure,observed_generation,sent_generation,received_this_tick\n");
            const std::uint32_t raw_header[]={0x4d36524f,1,152};outputs.write(raw_header,sizeof raw_header);
            std::vector<Frame>baseline;baseline.reserve(normal_ticks);
            {
                Module a(argv[1]);a.start();
                for(unsigned k=0;k<normal_ticks;++k){input(a,k,false);Frame f=a.output();baseline.push_back(f);outputs.write(&f,sizeof f);finite_outputs&=finite(f);}
                disabled=a.counts();check(disabled[0]==0&&disabled[2]==0&&disabled[3]==0&&disabled[4]==0,"actual_default_disabled_no_open_send");
                check(owned_udp()==0,"actual_disabled_process_UDP_zero");
                check(baseline[0].d[2]==0&&std::abs(baseline[0].v[2]-.001)<1e-12,"parent_reset_tick_time_semantics");
                check(baseline.back().d[2]>.11&&baseline.back().d[0]==0,"default_model_accepted_steps_healthy");
                a.finish();
            }
            check(concurrent==0&&owned_udp()==0,"disabled_fully_unloaded_no_socket");
            {
                Receiver r;receiver_port=r.port;env(keys[0],"1");env(keys[1],"127.0.0.1");env(keys[2],std::to_string(r.port).c_str());
                env(keys[3],"26090501");env(keys[4],expected_sha);
                Module a(argv[1]);a.start();std::uint64_t last_gen=0,last_sent=0;double last_sim=0;
                std::array<double,6>lag{};const unsigned order[]={4,0,3,5,1,2};
                auto observe=[&](unsigned k,const Frame&f){
                    auto stat=a.counts();auto ps=r.drain(stat[4]>last_sent);last_sent=stat[4];
                    if(ps.empty()&&stat[4]>0&&stat[1]==0)++repeat_ticks;
                    if(ps.size()>1)throw std::runtime_error("more_than_one_frame_per_step");
                    for(const auto&p:ps){++rx_count;packets.write(p.data(),p.size());
                        auto gen=integer(p,24,8);double sim=real(p,32);
                        packet_match&=std::memcmp(p.data(),"M6ROTOR1",8)==0&&integer(p,8,2)==1&&integer(p,10,2)==1&&integer(p,12,4)==128;
                        packet_match&=integer(p,16,8)==session&&gen==last_gen+1&&sim>last_sim&&std::abs(sim-gen*.01)<1e-12;
                        packet_match&=sim==f.d[2]&&f.d[26]==double(session)&&f.d[31]==0&&f.d[0]==0;
                        packet_match&=std::memcmp(p.data()+88,actual.data(),32)==0&&integer(p,120,4)==crc32(p.data(),120)&&integer(p,124,4)==0;
                        // Check scalar RK4 motor lag against stepPx4Rk4.
                        for(unsigned j=0;j<6;++j){const double target=(.1+.01*order[j])*32.145727009134916;
                            const double k1=(target-lag[j])/.12,k2=(target-(lag[j]+.005*k1))/.12;
                            const double k3=(target-(lag[j]+.005*k2))/.12,k4=(target-(lag[j]+.01*k3))/.12;
                            lag[j]+=.01*(k1+2*k2+2*k3+k4)/6;double v=real(p,40+8*j);
                            max_lag_error=std::max(max_lag_error,std::abs(v-lag[j]));lag_match&=std::isfinite(v)&&v>=0&&std::abs(v-lag[j])<1e-12;
                        }last_gen=gen;last_sim=sim;
                    }
                    std::fprintf(csv.f,"2,%u,%.17g,%.17g,%.0f,%.0f,%.0f,%llu,%llu,%llu,%zu\n",k,f.v[2],f.d[2],f.d[0],f.d[26],f.d[31],
                        static_cast<unsigned long long>(stat[1]),static_cast<unsigned long long>(stat[7]),static_cast<unsigned long long>(stat[8]),ps.size());
                };
                for(unsigned k=0;k<normal_ticks;++k){input(a,k,false);Frame f=a.output();outputs.write(&f,sizeof f);
                    finite_outputs&=finite(f);output_equal&=std::memcmp(&f,&baseline[k],sizeof f)==0;observe(k,f);}
                enabled=a.counts();
                check(output_equal,"enabled_disabled_all_152_original_outputs_bit_equal");
                check(packet_match,"actual_packets_identity_CRC_sequence_sync_output_time");
                check(lag_match&&max_lag_error<1e-12,"actual_six_lag_states_match_source_RK4_not_RPM");
                check(rx_count==12&&enabled[4]==12&&enabled[7]==12&&enabled[8]==12,"12_accepted_steps_exactly_12_real_packets");
                check(repeat_ticks>0&&enabled[5]>0,"duplicate_1ms_calls_do_not_emit_extra_10ms_samples");
                check(enabled[1]==0&&enabled[2]==1&&owned_udp()==2,"enabled_exact_one_sender_one_receiver");
                const auto before=rx_count;
                for(unsigned k=normal_ticks;k<161;++k){input(a,k,k<141);Frame f=a.output();outputs.write(&f,sizeof f);finite_outputs&=finite(f);observe(k,f);}
                auto failed=a.counts();Frame f=a.output();
                check(f.d[0]==1&&f.d[1]==1&&f.d[2]==baseline.back().d[2],"parent_invalid_control_freezes_accepted_state");
                check(failed[1]==static_cast<unsigned>(Failure::model_failed)&&failed[9]==1,"failed_actual_core_closes_observer");
                check(rx_count==before&&failed[4]==12&&failed[8]==12,"failed_then_good_input_no_packet_or_credit_wash");
                check(owned_udp()==1,"sender_released_after_fault_receiver_only");
                a.finish();closed=a.counts();check(closed[9]==1&&closed[4]==12,"destroy_preserves_counts_no_duplicate_close_or_sample");
                check(r.drain(false).empty(),"receiver_queue_empty_after_destroy");r.close();
            }
            clear_env();check(owned_udp()==0&&concurrent==0,"all_Dll_socket_resources_released");
            check(lifetimes==2&&max_concurrent==1,"two_sequential_lifetimes_max_one_singleton");
            check(finite_outputs,"original_outputs_all_finite_including_failure_tail");
        }catch(const std::exception&e){failure=e.what();}
        const bool ok=failure.empty()&&passed==checks;
        std::fprintf(result.f,"{\"status\":\"%s\",\"pass\":%s,\"checks_passed\":%u,\"checks_total\":%u,\"failure\":\"%s\",\"dll_sha256\":\"%s\",\"tests\":[",ok?"PASS_HOST_ACTUAL_DLL_LOCALHOST_OBSERVER":"FAIL_HOST_ACTUAL_DLL_LOCALHOST_OBSERVER",ok?"true":"false",passed,checks,escape(failure).c_str(),hex(actual).c_str());
        for(std::size_t k=0;k<names.size();++k)std::fprintf(result.f,"%s\"%s\"",k?",":"",names[k].c_str());
        std::fprintf(result.f,"],\"plant_lifetimes\":%u,\"plant_instances_concurrent_max\":%u,\"observer_packets_received\":%u,\"localhost_port\":%u,\"max_lag_state_abs_error_n\":%.17g,\"original_152_outputs_bit_equal\":%s,\"physical_model_step_s\":0.01,\"DLL_io_step_s\":0.001,\"session\":26090501,\"all_resources_released\":%s,\"COM_open\":0,\"CopterSim_started\":0,\"board_actions\":0,\"claim\":\"Candidate-DLL localhost observer probe\"}\n",lifetimes,max_concurrent,rx_count,receiver_port,max_lag_error,output_equal?"true":"false",concurrent==0?"true":"false");
        std::printf("%s checks=%u/%u packets=%u max_lag_error=%.17g failure=%s\n",ok?"PASS_HOST_DLL_LOOPBACK":"FAIL_HOST_DLL_LOOPBACK",passed,checks,rx_count,max_lag_error,failure.c_str());
        return ok?0:5;
    }catch(const std::exception&e){std::printf("HOST_TEST_EXCEPTION %s\n",e.what());return 6;}
}
