// Probe the control-cache DLL with the vendor export and model-start fixture.
#define wmain retained_rotor_probe_main
#include "probe_canonical_rotor_observer_dll.cpp"
#undef wmain
#include "../host_runtime/copter_observer/control_cache_observer.hpp"

namespace {
constexpr char cache_sha[]="D155F1E4A1824B4D10EAB0860FC8946AF616874421FE1F9FB1F1BE05A54285BB";
using Datagram=std::vector<std::uint8_t>;
std::vector<Datagram> drain_cache(Receiver&r,bool wait){
    if(wait){fd_set f;FD_ZERO(&f);FD_SET(r.s,&f);timeval t{0,100000};
        if(select(0,&f,nullptr,nullptr,&t)<=0)throw std::runtime_error("cache_packet_wait_100ms");}
    std::vector<Datagram> out;
    for(unsigned k=0;k<8;++k){std::array<std::uint8_t,217>b{};sockaddr_in a{};int len=sizeof(a);
        int n=recvfrom(r.s,reinterpret_cast<char*>(b.data()),static_cast<int>(b.size()),0,reinterpret_cast<sockaddr*>(&a),&len);
        if(n==SOCKET_ERROR){if(WSAGetLastError()==WSAEWOULDBLOCK)return out;throw std::runtime_error("cache_recv");}
        if((n!=128&&n!=216)||a.sin_addr.s_addr!=htonl(0x7f000001U))throw std::runtime_error("cache_packet_length_source");
        out.emplace_back(b.begin(),b.begin()+n);
    }throw std::runtime_error("cache_receiver_queue_bound");
}
std::uint64_t u64(const Datagram&p,std::size_t at,std::size_t n){std::uint64_t v=0;
    for(std::size_t k=0;k<n;++k)v|=std::uint64_t(p.at(at+k))<<(8*k);return v;}
double f64(const Datagram&p,std::size_t at){auto bits=u64(p,at,8);double v;std::memcpy(&v,&bits,8);return v;}
std::array<double,16> controls(unsigned tick,bool bad){std::array<double,16> u{};
    if(tick<=60||tick>100)for(unsigned j=0;j<6;++j)u[j]=.1+.01*j;
    if(bad)u[0]=-.1;return u;}
void cache_input(Module&a,unsigned tick,bool bad){
    if(tick==0){a.step();return;}
    double f[28]={};f[0]=2;f[1]=1+(tick-1)/10;f[2]=tick*.001;f[4]=2.21;f[7]=2;
    f[21]=1;f[22]=double(session);f[23]=f[2];f[24]=15;f[25]=1;f[26]=1;
    const auto u=controls(tick,bad);a.env(f);a.pwm(u.data());a.step();
}
}
int wmain(int argc,wchar_t**argv){
    // parent DLL, candidate DLL, receipt, per-tick CSV, cache raw,
    // rotor raw, original outputs raw. All files must be new.
    if(argc!=8)return 2;
    unsigned total=0,passed=0,cache_count=0,rotor_count=0,zero_count=0;
    std::string failure;std::vector<std::string>names;
    bool original_equal=true,cache_correct=true,rotor_correct=true,finite_all=true;
    unsigned rx_port=0,first_zero_tick=0;std::uint64_t last_gen=0,last_call=0;
    std::array<std::uint64_t,5> final_cache{};
    auto check=[&](bool ok,const char*n){++total;names.emplace_back(n);if(ok)++passed;else throw std::runtime_error(n);};
    try{File result(argv[3]),csv(argv[4]),cache_raw(argv[5]),rotor_raw(argv[6]),original_raw(argv[7]);
        try{EnvGuard guard;clear_env();
            check(hex(digest(argv[1]))==expected_sha,"exact_parent_DLL_identity");
            const auto actual=digest(argv[2]);check(hex(actual)==cache_sha,"exact_cache_candidate_DLL_identity");
            check(owned_udp()==0,"initial_no_UDP");
            std::fprintf(csv.f,"tick,core_time,failed,generation,cache_sent,cache_failure,rotor_sent,zero_received\n");
            const std::uint32_t h[]={0x4d364343,1,152};original_raw.write(h,sizeof h);
            std::vector<Frame> reference;reference.reserve(normal_ticks);
            {Module a(argv[1]);a.start();for(unsigned k=0;k<normal_ticks;++k){cache_input(a,k,false);
                Frame f=a.output();reference.push_back(f);original_raw.write(&f,sizeof f);finite_all&=finite(f);}
                check(reference.back().d[0]==0&&reference.back().d[2]>.11,"parent_original_model_accepts_input_sequence");a.finish();}
            check(concurrent==0&&owned_udp()==0,"parent_model_unloaded_before_candidate");
            {Module a(argv[2]);a.start();auto status=symbol<void(*)(std::uint64_t*)>(a.h,"DllControlCacheObserverStatus");
                for(unsigned k=0;k<normal_ticks;++k){cache_input(a,k,false);Frame f=a.output();
                    original_equal&=std::memcmp(&f,&reference[k],sizeof f)==0;original_raw.write(&f,sizeof f);finite_all&=finite(f);}
                std::array<std::uint64_t,5>s{};status(s.data());
                check(s==std::array<std::uint64_t,5>{},"default_disabled_cache_no_actions");
                check(a.counts()[2]==0&&owned_udp()==0,"default_disabled_no_socket");a.finish();}
            check(original_equal,"candidate_disabled_original_152_outputs_bit_equal");
            {Receiver r;rx_port=r.port;env(keys[0],"1");env(keys[1],"127.0.0.1");env(keys[2],std::to_string(r.port).c_str());
                env(keys[3],"26090501");env(keys[4],cache_sha);
                Module a(argv[2]);a.start();auto status=symbol<void(*)(std::uint64_t*)>(a.h,"DllControlCacheObserverStatus");
                auto observe=[&](unsigned k,const Frame&f){std::array<std::uint64_t,5>s{};status(s.data());auto rs=a.counts();
                    auto packets=drain_cache(r,s[1]>cache_count);unsigned zeros=0;
                    if(packets.size()>2)throw std::runtime_error("more_than_two_observer_packets_per_step");
                    for(const auto&p:packets){
                        if(p.size()==128){++rotor_count;rotor_raw.write(p.data(),p.size());
                            rotor_correct&=std::memcmp(p.data(),"M6ROTOR1",8)==0&&u64(p,12,4)==128&&u64(p,16,8)==session;
                            rotor_correct&=u64(p,24,8)==rotor_count&&f64(p,32)==f.d[2]&&std::memcmp(p.data()+88,actual.data(),32)==0;
                            rotor_correct&=u64(p,120,4)==crc32(p.data(),120)&&u64(p,124,4)==0;
                            continue;
                        }
                        ++cache_count;cache_raw.write(p.data(),p.size());const auto gen=u64(p,24,8),call=u64(p,40,8);
                        cache_correct&=std::memcmp(p.data(),"M6CACHE1",8)==0&&u64(p,8,2)==1&&u64(p,10,2)==1&&u64(p,12,4)==216;
                        cache_correct&=u64(p,16,8)==session&&gen==last_gen+1&&gen==k/10&&f64(p,32)==f.d[2];
                        cache_correct&=call==k+1&&call>last_call&&std::memcmp(p.data()+176,actual.data(),32)==0;
                        cache_correct&=u64(p,208,4)==crc32(p.data(),208)&&u64(p,212,4)==0;
                        const auto expected=controls(k,false);bool zero=true;
                        for(unsigned j=0;j<16;++j){const double v=f64(p,48+8*j);
                            cache_correct&=std::memcmp(&v,&expected[j],8)==0;zero&=v==0;}
                        if(zero){++zero_count;++zeros;if(first_zero_tick==0)first_zero_tick=k;}
                        last_gen=gen;last_call=call;
                    }
                    std::fprintf(csv.f,"%u,%.17g,%.0f,%llu,%llu,%llu,%llu,%u\n",k,f.d[2],f.d[0],
                        static_cast<unsigned long long>(rs[7]),static_cast<unsigned long long>(s[1]),
                        static_cast<unsigned long long>(s[3]),static_cast<unsigned long long>(rs[4]),zeros);
                };
                for(unsigned k=0;k<normal_ticks;++k){cache_input(a,k,false);Frame f=a.output();
                    original_equal&=std::memcmp(&f,&reference[k],sizeof f)==0;finite_all&=finite(f);original_raw.write(&f,sizeof f);observe(k,f);}
                status(final_cache.data());
                check(original_equal,"candidate_enabled_all_152_original_outputs_bit_equal");
                check(cache_correct,"actual_16_control_raw_bits_input_call_identity_CRC_model_time");
                check(rotor_correct&&rotor_count==12,"original_128_byte_rotor_path_retained");
                check(cache_count==12&&last_gen==12,"exact_one_cache_packet_per_accepted_10ms_step");
                check(first_zero_tick==70&&zero_count==4,"zero_input_proven_only_after_step_consumes_it_not_input_write");
                check(final_cache[0]==12&&final_cache[1]==12&&final_cache[2]==12&&final_cache[3]==0&&final_cache[4]>0,"cache_counts_and_held_1ms_steps");
                check(owned_udp()==2&&a.counts()[2]==1,"two_observers_share_one_existing_sender_and_one_receiver");
                for(unsigned k=normal_ticks;k<161;++k){cache_input(a,k,k<141);Frame f=a.output();finite_all&=finite(f);original_raw.write(&f,sizeof f);observe(k,f);}
                status(final_cache.data());Frame end=a.output();
                check(end.d[0]==1&&end.d[1]==1&&end.d[2]==reference.back().d[2],"existing_invalid_control_model_failure_unchanged");
                check(final_cache[3]==1&&final_cache[1]==12&&cache_count==12,"model_failure_latches_observer_no_credit_wash");
                check(owned_udp()==1,"fault_releases_only_existing_sender");
                a.finish();check(drain_cache(r,false).empty(),"all_datagrams_accounted_at_destroy");r.close();
            }
            clear_env();check(owned_udp()==0&&concurrent==0,"all_sockets_and_model_released");
            check(lifetimes==3&&max_concurrent==1,"three_sequential_lifetimes_max_one_actual_model");
            check(finite_all,"all_original_outputs_finite_including_failure_tail");
        }catch(const std::exception&e){failure=e.what();}
        const bool ok=failure.empty()&&passed==total;
        std::fprintf(result.f,"{\"status\":\"%s\",\"pass\":%s,\"checks_passed\":%u,\"checks_total\":%u,\"failure\":\"%s\",\"tests\":[",
            ok?"PASS_HOST_ACTUAL_DLL_SHARED_SOCKET_CONTROL_CACHE":"FAIL_HOST_ACTUAL_DLL_SHARED_SOCKET_CONTROL_CACHE",ok?"true":"false",passed,total,escape(failure).c_str());
        for(std::size_t k=0;k<names.size();++k)std::fprintf(result.f,"%s\"%s\"",k?",":"",names[k].c_str());
        std::fprintf(result.f,"],\"cache_packets\":%u,\"rotor_packets\":%u,\"all_zero_cache_packets\":%u,\"first_zero_consumed_tick\":%u,\"original_152_outputs_bit_equal\":%s,\"candidate_DLL_SHA256\":\"%s\",\"parent_DLL_SHA256\":\"%s\",\"localhost_port\":%u,\"model_lifetimes\":%u,\"max_concurrent_models\":%u,\"all_resources_released\":%s,\"COM_open\":0,\"CopterSim_started\":0,\"board_actions\":0,\"claim\":\"Generated-DLL observation probe\"}\n",
            cache_count,rotor_count,zero_count,first_zero_tick,original_equal?"true":"false",cache_sha,expected_sha,rx_port,lifetimes,max_concurrent,concurrent==0?"true":"false");
        std::printf("%s checks=%u/%u cache=%u rotor=%u zero=%u first_zero_tick=%u failure=%s\n",ok?"PASS_HOST_CACHE_DLL":"FAIL_HOST_CACHE_DLL",passed,total,cache_count,rotor_count,zero_count,first_zero_tick,failure.c_str());return ok?0:5;
    }catch(const std::exception&e){std::printf("HOST_TEST_EXCEPTION %s\n",e.what());return 6;}
}
