#pragma once
#include "rotor_observer.hpp"

// Observe the input buffer consumed by the model step through the rotor sink.
namespace gpenmpc_control_cache {
constexpr std::size_t packet_size=216;
using Packet=std::array<std::uint8_t,packet_size>;
struct Input {
    std::array<double,16> controls{};
    std::uint64_t original_input_call_count{0};
};
struct Stats {
    std::uint64_t accepted_steps{0},sent{0},send_attempts{0},failure{0},duplicates{0};
};
inline Input capture(const double *values,std::uint64_t count) noexcept {
    Input r{};std::memcpy(r.controls.data(),values,sizeof(double)*16);
    r.original_input_call_count=count;return r;
}
inline void put(Packet &p,std::size_t at,std::uint64_t v,std::size_t n) noexcept {
    for(std::size_t k=0;k<n;++k)p[at+k]=static_cast<std::uint8_t>((v>>(8*k))&255U);
}
inline void put_double(Packet &p,std::size_t at,double x) noexcept {
    std::uint64_t bits=0;std::memcpy(&bits,&x,8);put(p,at,bits,8);
}
inline Packet encode(const Input &i,const gpenmpc_rotor_observer::Sample &s,
                     const gpenmpc_rotor_observer::Config &c) noexcept {
    Packet p{};std::memcpy(p.data(),"M6CACHE1",8);
    put(p,8,1,2);put(p,10,1,2);put(p,12,packet_size,4);
    put(p,16,s.session,8);put(p,24,s.generation,8);put_double(p,32,s.sim_time_s);
    put(p,40,i.original_input_call_count,8);
    for(std::size_t k=0;k<16;++k)put_double(p,48+8*k,i.controls[k]);
    std::memcpy(p.data()+176,c.dll_sha.data(),32);
    put(p,208,gpenmpc_rotor_observer::crc32(p.data(),208),4);return p;
}
template<class Sink> class Observer {
public:
    explicit Observer(Sink &shared) noexcept:sink_(shared){}
    void after_step(const Input &before,std::uint64_t count_after,
                    const gpenmpc_rotor_observer::Sample &s,
                    const gpenmpc_rotor_observer::Config &c) noexcept {
        if(stats_.failure||!c.enabled)return;
        if(!gpenmpc_rotor_observer::valid_config(c)||s.failed){fail(1);return;}
        if(!s.valid)return;
        // Distinguish a newly arrived input from a held accepted 10 ms output.
        if(s.generation==last_generation_&&last_generation_!=0){++stats_.duplicates;return;}
        if(s.generation!=last_generation_+1||s.session!=c.session||s.generation==0
           ||!std::isfinite(s.sim_time_s)||s.sim_time_s<0
           ||(last_generation_&&s.sim_time_s<=last_time_)){fail(2);return;}
        if(before.original_input_call_count==0||before.original_input_call_count!=count_after
           ||before.original_input_call_count<last_input_){fail(3);return;}
        for(double v:before.controls)if(!std::isfinite(v)){fail(4);return;}
        const auto p=encode(before,s,c);++stats_.send_attempts;
        if(sink_.send(p.data(),p.size())!=static_cast<int>(p.size())){fail(5);return;}
        last_generation_=s.generation;last_time_=s.sim_time_s;
        last_input_=before.original_input_call_count;++stats_.accepted_steps;++stats_.sent;
    }
    const Stats &stats()const noexcept{return stats_;}
private:
    void fail(std::uint64_t r)noexcept{if(!stats_.failure)stats_.failure=r;}
    Sink &sink_;Stats stats_{};std::uint64_t last_generation_{0},last_input_{0};double last_time_{-1};
};
}
