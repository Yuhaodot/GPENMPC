#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

// Same-owner, observation-only sender. No model object, state propagation,
// control, truth position or socket implementation lives in this class.
namespace gpenmpc_rotor_observer {
constexpr std::size_t packet_size = 128;
using Packet = std::array<std::uint8_t, packet_size>;
// Represent uint64m_T with least-significant uint32 chunks first,
// preserving all 64 identity and generation bits.
inline std::uint64_t from_generated_words(std::uint32_t low,std::uint32_t high) noexcept {
    return static_cast<std::uint64_t>(low) | (static_cast<std::uint64_t>(high)<<32);
}
struct Config {
    bool enabled{false};
    std::uint32_t ipv4_host_order{0x7f000001U};
    std::uint16_t port{0};
    std::uint64_t session{0};
    std::array<std::uint8_t, 32> dll_sha{}; // verified-loader binding, NOT attestation
};
struct Sample {
    std::array<double, 6> rotor_n{};
    double sim_time_s{0};
    std::uint64_t generation{0};
    std::uint64_t session{0};
    bool valid{false};
    bool failed{false};
};
enum class Failure : std::uint64_t {
    none=0, invalid_config=1, reconfigure=2, socket_open=3, model_failed=4,
    invalid_sample=5, session_mismatch=6, initial_generation=7,
    generation_reversed=8, generation_gap=9, duplicate_conflict=10,
    time_not_advancing=11, send_error_or_partial=12, unexpected_reset=13,
    destroyed=14, invalid_environment=15
};
struct Stats {
    std::uint64_t enabled{0}, failure{0}, open_attempts{0}, send_attempts{0};
    std::uint64_t sent_packets{0}, duplicates{0}, invalid_steps_skipped{0};
    std::uint64_t last_observed_generation{0}, last_sent_generation{0};
    std::uint64_t close_calls{0}, missing_generations{0};
};
inline std::uint32_t crc32(const std::uint8_t *b, std::size_t n) noexcept {
    std::uint32_t c=0xffffffffU;
    for(std::size_t k=0;k<n;++k) {
        c^=b[k];
        for(unsigned j=0;j<8;++j) c=(c>>1)^((c&1U)?0xedb88320U:0U);
    }
    return ~c;
}
inline void put(Packet &p,std::size_t at,std::uint64_t v,std::size_t n) noexcept {
    for(std::size_t k=0;k<n;++k) p[at+k]=static_cast<std::uint8_t>((v>>(8*k))&255U);
}
inline void put_double(Packet &p,std::size_t at,double x) noexcept {
    static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559);
    std::uint64_t v=0; std::memcpy(&v,&x,sizeof(v)); put(p,at,v,8);
}
inline Packet encode(const Sample &s,const Config &c) noexcept {
    Packet p{}; const char magic[]="M6ROTOR1"; std::memcpy(p.data(),magic,8);
    put(p,8,1,2);put(p,10,1,2);put(p,12,packet_size,4);
    put(p,16,s.session,8);put(p,24,s.generation,8);put_double(p,32,s.sim_time_s);
    for(std::size_t k=0;k<6;++k) put_double(p,40+8*k,s.rotor_n[k]);
    std::memcpy(p.data()+88,c.dll_sha.data(),c.dll_sha.size());
    put(p,120,crc32(p.data(),120),4);return p;
}
inline bool valid_config(const Config &c) noexcept {
    bool nonzero=false;for(auto b:c.dll_sha)nonzero=nonzero||b!=0;
    return c.ipv4_host_order==0x7f000001U && c.port!=0 && c.session!=0 && nonzero;
}
// Sink interface: open(Config)->bool, send(uint8*,size)->int, close()->void.
// Production uses nonblocking localhost IO; tests inject a fake sink.
template<class Sink> class Observer {
public:
    explicit Observer(Sink &sink) noexcept:sink_(sink){}
    ~Observer(){close();}
    Observer(const Observer&)=delete;Observer&operator=(const Observer&)=delete;
    void configure(const Config &c) noexcept {
        if(destroyed_){fail(Failure::destroyed);return;}
        if(configured_){fail(Failure::reconfigure);return;}
        configured_=true;config_=c;stats_.enabled=c.enabled?1:0;
        if(!c.enabled)return;
        if(!valid_config(c)){fail(Failure::invalid_config);return;}
        ++stats_.open_attempts;
        if(!sink_.open(c)){fail(Failure::socket_open);return;}
        open_=true;
    }
    void after_step(const Sample &s) noexcept {
        if(destroyed_ || !configured_ || !config_.enabled || stats_.failure!=0)return;
        if(s.failed){fail(Failure::model_failed);return;}
        if(!s.valid){++stats_.invalid_steps_skipped;return;}
        if(s.session!=config_.session){fail(Failure::session_mismatch);return;}
        if(s.generation==0 || !std::isfinite(s.sim_time_s) || s.sim_time_s<0){fail(Failure::invalid_sample);return;}
        for(double v:s.rotor_n)if(!std::isfinite(v)||v<0){fail(Failure::invalid_sample);return;}
        if(stats_.last_observed_generation==0) {
            if(s.generation!=1){fail(Failure::initial_generation);return;}
        } else if(s.generation==stats_.last_observed_generation) {
            if(s.sim_time_s!=last_.sim_time_s || std::memcmp(s.rotor_n.data(),last_.rotor_n.data(),48)!=0)
                fail(Failure::duplicate_conflict);
            else ++stats_.duplicates;
            return;
        } else if(s.generation<stats_.last_observed_generation) {
            fail(Failure::generation_reversed);return;
        } else if(s.generation-stats_.last_observed_generation!=1) {
            stats_.missing_generations=s.generation-stats_.last_observed_generation-1;
            fail(Failure::generation_gap);return;
        } else if(s.sim_time_s<=last_.sim_time_s) {
            fail(Failure::time_not_advancing);return;
        }
        last_=s;stats_.last_observed_generation=s.generation;
        const Packet packet=encode(s,config_);++stats_.send_attempts;
        if(sink_.send(packet.data(),packet.size())!=static_cast<int>(packet.size())) {
            fail(Failure::send_error_or_partial);return;
        }
        ++stats_.sent_packets;stats_.last_sent_generation=s.generation;
    }
    void before_model_reset() noexcept {
        if(stats_.last_observed_generation!=0)fail(Failure::unexpected_reset);
    }
    void invalid_environment() noexcept {configured_=true;fail(Failure::invalid_environment);}
    void destroy() noexcept {close();destroyed_=true;}
    const Stats &stats()const noexcept{return stats_;}
    bool destroyed()const noexcept{return destroyed_;}
private:
    void close() noexcept {if(open_){sink_.close();open_=false;++stats_.close_calls;}}
    void fail(Failure f) noexcept {if(stats_.failure==0)stats_.failure=static_cast<std::uint64_t>(f);close();}
    Sink &sink_;Config config_{};Sample last_{};Stats stats_{};
    bool configured_{false},open_{false},destroyed_{false};
};
}
