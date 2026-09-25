#include "rotor_observer.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cstdlib>
using namespace gpenmpc_rotor_observer;
struct FakeSink {
    unsigned opens{0},sends{0},closes{0};bool open_ok{true};int result{128};Packet last{};
    bool open(const Config&){++opens;return open_ok;}
    int send(const std::uint8_t *p,std::size_t n){++sends;std::memcpy(last.data(),p,n);return result;}
    void close(){++closes;}
};
Config config(){Config c;c.enabled=true;c.port=20991;c.session=73;
    for(unsigned k=0;k<32;++k)c.dll_sha[k]=static_cast<std::uint8_t>(k+1);return c;}
Sample sample(){Sample s;s.rotor_n={0,1.25,2.5,10,20,32.145727009134916};
    s.sim_time_s=.01;s.generation=1;s.session=73;s.valid=true;return s;}
std::vector<std::string> names;
void check(bool pass,const char *name){if(!pass){std::cerr<<"FAIL: "<<name<<'\n';std::exit(2);}names.emplace_back(name);}
int main(){
    const auto c=config();const auto s=sample();Packet cross{};
    check(from_generated_words(0,0)==0,"generated_uint64_zero");
    check(from_generated_words(1,0)==1,"generated_uint64_one");
    check(from_generated_words(0x89abcdefU,0x01234567U)==0x0123456789abcdefULL,"generated_uint64_exact_high_bits");
    check(from_generated_words(0xffffffffU,0xffffffffU)==UINT64_MAX,"generated_uint64_full_width");
    {FakeSink sink;Observer<FakeSink> o(sink);o.after_step(s);check(sink.opens==0&&sink.sends==0,"default_disabled_no_socket");}
    {FakeSink sink;Observer<FakeSink> o(sink);Config d;o.configure(d);o.after_step(s);
        check(sink.opens==0&&sink.sends==0,"explicit_disabled_no_socket");}
    {FakeSink sink;Observer<FakeSink> o(sink);o.configure(c);o.after_step(s);cross=sink.last;
        check(sink.opens==1&&sink.sends==1&&o.stats().sent_packets==1,"enabled_single_send");
        check(o.stats().last_sent_generation==1,"source_generation_preserved");
        o.after_step(s);check(sink.sends==1&&o.stats().duplicates==1,"duplicate_not_sent");
        auto n=s;n.generation=2;n.sim_time_s=.02;o.after_step(n);
        check(sink.sends==2&&o.stats().last_sent_generation==2,"next_generation_one_send");
        o.destroy();check(sink.closes==1&&o.destroyed(),"destroy_closes_socket");
        o.destroy();check(sink.closes==1,"destroy_idempotent");
        n.generation=3;n.sim_time_s=.03;o.after_step(n);check(sink.sends==2,"destroy_prevents_future_send");}
    {FakeSink sink;Observer<FakeSink> o(sink);o.configure(c);auto f=s;f.failed=true;o.after_step(f);o.after_step(s);
        check(sink.sends==0&&sink.closes==1&&o.stats().failure==static_cast<unsigned>(Failure::model_failed),"failed_step_no_send_latched");}
    {FakeSink sink;Observer<FakeSink> o(sink);o.configure(c);auto f=s;f.valid=false;o.after_step(f);o.after_step(s);
        check(sink.sends==1&&o.stats().invalid_steps_skipped==1,"unaccepted_step_not_counted");}
    for(int result:{-1,0,12,127}){FakeSink sink;sink.result=result;Observer<FakeSink> o(sink);o.configure(c);o.after_step(s);
        auto n=s;n.generation=2;n.sim_time_s=.02;o.after_step(n);
        check(sink.sends==1&&sink.closes==1&&o.stats().sent_packets==0&&o.stats().last_sent_generation==0&&
            o.stats().last_observed_generation==1,"send_error_partial_no_retry_no_new_sample");}
    {FakeSink sink;sink.open_ok=false;Observer<FakeSink> o(sink);o.configure(c);o.after_step(s);
        check(sink.opens==1&&sink.sends==0&&o.stats().failure!=0,"open_error_no_send");}
    {FakeSink sink;Observer<FakeSink> o(sink);auto bad=c;bad.ipv4_host_order=0x08080808;o.configure(bad);o.after_step(s);
        check(sink.opens==0&&sink.sends==0,"nonlocalhost_rejected_before_open");}
    {FakeSink sink;Observer<FakeSink> o(sink);auto bad=c;bad.dll_sha={};o.configure(bad);
        check(sink.opens==0,"missing_dll_binding_no_open");}
    {FakeSink sink;Observer<FakeSink> o(sink);o.configure(c);auto bad=s;bad.session++;o.after_step(bad);
        check(sink.sends==0&&sink.closes==1,"session_mismatch");}
    {FakeSink sink;Observer<FakeSink> o(sink);o.configure(c);auto bad=s;bad.rotor_n[0]=-1;o.after_step(bad);
        check(sink.sends==0,"negative_rotor_not_sent");}
    {FakeSink sink;Observer<FakeSink> o(sink);o.configure(c);auto bad=s;bad.rotor_n[0]=NAN;o.after_step(bad);
        check(sink.sends==0,"nonfinite_rotor_not_sent");}
    {FakeSink sink;Observer<FakeSink> o(sink);o.configure(c);auto bad=s;bad.sim_time_s=INFINITY;o.after_step(bad);
        check(sink.sends==0,"nonfinite_time_not_sent");}
    {FakeSink sink;Observer<FakeSink> o(sink);o.configure(c);o.after_step(s);auto bad=s;bad.rotor_n[0]=1;o.after_step(bad);
        check(sink.sends==1&&o.stats().failure==static_cast<unsigned>(Failure::duplicate_conflict),"duplicate_conflict_latched");}
    {FakeSink sink;Observer<FakeSink> o(sink);o.configure(c);o.after_step(s);auto bad=s;bad.generation=3;bad.sim_time_s=.03;o.after_step(bad);
        check(sink.sends==1&&o.stats().missing_generations==1,"gap_accounted_not_synthesized");}
    {FakeSink sink;Observer<FakeSink> o(sink);o.configure(c);auto bad=s;bad.generation=2;o.after_step(bad);
        check(sink.sends==0,"initial_generation_must_be_one");}
    {FakeSink sink;Observer<FakeSink> o(sink);o.configure(c);o.after_step(s);o.before_model_reset();o.after_step(s);
        check(sink.sends==1&&sink.closes==1,"model_reset_after_sample_latched");}
    {FakeSink sink;Observer<FakeSink> o(sink);o.configure(c);o.configure(c);o.after_step(s);
        check(sink.opens==1&&sink.sends==0&&sink.closes==1,"no_reconfigure_mid_session");}
    // The same-owner step updates its own outputs once. The observer receives
    // const copies; neither enabled nor disabled may mutate any output byte.
    struct FakeModel {double original[152]{};Sample observed{};int step_count{0};
        void step(){++step_count;for(unsigned k=0;k<152;++k)original[k]=k+.125;observed=sample();}};
    for(bool enabled:{false,true}){FakeModel mmc;FakeSink sink;Observer<FakeSink> o(sink);auto cfg=c;cfg.enabled=enabled;o.configure(cfg);
        mmc.step();double before[152];std::memcpy(before,mmc.original,sizeof before);o.after_step(mmc.observed);
        check(mmc.step_count==1&&std::memcmp(before,mmc.original,sizeof before)==0,"one_same_owner_step_original_outputs_unchanged");}
    check(crc32(reinterpret_cast<const std::uint8_t*>("123456789"),9)==0xcbf43926U,"ieee_crc_known_vector");
    std::cout<<"{\"status\":\"PASS_CPP_SAME_OWNER_STUB_ONLY\",\"passed\":"<<names.size()<<",\"total\":"<<names.size()<<",\"tests\":[";
    for(std::size_t k=0;k<names.size();++k){if(k)std::cout<<',';std::cout<<'\"'<<names[k]<<'\"';}
    std::cout<<"],\"packet_hex\":\""<<std::hex<<std::setfill('0');for(auto b:cross)std::cout<<std::setw(2)<<static_cast<unsigned>(b);
    std::cout<<"\",\"real_socket_opens\":0,\"COM_open\":0,\"board_actions\":0,\"CopterSim_started\":0,\"live_loaded\":false}\n";
}
