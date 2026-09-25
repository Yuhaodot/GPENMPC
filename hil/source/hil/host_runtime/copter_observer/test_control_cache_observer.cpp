#include "control_cache_observer.hpp"
#include <vector>
#include <cstdio>
#include <cstdlib>
using namespace gpenmpc_control_cache;
struct Sink{std::vector<Packet> packets;int returned=216;
    int send(const std::uint8_t *b,std::size_t n){if(n!=216)std::abort();Packet p{};std::memcpy(p.data(),b,n);packets.push_back(p);return returned;}};
unsigned total=0;void check(bool b){++total;if(!b){std::fprintf(stderr,"check %u failed\n",total);std::exit(1);}}
std::uint64_t read(const Packet &p,unsigned at,unsigned n){std::uint64_t r=0;for(unsigned j=0;j<n;++j)r|=std::uint64_t(p[at+j])<<(8*j);return r;}
int main(){
    gpenmpc_rotor_observer::Config c{};c.enabled=true;c.port=31000;c.session=73;c.dll_sha.fill(5);
    gpenmpc_rotor_observer::Sample s{};s.session=73;s.generation=1;s.sim_time_s=.01;s.valid=true;
    double values[16]{};values[4]=.125;auto i=capture(values,1);
    Sink sink;Observer<Sink> o(sink);o.after_step(i,1,s,c);
    check(o.stats().sent==1&&o.stats().failure==0);check(sink.packets.size()==1);
    const auto p=sink.packets[0];check(std::memcmp(p.data(),"M6CACHE1",8)==0);
    check(read(p,8,2)==1&&read(p,10,2)==1&&read(p,12,4)==216);
    check(read(p,16,8)==73&&read(p,24,8)==1&&read(p,40,8)==1);
    for(unsigned k=0;k<16;++k){std::uint64_t b=0;std::memcpy(&b,&values[k],8);check(read(p,48+8*k,8)==b);}
    check(read(p,208,4)==gpenmpc_rotor_observer::crc32(p.data(),208));check(read(p,212,4)==0);
    values[4]=0;o.after_step(capture(values,2),2,s,c);
    check(o.stats().sent==1&&o.stats().duplicates==1); // Held prior step; zero was not consumed.
    s.generation=2;s.sim_time_s=.02;o.after_step(capture(values,2),2,s,c);
    check(o.stats().sent==2&&o.stats().accepted_steps==2);
    for(unsigned k=0;k<16;++k)check(read(sink.packets[1],48+8*k,8)==0);
    {Sink q;Observer<Sink>a(q);auto t=s;t.generation=1;t.sim_time_s=.01;a.after_step(capture(values,0),0,t,c);check(a.stats().failure==3&&q.packets.empty());}
    {Sink q;Observer<Sink>a(q);auto t=s;t.generation=1;t.sim_time_s=.01;a.after_step(capture(values,1),2,t,c);check(a.stats().failure==3);}
    {Sink q;Observer<Sink>a(q);auto t=s;t.generation=1;t.sim_time_s=.01;auto v=capture(values,1);v.controls[0]=std::numeric_limits<double>::quiet_NaN();a.after_step(v,1,t,c);check(a.stats().failure==4);a.after_step(i,1,t,c);check(q.packets.empty());}
    {Sink q;Observer<Sink>a(q);a.after_step(i,1,s,c);check(a.stats().failure==2);}
    {Sink q;Observer<Sink>a(q);auto t=s;t.generation=1;t.sim_time_s=.01;t.session=74;a.after_step(i,1,t,c);check(a.stats().failure==2);}
    {Sink q;q.returned=215;Observer<Sink>a(q);auto t=s;t.generation=1;t.sim_time_s=.01;a.after_step(i,1,t,c);check(a.stats().failure==5&&a.stats().sent==0&&a.stats().send_attempts==1);}
    {Sink q;Observer<Sink>a(q);auto t=s;t.valid=false;a.after_step(i,1,t,c);check(a.stats().sent==0&&a.stats().failure==0);}
    {Sink q;Observer<Sink>a(q);auto x=c;x.enabled=false;a.after_step(i,1,s,x);check(a.stats().sent==0&&a.stats().failure==0);}
    {Sink q;Observer<Sink>a(q);auto t=s;t.failed=true;a.after_step(i,1,t,c);check(a.stats().failure==1);}
    std::printf("{\"passed\":%u,\"total\":%u,\"real_socket_opens\":0,\"model_instances\":0,\"board_actions\":0}\n",total,total);
}
