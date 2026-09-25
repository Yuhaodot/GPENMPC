#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <initializer_list>

// Only the actual parser/dispatch function is under test. These substitutes
// record selection; no PX4 task, stream, socket or control producer exists.
#define DEFAULT_DEVICE_NAME "/dev/ttyS1"
#define OK 0
#define PX4_WARN(...) ((void)0)
static unsigned usage_calls{}, configure_calls{};
static float configured_rate{};
static std::string configured_stream;
static void usage() { ++usage_calls; }
static int px4_get_parameter_value(const char *text, int &out)
{
    char *end{}; const long value=std::strtol(text,&end,10);
    if(!end||*end||value<0||value>65535)return -1;
    out=static_cast<int>(value);return 0;
}
class Mavlink {
public:
    static int stream_command(int,char *[]);
    static Mavlink *get_instance_for_device(const char *) { static Mavlink value;return &value; }
    static Mavlink *get_instance_for_network_port(unsigned short) { static Mavlink value;return &value; }
    void configure_stream_threadsafe(const char *name,float rate)
    { ++configure_calls;configured_stream=name;configured_rate=rate; }
};

#include "actual_stream_command.inc"

static unsigned checks{},failed{};
static void check(const char *name,std::initializer_list<const char *> args,
                  bool accept,float rate=0.0f,const char *stream="GPENMPC_LOCAL_WIRE")
{
    usage_calls=configure_calls=0;configured_stream.clear();configured_rate=123.0f;
    std::vector<std::string> strings;
    for(const char *arg:args)strings.emplace_back(arg);
    std::vector<char *> argv;
    for(auto &arg:strings)argv.push_back(&arg[0]);
    argv.push_back(nullptr);
    const int rc=Mavlink::stream_command(static_cast<int>(strings.size()),argv.data());
    const bool pass=accept ? rc==0&&configure_calls==1&&usage_calls==0&&
        configured_rate==rate&&configured_stream==stream : rc!=0&&configure_calls==0;
    ++checks;if(!pass)++failed;
    std::printf("{\"name\":\"%s\",\"pass\":%s,\"return_code\":%d,\"configure_calls\":%u,\"rate\":%.9g}%s",
        name,pass?"true":"false",rc,configure_calls,static_cast<double>(configured_rate),checks==12?"":",");
}
int main()
{
    std::printf("{\"cases\":[");
    check("exact_local_minus_one",{"mavlink","stream","-d","/dev/ttyACM0","-s","GPENMPC_LOCAL_WIRE","-r","-1"},true,-1.0f);
    check("other_stream_minus_one_rejected",{"mavlink","stream","-d","/dev/ttyACM0","-s","HEARTBEAT","-r","-1"},false);
    check("other_device_minus_one_rejected",{"mavlink","stream","-d","/dev/ttyS1","-s","GPENMPC_LOCAL_WIRE","-r","-1"},false);
    check("explicit_minus_two_rejected",{"mavlink","stream","-d","/dev/ttyACM0","-s","GPENMPC_LOCAL_WIRE","-r","-2"},false);
    check("other_negative_rejected",{"mavlink","stream","-d","/dev/ttyACM0","-s","GPENMPC_LOCAL_WIRE","-r","-0.5"},false);
    check("local_omitted_rate_uses_default",{"mavlink","stream","-d","/dev/ttyACM0","-s","GPENMPC_LOCAL_WIRE"},true,-2.0f);
    check("other_omitted_rate_uses_default",{"mavlink","stream","-d","/dev/ttyS1","-s","HEARTBEAT"},true,-2.0f,"HEARTBEAT");
    check("zero_still_disables",{"mavlink","stream","-d","/dev/ttyACM0","-s","GPENMPC_LOCAL_WIRE","-r","0"},true,0.0f);
    check("positive_still_selected",{"mavlink","stream","-d","/dev/ttyACM0","-s","GPENMPC_LOCAL_WIRE","-r","20"},true,20.0f);
    check("implicit_device_minus_one_rejected",{"mavlink","stream","-s","GPENMPC_LOCAL_WIRE","-r","-1"},false);
    check("udp_minus_one_rejected",{"mavlink","stream","-u","14550","-s","GPENMPC_LOCAL_WIRE","-r","-1"},false);
    check("other_positive_unchanged",{"mavlink","stream","-u","14550","-s","HEARTBEAT","-r","5"},true,5.0f,"HEARTBEAT");
    std::printf("],\"checks\":%u,\"failed\":%u,\"pass\":%s}\n",checks,failed,failed?"false":"true");
    return failed?1:0;
}
