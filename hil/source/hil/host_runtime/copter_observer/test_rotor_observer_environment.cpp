#include "rotor_observer_win32.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
using namespace gpenmpc_rotor_observer;
std::vector<std::string> names;
void check(bool x,const char*n){if(!x){std::cerr<<n<<'\n';std::exit(2);}names.emplace_back(n);}
void set(const char*n,const char*v){if(!SetEnvironmentVariableA(n,v))std::exit(3);}
int main(){
    const char *keys[]={"GPENMPC_ROTOR_OBSERVER_ENABLE","GPENMPC_ROTOR_OBSERVER_IP","GPENMPC_ROTOR_OBSERVER_PORT", 
        "GPENMPC_ROTOR_OBSERVER_SESSION","GPENMPC_ROTOR_OBSERVER_DLL_SHA256"};
    for(const auto *k:keys)set(k,nullptr);
    Config c;check(environment_config(c)&&!c.enabled,"absent_default_disabled");
    set(keys[0],"0");check(environment_config(c)&&!c.enabled,"explicit_zero_disabled");
    set(keys[0],"true");check(!environment_config(c),"nonexact_enable_rejected");
    set(keys[0],"1");check(!environment_config(c),"enable_without_binding_rejected");
    set(keys[1],"127.0.0.1");set(keys[2],"20991");set(keys[3],"73");
    set(keys[4],"0102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20");
    check(environment_config(c)&&c.enabled&&c.port==20991&&c.session==73&&c.dll_sha[31]==32,"complete_localhost_binding");
    set(keys[1],"localhost");check(!environment_config(c),"dns_name_not_allowed");
    set(keys[1],"0.0.0.0");check(!environment_config(c),"any_interface_not_allowed");
    set(keys[1],"192.168.1.2");check(!environment_config(c),"remote_not_allowed");set(keys[1],"127.0.0.1");
    set(keys[2],"65536");check(!environment_config(c),"port_overflow");
    set(keys[2],"0");check(!environment_config(c),"port_zero");
    set(keys[2],"20991x");check(!environment_config(c),"port_suffix");set(keys[2],"20991");
    set(keys[3],"18446744073709551616");check(!environment_config(c),"session_overflow");
    set(keys[3],"18446744073709551615");check(environment_config(c)&&c.session==UINT64_MAX,"full_uint64_session_preserved");
    set(keys[3],"0");check(!environment_config(c),"session_zero");set(keys[3],"73");
    set(keys[4],"0102");check(!environment_config(c),"short_digest");
    set(keys[4],"g102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f20");check(!environment_config(c),"nonhex_digest");
    set(keys[4],"0000000000000000000000000000000000000000000000000000000000000000");check(!environment_config(c),"zero_digest");
    for(const auto *k:keys)set(k,nullptr);
    std::cout<<"{\"status\":\"PASS_ENV_PARSER_NO_SOCKETS\",\"passed\":"<<names.size()<<",\"total\":"<<names.size()<<",\"tests\":[";
    for(std::size_t k=0;k<names.size();++k){if(k)std::cout<<',';std::cout<<'\"'<<names[k]<<'\"';}
    std::cout<<"],\"real_socket_opens\":0,\"COM_open\":0,\"board_actions\":0}\n";
}
