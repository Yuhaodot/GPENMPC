#pragma once
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <windows.h>
#include "rotor_observer.hpp"

namespace gpenmpc_rotor_observer {
class LocalhostSink {
public:
    bool open(const Config &config) noexcept {
        if(!valid_config(config)||socket_!=INVALID_SOCKET)return false;
        WSADATA data{};
        if(WSAStartup(MAKEWORD(2,2),&data)!=0)return false;
        wsa_=true;socket_=::socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
        if(socket_==INVALID_SOCKET){close();return false;}
        u_long nonblocking=1;
        if(ioctlsocket(socket_,FIONBIO,&nonblocking)!=0){close();return false;}
        target_={};target_.sin_family=AF_INET;target_.sin_port=htons(config.port);
        target_.sin_addr.s_addr=htonl(0x7f000001U);return true;
    }
    int send(const std::uint8_t *bytes,std::size_t size) noexcept {
        if(socket_==INVALID_SOCKET||size!=packet_size)return -1;
        // Single nonblocking attempt, no wait/retry/reopen/background producer.
        return ::sendto(socket_,reinterpret_cast<const char*>(bytes),static_cast<int>(size),0,
            reinterpret_cast<const sockaddr*>(&target_),sizeof(target_));
    }
    void close() noexcept {
        if(socket_!=INVALID_SOCKET){closesocket(socket_);socket_=INVALID_SOCKET;}
        if(wsa_){WSACleanup();wsa_=false;}
    }
    ~LocalhostSink(){close();}
private:
    SOCKET socket_{INVALID_SOCKET};sockaddr_in target_{};bool wsa_{false};
};
// This process-local opt-in is supplied by the independently verifying outer
// loader BEFORE starting CopterSim. It is read exactly once. The expected DLL
// digest is NOT embedded in that same DLL and is NOT a signature/attestation.
inline bool read_env(const char *name,char *out,DWORD cap,bool &present) noexcept {
    const DWORD n=GetEnvironmentVariableA(name,out,cap);present=n!=0;
    return n<cap;
}
inline bool decimal(const char *s,std::uint64_t &v) noexcept {
    v=0;if(*s=='\0')return false;
    for(;*s;++s){if(*s<'0'||*s>'9')return false;const auto d=static_cast<unsigned>(*s-'0');
        if(v>(UINT64_MAX-d)/10)return false;v=v*10+d;}return true;
}
inline int hex_digit(char x)noexcept {
    if(x>='0'&&x<='9')return x-'0';if(x>='a'&&x<='f')return x-'a'+10;
    if(x>='A'&&x<='F')return x-'A'+10;return -1;
}
inline bool environment_config(Config &c) noexcept {
    char enabled[8]{};bool present=false;
    if(!read_env("GPENMPC_ROTOR_OBSERVER_ENABLE",enabled,sizeof(enabled),present))return false;
    if(!present||std::strcmp(enabled,"0")==0){c.enabled=false;return true;}
    if(std::strcmp(enabled,"1")!=0)return false;
    char address[32]{},port[16]{},session[32]{},sha[80]{};
    if(!read_env("GPENMPC_ROTOR_OBSERVER_IP",address,sizeof(address),present)||!present||std::strcmp(address,"127.0.0.1")!=0)return false;
    if(!read_env("GPENMPC_ROTOR_OBSERVER_PORT",port,sizeof(port),present)||!present)return false;
    if(!read_env("GPENMPC_ROTOR_OBSERVER_SESSION",session,sizeof(session),present)||!present)return false;
    if(!read_env("GPENMPC_ROTOR_OBSERVER_DLL_SHA256",sha,sizeof(sha),present)||!present||std::strlen(sha)!=64)return false;
    std::uint64_t p=0;if(!decimal(port,p)||p==0||p>65535||!decimal(session,c.session)||c.session==0)return false;
    c.port=static_cast<std::uint16_t>(p);c.ipv4_host_order=0x7f000001U;c.enabled=true;
    for(std::size_t k=0;k<32;++k){const int a=hex_digit(sha[2*k]),b=hex_digit(sha[2*k+1]);
        if(a<0||b<0)return false;c.dll_sha[k]=static_cast<std::uint8_t>(a*16+b);}
    return valid_config(c);
}
}
