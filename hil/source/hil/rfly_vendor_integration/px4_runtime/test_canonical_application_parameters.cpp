#include "../application_parameters/CanonicalApplicationParameters.hpp"
#include <cstdio>
#include <cstring>
#include <cstdint>
int main(int argc,char **argv){
    if(argc!=2)return 2;
    FILE *f=std::fopen(argv[1],"rb");if(!f)return 3;
    unsigned char bytes[420]{};const auto n=std::fread(bytes,1,sizeof(bytes),f);const int tail=std::fgetc(f);std::fclose(f);
    unsigned checks=0,failed=0;auto check=[&](bool ok){++checks;if(!ok)++failed;};
    check(n==420&&tail==EOF&&std::memcmp(bytes,"RAP1",4)==0);
    const auto p=gpenmpc_rfly_px4::canonical_application_parameters();unsigned offset=4;
    auto value=[&](double x){std::uint64_t observed=0,expected=0;std::memcpy(&expected,&x,8);
        for(unsigned j=0;j<8;++j)observed=(observed<<8)|bytes[offset++];
        check(observed==expected);};
    for(double x:p.kp)value(x);
    for(double x:p.kd)value(x);
    for(double x:p.kr)value(x);
    for(double x:p.kw)value(x);
    for(double x:p.drag)value(x);
    for(double x:p.inertia)value(x);
    for(double x:p.pseudoinverse)value(x);
    value(p.baseMass);value(p.totalThrust);value(p.rotorUpper);value(p.maxTilt);check(offset==420);
    const auto golden=gpenmpc_rfly_px4::kCanonicalApplicationParameterSha;
    check(gpenmpc_rfly_execution::parameter_sha256(p)==golden);
    auto altered=p;altered.kp[0]+=0.01;
    const auto dishonest_self_hash=gpenmpc_rfly_execution::parameter_sha256(altered);
    check(dishonest_self_hash!=golden); // self-consistency is not canonical membership
    std::printf("{\"checks\":%u,\"failed\":%u,\"parameters\":52}\n",checks,failed);
    return failed?1:0;
}
