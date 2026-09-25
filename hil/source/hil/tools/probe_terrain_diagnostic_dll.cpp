// Probe the DLL ABI offline.
// Run: exe old004.dll new005.dll trace.csv paired_outputs.bin
// Binary LE header: uint32 [0x4d365450,1,2560,319]. Each record:
// uint32 [case_index_1based,step_0based], double inputTerrain[15], then
// old and new [vehicle60,sensor30,gps30,diagnostic32], preserving raw bits.
#define NOMINMAX
#include <windows.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>

namespace {
template<class T> T symbol(HMODULE h,const char* name) {
    auto p=GetProcAddress(h,name);if(!p)throw std::runtime_error(name);
    return reinterpret_cast<T>(p);
}
struct Frame {std::array<double,60> v{};std::array<double,30> s{},g{};std::array<double,32> d{};};
struct Module {
    HMODULE h=nullptr;bool initialized=false;
    void (*reset)()=nullptr;void (*destroy)()=nullptr;void (*step)()=nullptr;
    void (*position)(const double*,const double*)=nullptr;void (*gps)(const double*)=nullptr;
    void (*pwm)(const double*)=nullptr;void (*terrain)(double*)=nullptr;
    void (*vehicle)(double*,int)=nullptr;void (*sensor)(double*,int)=nullptr;
    void (*gpsOut)(double*,int)=nullptr;void (*diagnostic)(double*)=nullptr;double (*period)()=nullptr;
    explicit Module(const wchar_t* path) {
        h=LoadLibraryW(path);if(!h)throw std::runtime_error("LoadLibraryW");
        try {
            reset=symbol<void(*)()>(h,"DllReInitModel");destroy=symbol<void(*)()>(h,"DllDestroyModel");
            step=symbol<void(*)()>(h,"Dllstep");
            position=symbol<void(*)(const double*,const double*)>(h,"DllInitPosAngState");
            gps=symbol<void(*)(const double*)>(h,"DllInitGpsPos");pwm=symbol<void(*)(const double*)>(h,"DllInputPWMs");
            terrain=symbol<void(*)(double*)>(h,"DllTerrainIn15d");
            vehicle=symbol<void(*)(double*,int)>(h,"DlloutVehileInfo60d");
            sensor=symbol<void(*)(double*,int)>(h,"DlloutHILSensor30d");
            gpsOut=symbol<void(*)(double*,int)>(h,"DlloutHILGPS30d");
            diagnostic=symbol<void(*)(double*)>(h,"DllOutCopterData");
            period=symbol<double(*)()>(h,"DllGetStep0");
            symbol<void(*)()>(h,"?DllCreatModel@@YAXXZ")();
        } catch(...) {FreeLibrary(h);h=nullptr;throw;}
    }
    ~Module() {if(h){if(initialized&&destroy)destroy();FreeLibrary(h);}}
    Module(const Module&)=delete;Module& operator=(const Module&)=delete;
    void begin(double z) {
        double p[3]={0,0,z},a[3]={0,0,0},origin[3]={40.1540302,116.2593683,50};
        position(p,a);gps(origin);reset();initialized=true;
        if(std::abs(period()-.001)>1e-12)throw std::runtime_error("DllGetStep0 not 1ms");
    }
    Frame tick(double u,const std::array<double,15>& input) {
        double controls[16]={};for(int j=0;j<6;++j)controls[j]=u;
        auto terrainSnapshot=input;terrain(terrainSnapshot.data());pwm(controls);step();
        Frame o;vehicle(o.v.data(),60);sensor(o.s.data(),30);gpsOut(o.g.data(),30);diagnostic(o.d.data());return o;
    }
};
double fromBits(std::uint64_t bits){double x;std::memcpy(&x,&bits,8);return x;}
enum Kind {NORMAL,HEIGHT_CHANGE,NONFINITE,COLD_NONFINITE,EXPLICIT_RESET,RELOADED_INSTANCE};
struct Case {std::string name;Kind kind;int channel=-1,flavor=-1,rows=60;};
std::vector<Case> cases() {
    std::vector<Case> out={{"NORMAL_3000",NORMAL,-1,-1,3000},{"HEIGHT_ONE_ULP",HEIGHT_CHANGE}};
    const char* flavors[]={"NAN_PAYLOAD123","PLUS_INF","MINUS_INF"};
    for(int channel=0;channel<15;++channel)for(int f=0;f<3;++f)
        out.push_back({"CHANNEL_"+std::to_string(channel+1)+"_"+flavors[f],NONFINITE,channel,f});
    out.push_back({"COLD_NONFINITE_THEN_GOOD",COLD_NONFINITE,14,0});
    out.push_back({"EXPLICIT_RESET_AFTER_FAULT",EXPLICIT_RESET});
    out.push_back({"UNLOAD_RELOAD_NEW_INSTANCE",RELOADED_INSTANCE});return out;
}
std::array<double,15> terrainInput(const Case& c,int k) {
    std::array<double,15> a{};
    // Use distinct finite fixture metadata to identify all 15 captured slots.
    if(c.kind!=NORMAL)for(int j=1;j<15;++j)a[j]=(j+1)*.03125;
    if(c.kind!=NORMAL)a[13]=fromBits(UINT64_C(0x8000000000000000));
    if(c.kind==HEIGHT_CHANGE){a[0]=1;if(k>=20&&k<30)a[0]=std::nextafter(1.,INFINITY);}
    if(c.kind==NONFINITE&&k>=20&&k<30) {
        const std::uint64_t values[]={UINT64_C(0x7ff800000000007b),UINT64_C(0x7ff0000000000000),UINT64_C(0xfff0000000000000)};
        a[c.channel]=fromBits(values[c.flavor]);
    }
    if(c.kind==COLD_NONFINITE&&k<10)a[14]=fromBits(UINT64_C(0xfff8000000000123));
    // Preserve the first fault after later invalid or restored inputs.
    if((c.kind==HEIGHT_CHANGE||c.kind==NONFINITE||c.kind==COLD_NONFINITE)&&k>=40)
        a[12]=fromBits(UINT64_C(0x7ff0000000000000));
    return a;
}
int expectedFirst(const Case& c){return c.kind==COLD_NONFINITE?0:((c.kind==HEIGHT_CHANGE||c.kind==NONFINITE)?20:-1);}
int expectedReason(const Case& c){return c.kind==HEIGHT_CHANGE?3:((c.kind==NONFINITE||c.kind==COLD_NONFINITE)?2:0);}
template<std::size_t N> double maxDifference(const std::array<double,N>& a,const std::array<double,N>& b) {
    double d=0;for(std::size_t j=0;j<N;++j){if(!std::isfinite(a[j])||!std::isfinite(b[j]))return INFINITY;d=std::max(d,std::abs(a[j]-b[j]));}return d;
}
template<std::size_t N> bool bitEqual(const std::array<double,N>& a,const std::array<double,N>& b){return std::memcmp(a.data(),b.data(),N*8)==0;}
void writeRaw(FILE* f,std::uint32_t ci,std::uint32_t step,const std::array<double,15>& in,const Frame& a,const Frame& b) {
    const std::uint32_t meta[]={ci,step};
    auto write=[&](const void* p,std::size_t size,std::size_t n){if(std::fwrite(p,size,n,f)!=n)throw std::runtime_error("raw write");};
    write(meta,4,2);write(in.data(),8,15);
    for(const Frame* p:{&a,&b}){write(p->v.data(),8,60);write(p->s.data(),8,30);write(p->g.data(),8,30);write(p->d.data(),8,32);}
}
struct File {FILE* f=nullptr;explicit File(const wchar_t* path){if(GetFileAttributesW(path)!=INVALID_FILE_ATTRIBUTES)throw std::runtime_error("output already exists");f=_wfopen(path,L"wb");if(!f)throw std::runtime_error("output open");}~File(){if(f)std::fclose(f);}};
}
int wmain(int argc,wchar_t** argv) {
    static_assert(sizeof(double)==8&&sizeof(std::uint32_t)==4,"binary ABI");
    if(argc!=5)return 2;
    try {
        File trace(argv[3]),raw(argv[4]);const std::uint32_t header[]={0x4d365450,1,2560,319};
        if(std::fwrite(header,4,4,raw.f)!=4)throw std::runtime_error("raw header");
        std::fprintf(trace.f,"case_index,case_name,step,instance_generation,expected_first_fault,expected_reason,failed,code,plant_time,capture,first_reason,locked_height,vehicle_max_abs_difference,sensor_max_abs_difference,gps_max_abs_difference,vehicle_bit_equal,sensor_bit_equal,gps_bit_equal,prefix7_bit_equal,issue_mask\n");
        auto old=std::make_unique<Module>(argv[1]);auto current=std::make_unique<Module>(argv[2]);
        if(old->h==current->h)throw std::runtime_error("old/new DLL module alias");
        const auto specs=cases();std::size_t rows=0,badRows=0;int generation=1;
        for(std::size_t ci=0;ci<specs.size();++ci) {
            const Case& c=specs[ci];
            if(c.kind==RELOADED_INSTANCE){old.reset();current.reset();old=std::make_unique<Module>(argv[1]);current=std::make_unique<Module>(argv[2]);++generation;if(old->h==current->h)throw std::runtime_error("reloaded DLL alias");}
            double locked=c.kind==HEIGHT_CHANGE?1.:0.;old->begin(locked);current->begin(locked);
            const int first=expectedFirst(c),reason=expectedReason(c);
            std::array<double,15> firstTerrain{};std::array<double,18> firstCapture{};double frozenTime=0;
            for(int k=0;k<c.rows;++k) {
                double u=0;if(c.kind==NORMAL){double t=k*.001;u=t<.5?0:(t<1.5?.7:.61);}
                auto input=terrainInput(c,k);Frame a=old->tick(u,input),b=current->tick(u,input);
                writeRaw(raw.f,static_cast<std::uint32_t>(ci+1),static_cast<std::uint32_t>(k),input,a,b);
                const double dv=maxDifference(a.v,b.v),ds=maxDifference(a.s,b.s),dg=maxDifference(a.g,b.g);
                bool prefix=std::memcmp(a.d.data(),b.d.data(),7*8)==0;std::uint32_t issue=0;
                if(!prefix)issue|=1;if(dv>1e-9)issue|=2;if(ds>1e-9)issue|=4;if(dg>1e-9)issue|=8;
                for(int j=0;j<7;++j)if(!std::isfinite(b.d[j]))issue|=16;
                if(b.d[6]!=1||b.d[25]!=1)issue|=16;
                for(int j=26;j<32;++j)if(b.d[j]!=0)issue|=16;
                for(int j=7;j<32;++j)if(a.d[j]!=0)issue|=2048;
                if(first<0||k<first) {
                    if(b.d[0]!=0||b.d[1]!=0)issue|=32;
                    for(int j=7;j<25;++j)if(b.d[j]!=0)issue|=32;
                } else {
                    if(k==first){firstTerrain=input;std::copy(b.d.begin()+7,b.d.begin()+25,firstCapture.begin());frozenTime=b.d[2];}
                    if(b.d[0]!=1||b.d[1]!=4||b.d[7]!=1||b.d[8]!=reason||b.d[9]!=locked)issue|=64;
                    if(std::memcmp(b.d.data()+10,firstTerrain.data(),15*8)!=0)issue|=128;
                    if(std::memcmp(b.d.data()+7,firstCapture.data(),18*8)!=0||b.d[2]!=frozenTime)issue|=256;
                }
                if(k==0&&(std::abs(b.v[2]-.001)>1e-12||b.d[2]!=0))issue|=512;
                if(k==0&&first!=0&&(b.d[0]!=0||std::abs(b.v[8]-locked)>1e-12))issue|=512;
                std::fprintf(trace.f,"%zu,%s,%d,%d,%d,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d,%d,%d,%d,%u\n",ci+1,c.name.c_str(),k,generation,first,reason,b.d[0],b.d[1],b.d[2],b.d[7],b.d[8],b.d[9],dv,ds,dg,bitEqual(a.v,b.v),bitEqual(a.s,b.s),bitEqual(a.g,b.g),prefix,issue);
                ++rows;if(issue){++badRows;if(badRows==1)std::printf("FIRST_FAILURE case=%s step=%d mask=%u\n",c.name.c_str(),k,issue);}
            }
        }
        if(std::fflush(raw.f)!=0||std::fflush(trace.f)!=0)throw std::runtime_error("output flush");
        std::printf("%s_TERRAIN_EXTENSION_DLL_PROBE cases=%zu rows=%zu bad_rows=%zu instance_generations=%d hardware=0\n",badRows?"FAIL":"PASS",specs.size(),rows,badRows,generation);
        return badRows?5:0;
    } catch(const std::exception& error){std::printf("PROBE_EXCEPTION %s\n",error.what());return 6;}
}
