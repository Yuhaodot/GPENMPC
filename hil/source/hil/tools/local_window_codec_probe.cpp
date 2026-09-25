// Host RWW1 encoder probe over the RWI1 fixture.
#include "../rfly_vendor_integration/px4_wire/CanonicalLocalWindowWire.hpp"
#include <cstdio>
#include <cstring>
#include <vector>
namespace ww=gpenmpc_local_window_wire;
int wmain(int argc,wchar_t**argv){
    if(argc!=3)return 2;
    FILE*f=_wfopen(argv[1],L"rb");if(!f)return 3;
    char magic[4]{};std::uint32_t nw{},nq{};
    if(std::fread(magic,1,4,f)!=4||std::memcmp(magic,"RWI1",4)||
        std::fread(&nw,4,1,f)!=1||std::fread(&nq,4,1,f)!=1||nw>32||nq!=3765)return 4;
    FILE*out=_wfopen(argv[2],L"wb");if(!out)return 5;
    for(std::uint32_t j=0;j<nw;++j){
        std::vector<std::uint8_t>raw(ww::window_bytes);ww::Window w{};
        if(std::fread(raw.data(),1,raw.size(),f)!=raw.size()||!ww::copy_wire_to_window(raw.data(),raw.size(),0,w))return 6;
        ww::Binding b{};b.identity={0x1122334455667788ULL,42,1,1};b.leg_index=w.leg_index;
        for(unsigned k=0;k<8;++k){b.execution_session_sha256[k]=k+0x11223344;b.task_sha256[k]=k+1;b.configuration_sha256[k]=k+33;
            b.reference_asset_sha256[k]=(std::uint32_t(w.reference_asset_sha256[k*4])<<24)|(std::uint32_t(w.reference_asset_sha256[k*4+1])<<16)|
                (std::uint32_t(w.reference_asset_sha256[k*4+2])<<8)|std::uint32_t(w.reference_asset_sha256[k*4+3]);}
        ww::Encoder enc(b,w);if(!enc.valid())return 7;
        for(std::size_t k=0;k<ww::fragment_count;++k){ww::Fragment p{};
            if(!enc.fragment(k,p)||std::fwrite(p.payload,1,128,out)!=128||std::fwrite(&p.length,1,1,out)!=1)return 8;}
    }
    const bool closed=std::fclose(out)==0;std::fclose(f);
    std::printf("CXX_RWW1 windows=%u fragments=%zu hardware=0\n",nw,std::size_t(nw)*ww::fragment_count);
    return closed?0:9;
}
