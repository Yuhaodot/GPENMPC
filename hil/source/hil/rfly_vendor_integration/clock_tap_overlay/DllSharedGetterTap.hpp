#pragma once
#include <windows.h>
#include <cwchar>
#include "ClockObservationTap.hpp"

// RDT1 Windows-x64 diagnostic memory ABI. The peer creates and initializes
// the named section before launching the child; the DLL opens that section.
namespace gpenmpc_clock_tap {
constexpr wchar_t section_environment[]=L"GPENMPC_CLOCK_TAP_SECTION";
constexpr wchar_t section_prefix[]=L"Local\\GPENMPCClockTap_";
constexpr std::uint32_t section_magic=0x31544452,section_version=1,section_capacity=32;
enum SectionState:LONG {Prepared=1,Claimed=2,Active=3,Closed=4};
enum SectionError:LONG {WriterBusy=1,CapacityExceeded=2,ClosedDuringGetter=4};
struct alignas(8) SharedHeader {
    std::uint32_t magic{},abi_version{},section_bytes{},record_bytes{},capacity{},producer_pid{};
    volatile LONG state{},published_count{};
    volatile LONG64 events{},dropped{};
    std::uint64_t peer_nonce{};
    volatile LONG sticky_errors{},writer_busy{};
};
struct alignas(8) SharedSection {SharedHeader header;DllGetterSample records[section_capacity];};
constexpr std::size_t section_bytes=10816;
static_assert(sizeof(DllGetterSample)==336,"Fixed getter record ABI");
static_assert(offsetof(DllGetterSample,hil_output30)==24&&offsetof(DllGetterSample,accepted_time_s)==264&&
              offsetof(DllGetterSample,rotor_lag6_n)==272&&offsetof(DllGetterSample,copied_length)==320&&
              offsetof(DllGetterSample,getter_thread_id)==324&&offsetof(DllGetterSample,observation_valid)==328&&
              offsetof(DllGetterSample,pair_scope)==331,"Fixed getter record offsets");
static_assert(sizeof(SharedHeader)==64&&offsetof(SharedHeader,state)==24&&offsetof(SharedHeader,published_count)==28&&
              offsetof(SharedHeader,events)==32&&offsetof(SharedHeader,dropped)==40&&offsetof(SharedHeader,peer_nonce)==48&&
              offsetof(SharedHeader,sticky_errors)==56&&offsetof(SharedHeader,writer_busy)==60,"Fixed shared header offsets");
static_assert(sizeof(SharedSection)==section_bytes&&offsetof(SharedSection,records)==64,"RDT1 10816-byte ABI");
inline LONG load_long(volatile LONG*p)noexcept{return InterlockedCompareExchange(p,0,0);}
inline LONG64 load_long64(volatile LONG64*p)noexcept{return InterlockedCompareExchange64(p,0,0);}
// Peer only, BEFORE child start and before any producer claim. Never call this
// to reset a live/retained section. The caller creates a unique new object.
inline bool initialize_shared_for_peer(SharedSection&s,std::uint64_t nonce)noexcept {
    if(!nonce)return false;
    std::memset(&s,0,sizeof s);s.header.magic=section_magic;s.header.abi_version=section_version;
    s.header.section_bytes=static_cast<std::uint32_t>(section_bytes);
    s.header.record_bytes=sizeof(DllGetterSample);s.header.capacity=section_capacity;
    s.header.peer_nonce=nonce;InterlockedExchange(&s.header.state,Prepared);return true;
}

// Producer helper only; all metadata/pointer setup is model-initialize-only.
// Concurrent getters only touch Interlocked counters or the CAS-owned record.
// This protects diagnostics, NEVER mmc.step or the model output snapshot.
class SharedGetterWriter final {
public:
    ~SharedGetterWriter(){if(view_)UnmapViewOfFile(view_);if(handle_)CloseHandle(handle_);}
    void attach_once_at_model_initialize()noexcept {
        if(attempted_)return;attempted_=true;
        wchar_t name[192]{};const DWORD n=GetEnvironmentVariableW(section_environment,name,192);
        if(!n||n>=192)return;
        const std::size_t prefix_size=sizeof(section_prefix)/sizeof(wchar_t)-1;
        if(n<=prefix_size||std::wmemcmp(name,section_prefix,prefix_size)!=0)return;
        for(std::size_t j=prefix_size;j<n;++j){const wchar_t c=name[j];
            if(!((c>=L'0'&&c<=L'9')||(c>=L'a'&&c<=L'f')||(c>=L'A'&&c<=L'F')||c==L'_'))return;}
        HANDLE h=OpenFileMappingW(FILE_MAP_ALL_ACCESS,FALSE,name);if(!h)return;
        auto*v=static_cast<SharedSection*>(MapViewOfFile(h,FILE_MAP_ALL_ACCESS,0,0,section_bytes));
        if(!v){CloseHandle(h);return;}auto&b=v->header;
        const bool valid=b.magic==section_magic&&b.abi_version==section_version&&b.section_bytes==section_bytes&&
            b.record_bytes==sizeof(DllGetterSample)&&b.capacity==section_capacity&&b.producer_pid==0&&b.peer_nonce!=0&&
            load_long(&b.published_count)==0&&load_long64(&b.events)==0&&load_long64(&b.dropped)==0&&
            load_long(&b.sticky_errors)==0&&load_long(&b.writer_busy)==0;
        if(!valid||InterlockedCompareExchange(&b.state,Claimed,Prepared)!=Prepared){UnmapViewOfFile(v);CloseHandle(h);return;}
        b.producer_pid=GetCurrentProcessId();handle_=h;view_=v;InterlockedExchange(&b.state,Active);
    }
    bool enabled()const noexcept{return view_&&load_long(&view_->header.state)==Active;}
    void observe(DllGetterSample sample)noexcept {
        if(!view_)return;auto&b=view_->header;
        if(load_long(&b.state)!=Active)return;
        sample.event_ordinal=static_cast<std::uint64_t>(InterlockedIncrement64(&b.events));
        if(InterlockedCompareExchange(&b.writer_busy,1,0)!=0){
            InterlockedIncrement64(&b.dropped);InterlockedOr(&b.sticky_errors,WriterBusy);return;}
        if(load_long(&b.state)!=Active){InterlockedIncrement64(&b.dropped);InterlockedOr(&b.sticky_errors,ClosedDuringGetter);}
        else {
            const LONG count=load_long(&b.published_count);
            if(count<0||count>=static_cast<LONG>(section_capacity)){
                InterlockedIncrement64(&b.dropped);InterlockedOr(&b.sticky_errors,CapacityExceeded);
            }else {
                view_->records[count]=sample;
                // The release/full barrier publishes an immutable prefix for readers.
                InterlockedExchange(&b.published_count,count+1);
            }
        }
        InterlockedExchange(&b.writer_busy,0);
    }
    void model_destroy_requested()noexcept {
        if(view_)InterlockedExchange(&view_->header.state,Closed);
        // Do not unmap underneath a getter. Keep this bounded mapping until
        // DLL unload (no outstanding calls per loader contract). Closed is a
        // Destruction notification; thread joins are handled separately.
    }
private:
    bool attempted_{};HANDLE handle_{};SharedSection*view_{};
};
} // namespace gpenmpc_clock_tap
