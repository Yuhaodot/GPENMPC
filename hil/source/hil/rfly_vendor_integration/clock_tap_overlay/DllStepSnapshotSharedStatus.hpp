#pragma once
#include "DllGetterRing.hpp"

// Independent SSS1 read-only diagnostic ABI. No control, clear or recovery API.
// The peer creates and retains this mapping, with the SAME RDR1 peer nonce.
// Values are individually atomic; a running multi-field read is NOT one epoch.
// Only externally established owner quiescence/exit permits final accounting.
namespace gpenmpc_snapshot_status {
constexpr wchar_t environment[]=L"GPENMPC_STEP_SNAPSHOT_SECTION";
constexpr wchar_t prefix[]=L"Local\\GPENMPCStepSnapshot_";
constexpr std::uint32_t magic=0x31535353,version=1,counter_count=10;
enum State:LONG {Prepared=1,Claimed=2,Active=3,Closed=4};
enum Error:LONG {MirrorContention=1,SecondClaim=2};
struct alignas(8) Section {
    std::uint32_t magic_value{},abi_version{},section_bytes{},counters_count{};
    std::uint64_t peer_nonce{};
    std::uint32_t producer_pid{};volatile LONG state{};
    volatile LONG64 mirror_calls{},mirror_contentions{},mirror_completions{};
    volatile LONG sticky_errors{};std::uint32_t reserved{};
    volatile LONG64 counters[counter_count]{};
};
constexpr std::size_t bytes=144;
static_assert(sizeof(Section)==bytes&&offsetof(Section,peer_nonce)==16&&offsetof(Section,producer_pid)==24&&
    offsetof(Section,state)==28&&offsetof(Section,mirror_calls)==32&&offsetof(Section,sticky_errors)==56&&offsetof(Section,counters)==64,"SSS1 offsets");
// counters: version, first sticky SnapshotStore fault, initialize entries,
// step entries, completed snapshots, getter entries, matches, before-first-step,
// rejected observations, local-tap contention. EXACT original export indices.
struct View {
    std::uint64_t peer_nonce{},counters[counter_count]{},mirror_calls{},mirror_contentions{},mirror_completions{};
    std::uint32_t producer_pid{};LONG state{},sticky_errors{};
};
inline bool shape(const Section&s)noexcept {
    return s.magic_value==magic&&s.abi_version==version&&s.section_bytes==bytes&&s.counters_count==counter_count&&s.peer_nonce&&s.reserved==0;
}
inline bool initialize_for_peer(Section&s,std::uint64_t same_ring_nonce)noexcept {
    if(!same_ring_nonce)return false;std::memset(&s,0,sizeof s);
    s.magic_value=magic;s.abi_version=version;s.section_bytes=bytes;s.counters_count=counter_count;s.peer_nonce=same_ring_nonce;
    InterlockedExchange(&s.state,Prepared);return true;
}
inline bool read_atomic(Section&s,View&out)noexcept {
    if(!shape(s))return false;View v{};v.peer_nonce=s.peer_nonce;v.producer_pid=s.producer_pid;
    v.state=gpenmpc_clock_ring::load(&s.state);v.sticky_errors=gpenmpc_clock_ring::load(&s.sticky_errors);
    v.mirror_calls=static_cast<std::uint64_t>(gpenmpc_clock_ring::load64(&s.mirror_calls));
    v.mirror_contentions=static_cast<std::uint64_t>(gpenmpc_clock_ring::load64(&s.mirror_contentions));
    v.mirror_completions=static_cast<std::uint64_t>(gpenmpc_clock_ring::load64(&s.mirror_completions));
    for(unsigned k=0;k<counter_count;++k)v.counters[k]=static_cast<std::uint64_t>(gpenmpc_clock_ring::load64(&s.counters[k]));
    out=v;return true;
}
inline bool valid_name(const wchar_t*name,DWORD n,const wchar_t*required,std::size_t p)noexcept {
    if(!n||n>=192||n<=p||std::wmemcmp(name,required,p)!=0)return false;
    for(std::size_t j=p;j<n;++j){const wchar_t c=name[j];if(!((c>=L'0'&&c<=L'9')||(c>=L'A'&&c<=L'F')||(c>=L'a'&&c<=L'f')||c==L'_'))return false;}
    return true;
}
class Writer final {
public:
    Writer()noexcept=default;Writer(const Writer&)=delete;Writer&operator=(const Writer&)=delete;
    ~Writer(){if(view_)UnmapViewOfFile(view_);if(handle_)CloseHandle(handle_);}
    bool attach_once_after_ring_initialize()noexcept {
        if(attempted_)return false;attempted_=true;
        wchar_t name[192]{},ring_name[192]{};
        const auto n=GetEnvironmentVariableW(environment,name,192);
        const auto r=GetEnvironmentVariableW(gpenmpc_clock_ring::environment,ring_name,192);
        if(!valid_name(name,n,prefix,sizeof(prefix)/sizeof(wchar_t)-1)||
           !valid_name(ring_name,r,gpenmpc_clock_ring::prefix,sizeof(gpenmpc_clock_ring::prefix)/sizeof(wchar_t)-1))return false;
        // Open existing mappings only. Actual active RDR1 owner and nonce bind
        // this status to the same observer, not an independently asserted PID.
        HANDLE rh=OpenFileMappingW(FILE_MAP_ALL_ACCESS,FALSE,ring_name);if(!rh)return false;
        auto*rs=static_cast<gpenmpc_clock_ring::Section*>(MapViewOfFile(rh,FILE_MAP_ALL_ACCESS,0,0,gpenmpc_clock_ring::bytes));
        std::uint64_t nonce=0;
        if(rs){
            if(gpenmpc_clock_ring::shape(rs->header)&&rs->header.producer_pid==GetCurrentProcessId()&&
               gpenmpc_clock_ring::load(&rs->header.state)==gpenmpc_clock_ring::Active){nonce=rs->header.peer_nonce;}
            UnmapViewOfFile(rs);
        }
        CloseHandle(rh);
        if(!nonce)return false;
        HANDLE h=OpenFileMappingW(FILE_MAP_ALL_ACCESS,FALSE,name);if(!h)return false;
        auto*s=static_cast<Section*>(MapViewOfFile(h,FILE_MAP_ALL_ACCESS,0,0,bytes));
        if(!s){CloseHandle(h);return false;}
        bool empty=shape(*s)&&s->peer_nonce==nonce&&s->producer_pid==0&&gpenmpc_clock_ring::load(&s->sticky_errors)==0&&
            gpenmpc_clock_ring::load64(&s->mirror_calls)==0&&gpenmpc_clock_ring::load64(&s->mirror_contentions)==0&&gpenmpc_clock_ring::load64(&s->mirror_completions)==0;
        for(unsigned k=0;k<counter_count;++k)empty=empty&&gpenmpc_clock_ring::load64(&s->counters[k])==0;
        if(!empty){UnmapViewOfFile(s);CloseHandle(h);return false;}
        if(InterlockedCompareExchange(&s->state,Claimed,Prepared)!=Prepared){InterlockedOr(&s->sticky_errors,SecondClaim);UnmapViewOfFile(s);CloseHandle(h);return false;}
        s->producer_pid=GetCurrentProcessId();view_=s;handle_=h;InterlockedExchange(&s->state,Active);return true;
    }
    void publish(const std::uint64_t original[counter_count])noexcept {
        if(!view_)return;auto&s=*view_;InterlockedIncrement64(&s.mirror_calls);bool complete=true;
        // All original values are monotonic (fault is permanently first-fault).
        // Max prevents a concurrent older sample from overwriting newer values.
        // Bounded 8 CAS attempts per field; no wait/lock. Failure is explicit.
        for(unsigned k=0;k<counter_count;++k){bool done=false;
            auto observed=gpenmpc_clock_ring::load64(&s.counters[k]);
            for(unsigned attempt=0;attempt<8;++attempt){
                if(static_cast<std::uint64_t>(observed)>=original[k]){done=true;break;}
                const auto previous=InterlockedCompareExchange64(&s.counters[k],static_cast<LONG64>(original[k]),observed);
                if(previous==observed){done=true;break;}observed=previous;
            }
            complete=complete&&done;
        }
        if(complete)InterlockedIncrement64(&s.mirror_completions);
        else {InterlockedIncrement64(&s.mirror_contentions);InterlockedOr(&s.sticky_errors,MirrorContention);}
    }
    void destroy_requested()noexcept {if(view_)InterlockedExchange(&view_->state,Closed);}
private:bool attempted_{};HANDLE handle_{};Section*view_{};
};
} // namespace gpenmpc_snapshot_status
