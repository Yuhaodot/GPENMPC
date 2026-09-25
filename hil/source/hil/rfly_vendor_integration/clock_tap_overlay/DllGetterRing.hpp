#pragma once
#include <windows.h>
#include <cwchar>
#include <climits>
#include "ClockObservationTap.hpp"

// RDR1 diagnostic memory ABI with a 256-record buffer.
// The ring serializes diagnostic writers independently of the generated model.
namespace gpenmpc_clock_ring {
using Sample=gpenmpc_clock_tap::DllGetterSample;
constexpr wchar_t environment[]=L"GPENMPC_CLOCK_RING_SECTION";
constexpr wchar_t prefix[]=L"Local\\GPENMPCClockRing_";
constexpr std::uint32_t magic=0x31524452,version=1,capacity=256;
constexpr std::size_t bytes=86176;
enum State:LONG {Prepared=1,Claimed=2,Active=3,Closed=4};
enum ConsumerState:LONG {ConsumerReady=1,ConsumerClaimed=2,ConsumerActive=3,ConsumerRetired=4};
enum Error:LONG {Overflow=1,WriterConflict=2,ConsumerGone=4,SessionDrift=8,
                ConsumerConflict=16,CursorInvalid=32,SecondClaim=64,ClosedDuringWrite=128};
struct alignas(8) Header {
    std::uint32_t magic_value{},abi_version{},section_bytes{},record_bytes{},record_capacity{},producer_pid{};
    volatile LONG state{},writer_busy{};
    volatile LONG64 published_write{},consumed_read{},events{},dropped{};
    std::uint64_t peer_nonce{};
    std::uint32_t consumer_pid{};volatile LONG consumer_state{},sticky_errors{},consumer_busy{};
    std::uint64_t first_observed_session{};
    volatile LONG64 overflow_events{},writer_conflicts{},consumer_retired_events{},session_drift_events{},
        consumer_retire_calls{},malformed_cursor_events{},first_fault_event{},second_claim_conflicts{};
};
struct alignas(8) Section {Header header;Sample records[capacity];};
static_assert(sizeof(Sample)==336&&sizeof(Header)==160&&sizeof(Section)==bytes&&offsetof(Section,records)==160,"RDR1 fixed ABI");
static_assert(offsetof(Header,state)==24&&offsetof(Header,writer_busy)==28&&offsetof(Header,published_write)==32&&
 offsetof(Header,consumed_read)==40&&offsetof(Header,events)==48&&offsetof(Header,dropped)==56&&offsetof(Header,peer_nonce)==64&&
 offsetof(Header,consumer_pid)==72&&offsetof(Header,consumer_state)==76&&offsetof(Header,sticky_errors)==80&&offsetof(Header,consumer_busy)==84&&
 offsetof(Header,first_observed_session)==88&&offsetof(Header,overflow_events)==96&&offsetof(Header,writer_conflicts)==104&&
 offsetof(Header,consumer_retired_events)==112&&offsetof(Header,session_drift_events)==120&&offsetof(Header,consumer_retire_calls)==128&&
 offsetof(Header,malformed_cursor_events)==136&&offsetof(Header,first_fault_event)==144&&offsetof(Header,second_claim_conflicts)==152,"RDR1 offsets");
inline LONG load(volatile LONG*p)noexcept{return InterlockedCompareExchange(p,0,0);}
inline LONG64 load64(volatile LONG64*p)noexcept{return InterlockedCompareExchange64(p,0,0);}
inline void fault(Header&h,Error e,LONG64 event)noexcept {
    InterlockedOr(&h.sticky_errors,e);
    // A fault before any getter uses ordinal 0; sticky_errors still proves it.
    if(event>0)InterlockedCompareExchange64(&h.first_fault_event,event,0);
}
inline bool shape(const Header&h)noexcept {
    return h.magic_value==magic&&h.abi_version==version&&h.section_bytes==bytes&&
        h.record_bytes==sizeof(Sample)&&h.record_capacity==capacity&&h.peer_nonce!=0;
}
inline bool initialize_for_peer(Section&s,std::uint64_t nonce)noexcept {
    if(!nonce)return false;std::memset(&s,0,sizeof s);auto&h=s.header;
    h.magic_value=magic;h.abi_version=version;h.section_bytes=bytes;h.record_bytes=sizeof(Sample);h.record_capacity=capacity;
    h.peer_nonce=nonce;InterlockedExchange(&h.consumer_state,ConsumerReady);InterlockedExchange(&h.state,Prepared);return true;
}
enum class Read {Record,Empty,Retired,Rejected};
// Exactly one successful claim; no unclaim/reset/recovery exists. Reads return
// copies, never a slot pointer that a later producer could overwrite.
class Consumer final {
public:
    Consumer()noexcept=default;
    Consumer(const Consumer&)=delete;Consumer&operator=(const Consumer&)=delete;
    Consumer(Consumer&&)=delete;Consumer&operator=(Consumer&&)=delete;
    bool claim(Section&s)noexcept {
        if(view_||!shape(s.header))return false;auto&h=s.header;
        if(InterlockedCompareExchange(&h.consumer_state,ConsumerClaimed,ConsumerReady)!=ConsumerReady){
            InterlockedIncrement64(&h.second_claim_conflicts);fault(h,SecondClaim,load64(&h.events));return false;}
        h.consumer_pid=GetCurrentProcessId();view_=&s;InterlockedExchange(&h.consumer_state,ConsumerActive);return true;
    }
    Read take(Sample&out)noexcept {
        if(!view_)return Read::Rejected;auto&h=view_->header;
        if(load(&h.consumer_state)!=ConsumerActive)return Read::Retired;
        if(InterlockedCompareExchange(&h.consumer_busy,1,0)!=0){fault(h,ConsumerConflict,load64(&h.events));return Read::Rejected;}
        Read result=Read::Empty;
        const LONG64 r=load64(&h.consumed_read),w=load64(&h.published_write);
        if(load(&h.consumer_state)!=ConsumerActive)result=Read::Retired;
        else if(r<0||w<r||w-r>capacity){InterlockedIncrement64(&h.malformed_cursor_events);fault(h,CursorInvalid,load64(&h.events));result=Read::Rejected;}
        else if(r<w){out=view_->records[static_cast<std::size_t>(r)%capacity];
            // Copy completes before the only consumer releases this slot.
            InterlockedExchange64(&h.consumed_read,r+1);result=Read::Record;}
        InterlockedExchange(&h.consumer_busy,0);return result;
    }
    void retire()noexcept {
        if(!view_)return;auto&h=view_->header;InterlockedIncrement64(&h.consumer_retire_calls);
        InterlockedExchange(&h.consumer_state,ConsumerRetired);fault(h,ConsumerGone,load64(&h.events));
    }
private:Section*view_{};
};
class Writer final {
public:
    Writer()noexcept=default;
    Writer(const Writer&)=delete;Writer&operator=(const Writer&)=delete;
    Writer(Writer&&)=delete;Writer&operator=(Writer&&)=delete;
    ~Writer(){if(view_)UnmapViewOfFile(view_);if(handle_)CloseHandle(handle_);}
    void attach_once_at_initialize()noexcept {
        if(attempted_)return;attempted_=true;
        wchar_t name[192]{};const DWORD n=GetEnvironmentVariableW(environment,name,192);
        constexpr std::size_t p=sizeof(prefix)/sizeof(wchar_t)-1;
        if(!n||n>=192||n<=p||std::wmemcmp(name,prefix,p)!=0)return;
        for(std::size_t j=p;j<n;++j){const wchar_t c=name[j];if(!((c>=L'0'&&c<=L'9')||(c>=L'A'&&c<=L'F')||(c>=L'a'&&c<=L'f')||c==L'_'))return;}
        HANDLE handle=OpenFileMappingW(FILE_MAP_ALL_ACCESS,FALSE,name);if(!handle)return;
        auto*s=static_cast<Section*>(MapViewOfFile(handle,FILE_MAP_ALL_ACCESS,0,0,bytes));
        if(!s){CloseHandle(handle);return;}auto&h=s->header;
        if(shape(h)&&load(&h.state)!=Prepared){InterlockedIncrement64(&h.second_claim_conflicts);fault(h,SecondClaim,load64(&h.events));UnmapViewOfFile(s);CloseHandle(handle);return;}
        const bool valid=shape(h)&&h.producer_pid==0&&load(&h.consumer_state)==ConsumerActive&&h.consumer_pid&&
            load64(&h.published_write)==0&&load64(&h.consumed_read)==0&&load64(&h.events)==0&&load64(&h.dropped)==0&&
            load(&h.writer_busy)==0&&load(&h.sticky_errors)==0;
        if(!valid){UnmapViewOfFile(s);CloseHandle(handle);return;}
        if(InterlockedCompareExchange(&h.state,Claimed,Prepared)!=Prepared){
            InterlockedIncrement64(&h.second_claim_conflicts);fault(h,SecondClaim,load64(&h.events));UnmapViewOfFile(s);CloseHandle(handle);return;}
        h.producer_pid=GetCurrentProcessId();view_=s;handle_=handle;InterlockedExchange(&h.state,Active);
    }
    bool enabled()const noexcept{return view_&&load(&view_->header.state)==Active;}
    void observe(Sample sample)noexcept {
        if(!view_)return;auto&h=view_->header;if(load(&h.state)!=Active)return;
        const LONG64 event=InterlockedIncrement64(&h.events);sample.event_ordinal=static_cast<std::uint64_t>(event);
        if(InterlockedCompareExchange(&h.writer_busy,1,0)!=0){InterlockedIncrement64(&h.writer_conflicts);InterlockedIncrement64(&h.dropped);fault(h,WriterConflict,event);return;}
        if(load(&h.state)!=Active){InterlockedIncrement64(&h.dropped);fault(h,ClosedDuringWrite,event);}
        else if(load(&h.consumer_state)!=ConsumerActive){InterlockedIncrement64(&h.consumer_retired_events);InterlockedIncrement64(&h.dropped);fault(h,ConsumerGone,event);}
        else {
            if(h.first_observed_session==0&&sample.plant_session!=0)h.first_observed_session=sample.plant_session;
            else if(h.first_observed_session!=0&&sample.plant_session!=h.first_observed_session){
                InterlockedIncrement64(&h.session_drift_events);fault(h,SessionDrift,event);}
            const LONG64 w=load64(&h.published_write),r=load64(&h.consumed_read);
            if(r<0||w<r||w-r>capacity||w==LLONG_MAX){InterlockedIncrement64(&h.malformed_cursor_events);InterlockedIncrement64(&h.dropped);fault(h,CursorInvalid,event);}
            else if(w-r==capacity){InterlockedIncrement64(&h.overflow_events);InterlockedIncrement64(&h.dropped);fault(h,Overflow,event);}
            else {view_->records[static_cast<std::size_t>(w)%capacity]=sample;InterlockedExchange64(&h.published_write,w+1);}
        }
        InterlockedExchange(&h.writer_busy,0);
    }
    void destroy_requested()noexcept {if(view_)InterlockedExchange(&view_->header.state,Closed);}
    // Mapping lifetime extends until DLL unload; join threads separately.
private:bool attempted_{};HANDLE handle_{};Section*view_{};
};
} // namespace gpenmpc_clock_ring
