// MATLAB consumer for the RDR1 and SSS1 observation ABIs.
#define WIN32_LEAN_AND_MEAN
#include <mex.h>
#include "../rfly_vendor_integration/clock_tap_overlay/DllStepSnapshotSharedStatus.hpp"
#include <string>
#include <memory>
#include <limits>
namespace ring=gpenmpc_clock_ring;namespace ss=gpenmpc_snapshot_status;
namespace {
struct Owner {
    HANDLE ring_handle{},status_handle{};ring::Section*r{};ss::Section*s{};
    ring::Consumer consumer;std::wstring ring_name,status_name;
    std::uint64_t nonce{},frequency{},last_event{},total{};bool failed{},claimed{};
    ring::Sample records[ring::capacity]{};std::uint64_t reads[ring::capacity]{};
    // Drain the single RDR consumer independently of MATLAB processing.
    // Preserve each record and its read timestamp.
    static constexpr unsigned pending_capacity=8192;
    ring::Sample pending[pending_capacity]{};std::uint64_t pending_reads[pending_capacity]{};
    unsigned head{},count{};SRWLOCK lock=SRWLOCK_INIT;HANDLE worker{};volatile LONG stopping{};
    struct Lock {SRWLOCK&v;explicit Lock(SRWLOCK&x):v(x){AcquireSRWLockExclusive(&v);}~Lock(){ReleaseSRWLockExclusive(&v);}};
    static DWORD WINAPI pump(void*p){auto&o=*static_cast<Owner*>(p);
        while(!InterlockedCompareExchange(&o.stopping,0,0)){o.collect();Sleep(1);}return 0;}
    std::uint64_t now(){LARGE_INTEGER q{};if(!QueryPerformanceCounter(&q)||q.QuadPart<=0||!frequency)return 0;
        const auto ticks=static_cast<std::uint64_t>(q.QuadPart),sec=ticks/frequency,rem=ticks%frequency;
        constexpr auto billion=std::uint64_t(1000000000);
        if(sec>UINT64_MAX/billion||rem>UINT64_MAX/billion)return 0;
        const auto whole=sec*billion,frac=rem*billion/frequency;
        return whole<=UINT64_MAX-frac?whole+frac:0;}
    bool open(){LARGE_INTEGER q{},f{};if(!QueryPerformanceCounter(&q)||!QueryPerformanceFrequency(&f)||q.QuadPart<=0||f.QuadPart<=0)return false;
        nonce=static_cast<std::uint64_t>(q.QuadPart);frequency=static_cast<std::uint64_t>(f.QuadPart);
        const auto suffix=std::to_wstring(GetCurrentProcessId())+L"_"+std::to_wstring(nonce);
        ring_name=std::wstring(ring::prefix)+suffix;status_name=std::wstring(ss::prefix)+suffix;
        ring_handle=CreateFileMappingW(INVALID_HANDLE_VALUE,nullptr,PAGE_READWRITE,0,ring::bytes,ring_name.c_str());
        if(!ring_handle||GetLastError()==ERROR_ALREADY_EXISTS)return false;
        r=static_cast<ring::Section*>(MapViewOfFile(ring_handle,FILE_MAP_ALL_ACCESS,0,0,ring::bytes));
        if(!r||!ring::initialize_for_peer(*r,nonce)||!consumer.claim(*r))return false;claimed=true;
        status_handle=CreateFileMappingW(INVALID_HANDLE_VALUE,nullptr,PAGE_READWRITE,0,ss::bytes,status_name.c_str());
        if(!status_handle||GetLastError()==ERROR_ALREADY_EXISTS)return false;
        s=static_cast<ss::Section*>(MapViewOfFile(status_handle,FILE_MAP_ALL_ACCESS,0,0,ss::bytes));
        if(!s||!ss::initialize_for_peer(*s,nonce))return false;
        worker=CreateThread(nullptr,0,&pump,this,0,nullptr);return worker!=nullptr;}
    // Pass the two environment names to the owned CopterSim child only.
    void collect(){Lock held(lock);if(failed)return;
        for(unsigned n=0;n<ring::capacity;++n){ring::Sample item{};const auto result=consumer.take(item);
            if(result==ring::Read::Empty)break;
            if(result!=ring::Read::Record){failed=true;break;}
            const auto read_ns=now();
            if(count==pending_capacity){failed=true;break;}
            const auto index=(head+count)%pending_capacity;pending[index]=item;pending_reads[index]=read_ns;++count;
            if(!read_ns||item.event_ordinal!=last_event+1){failed=true;break;}
            last_event=item.event_ordinal;
        }
        ss::View view{};if(!ss::read_atomic(*s,view)||ring::load(&r->header.sticky_errors)||
            view.sticky_errors||view.counters[1]||view.peer_nonce!=nonce||
            (view.producer_pid&&r->header.producer_pid&&view.producer_pid!=r->header.producer_pid))failed=true;
    }
    unsigned drain(){Lock held(lock);const unsigned n=count<ring::capacity?count:ring::capacity;
        for(unsigned k=0;k<n;++k){const auto index=(head+k)%pending_capacity;records[k]=pending[index];reads[k]=pending_reads[index];}
        head=(head+n)%pending_capacity;count-=n;total+=n;return n;}
    ~Owner(){InterlockedExchange(&stopping,1);if(worker){WaitForSingleObject(worker,INFINITE);CloseHandle(worker);}
        if(claimed)consumer.retire();if(s)UnmapViewOfFile(s);if(r)UnmapViewOfFile(r);
        if(status_handle)CloseHandle(status_handle);if(ring_handle)CloseHandle(ring_handle);}
};
std::unique_ptr<Owner> owner;
mxArray*number(std::uint64_t v){auto*a=mxCreateNumericMatrix(1,1,mxUINT64_CLASS,mxREAL);std::memcpy(mxGetData(a),&v,8);return a;}
mxArray*raw(const void*p,std::size_t m,std::size_t n=1){auto*a=mxCreateNumericMatrix(m,n,mxUINT8_CLASS,mxREAL);if(m*n)std::memcpy(mxGetData(a),p,m*n);return a;}
mxArray*wide(const std::wstring&v){std::string a(v.begin(),v.end());return mxCreateString(a.c_str());}
void set(mxArray*a,const char*k,mxArray*v){mxSetField(a,0,k,v);}
mxArray*result(unsigned n){
    const char*keys[]={"records","original_read_ns","ring_header","step_status","ring_name","status_name",
        "peer_nonce","consumer_pid","total_read","failed","clock_domain","snapshot_coherent","board_authority"};
    auto*a=mxCreateStructMatrix(1,1,13,keys);auto&o=*owner;Owner::Lock held(o.lock);
    set(a,"records",raw(o.records,sizeof(ring::Sample),n));
    auto*t=mxCreateNumericMatrix(n,1,mxUINT64_CLASS,mxREAL);if(n)std::memcpy(mxGetData(t),o.reads,n*8);set(a,"original_read_ns",t);
    // Read diagnostic header fields atomically; the combined result is not a coherent epoch snapshot.
    ring::Header h{};auto&v=o.r->header;
    h.magic_value=v.magic_value;h.abi_version=v.abi_version;h.section_bytes=v.section_bytes;
    h.record_bytes=v.record_bytes;h.record_capacity=v.record_capacity;h.producer_pid=v.producer_pid;
    h.state=ring::load(&v.state);h.writer_busy=ring::load(&v.writer_busy);
    h.published_write=ring::load64(&v.published_write);h.consumed_read=ring::load64(&v.consumed_read);
    h.events=ring::load64(&v.events);h.dropped=ring::load64(&v.dropped);h.peer_nonce=v.peer_nonce;
    h.consumer_pid=v.consumer_pid;h.consumer_state=ring::load(&v.consumer_state);h.sticky_errors=ring::load(&v.sticky_errors);
    h.consumer_busy=ring::load(&v.consumer_busy);h.first_observed_session=v.first_observed_session;
    h.overflow_events=ring::load64(&v.overflow_events);h.writer_conflicts=ring::load64(&v.writer_conflicts);
    h.consumer_retired_events=ring::load64(&v.consumer_retired_events);h.session_drift_events=ring::load64(&v.session_drift_events);
    h.consumer_retire_calls=ring::load64(&v.consumer_retire_calls);h.malformed_cursor_events=ring::load64(&v.malformed_cursor_events);
    h.first_fault_event=ring::load64(&v.first_fault_event);h.second_claim_conflicts=ring::load64(&v.second_claim_conflicts);
    set(a,"ring_header",raw(&h,sizeof h));ss::View view{};if(!ss::read_atomic(*o.s,view))o.failed=true;
    ss::Section s{};ss::initialize_for_peer(s,o.nonce);s.producer_pid=view.producer_pid;s.state=view.state;
    s.sticky_errors=view.sticky_errors;s.mirror_calls=view.mirror_calls;s.mirror_contentions=view.mirror_contentions;
    s.mirror_completions=view.mirror_completions;for(unsigned i=0;i<ss::counter_count;++i)s.counters[i]=view.counters[i];
    set(a,"step_status",raw(&s,sizeof s));set(a,"ring_name",wide(o.ring_name));set(a,"status_name",wide(o.status_name));
    set(a,"peer_nonce",number(o.nonce));set(a,"consumer_pid",number(GetCurrentProcessId()));set(a,"total_read",number(o.total));
    set(a,"failed",mxCreateLogicalScalar(o.failed));set(a,"clock_domain",mxCreateString("WINDOWS_QPC_INTEGER_FLOOR_NS_SAME_STOPWATCH"));
    set(a,"snapshot_coherent",mxCreateLogicalScalar(false));set(a,"board_authority",mxCreateLogicalScalar(false));return a;
}
void cleanup(){owner.reset();}
}
void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
    if(nrhs!=1||nlhs!=1||!mxIsChar(prhs[0]))mexErrMsgIdAndTxt("gpenmpc:RdrUsage","One command and one output required.");
    char command[16]{};if(mxGetString(prhs[0],command,sizeof command))mexErrMsgIdAndTxt("gpenmpc:RdrCommand","Invalid command.");
    if(!std::strcmp(command,"open")){
        if(owner)mexErrMsgIdAndTxt("gpenmpc:RdrOwned","Only one consumer, no reset/reopen of an active session.");
        std::unique_ptr<Owner> candidate(new Owner);if(!candidate->open()){candidate.reset();mexErrMsgIdAndTxt("gpenmpc:RdrCreate","Unique original observation sections unavailable.");}
        owner=std::move(candidate);mexLock();mexAtExit(cleanup);plhs[0]=result(0);return;
    }
    if(!owner)mexErrMsgIdAndTxt("gpenmpc:RdrAbsent","No original observation owner.");
    if(!std::strcmp(command,"drain")){plhs[0]=result(owner->drain());return;}
    if(!std::strcmp(command,"close")){
        plhs[0]=result(owner->drain());cleanup();mexUnlock();return;
    }
    mexErrMsgIdAndTxt("gpenmpc:RdrCommand","Only open/drain/close; no control or write API.");
}
