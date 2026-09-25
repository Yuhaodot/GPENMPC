#include "SharedOutputRegistry.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <sys/syscall.h>
#include <unistd.h>

using namespace gpenmpc_rfly_stream;
namespace {
unsigned checks{0},failures{0};
std::atomic<unsigned> observed_initializations{0},wrong_mutex_attributes{0};
std::atomic<bool> observe_initialization{false};
thread_local bool inject_init_failure{false};
thread_local bool inject_post_unlock_failure{false};
void check(bool ok,const char *label)
{
    ++checks;if(!ok){++failures;std::fprintf(stderr,"FAIL: %s\n",label);}
}

struct OwnerState {
    std::atomic<unsigned> chooses{0},accepts{0},calls_after_destroy{0};
    std::atomic<bool> alive{true};
    std::mutex mutex;
    std::condition_variable changed;
    bool entered{false},release{false};
    unsigned sequence{0},callback_finished{0},unbind_finished{0};
};
class MockOwner final:public Authority {
public:
    explicit MockOwner(OwnerState &state):state_(state){}
    ~MockOwner()override{state_.alive=false;}
    Source chosen{Source::RflyOutputs};
    bool accepted{true},block_choose{false},block_accept{false},abandon_in_choose{false};
    Source choose(std::uint64_t,const vehicle_status_s &,const vehicle_control_mode_s &)noexcept override
    {
        ++state_.chooses;if(!state_.alive)++state_.calls_after_destroy;
        if(abandon_in_choose){
            // Actual Linux thread death with no C++ forced-unwind crossing the
            // production noexcept callback. Kernel robust-list handling runs.
            (void)::syscall(SYS_exit,0);__builtin_unreachable();
        }
        if(block_choose)wait_inside_callback();
        return chosen;
    }
    bool accept(const Observation &,std::uint64_t,OriginalValidity &original)noexcept override
    {
        ++state_.accepts;if(!state_.alive)++state_.calls_after_destroy;
        if(block_accept)wait_inside_callback();
        original.valid_until_us=accepted?1777:0;return accepted;
    }
private:
    void wait_inside_callback()noexcept
    {
        std::unique_lock<std::mutex> lock(state_.mutex);
        state_.entered=true;state_.changed.notify_all();
        state_.changed.wait(lock,[this](){return state_.release;});
        state_.callback_finished=++state_.sequence;
    }
    OwnerState &state_;
};
const vehicle_status_s status{};
const vehicle_control_mode_s mode{};
Observation observation()
{
    Observation value{};value.source=Source::RflyOutputs;
    value.output.timestamp=1000;value.output_uorb_generation=7;return value;
}
void singleton_initialization()
{
    constexpr unsigned count=24;
    std::atomic<unsigned> waiting{0};std::atomic<bool> go{false};
    SharedOutputRegistry *seen[count]{};
    std::vector<std::thread> threads;threads.reserve(count);
    observe_initialization=true;
    for(unsigned i=0;i<count;++i)threads.emplace_back([&,i](){
        ++waiting;while(!go.load())std::this_thread::yield();seen[i]=shared_output_registry();
    });
    while(waiting.load()!=count)std::this_thread::yield();
    go=true;for(auto &thread:threads)thread.join();observe_initialization=false;
    auto *single=shared_output_registry();
    check(single && single->ready(),"real process singleton ready");
    check(observed_initializations==1,"24 concurrent calls construct exactly one mutex/singleton");
    check(wrong_mutex_attributes==0,"observed singleton mutex requests real PI and robust attributes");
    for(auto *entry:seen)check(!entry || entry==single,"initializing caller may get nullptr, never another singleton");
    int link=0;Router router(nullptr,&link);OriginalValidity expiry{123};
    check(router.choose(1000,status,mode)==Source::Unavailable,"default lazy singleton Router unbound unavailable");
    check(!router.accept(observation(),1000,expiry) && expiry.valid_until_us==0,"unbound accept clears expiry; no fallback");
}
void binding_and_consumption()
{
    SharedOutputRegistry registry;OwnerState state,other_state;MockOwner owner(state),other(other_state);
    int link=0,foreign=0;Registration registration{},duplicate{99},invalid{99};
    Router primary(&registry,Link{&link,1}),competitor(&registry,Link{&link,1}),wrong_link(&registry,Link{&foreign,1});
    Router reused_address(&registry,Link{&link,2});
    OriginalValidity expiry{321};
    check(registry.ready(),"independent real registry mutex ready");
    check(registry.bind(Link{},owner,invalid)==Bind::Invalid && invalid.generation==0,"null actual-link address rejected");
    check(primary.choose(1000,status,mode)==Source::Unavailable,"unbound registry never native fallback");
    check(!primary.accept(observation(),1000,expiry) && expiry.valid_until_us==0,"accept without selection rejected");
    check(registry.bind(Link{&link,1},owner,registration)==Bind::Registered && registration.generation!=0,"bind original exact owner");
    check(registry.bind(Link{&foreign,1},other,duplicate)==Bind::Busy && duplicate.generation==0,"duplicate binding cannot preempt owner");
    check(wrong_link.choose(1000,status,mode)==Source::Unavailable && state.chooses==0,"wrong actual-link address never enters owner");
    check(reused_address.choose(1000,status,mode)==Source::Unavailable,"same address new link generation cannot borrow old owner");
    check(primary.choose(1000,status,mode)==Source::RflyOutputs,"primary router selects canonical owner");
    check(competitor.choose(1000,status,mode)==Source::Unavailable,"second consumer cannot borrow same link");
    check(primary.accept(observation(),1000,expiry) && expiry.valid_until_us==1777,"owner original expiry passed without renewal");
    const auto accepts=state.accepts.load();expiry.valid_until_us=321;
    check(!primary.accept(observation(),1000,expiry) && expiry.valid_until_us==0 && state.accepts==accepts,
          "Router consumes its selection once, before callback");
    owner.accepted=false;
    check(primary.choose(1000,status,mode)==Source::RflyOutputs,"negative accept setup");
    check(!primary.accept(observation(),1000,expiry),"owner rejection retained");
    const auto rejected_accepts=state.accepts.load();
    check(!primary.accept(observation(),1000,expiry) && state.accepts==rejected_accepts,"failed accept selection also single-use");
    owner.chosen=Source::Unavailable;
    check(primary.choose(1000,status,mode)==Source::Unavailable,"owner unavailable not replaced by native");
    check(!primary.accept(observation(),1000,expiry) && state.accepts==rejected_accepts,"unavailable choose leaves no selection");
    Registration wrong{registration.generation+1};
    check(registry.unbind(wrong)==Detach::Mismatch,"wrong generation cannot detach owner");
    check(registry.unbind(registration)==Detach::Detached,"exact original registration detaches");
    check(registry.unbind(registration)==Detach::NotBound,"already unbound is not a second detach receipt");
    check(other_state.chooses==0 && other_state.accepts==0,"rejected competing owner never called");
}
void generation_and_view_lifetimes()
{
    SharedOutputRegistry registry;OwnerState old_state,new_state;MockOwner old_owner(old_state),new_owner(new_state);
    int link=0;Registration first{},second{};OriginalValidity expiry{123};
    Router router(&registry,Link{&link,1});
    check(registry.bind(Link{&link,1},old_owner,first)==Bind::Registered,"old binding setup");
    check(router.choose(1000,status,mode)==Source::RflyOutputs,"selection before rebind");
    check(registry.unbind(first)==Detach::Detached,"detach between choose and accept");
    check(registry.bind(Link{&link,1},new_owner,second)==Bind::Registered && second.generation>first.generation,"rebind advances private generation");
    check(!router.accept(observation(),1000,expiry) && expiry.valid_until_us==0 && new_state.accepts==0,
          "stale selection cannot call replacement owner");
    check(router.choose(1000,status,mode)==Source::RflyOutputs && router.accept(observation(),1000,expiry),"fresh selection reaches replacement owner");
    check(registry.unbind(first)==Detach::Mismatch,"old cleanup token cannot detach new owner");
    check(registry.unbind(second)==Detach::Detached,"replacement cleanup");
    check(registry.bind(Link{&link,1},new_owner,second)==Bind::Registered,"view lifetime binding");
    {Router temporary(&registry,Link{&link,1});check(temporary.choose(1000,status,mode)==Source::RflyOutputs,"temporary view claims consumer");}
    check(router.choose(1000,status,mode)==Source::RflyOutputs,"destroyed view releases consumer slot");
    check(registry.unbind(second)==Detach::Detached,"view lifetime cleanup");
}
void blocked_callback_detach(bool block_accept)
{
    SharedOutputRegistry registry;OwnerState state;int link=0;Registration receipt{};
    std::unique_ptr<MockOwner> owner(new MockOwner(state));
    owner->block_choose=!block_accept;owner->block_accept=block_accept;
    Router router(&registry,Link{&link,1});
    check(registry.bind(Link{&link,1},*owner,receipt)==Bind::Registered,"blocked callback binding");
    if(block_accept)check(router.choose(1000,status,mode)==Source::RflyOutputs,"blocked accept initial choose");
    std::thread callback([&](){if(block_accept){OriginalValidity expiry{};(void)router.accept(observation(),1000,expiry);}
        else (void)router.choose(1000,status,mode);});
    {std::unique_lock<std::mutex> lock(state.mutex);state.changed.wait(lock,[&](){return state.entered;});}
    std::promise<void> detacher_started;auto started=detacher_started.get_future();
    auto detach_result=std::async(std::launch::async,[&](){detacher_started.set_value();
        const auto value=registry.unbind(receipt);std::lock_guard<std::mutex> lock(state.mutex);
        state.unbind_finished=++state.sequence;return value;});
    started.wait();
    // Fixture scheduling observation.
    check(detach_result.wait_for(std::chrono::milliseconds(20))==std::future_status::timeout,
          "unbind cannot return while real owner callback is blocked");
    {std::lock_guard<std::mutex> lock(state.mutex);state.release=true;state.changed.notify_all();}
    callback.join();check(detach_result.get()==Detach::Detached,"detach returns after callback release");
    check(state.callback_finished && state.unbind_finished>state.callback_finished,"causal callback completion precedes detach receipt");
    const auto choose_count=state.chooses.load(),accept_count=state.accepts.load();
    owner.reset();
    OriginalValidity expiry{55};
    for(unsigned i=0;i<10;++i){
        check(router.choose(1000,status,mode)==Source::Unavailable,"destroyed owner never reentered after detach");
        check(!router.accept(observation(),1000,expiry) && expiry.valid_until_us==0,"post-detach accept never falls back");
    }
    check(state.chooses==choose_count && state.accepts==accept_count && state.calls_after_destroy==0,"owner callback counts unchanged after destruction");
}
void actual_owner_death()
{
    SharedOutputRegistry registry;OwnerState state;MockOwner owner(state);owner.abandon_in_choose=true;
    int link=0,view=0;Registration receipt{};
    check(registry.bind(Link{&link,1},owner,receipt)==Bind::Registered,"owner-death registry setup");
    struct Args {SharedOutputRegistry *registry;Link link;const void *view;} args{&registry,Link{&link,1},&view};
    pthread_t abandoned{};
    const int created=pthread_create(&abandoned,nullptr,[](void *opaque)->void *{
        auto *input=static_cast<Args *>(opaque);Registration selected{};
        (void)input->registry->choose(input->link,input->view,1000,status,mode,selected);return nullptr;
    },&args);
    check(created==0,"real pthread callback-abandon thread created");
    if(created==0){
        check(pthread_join(abandoned,nullptr)==0 && state.chooses==1,"real callback thread died while owning robust registry mutex");
        check(registry.unbind(receipt)==Detach::Unproven,"EOWNERDEAD cannot manufacture Detached");
        check(registry.unbind(receipt)==Detach::Unproven,"poisoned owner remains retained on repeated cleanup");
        Registration selection{};OriginalValidity expiry{77};
        check(registry.choose(Link{&link,1},&view,1000,status,mode,selection)==Source::Unavailable,"poisoned choose unavailable");
        check(!registry.accept(Link{&link,1},&view,receipt,observation(),1000,expiry) && expiry.valid_until_us==0,"poisoned accept clears expiry");
        Registration replacement{};
        check(registry.bind(Link{&link,1},owner,replacement)==Bind::Unproven,"poisoned registry cannot rebind");
        RegistryDiagnostics diagnostics{};
        check(!registry.diagnostics(diagnostics) && state.chooses==1 && state.accepts==0,"no owner callback after owner-death poison");
    }
    // Owner storage intentionally retained through registry usage, irrespective
    // of Unproven; fixture teardown is not an application cleanup authorization.
}
void explicit_api_fault_injections()
{
    int link=0;OwnerState state;MockOwner owner(state);
    inject_init_failure=true;
    SharedOutputRegistry unavailable;Registration receipt{};
    check(!unavailable.ready(),"INJECTED pthread initialization error remains not ready");
    check(unavailable.bind(Link{&link,1},owner,receipt)==Bind::Unproven && receipt.generation==0,"unknown initialization cannot bind");
    check(unavailable.unbind(receipt)==Detach::Unproven,"unknown mutex returns Unproven");
    SharedOutputRegistry registry;
    inject_post_unlock_failure=true;
    check(registry.bind(Link{&link,1},owner,receipt)==Bind::Unproven && receipt.generation!=0,
          "INJECTED post-unlock error preserves ambiguous binding cleanup receipt");
    RegistryDiagnostics diagnostics{};
    check(registry.diagnostics(diagnostics) && diagnostics.bound,"ambiguous bind may still contain owner; retain storage");
    check(registry.unbind(receipt)==Detach::Detached,"retained ambiguous receipt can clean actual installed binding");
}
} // namespace

extern "C" int __real_pthread_mutex_init(pthread_mutex_t *,const pthread_mutexattr_t *) noexcept;
extern "C" int __wrap_pthread_mutex_init(pthread_mutex_t *mutex,const pthread_mutexattr_t *attributes) noexcept
{
    if(inject_init_failure){inject_init_failure=false;return EAGAIN;}
    if(observe_initialization.load()){
        ++observed_initializations;int protocol=0,robust=0;
        if(!attributes || pthread_mutexattr_getprotocol(attributes,&protocol)!=0 || protocol!=PTHREAD_PRIO_INHERIT ||
           pthread_mutexattr_getrobust(attributes,&robust)!=0 || robust!=PTHREAD_MUTEX_ROBUST)++wrong_mutex_attributes;
    }
    return __real_pthread_mutex_init(mutex,attributes);
}
extern "C" int __real_pthread_mutex_unlock(pthread_mutex_t *) noexcept;
extern "C" int __wrap_pthread_mutex_unlock(pthread_mutex_t *mutex) noexcept
{
    const int result=__real_pthread_mutex_unlock(mutex);
    if(inject_post_unlock_failure && result==0){inject_post_unlock_failure=false;return EPERM;}
    return result;
}

int main()
{
    singleton_initialization();binding_and_consumption();generation_and_view_lifetimes();
    blocked_callback_detach(false);blocked_callback_detach(true);
    actual_owner_death();explicit_api_fault_injections();
    std::printf("{\"scope\":\"HOST_REAL_PTHREAD_PI_ROBUST_REGISTRY_MOCK_OWNER\",\"checks\":%u,\"failed\":%u,"
        "\"singleton_concurrent_callers\":24,\"singleton_mutex_initializations\":%u,\"actual_host_callback_owner_death\":true,"
        "\"explicit_injected_api_error_cases\":2,\"target_scheduler_executed\":false,\"live_stream_factory_executed\":false,"
        "\"identity_or_physical_authority_proven\":false}\n",checks,failures,observed_initializations.load());
    return failures?1:0;
}
