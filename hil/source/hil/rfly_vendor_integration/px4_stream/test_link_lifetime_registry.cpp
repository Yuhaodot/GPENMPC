#include "LinkLifetimeRegistry.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <future>
#include <mutex>
#include <thread>
#include <vector>
#include <sys/syscall.h>
#include <unistd.h>

using namespace gpenmpc_rfly_stream;
namespace {
unsigned checks=0,failures=0;
thread_local bool fail_init=false,fail_after_unlock=false;
void check(bool ok,const char *name){++checks;if(!ok){++failures;std::fprintf(stderr,"FAIL: %s\n",name);}}
struct HostLink {bool usb;};
struct ReadResult {bool usb{false};unsigned calls{0};};
bool read_usb(const void *pointer,void *result)noexcept
{
    auto &out=*static_cast<ReadResult *>(result);++out.calls;
    out.usb=static_cast<const HostLink *>(pointer)->usb;return true;
}
bool reject_read(const void *,void *)noexcept{return false;}
void basic_and_same_address()
{
    LinkLifetimeRegistry registry;HostLink link{true};ReadResult result{};LinkToken first{},found{};
    check(registry.ready(),"real pthread registry initialized");
    check(registry.lookup(&link,found)==LinkAccess::Unavailable,"lookup cannot self-register unknown pointer");
    check(registry.register_constructed(&link,first)==LinkRegistration::Registered && first.generation!=0,"actual construction registration");
    check(registry.lookup(&link,found)==LinkAccess::Unavailable && !found.pointer,"constructed but not activated is invisible");
    check(registry.read(first,read_usb,&result)==LinkAccess::Unavailable && result.calls==0,"no getter read concurrent with initialization");
    check(registry.activate(&link)==LinkAccess::Read,"final-property initialization activation");
    check(registry.lookup(&link,found)==LinkAccess::Read && found==first,"exact live token lookup");
    check(registry.read(first,read_usb,&result)==LinkAccess::Read && result.usb && result.calls==1,"bounded read of current live object");
    auto wrong=first;++wrong.generation;
    check(registry.read(wrong,read_usb,&result)==LinkAccess::Unavailable && result.calls==1,"wrong lifecycle generation rejected before dereference");
    check(registry.retire(wrong)==LinkRetirement::Mismatch,"wrong generation cannot retire real object");
    check(registry.read(first,nullptr,&result)==LinkAccess::Unavailable,"missing callback rejected");
    check(registry.read(first,read_usb,nullptr)==LinkAccess::Unavailable,"missing output storage rejected");
    check(registry.read(first,reject_read,&result)==LinkAccess::Rejected,"reader failure not labelled successful observation");
    check(registry.retire(first)==LinkRetirement::Quiesced,"original link borrow access retired");
    check(registry.retire_instance(&link)==LinkRetirement::NeverRegistered,"defensive destructor repeat proves no current borrower");
    check(registry.read(first,read_usb,&result)==LinkAccess::Unavailable && result.calls==1,"old pointer cannot be read after retirement");
    // Destroy/reconstruct at the actual same address without retaining a C++
    // reference across its lifetime; old generation must never become usable.
    link.~HostLink();new(&link) HostLink{false};LinkToken second{};
    check(registry.register_constructed(&link,second)==LinkRegistration::Registered && second.generation>first.generation,"same-address new construction gets new process-local generation");
    check(registry.activate(&link)==LinkAccess::Read,"replacement activation");
    check(registry.read(first,read_usb,&result)==LinkAccess::Unavailable && result.calls==1,"ABA old token cannot borrow replacement object");
    check(registry.read(second,read_usb,&result)==LinkAccess::Read && !result.usb && result.calls==2,"fresh token sees actual replacement property");
    LinkToken duplicate{};
    check(registry.register_constructed(&link,duplicate)==LinkRegistration::Conflict && !duplicate.pointer,"duplicate constructor fails closed");
    check(registry.lookup(&link,found)==LinkAccess::Unavailable,"duplicate lifecycle leaves object Closing, not revived");
    check(registry.read(second,read_usb,&result)==LinkAccess::Unavailable && result.calls==2,"old token disabled after conflicting construction");
    check(registry.retire(second)==LinkRetirement::Quiesced,"conflicting construction cleanup");
    check(registry.register_constructed(nullptr,duplicate)==LinkRegistration::Invalid,"null registration rejected");
}
void full_storage()
{
    LinkLifetimeRegistry registry;HostLink links[LinkLifetimeRegistry::capacity+1]{};
    LinkToken tokens[LinkLifetimeRegistry::capacity]{};
    for(unsigned i=0;i<LinkLifetimeRegistry::capacity;++i)
        check(registry.register_constructed(&links[i],tokens[i])==LinkRegistration::Registered,"bounded link slot registered");
    LinkToken overflow{};
    check(registry.register_constructed(&links[LinkLifetimeRegistry::capacity],overflow)==LinkRegistration::Full && !overflow.pointer,"full registry cannot reuse another live address");
    check(registry.retire(tokens[3])==LinkRetirement::Quiesced,"release one constructed slot without activation");
    check(registry.register_constructed(&links[LinkLifetimeRegistry::capacity],overflow)==LinkRegistration::Registered,"freed slot reusable with newer generation");
}
struct Block {
    std::mutex mutex;std::condition_variable changed;bool entered=false,release=false;
    unsigned order=0,read_finished=0,retire_finished=0;
};
bool blocked_read(const void *,void *opaque)noexcept
{
    auto &state=*static_cast<Block *>(opaque);
    // Intentionally blocked HOST fixture callback to challenge retirement;
    // the production USB getter does not take this lock or wait.
    std::unique_lock<std::mutex> lock(state.mutex);state.entered=true;state.changed.notify_all();
    state.changed.wait(lock,[&](){return state.release;});state.read_finished=++state.order;return true;
}
void pending_read()
{
    LinkLifetimeRegistry registry;HostLink link{true};LinkToken token{};Block state;
    check(registry.register_constructed(&link,token)==LinkRegistration::Registered && registry.activate(&link)==LinkAccess::Read,"blocked-read lifecycle setup");
    LinkAccess outcome=LinkAccess::Unavailable;
    std::thread reader([&](){outcome=registry.read(token,blocked_read,&state);});
    {std::unique_lock<std::mutex> lock(state.mutex);state.changed.wait(lock,[&](){return state.entered;});}
    std::promise<void> requested;auto request=requested.get_future();
    auto retired=std::async(std::launch::async,[&](){requested.set_value();auto result=registry.retire_instance(&link);
        std::lock_guard<std::mutex> lock(state.mutex);state.retire_finished=++state.order;return result;});
    request.wait();
    check(retired.wait_for(std::chrono::milliseconds(20))==std::future_status::timeout,"retire waits until actual entered borrow exits");
    {std::lock_guard<std::mutex> lock(state.mutex);state.release=true;state.changed.notify_all();}
    reader.join();check(outcome==LinkAccess::Read && retired.get()==LinkRetirement::Quiesced,"read finishes before checked destruction permission");
    check(state.read_finished && state.retire_finished>state.read_finished,"causal read completion precedes Quiesced");
    ReadResult result{};check(registry.read(token,read_usb,&result)==LinkAccess::Unavailable && !result.calls,"no dereference after checked retirement");
}
bool abandoned_read(const void *,void *opaque)noexcept
{
    ++*static_cast<std::atomic<unsigned> *>(opaque);
    (void)::syscall(SYS_exit,0);__builtin_unreachable();
}
void real_owner_death()
{
    LinkLifetimeRegistry registry;HostLink link{true};LinkToken token{};std::atomic<unsigned> calls{0};
    check(registry.register_constructed(&link,token)==LinkRegistration::Registered && registry.activate(&link)==LinkAccess::Read,"robust abandoned-read setup");
    struct Args {LinkLifetimeRegistry *registry;LinkToken token;std::atomic<unsigned> *calls;} args{&registry,token,&calls};
    pthread_t task{};int created=pthread_create(&task,nullptr,[](void *opaque)->void *{
        auto *input=static_cast<Args *>(opaque);(void)input->registry->read(input->token,abandoned_read,input->calls);return nullptr;
    },&args);
    check(created==0,"actual Linux borrowed-reader thread created");
    if(created==0){
        check(pthread_join(task,nullptr)==0 && calls==1,"actual robust owner died within borrowed read");
        check(registry.retire_instance(&link)==LinkRetirement::Unproven,"ownerdeath never manufactures deletion permission");
        check(registry.retire_instance(&link)==LinkRetirement::Unproven,"permanent poison retains deletion uncertainty");
        ReadResult result{};check(registry.read(token,read_usb,&result)==LinkAccess::Unproven && !result.calls,"poisoned registry cannot dereference link");
    }
}
void faults_and_singleton()
{
    HostLink link{true};LinkToken token{};fail_init=true;LinkLifetimeRegistry unknown;
    check(!unknown.ready() && unknown.retire_instance(&link)==LinkRetirement::Unproven,"injected initialization failure retains the instance");
    LinkLifetimeRegistry registry;fail_after_unlock=true;
    check(registry.register_constructed(&link,token)==LinkRegistration::Unproven && token.pointer==&link,"INJECTED unlock uncertainty retains constructed-token cleanup handle");
    check(registry.retire(token)==LinkRetirement::Quiesced,"ambiguous registration cleanup remains possible");
    constexpr unsigned n=16;LinkLifetimeRegistry *seen[n]{};std::atomic<bool> go{false};std::vector<std::thread> tasks;
    for(unsigned i=0;i<n;++i)tasks.emplace_back([&,i](){while(!go)std::this_thread::yield();seen[i]=link_lifetime_registry();});
    go=true;for(auto &task:tasks)task.join();auto *single=link_lifetime_registry();
    check(single && single->ready(),"process-lifetime singleton ready");
    for(auto *entry:seen)check(!entry || entry==single,"concurrent singleton returns same instance or initializing nullptr");
}
}
extern "C" int __real_pthread_mutex_init(pthread_mutex_t *,const pthread_mutexattr_t *)noexcept;
extern "C" int __wrap_pthread_mutex_init(pthread_mutex_t *mutex,const pthread_mutexattr_t *attr)noexcept
{if(fail_init){fail_init=false;return EAGAIN;}return __real_pthread_mutex_init(mutex,attr);}
extern "C" int __real_pthread_mutex_unlock(pthread_mutex_t *)noexcept;
extern "C" int __wrap_pthread_mutex_unlock(pthread_mutex_t *mutex)noexcept
{const int result=__real_pthread_mutex_unlock(mutex);if(fail_after_unlock && result==0){fail_after_unlock=false;return EPERM;}return result;}
int main()
{
    basic_and_same_address();full_storage();pending_read();real_owner_death();faults_and_singleton();
    std::printf("{\"scope\":\"HOST_REAL_PTHREAD_LINK_BORROW_MOCK_LINK_DATA\",\"checks\":%u,\"failed\":%u,\"actual_host_owner_death\":true,\"explicit_api_fault_injections\":2,\"nuttx_execution\":false,\"mavlink_executed\":false,\"boot_identity_proven\":false,\"physical_evidence_proven\":false}\n",checks,failures);
    return failures?1:0;
}
