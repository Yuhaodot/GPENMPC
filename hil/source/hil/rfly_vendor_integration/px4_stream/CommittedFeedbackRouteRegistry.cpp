#include "CommittedFeedbackRouteRegistry.hpp"
#include <new>
namespace gpenmpc_rfly_stream {
namespace {alignas(CommittedFeedbackRouteRegistry) unsigned char storage[sizeof(CommittedFeedbackRouteRegistry)]{};unsigned char state{};}
CommittedFeedbackRouteRegistry*committed_feedback_route_registry()noexcept{
    const auto s=__atomic_load_n(&state,__ATOMIC_ACQUIRE);
    if(s==2)return reinterpret_cast<CommittedFeedbackRouteRegistry*>(storage);
    if(s!=0)return nullptr;
    unsigned char expected=0;
    if(!__atomic_compare_exchange_n(&state,&expected,1,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))return nullptr;
    auto*r=new(storage) CommittedFeedbackRouteRegistry();const bool ready=r->ready();
    __atomic_store_n(&state,static_cast<unsigned char>(ready?2:3),__ATOMIC_RELEASE);return ready?r:nullptr;
}
} // namespace gpenmpc_rfly_stream
