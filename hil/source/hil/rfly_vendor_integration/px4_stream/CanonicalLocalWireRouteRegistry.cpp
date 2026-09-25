#include "CanonicalLocalWireRouteRegistry.hpp"
#include <new>
namespace gpenmpc_rfly_stream {
LocalWireBind CanonicalLocalWireRouteRegistry::bind(LinkToken link,gpenmpc_rfly_px4::CanonicalLocalWireOutbox&outbox,LocalWireRegistration&r)noexcept{
    r={};if(!link.pointer||!link.generation||outbox.closed()||outbox.fault()!=gpenmpc_rfly_px4::LocalWireFault::None)return LocalWireBind::Invalid;
    if(!mutex_.lock())return LocalWireBind::Unproven;
    const bool ok=!outbox_&&next_generation_!=UINT64_MAX;
    if(ok){outbox_=&outbox;link_=link;view_=nullptr;registration_.generation=++next_generation_;r=registration_;}
    return mutex_.unlock()?(ok?LocalWireBind::Bound:LocalWireBind::Busy):LocalWireBind::Unproven;
}
LocalWireDetach CanonicalLocalWireRouteRegistry::unbind(LocalWireRegistration r)noexcept{
    if(!mutex_.lock())return LocalWireDetach::Unproven;
    LocalWireDetach result=LocalWireDetach::Mismatch;
    if(!outbox_)result=LocalWireDetach::NotBound;
    else if(r.generation&&r.generation==registration_.generation){
        outbox_->close();outbox_=nullptr;link_={};view_=nullptr;registration_={};result=LocalWireDetach::Detached;
    }
    return mutex_.unlock()?result:LocalWireDetach::Unproven;
}
gpenmpc_rfly_px4::LocalWireTake CanonicalLocalWireRouteRegistry::take(LinkToken link,const void*view,gpenmpc_rfly_px4::LocalWireFragment&out,std::uint64_t now)noexcept{
    using R=gpenmpc_rfly_px4::LocalWireTake;out={};if(!view||!mutex_.lock())return R::Unavailable;
    R result=R::Unavailable;
    if(outbox_&&link==link_&&(!view_||view_==view)){
        view_=view;result=outbox_->take(view,out,now);
        if(result==R::Fragment)++diagnostics_.copied_fragments;
        else if(result==R::Empty)++diagnostics_.empty;
        else if(result==R::Expired)++diagnostics_.expired;
    }else ++diagnostics_.rejected;
    if(!mutex_.unlock()){out={};return R::Unavailable;}return result;
}
bool CanonicalLocalWireRouteRegistry::release_view(LinkToken link,const void*view)noexcept{
    if(!mutex_.lock())return false;
    const bool match=outbox_&&view&&link==link_&&view==view_;
    if(match)outbox_->close(); // retain original pointer/view until explicit unbind
    return mutex_.unlock()&&match;
}
bool CanonicalLocalWireRouteRegistry::diagnostics(LocalWireRouteDiagnostics&out)noexcept{
    if(!mutex_.lock())return false;
    out=diagnostics_;return mutex_.unlock();
}
namespace {
alignas(CanonicalLocalWireRouteRegistry) unsigned char storage[sizeof(CanonicalLocalWireRouteRegistry)]{};
unsigned char state{};
static_assert(__atomic_always_lock_free(sizeof(state),nullptr),"actual target atomics, no libatomic");
}
CanonicalLocalWireRouteRegistry*canonical_local_wire_route_registry()noexcept{
    const auto value=__atomic_load_n(&state,__ATOMIC_ACQUIRE);
    if(value==2)return reinterpret_cast<CanonicalLocalWireRouteRegistry*>(storage);
    if(value!=0)return nullptr;
    unsigned char expected=0;if(!__atomic_compare_exchange_n(&state,&expected,1,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))return nullptr;
    auto*r=new(storage)CanonicalLocalWireRouteRegistry();const bool ok=r->ready();
    __atomic_store_n(&state,static_cast<unsigned char>(ok?2:3),__ATOMIC_RELEASE);return ok?r:nullptr;
}
}
