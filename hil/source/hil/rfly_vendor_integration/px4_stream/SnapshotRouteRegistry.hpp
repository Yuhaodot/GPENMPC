#pragma once
#include "../px4_runtime/SnapshotOutbox.hpp"
#include "../px4_runtime/InheritingMutex.hpp"
#include "LinkLifetimeRegistry.hpp"
namespace gpenmpc_rfly_stream {
enum class SnapshotBind:std::uint8_t{Bound,Busy,Invalid,Unproven};
enum class SnapshotDetach:std::uint8_t{Detached,NotBound,Mismatch,Unproven};
struct SnapshotRegistration{std::uint64_t generation{};};
struct SnapshotRouteDiagnostics{std::uint64_t copied_fragments{},empty{},expired{},rejected{};};
// Observations only. Exact link + one stream consumer, no control authority.
// Pump/outbox storage lives until unbind returns Detached/NotBound. Callbacks
// copy under PI mutex; no Mavlink write occurs while this mutex is held.
class SnapshotRouteRegistry final {
public:
    bool ready()const noexcept{return mutex_.ready();}
    SnapshotBind bind(LinkToken link,gpenmpc_rfly_px4::SnapshotOutbox &outbox,
                      SnapshotRegistration &receipt)noexcept{
        receipt={};if(!link.pointer||!link.generation)return SnapshotBind::Invalid;
        if(!mutex_.lock())return SnapshotBind::Unproven;
        const bool ok=!outbox_&&next_generation_!=UINT64_MAX;
        if(ok){outbox_=&outbox;link_=link;view_=nullptr;
            registration_.generation=++next_generation_;receipt=registration_;}
        if(!mutex_.unlock())return SnapshotBind::Unproven;
        return ok?SnapshotBind::Bound:SnapshotBind::Busy;
    }
    SnapshotDetach unbind(SnapshotRegistration receipt)noexcept{
        if(!mutex_.lock())return SnapshotDetach::Unproven;
        SnapshotDetach r=SnapshotDetach::Mismatch;
        if(!outbox_)r=SnapshotDetach::NotBound;
        else if(receipt.generation&&receipt.generation==registration_.generation){
            outbox_=nullptr;link_={};view_=nullptr;registration_={};r=SnapshotDetach::Detached;}
        return mutex_.unlock()?r:SnapshotDetach::Unproven;
    }
    gpenmpc_rfly_px4::ExportTake take(LinkToken link,const void *view,
        gpenmpc_rfly_px4::SnapshotFragment &out,std::uint64_t now)noexcept{
        using R=gpenmpc_rfly_px4::ExportTake;out={};
        if(!view||!mutex_.lock())return R::Stopped;
        R r=R::Stopped;
        if(outbox_&&link==link_&&(!view_||view_==view)){
            view_=view;r=outbox_->take(out,now);
            if(r==R::Fragment)++diagnostics_.copied_fragments;
            else if(r==R::Empty)++diagnostics_.empty;
            else if(r==R::Expired)++diagnostics_.expired;
        }else ++diagnostics_.rejected;
        if(!mutex_.unlock()){out={};return R::Stopped;}return r;
    }
    bool release_view(LinkToken link,const void *view)noexcept{
        if(!mutex_.lock())return false;
        if(link==link_&&view==view_)view_=nullptr;
        return mutex_.unlock();
    }
    bool diagnostics(SnapshotRouteDiagnostics &out)noexcept{
        if(!mutex_.lock())return false;
        out=diagnostics_;return mutex_.unlock();
    }
private:
    gpenmpc_rfly_px4::InheritingMutex mutex_{};
    gpenmpc_rfly_px4::SnapshotOutbox *outbox_{};
    LinkToken link_{};const void *view_{};
    SnapshotRegistration registration_{};std::uint64_t next_generation_{};
    SnapshotRouteDiagnostics diagnostics_{};
};
SnapshotRouteRegistry *snapshot_route_registry()noexcept;
} // namespace gpenmpc_rfly_stream
