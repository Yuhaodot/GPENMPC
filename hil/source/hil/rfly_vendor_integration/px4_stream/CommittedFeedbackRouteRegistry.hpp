#pragma once
#include "../px4_runtime/CommittedFeedbackOutbox.hpp"
#include "../px4_runtime/InheritingMutex.hpp"
#include "LinkLifetimeRegistry.hpp"
namespace gpenmpc_rfly_stream {
enum class FeedbackBind:std::uint8_t{Bound,Busy,Invalid,Unproven};
enum class FeedbackDetach:std::uint8_t{Detached,NotBound,Mismatch,Unproven};
struct FeedbackRegistration{std::uint64_t generation{};};
// Same exact single-link lifetime binding as private snapshot. Unbind must
// complete before pump/outbox storage is destroyed. No control authority.
class CommittedFeedbackRouteRegistry final {
public:
    bool ready()const noexcept{return mutex_.ready();}
    FeedbackBind bind(LinkToken link,gpenmpc_rfly_px4::CommittedFeedbackOutbox&outbox,FeedbackRegistration&receipt)noexcept{
        receipt={};if(!link.pointer||!link.generation)return FeedbackBind::Invalid;
        if(!mutex_.lock())return FeedbackBind::Unproven;
        const bool ok=!outbox_&&next_generation_!=UINT64_MAX;
        if(ok){outbox_=&outbox;link_=link;view_=nullptr;registration_.generation=++next_generation_;receipt=registration_;}
        if(!mutex_.unlock())return FeedbackBind::Unproven;
        return ok?FeedbackBind::Bound:FeedbackBind::Busy;
    }
    FeedbackDetach unbind(FeedbackRegistration receipt)noexcept{
        if(!mutex_.lock())return FeedbackDetach::Unproven;
        auto r=FeedbackDetach::Mismatch;
        if(!outbox_)r=FeedbackDetach::NotBound;
        else if(receipt.generation&&receipt.generation==registration_.generation){
            if(outbox_->pending())outbox_->interrupt();
            outbox_=nullptr;link_={};view_=nullptr;registration_={};r=FeedbackDetach::Detached;
        }
        return mutex_.unlock()?r:FeedbackDetach::Unproven;
    }
    gpenmpc_rfly_px4::FeedbackExport copy_next(LinkToken link,const void*view,gpenmpc_rfly_px4::FeedbackFragment&out,std::uint64_t now)noexcept{
        using R=gpenmpc_rfly_px4::FeedbackExport;out={};if(!view||!mutex_.lock())return R::Interrupted;
        auto r=R::Interrupted;
        if(outbox_&&link==link_&&(!view_||view_==view)){view_=view;r=outbox_->copy_next(out,now);}
        if(!mutex_.unlock()){out={};return R::Interrupted;}return r;
    }
    bool retire_copy(LinkToken link,const void*view,std::uint64_t generation,std::uint8_t index)noexcept{
        if(!mutex_.lock())return false;
        const bool ok=outbox_&&link==link_&&view_==view&&outbox_->retire_copy(generation,index);
        return mutex_.unlock()&&ok;
    }
    bool allows_send(LinkToken link,const void*view,std::uint64_t generation,std::uint8_t index,std::uint64_t now)noexcept{
        if(!mutex_.lock())return false;
        const bool ok=outbox_&&link==link_&&view_==view&&outbox_->allows_send(generation,index,now);
        return mutex_.unlock()&&ok;
    }
    bool interrupt(LinkToken link,const void*view)noexcept{
        if(!mutex_.lock())return false;
        const bool ok=outbox_&&link==link_&&view_==view;
        if(ok)outbox_->interrupt();
        return mutex_.unlock()&&ok;
    }
    bool release_view(LinkToken link,const void*view)noexcept{
        if(!mutex_.lock())return false;
        if(link==link_&&view==view_){if(outbox_&&outbox_->pending())outbox_->interrupt();view_=nullptr;}
        return mutex_.unlock();
    }
private:
    gpenmpc_rfly_px4::InheritingMutex mutex_{};gpenmpc_rfly_px4::CommittedFeedbackOutbox*outbox_{};
    LinkToken link_{};const void*view_{};FeedbackRegistration registration_{};std::uint64_t next_generation_{};
};
CommittedFeedbackRouteRegistry*committed_feedback_route_registry()noexcept;
} // namespace gpenmpc_rfly_stream
