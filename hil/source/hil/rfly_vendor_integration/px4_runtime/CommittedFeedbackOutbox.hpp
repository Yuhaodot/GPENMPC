#pragma once
#include "../px4_wire/CommittedFeedbackWire.hpp"
namespace gpenmpc_rfly_px4 {
enum class FeedbackExport:std::uint8_t{Empty,Fragment,Interrupted};
struct FeedbackFragment {
    gpenmpc_argument_transport::Fragment fragment{};
    std::uint64_t generation{},original_commit_us{},original_valid_until_us{};
    std::uint8_t index{},target_system{},target_component{};
    FeedbackDisposition disposition{FeedbackDisposition::Empty};
};
// One pump producer, one stream consumer. The last copied fragment is retired
// explicitly AFTER the stream's void send call; this is local buffer ownership,
// never a wire/plant acknowledgement. Unconsumed raw evidence cannot be replaced.
class CommittedFeedbackOutbox final {
public:
    bool publish(const CommittedFeedback&f,std::uint8_t system,std::uint8_t component)noexcept{
        if(revoked()||interrupted()||pending()||!system||!component||!gpenmpc_feedback_wire::valid(f))return false;
        current_=f;system_=system;component_=component;next_=0;copied_=false;
        __atomic_store_n(&interrupted_,0,__ATOMIC_RELAXED);
        __atomic_store_n(&ready_,1,__ATOMIC_RELEASE);return true;
    }
    FeedbackExport copy_next(FeedbackFragment&out,std::uint64_t now)noexcept{
        out={};if(!pending())return FeedbackExport::Empty;
        if(interrupted())return FeedbackExport::Interrupted;
        if(!next_&&!copied_){
            if(revoked()||now<current_.commit_completed_us)current_.disposition=FeedbackDisposition::Revoked;
            else if(now>current_.original_valid_until_us&&current_.disposition==FeedbackDisposition::Fresh)
                current_.disposition=FeedbackDisposition::HistoricalExpired;
            if(!gpenmpc_feedback_wire::encode(current_,bytes_)){interrupt();return FeedbackExport::Interrupted;}
        }else if(current_.disposition==FeedbackDisposition::Fresh&&
            (revoked()||now<current_.commit_completed_us||now>current_.original_valid_until_us)){
            interrupt();return FeedbackExport::Interrupted;
        }
        if(!gpenmpc_feedback_wire::fragment(bytes_,next_,out.fragment)){interrupt();return FeedbackExport::Interrupted;}
        out.generation=current_.token.output_generation;out.index=next_;out.target_system=system_;out.target_component=component_;
        out.original_commit_us=current_.commit_completed_us;out.original_valid_until_us=current_.original_valid_until_us;
        out.disposition=current_.disposition;copied_=true;return FeedbackExport::Fragment;
    }
    bool retire_copy(std::uint64_t generation,std::uint8_t index)noexcept{
        if(!pending()||interrupted()||!copied_||generation!=current_.token.output_generation||index!=next_)return false;
        copied_=false;++next_;
        if(next_==gpenmpc_feedback_wire::fragment_count)__atomic_store_n(&ready_,0,__ATOMIC_RELEASE);
        return true;
    }
    bool allows_send(std::uint64_t generation,std::uint8_t index,std::uint64_t now)const noexcept{
        if(!pending()||interrupted()||!copied_||generation!=current_.token.output_generation||index!=next_)return false;
        return current_.disposition!=FeedbackDisposition::Fresh||
            (!revoked()&&now>=current_.commit_completed_us&&now<=current_.original_valid_until_us);
    }
    // Mid-batch expiry/revocation never rewrites the already emitted prefix or
    // retries a partial message. Raw remains for one explicit audit flush.
    void interrupt()noexcept{__atomic_store_n(&interrupted_,1,__ATOMIC_RELEASE);}
    FeedbackDisposition take_interrupted_raw(CommittedFeedback&out)noexcept{
        out={};if(!pending()||!interrupted())return FeedbackDisposition::Empty;
        out=current_;out.disposition=revoked()?FeedbackDisposition::Revoked:FeedbackDisposition::HistoricalExpired;
        __atomic_store_n(&ready_,0,__ATOMIC_RELEASE);return out.disposition;
    }
    void revoke()noexcept{__atomic_store_n(&revoked_,1,__ATOMIC_RELEASE);}
    bool pending()const noexcept{return __atomic_load_n(&ready_,__ATOMIC_ACQUIRE)!=0;}
    bool interrupted()const noexcept{return __atomic_load_n(&interrupted_,__ATOMIC_ACQUIRE)!=0;}
    bool revoked()const noexcept{return __atomic_load_n(&revoked_,__ATOMIC_ACQUIRE)!=0;}
private:
    CommittedFeedback current_{};gpenmpc_feedback_wire::Bytes bytes_{};
    std::uint8_t system_{},component_{},next_{},ready_{},revoked_{},interrupted_{};bool copied_{};
};
static_assert(__atomic_always_lock_free(1,nullptr),"actual target lock-free byte handoff required");
} // namespace gpenmpc_rfly_px4
