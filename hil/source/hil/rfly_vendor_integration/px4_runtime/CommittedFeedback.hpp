#pragma once
#include "../RflySnapshotBoundExecutor.hpp"
namespace gpenmpc_rfly_px4 {
enum class FeedbackDisposition:std::uint8_t{Empty=0,Fresh=1,HistoricalExpired=2,Revoked=3};
struct CommittedFeedback {
    FeedbackDisposition disposition{FeedbackDisposition::Empty};
    gpenmpc_rfly_state_execution::SnapshotTicket snapshot_ticket{};
    gpenmpc_consumption::Token token{};
    gpenmpc_rfly_execution::Digest configuration_payload_sha256{},approved_parameter_sha256{},
        matlab_extraction_source_sha256{},generated_arm_source_sha256{},wrapper_matlab_source_sha256{};
    gpenmpc_portable::Array<double,61>actual61{};
    gpenmpc_portable::Array<float,16>published_control16{};
    std::uint64_t kernel_completed_us{},publication_us{},commit_completed_us{},original_valid_until_us{};
};
// Single Io owner and one-shot observing consumer, not an authority provider.
// Called only AFTER the production Io's final confirmation succeeds.
class CommittedFeedbackLatch final {
public:
    CommittedFeedbackLatch()=default;
    CommittedFeedbackLatch(const CommittedFeedbackLatch&)=delete;
    CommittedFeedbackLatch&operator=(const CommittedFeedbackLatch&)=delete;
    bool record_success(const gpenmpc_rfly_state_execution::NumericalPrepared&p,
        const gpenmpc_rfly_state_execution::NumericalReceipt&r,std::uint64_t original_commit_us,
        std::uint64_t original_valid_until_us)noexcept{
        if(revoked_||ready_){revoke();return false;} // never overwrite unconsumed evidence
        const auto&a=p.execution;const auto&c=r.execution;const auto&b=c.binding;
        if(!p.numerically_prepared||!a.numerically_prepared||!r.receipt_valid||!c.publication_receipt_valid||!b.valid||
           p.snapshot_ticket!=r.snapshot_ticket||!(a.token==b.consumed)||a.token.output_generation<=last_generation_||
           a.approved_configuration_sha256!=c.configuration_payload_sha256||a.generated_c_source_sha256!=c.generated_arm_source_sha256||
           a.wrapper_matlab_source_sha256!=c.wrapper_matlab_source_sha256||a.kernel_completed_us!=b.kernel_completed_us||
           std::memcmp(a.rfly_controls16.data(),c.published_control.data(),64)!=0||
           std::memcmp(a.actual61.data(),b.actual_output.wrench.data(),32)!=0||
           std::memcmp(a.actual61.data()+4,b.actual_output.rotor.data(),48)!=0||
           !b.kernel_completed_us||b.direct_motor_publication_us<b.kernel_completed_us||
           original_commit_us<b.direct_motor_publication_us||original_valid_until_us<original_commit_us){revoke();return false;}
        for(double v:a.actual61)if(!std::isfinite(v)){revoke();return false;}
        current_={};current_.snapshot_ticket=r.snapshot_ticket;current_.token=b.consumed;
        current_.configuration_payload_sha256=c.configuration_payload_sha256;current_.approved_parameter_sha256=a.approved_parameter_sha256;
        current_.matlab_extraction_source_sha256=a.matlab_extraction_source_sha256;current_.generated_arm_source_sha256=c.generated_arm_source_sha256;
        current_.wrapper_matlab_source_sha256=c.wrapper_matlab_source_sha256;current_.actual61=a.actual61;current_.published_control16=c.published_control;
        current_.kernel_completed_us=b.kernel_completed_us;current_.publication_us=b.direct_motor_publication_us;
        current_.commit_completed_us=original_commit_us;current_.original_valid_until_us=original_valid_until_us;
        last_generation_=a.token.output_generation;ready_=true;return true;
    }
    FeedbackDisposition take(CommittedFeedback&out,std::uint64_t actual_now)noexcept{
        out={};if(!ready_)return FeedbackDisposition::Empty;
        out=current_;ready_=false;
        // Bad/revoked state may still expose ONCE the raw actual committed data
        // for audit, but can never regain Fresh by replacing a clock or expiry.
        if(revoked_||actual_now<out.commit_completed_us)out.disposition=FeedbackDisposition::Revoked;
        else out.disposition=actual_now>out.original_valid_until_us?FeedbackDisposition::HistoricalExpired:FeedbackDisposition::Fresh;
        return out.disposition;
    }
    void revoke()noexcept{revoked_=true;} // retain already committed raw evidence
    bool pending()const noexcept{return ready_;}
    bool revoked()const noexcept{return revoked_;}
private:CommittedFeedback current_{};std::uint64_t last_generation_{};bool ready_{},revoked_{};
};
} // namespace gpenmpc_rfly_px4
