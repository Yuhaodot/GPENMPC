#include "CommittedFeedbackWire.hpp"
#include "../px4_runtime/CommittedFeedbackOutbox.hpp"
#include "../px4_runtime/CanonicalExchangePump.hpp"
extern "C" bool gpenmpc_feedback_outbox_publish_target(gpenmpc_rfly_px4::CommittedFeedbackOutbox&box,const gpenmpc_rfly_px4::CommittedFeedback&f)noexcept{return box.publish(f,42,191);}
extern "C" gpenmpc_rfly_px4::FeedbackExport gpenmpc_feedback_outbox_copy_target(gpenmpc_rfly_px4::CommittedFeedbackOutbox&box,gpenmpc_rfly_px4::FeedbackFragment&f,std::uint64_t now)noexcept{return box.copy_next(f,now);}
extern "C" bool gpenmpc_feedback_outbox_retire_target(gpenmpc_rfly_px4::CommittedFeedbackOutbox&box,std::uint64_t gen,std::uint8_t index)noexcept{return box.retire_copy(gen,index);}
extern "C" bool gpenmpc_feedback_capture_target(gpenmpc_rfly_px4::CommittedFeedbackLatch&latch,
    const gpenmpc_rfly_state_execution::NumericalPrepared&prepared,const gpenmpc_rfly_state_execution::NumericalReceipt&receipt,
    std::uint64_t committed,std::uint64_t until)noexcept{return latch.record_success(prepared,receipt,committed,until);}
extern "C" gpenmpc_rfly_px4::FeedbackDisposition gpenmpc_feedback_take_target(gpenmpc_rfly_px4::CommittedFeedbackLatch&latch,
    gpenmpc_rfly_px4::CommittedFeedback&out,std::uint64_t now)noexcept{return latch.take(out,now);}
extern "C" bool gpenmpc_feedback_encode_target(const gpenmpc_rfly_px4::CommittedFeedback&feedback,
    gpenmpc_feedback_wire::Bytes&out)noexcept{return gpenmpc_feedback_wire::encode(feedback,out);}
extern "C" bool gpenmpc_feedback_decode_target(const gpenmpc_feedback_wire::Bytes&bytes,
    gpenmpc_rfly_px4::CommittedFeedback&out)noexcept{return gpenmpc_feedback_wire::decode(bytes,out);}
extern "C" bool gpenmpc_feedback_fragment_target(const gpenmpc_feedback_wire::Bytes&bytes,unsigned index,
    gpenmpc_argument_transport::Fragment&out)noexcept{return gpenmpc_feedback_wire::fragment(bytes,index,out);}
extern "C" {
char gpenmpc_feedback_size_Latch[sizeof(gpenmpc_rfly_px4::CommittedFeedbackLatch)];
char gpenmpc_feedback_size_Feedback[sizeof(gpenmpc_rfly_px4::CommittedFeedback)];
char gpenmpc_feedback_size_Wire[sizeof(gpenmpc_feedback_wire::Bytes)];
char gpenmpc_feedback_size_Outbox[sizeof(gpenmpc_rfly_px4::CommittedFeedbackOutbox)];
char gpenmpc_feedback_size_Io[sizeof(gpenmpc_rfly_px4::Px4CanonicalIo)];
char gpenmpc_feedback_size_Pump[sizeof(gpenmpc_rfly_px4::CanonicalExchangePump)];
}
