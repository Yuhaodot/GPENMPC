// Actual ARM compilation only; each stage uses the production API/type. This
// probe has no scheduler, simulated uORB publisher, device, or execution path.
#include "SlimArgumentTransport.hpp"
#include <uORB/topics/uORBTopics.hpp>
#include <uORB/topics/gpenmpc_full_inner_ingress.h>
static_assert(ORB_TOPICS_COUNT==311,"same generated alias+ingress table across target headers");
extern "C" std::uint8_t gpenmpc_slim_parse_actual_nuttx(mavlink_message_t&buffer,mavlink_status_t&status,
    std::uint8_t byte,mavlink_message_t&parsed,mavlink_status_t&report)noexcept{
    return mavlink_frame_char_buffer(&buffer,&status,byte,&parsed,&report);
}
extern "C" gpenmpc_ingress::Result gpenmpc_slim_ingress_actual_nuttx(gpenmpc_ingress::Receiver&receiver,
    const mavlink_message_t&frame,std::uint64_t original_hrt,std::uint8_t local_system,
    std::uint8_t local_component,std::uint8_t instance)noexcept{
    return receiver.receive(frame,original_hrt,local_system,local_component,instance);
}
extern "C" bool gpenmpc_slim_expect_actual_nuttx(gpenmpc_slim_transport::SlimAssembler&assembler,
    const gpenmpc_argument_transport::Metadata&expected,std::uint64_t now)noexcept{return assembler.expect(expected,now);}
extern "C" bool gpenmpc_slim_assemble_actual_nuttx(gpenmpc_slim_transport::SlimAssembler&assembler,
    const gpenmpc_full_inner_ingress_s&topic,std::uint64_t original_uorb_generation,std::uint64_t now)noexcept{
    return assembler.receive(gpenmpc_argument_transport::from_topic(topic,original_uorb_generation),now);
}
extern "C" bool gpenmpc_slim_take_actual_nuttx(gpenmpc_slim_transport::SlimAssembler&assembler,
    gpenmpc_slim_transport::SlimCompleted&done,std::uint64_t now)noexcept{return assembler.take(done,now);}
extern "C" bool gpenmpc_slim_bound_prepare_actual_nuttx(gpenmpc_rfly_slim::SnapshotExecutor<4>&executor,
    const gpenmpc_slim_transport::SlimCompleted&done,const gpenmpc_consumption::Reference&reference,
    const gpenmpc_consumption::OuterCommand&outer,const gpenmpc_rfly_execution::Configuration&approved,
    gpenmpc_rfly_slim::Command&private_scratch,gpenmpc_rfly_state_execution::NumericalPrepared&prepared)noexcept{
    return gpenmpc_slim_transport::bind_command(done,reference,outer,approved,private_scratch)&&
        executor.prepareNumericalOnly(private_scratch,prepared);
}
extern "C" bool gpenmpc_slim_bound_commit_actual_nuttx(gpenmpc_rfly_slim::SnapshotExecutor<4>&executor,
    const gpenmpc_rfly_state_execution::SnapshotTicket&ticket,const gpenmpc_rfly_execution::BackendRflyAck&ack,
    gpenmpc_rfly_state_execution::NumericalReceipt&receipt)noexcept{return executor.commitNumericalReceipt(ticket,ack,receipt);}
extern "C" {
char gpenmpc_transport_target_size_Assembler[sizeof(gpenmpc_slim_transport::SlimAssembler)];
char gpenmpc_transport_target_size_Completed[sizeof(gpenmpc_slim_transport::SlimCompleted)];
char gpenmpc_transport_target_size_Receiver[sizeof(gpenmpc_ingress::Receiver)];
char gpenmpc_transport_target_size_Message[sizeof(gpenmpc_slim_transport::SlimMessage)];
}
