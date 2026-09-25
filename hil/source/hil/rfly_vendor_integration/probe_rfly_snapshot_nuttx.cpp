// Compile only with the actual PX4 FMU-v6c / NuttX gnu++14 database command.
#include "RflySnapshotBoundExecutor.hpp"

using RflyStore=gpenmpc_rfly_state_execution::RflySnapshotBoundExecutor<4>;
static std::uint64_t rfly_probe_clock(void*context)noexcept{return *static_cast<std::uint64_t*>(context);}
// Persistent storage avoids placing the complete >9KB snapshot core on a tick
// stack.
extern "C" RflyStore*gpenmpc_rfly_construct_actual_nuttx(
    const gpenmpc_odometry::Configuration&source,
    const gpenmpc_rfly_execution::Configuration&configuration,std::uint64_t&now)noexcept{
    static RflyStore store(source,configuration,{rfly_probe_clock,&now});return &store;
}
extern "C" bool gpenmpc_rfly_capture_actual_nuttx(RflyStore&store,
    const vehicle_odometry_s&raw,std::uint32_t original_generation,
    std::uint64_t original_receipt,gpenmpc_consumption::Identity identity,
    const void*topic,std::uint8_t instance,gpenmpc_rfly_state_execution::SnapshotTicket&ticket)noexcept{
    return store.capture(raw,original_generation,original_receipt,identity,topic,instance,ticket);
}
extern "C" bool gpenmpc_rfly_prepare_actual_nuttx(RflyStore&store,
    const gpenmpc_rfly_state_execution::NumericalCommand&command,
    gpenmpc_rfly_state_execution::NumericalPrepared&prepared)noexcept{
    return store.prepareNumericalOnly(command,prepared);
}
extern "C" bool gpenmpc_rfly_commit_actual_nuttx(RflyStore&store,
    const gpenmpc_rfly_state_execution::SnapshotTicket&ticket,
    const gpenmpc_rfly_execution::BackendRflyAck&ack,
    gpenmpc_rfly_state_execution::NumericalReceipt&receipt)noexcept{
    return store.commitNumericalReceipt(ticket,ack,receipt);
}
extern "C" {
char gpenmpc_rfly_target_size_SnapshotStore[sizeof(RflyStore)];
char gpenmpc_rfly_target_size_Executor[sizeof(gpenmpc_rfly_execution::CanonicalRflyExecutor)];
char gpenmpc_rfly_target_size_Prepared[sizeof(gpenmpc_rfly_execution::Prepared)];
char gpenmpc_rfly_target_size_Command[sizeof(gpenmpc_rfly_state_execution::NumericalCommand)];
char gpenmpc_rfly_target_size_Ack16[sizeof(gpenmpc_rfly_execution::BackendRflyAck)];
}
