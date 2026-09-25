// Compile-only: actual FMUv6C/NuttX gnu++14 flags, never executed on a board.
#include "SlimSnapshotExecutor.hpp"
using SlimStore=gpenmpc_rfly_slim::SnapshotExecutor<4>;
static std::uint64_t slim_probe_clock(void*context)noexcept{return *static_cast<std::uint64_t*>(context);}
extern "C" SlimStore*gpenmpc_slim_construct_actual_nuttx(
    const gpenmpc_odometry::Configuration&source,
    const gpenmpc_rfly_execution::Configuration&configuration,std::uint64_t&now)noexcept{
    static SlimStore store(source,configuration,{slim_probe_clock,&now});return &store;
}
extern "C" bool gpenmpc_slim_capture_actual_nuttx(SlimStore&store,const vehicle_odometry_s&raw,
    std::uint32_t original_generation,std::uint64_t original_receipt,gpenmpc_consumption::Identity identity,
    const void*topic,std::uint8_t instance,gpenmpc_rfly_state_execution::SnapshotTicket&ticket)noexcept{
    return store.capture(raw,original_generation,original_receipt,identity,topic,instance,ticket);
}
extern "C" const gpenmpc_odometry::Snapshot*gpenmpc_slim_snapshot_actual_nuttx(const SlimStore&store,
    const gpenmpc_rfly_state_execution::SnapshotTicket&ticket)noexcept{return store.snapshot(ticket);}
extern "C" bool gpenmpc_slim_prepare_actual_nuttx(SlimStore&store,const gpenmpc_rfly_slim::Command&command,
    gpenmpc_rfly_state_execution::NumericalPrepared&prepared)noexcept{return store.prepareNumericalOnly(command,prepared);}
extern "C" bool gpenmpc_slim_commit_actual_nuttx(SlimStore&store,const gpenmpc_rfly_state_execution::SnapshotTicket&ticket,
    const gpenmpc_rfly_execution::BackendRflyAck&ack,gpenmpc_rfly_state_execution::NumericalReceipt&receipt)noexcept{
    return store.commitNumericalReceipt(ticket,ack,receipt);
}
extern "C" {
char gpenmpc_slim_target_size_Store[sizeof(SlimStore)];
char gpenmpc_slim_target_size_Command[sizeof(gpenmpc_rfly_slim::Command)];
char gpenmpc_slim_target_size_KernelScratch[sizeof(gpenmpc_consumption::KernelArguments)];
char gpenmpc_slim_target_size_ExpandedScratch[sizeof(gpenmpc_rfly_state_execution::NumericalCommand)];
char gpenmpc_slim_target_size_NumericalPrepared[sizeof(gpenmpc_rfly_state_execution::NumericalPrepared)];
char gpenmpc_slim_target_size_Ack16[sizeof(gpenmpc_rfly_execution::BackendRflyAck)];
}
