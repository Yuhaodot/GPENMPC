#pragma once
#include "SlimKernelCodec.hpp"

namespace gpenmpc_rfly_slim {
// Only numerical fields differ from the existing command. No host state or
// host parameter copy is accepted by this entry point.
struct Command {
    gpenmpc_rfly_state_execution::SnapshotTicket snapshot_ticket{};
    Bytes arguments{};
    Digest argument_sha256{},configuration_sha256{},matlab_source_sha256{},generated_source_sha256{},wrapper_matlab_source_sha256{};
    gpenmpc_consumption::Reference reference{};
    gpenmpc_consumption::OuterCommand outer{};
    std::uint64_t original_board_ingress_us{0};
};
template<std::size_t Capacity=4>class SnapshotExecutor final {
public:
    SnapshotExecutor(const gpenmpc_odometry::Configuration &state,const gpenmpc_rfly_execution::Configuration &execution,
                     gpenmpc_rfly_execution::Clock clock)noexcept:configuration_(execution),core_(state,execution,clock){}
    bool capture(const vehicle_odometry_s&raw,std::uint32_t generation,std::uint64_t original_receipt,
                 gpenmpc_consumption::Identity identity,const void*topic,std::uint8_t instance,
                 gpenmpc_rfly_state_execution::SnapshotTicket&ticket)noexcept{
        if(front_failure_!=gpenmpc_rfly_state_execution::Failure::None){ticket={};return false;}
        return core_.capture(raw,generation,original_receipt,identity,topic,instance,ticket);
    }
    // Store-owned read-only view, with the same lifetime as the core accessor.
    // It does not accept HOST state, parameters, or replacement timestamps.
    const gpenmpc_odometry::Snapshot*snapshot(const gpenmpc_rfly_state_execution::SnapshotTicket&ticket)const noexcept{
        if(failure()!=gpenmpc_rfly_state_execution::Failure::None)return nullptr;
        return core_.snapshot(ticket);
    }
    bool releaseObservation(const gpenmpc_rfly_state_execution::SnapshotTicket&ticket)noexcept{
        return failure()==gpenmpc_rfly_state_execution::Failure::None&&core_.releaseObservation(ticket);
    }
    bool prepareNumericalOnly(const Command&command,gpenmpc_rfly_state_execution::NumericalPrepared&out)noexcept{
        out={};if(failure()!=gpenmpc_rfly_state_execution::Failure::None)return false;
        const auto*snapshot=core_.snapshot(command.snapshot_ticket);
        if(!snapshot)return fail(gpenmpc_rfly_state_execution::Failure::Ticket);
        if(command.configuration_sha256!=configuration_.configuration_payload_sha256||
           command.matlab_source_sha256!=configuration_.matlab_extraction_source_sha256||
           command.generated_source_sha256!=configuration_.kernel_source_sha256||
           command.wrapper_matlab_source_sha256!=configuration_.wrapper_matlab_source_sha256)
            return fail(gpenmpc_rfly_state_execution::Failure::SourceConfiguration);
        arguments_scratch_={};expanded_scratch_={};
        if(!decode(command.arguments.data(),command.arguments.size(),command.argument_sha256,
                   *snapshot,configuration_,arguments_scratch_,codec_failure_))return fail(gpenmpc_rfly_state_execution::Failure::Abi);
        auto&expanded=expanded_scratch_;
        expanded.snapshot_ticket=command.snapshot_ticket;expand_into(arguments_scratch_,expanded.arguments);
        expanded.argument_sha256=digest(expanded.arguments);expanded.configuration_sha256=command.configuration_sha256;
        expanded.matlab_source_sha256=command.matlab_source_sha256;expanded.generated_source_sha256=command.generated_source_sha256;
        expanded.wrapper_matlab_source_sha256=command.wrapper_matlab_source_sha256;
        expanded.reference=command.reference;expanded.outer=command.outer;
        expanded.original_board_ingress_us=command.original_board_ingress_us;
        // The private store validates ticket replay, age, source, parameters and ACK.
        return core_.prepareNumericalOnly(expanded,out);
    }
    bool commitNumericalReceipt(const gpenmpc_rfly_state_execution::SnapshotTicket&ticket,
        const gpenmpc_rfly_execution::BackendRflyAck&ack,gpenmpc_rfly_state_execution::NumericalReceipt&out)noexcept{
        out={};if(failure()!=gpenmpc_rfly_state_execution::Failure::None)return false;
        return core_.commitNumericalReceipt(ticket,ack,out);
    }
    gpenmpc_rfly_state_execution::Failure failure()const noexcept{
        return front_failure_==gpenmpc_rfly_state_execution::Failure::None?core_.failure():front_failure_;
    }
    Failure codec_failure()const noexcept{return codec_failure_;}
    std::uint64_t kernel_calls()const noexcept{return core_.kernel_calls();}
private:
    bool fail(gpenmpc_rfly_state_execution::Failure reason)noexcept{front_failure_=reason;return false;}
    const gpenmpc_rfly_execution::Configuration configuration_;
    gpenmpc_rfly_state_execution::RflySnapshotBoundExecutor<Capacity>core_;
    // Serialized single-owner access with fixed scratch storage.
    gpenmpc_consumption::KernelArguments arguments_scratch_{};
    gpenmpc_rfly_state_execution::NumericalCommand expanded_scratch_{};
    gpenmpc_rfly_state_execution::Failure front_failure_{gpenmpc_rfly_state_execution::Failure::None};
    Failure codec_failure_{Failure::None};
};
} // namespace gpenmpc_rfly_slim
