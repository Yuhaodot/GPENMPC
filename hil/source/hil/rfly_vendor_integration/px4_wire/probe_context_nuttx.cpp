#include "RflyContextAssembler.hpp"
#include "RflyIngressDispatch.hpp"
#include "../px4_runtime/Px4CanonicalIo.hpp"
#include <uORB/topics/uORBTopics.hpp>
#include <uORB/topics/gpenmpc_full_inner_ingress.h>
static_assert(ORB_TOPICS_COUNT==311,"same full topic metadata");
extern "C" bool gpenmpc_context_export_nuttx(gpenmpc_context_wire::AnchorStore<2>&anchors,
    const gpenmpc_rfly_px4::Px4CanonicalIo&io,const gpenmpc_rfly_state_execution::SnapshotTicket&ticket,
    const gpenmpc_rfly_execution::Configuration&approved,std::uint64_t now,gpenmpc_snapshot_wire::Bytes&out)noexcept{
    return anchors.record(io,ticket,approved,now,out);
}
extern "C" bool gpenmpc_context_accept_nuttx(gpenmpc_context_wire::ContextBinding&binding,
    const gpenmpc_context_wire::Context&context,const gpenmpc_context_wire::AnchorStore<2>&anchors,
    std::uint64_t original,std::uint64_t now)noexcept{return binding.accept(context,anchors,original,now);}
extern "C" bool gpenmpc_context_assemble_nuttx(gpenmpc_context_wire::Assembler&assembler,
    const gpenmpc_full_inner_ingress_s&topic,std::uint64_t generation,std::uint64_t now)noexcept{
    return assembler.receive(gpenmpc_argument_transport::from_topic(topic,generation),now);
}
extern "C" bool gpenmpc_context_take_nuttx(gpenmpc_context_wire::Assembler&assembler,
    gpenmpc_context_wire::Context&context,std::uint64_t&original,std::uint64_t now)noexcept{return assembler.take(context,original,now);}
extern "C" bool gpenmpc_dispatch_receive_nuttx(gpenmpc_rfly_wire::IngressDispatch&dispatch,
    const gpenmpc_full_inner_ingress_s&topic,std::uint64_t generation,std::uint64_t now)noexcept{
    return dispatch.receive(gpenmpc_argument_transport::from_topic(topic,generation),now);
}
extern "C" bool gpenmpc_dispatch_context_nuttx(gpenmpc_rfly_wire::IngressDispatch&dispatch,
    std::uint64_t generation,std::uint64_t now)noexcept{return dispatch.expect_context(generation,now);}
extern "C" bool gpenmpc_dispatch_numeric_nuttx(gpenmpc_rfly_wire::IngressDispatch&dispatch,
    const gpenmpc_argument_transport::Metadata&metadata,std::uint64_t now)noexcept{return dispatch.expect_numerical(metadata,now);}
extern "C" {
char gpenmpc_context_size_AnchorStore2[sizeof(gpenmpc_context_wire::AnchorStore<2>)];
char gpenmpc_context_size_ContextBinding[sizeof(gpenmpc_context_wire::ContextBinding)];
char gpenmpc_context_size_Assembler[sizeof(gpenmpc_context_wire::Assembler)];
char gpenmpc_context_size_Dispatcher[sizeof(gpenmpc_rfly_wire::IngressDispatch)];
}
