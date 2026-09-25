#pragma once
#include "RflySnapshotWire.hpp"
#include "../px4_runtime/CommittedFeedback.hpp"
namespace gpenmpc_feedback_wire {
using gpenmpc_rfly_px4::CommittedFeedback;
using gpenmpc_rfly_px4::FeedbackDisposition;
constexpr std::size_t message_bytes=1112,fragment_count=10;
constexpr std::uint8_t schema_version=5;
using Bytes=gpenmpc_portable::Array<std::uint8_t,message_bytes>;
inline bool valid(const CommittedFeedback&f)noexcept{
    const auto d=static_cast<unsigned>(f.disposition);
    if(d<1||d>3||!f.token.identity.uid||!f.token.identity.boot_generation||!f.token.identity.system||!f.token.identity.component||
       !f.token.output_generation||!f.token.sample_generation||f.token.publication_path!=gpenmpc_consumption::PublicationPath::DirectCanonicalMotors||!f.kernel_completed_us||
       f.publication_us<f.kernel_completed_us||f.commit_completed_us<f.publication_us||
       f.original_valid_until_us<f.commit_completed_us||f.token.kernel_source_sha256!=f.generated_arm_source_sha256)return false;
    for(double v:f.actual61)if(!std::isfinite(v))return false;
    for(float v:f.published_control16)if(!std::isfinite(v)||v<-1.f||v>1.f)return false;
    return true;
}
#define GPENMPC_FEEDBACK_TOKEN_U64(F) F(transaction) F(output_generation) F(sample_generation) F(timestamp_sample_us) \
 F(source_generation_delta) F(state_publication_us) F(state_board_rx_us) F(control_tick_us) F(sample_delta_us) \
 F(actual_tick_delta_us) F(reference_generation) F(reference_timestamp_us) F(reference_board_rx_us) F(reference_valid_until_us) \
 F(outer_generation) F(outer_board_rx_us) F(outer_valid_until_us) F(outer_based_on_sample_generation) F(outer_based_on_timestamp_sample_us)
inline bool encode(const CommittedFeedback&f,Bytes&out)noexcept{
    out={};if(!valid(f))return false;gpenmpc_snapshot_wire::Writer w(out.data());
    w.byte('R');w.byte('F');w.byte('C');w.byte('1');w.byte(static_cast<std::uint8_t>(f.disposition));w.bytes(f.snapshot_ticket);
    const auto&t=f.token;w.u64(t.identity.uid);w.u64(t.identity.boot_generation);w.byte(t.identity.system);w.byte(t.identity.component);
    w.byte(static_cast<std::uint8_t>(t.publication_path));
#define RF_WRITE(name) w.u64(t.name);
    GPENMPC_FEEDBACK_TOKEN_U64(RF_WRITE)
#undef RF_WRITE
    w.bytes(gpenmpc_argument_transport::bytes_of(t.outer_payload_sha256));w.bytes(gpenmpc_argument_transport::bytes_of(t.full_input_sha256));
    w.bytes(gpenmpc_argument_transport::bytes_of(t.kernel_argument_sha256));w.bytes(gpenmpc_argument_transport::bytes_of(t.kernel_source_sha256));
    w.bytes(gpenmpc_argument_transport::bytes_of(f.configuration_payload_sha256));w.bytes(gpenmpc_argument_transport::bytes_of(f.approved_parameter_sha256));
    w.bytes(gpenmpc_argument_transport::bytes_of(f.matlab_extraction_source_sha256));w.bytes(gpenmpc_argument_transport::bytes_of(f.generated_arm_source_sha256));
    w.bytes(gpenmpc_argument_transport::bytes_of(f.wrapper_matlab_source_sha256));
    for(double v:f.actual61)w.real(v);
    for(float v:f.published_control16)w.f32(v);
    w.u64(f.kernel_completed_us);w.u64(f.publication_us);w.u64(f.commit_completed_us);w.u64(f.original_valid_until_us);
    w.bytes(gpenmpc_argument_transport::digest(out.data(),message_bytes-32));return true;
}
inline bool decode(const Bytes&bytes,CommittedFeedback&out)noexcept{
    out={};if(bytes[0]!='R'||bytes[1]!='F'||bytes[2]!='C'||bytes[3]!='1')return false;
    const auto sha=gpenmpc_argument_transport::digest(bytes.data(),message_bytes-32);
    if(std::memcmp(sha.data(),bytes.data()+message_bytes-32,32)!=0)return false;
    out.disposition=static_cast<FeedbackDisposition>(bytes[4]);std::memcpy(out.snapshot_ticket.data(),bytes.data()+5,32);
    gpenmpc_kernel_abi::Reader r(bytes.data()+37);auto&t=out.token;
    t.identity.uid=r.u64();t.identity.boot_generation=r.u64();t.identity.system=r.byte();t.identity.component=r.byte();
    t.publication_path=static_cast<gpenmpc_consumption::PublicationPath>(r.byte());
#define RF_READ(name) t.name=r.u64();
    GPENMPC_FEEDBACK_TOKEN_U64(RF_READ)
#undef RF_READ
    auto digest_read=[&r](gpenmpc_rfly_execution::Digest&d){for(auto&w:d){w=0;for(unsigned j=0;j<4;++j)w=(w<<8)|r.byte();}};
    digest_read(t.outer_payload_sha256);digest_read(t.full_input_sha256);digest_read(t.kernel_argument_sha256);digest_read(t.kernel_source_sha256);
    digest_read(out.configuration_payload_sha256);digest_read(out.approved_parameter_sha256);digest_read(out.matlab_extraction_source_sha256);
    digest_read(out.generated_arm_source_sha256);digest_read(out.wrapper_matlab_source_sha256);r.reals(out.actual61);
    for(auto&v:out.published_control16){std::uint32_t bits=0;for(unsigned j=0;j<4;++j)bits=(bits<<8)|r.byte();std::memcpy(&v,&bits,4);}
    out.kernel_completed_us=r.u64();out.publication_us=r.u64();out.commit_completed_us=r.u64();out.original_valid_until_us=r.u64();return valid(out);
}
#undef GPENMPC_FEEDBACK_TOKEN_U64
inline bool fragment(const Bytes&bytes,unsigned index,gpenmpc_argument_transport::Fragment&out)noexcept{
    out={};if(index>=fragment_count)return false;const auto generation=gpenmpc_argument_transport::get64(bytes.data()+64);
    if(!generation)return false;
    const std::size_t offset=index*119,n=message_bytes-offset<119?message_bytes-offset:119;
    out.payload[0]=std::uint8_t((schema_version<<4)|index);gpenmpc_argument_transport::put64(out.payload+1,generation);
    std::memcpy(out.payload+9,bytes.data()+offset,n);out.length=std::uint8_t(n+9);return true;
}
} // namespace gpenmpc_feedback_wire
