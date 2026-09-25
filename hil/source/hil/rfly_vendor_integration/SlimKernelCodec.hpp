#pragma once
// Snapshot-bound numerical codec for 30 binary64 values, continuity and
// both source generations. Transport framing is supplied by the caller.
#include "RflySnapshotBoundExecutor.hpp"
#include "CanonicalSnapshotStateMapping.hpp"

namespace gpenmpc_rfly_slim {
using Digest=gpenmpc_rfly_execution::Digest;
constexpr std::size_t encoded_size=261;
using Bytes=gpenmpc_portable::Array<std::uint8_t,encoded_size>;
using Expanded=gpenmpc_portable::Array<std::uint8_t,gpenmpc_kernel_abi::encoded_size>;
enum class Failure {None,Length,Magic,DigestMismatch,Boolean,Nonfinite,Snapshot,Configuration,Lineage};

class Writer {
public:
    explicit Writer(std::uint8_t*p)noexcept:p_(p){}
    void byte(std::uint8_t v)noexcept{*p_++=v;}
    void u64(std::uint64_t v)noexcept{for(unsigned i=0;i<8;++i)byte(std::uint8_t(v>>(56-8*i)));}
    void real(double v)noexcept{std::uint64_t b=0;std::memcpy(&b,&v,8);u64(b);}
    template<std::size_t N>void reals(const gpenmpc_portable::Array<double,N>&a)noexcept{for(double v:a)real(v);}
private:std::uint8_t*p_;
};
template<std::size_t N>inline Digest digest(const gpenmpc_portable::Array<std::uint8_t,N>&a)noexcept{
    gpenmpc_consumption::CanonicalSha256 hash;for(auto v:a)hash.byte(v);return hash.finish();
}
inline Bytes encode(const gpenmpc_consumption::KernelArguments&k)noexcept{
    Bytes out{};Writer w(out.data());w.byte('R');w.byte('K');w.byte('S');w.byte('1');
    w.reals(k.refP);w.reals(k.refV);w.reals(k.refA);w.real(k.payload);
    w.reals(k.windXY);w.reals(k.augmentation);w.byte(k.continuityEnabled);
    w.reals(k.commandR);w.reals(k.commandOmega);w.reals(k.commandOmegaDot);
    w.u64(k.augmentation_state_generation);w.u64(k.continuity_state_generation);return out;
}
inline void expand_into(const gpenmpc_consumption::KernelArguments&k,Expanded&out)noexcept{
    Writer w(out.data());w.byte('R');w.byte('A');w.byte('K');w.byte('1');
    w.reals(k.x);w.reals(k.refP);w.reals(k.refV);w.reals(k.refA);w.real(k.payload);
    w.reals(k.windXY);w.reals(k.augmentation);w.byte(k.continuityEnabled);
    w.reals(k.commandR);w.reals(k.commandOmega);w.reals(k.commandOmegaDot);
    const auto&p=k.parameters;w.reals(p.kp);w.reals(p.kd);w.reals(p.kr);w.reals(p.kw);
    w.reals(p.drag);w.reals(p.inertia);w.reals(p.pseudoinverse);
    w.real(p.baseMass);w.real(p.totalThrust);w.real(p.rotorUpper);w.real(p.maxTilt);
    w.u64(k.augmentation_state_generation);w.u64(k.continuity_state_generation);
}
inline Expanded expand_bytes(const gpenmpc_consumption::KernelArguments&k)noexcept{
    Expanded out{};expand_into(k,out);return out;
}

inline bool decode(const std::uint8_t*data,std::size_t size,const Digest&expected,
    const gpenmpc_odometry::Snapshot&snapshot,const gpenmpc_rfly_execution::Configuration&approved,
    gpenmpc_consumption::KernelArguments&out,Failure&failure)noexcept{
    out={};failure=Failure::None;
    if(!data||size!=encoded_size){failure=Failure::Length;return false;}
    if(data[0]!='R'||data[1]!='K'||data[2]!='S'||data[3]!='1'){failure=Failure::Magic;return false;}
    gpenmpc_consumption::CanonicalSha256 hash;for(std::size_t i=0;i<size;++i)hash.byte(data[i]);
    if(hash.finish()!=expected){failure=Failure::DigestMismatch;return false;}
    if(!snapshot.valid()){failure=Failure::Snapshot;return false;}
    const auto&s=snapshot.estimator();
    if(!(s.identity==approved.identity)||approved.configuration_payload_sha256!=gpenmpc_rfly_execution::kCanonicalConfigurationSha||
       approved.kernel_source_sha256!=gpenmpc_rfly_execution::kGeneratedArmSourceSha||
       approved.matlab_extraction_source_sha256!=gpenmpc_rfly_execution::kMatlabExtractionSha||
       approved.wrapper_matlab_source_sha256!=gpenmpc_rfly_execution::kWrapperMatlabSourceSha||
       gpenmpc_rfly_execution::parameter_sha256(approved.approved_parameters)!=approved.approved_parameter_sha256||
       approved.encoding!=gpenmpc_rfly_execution::RflyEncoding::OfficialHIL16CtrlsNorm){failure=Failure::Configuration;return false;}
    gpenmpc_consumption::KernelArguments k{};gpenmpc_kernel_abi::Reader r(data+4);
    r.reals(k.refP);r.reals(k.refV);r.reals(k.refA);k.payload=r.real();
    r.reals(k.windXY);r.reals(k.augmentation);const auto boolean=r.byte();
    if(boolean>1){failure=Failure::Boolean;return false;}k.continuityEnabled=boolean!=0;
    r.reals(k.commandR);r.reals(k.commandOmega);r.reals(k.commandOmegaDot);
    k.augmentation_state_generation=r.u64();k.continuity_state_generation=r.u64();
    if(k.augmentation_state_generation!=s.generation||k.continuity_state_generation!=s.generation){failure=Failure::Lineage;return false;}
    // The state and all 52 immutable parameter doubles come only from the
    // store-owned snapshot and the executor's startup configuration, never wire.
    gpenmpc_portable::Array<double,13> x13{};
    if(!gpenmpc_snapshot_mapping::state13(snapshot,x13)){failure=Failure::Nonfinite;return false;}
    for(unsigned j=0;j<13;++j)k.x[j]=x13[j]; // k.x[13:18] retains the original explicit zero tail
    k.parameters=approved.approved_parameters;
    if(!gpenmpc_consumption::finite_kernel(k)){failure=Failure::Nonfinite;return false;}
    out=k;return true;
}
} // namespace gpenmpc_rfly_slim
