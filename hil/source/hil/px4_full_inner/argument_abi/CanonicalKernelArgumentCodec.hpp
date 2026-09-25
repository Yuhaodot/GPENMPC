#pragma once
// Numerical test/deployment ABI only; no COM, flight mode or publication.
#include "../consumption/ConsumptionBinding.hpp"
#include <cstddef>
#include <cstring>

namespace gpenmpc_kernel_abi {
constexpr std::size_t encoded_size = 829;
enum class DecodeFailure { None, Length, Magic, Boolean, Nonfinite, Generation, Digest };
class Reader {
public:
    explicit Reader(const std::uint8_t *p) noexcept : p_(p) {}
    std::uint8_t byte() noexcept { return p_[offset_++]; }
    std::uint64_t u64() noexcept {
        std::uint64_t n=0; for (unsigned k=0;k<8;++k) n=(n<<8)|byte(); return n;
    }
    double real() noexcept { const auto n=u64();double d=0;std::memcpy(&d,&n,8);return d; }
    template<std::size_t N> void reals(gpenmpc_portable::Array<double,N> &x) noexcept { for(auto &v:x)v=real(); }
private:
    const std::uint8_t *p_;std::size_t offset_{0};
};
inline bool decode(const std::uint8_t *bytes,std::size_t size,
                   const gpenmpc_portable::Array<std::uint32_t,8>& expected_digest,
                   gpenmpc_consumption::KernelArguments &result,DecodeFailure &failure) noexcept {
    result={};failure=DecodeFailure::None;
    if(!bytes||size!=encoded_size){failure=DecodeFailure::Length;return false;}
    if(bytes[0]!='R'||bytes[1]!='A'||bytes[2]!='K'||bytes[3]!='1'){failure=DecodeFailure::Magic;return false;}
    gpenmpc_consumption::CanonicalSha256 hash;
    for(std::size_t k=0;k<size;++k)hash.byte(bytes[k]);
    if(hash.finish()!=expected_digest){failure=DecodeFailure::Digest;return false;}
    Reader r(bytes+4);gpenmpc_consumption::KernelArguments k{};
    r.reals(k.x);r.reals(k.refP);r.reals(k.refV);r.reals(k.refA);k.payload=r.real();
    r.reals(k.windXY);r.reals(k.augmentation);const auto continuity=r.byte();
    if(continuity>1){failure=DecodeFailure::Boolean;return false;}
    k.continuityEnabled=continuity!=0;
    r.reals(k.commandR);r.reals(k.commandOmega);r.reals(k.commandOmegaDot);
    auto &p=k.parameters;
    r.reals(p.kp);r.reals(p.kd);r.reals(p.kr);r.reals(p.kw);r.reals(p.drag);
    r.reals(p.inertia);r.reals(p.pseudoinverse);
    p.baseMass=r.real();p.totalThrust=r.real();p.rotorUpper=r.real();p.maxTilt=r.real();
    k.augmentation_state_generation=r.u64();k.continuity_state_generation=r.u64();
    if(!gpenmpc_consumption::finite_kernel(k)){failure=DecodeFailure::Nonfinite;return false;}
    if(!k.augmentation_state_generation||!k.continuity_state_generation){failure=DecodeFailure::Generation;return false;}
    // Independent typed-field digest must agree with MATLAB's byte digest.
    if(gpenmpc_consumption::kernel_arguments_sha256(k)!=expected_digest){failure=DecodeFailure::Digest;return false;}
    result=k;return true;
}
} // namespace gpenmpc_kernel_abi
