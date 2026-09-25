#pragma once
// Host policy for executing the ARM-target generated C.
#include "../px4_full_inner/consumption/ConsumptionBinding.hpp"
extern "C" {
#include "GPENMPC_Rfly_Canonical_Controller.h"
}

namespace gpenmpc_rfly_execution {
enum class RflyEncoding : std::uint8_t { Unspecified, OfficialHIL16CtrlsNorm };
class SimulinkCanonicalPolicy final {
public:
    static constexpr double upper_n()noexcept{return 32.145727009134916;}

    bool execute(const gpenmpc_consumption::KernelArguments &k, bool generation_accepted,
                 gpenmpc_portable::Array<double,61> &full,
                 gpenmpc_portable::Array<float,16> &controls) noexcept {
        full={};controls={};
        // Global U/Y storage in this generated algebraic model is single-owner.
        // A concurrent attempt is rejected, never an extra or interleaved step.
        if(__atomic_test_and_set(&model_lock(),__ATOMIC_ACQUIRE))return false;
        GPENMPC_Rfly_Canonical_Control_U={};
        GPENMPC_Rfly_Canonical_Control_Y={};
        double *dst=GPENMPC_Rfly_Canonical_Control_U.KernelArguments101;
        append(dst,k.x);append(dst,k.refP);append(dst,k.refV);append(dst,k.refA);
        *dst++=k.payload;append(dst,k.windXY);append(dst,k.augmentation);
        append(dst,k.commandR);append(dst,k.commandOmega);append(dst,k.commandOmegaDot);
        const auto&p=k.parameters;
        append(dst,p.kp);append(dst,p.kd);append(dst,p.kr);append(dst,p.kw);append(dst,p.drag);
        append(dst,p.inertia);append(dst,p.pseudoinverse);
        *dst++=p.baseMass;*dst++=p.totalThrust;*dst++=p.rotorUpper;*dst++=p.maxTilt;
        if(dst!=GPENMPC_Rfly_Canonical_Control_U.KernelArguments101+101){
            __atomic_clear(&model_lock(),__ATOMIC_RELEASE);return false;
        }
        GPENMPC_Rfly_Canonical_Control_U.ContinuityEnabled=k.continuityEnabled;
        GPENMPC_Rfly_Canonical_Control_U.InputGenerationAccepted=generation_accepted;
        // Generated initialize/terminate are empty; this stateless step is the
        // only numerical kernel called. Generation acceptance is supplied only
        // after the snapshot and transaction checks pass.
        GPENMPC_Rfly_Canonical_Controller_step();++step_calls_;
        const bool valid=GPENMPC_Rfly_Canonical_Control_Y.OutputValid;
        if(valid){
            for(unsigned i=0;i<61;++i)full[i]=GPENMPC_Rfly_Canonical_Control_Y.FullKernel61[i];
            for(unsigned i=0;i<16;++i)controls[i]=GPENMPC_Rfly_Canonical_Control_Y.Controls16[i];
        }
        __atomic_clear(&model_lock(),__ATOMIC_RELEASE);return valid;
    }
    std::uint64_t step_calls()const noexcept{return step_calls_;}
    static bool encoding_matches(const gpenmpc_portable::Array<double,6>&rotor,
                                 const gpenmpc_portable::Array<float,16>&controls)noexcept {
        gpenmpc_portable::Array<float,16> expected{};
        constexpr unsigned canonical_to_official[6]={4,0,3,5,1,2};
        for(unsigned j=0;j<6;++j){
            if(!std::isfinite(rotor[j])||rotor[j]<0.0||rotor[j]>upper_n())return false;
            expected[canonical_to_official[j]]=static_cast<float>(rotor[j]/upper_n());
        }
        // Verify the step's payload, do not replace it with this computed oracle.
        for(unsigned j=0;j<16;++j){
            if(!std::isfinite(controls[j]))return false;
            std::uint32_t a=0,b=0;std::memcpy(&a,&controls[j],4);std::memcpy(&b,&expected[j],4);
            if(a!=b)return false;
        }
        return true;
    }
private:
    template<std::size_t N>static void append(double*&dst,const gpenmpc_portable::Array<double,N>&src)noexcept{
        for(double value:src)*dst++=value;
    }
    // Lock-free byte atomic for the C++14 host and Cortex-M7 targets.
    static unsigned char &model_lock()noexcept{
        static_assert(__atomic_always_lock_free(1,nullptr),"requires lock-free byte atomic");
        static unsigned char value=0;return value;
    }
    std::uint64_t step_calls_{0};
};
} // namespace gpenmpc_rfly_execution
