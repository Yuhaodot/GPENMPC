#pragma once
// Numerical executor with admission, mapping and transaction checks.
// Uses the generated controller identity and a 16-float acknowledgement.
// The caller owns publication, scheduling and board safety admission;
// caller-supplied input flags do not replace those checks.
#include "../px4_full_inner/consumption/ConsumptionBinding.hpp"
#include "SimulinkCanonicalPolicy.hpp"
#include <cstring>

namespace gpenmpc_rfly_execution {
using namespace gpenmpc_consumption;
using Digest = gpenmpc_portable::Array<std::uint32_t,8>;
constexpr Digest kGeneratedArmSourceSha{0xA47F1C25,0x5BDAC1DA,0xE712494B,0xE8D9D66F,0xC4F83138,0xDBA9B0C2,0xE3A31E11,0x4D4ABD0F};
constexpr Digest kWrapperMatlabSourceSha{0x9117D3CD,0xF8E924A2,0x53F4C444,0xCDD467E4,0x850D6F11,0xCBF5744B,0x26375B95,0x0E2A95B5};
constexpr Digest kMatlabExtractionSha{0x52F714D6,0x5C4A3CD5,0x879FFD44,0x78F60379,0xF68CD5F6,0x5A38F597,0x1B91BBC0,0x1D0E4AB8};
constexpr Digest kCanonicalConfigurationSha{0xA859433D,0x0AA77401,0x3341444A,0x4B9AE971,0xA34F12B4,0xFB89C0A0,0x04B2CB3E,0x013FCEBA};

enum class Coordinates : std::uint8_t { Unspecified, StateAndReferenceAreTaskLocalNed };
enum class Error : std::uint8_t {
    None, Configuration, Pending, InputAdmission, SourceIdentity, ParameterIdentity,
    Quaternion, StateMapping, ReferenceMapping, HiddenRotorState, StateGeneration,
    ContinuityDisabled, KernelInvalid, ConversionInvalid, KernelCompletion,
    NotPrepared, AckMismatch, BackendFailure, CommitRejected, Exception
};

inline Digest parameter_sha256(const KernelParameters &p) noexcept {
    CanonicalSha256 h; h.u32(0x52415031); // RAP1, not raw struct memory
    h.reals(p.kp);h.reals(p.kd);h.reals(p.kr);h.reals(p.kw);h.reals(p.drag);
    h.reals(p.inertia);h.reals(p.pseudoinverse);
    h.real(p.baseMass);h.real(p.totalThrust);h.real(p.rotorUpper);h.real(p.maxTilt);
    return h.finish();
}

struct Configuration {
    Identity identity{};
    Limits limits{};
    Coordinates coordinates{Coordinates::Unspecified};
    KernelParameters approved_parameters{};
    Digest approved_parameter_sha256{};
    Digest configuration_payload_sha256{}; // independently checked by outer service
    Digest kernel_source_sha256{};
    Digest matlab_extraction_source_sha256{};
    Digest wrapper_matlab_source_sha256{};
    RflyEncoding encoding{RflyEncoding::Unspecified};
};

struct ExecutionInput {
    Input consumed{}; // consumed.kernel_source_sha256 = GENERATED C source SHA
    // kernel_source_sha256 identifies the MATLAB extraction source.
    Digest matlab_extraction_source_sha256{};
    Digest wrapper_matlab_source_sha256{};
};

struct Clock {
    std::uint64_t (*now_us)(void *) noexcept{nullptr};
    void *context{nullptr};
};

struct Prepared {
    bool numerically_prepared{false};
    bool board_authority_proven{false}; // always false here
    Token token{};
    Digest approved_configuration_sha256{};
    Digest approved_parameter_sha256{};
    Digest matlab_extraction_source_sha256{}, generated_c_source_sha256{};
    KernelOutput actual_output{};
    gpenmpc_portable::Array<double,61> actual61{};
    Digest wrapper_matlab_source_sha256{};
    RflyEncoding encoding{RflyEncoding::Unspecified};
    gpenmpc_portable::Array<float,16> rfly_controls16{};
    std::uint64_t kernel_completed_us{0};
};

// Receipt for the published 16-float normalized payload. The caller supplies
// the publisher result; downstream transport delivery is tracked separately.
struct BackendRflyAck {
    Token token{};
    bool publish_succeeded{false};
    std::uint64_t output_generation{0}, publication_us{0};
    gpenmpc_portable::Array<float,16> published_control{};
    RflyEncoding encoding{RflyEncoding::Unspecified};
    Digest generated_arm_source_sha256{}, wrapper_matlab_source_sha256{};
};
struct Committed {
    bool publication_receipt_valid{false};
    bool pwm_sim_consumption_proven{false}; // always false here
    Receipt binding{};
    Digest configuration_payload_sha256{};
    gpenmpc_portable::Array<float,16> published_control{};
    RflyEncoding encoding{RflyEncoding::Unspecified};
    Digest generated_arm_source_sha256{}, wrapper_matlab_source_sha256{};
};

class CanonicalRflyExecutor final {
public:
    CanonicalRflyExecutor(const Configuration &config, Clock clock) noexcept : config_(config), clock_(clock), binding_(config.identity,config.limits) {
        if (binding_.failed() || !clock.now_us || config.coordinates!=Coordinates::StateAndReferenceAreTaskLocalNed ||
            config.limits.publication_path!=PublicationPath::DirectCanonicalMotors ||
            config.kernel_source_sha256!=kGeneratedArmSourceSha || config.configuration_payload_sha256!=kCanonicalConfigurationSha ||
            config.matlab_extraction_source_sha256!=kMatlabExtractionSha ||
            parameter_sha256(config.approved_parameters)!=config.approved_parameter_sha256 ||
            config.wrapper_matlab_source_sha256!=kWrapperMatlabSourceSha ||
            !gpenmpc_portable::exact_equal(config.approved_parameters.rotorUpper,SimulinkCanonicalPolicy::upper_n()) ||
            config.encoding!=RflyEncoding::OfficialHIL16CtrlsNorm) {
            fail(Error::Configuration); return;
        }
        // The policy owns the generated Simulink step and
        // official 16-channel encoding. No legacy kernel or PWM conversion.
    }

    bool prepare(const ExecutionInput &execution, Prepared &out) noexcept {
        out={};out.rfly_controls16.fill(gpenmpc_portable::Ieee754<float>::quiet_NaN());
        if (error_!=Error::None) return false;
        if (pending_) return fail(Error::Pending);
        const auto &in=execution.consumed;
        if (execution.matlab_extraction_source_sha256!=config_.matlab_extraction_source_sha256) return fail(Error::SourceIdentity);
        if (in.kernel_source_sha256!=config_.kernel_source_sha256 ||
            execution.wrapper_matlab_source_sha256!=config_.wrapper_matlab_source_sha256) return fail(Error::SourceIdentity);
        if (parameter_sha256(in.kernel.parameters)!=config_.approved_parameter_sha256) return fail(Error::ParameterIdentity);
        if (!in.kernel.continuityEnabled) return fail(Error::ContinuityDisabled);
        if (in.kernel.augmentation_state_generation!=in.state.generation ||
            in.kernel.continuity_state_generation!=in.state.generation || !in.state.generation) return fail(Error::StateGeneration);
        if (!mapped_input_matches(in)) return false;
        Token token{}; if (!binding_.begin(in,token)) return fail(Error::InputAdmission);
        const auto *bound=binding_.pending_input();
        if (!bound) return fail(Error::InputAdmission);
        const auto &k=bound->kernel;
        // Private scratch is never exposed before every completion check passes.
        // Reusing the existing pending member avoids a large per-tick stack
        // candidate and its extra copy. Failure clears pending, not publication.
        prepared_={};
        auto &candidate=prepared_;candidate.token=token;
        candidate.approved_configuration_sha256=config_.configuration_payload_sha256;
        candidate.approved_parameter_sha256=config_.approved_parameter_sha256;
        candidate.matlab_extraction_source_sha256=execution.matlab_extraction_source_sha256;
        candidate.generated_c_source_sha256=in.kernel_source_sha256;
        candidate.wrapper_matlab_source_sha256=config_.wrapper_matlab_source_sha256;
        candidate.encoding=config_.encoding;
        const bool valid=policy_.execute(k, true, candidate.actual61, candidate.rfly_controls16);
        kernel_calls_=policy_.step_calls();
        gpenmpc_portable::copy_n(candidate.actual61.begin(),4,candidate.actual_output.wrench.begin());
        gpenmpc_portable::copy_n(candidate.actual61.begin()+4,6,candidate.actual_output.rotor.begin());
        if (!valid || !finite(candidate.actual61)) return fail(Error::KernelInvalid);
        candidate.kernel_completed_us=clock_.now_us(clock_.context);
        if (!binding_.kernel_completed(token,candidate.kernel_completed_us,true,candidate.actual_output,k)) return fail(Error::KernelCompletion);
        if (!SimulinkCanonicalPolicy::encoding_matches(candidate.actual_output.rotor,candidate.rfly_controls16)) return fail(Error::ConversionInvalid);
        candidate.numerically_prepared=true;pending_=true;out=candidate;return true;
    }

    bool commit(const BackendRflyAck &ack, Committed &out) noexcept {
        out={};
        if (error_!=Error::None) return false;
        last_ack_=ack;ack_observed_=true;
        if (!pending_) return fail(Error::NotPrepared);
        if (!(ack.token==prepared_.token) || ack.output_generation!=prepared_.token.output_generation ||
            !same_control_bits(ack.published_control,prepared_.rfly_controls16) ||
            ack.encoding!=prepared_.encoding || ack.generated_arm_source_sha256!=prepared_.generated_c_source_sha256 ||
            ack.wrapper_matlab_source_sha256!=prepared_.wrapper_matlab_source_sha256) return fail(Error::AckMismatch);
        MotorPublicationAck committed{ack.token,ack.publish_succeeded,ack.output_generation,ack.publication_us,
            prepared_.actual_output.rotor,kernel_output_sha256(prepared_.actual_output)};
        Receipt receipt{};
        if (!binding_.motors_committed(committed,clock_.now_us(clock_.context),receipt))
            return fail(ack.publish_succeeded?Error::CommitRejected:Error::BackendFailure);
        out.publication_receipt_valid=true;out.binding=receipt;
        out.configuration_payload_sha256=config_.configuration_payload_sha256;out.published_control=ack.published_control;
        out.encoding=ack.encoding;out.generated_arm_source_sha256=ack.generated_arm_source_sha256;
        out.wrapper_matlab_source_sha256=ack.wrapper_matlab_source_sha256;
        pending_=false;return true;
    }

    void note_exception() noexcept { binding_.note_exception();fail(Error::Exception); }
    Error error() const noexcept { return error_; }
    Fault consumption_fault() const noexcept { return binding_.fault(); }
    std::uint64_t kernel_calls() const noexcept { return kernel_calls_; }
    std::uint64_t committed_outputs() const noexcept { return binding_.output_count(); }
    const BackendRflyAck *last_backend_ack() const noexcept { return ack_observed_?&last_ack_:nullptr; }

private:
    bool fail(Error e) noexcept { if (error_==Error::None) error_=e;pending_=false;return false; }
    bool mapped_input_matches(const Input &in) noexcept {
        const auto &s=in.state;const auto &k=in.kernel;
        double n2=0;for (double v:s.q) n2+=v*v;const double n=std::sqrt(n2);
        if (!finite(s.q) || !std::isfinite(n) || std::abs(n-1.0)>1e-6) return fail(Error::Quaternion);
        // Coordinate mapping from px4EstimateState.m. Inputs are in task NED;
        // the mapping uses no additional origin offset.
        const gpenmpc_portable::Array<double,13> expected{s.p[0],s.p[1],-s.p[2],s.v[0],s.v[1],-s.v[2],
            s.q[0]/n,-s.q[1]/n,-s.q[2]/n,s.q[3]/n,-s.body_rates[0],-s.body_rates[1],s.body_rates[2]};
        for (unsigned j=0;j<13;++j) {
            // MATLAB norm and C sum/sqrt may differ by a few binary64 ulps.
            // Only mapped quaternion values use this representation allowance;
            // no controller state is changed and all other axes stay exact.
            if (j>=6 && j<10) {
                if (!std::isfinite(k.x[j]) || std::abs(k.x[j]-expected[j])>1e-12) return fail(Error::StateMapping);
            } else if (!gpenmpc_portable::exact_equal(k.x[j],expected[j])) return fail(Error::StateMapping);
        }
        for (unsigned j=13;j<19;++j) if (!gpenmpc_portable::exact_equal(k.x[j],0.0)) return fail(Error::HiddenRotorState);
        for (unsigned j=0;j<3;++j) {
            const double sign=j==2?-1.0:1.0;
            if (!gpenmpc_portable::exact_equal(k.refP[j],sign*in.reference.p[j]) ||
                !gpenmpc_portable::exact_equal(k.refV[j],sign*in.reference.v[j]) || !gpenmpc_portable::exact_equal(k.refA[j],sign*in.reference.a[j]))
                return fail(Error::ReferenceMapping);
        }
        return true;
    }
    static bool same_control_bits(const gpenmpc_portable::Array<float,16> &a,const gpenmpc_portable::Array<float,16> &b) noexcept {
        static_assert(sizeof(float)==4 && gpenmpc_portable::Ieee754<float>::is_iec559,"requires IEEE float32");
        for (unsigned j=0;j<16;++j) {
            std::uint32_t x=0,y=0;std::memcpy(&x,&a[j],4);std::memcpy(&y,&b[j],4);if(x!=y)return false;
        }
        return true;
    }
    Configuration config_{};Clock clock_{};ConsumptionBinding binding_;
    SimulinkCanonicalPolicy policy_{}; // explicit algebraic codegen+encoding policy
    Error error_{Error::None};bool pending_{false},ack_observed_{false};std::uint64_t kernel_calls_{0};
    Prepared prepared_{};BackendRflyAck last_ack_{};
};
} // namespace gpenmpc_rfly_execution
