#pragma once

// All times are ORIGINAL board monotonic microseconds within one verified boot.
#include "../portable/CanonicalPortable.hpp"
#include <cmath>
#include <cstdint>
#include "CanonicalSha256.hpp"

namespace gpenmpc_consumption {

// CurrentPhysicalCausalRuntime accepts actual 0 < dt <= 0.0100001 s.
// PX4 sample timestamps have integer-us resolution, so the largest legal
// source delta is 10000 us. Do NOT turn this upper bound into an exact cadence.
constexpr std::uint64_t canonical_dt_us = 10000;
enum class PublicationPath : std::uint8_t { Unbound, LegacyThrustTorque, DirectCanonicalMotors };

struct Identity {
    std::uint64_t uid{0};                 // never convert through double
    std::uint64_t boot_generation{0};     // supplied by verified boot identity
    std::uint8_t system{0}, component{0};
};
inline bool operator==(const Identity &a, const Identity &b) noexcept {
    return a.uid == b.uid && a.boot_generation == b.boot_generation &&
           a.system == b.system && a.component == b.component;
}

// Explicit caller-supplied freshness/work bounds.
struct Limits {
    std::uint64_t sample_max_age_us{0};
    std::uint64_t reference_max_age_us{0};
    std::uint64_t outer_max_age_us{0};
    std::uint64_t transaction_max_wall_us{0};
    PublicationPath publication_path{PublicationPath::Unbound};
};

struct Estimator {
    Identity identity{};
    std::uint64_t generation{0};          // real new atomic uORB publication
    std::uint64_t timestamp_sample_us{0}; // vehicle_odometry.timestamp_sample
    std::uint64_t publication_us{0};      // vehicle_odometry.timestamp
    std::uint64_t board_rx_us{0};         // ORIGINAL consumer receipt time
    std::uint8_t pose_frame{0}, velocity_frame{0}, reset_counter{0};
    gpenmpc_portable::Array<double, 3> p{}, v{}, body_rates{};
    gpenmpc_portable::Array<double, 4> q{};            // FRD -> NED, w,x,y,z; no normalization
};

struct Reference {
    Identity identity{};
    std::uint64_t generation{0};
    std::uint64_t outer_generation{0};
    std::uint64_t timestamp_us{0};        // original board reference creation
    std::uint64_t board_rx_us{0};         // original accepted receive event
    std::uint64_t valid_until_us{0};
    gpenmpc_portable::Array<double, 3> p{}, v{}, a{};   // NED; no hidden TASK_ORIGIN transform
    double yaw{0}, yaw_rate{0};
};

struct OuterCommand {
    Identity identity{};
    std::uint64_t generation{0};
    std::uint64_t based_on_sample_generation{0};
    std::uint64_t based_on_timestamp_sample_us{0};
    std::uint64_t board_rx_us{0};
    std::uint64_t valid_until_us{0};
    // Opaque application payload identity; producer/receiver must compute it
    // over the actual immutable outer payload, not a label or arrival order.
    gpenmpc_portable::Array<std::uint32_t, 8> payload_sha256{};
};

// Explicit numerical arguments, mirroring the generated subkernel signature.
// No struct padding is
// hashed.
struct KernelParameters {
    gpenmpc_portable::Array<double, 3> kp{}, kd{}, kr{}, kw{}, drag{};
    gpenmpc_portable::Array<double, 9> inertia{};
    gpenmpc_portable::Array<double, 24> pseudoinverse{};
    double baseMass{0}, totalThrust{0}, rotorUpper{0}, maxTilt{0};
};
struct KernelArguments {
    gpenmpc_portable::Array<double, 19> x{};
    gpenmpc_portable::Array<double, 3> refP{}, refV{}, refA{};
    double payload{0};
    gpenmpc_portable::Array<double, 2> windXY{};
    gpenmpc_portable::Array<double, 3> augmentation{};
    bool continuityEnabled{false};
    gpenmpc_portable::Array<double, 9> commandR{};
    gpenmpc_portable::Array<double, 3> commandOmega{}, commandOmegaDot{};
    KernelParameters parameters{};
    std::uint64_t augmentation_state_generation{0}, continuity_state_generation{0};
};

struct Input {
    Estimator state{};
    Reference reference{};
    OuterCommand outer{};
    std::uint64_t control_tick_us{0};     // actual begin HRT, NOT sample time
    std::uint64_t expected_reference_generation{0};
    std::uint64_t expected_outer_generation{0};
    bool armed{false}, offboard{false}, controller_selected{false};
    bool native_conflicting_publishers_disabled{false};
    KernelArguments kernel{};
    gpenmpc_portable::Array<std::uint32_t, 8> kernel_source_sha256{};
};

enum class Fault : std::uint8_t {
    None, BadConfiguration, IdentityMismatch, AuthorityNotAdmitted,
    InvalidState, InvalidReference, InvalidOuter, FutureOrInconsistentTime,
    StateStale, ReferenceStale, OuterStale, DuplicateState, StateRegression,
    GenerationGap, SampleTimeGap, ResetChanged, TickRegression,
    ReferenceMismatch, ReferenceRegression, ReferenceMutation,
    OuterRegression, OuterMutation, OuterLineage, PendingTransaction,
    TokenMismatch, KernelInvalid, Exception, PublicationFailed,
    PublicationMismatch, TransactionDeadline, CounterOverflow, KernelInputInvalid,
    KernelArgumentMismatch, PublicationPathMismatch
};

struct Token {
    Identity identity{};
    PublicationPath publication_path{PublicationPath::Unbound};
    std::uint64_t transaction{0}, output_generation{0};
    std::uint64_t sample_generation{0}, timestamp_sample_us{0};
    std::uint64_t source_generation_delta{0};
    std::uint64_t state_publication_us{0}, state_board_rx_us{0};
    std::uint64_t control_tick_us{0}, sample_delta_us{0}, actual_tick_delta_us{0};
    std::uint64_t reference_generation{0}, reference_timestamp_us{0};
    std::uint64_t reference_board_rx_us{0}, reference_valid_until_us{0};
    std::uint64_t outer_generation{0}, outer_board_rx_us{0}, outer_valid_until_us{0};
    std::uint64_t outer_based_on_sample_generation{0}, outer_based_on_timestamp_sample_us{0};
    gpenmpc_portable::Array<std::uint32_t, 8> outer_payload_sha256{};
    gpenmpc_portable::Array<std::uint32_t, 8> full_input_sha256{}, kernel_argument_sha256{}, kernel_source_sha256{};
};
inline bool operator==(const Token &a, const Token &b) noexcept {
    return a.identity == b.identity && a.publication_path == b.publication_path && a.transaction == b.transaction &&
        a.output_generation == b.output_generation && a.sample_generation == b.sample_generation &&
        a.source_generation_delta == b.source_generation_delta &&
        a.timestamp_sample_us == b.timestamp_sample_us && a.state_publication_us == b.state_publication_us &&
        a.state_board_rx_us == b.state_board_rx_us && a.control_tick_us == b.control_tick_us &&
        a.sample_delta_us == b.sample_delta_us && a.actual_tick_delta_us == b.actual_tick_delta_us &&
        a.reference_generation == b.reference_generation && a.reference_timestamp_us == b.reference_timestamp_us &&
        a.reference_board_rx_us == b.reference_board_rx_us && a.reference_valid_until_us == b.reference_valid_until_us &&
        a.outer_generation == b.outer_generation && a.outer_board_rx_us == b.outer_board_rx_us &&
        a.outer_valid_until_us == b.outer_valid_until_us &&
        a.outer_based_on_sample_generation == b.outer_based_on_sample_generation &&
        a.outer_based_on_timestamp_sample_us == b.outer_based_on_timestamp_sample_us &&
        a.outer_payload_sha256 == b.outer_payload_sha256 && a.full_input_sha256 == b.full_input_sha256 &&
        a.kernel_argument_sha256 == b.kernel_argument_sha256 && a.kernel_source_sha256 == b.kernel_source_sha256;
}

struct KernelOutput {
    gpenmpc_portable::Array<double, 4> wrench{};        // actual generated-kernel output
    gpenmpc_portable::Array<double, 6> rotor{};
};
inline gpenmpc_portable::Array<std::uint32_t, 8> kernel_output_sha256(const KernelOutput &o) noexcept {
    CanonicalSha256 h; h.u32(0x52414f31); h.reals(o.wrench); h.reals(o.rotor); return h.finish();
}

struct PublicationAck {
    Token token{};
    // A future BOARD publisher supplies these after successful actual calls.
    // Neither HIL_ACTUATOR_CONTROLS timestamp nor a host send is such an ACK.
    bool thrust_published{false}, torque_published{false};
    std::uint64_t thrust_generation{0}, torque_generation{0};
    std::uint64_t thrust_publication_us{0}, torque_publication_us{0};
    // Hash of the source envelope accepted by the publisher. Normalized uORB
    // values and SI wrench/rotor values use separate conversion validation.
    gpenmpc_portable::Array<std::uint32_t, 8> consumed_output_sha256{};
};

struct MotorPublicationAck {
    Token token{};
    bool motors_published{false};
    std::uint64_t generation{0}, publication_us{0};
    // Actual canonical pinv+clip six values, preserved through the explicitly
    // selected direct interface. No PX4 reallocation may intervene on this path.
    gpenmpc_portable::Array<double, 6> canonical_rotor_thrust_n{};
    gpenmpc_portable::Array<std::uint32_t, 8> consumed_output_sha256{};
};

struct Receipt {
    bool valid{false};
    Token consumed{};
    std::uint64_t kernel_completed_us{0};
    std::uint64_t thrust_publication_us{0}, torque_publication_us{0};
    std::uint64_t direct_motor_publication_us{0};
    KernelOutput actual_output{};
};

template<std::size_t N> inline bool finite(const gpenmpc_portable::Array<double, N> &x) noexcept {
    for (double v : x) { if (!std::isfinite(v)) return false; }
    return true;
}

inline void hash_identity(CanonicalSha256 &h, const Identity &i) noexcept {
    h.u64(i.uid); h.u64(i.boot_generation); h.byte(i.system); h.byte(i.component);
}
inline void hash_kernel_fields(CanonicalSha256 &h, const KernelArguments &k) noexcept {
    h.reals(k.x); h.reals(k.refP); h.reals(k.refV); h.reals(k.refA); h.real(k.payload);
    h.reals(k.windXY); h.reals(k.augmentation); h.byte(k.continuityEnabled);
    h.reals(k.commandR); h.reals(k.commandOmega); h.reals(k.commandOmegaDot);
    const auto &p = k.parameters;
    h.reals(p.kp); h.reals(p.kd); h.reals(p.kr); h.reals(p.kw); h.reals(p.drag);
    h.reals(p.inertia); h.reals(p.pseudoinverse); h.real(p.baseMass); h.real(p.totalThrust); h.real(p.rotorUpper); h.real(p.maxTilt);
    h.u64(k.augmentation_state_generation); h.u64(k.continuity_state_generation);
}
inline gpenmpc_portable::Array<std::uint32_t, 8> kernel_arguments_sha256(const KernelArguments &k) noexcept {
    CanonicalSha256 h; h.u32(0x52414b31); hash_kernel_fields(h, k); return h.finish();
}
inline gpenmpc_portable::Array<std::uint32_t, 8> full_input_sha256(const Input &in) noexcept {
    CanonicalSha256 h; h.u32(0x52414331); // domain/schema RAC1
    const auto &s = in.state; const auto &r = in.reference; const auto &o = in.outer;
    hash_identity(h, s.identity); h.u64(s.generation); h.u64(s.timestamp_sample_us); h.u64(s.publication_us); h.u64(s.board_rx_us);
    h.byte(s.pose_frame); h.byte(s.velocity_frame); h.byte(s.reset_counter); h.reals(s.p); h.reals(s.v); h.reals(s.body_rates); h.reals(s.q);
    hash_identity(h, r.identity); h.u64(r.generation); h.u64(r.outer_generation); h.u64(r.timestamp_us); h.u64(r.board_rx_us);
    h.u64(r.valid_until_us); h.reals(r.p); h.reals(r.v); h.reals(r.a); h.real(r.yaw); h.real(r.yaw_rate);
    hash_identity(h, o.identity); h.u64(o.generation); h.u64(o.based_on_sample_generation); h.u64(o.based_on_timestamp_sample_us);
    h.u64(o.board_rx_us); h.u64(o.valid_until_us); h.words(o.payload_sha256);
    h.u64(in.control_tick_us); h.u64(in.expected_reference_generation); h.u64(in.expected_outer_generation);
    h.byte(in.armed); h.byte(in.offboard); h.byte(in.controller_selected); h.byte(in.native_conflicting_publishers_disabled);
    hash_kernel_fields(h, in.kernel); h.words(in.kernel_source_sha256); return h.finish();
}
inline bool finite_kernel(const KernelArguments &k) noexcept {
    const auto &p = k.parameters;
    return finite(k.x) && finite(k.refP) && finite(k.refV) && finite(k.refA) && std::isfinite(k.payload) &&
        finite(k.windXY) && finite(k.augmentation) && finite(k.commandR) && finite(k.commandOmega) && finite(k.commandOmegaDot) &&
        finite(p.kp) && finite(p.kd) && finite(p.kr) && finite(p.kw) && finite(p.drag) && finite(p.inertia) && finite(p.pseudoinverse) &&
        std::isfinite(p.baseMass) && std::isfinite(p.totalThrust) && std::isfinite(p.rotorUpper) && std::isfinite(p.maxTilt);
}

class ConsumptionBinding final {
public:
    ConsumptionBinding(Identity identity, Limits limits) noexcept : identity_(identity), limits_(limits) {
        if (!identity.uid || !identity.boot_generation || !identity.system || !identity.component ||
            !limits.sample_max_age_us || !limits.reference_max_age_us || !limits.outer_max_age_us ||
            !limits.transaction_max_wall_us || (limits.publication_path != PublicationPath::LegacyThrustTorque &&
            limits.publication_path != PublicationPath::DirectCanonicalMotors)) { latch(Fault::BadConfiguration); }
    }

    bool begin(const Input &in, Token &token) noexcept {
        token = {};
        if (fault_ != Fault::None) return false;
        if (pending_) return latch(Fault::PendingTransaction);
        const auto &s = in.state; const auto &r = in.reference; const auto &o = in.outer;
        const auto now = in.control_tick_us;
        if (!(s.identity == identity_) || !(r.identity == identity_) || !(o.identity == identity_))
            return latch(Fault::IdentityMismatch);
        if (!in.armed || !in.offboard || !in.controller_selected || !in.native_conflicting_publishers_disabled)
            return latch(Fault::AuthorityNotAdmitted);
        double norm2 = 0; for (double v : s.q) norm2 += v * v;
        // The 1e-3 norm tolerance is representation validation, not a controller
        // performance gate; invalid quaternions are never silently normalized.
        if (!s.generation || s.pose_frame != 1 || s.velocity_frame != 1 ||
            !finite(s.p) || !finite(s.v) || !finite(s.q) || !finite(s.body_rates) || std::abs(norm2 - 1.0) > 1e-3)
            return latch(Fault::InvalidState);
        if (!r.generation || !r.outer_generation || !finite(r.p) || !finite(r.v) || !finite(r.a) ||
            !std::isfinite(r.yaw) || !std::isfinite(r.yaw_rate)) return latch(Fault::InvalidReference);
        bool payload_hash_nonzero = false; for (auto word : o.payload_sha256) payload_hash_nonzero |= word != 0;
        if (!o.generation || !o.based_on_sample_generation || !o.based_on_timestamp_sample_us || !payload_hash_nonzero)
            return latch(Fault::InvalidOuter);
        bool source_hash_nonzero = false; for (auto word : in.kernel_source_sha256) source_hash_nonzero |= word != 0;
        if (!source_hash_nonzero || !finite_kernel(in.kernel)) return latch(Fault::KernelInputInvalid);
        if (!now || !s.timestamp_sample_us || !s.publication_us || !s.board_rx_us || !r.timestamp_us ||
            !r.board_rx_us || !o.board_rx_us || s.timestamp_sample_us > s.publication_us ||
            s.publication_us > s.board_rx_us || s.board_rx_us > now || r.timestamp_us > r.board_rx_us ||
            r.board_rx_us > now || o.board_rx_us > r.board_rx_us || o.based_on_timestamp_sample_us > o.board_rx_us ||
            r.valid_until_us < r.board_rx_us || o.valid_until_us < o.board_rx_us)
            return latch(Fault::FutureOrInconsistentTime);
        if (now - s.timestamp_sample_us > limits_.sample_max_age_us) return latch(Fault::StateStale);
        if (now - r.board_rx_us > limits_.reference_max_age_us || now > r.valid_until_us) return latch(Fault::ReferenceStale);
        if (now - o.board_rx_us > limits_.outer_max_age_us || now > o.valid_until_us) return latch(Fault::OuterStale);
        if (r.generation != in.expected_reference_generation || o.generation != in.expected_outer_generation ||
            r.outer_generation != o.generation) return latch(Fault::ReferenceMismatch);
        if (o.based_on_sample_generation > s.generation || o.based_on_timestamp_sample_us > s.timestamp_sample_us)
            return latch(Fault::OuterLineage);
        if (o.based_on_sample_generation == s.generation && o.based_on_timestamp_sample_us != s.timestamp_sample_us)
            return latch(Fault::OuterLineage);
        if (last_.state.generation) {
            if (s.generation == last_.state.generation || s.timestamp_sample_us == last_.state.timestamp_sample_us)
                return latch(Fault::DuplicateState);
            if (s.generation < last_.state.generation || s.timestamp_sample_us < last_.state.timestamp_sample_us)
                return latch(Fault::StateRegression);
            // No clamping, interpolation, held sample, or timestamp relabeling.
            if (s.timestamp_sample_us - last_.state.timestamp_sample_us > canonical_dt_us)
                return latch(Fault::SampleTimeGap);
            if (s.reset_counter != last_.state.reset_counter) return latch(Fault::ResetChanged);
            if (now <= last_.control_tick_us) return latch(Fault::TickRegression);
            if (r.generation < last_.reference.generation) return latch(Fault::ReferenceRegression);
            if (r.generation == last_.reference.generation && !same_reference(r, last_.reference))
                return latch(Fault::ReferenceMutation);
            if (o.generation < last_.outer.generation) return latch(Fault::OuterRegression);
            if (o.generation == last_.outer.generation && !same_outer(o, last_.outer)) return latch(Fault::OuterMutation);
            if (r.generation > last_.reference.generation &&
                (r.board_rx_us <= last_.reference.board_rx_us || r.timestamp_us <= last_.reference.timestamp_us))
                return latch(Fault::ReferenceMutation);
            if (o.generation > last_.outer.generation && o.board_rx_us <= last_.outer.board_rx_us)
                return latch(Fault::OuterMutation);
        }
        if (transactions_ == UINT64_MAX || outputs_ == UINT64_MAX)
            return latch(Fault::CounterOverflow);
        Token t{}; t.identity = identity_; t.publication_path = limits_.publication_path;
        t.transaction = ++transactions_; t.output_generation = outputs_ + 1;
        t.sample_generation = s.generation; t.timestamp_sample_us = s.timestamp_sample_us;
        t.source_generation_delta = last_.state.generation ? s.generation - last_.state.generation : 0;
        t.state_publication_us = s.publication_us; t.state_board_rx_us = s.board_rx_us; t.control_tick_us = now;
        t.sample_delta_us = last_.state.generation ? s.timestamp_sample_us - last_.state.timestamp_sample_us : 0;
        t.actual_tick_delta_us = last_.state.generation ? now - last_.control_tick_us : 0;
        t.reference_generation = r.generation; t.reference_timestamp_us = r.timestamp_us;
        t.reference_board_rx_us = r.board_rx_us; t.reference_valid_until_us = r.valid_until_us;
        t.outer_generation = o.generation; t.outer_board_rx_us = o.board_rx_us; t.outer_valid_until_us = o.valid_until_us;
        t.outer_based_on_sample_generation = o.based_on_sample_generation;
        t.outer_based_on_timestamp_sample_us = o.based_on_timestamp_sample_us;
        t.outer_payload_sha256 = o.payload_sha256;
        t.full_input_sha256 = full_input_sha256(in); t.kernel_argument_sha256 = kernel_arguments_sha256(in.kernel);
        t.kernel_source_sha256 = in.kernel_source_sha256;
        pending_input_ = in; pending_token_ = t; pending_ = true; kernel_done_ = false; token = t;
        return true;
    }

    // The integration invokes the actual kernel on this EXACT copied input.
    // nullptr unless a transaction is admitted; no mutable input accessor.
    const Input *pending_input() const noexcept { return pending_ && fault_ == Fault::None ? &pending_input_ : nullptr; }

    bool kernel_completed(const Token &token, std::uint64_t now, bool valid, const KernelOutput &out,
                          const KernelArguments &executed_arguments) noexcept {
        if (fault_ != Fault::None) return false;
        if (!pending_ || kernel_done_ || !(token == pending_token_)) return latch(Fault::TokenMismatch);
        if (!time_in_transaction(now)) return false;
        if (kernel_arguments_sha256(executed_arguments) != pending_token_.kernel_argument_sha256)
            return latch(Fault::KernelArgumentMismatch);
        if (!valid || !finite(out.wrench) || !finite(out.rotor)) return latch(Fault::KernelInvalid);
        pending_output_ = out; kernel_completed_us_ = now; kernel_done_ = true; return true;
    }

    bool output_committed(const PublicationAck &ack, std::uint64_t now, Receipt &receipt) noexcept {
        receipt = {};
        if (fault_ != Fault::None) return false;
        if (limits_.publication_path != PublicationPath::LegacyThrustTorque) return latch(Fault::PublicationPathMismatch);
        last_publication_attempt_ = ack; publication_attempt_observed_ = true;
        if (!pending_ || !kernel_done_ || !(ack.token == pending_token_)) return latch(Fault::TokenMismatch);
        if (!time_in_transaction(now)) return false;
        if (!ack.thrust_published || !ack.torque_published) return latch(Fault::PublicationFailed);
        if (ack.thrust_generation != pending_token_.output_generation || ack.torque_generation != pending_token_.output_generation ||
            ack.consumed_output_sha256 != kernel_output_sha256(pending_output_) ||
            ack.thrust_publication_us < kernel_completed_us_ || ack.torque_publication_us < kernel_completed_us_ ||
            ack.thrust_publication_us > now || ack.torque_publication_us > now) return latch(Fault::PublicationMismatch);
        ++outputs_; last_ = pending_input_;
        receipt.valid = true; receipt.consumed = pending_token_; receipt.kernel_completed_us = kernel_completed_us_;
        receipt.thrust_publication_us = ack.thrust_publication_us; receipt.torque_publication_us = ack.torque_publication_us;
        receipt.actual_output = pending_output_; pending_ = false; kernel_done_ = false; return true;
    }

    bool motors_committed(const MotorPublicationAck &ack, std::uint64_t now, Receipt &receipt) noexcept {
        receipt = {};
        if (fault_ != Fault::None) return false;
        if (limits_.publication_path != PublicationPath::DirectCanonicalMotors) return latch(Fault::PublicationPathMismatch);
        last_motor_attempt_ = ack; motor_attempt_observed_ = true;
        if (!pending_ || !kernel_done_ || !(ack.token == pending_token_)) return latch(Fault::TokenMismatch);
        if (!time_in_transaction(now)) return false;
        if (!ack.motors_published) return latch(Fault::PublicationFailed);
        if (ack.generation != pending_token_.output_generation || !finite(ack.canonical_rotor_thrust_n) ||
            ack.canonical_rotor_thrust_n != pending_output_.rotor || ack.consumed_output_sha256 != kernel_output_sha256(pending_output_) ||
            ack.publication_us < kernel_completed_us_ || ack.publication_us > now) return latch(Fault::PublicationMismatch);
        ++outputs_; last_ = pending_input_;
        receipt.valid = true; receipt.consumed = pending_token_; receipt.kernel_completed_us = kernel_completed_us_;
        receipt.direct_motor_publication_us = ack.publication_us; receipt.actual_output = pending_output_;
        pending_ = false; kernel_done_ = false; return true;
    }

    void note_exception() noexcept { latch(Fault::Exception); }
    Fault fault() const noexcept { return fault_; }
    std::uint64_t output_count() const noexcept { return outputs_; }
    std::uint64_t admitted_transaction_count() const noexcept { return transactions_; }
    bool failed() const noexcept { return fault_ != Fault::None; }
    // Partial publisher success is not erased by fail-closed rejection. The
    // supervising integration must retain this and perform its safe recovery.
    const PublicationAck *last_publication_attempt() const noexcept {
        return publication_attempt_observed_ ? &last_publication_attempt_ : nullptr;
    }
    const MotorPublicationAck *last_motor_attempt() const noexcept { return motor_attempt_observed_ ? &last_motor_attempt_ : nullptr; }

private:
    bool latch(Fault f) noexcept { if (fault_ == Fault::None) fault_ = f; pending_ = false; kernel_done_ = false; return false; }
    bool time_in_transaction(std::uint64_t now) noexcept {
        if (now < pending_token_.control_tick_us) return latch(Fault::FutureOrInconsistentTime);
        if (now - pending_token_.control_tick_us > limits_.transaction_max_wall_us) return latch(Fault::TransactionDeadline);
        if (now - pending_token_.timestamp_sample_us > limits_.sample_max_age_us) return latch(Fault::StateStale);
        if (now - pending_token_.reference_board_rx_us > limits_.reference_max_age_us || now > pending_token_.reference_valid_until_us)
            return latch(Fault::ReferenceStale);
        if (now - pending_token_.outer_board_rx_us > limits_.outer_max_age_us || now > pending_token_.outer_valid_until_us)
            return latch(Fault::OuterStale);
        return true;
    }
    static bool same_reference(const Reference &a, const Reference &b) noexcept {
        return a.outer_generation == b.outer_generation && a.timestamp_us == b.timestamp_us && a.board_rx_us == b.board_rx_us &&
            a.valid_until_us == b.valid_until_us && a.p == b.p && a.v == b.v && a.a == b.a &&
            gpenmpc_portable::exact_equal(a.yaw,b.yaw) && gpenmpc_portable::exact_equal(a.yaw_rate,b.yaw_rate);
    }
    static bool same_outer(const OuterCommand &a, const OuterCommand &b) noexcept {
        return a.based_on_sample_generation == b.based_on_sample_generation && a.based_on_timestamp_sample_us == b.based_on_timestamp_sample_us &&
            a.board_rx_us == b.board_rx_us && a.valid_until_us == b.valid_until_us && a.payload_sha256 == b.payload_sha256;
    }
    Identity identity_{}; Limits limits_{}; Fault fault_{Fault::None};
    std::uint64_t transactions_{0}, outputs_{0}, kernel_completed_us_{0};
    bool pending_{false}, kernel_done_{false};
    Input last_{}, pending_input_{}; Token pending_token_{}; KernelOutput pending_output_{};
    PublicationAck last_publication_attempt_{}; bool publication_attempt_observed_{false};
    MotorPublicationAck last_motor_attempt_{}; bool motor_attempt_observed_{false};
};

} // namespace gpenmpc_consumption
