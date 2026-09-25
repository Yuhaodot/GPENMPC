#pragma once

// Host-only output-selection prototype.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace gpenmpc::rfly_host {
constexpr std::size_t kChannels = 16;
enum class Source { None, NativeOutputsSim, RflyOutputs };
enum class Fault {
    None, InvalidConfiguration, UnsafeEnvironment, ClockRegression,
    MissingOutput, IdentityMismatch, WrongOutputCount, ZeroTimestamp,
    FutureTimestamp, StaleTimestamp, ReplayTimestamp, PreSelectionTimestamp,
    NonfiniteControl, OutOfRangeControl
};
enum class TransitionRejection { None, InvalidTarget, UnsafeTransition,
                                 FaultRequiresRecovery, ClockRegression };

struct Config {
    // All supplied explicitly by caller. No live age limit is chosen here.
    std::uint64_t max_age_us;
    std::uint64_t session_id;
    std::uint64_t native_producer_id;
    std::uint64_t rfly_producer_id;
    std::size_t required_outputs;
};
struct SafetyContext {
    // Unknown evidence must remain false, not be inferred from control values.
    bool explicitly_disarmed = false;
    bool hil_enabled = false;
    bool physical_outputs_disabled = false;
};
struct Sample {
    Source source = Source::None;
    std::uint64_t session_id = 0;
    std::uint64_t producer_id = 0;
    std::uint64_t timestamp_us = 0;
    std::size_t noutputs = 0;
    std::array<float, kChannels> controls{};
};
struct Output {
    Source selected_source = Source::NativeOutputsSim;
    Source emitted_source = Source::None;
    Fault fault = Fault::None;
    std::size_t noutputs = 0;
    std::array<float, kChannels> controls{};
    std::uint8_t mode_flags = 0; // Never emits an arm, Loiter or other command.
    bool valid = false;
};
struct TransitionResult {
    bool accepted;
    TransitionRejection rejection;
};

class HilOutputSelector {
public:
    explicit HilOutputSelector(Config config) : config_(config) {
        if (config_.max_age_us == 0 || config_.session_id == 0 ||
            config_.native_producer_id == 0 || config_.rfly_producer_id == 0 ||
            config_.native_producer_id == config_.rfly_producer_id ||
            config_.required_outputs == 0 || config_.required_outputs > kChannels) {
            fault_ = Fault::InvalidConfiguration;
        }
    }

    Source selected_source() const { return selected_; }
    Fault fault() const { return fault_; }

    // Same-source requests are also guarded, and never reset replay history.
    TransitionResult request_mode(Source target, const SafetyContext &safety,
                                  std::uint64_t now_us) {
        return transition(target, safety, now_us, false);
    }

    // Recovery is explicit. Retain replay watermarks; source restart requires
    // a new instance and a caller-established session.
    TransitionResult recover_to(Source target, const SafetyContext &safety,
                                std::uint64_t now_us) {
        return transition(target, safety, now_us, true);
    }

    Output evaluate(std::uint64_t now_us, const SafetyContext &safety,
                    const Sample *native, const Sample *rfly) {
        if (fault_ != Fault::None) { return zero_output(); }
        if (!safety.hil_enabled || !safety.physical_outputs_disabled) {
            return fail(Fault::UnsafeEnvironment);
        }
        if (have_clock_ && now_us < last_now_us_) { return fail(Fault::ClockRegression); }
        last_now_us_ = now_us;
        have_clock_ = true;

        // The unselected producer is deliberately never read or combined.
        const Sample *sample = selected_ == Source::RflyOutputs ? rfly : native;
        if (sample == nullptr) { return fail(Fault::MissingOutput); }
        const auto expected_producer = selected_ == Source::RflyOutputs
                                       ? config_.rfly_producer_id : config_.native_producer_id;
        if (sample->source != selected_ || sample->session_id != config_.session_id ||
            sample->producer_id != expected_producer) { return fail(Fault::IdentityMismatch); }
        if (sample->noutputs != config_.required_outputs) { return fail(Fault::WrongOutputCount); }
        if (sample->timestamp_us == 0) { return fail(Fault::ZeroTimestamp); }
        if (sample->timestamp_us > now_us) { return fail(Fault::FutureTimestamp); }
        if (now_us - sample->timestamp_us > config_.max_age_us) { return fail(Fault::StaleTimestamp); }
        const std::size_t source_index = selected_ == Source::RflyOutputs ? 1 : 0;
        if (sample->timestamp_us <= last_sample_us_[source_index]) { return fail(Fault::ReplayTimestamp); }
        if (sample->timestamp_us < selection_barrier_us_) { return fail(Fault::PreSelectionTimestamp); }
        for (std::size_t i = 0; i < sample->noutputs; ++i) {
            const float value = sample->controls[i];
            if (!std::isfinite(value)) { return fail(Fault::NonfiniteControl); }
            if (value < -1.0F || value > 1.0F) { return fail(Fault::OutOfRangeControl); }
        }

        last_sample_us_[source_index] = sample->timestamp_us;
        Output result;
        result.selected_source = selected_;
        result.emitted_source = selected_;
        result.noutputs = config_.required_outputs;
        // Values beyond the declared output count remain zero.
        for (std::size_t i = 0; i < sample->noutputs; ++i) { result.controls[i] = sample->controls[i]; }
        result.valid = true;
        return result;
    }

private:
    TransitionResult transition(Source target, const SafetyContext &safety,
                                std::uint64_t now_us, bool recovery) {
        if (target != Source::NativeOutputsSim && target != Source::RflyOutputs) {
            return {false, TransitionRejection::InvalidTarget};
        }
        if (!safety.explicitly_disarmed || !safety.hil_enabled || !safety.physical_outputs_disabled) {
            return {false, TransitionRejection::UnsafeTransition};
        }
        if (fault_ == Fault::InvalidConfiguration || (fault_ != Fault::None && !recovery)) {
            return {false, TransitionRejection::FaultRequiresRecovery};
        }
        if (have_clock_ && now_us < last_now_us_) {
            return {false, TransitionRejection::ClockRegression};
        }
        if (target != selected_ || (recovery && fault_ != Fault::None)) {
            selection_barrier_us_ = now_us;
        }
        selected_ = target;
        if (recovery) { fault_ = Fault::None; }
        have_clock_ = true;
        last_now_us_ = now_us;
        return {true, TransitionRejection::None};
    }

    Output zero_output() const {
        Output result;
        result.selected_source = selected_;
        result.noutputs = config_.required_outputs <= kChannels ? config_.required_outputs : 0;
        result.fault = fault_;
        return result;
    }
    Output fail(Fault reason) { fault_ = reason; return zero_output(); }

    Config config_;
    Source selected_ = Source::NativeOutputsSim;
    Fault fault_ = Fault::None;
    std::uint64_t selection_barrier_us_ = 0;
    std::array<std::uint64_t, 2> last_sample_us_{};
    std::uint64_t last_now_us_ = 0;
    bool have_clock_ = false;
};
} // namespace gpenmpc::rfly_host
