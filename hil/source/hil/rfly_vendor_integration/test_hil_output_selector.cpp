#include "hil_output_selector.hpp"
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace gpenmpc::rfly_host;
namespace {
// Synthetic test fixture only: 100 us is NOT a proposed live timing gate.
constexpr Config config{100, 42, 11, 22, 16};
constexpr SafetyContext safe{true, true, true};
constexpr SafetyContext active{false, true, true};
struct Case { std::string name; bool passed; };
std::vector<Case> cases;
void check(const std::string &name, bool passed) { cases.push_back({name, passed}); }
Sample frame(Source source, std::uint64_t stamp, float value) {
    Sample result;
    result.source = source;
    result.session_id = config.session_id;
    result.producer_id = source == Source::RflyOutputs ? config.rfly_producer_id : config.native_producer_id;
    result.timestamp_us = stamp;
    result.noutputs = config.required_outputs;
    result.controls.fill(value);
    return result;
}
bool zeros(const Output &out) {
    if (out.valid || out.emitted_source != Source::None || out.mode_flags != 0) { return false; }
    for (const auto value : out.controls) { if (value != 0.0F) { return false; } }
    return true;
}
bool only(const Output &out, Source source, float value) {
    if (!out.valid || out.fault != Fault::None || out.selected_source != source ||
        out.emitted_source != source || out.mode_flags != 0 || out.noutputs != 16) { return false; }
    for (const auto control : out.controls) { if (control != value) { return false; } }
    return true;
}
void fault_case(const std::string &name, Sample bad, Fault expected,
                std::uint64_t now = 1000) {
    HilOutputSelector selector(config);
    const bool selected = selector.request_mode(Source::RflyOutputs, safe, 900).accepted;
    auto native = frame(Source::NativeOutputsSim, now, 0.25F);
    const auto out = selector.evaluate(now, active, &native, &bad);
    check(name, selected && zeros(out) && out.fault == expected &&
          selector.selected_source() == Source::RflyOutputs);
    auto good = frame(Source::RflyOutputs, now + 1, -0.5F);
    const auto held = selector.evaluate(now + 1, active, &native, &good);
    check(name + "_latched_no_native_fallback", zeros(held) && held.fault == expected &&
          held.selected_source == Source::RflyOutputs);
}
}

int main(int argc, char **argv) {
    auto native = frame(Source::NativeOutputsSim, 1000, 0.25F);
    auto rfly = frame(Source::RflyOutputs, 1000, -0.5F);
    {
        HilOutputSelector selector(config);
        check("default_native_outputs_sim", selector.selected_source() == Source::NativeOutputsSim);
        check("native_does_not_mix_rfly", only(selector.evaluate(1000, safe, &native, &rfly),
                                               Source::NativeOutputsSim, 0.25F));
        check("explicit_disarmed_select_rfly", selector.request_mode(Source::RflyOutputs, safe, 1001).accepted);
        rfly.timestamp_us = 1001;
        check("rfly_does_not_mix_native", only(selector.evaluate(1001, active, &native, &rfly),
                                               Source::RflyOutputs, -0.5F));
        check("active_native_switch_rejected", !selector.request_mode(Source::NativeOutputsSim, active, 1002).accepted);
        rfly.timestamp_us = 1002;
        check("rejected_switch_retains_rfly", only(selector.evaluate(1002, active, &native, &rfly),
                                                   Source::RflyOutputs, -0.5F));
        check("disarmed_restore_outputs_sim", selector.request_mode(Source::NativeOutputsSim, safe, 1003).accepted);
        native.timestamp_us = 1003;
        check("restored_native_only", only(selector.evaluate(1003, safe, &native, &rfly),
                                           Source::NativeOutputsSim, 0.25F));
    }
    {
        bool exclusive = true;
        HilOutputSelector selector(config);
        for (std::uint64_t i = 0; i < 64; ++i) {
            const auto target = i % 2 == 0 ? Source::NativeOutputsSim : Source::RflyOutputs;
            const auto now = 2000 + i;
            native = frame(Source::NativeOutputsSim, now, 0.25F);
            rfly = frame(Source::RflyOutputs, now, -0.5F);
            exclusive = exclusive && selector.request_mode(target, safe, now).accepted &&
                        only(selector.evaluate(now, active, &native, &rfly), target,
                             target == Source::NativeOutputsSim ? 0.25F : -0.5F);
        }
        check("64_transitions_no_mixed_publication", exclusive);
    }
    for (std::size_t i = 0; i < 3; ++i) {
        HilOutputSelector selector(config);
        auto context = safe;
        if (i == 0) { context.explicitly_disarmed = false; }
        if (i == 1) { context.hil_enabled = false; }
        if (i == 2) { context.physical_outputs_disabled = false; }
        check("transition_requires_explicit_guard_" + std::to_string(i),
              !selector.request_mode(Source::RflyOutputs, context, 1000).accepted &&
              selector.selected_source() == Source::NativeOutputsSim);
    }
    {
        HilOutputSelector selector(config);
        check("unknown_guards_reject", !selector.request_mode(Source::RflyOutputs, {}, 1000).accepted);
        check("invalid_mode_reject", !selector.request_mode(Source::None, safe, 1000).accepted);
    }
    auto bad = frame(Source::RflyOutputs, 899, -0.5F);
    fault_case("stale", bad, Fault::StaleTimestamp);
    bad.timestamp_us = 1001; fault_case("future", bad, Fault::FutureTimestamp);
    bad.timestamp_us = 0; fault_case("zero_stamp", bad, Fault::ZeroTimestamp);
    bad = frame(Source::RflyOutputs, 1000, -0.5F);
    bad.controls[7] = std::numeric_limits<float>::quiet_NaN();
    fault_case("nan", bad, Fault::NonfiniteControl);
    bad.controls[7] = std::numeric_limits<float>::infinity();
    fault_case("infinity", bad, Fault::NonfiniteControl);
    bad.controls[7] = 1.0001F; fault_case("above_range", bad, Fault::OutOfRangeControl);
    bad.controls[7] = -1.0001F; fault_case("below_range", bad, Fault::OutOfRangeControl);
    bad = frame(Source::RflyOutputs, 1000, -0.5F);
    bad.noutputs = 15; fault_case("missing_channel", bad, Fault::WrongOutputCount);
    bad.noutputs = 17; fault_case("excess_channels", bad, Fault::WrongOutputCount);
    bad = frame(Source::RflyOutputs, 1000, -0.5F);
    bad.session_id = 99; fault_case("session_identity", bad, Fault::IdentityMismatch);
    bad = frame(Source::RflyOutputs, 1000, -0.5F);
    bad.producer_id = 11; fault_case("producer_identity", bad, Fault::IdentityMismatch);
    bad = frame(Source::NativeOutputsSim, 1000, 0.25F);
    fault_case("source_identity", bad, Fault::IdentityMismatch);
    {
        HilOutputSelector selector(config);
        selector.request_mode(Source::RflyOutputs, safe, 900);
        native = frame(Source::NativeOutputsSim, 1000, 0.25F);
        const auto out = selector.evaluate(1000, active, &native, nullptr);
        check("missing_rfly_not_native_fallback", zeros(out) && out.fault == Fault::MissingOutput &&
              out.selected_source == Source::RflyOutputs);
    }
    {
        HilOutputSelector selector(config);
        rfly = frame(Source::RflyOutputs, 1000, -0.5F);
        selector.request_mode(Source::RflyOutputs, safe, 900);
        selector.evaluate(1000, active, nullptr, &rfly);
        const auto replay = selector.evaluate(1001, active, nullptr, &rfly);
        check("equal_stamp_replay_fault", zeros(replay) && replay.fault == Fault::ReplayTimestamp);
        check("ordinary_mode_request_cannot_clear_fault",
              !selector.request_mode(Source::NativeOutputsSim, safe, 1002).accepted);
        check("active_recovery_rejected", !selector.recover_to(Source::NativeOutputsSim, active, 1002).accepted);
        check("disarmed_explicit_recovery_accepted", selector.recover_to(Source::NativeOutputsSim, safe, 1002).accepted);
        native = frame(Source::NativeOutputsSim, 1002, 0.25F);
        check("recovery_restores_outputs_sim", only(selector.evaluate(1002, safe, &native, nullptr),
                                                    Source::NativeOutputsSim, 0.25F));
        selector.request_mode(Source::RflyOutputs, safe, 1003);
        const auto replay_after_switch = selector.evaluate(1003, safe, &native, &rfly);
        check("replay_watermark_survives_recovery_and_switch", zeros(replay_after_switch) &&
              replay_after_switch.fault == Fault::ReplayTimestamp);
    }
    {
        HilOutputSelector selector(config);
        selector.request_mode(Source::RflyOutputs, safe, 900);
        rfly = frame(Source::RflyOutputs, 1000, -0.5F);
        selector.evaluate(1000, active, nullptr, &rfly);
        rfly.timestamp_us = 999;
        const auto out = selector.evaluate(1001, active, nullptr, &rfly);
        check("older_stamp_replay_fault", zeros(out) && out.fault == Fault::ReplayTimestamp);
    }
    {
        HilOutputSelector selector(config);
        selector.request_mode(Source::RflyOutputs, safe, 1000);
        rfly = frame(Source::RflyOutputs, 999, -0.5F);
        const auto out = selector.evaluate(1000, active, nullptr, &rfly);
        check("pre_selection_sample_rejected", zeros(out) && out.fault == Fault::PreSelectionTimestamp);
    }
    {
        HilOutputSelector selector(config);
        native = frame(Source::NativeOutputsSim, 900, -1.0F);
        native.controls[1] = 1.0F;
        const auto out = selector.evaluate(1000, safe, &native, nullptr);
        check("inclusive_age_and_range_boundary", out.valid && out.controls[0] == -1.0F &&
              out.controls[1] == 1.0F && out.mode_flags == 0);
    }
    {
        HilOutputSelector selector(config);
        native = frame(Source::NativeOutputsSim, 1000, 0.25F);
        rfly = frame(Source::RflyOutputs, 999999, std::numeric_limits<float>::quiet_NaN());
        check("invalid_unselected_source_ignored", only(selector.evaluate(1000, safe, &native, &rfly),
                                                      Source::NativeOutputsSim, 0.25F));
    }
    for (std::size_t i = 0; i < 2; ++i) {
        HilOutputSelector selector(config);
        selector.request_mode(Source::RflyOutputs, safe, 900);
        auto context = active;
        if (i == 0) { context.hil_enabled = false; }
        else { context.physical_outputs_disabled = false; }
        rfly = frame(Source::RflyOutputs, 1000, -0.5F);
        const auto out = selector.evaluate(1000, context, nullptr, &rfly);
        check("active_environment_loss_fault_" + std::to_string(i), zeros(out) &&
              out.fault == Fault::UnsafeEnvironment && out.selected_source == Source::RflyOutputs);
    }
    {
        HilOutputSelector selector(config);
        native = frame(Source::NativeOutputsSim, 1000, 0.25F);
        selector.evaluate(1000, safe, &native, nullptr);
        const auto out = selector.evaluate(999, safe, &native, nullptr);
        check("host_clock_regression_fault", zeros(out) && out.fault == Fault::ClockRegression);
        check("backdated_recovery_rejected", !selector.recover_to(Source::NativeOutputsSim, safe, 999).accepted);
    }
    {
        auto invalid = config; invalid.max_age_us = 0;
        HilOutputSelector selector(invalid);
        const auto out = selector.evaluate(1000, safe, &native, &rfly);
        check("missing_age_policy_invalid_configuration", zeros(out) && out.fault == Fault::InvalidConfiguration &&
              !selector.recover_to(Source::NativeOutputsSim, safe, 1001).accepted);
    }
    {
        auto six = config; six.required_outputs = 6;
        HilOutputSelector selector(six);
        native = frame(Source::NativeOutputsSim, 1000, 0.25F);
        native.noutputs = 6;
        native.controls[6] = std::numeric_limits<float>::quiet_NaN();
        const auto out = selector.evaluate(1000, safe, &native, nullptr);
        check("caller_declared_count_zero_pads_unused", out.valid && out.noutputs == 6 &&
              out.controls[5] == 0.25F && out.controls[6] == 0.0F && out.controls[15] == 0.0F);
    }

    std::size_t passed = 0;
    for (const auto &item : cases) { if (item.passed) { ++passed; } }
    std::string json = "{\n  \"scope\": \"HOST_ONLY_SELECTOR_PROTOTYPE\",\n"
                       "  \"production_integrated\": false,\n  \"board_failsafe_validated\": false,\n"
                       "  \"uorb_mavlink_or_devices_used\": false,\n"
                       "  \"live_age_threshold_selected\": false,\n"
                       "  \"synthetic_test_age_limit_us\": 100,\n"
                       "  \"passed\": " + std::to_string(passed) + ",\n  \"total\": " +
                       std::to_string(cases.size()) + ",\n  \"status\": \"" +
                       (passed == cases.size() ? "PASS" : "FAIL") + "\",\n  \"checks\": {\n";
    for (std::size_t i = 0; i < cases.size(); ++i) {
        json += "    \"" + cases[i].name + "\": " + (cases[i].passed ? "true" : "false") +
                (i + 1 == cases.size() ? "\n" : ",\n");
    }
    json += "  }\n}\n";
    std::cout << json;
    if (argc == 2) {
        // Write test results to the explicit output path.
        std::ifstream existing(argv[1]);
        if (existing.good()) { std::cerr << "Refusing to overwrite result file\n"; return 2; }
        std::ofstream result(argv[1]);
        result << json;
        if (!result.good()) { return 3; }
    }
    return passed == cases.size() ? 0 : 1;
}
