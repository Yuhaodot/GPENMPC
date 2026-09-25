#include "BoardSafetyEvidence.hpp"
#include <cstdio>
#include <fstream>

using namespace gpenmpc_rfly_px4;
int main(int argc, char **argv)
{
    if (argc != 2 || std::ifstream(argv[1]).good()) { return 64; }
    unsigned checks = 0, failures = 0;
    const auto check = [&](bool ok) { ++checks; if (!ok) { ++failures; } };
    BoardSafetyEvidence e{};
    check(!e.require_identity_only()); check(!e.require_active_direct()); check(!e.require_physical_facts());
    e.uid = e.mavlink_identity = GuardFact::Pass;
    check(e.require_identity_only()); check(!e.require_active_direct());
    check(e.boot_session == GuardFact::Unknown && e.identity.boot_generation == 0);
    GuardFact BoardSafetyEvidence::* identity[] = {&BoardSafetyEvidence::uid, &BoardSafetyEvidence::mavlink_identity};
    for (auto field : identity) {
        for (auto value : {GuardFact::Unknown, GuardFact::Fail}) {
            e.*field = value; check(!e.require_identity_only());
        }
        e.*field = GuardFact::Pass;
    }
    GuardFact BoardSafetyEvidence::* active[] = {&BoardSafetyEvidence::usb_transport,
        &BoardSafetyEvidence::hil_configuration, &BoardSafetyEvidence::telemetry_fresh,
        &BoardSafetyEvidence::active_direct_mode, &BoardSafetyEvidence::native_controllers_disabled};
    for (auto field : active) { e.*field = GuardFact::Pass; }
    check(e.require_active_direct());
    for (auto field : active) {
        for (auto value : {GuardFact::Unknown, GuardFact::Fail}) {
            e.*field = value; check(!e.require_active_direct()); check(e.require_identity_only());
        }
        e.*field = GuardFact::Pass;
    }
    GuardFact BoardSafetyEvidence::* physical[] = {&BoardSafetyEvidence::pwm_functions_zero,
        &BoardSafetyEvidence::pwm_out_stopped, &BoardSafetyEvidence::io_driver_stopped, &BoardSafetyEvidence::dshot_stopped,
        &BoardSafetyEvidence::usb_power_observed, &BoardSafetyEvidence::external_physical_isolation};
    for (auto field : physical) { e.*field = GuardFact::Pass; }
    check(e.require_physical_facts());
    for (auto field : physical) {
        for (auto value : {GuardFact::Unknown, GuardFact::Fail}) {
            e.*field = value; check(!e.require_physical_facts());
        }
        e.*field = GuardFact::Pass;
    }
    e.external_physical_isolation = GuardFact::Unknown;
    e.raw_usb_connected = e.raw_usb_valid = e.raw_servo_valid = 1;
    e.raw_brick_valid = 0;
    check(!e.require_physical_facts()); // USB/raw power flags do not create isolation.
    check(guard_original_expiry(0, 100) == 0);
    check(guard_original_expiry(100, 0) == 0);
    check(guard_original_expiry(UINT64_MAX, 1) == 0);
    check(guard_original_expiry(UINT64_MAX - 1, 1) == UINT64_MAX);
    check(guard_original_expiry(UINT64_C(9007199254741111), 99) == UINT64_C(9007199254741210));
    std::ofstream out(argv[1]);
    out << "{\"scope\":\"HOST_FACT_PREDICATES_ONLY_NOT_BOARD_GUARD_EXECUTION\",\"checks\":"
        << checks << ",\"failures\":" << failures << ",\"hardware_reads\":0}\n";
    std::printf("HOST fact predicates: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
