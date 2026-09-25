#pragma once
#include "../CanonicalFullInnerConsumption.hpp"
namespace gpenmpc_rfly_px4 {
enum class LocalStateKind:std::uint8_t {Unavailable,InitialUncommitted,CommittedNoGp,
    CommittedAwaitingGp,CommittedGpReady,HistoricalUnusable};
struct LocalInstalledNumerics {
    gpenmpc_full_consumption::StatefulToken token{};
    double actual_input36[36]{},actual_kernel61[61]{},actual_request19[19]{};
    float actual_published_control16[16]{};
    std::uint64_t joint_install_count{},original_publication_us{};
    bool present{};
};
// POD-like fixed-shape same-owner observation, NOT a wire/authority format.
// state.pending70 is the NEXT OPEN prediction, never LatestClosedEvidence.
// Before First executes, state64 is zero STORAGE and numeric_installed=false;
// InitialUncommitted is not an initialized/valid observer state.
struct LocalNumericalObservation {
    LocalStateKind kind{LocalStateKind::Unavailable};
    gpenmpc_full_inner_state state{};
    gpenmpc_full_inner_closed_evidence closed_gp{};
    gpenmpc_full_inner_diagnostics actual_diagnostics{};
    gpenmpc_full_inner_configuration configuration{};
    gpenmpc_full_inner_build_identity build{};
    LocalInstalledNumerics last_installed{};
    // Owner event counters.
    std::uint64_t actual_joint_installs{},actual_gp_fills{};
    std::uint32_t io_first_fault{};
    bool copied{},capture_pending{},execution_started{},unread_execution_feedback{},
        installed_evidence_matches_state{},latest_closed_gp_evidence_available{},
        control_authority{},source_freshness_granted{};
};
}
