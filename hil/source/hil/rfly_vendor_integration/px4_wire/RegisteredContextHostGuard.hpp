#pragma once
// Explicit HOST test seam only: the real session ledger, ticket checks, identity
// binding and PI mutex execute; only raw hardware evidence and HRT are simulated.
#include "../px4_runtime/SessionBoundGuard.hpp"
#include <drivers/drv_hrt.h>
namespace gpenmpc_rfly_px4 {
bool gpenmpc_context_simulated_board_evidence(BoardSafetyEvidence&)noexcept;
struct ContextHostClock{std::uint64_t now()const noexcept{return hrt_absolute_time();}};
struct ContextHostRawGuard{bool observe(BoardSafetyEvidence&e)noexcept{return gpenmpc_context_simulated_board_evidence(e);}};
using RegisteredContextGuard=SessionBoundGuard<ContextHostRawGuard,ContextHostClock>;
} // namespace gpenmpc_rfly_px4
