#pragma once
#include <cstdint>

#ifndef GPENMPC_BOARD_UID
#error "Configure the expected board identity with GPENMPC_BOARD_UID"
#endif

namespace gpenmpc_board_configuration {
constexpr std::uint64_t expected_uid = GPENMPC_BOARD_UID;
static_assert(expected_uid != 0, "The expected board UID must be nonzero");
}
