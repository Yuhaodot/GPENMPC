#include "Px4ReadOnlyGuard.hpp"
#include "CanonicalOutputAuthority.hpp"
#include <drivers/drv_hrt.h>

namespace gpenmpc_rfly_px4 {
struct GuardActualHrtClock {
    std::uint64_t now() const noexcept { return hrt_absolute_time(); }
};

// Explicit instantiation checks every virtual method against the actual guard
// and target pthread/uORB interfaces; it creates no object and runs no code.
template class CanonicalOutputAuthority<Px4ReadOnlyGuard, GuardActualHrtClock>;
} // namespace gpenmpc_rfly_px4
