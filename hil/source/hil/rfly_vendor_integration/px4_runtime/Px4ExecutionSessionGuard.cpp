#include "Px4ExecutionSessionGuard.hpp"
#include <drivers/drv_hrt.h>

namespace gpenmpc_rfly_px4 {
std::uint64_t Px4SessionClock::now()const noexcept{return hrt_absolute_time();}
Px4ExecutionSessionGuard *Px4ExecutionSessionGuard::create(Mavlink &actual_link,
    const SessionBoardIdentity &expected,const SessionDigest &configuration,std::uint64_t age,
    std::uint64_t commander_age) noexcept
{
    if(!age || (commander_age && commander_age<age) || !expected.uid || !expected.system ||
        !expected.component || !nonzero_session_digest(configuration))return nullptr;
    auto *ledger=execution_session_ledger();
    auto *links=gpenmpc_rfly_stream::link_lifetime_registry();
    gpenmpc_rfly_stream::LinkToken token{};
    if(!ledger || !links || links->lookup(&actual_link,token)!=gpenmpc_rfly_stream::LinkAccess::Read)return nullptr;
    // Real PX4 C++14 uses -fcheck-new/-fno-exceptions. Allocation failure returns
    // nullptr; caller must keep the configuration unavailable, not use a mock.
    return new Px4ExecutionSessionGuard(actual_link,expected,configuration,age,commander_age,*ledger,*links,token);
}
// Force the whole real RawGuard composition into the ARM translation unit,
// not merely the facade constructor or a HOST declaration stub.
template class SessionBoundGuard<Px4ReadOnlyGuard,Px4SessionClock>;
} // namespace gpenmpc_rfly_px4
