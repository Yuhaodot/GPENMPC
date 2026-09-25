#pragma once
#include "LinkLifetimeRegistry.hpp"

namespace gpenmpc_rfly_stream {

// Each original native SET_POSITION_TARGET_{LOCAL_NED,GLOBAL_INT} and
// SET_ATTITUDE_TARGET handler holds this from entry through every return.
// No registry lock is held while the handler runs. A canonical reservation
// cannot start while this exact scoped borrower is entered; once reserved,
// all three native handlers are rejected on every link (OCM is a global topic).
class NativeOffboardBorrow final {
public:
    explicit NativeOffboardBorrow(const void *actual_link) noexcept:
        registry_(link_lifetime_registry())
    {
        entered_=registry_ && registry_->enter_native_offboard(actual_link,borrow_)==LinkAccess::Read;
    }
    ~NativeOffboardBorrow()
    {
        if(entered_)(void)registry_->leave_native_offboard(borrow_);
    }
    NativeOffboardBorrow(const NativeOffboardBorrow &)=delete;
    NativeOffboardBorrow &operator=(const NativeOffboardBorrow &)=delete;
    explicit operator bool()const noexcept{return entered_;}
private:
    LinkLifetimeRegistry *registry_{};
    NativeBorrowToken borrow_{};
    bool entered_{false};
};
} // namespace gpenmpc_rfly_stream
