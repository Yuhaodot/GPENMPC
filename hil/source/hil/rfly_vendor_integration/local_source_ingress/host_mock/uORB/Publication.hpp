#pragma once
#include <uORB/uORB.h>
#include <uORB/topics/gpenmpc_original_hil_receipt.h>
namespace original_hil_mock {
extern bool next_publish_result;
extern unsigned attempts;
extern gpenmpc_original_hil_receipt_s last_attempt;
}
namespace uORB {
template<class T>class Publication {
public:
    explicit Publication(const orb_metadata*) noexcept {}
    bool publish(const T&m) noexcept {
        ++original_hil_mock::attempts;
        original_hil_mock::last_attempt=m;
        return original_hil_mock::next_publish_result;
    }
};
}
