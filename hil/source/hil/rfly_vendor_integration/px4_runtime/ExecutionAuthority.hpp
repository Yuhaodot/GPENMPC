#pragma once

#include "../../px4_full_inner/consumption/ConsumptionBinding.hpp"
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

namespace gpenmpc_rfly_px4 {
using Identity=gpenmpc_consumption::Identity;
using Token=gpenmpc_consumption::Token;

// Installation binds exact values and ORIGINAL expiry before orb_publish.
// Revocation retires that lease without waiting for another uORB publication.
// Neither operation is an arm/mode command or downstream stopping evidence.
class Authority {
public:
    virtual ~Authority()=default;
    virtual bool observed_identity(Identity &out) noexcept=0;
    virtual bool validate(const Token &,const vehicle_status_s &,
                          const vehicle_control_mode_s &,const offboard_control_mode_s &,
                          uint64_t now_us,uint64_t &original_authority_expiry_us) noexcept=0;
    virtual bool install_lease(const Token &,const actuator_outputs_s &,
                               uint64_t original_valid_until_us) noexcept=0;
    // Actual successful orb_publish AND numerical commit must precede stream
    // visibility. An installed, unconfirmed lease cannot be transmitted.
    virtual bool confirm_publication(const Token &,uint64_t original_publication_receipt_us,
                                     uint64_t original_commit_completed_us) noexcept=0;
    virtual void revoke() noexcept=0;
};
}
