#include <common/mavlink.h>
#include "Px4OriginalHilReceipt.hpp"
// Caller-owned serial receiver object and scratch.
extern "C" bool gpenmpc_original_hil_pack_record(
    gpenmpc_hil_endpoint::Px4OriginalHilReceipt*observer,mavlink_message_t*message,
    mavlink_hil_sensor_t*decoded,std::uint64_t wire_time,std::uint64_t original_hrt,
    const void*receiver,const void*link,std::int32_t instance,std::int32_t channel,
    std::uint32_t fields,bool gyro_called,bool accel_called) noexcept {
    mavlink_msg_hil_sensor_pack(255,0,message,wire_time,
        1.0f,2.0f,3.0f,4.0f,5.0f,6.0f,7.0f,8.0f,9.0f,
        10.0f,11.0f,12.0f,13.0f,fields,0);
    mavlink_msg_hil_sensor_decode(message,decoded);
    return observer->record(*message,*decoded,original_hrt,receiver,link,instance,channel,
                            _MAV_PAYLOAD(message),gyro_called,accel_called);
}
