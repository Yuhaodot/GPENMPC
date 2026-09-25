#pragma once
#include "../clock_tap_overlay/ClockObservationTap.hpp"
#include <uORB/Publication.hpp>
#include <uORB/topics/gpenmpc_original_hil_receipt.h>

namespace gpenmpc_hil_endpoint {
// One object per MavlinkReceiver, accessed by its receiver thread.
// Capture follows PX4 sensor updates. Consumers reject missing or faulted records.
class Px4OriginalHilReceipt final {
public:
    Px4OriginalHilReceipt()noexcept=default;
    Px4OriginalHilReceipt(const Px4OriginalHilReceipt&)=delete;
    Px4OriginalHilReceipt&operator=(const Px4OriginalHilReceipt&)=delete;
    template<class Message,class Hil>
    bool record(const Message&msg,const Hil&hil,std::uint64_t original_receiver_hrt,
        const void*actual_receiver,const void*actual_link,std::int32_t instance,std::int32_t channel,
        const void*original_payload,bool actual_gyro_call,bool actual_accel_call,
        std::int16_t actual_gyro_instance=-1,std::int16_t actual_accel_instance=-1,
        std::uint32_t actual_gyro_device_id=0,std::uint32_t actual_accel_device_id=0)noexcept{
        if(sequence_==UINT64_MAX){overflow_=true;return false;}
        const auto original=gpenmpc_clock_tap::receiver_sample(msg,hil,original_receiver_hrt,instance,channel,original_payload);
        message_={};message_.timestamp=original.original_receiver_hrt_us;
        message_.wire_time_usec=original.original_wire_time_us;
        message_.receiver_address=reinterpret_cast<std::uintptr_t>(actual_receiver);
        message_.link_address=reinterpret_cast<std::uintptr_t>(actual_link);
        message_.original_event_sequence=++sequence_;message_.prior_publication_failures=failures_;
        message_.receiver_instance=original.receiver_instance;message_.channel=original.channel;
        message_.fields_updated=original.fields_updated;message_.system_id=original.system;
        message_.component_id=original.component;message_.mavlink_sequence=original.sequence;
        message_.payload_length=original.payload_length;message_.sensor_id=original.sensor_id;
        std::memcpy(message_.original_payload,original.original_payload65,sizeof message_.original_payload);
        message_.gyro_update_called=actual_gyro_call;message_.accel_update_called=actual_accel_call;
        message_.gyro_topic_instance=actual_gyro_call?actual_gyro_instance:-1;
        message_.accel_topic_instance=actual_accel_call?actual_accel_instance:-1;
        message_.gyro_device_id=actual_gyro_call?actual_gyro_device_id:0;
        message_.accel_device_id=actual_accel_call?actual_accel_device_id:0;
        const bool ok=publication_.publish(message_);
        if(!ok){if(failures_!=UINT64_MAX)++failures_;else overflow_=true;}
        return ok&&!overflow_;
    }
    std::uint64_t attempted_after_quiescence()const noexcept{return sequence_;}
    std::uint64_t failed_after_quiescence()const noexcept{return failures_;}
    bool overflow_after_quiescence()const noexcept{return overflow_;}
private:
    uORB::Publication<gpenmpc_original_hil_receipt_s> publication_{ORB_ID(gpenmpc_original_hil_receipt)};
    gpenmpc_original_hil_receipt_s message_{};
    std::uint64_t sequence_{},failures_{};bool overflow_{};
};
}
