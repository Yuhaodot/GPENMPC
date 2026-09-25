#include "Px4OriginalHilReceiptReader.hpp"
#include <cstring>
namespace gpenmpc_hil_endpoint_reader {
Px4OriginalHilReceiptReader::Access::Access(Px4OriginalHilReceiptReader&s)noexcept:self(s),held(false){
    static_assert(__atomic_always_lock_free(sizeof(std::uint32_t),nullptr),"Actual lock-free 32-bit CAS required");
    std::uint32_t expected=0;held=__atomic_compare_exchange_n(&self.busy_,&expected,1,false,__ATOMIC_ACQUIRE,__ATOMIC_RELAXED);
    if(!held)(void)self.fail(Fault::AccessConflict);
}
Px4OriginalHilReceiptReader::Access::~Access()noexcept{if(held)__atomic_store_n(&self.busy_,0,__ATOMIC_RELEASE);}
bool Px4OriginalHilReceiptReader::fail(Fault f)noexcept{
    std::uint32_t expected=0;(void)__atomic_compare_exchange_n(&fault_,&expected,static_cast<std::uint32_t>(f),false,__ATOMIC_RELEASE,__ATOMIC_RELAXED);return false;
}
Px4OriginalHilReceiptReader::Px4OriginalHilReceiptReader(const Configuration&c)noexcept:configuration_(c){
    const auto&i=c.independently_observed_board_identity;
    if(!c.expected_registered_link||c.expected_receiver_instance<0||c.expected_channel<0||!i.uid||!i.boot_generation||!i.system||!i.component)
        (void)fail(Fault::Configuration);
}
bool Px4OriginalHilReceiptReader::observe(const gpenmpc_original_hil_receipt_s&m,std::uint32_t generation)noexcept{
    if(diagnostics_.observed_topic_records==UINT64_MAX)return fail(Fault::CounterOverflow);
    ++diagnostics_.observed_topic_records;diagnostics_.last_observed={m,generation};
    if(!generation||!m.original_event_sequence||!m.timestamp||!m.receiver_address||m.receiver_address>UINTPTR_MAX||
       !m.link_address||m.link_address>UINTPTR_MAX||!m.payload_length||m.payload_length>65)return fail(Fault::Malformed);
    if(m.prior_publication_failures)return fail(Fault::ProducerPublicationFailure);
    if(m.link_address!=reinterpret_cast<std::uintptr_t>(configuration_.expected_registered_link)||
       m.receiver_instance!=configuration_.expected_receiver_instance||m.channel!=configuration_.expected_channel||
       m.system_id!=configuration_.expected_noui_system||m.component_id!=configuration_.expected_noui_component||
       (bound_&&m.receiver_address!=diagnostics_.observed_receiver_address))return fail(Fault::OwnerMismatch);
    if(bound_){
        if(diagnostics_.last_original_subscription_generation==UINT32_MAX||diagnostics_.last_original_event_sequence==UINT64_MAX)return fail(Fault::CounterOverflow);
        if(generation!=diagnostics_.last_original_subscription_generation+1)return fail(Fault::TopicGap);
        if(m.original_event_sequence!=diagnostics_.last_original_event_sequence+1)return fail(Fault::ProducerGap);
    }
    // Record contains the original actual call booleans. A call claimed with
    // missing required fields is malformed; fields alone never invent a call.
    if((m.gyro_update_called&&(m.fields_updated&0x38U)!=0x38U)||
       (m.accel_update_called&&(m.fields_updated&0x07U)!=0x07U))return fail(Fault::Malformed);
    if(count_==capacity)return fail(Fault::RetentionOverflow);
    if(!bound_){
        gpenmpc_source_receipt::Owner owner{};owner.receiver=reinterpret_cast<const void*>(static_cast<std::uintptr_t>(m.receiver_address));
        owner.instance=m.receiver_instance;owner.channel=m.channel;owner.system=m.system_id;owner.component=m.component_id;
        owner.board_identity=configuration_.independently_observed_board_identity;
        if(!lookup_.bind_once_quiescent(true,owner))return fail(Fault::Lookup);
        bound_=true;diagnostics_.observed_baseline=true;diagnostics_.observed_receiver_address=m.receiver_address;
        diagnostics_.first_original_subscription_generation=generation;diagnostics_.first_original_event_sequence=m.original_event_sequence;
        diagnostics_.first_original_receiver_hrt_us=m.timestamp;
    }
    gpenmpc_clock_tap::ReceiverSample tap{};tap.message_id=107;tap.original_receiver_hrt_us=m.timestamp;tap.original_wire_time_us=m.wire_time_usec;
    tap.event_ordinal=m.original_event_sequence;tap.receiver_instance=m.receiver_instance;tap.channel=m.channel;
    tap.system=m.system_id;tap.component=m.component_id;tap.sequence=m.mavlink_sequence;tap.payload_length=m.payload_length;
    tap.fields_updated=m.fields_updated;tap.sensor_id=m.sensor_id;std::memcpy(tap.original_payload65,m.original_payload,65);
    if(!lookup_.observe(reinterpret_cast<const void*>(static_cast<std::uintptr_t>(m.receiver_address)),tap))return fail(Fault::Lookup);
    retained_[count_++]={m,generation};++diagnostics_.accepted_records;
    diagnostics_.last_original_subscription_generation=generation;diagnostics_.last_original_event_sequence=m.original_event_sequence;
    diagnostics_.last_original_receiver_hrt_us=m.timestamp;return true;
}
Drain Px4OriginalHilReceiptReader::drain()noexcept{
    Access access(*this);if(!access.held||fault()!=Fault::None)return Drain::Unavailable;
    std::size_t read=0;
    for(;read<drain_limit;++read){
        if(!subscription_.update(&message_))break;
        const auto original_generation=subscription_.get_last_generation();
        if(!observe(message_,original_generation))return Drain::Unavailable;
    }
    if(fault()!=Fault::None)return Drain::Unavailable;
    return read?Drain::Observed:Drain::NoUpdate;
}
bool Px4OriginalHilReceiptReader::lookup(const gpenmpc_odometry::Snapshot&s,Receipt&out)noexcept{
    out={};Access access(*this);if(!access.held||fault()!=Fault::None)return false;
    // A lookup before baseline is an explicit missing-endpoint failure, never
    // a reason to build a receiver identity from the requested Snapshot.
    if(!bound_)return fail(Fault::Lookup);
    gpenmpc_source_receipt::Receipt endpoint{};if(!lookup_.lookup(s,endpoint))return fail(Fault::Lookup);
    std::size_t index=0;for(;index<count_;++index)if(retained_[index].topic.timestamp==endpoint.original.original_receiver_hrt_us)break;
    if(index==count_)return fail(Fault::Lookup);
    if(!retained_[index].topic.gyro_update_called)return fail(Fault::ActualGyroNotCalled);
    if(diagnostics_.successful_lookups==UINT64_MAX)return fail(Fault::CounterOverflow);
    if(!gpenmpc_local_input::snapshot_key(s,out.original_snapshot_key))return fail(Fault::Lookup);
    out.original_source_topic=s.source_topic();out.original_source_instance=s.source_instance();
    ++diagnostics_.successful_lookups;out.endpoint=endpoint;out.original=retained_[index];
    if(fault()!=Fault::None){out={};return false;}return true;
}
bool Px4OriginalHilReceiptReader::retire_through(std::uint64_t endpoint)noexcept{
    Access access(*this);if(!access.held||fault()!=Fault::None)return false;
    if(!bound_||!lookup_.retire_through(endpoint))return fail(Fault::Retirement);
    std::size_t retired=0;while(retired<count_&&retained_[retired].topic.timestamp<=endpoint)++retired;
    if(!retired||retained_[retired-1].topic.timestamp!=endpoint||diagnostics_.retired_records>UINT64_MAX-retired)return fail(Fault::Retirement);
    for(std::size_t i=retired;i<count_;++i)retained_[i-retired]=retained_[i];
    count_-=retired;diagnostics_.retired_records+=retired;return true;
}
void Px4OriginalHilReceiptReader::stop()noexcept{(void)fail(Fault::Stopped);lookup_.stop();}
bool Px4OriginalHilReceiptReader::diagnostics(Diagnostics&out)noexcept{
    out={};Access access(*this);if(!access.held)return false;out=diagnostics_;out.retained=count_;out.first_fault=fault();out.lookup_fault=lookup_.fault();return true;
}
bool Px4OriginalHilReceiptReader::audit_copy(std::size_t i,Original&out)noexcept{
    out={};Access access(*this);if(!access.held||i>=count_)return false;out=retained_[i];return true;
}
}
