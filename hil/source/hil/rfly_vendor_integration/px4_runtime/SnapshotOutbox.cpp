#include "SnapshotOutbox.hpp"
namespace gpenmpc_rfly_px4 {
static_assert(__atomic_always_lock_free(1,nullptr),"outbox requires actual target lock-free byte operations");
bool SnapshotOutbox::publish(const gpenmpc_snapshot_wire::Bytes&bytes,std::uint64_t sample,std::uint64_t until,
    std::uint8_t target_system,std::uint8_t target_component)noexcept{
    if(__atomic_load_n(&closed_,__ATOMIC_ACQUIRE)||failed()||pending()||!sample||until<sample||!target_system||!target_component)return false;
    bytes_=bytes;sample_=sample;until_=until;target_system_=target_system;target_component_=target_component;next_=0;
    __atomic_store_n(&ready_,1,__ATOMIC_RELEASE);return true;
}
ExportTake SnapshotOutbox::take(SnapshotFragment&out,std::uint64_t now)noexcept{
    out={};if(__atomic_load_n(&closed_,__ATOMIC_ACQUIRE))return ExportTake::Stopped;
    if(!pending())return ExportTake::Empty;
    if(now<sample_||now>until_){__atomic_store_n(&expired_,1,__ATOMIC_RELEASE);__atomic_store_n(&ready_,0,__ATOMIC_RELEASE);return ExportTake::Expired;}
    if(!gpenmpc_snapshot_wire::fragment(bytes_,next_,out.fragment)){close();return ExportTake::Stopped;}
    out.original_source_sample_us=sample_;out.original_valid_until_us=until_;out.target_system=target_system_;out.target_component=target_component_;
    ++next_;if(next_==gpenmpc_snapshot_wire::fragment_count)__atomic_store_n(&ready_,0,__ATOMIC_RELEASE);
    return ExportTake::Fragment; // Fragment copied to the caller.
}
} // namespace gpenmpc_rfly_px4
