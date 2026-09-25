#pragma once
#include "../px4_wire/RflySnapshotWireTypes.hpp"
namespace gpenmpc_rfly_px4 {
enum class ExportTake:std::uint8_t{Empty,Fragment,Expired,Stopped};
struct SnapshotFragment {
    gpenmpc_snapshot_wire::Fragment fragment{};
    std::uint64_t original_source_sample_us{},original_valid_until_us{};
    std::uint8_t target_system{},target_component{};
};
// A pump producer and MAVLink consumer exchange immutable messages
// through release/acquire synchronization.
class SnapshotOutbox final {
public:
    bool publish(const gpenmpc_snapshot_wire::Bytes&,std::uint64_t sample,std::uint64_t original_until,
                 std::uint8_t target_system,std::uint8_t target_component)noexcept;
    ExportTake take(SnapshotFragment&,std::uint64_t actual_now)noexcept;
    void close()noexcept{__atomic_store_n(&closed_,1,__ATOMIC_RELEASE);}
    bool failed()const noexcept{return __atomic_load_n(&expired_,__ATOMIC_ACQUIRE)!=0;}
    bool pending()const noexcept{return __atomic_load_n(&ready_,__ATOMIC_ACQUIRE)!=0;}
private:
    gpenmpc_snapshot_wire::Bytes bytes_{};std::uint64_t sample_{},until_{};
    std::uint8_t target_system_{},target_component_{},next_{};
    std::uint8_t ready_{},closed_{},expired_{};
};
} // namespace gpenmpc_rfly_px4
