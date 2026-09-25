#pragma once
#include "CanonicalLocalWindowWire.hpp"
#include "../../px4_full_inner/argument_transport/CanonicalArgumentTransport.hpp"

namespace gpenmpc_local_window_wire {
using Arrival=gpenmpc_argument_transport::Arrival;
struct Configuration {Binding expected{};std::uint64_t max_assembly_us{};};
enum class Fault:std::uint8_t {None,Configuration,Clock,Expired,Schema,Index,Length,Padding,
    Generation,BindingMismatch,Integrity,Busy,Release,Stopped};
struct Diagnostics {
    Fault first_fault{Fault::None};
    std::uint64_t first_fault_processing_us{},first_original_arrival_us{},last_original_arrival_us{},
        completed_processing_us{},last_processing_us{},highest_started_generation{},released_generation{},
        received_fragments{},completed_windows{},released_windows{};
    bool control_authority{},source_freshness_granted{}; // permanently false
};
// One real owner serializes calls, after its ONE central ingress envelope /
// actual uORB generation audit. No second subscriber, counter renumbering or
// public 'already_valid' receipt. receive() checks only this schema's framing,
// binding and original times; alone it is NOT an ingress-authority validator.
// Other schema messages must still traverse the same central global audit.
class Assembler final {
public:
    explicit Assembler(const Configuration&)noexcept;
    Assembler(const Assembler&)=delete;Assembler&operator=(const Assembler&)=delete;
    bool receive(const Arrival&,std::uint64_t actual_now)noexcept;
    bool tick(std::uint64_t actual_now)noexcept;
    const Window*ready_window(std::uint64_t actual_now)noexcept;
    // Consumer must first call the actual serialized Io.load_window. This
    // release only ends the immutable borrow; it is NOT a load/phase receipt.
    // Failed load: call stop(), retain staging, do not release and restart.
    bool release(std::uint64_t original_window_generation)noexcept;
    void stop(std::uint64_t actual_now)noexcept;
    const Window&retained_staging()const noexcept{return window_;}
    const Diagnostics&diagnostics()const noexcept{return d_;}
    const Arrival*first_fault_arrival()const noexcept{return have_fault_arrival_?&fault_arrival_:nullptr;}
    bool failed()const noexcept{return d_.first_fault!=Fault::None;}
private:
    bool fail(Fault,std::uint64_t)noexcept;
    bool accept_bytes(const std::uint8_t*,std::size_t)noexcept;
    const Configuration configuration_;
    Window window_{}; // single bounded staging; NEVER an installed reference
    gpenmpc_consumption::CanonicalSha256 sha_{};
    std::uint8_t manifest_[manifest_bytes]{},received_hash_[32]{};
    Arrival fault_arrival_{};
    const Arrival*inside_arrival_{};
    Diagnostics d_{};
    std::size_t bytes_received_{},next_{};
    std::uint64_t generation_{};
    bool active_{},ready_{},have_fault_arrival_{};
};
}
