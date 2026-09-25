#pragma once

#include "../px4_runtime/InheritingMutex.hpp"
#include "RegisteredHostHeartbeat.hpp"
#include <cstdint>

namespace gpenmpc_rfly_px4 {struct SessionEcho;}

namespace gpenmpc_rfly_stream {

struct LinkToken {
    const void *pointer{nullptr};
    std::uint64_t generation{0};
};
inline bool operator==(const LinkToken &a,const LinkToken &b)noexcept
{return a.pointer==b.pointer && a.generation==b.generation;}
inline bool operator!=(const LinkToken &a,const LinkToken &b)noexcept{return !(a==b);}
enum class LinkRegistration:std::uint8_t {Registered,Invalid,Conflict,Full,Unproven};
enum class LinkAccess:std::uint8_t {Read,Unavailable,Rejected,Unproven};
enum class LinkRetirement:std::uint8_t {Quiesced,NeverRegistered,Mismatch,Unproven};
enum class CanonicalBind:std::uint8_t {Bound,Busy,Unavailable,Invalid,Unproven};
struct CanonicalReservation {
    LinkToken link{};const void *owner{};std::uint64_t generation{};HostHeartbeatBinding heartbeat{};
};
inline bool operator==(const CanonicalReservation &a,const CanonicalReservation &b)noexcept
{return a.link==b.link&&a.owner==b.owner&&a.generation==b.generation&&a.heartbeat==b.heartbeat;}
struct NativeBorrowToken {LinkToken link{};std::uint64_t generation{};};
// The callback must be a bounded read of immutable, initialized link properties.
// It must not retain/dereference pointer after return, call any registry/owner,
// acquire another lock, or cause task/object destruction. The PI mutex remains
// held for the complete callback; no raw-pointer lookup API is provided.
// C++14 ABI: callback implementations must be noexcept, but noexcept cannot be
// part of this pointer typedef without -Wnoexcept-type under the real toolchain.
using LinkReadCallback=bool (*)(const void *pointer,void *result);

class LinkLifetimeRegistry final {
public:
    static constexpr unsigned capacity=16; // bounded storage, Full rejects
    LinkLifetimeRegistry()noexcept=default;
    LinkLifetimeRegistry(const LinkLifetimeRegistry &)=delete;
    LinkLifetimeRegistry &operator=(const LinkLifetimeRegistry &)=delete;
    bool ready()const noexcept{return mutex_.ready();}
    LinkRegistration register_constructed(const void *pointer,LinkToken &token)noexcept;
    // Actual Mavlink task_main invokes this only AFTER final USB-property write.
    LinkAccess activate(const void *pointer)noexcept;
    LinkAccess lookup(const void *pointer,LinkToken &token)noexcept;
    LinkAccess read(const LinkToken &token,LinkReadCallback callback,void *result)noexcept;
    LinkRetirement retire(const LinkToken &token)noexcept;
    // Actual destruction path has the exact object address. Checked before
    // every delete; destructor's repeat is only an idempotent defensive check.
    LinkRetirement retire_instance(const void *pointer)noexcept;
    // Only the confirmed Context's acquire, under the real module lifecycle
    // mutex after its real legacy-stopped check, may bind this reservation.
    // These members use THIS registry's one PI mutex and existing Entry.
    CanonicalBind bind_canonical(const gpenmpc_rfly_px4::SessionEcho &echo,
        std::uint8_t host_system,std::uint8_t host_component,const void *owner,CanonicalReservation &out)noexcept;
    LinkAccess release_canonical(const CanonicalReservation &reservation)noexcept;
    LinkAccess read_heartbeat(const CanonicalReservation &reservation,std::uint64_t now,
        std::uint64_t max_age,HostHeartbeatSnapshot &out)noexcept;
    HeartbeatRecord record_heartbeat(const void *actual_link,std::uint8_t system,std::uint8_t component,
        std::uint8_t type,std::uint8_t autopilot,std::uint64_t original_receiver_hrt_us)noexcept;
    // Query is sufficient only for legacy task_spawn under px4_modules_mutex.
    // The three concurrent MAVLink handlers MUST use the scoped enter/leave.
    LinkAccess native_offboard_allowed()noexcept;
    LinkAccess enter_native_offboard(const void *actual_link,NativeBorrowToken &out)noexcept;
    LinkAccess leave_native_offboard(const NativeBorrowToken &borrow)noexcept;
private:
    enum class State:std::uint8_t {Empty,Constructed,Live,Closing};
    struct Entry {
        LinkToken token{};State state{State::Empty};
        CanonicalReservation reservation{};RegisteredHostHeartbeat heartbeat{};
        std::uint64_t native_borrow_generation{};
    };
    LinkRetirement retire_locked(const void *pointer,std::uint64_t generation,bool exact)noexcept;
    gpenmpc_rfly_px4::InheritingMutex mutex_{};
    Entry entries_[capacity]{};
    std::uint64_t next_generation_{0};
    std::uint64_t next_reservation_generation_{0},next_native_borrow_generation_{0};
};

// Lazy process-lifetime instance. Initialization failure/contention -> nullptr,
// never an unguarded access. Generation is NOT a persistent boot/session nonce.
LinkLifetimeRegistry *link_lifetime_registry()noexcept;

} // namespace gpenmpc_rfly_stream
