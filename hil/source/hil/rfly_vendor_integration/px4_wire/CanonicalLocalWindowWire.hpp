#pragma once
// RWW1: a complete ORIGINAL fixed window, never a host-generated control or
// source timestamp. Binary64 payloads (including unused NaN payload bits) are
// copied as bytes; this codec performs no interpolation or floating arithmetic.
#include "../full_inner_abi/CanonicalFullInnerAbi.h"
#include "RflySnapshotWireTypes.hpp"
#include "../../px4_full_inner/consumption/ConsumptionBinding.hpp"

namespace gpenmpc_local_window_wire {
using Window=gpenmpc_full_inner_window;
using Identity=gpenmpc_consumption::Identity;
using Hash=gpenmpc_portable::Array<std::uint32_t,8>;
using Fragment=gpenmpc_snapshot_wire::Fragment;
constexpr std::uint8_t schema=11;
constexpr std::size_t window_bytes=28484,manifest_bytes=162,message_bytes=28678;
constexpr std::size_t data_per_fragment=118,fragments_per_block=16,fragment_count=244,block_count=16;
struct Binding {
    Identity identity{};
    // These are independently registered owner values, not authentication by
    // self-reported wire hashes. A match alone grants no control permission.
    Hash execution_session_sha256{},task_sha256{},configuration_sha256{},reference_asset_sha256{};
    std::uint32_t leg_index{};
};
bool valid_binding(const Binding&)noexcept;
bool matches_window(const Binding&,const Window&)noexcept;
void encode_manifest(const Binding&,std::uint64_t generation,std::uint8_t out[manifest_bytes])noexcept;
bool copy_window_to_wire(const Window&,std::size_t offset,std::uint8_t*,std::size_t bytes)noexcept;
bool copy_wire_to_window(const std::uint8_t*,std::size_t bytes,std::size_t offset,Window&)noexcept;
// Same RWI1 single-window field order and little-endian numeric representation
// as CONTINUOUS_REFERENCE_FIXTURE.bin, without the fixture's global file header.
// POD compiler padding is not serialized. 20 spans, no temporary 28 KB buffer.
class Encoder final {
public:
    Encoder(const Binding&,const Window&)noexcept;
    Encoder(const Encoder&)=delete;Encoder&operator=(const Encoder&)=delete;
    bool valid()const noexcept{return valid_;}
    bool fragment(std::size_t absolute_index,Fragment&)const noexcept;
    const Hash&digest()const noexcept{return digest_;}
private:
    const Window*window_;
    std::uint8_t manifest_[manifest_bytes]{};
    Hash digest_{};
    std::uint64_t generation_{};
    bool valid_{};
};
// Frozen window must stay immutable through Encoder's last fragment. Mutation
// cannot silently replace a message: the final whole-message SHA will reject.
}
