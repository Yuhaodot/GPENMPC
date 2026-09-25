#pragma once
#include "../px4_runtime/CanonicalLocalWireOutbox.hpp"
#include "LinkLifetimeRegistry.hpp"
namespace gpenmpc_rfly_stream {
enum class LocalWireBind:std::uint8_t {Bound,Busy,Invalid,Unproven};
enum class LocalWireDetach:std::uint8_t {Detached,NotBound,Mismatch,Unproven};
struct LocalWireRegistration{std::uint64_t generation{};};
struct LocalWireRouteDiagnostics{std::uint64_t copied_fragments{},empty{},expired{},rejected{};};
class CanonicalLocalWireRouteRegistry final {
public:
    CanonicalLocalWireRouteRegistry()noexcept=default;
    CanonicalLocalWireRouteRegistry(const CanonicalLocalWireRouteRegistry&)=delete;
    CanonicalLocalWireRouteRegistry&operator=(const CanonicalLocalWireRouteRegistry&)=delete;
    bool ready()const noexcept{return mutex_.ready();}
    LocalWireBind bind(LinkToken,gpenmpc_rfly_px4::CanonicalLocalWireOutbox&,LocalWireRegistration&)noexcept;
    // Detach holds the copy-callback PI lock, closes the outbox and rejects
    // further takes. Retain storage until Detached or NotBound.
    LocalWireDetach unbind(LocalWireRegistration)noexcept;
    gpenmpc_rfly_px4::LocalWireTake take(LinkToken,const void*view,gpenmpc_rfly_px4::LocalWireFragment&,std::uint64_t actual_now)noexcept;
    // View destruction permanently closes this route's outbox, including a
    // partial message. A replacement stream cannot resume/relabel its tail.
    bool release_view(LinkToken,const void*view)noexcept;
    bool diagnostics(LocalWireRouteDiagnostics&)noexcept;
private:
    gpenmpc_rfly_px4::InheritingMutex mutex_{};
    gpenmpc_rfly_px4::CanonicalLocalWireOutbox*outbox_{};
    LinkToken link_{};const void*view_{};LocalWireRegistration registration_{};
    std::uint64_t next_generation_{};LocalWireRouteDiagnostics diagnostics_{};
};
CanonicalLocalWireRouteRegistry*canonical_local_wire_route_registry()noexcept;
}
