#pragma once

#include "RflyStreamAuthority.hpp"
#include "LinkLifetimeRegistry.hpp"
#include "../px4_runtime/InheritingMutex.hpp"

namespace gpenmpc_rfly_stream {

// Bind both pointer and process-local lifetime generation.
// The guard separately validates USB and physical isolation.
using Link = LinkToken;
struct Registration { std::uint64_t generation{0}; };
enum class Bind : std::uint8_t { Registered, Busy, Invalid, Unproven };
enum class Detach : std::uint8_t { Detached, NotBound, Mismatch, Unproven };
struct RegistryDiagnostics {
    std::uint64_t bindings{0},unbindings{0},choose_calls{0},accept_calls{0};
    std::uint64_t owner_choose_calls{0},owner_accept_calls{0};
    bool bound{false},view_claimed{false};
};

// One exact link, one owner, one active stream consumer. A registration does
// not grant control: the owner's choose/accept still establish actual authority.
// Lock order is registry -> owner; never call detach while holding owner lock.
// Detach waits for in-flight callbacks and nulls the pointer before returning.
// On Unproven, retain owner and guard storage.
class SharedOutputRegistry final {
public:
    SharedOutputRegistry() noexcept = default;
    SharedOutputRegistry(const SharedOutputRegistry &)=delete;
    SharedOutputRegistry &operator=(const SharedOutputRegistry &)=delete;
    bool ready()const noexcept{return mutex_.ready();}

    // Unproven may mean the pointer was installed but unlock failed: retain
    // owner storage and use receipt for cleanup, never destroy on bool failure.
    Bind bind(Link link,Authority &owner,Registration &receipt)noexcept
    {
        receipt={};
        if(!link.pointer || !link.generation)return Bind::Invalid;
        if(!mutex_.lock())return Bind::Unproven;
        const bool ok=!owner_ && next_generation_!=UINT64_MAX;
        if(ok){
            owner_=&owner;link_=link;view_=nullptr;
            ++next_generation_;registration_.generation=next_generation_;
            receipt=registration_;++diagnostics_.bindings;
        }
        if(!mutex_.unlock())return Bind::Unproven;
        return ok?Bind::Registered:Bind::Busy;
    }

    Detach unbind(const Registration &receipt)noexcept
    {
        if(!mutex_.lock())return Detach::Unproven;
        Detach result=Detach::Mismatch;
        if(!owner_)result=Detach::NotBound;
        else if(receipt.generation && receipt.generation==registration_.generation){
            owner_=nullptr;link_={};view_=nullptr;registration_={};
            ++diagnostics_.unbindings;result=Detach::Detached;
        }
        return mutex_.unlock()?result:Detach::Unproven;
    }

    Source choose(Link link,const void *view,std::uint64_t now,
                  const vehicle_status_s &status,const vehicle_control_mode_s &mode,
                  Registration &selection)noexcept
    {
        selection={};
        if(!view || !mutex_.lock())return Source::Unavailable;
        ++diagnostics_.choose_calls;
        Source source=Source::Unavailable;
        if(owner_ && link==link_ && (!view_ || view_==view)){
            view_=view;++diagnostics_.owner_choose_calls;
            source=owner_->choose(now,status,mode);
            if(source!=Source::Unavailable)selection=registration_;
        }
        if(!mutex_.unlock()){selection={};return Source::Unavailable;}
        return source;
    }

    bool accept(Link link,const void *view,const Registration &selection,
                const Observation &observation,std::uint64_t now,
                OriginalValidity &original)noexcept
    {
        original={};
        if(!mutex_.lock())return false;
        ++diagnostics_.accept_calls;
        bool ok=false;
        if(owner_ && link==link_ && view && view==view_ && selection.generation &&
           selection.generation==registration_.generation){
            ++diagnostics_.owner_accept_calls;
            ok=owner_->accept(observation,now,original);
        }
        if(!mutex_.unlock())ok=false;
        if(!ok)original={};
        return ok;
    }

    bool release_view(Link link,const void *view)noexcept
    {
        if(!mutex_.lock())return false;
        if(link==link_ && view && view==view_)view_=nullptr;
        return mutex_.unlock();
    }

    bool diagnostics(RegistryDiagnostics &out)noexcept
    {
        if(!mutex_.lock())return false;
        out=diagnostics_;out.bound=owner_!=nullptr;out.view_claimed=view_!=nullptr;
        return mutex_.unlock();
    }
private:
    gpenmpc_rfly_px4::InheritingMutex mutex_{};
    Authority *owner_{nullptr};
    Link link_{};
    const void *view_{nullptr};
    Registration registration_{};
    std::uint64_t next_generation_{0};
    RegistryDiagnostics diagnostics_{};
};

// Explicit process-lifetime singleton, initialized once on a real task (not a
// C++ global constructor or unprotected local-static with PX4's disabled guard).
// Concurrent initialization returns nullptr, never spins or invents authority.
// It owns only routing and a mutex; never owns the module's Authority/Guard.
SharedOutputRegistry *shared_output_registry() noexcept;

class Router final : public Authority {
public:
    // An already obtained local token (also used by explicit mock fixtures).
    Router(SharedOutputRegistry *registry,Link link)noexcept:
        registry_(registry),link_(link),actual_pointer_(link.pointer),resolved_(true){}
    // The real stream can be constructed BEFORE Mavlink task initialization.
    // Resolve once after Live, never re-resolve a retired/same-address object.
    Router(SharedOutputRegistry *registry,const void *actual_pointer)noexcept:
        registry_(registry),actual_pointer_(actual_pointer){}
    ~Router()override{if(registry_)(void)registry_->release_view(link_,this);}
    Router(const Router &)=delete;
    Router &operator=(const Router &)=delete;
    Source choose(std::uint64_t now,const vehicle_status_s &status,
                  const vehicle_control_mode_s &mode)noexcept override
    {
        selection_={};
        if(!resolved_){
            auto *lifetime=link_lifetime_registry();
            if(!lifetime || lifetime->lookup(actual_pointer_,link_)!=LinkAccess::Read)
                return Source::Unavailable;
            resolved_=true;
        }
        if(!registry_)registry_=shared_output_registry();
        return registry_?registry_->choose(link_,this,now,status,mode,selection_):Source::Unavailable;
    }
    bool accept(const Observation &observation,std::uint64_t now,
                OriginalValidity &original)noexcept override
    {
        const Registration selected=selection_;selection_={};
        if(!registry_){original={};return false;}
        return registry_->accept(link_,this,selected,observation,now,original);
    }
private:
    SharedOutputRegistry *registry_;
    Link link_{};
    const void *actual_pointer_{nullptr};
    bool resolved_{false};
    Registration selection_{};
};

} // namespace gpenmpc_rfly_stream
