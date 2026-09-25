#include "LinkLifetimeRegistry.hpp"
#include <new>

namespace gpenmpc_rfly_stream {
LinkRegistration LinkLifetimeRegistry::register_constructed(const void *pointer,LinkToken &token)noexcept
{
    token={};if(!pointer)return LinkRegistration::Invalid;
    if(!mutex_.lock())return LinkRegistration::Unproven;
    Entry *empty=nullptr;LinkRegistration outcome=LinkRegistration::Full;
    for(auto &entry:entries_){
        if(entry.state!=State::Empty && entry.token.pointer==pointer){
            // A duplicate construction must not revive or reuse an older
            // generation. Fail closed even if its destructor hook was missed.
            entry.state=State::Closing;
            outcome=LinkRegistration::Conflict;empty=nullptr;break;
        }
        if(entry.state==State::Empty && !empty)empty=&entry;
    }
    if(empty && next_generation_!=UINT64_MAX){
        empty->token={pointer,++next_generation_};empty->state=State::Constructed;
        token=empty->token;outcome=LinkRegistration::Registered;
    }
    return mutex_.unlock()?outcome:LinkRegistration::Unproven;
}

LinkAccess LinkLifetimeRegistry::activate(const void *pointer)noexcept
{
    if(!pointer)return LinkAccess::Unavailable;
    if(!mutex_.lock())return LinkAccess::Unproven;
    LinkAccess outcome=LinkAccess::Unavailable;
    for(auto &entry:entries_)if(entry.token.pointer==pointer && entry.state==State::Constructed){
        entry.state=State::Live;outcome=LinkAccess::Read;break;
    }
    return mutex_.unlock()?outcome:LinkAccess::Unproven;
}

LinkAccess LinkLifetimeRegistry::lookup(const void *pointer,LinkToken &token)noexcept
{
    token={};if(!pointer)return LinkAccess::Unavailable;
    if(!mutex_.lock())return LinkAccess::Unproven;
    LinkAccess outcome=LinkAccess::Unavailable;
    for(const auto &entry:entries_)if(entry.token.pointer==pointer && entry.state==State::Live){
        token=entry.token;outcome=LinkAccess::Read;break;
    }
    if(!mutex_.unlock()){token={};return LinkAccess::Unproven;}
    return outcome;
}

LinkAccess LinkLifetimeRegistry::read(const LinkToken &token,LinkReadCallback callback,void *result)noexcept
{
    if(!token.pointer || !token.generation || !callback || !result)return LinkAccess::Unavailable;
    if(!mutex_.lock())return LinkAccess::Unproven;
    LinkAccess outcome=LinkAccess::Unavailable;
    for(const auto &entry:entries_)if(entry.state==State::Live && entry.token==token){
        outcome=callback(entry.token.pointer,result)?LinkAccess::Read:LinkAccess::Rejected;break;
    }
    // Caller must discard callback output unless the returned status is Read.
    return mutex_.unlock()?outcome:LinkAccess::Unproven;
}

LinkRetirement LinkLifetimeRegistry::retire_locked(const void *pointer,std::uint64_t generation,bool exact)noexcept
{
    for(auto &entry:entries_)if(entry.state!=State::Empty && entry.token.pointer==pointer){
        if(exact && entry.token.generation!=generation)return LinkRetirement::Mismatch;
        // Acquiring the same mutex has already drained any entered callback.
        // Closing precedes invalidation, so old-generation reads cannot reopen.
        entry.state=State::Closing;entry.heartbeat.retire();
        // Native handlers do not hold the PI mutex across their work. Their
        // entered scope is independently counted; never permit destruction
        // while one still owns the exact link. Its leave can drain Closing.
        if(entry.native_borrow_generation)return LinkRetirement::Unproven;
        // Link retirement is not canonical control/plant detachment. Preserve
        // this tombstone's global reservation until that exact Context closes.
        if(!entry.reservation.generation)entry={};
        return LinkRetirement::Quiesced;
    }
    // A healthy mutex and a complete table scan establish that no borrower is registered.
    return LinkRetirement::NeverRegistered;
}

LinkRetirement LinkLifetimeRegistry::retire(const LinkToken &token)noexcept
{
    if(!token.pointer || !token.generation)return LinkRetirement::Mismatch;
    if(!mutex_.lock())return LinkRetirement::Unproven;
    const auto outcome=retire_locked(token.pointer,token.generation,true);
    return mutex_.unlock()?outcome:LinkRetirement::Unproven;
}

LinkRetirement LinkLifetimeRegistry::retire_instance(const void *pointer)noexcept
{
    if(!pointer)return LinkRetirement::Mismatch;
    if(!mutex_.lock())return LinkRetirement::Unproven;
    const auto outcome=retire_locked(pointer,0,false);
    return mutex_.unlock()?outcome:LinkRetirement::Unproven;
}

namespace {
alignas(LinkLifetimeRegistry) unsigned char storage[sizeof(LinkLifetimeRegistry)]{};
unsigned char initialization{0};
static_assert(__atomic_always_lock_free(sizeof(initialization),nullptr),"no libatomic");
}
LinkLifetimeRegistry *link_lifetime_registry()noexcept
{
    const auto state=__atomic_load_n(&initialization,__ATOMIC_ACQUIRE);
    if(state==2)return reinterpret_cast<LinkLifetimeRegistry *>(storage);
    if(state!=0)return nullptr;
    unsigned char expected=0;
    if(!__atomic_compare_exchange_n(&initialization,&expected,1,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))return nullptr;
    auto *registry=new(storage) LinkLifetimeRegistry();
    const bool ready=registry->ready();
    __atomic_store_n(&initialization,static_cast<unsigned char>(ready?2:3),__ATOMIC_RELEASE);
    return ready?registry:nullptr;
}
} // namespace gpenmpc_rfly_stream
