#include "ExecutionSessionRegistration.hpp"
#include <new>

namespace gpenmpc_rfly_px4 {
bool nonzero_session_digest(const SessionDigest &digest) noexcept
{ for(auto word:digest){if(word)return true;} return false; }
bool same_session_echo(const SessionEcho &a,const SessionEcho &b) noexcept
{
    return a.host_challenge==b.host_challenge && a.observed_identity==b.observed_identity &&
        a.board_registration_hrt_us==b.board_registration_hrt_us &&
        a.process_session_generation==b.process_session_generation && a.link==b.link &&
        a.configuration_sha256==b.configuration_sha256 && a.identity_semantics==b.identity_semantics;
}
SessionDigest execution_session_digest(const SessionEcho &e) noexcept
{
    gpenmpc_consumption::CanonicalSha256 hash;
    hash.u32(0x52534531); // RSE1 execution-session digest domain
    hash.byte(static_cast<std::uint8_t>(e.identity_semantics));
    hash.u64(e.host_challenge.high);hash.u64(e.host_challenge.low);
    hash.u64(e.observed_identity.uid);hash.byte(e.observed_identity.system);hash.byte(e.observed_identity.component);
    hash.u64(e.board_registration_hrt_us);hash.u64(e.process_session_generation);
    // The process-local registry binds the pointer to a unique generation.
    // Echo and observation checks also compare the complete LinkToken.
    hash.u64(e.link.generation);hash.words(e.configuration_sha256);
    return hash.finish();
}
SessionRegistration ExecutionSessionLedger::reserve(const HostSessionChallenge &challenge,
                                                   std::uint64_t &assigned) noexcept
{
    assigned=0;
    if(!challenge.high && !challenge.low)return SessionRegistration::Invalid;
    if(!mutex_.lock())return SessionRegistration::Unproven;
    SessionRegistration result=SessionRegistration::Registered;
    if(__atomic_load_n(&poisoned_,__ATOMIC_ACQUIRE))result=SessionRegistration::Unproven;
    for(unsigned i=0;result==SessionRegistration::Registered && i<count_;++i)
        if(used_[i]==challenge)result=SessionRegistration::Replay;
    if(result==SessionRegistration::Registered && (count_==capacity || generation_==UINT64_MAX))
        result=SessionRegistration::Full;
    if(result==SessionRegistration::Registered){used_[count_++]=challenge;assigned=++generation_;}
    if(!mutex_.unlock()){
        // Assigned challenge remains burned if the unlock outcome is unknown.
        // Never advertise successful registration from an unproven mutex exit.
        __atomic_store_n(&poisoned_,true,__ATOMIC_RELEASE);
        assigned=0;return SessionRegistration::Unproven;
    }
    return result;
}
ExecutionSessionLedger *execution_session_ledger() noexcept
{
    alignas(ExecutionSessionLedger) static unsigned char storage[sizeof(ExecutionSessionLedger)];
    static unsigned char state=0; // 0 empty, 1 constructing, 2 ready, 3 failed
    static_assert(__atomic_always_lock_free(sizeof(state),nullptr),"no libatomic singleton");
    auto value=__atomic_load_n(&state,__ATOMIC_ACQUIRE);
    if(value==2)return reinterpret_cast<ExecutionSessionLedger *>(storage);
    if(value!=0)return nullptr;
    unsigned char expected=0;
    if(!__atomic_compare_exchange_n(&state,&expected,1,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))return nullptr;
    auto *ledger=new(storage) ExecutionSessionLedger;
    const bool ready=ledger->ready();
    __atomic_store_n(&state,static_cast<unsigned char>(ready?2:3),__ATOMIC_RELEASE);
    return ready?ledger:nullptr;
}
} // namespace gpenmpc_rfly_px4
