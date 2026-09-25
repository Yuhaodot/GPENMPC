#include "SharedOutputRegistry.hpp"
#include <new>

namespace gpenmpc_rfly_stream {
namespace {
alignas(SharedOutputRegistry) unsigned char registry_storage[sizeof(SharedOutputRegistry)]{};
// Constant-initialized BSS. 0=uninitialized,1=constructing,2=ready,3=failed.
unsigned char initialization{0};
static_assert(__atomic_always_lock_free(sizeof(initialization),nullptr),"no libatomic");
}

SharedOutputRegistry *shared_output_registry()noexcept
{
    unsigned char state=__atomic_load_n(&initialization,__ATOMIC_ACQUIRE);
    if(state==2)return reinterpret_cast<SharedOutputRegistry *>(registry_storage);
    if(state!=0)return nullptr;
    unsigned char expected=0;
    if(!__atomic_compare_exchange_n(&initialization,&expected,1,false,
                                    __ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))return nullptr;
    auto *registry=new(registry_storage) SharedOutputRegistry();
    const bool ready=registry->ready();
    __atomic_store_n(&initialization,static_cast<unsigned char>(ready?2:3),__ATOMIC_RELEASE);
    return ready?registry:nullptr;
}
} // namespace gpenmpc_rfly_stream
