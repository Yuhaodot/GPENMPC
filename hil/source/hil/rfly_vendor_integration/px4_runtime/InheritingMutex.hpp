#pragma once

#include <pthread.h>
#include <errno.h>

namespace gpenmpc_rfly_px4 {

// Shared by the execution work item and the real MAVLink stream. In particular,
// a preempted lower-priority owner must not be protected by a spin-lock. NuttX's
// default mutex protocol is NONE; request priority inheritance explicitly.
class InheritingMutex final {
public:
    InheritingMutex() noexcept
    {
        pthread_mutexattr_t attr{};
        if(pthread_mutexattr_init(&attr)!=0)return;
        const int protocol=pthread_mutexattr_setprotocol(&attr,PTHREAD_PRIO_INHERIT);
#if defined(__PX4_NUTTX)
        // CONFIG_PTHREAD_MUTEX_ROBUST selects robust mutexes at compile time.
#if defined(CONFIG_PTHREAD_MUTEX_ROBUST)
        const bool robust=true;
#else
        const bool robust=false;
#endif
#else
        const bool robust=pthread_mutexattr_setrobust(&attr,PTHREAD_MUTEX_ROBUST)==0;
#endif
        const int initialized=protocol==0 && robust?pthread_mutex_init(&mutex_,&attr):-1;
        const int destroyed=pthread_mutexattr_destroy(&attr);
        ready_=initialized==0 && destroyed==0;
        if(initialized==0 && !ready_)(void)pthread_mutex_destroy(&mutex_);
    }
    ~InheritingMutex(){if(ready_)(void)pthread_mutex_destroy(&mutex_);}
    InheritingMutex(const InheritingMutex &)=delete;
    InheritingMutex &operator=(const InheritingMutex &)=delete;
    bool lock()noexcept
    {
        if(!ready_ || __atomic_load_n(&failed_,__ATOMIC_ACQUIRE))return false;
        const int rc=pthread_mutex_lock(&mutex_);
        if(rc==0){
            if(__atomic_load_n(&failed_,__ATOMIC_ACQUIRE)){(void)pthread_mutex_unlock(&mutex_);return false;}
            return true;
        }
#if defined(EOWNERDEAD)
        if(rc==EOWNERDEAD){
            // This NuttX pthread_mutex_take may return EOWNERDEAD BEFORE
            // acquiring/recording this caller as owner. Poison is therefore
            // atomic, not protected by an assumed lock. consistent() resets
            // its dead owner/semaphore; do NOT unlock a lock we do not own.
            __atomic_store_n(&failed_,true,__ATOMIC_RELEASE);
            (void)pthread_mutex_consistent(&mutex_);
#if !defined(__PX4_NUTTX)
            // POSIX/Linux does return with the robust mutex acquired.
            (void)pthread_mutex_unlock(&mutex_);
#endif
        }
#endif
        return false;
    }
    bool unlock()noexcept{return ready_ && pthread_mutex_unlock(&mutex_)==0;}
    bool ready()const noexcept{return ready_;}
private:
    pthread_mutex_t mutex_{};
    bool ready_{false};
    static_assert(__atomic_always_lock_free(sizeof(bool),nullptr),"no libatomic on control path");
    bool failed_{false}; // EOWNERDEAD does not imply acquisition on this NuttX
};

} // namespace gpenmpc_rfly_px4
