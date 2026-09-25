#include "CanonicalLocalWireOutbox.hpp"
#include <drivers/drv_hrt.h>
namespace gpenmpc_rfly_px4 {
static_assert(__atomic_always_lock_free(1,nullptr),"actual lock-free byte handoff required");
static_assert(gpenmpc_local_gp_wire::request_bytes<=CanonicalLocalWireOutbox::capacity,"explicit request capacity");
static_assert(gpenmpc_local_snapshot_wire::message_bytes<=CanonicalLocalWireOutbox::capacity,"explicit snapshot capacity");
static_assert(gpenmpc_local_committed_wire::message_bytes==CanonicalLocalWireOutbox::capacity,"explicit committed-state capacity");
CanonicalLocalWireOutbox::CanonicalLocalWireOutbox(const LocalWireConfiguration&c)noexcept:config_(c){
    if(!gpenmpc_local_gp_wire::identity_valid(c.expected_session)||!c.producer_identity||!c.target_system||!c.target_component||
       (!c.gp_request_transport_max_age_us&&!c.snapshot_transport_max_age_us))fail(LocalWireFault::Configuration);
}
bool CanonicalLocalWireOutbox::fail(LocalWireFault f)noexcept{
    std::uint8_t expected=0;__atomic_compare_exchange_n(&fault_,&expected,static_cast<std::uint8_t>(f),false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE);return false;
}
bool CanonicalLocalWireOutbox::enter(std::uint8_t&busy)noexcept{
    std::uint8_t expected=0;return __atomic_compare_exchange_n(&busy,&expected,1,false,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE);
}
void CanonicalLocalWireOutbox::leave(std::uint8_t&busy)noexcept{__atomic_store_n(&busy,0,__ATOMIC_RELEASE);}
LocalWirePublish CanonicalLocalWireOutbox::publish_gp(const gpenmpc_local_gp_wire::RequestBytes&b,const void*producer)noexcept{
    using R=LocalWirePublish;
    if(closed()||fault()!=LocalWireFault::None)return R::Unavailable;
    if(producer!=config_.producer_identity){fail(LocalWireFault::ProducerIdentity);return R::Unavailable;}
    if(!enter(producer_busy_)){fail(LocalWireFault::ProducerConflict);return R::Unavailable;}
    R result=R::Unavailable;
    if(!closed()&&fault()==LocalWireFault::None){
        if(numerical_pending())result=R::Busy;
        else{
            gpenmpc_local_gp_wire::Request q{};
            if(!config_.gp_request_transport_max_age_us||!gpenmpc_local_gp_wire::decode(b,q))fail(LocalWireFault::InvalidMessage);
            else if(!(q.identity==config_.expected_session))fail(LocalWireFault::SessionMismatch);
            else if(config_.gp_request_transport_max_age_us>UINT64_MAX-q.original_publication_us)fail(LocalWireFault::Clock);
            else{
                LocalWireMetadata m{};m.schema=gpenmpc_local_gp_wire::request_schema;m.message_bytes=b.size();
                m.fragment_count=gpenmpc_local_gp_wire::fragment_count;m.generation=q.output_generation;m.source_generation=q.source_generation;
                m.original_source_timestamp_ns=q.source_timestamp_ns;m.original_anchor_us=q.original_publication_us;
                // Owner-frozen TRANSPORT budget; motor publication expiry is
                // retained in wire bytes but is NOT the GP transfer deadline.
                m.original_transport_valid_until_us=q.original_publication_us+config_.gp_request_transport_max_age_us;
                m.message_sha256=gpenmpc_local_gp_wire::digest(b.data(),b.size());result=publish_checked(b.data(),b.size(),m,0);
            }
        }
    }
    if(closed()||fault()!=LocalWireFault::None)result=R::Unavailable;
    leave(producer_busy_);return result;
}
LocalWirePublish CanonicalLocalWireOutbox::publish_snapshot(const gpenmpc_local_snapshot_wire::Bytes&b,const void*producer)noexcept{
    using R=LocalWirePublish;
    if(closed()||fault()!=LocalWireFault::None)return R::Unavailable;
    if(producer!=config_.producer_identity){fail(LocalWireFault::ProducerIdentity);return R::Unavailable;}
    if(!enter(producer_busy_)){fail(LocalWireFault::ProducerConflict);return R::Unavailable;}
    R result=R::Unavailable;
    if(!closed()&&fault()==LocalWireFault::None){
        if(numerical_pending())result=R::Busy;
        else{
            gpenmpc_local_snapshot_wire::Observation q{};
            if(!config_.snapshot_transport_max_age_us||!gpenmpc_local_snapshot_wire::decode(b,q))fail(LocalWireFault::InvalidMessage);
            else if(!(q.source.identity==config_.expected_session))fail(LocalWireFault::SessionMismatch);
            else if(config_.snapshot_transport_max_age_us>UINT64_MAX-q.source.sample_us)fail(LocalWireFault::Clock);
            else{
                LocalWireMetadata m{};m.schema=gpenmpc_local_snapshot_wire::schema;m.message_bytes=b.size();
                m.fragment_count=gpenmpc_local_snapshot_wire::fragment_count;m.generation=q.source.source_generation;m.source_generation=q.source.source_generation;
                // Snapshot time uses HRT microseconds; the GP-only ns field is zero.
                m.original_source_sample_us=q.source.sample_us;m.original_anchor_us=q.source.sample_us;
                m.original_transport_valid_until_us=q.source.sample_us+config_.snapshot_transport_max_age_us;
                m.message_sha256=gpenmpc_local_gp_wire::digest(b.data(),b.size());result=publish_checked(b.data(),b.size(),m,1);
            }
        }
    }
    if(closed()||fault()!=LocalWireFault::None)result=R::Unavailable;
    leave(producer_busy_);return result;
}
LocalWirePublish CanonicalLocalWireOutbox::publish_checked(const std::uint8_t*b,std::size_t n,const LocalWireMetadata&m,unsigned kind)noexcept{
    const auto source_time=kind==1?m.original_source_sample_us:m.original_source_timestamp_ns;
    if(!n||n>capacity||m.message_bytes!=n||m.fragment_count!=(n+118)/119||m.fragment_count>max_fragments||
       !m.generation||!m.source_generation||!source_time||!m.original_anchor_us){fail(LocalWireFault::InvalidMessage);return LocalWirePublish::Unavailable;}
    if(m.generation<=last_generation_[kind]||m.source_generation<=last_source_generation_[kind]||
       source_time<=last_source_time_[kind]){fail(LocalWireFault::Generation);return LocalWirePublish::Unavailable;}
    if(published_==UINT64_MAX){fail(LocalWireFault::CounterOverflow);return LocalWirePublish::Unavailable;}
    const bool history=kind==2&&m.component_history_only;
    auto*body=history?history_bytes_:bytes_;
    std::memcpy(body,b,n);std::memset(body+n,0,capacity-n);
    if(history){history_metadata_=m;history_next_=0;}else{metadata_=m;next_=0;}
    if(gpenmpc_local_gp_wire::digest(body,n)!=m.message_sha256){fail(LocalWireFault::Integrity);return LocalWirePublish::Unavailable;}
    last_generation_[kind]=m.generation;last_source_generation_[kind]=m.source_generation;last_source_time_[kind]=source_time;
    ++published_;__atomic_store_n(history?&history_ready_:&ready_,1,__ATOMIC_RELEASE);return LocalWirePublish::Published;
}
LocalWirePublish CanonicalLocalWireOutbox::publish_committed(const gpenmpc_local_committed_wire::Bytes&b,const void*producer,bool history_only,bool timely_outer_observation)noexcept{
    using R=LocalWirePublish;if(closed()||fault()!=LocalWireFault::None)return R::Unavailable;
    if(producer!=config_.producer_identity){fail(LocalWireFault::ProducerIdentity);return R::Unavailable;}
    if(!enter(producer_busy_)){fail(LocalWireFault::ProducerConflict);return R::Unavailable;}
    R result=R::Unavailable;
    if(!closed()&&fault()==LocalWireFault::None){
        if(history_only?history_pending():pending())result=R::Busy;
        else{
            gpenmpc_local_committed_wire::Observation q{};
            if(!config_.snapshot_transport_max_age_us||!gpenmpc_local_committed_wire::decode(b,q))fail(LocalWireFault::InvalidMessage);
            else if(!(q.identity==config_.expected_session))fail(LocalWireFault::SessionMismatch);
            else if(config_.snapshot_transport_max_age_us>UINT64_MAX-q.original_publication_us)fail(LocalWireFault::Clock);
            else{
                LocalWireMetadata m{};m.schema=gpenmpc_local_committed_wire::schema;m.message_bytes=b.size();
                m.component_history_only=history_only;
                // Component history yields after each fragment. Full-method observations
                // use the bounded RLS-sized slice in take().
                m.yield_after_fragment=history_only&&!timely_outer_observation;
                m.fragment_count=gpenmpc_local_committed_wire::fragment_count;m.generation=q.output_generation;m.source_generation=q.source_generation;
                m.original_source_timestamp_ns=q.source_timestamp_ns;m.original_anchor_us=q.original_publication_us;
                m.original_transport_valid_until_us=q.original_publication_us+config_.snapshot_transport_max_age_us;
                m.message_sha256=gpenmpc_local_gp_wire::digest(b.data(),b.size());result=publish_checked(b.data(),b.size(),m,2);
            }
        }
    }
    if(closed()||fault()!=LocalWireFault::None){result=R::Unavailable;}
    leave(producer_busy_);return result;
}
LocalWireTake CanonicalLocalWireOutbox::take(const void*consumer,LocalWireFragment&out,std::uint64_t now)noexcept{
    using R=LocalWireTake;out={};if(closed())return R::Stopped;
    const auto existing=fault();if(existing!=LocalWireFault::None)return existing==LocalWireFault::Expired?R::Expired:R::Unavailable;
    if(!consumer){fail(LocalWireFault::ConsumerIdentity);return R::Unavailable;}
    if(!enter(consumer_busy_)){fail(LocalWireFault::ConsumerConflict);return R::Unavailable;}
    R result=R::Unavailable;
    if(!closed()&&fault()==LocalWireFault::None){
        if(consumer_&&consumer_!=consumer)fail(LocalWireFault::ConsumerIdentity);
        else{
            consumer_=consumer;
            // The live caller cannot sample HRT before registry locking or
            // the ready acquire: the producer may publish a newer state in
            // that interval. Read actual HRT only after acquiring this stable
            // message. Source/anchor/expiry bytes are never renewed. Explicit
            // nonzero times remain solely for existing deterministic fixtures.
            const bool numerical=numerical_pending();
            const bool message_ready=numerical||history_pending();
            const auto&metadata=numerical?metadata_:history_metadata_;
            const auto*body=numerical?bytes_:history_bytes_;
            auto&next=numerical?next_:history_next_;
            if(message_ready&&!now)now=hrt_absolute_time();
            if(!message_ready)result=R::Empty;
            else if(now<metadata.original_anchor_us)fail(LocalWireFault::Clock);
            else if(now>metadata.original_transport_valid_until_us&&
                !(metadata.component_history_only&&metadata.schema==gpenmpc_local_committed_wire::schema)){
                fail(LocalWireFault::Expired);result=R::Expired;}
            else if(next>=metadata.fragment_count||gpenmpc_local_gp_wire::digest(body,metadata.message_bytes)!=metadata.message_sha256)fail(LocalWireFault::Integrity);
            else if(copied_==UINT64_MAX||completed_==UINT64_MAX)fail(LocalWireFault::CounterOverflow);
            else{
                auto&f=out.fragment;const std::size_t offset=next*119,n=metadata.message_bytes-offset<119?metadata.message_bytes-offset:119;
                f.payload[0]=static_cast<std::uint8_t>((metadata.schema<<4)|next);
                gpenmpc_snapshot_wire::snapshot_detail::put64(f.payload+1,metadata.generation);
                std::memcpy(f.payload+9,body+offset,n);f.length=static_cast<std::uint8_t>(n+9);
                out.original=metadata;
                if(metadata.component_history_only&&!metadata.yield_after_fragment)
                    out.original.yield_after_fragment=(next+1)%gpenmpc_local_snapshot_wire::fragment_count==0;
                out.target_system=config_.target_system;out.target_component=config_.target_component;
                ++copied_;++next;result=R::Fragment;
                if(next==metadata.fragment_count){++completed_;__atomic_store_n(numerical?&ready_:&history_ready_,0,__ATOMIC_RELEASE);}
            }
        }
    }
    if(result==R::Fragment&&(closed()||fault()!=LocalWireFault::None)){
        out={};result=closed()?R::Stopped:fault()==LocalWireFault::Expired?R::Expired:R::Unavailable;
    }
    leave(consumer_busy_);return result;
}
bool CanonicalLocalWireOutbox::audit_copy(LocalWireAudit&out)noexcept{
    out={};if(!closed()&&fault()==LocalWireFault::None)return false;
    if(!enter(producer_busy_))return false;
    if(!enter(consumer_busy_)){leave(producer_busy_);return false;}
    const bool history=!numerical_pending()&&history_pending();
    std::memcpy(out.message,history?history_bytes_:bytes_,sizeof bytes_);out.original=history?history_metadata_:metadata_;out.published_messages=published_;
    out.copied_fragments=copied_;out.fully_copied_messages=completed_;out.next_fragment=history?history_next_:next_;
    out.pending=pending();out.closed=closed();out.first_fault=fault();leave(consumer_busy_);leave(producer_busy_);return true;
}
}
