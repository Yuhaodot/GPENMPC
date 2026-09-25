#pragma once
// Installs paired numerical/reference candidates after backend publication.
// One serial owner must keep both stores and their workspace stable. Both pure
// validators pass before the noexcept local copies; publication is irreversible.
// The caller handles IO, authority, age checks and fault/reset recovery.
#include "CanonicalLocalInnerStateStore.hpp"
#include "CanonicalReferenceStateStore.hpp"

namespace gpenmpc_joint_math {
enum class Failure:std::uint8_t {None,BackendEvent,StatePair,NumericReceipt,ReferenceReceipt,Consumption,UnexpectedInstall};
struct BackendPublication {
    std::uint64_t source_timestamp_ns{},source_generation{},output_generation{},original_publication_us{};
    float actual_control16[16]{};
    bool output_published{};
};
struct Diagnostics {
    std::uint64_t attempts{},published_event_reports{},unique_publications_reported{},duplicate_event_reports{},
        regressed_or_unidentified_reports{},joint_installs{},numeric_installs{},reference_installs{},unexpected_partial_installs{};
    BackendPublication last_report{},last_unique_publication{};
    Failure failure{Failure::None};
};
class CanonicalJointStateInstaller final {
public:
    using Numeric=gpenmpc_local_math::CanonicalLocalInnerStateStore;
    using Reference=gpenmpc_reference_math::CanonicalReferenceStateStore;
    CanonicalJointStateInstaller(Numeric &numeric,Reference &reference)noexcept:numeric_(numeric),reference_(reference){}
    CanonicalJointStateInstaller(const CanonicalJointStateInstaller&)=delete;
    CanonicalJointStateInstaller&operator=(const CanonicalJointStateInstaller&)=delete;
    // Populate BackendPublication from the publisher's completed operation.
    bool validate_publication(const BackendPublication &event,const gpenmpc_local_math::PublicationReceipt &numeric,
        const gpenmpc_reference_math::PublicationReceipt &reference)const noexcept{
        return diagnostics_.failure==Failure::None&&validation_failure(event,numeric,reference)==Failure::None;
    }
    bool install_after_publication(const BackendPublication &event,const gpenmpc_local_math::PublicationReceipt &numeric,
        const gpenmpc_reference_math::PublicationReceipt &reference)noexcept{
        ++diagnostics_.attempts;record(event);
        if(diagnostics_.failure!=Failure::None)return false;
        const Failure reason=validation_failure(event,numeric,reference);
        if(reason!=Failure::None)return fail(reason);
        // Under the serial-owner contract these independent installs share stable preconditions.
        const bool numeric_ok=numeric_.install_after_publication(numeric);
        if(numeric_ok)++diagnostics_.numeric_installs;
        if(!numeric_ok)return fail(Failure::UnexpectedInstall);
        const bool reference_ok=reference_.install_after_publication(reference);
        if(reference_ok)++diagnostics_.reference_installs;
        if(!reference_ok){++diagnostics_.unexpected_partial_installs;return fail(Failure::UnexpectedInstall);}
        ++diagnostics_.joint_installs;return true;
    }
    const Diagnostics &diagnostics()const noexcept{return diagnostics_;}
private:
    Failure validation_failure(const BackendPublication&e,const gpenmpc_local_math::PublicationReceipt&n,
        const gpenmpc_reference_math::PublicationReceipt&r)const noexcept{
        if(!e.output_published||!e.source_timestamp_ns||!e.source_generation||!e.output_generation||!e.original_publication_us||
           e.source_timestamp_ns!=n.source_timestamp_ns||e.source_timestamp_ns!=r.source_timestamp_ns||
           e.source_generation!=n.source_generation||e.source_generation!=r.source_generation||
           e.output_generation!=n.output_generation||e.output_generation!=r.output_generation||
           e.original_publication_us!=n.original_publication_us||e.original_publication_us!=r.original_publication_us||
           std::memcmp(e.actual_control16,n.actual_control16,sizeof e.actual_control16))return Failure::BackendEvent;
        if(numeric_.installed_count()!=reference_.installed_count())return Failure::StatePair;
        if(!numeric_.validate_publication(n))return Failure::NumericReceipt;
        if(!reference_.validate_publication(r))return Failure::ReferenceReceipt;
        const auto *nc=numeric_.candidate();const auto *rc=reference_.candidate();
        if(!nc||!rc||std::memcmp(nc->consumed_reference_pvaj,r.actual_reference_pvaj,12*sizeof(double)))return Failure::Consumption;
        // r.actual_reference_pvaj is already checked bitwise against the
        // real query/FromJet candidate by reference.validate_publication.
        return Failure::None;
    }
    void record(const BackendPublication &event)noexcept{
        diagnostics_.last_report=event;
        if(!event.output_published)return;
        ++diagnostics_.published_event_reports;
        const auto&previous=diagnostics_.last_unique_publication;
        if(event.output_generation&&event.original_publication_us&&event.output_generation>previous.output_generation){
            ++diagnostics_.unique_publications_reported;diagnostics_.last_unique_publication=event;
        }else if(event.output_generation&&event.output_generation==previous.output_generation&&
                 event.original_publication_us==previous.original_publication_us){++diagnostics_.duplicate_event_reports;
        }else{++diagnostics_.regressed_or_unidentified_reports;}
    }
    bool fail(Failure reason)noexcept{
        if(diagnostics_.failure==Failure::None)diagnostics_.failure=reason;
        // A late group receipt must not overwrite an earlier concrete
        // numerical/query failure with the less specific publication fault.
        if(numeric_.failure()==gpenmpc_local_math::Failure::None)numeric_.reject_joint_publication();
        if(reference_.failure()==gpenmpc_reference_math::Failure::None)reference_.reject_joint_publication();
        return false;
    }
    Numeric &numeric_;Reference &reference_;Diagnostics diagnostics_{};
};
} // namespace gpenmpc_joint_math
