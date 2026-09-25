#include "mavlink_main.h"
#include "CanonicalApplicationOwner.hpp"
#include "ConfiguredBoardIdentity.hpp"
#include "../application_parameters/CanonicalApplicationParameters.hpp"
#include "../px4_wire/CommittedFeedbackWire.hpp"
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>

// Implemented in the derived MAVLink TU under its actual instance mutex. A
// get_instance() result is not used as an unprotected long-lived raw pointer.
using GPENMPCLockedMavlinkVisitor=bool (*)(Mavlink &,void *);
extern bool gpenmpc_visit_locked_mavlink_device(const char *,GPENMPCLockedMavlinkVisitor,void *) noexcept;

namespace {
using namespace gpenmpc_rfly_px4;
CanonicalApplicationOwner *owner{};
InheritingMutex entry_mutex{};
// NSH caller stacks must not contain Context/Io or a multi-KiB raw receipt.
ApplicationObservation observation{};
gpenmpc_feedback_wire::Bytes evidence_wire{};
RegisteredContextConfiguration profile{};
HostSessionChallenge challenge{};
SessionEcho echo{};
ApplicationResult preparation{ApplicationResult::Unavailable};
gpenmpc_portable::Array<double,3> requested_origin{};
std::uint8_t host_system{},host_component{};

bool hex_digit(char c,unsigned &v) noexcept
{
    if(c>='0'&&c<='9'){v=unsigned(c-'0');return true;}
    if(c>='a'&&c<='f'){v=unsigned(c-'a'+10);return true;}
    if(c>='A'&&c<='F'){v=unsigned(c-'A'+10);return true;}return false;
}
bool hex64(const char *text,std::uint64_t &out) noexcept
{
    out=0;for(unsigned k=0;k<16;++k){unsigned v{};if(!hex_digit(text[k],v))return false;out=(out<<4)|v;}return true;
}
bool digest(const char *text,SessionDigest &out) noexcept
{
    if(!text||std::strlen(text)!=64)return false;
    out={};
    for(unsigned k=0;k<64;++k){unsigned v{};if(!hex_digit(text[k],v))return false;out[k/8]=(out[k/8]<<4)|v;}
    return nonzero_session_digest(out);
}
bool byte(const char *text,std::uint8_t &out) noexcept
{
    if(!text||!*text)return false;
    char *end{};errno=0;const long v=std::strtol(text,&end,10);
    if(errno||!end||*end||v<1||v>255)return false;
    out=static_cast<std::uint8_t>(v);return true;
}
bool real(const char *text,double &out) noexcept
{
    if(!text||!*text)return false;
    char *end{};errno=0;out=std::strtod(text,&end);
    return !errno&&end&&!*end&&std::isfinite(out);
}
int retained_evidence(const char *which,gpenmpc_feedback_wire::Bytes &out) noexcept
{
    // This is a post-release audit read, not an active outbox consumer. Keep
    // both original raw slots and all their original times/dispositions.
    if(owner||observation.state!=ApplicationState::Retired)return -EBUSY;
    if(!which)return -EINVAL;
    const CommittedFeedback *raw{};
    if(std::strcmp(which,"failed")==0)raw=&observation.failed_raw_feedback;
    else if(std::strcmp(which,"interrupted")==0)raw=&observation.interrupted_raw_feedback;
    else return -EINVAL;
    if(raw->disposition==FeedbackDisposition::Empty)return -ENODATA;
    // In particular, never repair a failed record's times to satisfy a wire
    // validator. An unencodable raw record remains available in memory.
    return gpenmpc_feedback_wire::encode(*raw,out)?0:-ERANGE;
}
int print_retained_raw(const char *which,const CommittedFeedback &raw) noexcept
{
    // Diagnostic domain with binary64/binary32 hex preserving NaN, signed zero
    // and original timestamps.
    bool ok=std::printf("RFLY_RETAINED_RAW slot=%s disposition=%u audit_only=1 control_authority=0\n",which,unsigned(raw.disposition))>=0;
    ok=(std::printf("ticket=")>=0)&&ok;
    for(auto v:raw.snapshot_ticket)ok=(std::printf("%02X",unsigned(v))>=0)&&ok;
    ok=(std::printf("\n")>=0)&&ok;
    const auto &t=raw.token;
    auto u64=[&ok](const char *name,std::uint64_t value){ok=(std::printf("%s=%016llX\n",name,static_cast<unsigned long long>(value))>=0)&&ok;};
    u64("identity.uid",t.identity.uid);u64("identity.boot_generation",t.identity.boot_generation);
    u64("identity.system",t.identity.system);u64("identity.component",t.identity.component);u64("publication_path",static_cast<std::uint8_t>(t.publication_path));
#define GPENMPC_PRINT_TOKEN(name) u64("token." #name,t.name)
    GPENMPC_PRINT_TOKEN(transaction);GPENMPC_PRINT_TOKEN(output_generation);GPENMPC_PRINT_TOKEN(sample_generation);
    GPENMPC_PRINT_TOKEN(timestamp_sample_us);GPENMPC_PRINT_TOKEN(source_generation_delta);GPENMPC_PRINT_TOKEN(state_publication_us);
    GPENMPC_PRINT_TOKEN(state_board_rx_us);GPENMPC_PRINT_TOKEN(control_tick_us);GPENMPC_PRINT_TOKEN(sample_delta_us);
    GPENMPC_PRINT_TOKEN(actual_tick_delta_us);GPENMPC_PRINT_TOKEN(reference_generation);GPENMPC_PRINT_TOKEN(reference_timestamp_us);
    GPENMPC_PRINT_TOKEN(reference_board_rx_us);GPENMPC_PRINT_TOKEN(reference_valid_until_us);GPENMPC_PRINT_TOKEN(outer_generation);
    GPENMPC_PRINT_TOKEN(outer_board_rx_us);GPENMPC_PRINT_TOKEN(outer_valid_until_us);GPENMPC_PRINT_TOKEN(outer_based_on_sample_generation);
    GPENMPC_PRINT_TOKEN(outer_based_on_timestamp_sample_us);
#undef GPENMPC_PRINT_TOKEN
    auto sha=[&ok](const char *name,const SessionDigest &value){
        ok=(std::printf("%s=",name)>=0)&&ok;
        for(auto v:value)ok=(std::printf("%08lX",static_cast<unsigned long>(v))>=0)&&ok;
        ok=(std::printf("\n")>=0)&&ok;
    };
    sha("token.outer_payload_sha256",t.outer_payload_sha256);sha("token.full_input_sha256",t.full_input_sha256);
    sha("token.kernel_argument_sha256",t.kernel_argument_sha256);sha("token.kernel_source_sha256",t.kernel_source_sha256);
    sha("configuration_payload_sha256",raw.configuration_payload_sha256);sha("approved_parameter_sha256",raw.approved_parameter_sha256);
    sha("matlab_extraction_source_sha256",raw.matlab_extraction_source_sha256);sha("generated_arm_source_sha256",raw.generated_arm_source_sha256);
    sha("wrapper_matlab_source_sha256",raw.wrapper_matlab_source_sha256);
    for(std::size_t k=0;k<raw.actual61.size();++k){std::uint64_t bits{};std::memcpy(&bits,&raw.actual61[k],sizeof(bits));
        ok=(std::printf("actual61_bits[%u]=%016llX\n",unsigned(k),static_cast<unsigned long long>(bits))>=0)&&ok;}
    for(std::size_t k=0;k<raw.published_control16.size();++k){std::uint32_t bits{};std::memcpy(&bits,&raw.published_control16[k],sizeof(bits));
        ok=(std::printf("control16_bits[%u]=%08lX\n",unsigned(k),static_cast<unsigned long>(bits))>=0)&&ok;}
    u64("kernel_completed_us",raw.kernel_completed_us);u64("publication_us",raw.publication_us);
    u64("commit_completed_us",raw.commit_completed_us);u64("original_valid_until_us",raw.original_valid_until_us);
    if(!ok)return -EIO;
    return std::printf("RFLY_RETAINED_RAW_END persistence_proven=0\n")>=0?0:-EIO;
}
void print_digest(const SessionDigest &d) noexcept{for(auto v:d)std::printf("%08lX",static_cast<unsigned long>(v));}
void print_echo() noexcept
{
    const auto &e=observation.echo;
    std::printf("RFLY_SESSION state=%u challenge=%016llX%016llX uid=%llu system=%u component=%u registration_hrt_us=%llu session_generation=%llu link_generation=%llu semantics=%u config_sha=",
        static_cast<unsigned>(observation.state),static_cast<unsigned long long>(e.host_challenge.high),
        static_cast<unsigned long long>(e.host_challenge.low),static_cast<unsigned long long>(e.observed_identity.uid),
        static_cast<unsigned>(e.observed_identity.system),static_cast<unsigned>(e.observed_identity.component),
        static_cast<unsigned long long>(e.board_registration_hrt_us),static_cast<unsigned long long>(e.process_session_generation),
        static_cast<unsigned long long>(e.link.generation),static_cast<unsigned>(e.identity_semantics));
    print_digest(e.configuration_sha256);std::printf(" session_sha=");print_digest(execution_session_digest(e));
    std::printf(" parameter_sha=");print_digest(kCanonicalApplicationParameterSha);
    std::printf(" registered=%u echo_confirmed=%u declared_isolation=%u declaration_is_sensor_proof=0 session_fault=%u start_requests=%llu stop_requests=%llu\n",
        unsigned(observation.session.registered),unsigned(observation.session.echo_confirmed),
        unsigned(observation.session.human_declaration_bound),unsigned(observation.session.first_fault),
        static_cast<unsigned long long>(observation.start_requests),static_cast<unsigned long long>(observation.stop_requests));
}
bool prepare_locked(Mavlink &actual,void *) noexcept
{
    const int instance=actual.get_instance_id();if(instance<0||instance>255)return false;
    uORB::Subscription odometry{ORB_ID(vehicle_odometry)};vehicle_odometry_s original{};
    if(!odometry.copy(&original))return false;
    const auto now=hrt_absolute_time();
    if(!original.timestamp_sample||original.timestamp_sample>now||
       now-original.timestamp_sample>5000)return false;
    profile={};auto &m=profile.module;auto &e=m.execution;
    m.source.vehicle_odometry_topic=ORB_ID(vehicle_odometry);m.source.instance=0;
    m.source.initial_reset_counter=original.reset_counter;m.source.coordinates=gpenmpc_odometry::Coordinates::ExplicitTranslatedNed;
    m.source.task_origin_ned_m=requested_origin;m.source.sample_max_age_us=5000;
    e.limits={5000,400000,400000,4000,gpenmpc_consumption::PublicationPath::DirectCanonicalMotors};
    e.coordinates=gpenmpc_rfly_execution::Coordinates::StateAndReferenceAreTaskLocalNed;
    e.approved_parameters=canonical_application_parameters();e.approved_parameter_sha256=kCanonicalApplicationParameterSha;
    e.configuration_payload_sha256=gpenmpc_rfly_execution::kCanonicalConfigurationSha;
    e.kernel_source_sha256=gpenmpc_rfly_execution::kGeneratedArmSourceSha;
    e.matlab_extraction_source_sha256=gpenmpc_rfly_execution::kMatlabExtractionSha;
    e.wrapper_matlab_source_sha256=gpenmpc_rfly_execution::kWrapperMatlabSourceSha;
    e.encoding=gpenmpc_rfly_execution::RflyEncoding::OfficialHIL16CtrlsNorm;
    // Runtime resource allocation; the adapter enforces actual dt <= 10000 us.
    m.telemetry_max_age_us=100000;m.poll_period_us=1000;
    m.task_priority=SCHED_PRIORITY_ATTITUDE_CONTROL;m.task_stack_bytes=8192;
    profile.transport={host_system,host_component,1,1,static_cast<std::uint8_t>(instance),5000};
    profile.ingress_topic_instance=0;profile.retained_anchor_capacity=64;profile.native_land_tail_max_us=30000000;
    preparation=owner->prepare(actual,{gpenmpc_board_configuration::expected_uid,1,1},challenge,profile,echo);
    return preparation==ApplicationResult::Ready;
}
int entry(int argc,char *argv[]) noexcept
{
    if(argc<2||!argv||!argv[1])return -EINVAL;
    const auto *command=argv[1];
    if(std::strcmp(command,"prepare")==0){
        if(argc!=9||owner||!argv[2]||std::strcmp(argv[2],"/dev/ttyACM0")!=0||
           !argv[3]||std::strlen(argv[3])!=32||!hex64(argv[3],challenge.high)||!hex64(argv[3]+16,challenge.low)||
           (!challenge.high&&!challenge.low)||!real(argv[4],requested_origin[0])||
           !real(argv[5],requested_origin[1])||!real(argv[6],requested_origin[2])||
           !byte(argv[7],host_system)||!byte(argv[8],host_component))return -EINVAL;
        owner=new CanonicalApplicationOwner;if(!owner)return -ENOMEM;
        preparation=ApplicationResult::Unavailable;
        const bool ok=gpenmpc_visit_locked_mavlink_device(argv[2],prepare_locked,nullptr);
        (void)owner->observe(observation);print_echo();
        std::printf("RFLY_PROFILE sample_age_us=5000 reference_age_us=400000 outer_age_us=400000 kernel_to_publish_us=4000 assembly_us=5000 telemetry_age_us=100000 poll_us=1000 stack_bytes=8192 native_tail_us=30000000 actual_instance=%u original_reset=%u origin=%.17g,%.17g,%.17g\n",
            unsigned(profile.transport.receiver_instance),unsigned(profile.module.source.initial_reset_counter),
            requested_origin[0],requested_origin[1],requested_origin[2]);
        return ok?0:-EACCES;
    }
    if(std::strcmp(command,"evidence")==0){
        if(argc!=3)return -EINVAL;
        const int result=retained_evidence(argv[2],evidence_wire);
        if(result==-ERANGE){
            const auto &raw=std::strcmp(argv[2],"failed")==0?observation.failed_raw_feedback:observation.interrupted_raw_feedback;
            return print_retained_raw(argv[2],raw);
        }
        if(result)return result;
        bool ok=std::printf("RFLY_EVIDENCE slot=%s format=RFC1 bytes=%u audit_only=1 control_authority=0 persistence_proven=0\n",argv[2],unsigned(evidence_wire.size()))>=0;
        for(std::size_t offset=0;offset<evidence_wire.size();offset+=32){
            ok=(std::printf("RFC1_HEX %04u ",unsigned(offset))>=0)&&ok;
            const auto remaining=evidence_wire.size()-offset;
            const auto count=remaining<32?remaining:32;
            for(std::size_t k=0;k<count;++k)ok=(std::printf("%02X",unsigned(evidence_wire[offset+k]))>=0)&&ok;
            ok=(std::printf("\n")>=0)&&ok;
        }
        return ok?0:-EIO;
    }
    if(!owner)return -ENODEV;
    if(std::strcmp(command,"confirm")==0){
        SessionDigest returned{},record{};
        if(argc!=4||!digest(argv[2],returned)||!digest(argv[3],record)||
           !owner->observe(observation)||returned!=execution_session_digest(observation.echo))return -EINVAL;
        SessionPhysicalDeclaration d{};d.source=PhysicalDeclarationSource::OperatorUsbIsolationDeclaration;
        d.physical_setup_record_sha256=record;d.exact_session_sha256=returned;
        d.usb_only=d.props_removed=d.no_actuator_propulsion_power=HumanIsolationClaim::Declared;
        const auto r=owner->confirm(observation.echo,d);(void)owner->observe(observation);print_echo();
        return r==ApplicationResult::Ready?0:-EACCES;
    }
    if(argc!=2)return -EINVAL;
    if(std::strcmp(command,"start")==0)return owner->start()==ApplicationResult::Ready?0:-EACCES;
    if(std::strcmp(command,"stop")==0)return owner->stop()==ApplicationResult::Pending?0:-EIO;
    if(std::strcmp(command,"status")==0){const bool ok=owner->observe(observation);print_echo();return ok?0:-EIO;}
    if(std::strcmp(command,"release")==0){
        const auto r=owner->release(observation);print_echo();
        std::printf("RFLY_RELEASE result=%u context_observed_after_stop=%u disarmed=%u virtual_zero_stream_accepted=%u plant_cache_zero_proven=0 publication_attempts=%llu publication_successes=%llu raw_evidence_available=%u\n",
            unsigned(r),unsigned(observation.context_observed_after_stop),unsigned(observation.context.board_disarmed_observed),
            unsigned(observation.context.virtual_zero_stream_accepted),
            static_cast<unsigned long long>(observation.context.actual_publication_attempts),
            static_cast<unsigned long long>(observation.context.actual_publication_successes),unsigned(observation.raw_evidence_available));
        if(r!=ApplicationResult::Detached)return -EBUSY;
        // The retained observation is process-lifetime evidence until the next
        // explicitly new prepare. No failed controls are retransmitted here.
        delete owner;owner=nullptr;return 0;
    }
    return -EINVAL;
}
}
extern "C" __EXPORT int gpenmpc_rfly_session_main(int argc,char *argv[])
{
    if(!entry_mutex.lock())return -EIO;
    const int r=entry(argc,argv);return entry_mutex.unlock()?r:-EIO;
}
