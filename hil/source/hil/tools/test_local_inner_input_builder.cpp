// Test PX4 topic -> Snapshot -> Slim mapping -> input builder.
// Rotor, reference and environment bindings are host fixtures.
#include "CanonicalLocalInnerInputBuilder.hpp"
#include <cstdio>
#include <limits>
#include <type_traits>
namespace ib=gpenmpc_local_input;
namespace od=gpenmpc_odometry;
static int topic=0,wrong_topic=0;
static unsigned checks=0,passed=0;
static double prototype[36]{};
static ib::Hash configuration_sha{},task_sha{},fixture_sha{};
struct Raw {std::uint64_t generation;double ned[13],mapped[13];};
static Raw rows[76];
static void check(const char*n,bool ok){++checks;passed+=ok;if(!ok)std::fprintf(stderr,"FAIL %s\n",n);}
static bool take(FILE*f,void*p,std::size_t s,std::size_t n){return std::fread(p,s,n,f)==n;}
static bool bits(const void*a,const void*b,std::size_t n){return std::memcmp(a,b,n)==0;}
static bool hash(const wchar_t*s,ib::Hash&out){for(unsigned j=0;j<64;++j){unsigned v;
    if(s[j]>=L'0'&&s[j]<=L'9')v=unsigned(s[j]-L'0');else if(s[j]>=L'A'&&s[j]<=L'F')v=unsigned(s[j]-L'A'+10);
    else if(s[j]>=L'a'&&s[j]<=L'f')v=unsigned(s[j]-L'a'+10);else return false;
    out[j/8]=(out[j/8]<<4)|v;}return s[64]==0;}
static bool load(const wchar_t*mapped,const wchar_t*inner){
    FILE*f=_wfopen(mapped,L"rb");if(!f)return false;std::uint32_t n=0;
    if(!take(f,&n,4,1)||n!=76)return false;
    for(auto&r:rows)if(!take(f,&r.generation,8,1)||!take(f,r.ned,8,13)||!take(f,r.mapped,8,13))return false;
    const bool eof=std::fgetc(f)==EOF;std::fclose(f);if(!eof)return false;
    f=_wfopen(inner,L"rb");if(!f)return false;char magic[4];
    const bool ok=take(f,magic,1,4)&&!std::memcmp(magic,"LCI1",4)&&take(f,&n,4,1)&&n==60&&take(f,prototype,8,36);
    std::fclose(f);return ok;
}
static ib::Identity identity(){return {0x1122334455667788ULL,42,1,1};}
static od::Configuration config(){od::Configuration c{};c.identity=identity();c.vehicle_odometry_topic=&topic;
    c.coordinates=od::Coordinates::ExplicitTranslatedNed;c.sample_max_age_us=5000;return c;} // Fixture freshness bound.
static vehicle_odometry_s message(const Raw&r,std::uint64_t sample=1000000){vehicle_odometry_s m{};
    m.timestamp_sample=sample;m.timestamp=sample+100;m.pose_frame=m.POSE_FRAME_NED;m.velocity_frame=m.VELOCITY_FRAME_NED;
    for(unsigned j=0;j<3;++j){m.position[j]=static_cast<float>(r.ned[j]);m.velocity[j]=static_cast<float>(r.ned[j+3]);m.angular_velocity[j]=static_cast<float>(r.ned[j+10]);}
    for(unsigned j=0;j<4;++j)m.q[j]=static_cast<float>(r.ned[j+6]);
    m.position_variance[0]=std::numeric_limits<float>::quiet_NaN();return m;
}
static bool capture(od::AtomicOdometryAdapter&a,const Raw&r,od::Snapshot&s,std::uint64_t t=1000000,std::uint32_t generation=0){
    const auto m=message(r,t);return a.ingest(m,generation?generation:static_cast<std::uint32_t>(r.generation),t+200,t+300,identity(),&topic,0,s);}
struct Bindings {
    ib::Context context;ib::BoundRotorLag rotor;ib::BoundReference reference;ib::BoundPayload payload;ib::BoundWind wind;ib::ExplicitInitialInterval initial;
};
static Bindings bindings(const od::Snapshot&s){Bindings b{};ib::SnapshotKey key{};ib::snapshot_key(s,key);
    b.context.observed_session=identity();b.context.task_sha256=b.context.reference_asset_sha256=task_sha;b.context.configuration_sha256=configuration_sha;
    b.context.explicit_leg=static_cast<std::uint64_t>(prototype[35]);b.context.step_kind=ib::StepKind::FirstOfExplicitLeg;
    b.rotor.source=b.reference.source=b.payload.source=b.wind.source=key;
    b.rotor.value_kind=ib::RotorValueKind::OriginalPlantLagState;b.rotor.verified_association_receipt_sha256=fixture_sha;
    auto&r=b.rotor.original_observation;for(unsigned j=0;j<6;++j)r.observed_thrust_n[j]=prototype[13+j];
    r.dll_generation=9;r.dll_session=11;r.original_host_receive_ns=7000000000ULL;r.original_board_ingress_us=s.estimator().board_rx_us;
    r.original_sim_time_s=1.25;r.original_observation_sha=fixture_sha; // explicit synthetic clock association in this HOST test only
    auto&ref=b.reference.candidate;ref.leg=b.context.explicit_leg;ref.window_generation=7;ref.candidate_token=13;ref.original_source_generation=s.estimator().generation;
    for(unsigned j=0;j<3;++j){ref.position_m[j]=prototype[19+j];ref.velocity_mps[j]=prototype[22+j];ref.acceleration_mps2[j]=prototype[25+j];ref.jerk_mps3[j]=prototype[28+j];}
    ref.phase_before_s=1;ref.phase_after_s=1.009;b.reference.task_sha256=b.reference.asset_sha256=task_sha;b.reference.configuration_sha256=configuration_sha;b.reference.candidate_evidence_sha256=fixture_sha;
    b.payload.task_sha256=task_sha;b.payload.original_schedule_evidence_sha256=fixture_sha;b.payload.original_schedule_generation=1;b.payload.payload_kg=prototype[31];
    b.wind.original_estimate_evidence_sha256=fixture_sha;b.wind.original_estimate_generation=3;b.wind.estimate_xy_mps={prototype[32],prototype[33]};
    b.initial.configuration_sha256=configuration_sha;b.initial.original_configuration_receipt_sha256=fixture_sha;b.initial.leg=b.context.explicit_leg;b.initial.configured_dt_s=prototype[34];return b;
}
static bool build(const od::Snapshot&s,const Bindings&b,ib::Result&out){return ib::build(s,b.context,&b.rotor,&b.reference,&b.payload,&b.wind,
    b.context.step_kind==ib::StepKind::FirstOfExplicitLeg?&b.initial:nullptr,out);}
static bool empty_output(const ib::Result&r){if(r.assembled||r.tags2[0]||r.tags2[1])return false;for(double v:r.input36)if(!std::isnan(v))return false;return true;}
struct Subscription {
    vehicle_odometry_s m{};bool available=true;unsigned updates{},gets{},generation=17;
    bool update(void*out){++updates;if(!available)return false;std::memcpy(out,&m,sizeof m);available=false;return true;}
    unsigned get_last_generation(){++gets;return generation;}const void*get_topic()const{return &topic;}std::uint8_t get_instance()const{return 0;}
};
int wmain(int argc,wchar_t**argv){
    if(argc!=7||!hash(argv[4],configuration_sha)||!hash(argv[5],task_sha)||!hash(argv[6],fixture_sha)||!load(argv[1],argv[2]))return 2;
    static_assert(!std::is_constructible<od::Snapshot,vehicle_odometry_s>::value,"raw message cannot bypass private snapshot factory");
    static_assert(std::is_same<decltype(vehicle_odometry_s::timestamp_sample),std::uint64_t>::value,"use actual generated topic timestamp type");
    unsigned exact=0,exact_other=0,valid=0;double q_error=0;ib::Result output{};
    for(const auto&r:rows){od::AtomicOdometryAdapter factory(config());od::Snapshot snapshot;
        const bool captured=capture(factory,r,snapshot);check("actual PX4 topic to private AtomicOdometryAdapter snapshot",captured);
        const auto original=snapshot;auto b=bindings(snapshot);const auto saved=b;
        const bool ok=build(snapshot,b,output);valid+=ok;check("all bound input fields accepted without authority",ok&&!output.board_authority&&!output.external_clock_association_proven_here);
        bool other=true;for(unsigned j=0;j<13;++j){if(j<6||j>=10)other=other&&bits(output.input36+j,r.mapped+j,8);
            else{const double e=std::fabs(output.input36[j]-r.mapped[j]);if(e>q_error)q_error=e;}}
        exact+=bits(output.input36,r.mapped,13*8);exact_other+=other;
        check("entire remaining input is copied unchanged from explicit original fixtures",bits(output.input36+13,prototype+13,23*8));
        check("sample tags retain original uint64 HRT and uORB generation",output.tags2[0]==snapshot.estimator().timestamp_sample_us*1000&&output.tags2[1]==r.generation&&
            output.source.sample_delta_us==0&&output.interval_basis==ib::IntervalBasis::ExplicitConfiguredLegInitial);
        check("builder is pure and does not mutate raw snapshot or external bindings",bits(&snapshot.raw(),&original.raw(),sizeof(vehicle_odometry_s))&&bits(&b,&saved,sizeof b));
    }
    check("76 mapped x13 preserve nonquaternion bits and prior MATLAB quaternion tolerance",valid==76&&exact_other==76&&q_error<=1e-12);
    od::AtomicOdometryAdapter factory(config());od::Snapshot s;bool ok=capture(factory,rows[0],s,1000000,11);auto b=bindings(s);
    for(unsigned kind=0;kind<4;++kind){const bool accepted=ib::build(s,b.context,kind==0?nullptr:&b.rotor,kind==1?nullptr:&b.reference,
        kind==2?nullptr:&b.payload,kind==3?nullptr:&b.wind,&b.initial,output);
        check("missing external source data is absent not zero or synthetic",!accepted&&empty_output(output));}
    {od::Snapshot invalid;check("default private Snapshot remains invalid",!build(invalid,b,output)&&output.failure==ib::Failure::Snapshot&&empty_output(output));}
    {check("first source does not invent a 10ms sample interval",!ib::build(s,b.context,&b.rotor,&b.reference,&b.payload,&b.wind,nullptr,output)&&
        output.failure==ib::Failure::MissingInitialInterval&&empty_output(output));}
    {auto c=b;c.initial.configured_dt_s=0.009;check("explicit nondefault initial interval is used with its own provenance",build(s,c,output)&&
        output.input36[34]==0.009&&output.source.sample_delta_us==0&&output.initial_interval_configuration_receipt_sha256==fixture_sha);}
    for(unsigned kind=0;kind<9;++kind){auto c=b;
        if(kind==0)++c.rotor.source.identity.uid;if(kind==1)++c.reference.source.identity.boot_generation;
        if(kind==2)++c.payload.source.sample_us;if(kind==3)++c.wind.source.original_receipt_us;
        if(kind==4)++c.rotor.source.source_generation;if(kind==5)c.reference.source.state_and_origin_sha256[0]^=1;
        if(kind==6)++c.payload.source.reset_counter;if(kind==7)++c.wind.source.sample_delta_us;if(kind==8)++c.wind.source.generation_delta;
        check("changed source binding rejected before input construction",!build(s,c,output)&&output.failure==ib::Failure::SourceBinding&&empty_output(output));}
    for(unsigned kind=0;kind<11;++kind){auto c=b;auto&r=c.rotor.original_observation;
        if(kind==0)c.rotor.value_kind=ib::RotorValueKind::ActuatorCommand;
        if(kind==1)c.rotor.verified_association_receipt_sha256={};if(kind==2)r.original_observation_sha={};
        if(kind==3)r.dll_generation=0;if(kind==4)r.dll_session=0;if(kind==5)r.original_host_receive_ns=0;
        if(kind==6)r.original_board_ingress_us=0;if(kind==7)r.original_sim_time_s=std::numeric_limits<double>::quiet_NaN();
        if(kind==8)r.observed_thrust_n[2]=std::numeric_limits<double>::infinity();if(kind==9)r.observed_thrust_n[0]=-1;
        if(kind==10)c.rotor.value_kind=ib::RotorValueKind::Unspecified;
        check("commands and unbound or invalid rotor data are not lag-state observations",!build(s,c,output)&&output.failure==ib::Failure::Rotor&&empty_output(output));}
    for(unsigned kind=0;kind<10;++kind){auto c=b;
        if(kind==0)c.reference.task_sha256[0]^=1;if(kind==1)c.reference.configuration_sha256[0]^=1;if(kind==2)c.reference.asset_sha256[0]^=1;
        if(kind==3)++c.reference.candidate.leg;if(kind==4)c.reference.candidate.candidate_token=0;if(kind==5)c.reference.candidate.window_generation=0;
        if(kind==6)++c.reference.candidate.original_source_generation;if(kind==7)c.reference.candidate.jerk_mps3[0]=std::numeric_limits<double>::quiet_NaN();
        if(kind==8)c.reference.candidate.phase_after_s=0;if(kind==9)c.reference.candidate_evidence_sha256={};
        check("reference identity all four derivative sources and candidate metadata checked",!build(s,c,output)&&output.failure==ib::Failure::Reference&&empty_output(output));}
    for(unsigned kind=0;kind<5;++kind){auto c=b;if(kind==0)c.payload.payload_kg=-1;if(kind==1)c.payload.payload_kg=std::numeric_limits<double>::quiet_NaN();
        if(kind==2)c.payload.original_schedule_generation=0;if(kind==3)c.payload.original_schedule_evidence_sha256={};if(kind==4)c.payload.task_sha256[0]^=1;
        check("payload requires original task schedule binding",!build(s,c,output)&&output.failure==ib::Failure::Payload&&empty_output(output));}
    for(unsigned kind=0;kind<3;++kind){auto c=b;if(kind==0)c.wind.estimate_xy_mps[1]=std::numeric_limits<double>::quiet_NaN();
        if(kind==1)c.wind.original_estimate_generation=0;if(kind==2)c.wind.original_estimate_evidence_sha256={};
        check("wind estimate requires explicit original evidence",!build(s,c,output)&&output.failure==ib::Failure::Wind&&empty_output(output));}
    for(unsigned kind=0;kind<7;++kind){auto c=b;if(kind==0)c.initial.configured_dt_s=0;if(kind==1)c.initial.configured_dt_s=.001;
        if(kind==2)c.initial.configured_dt_s=.011;if(kind==3)c.initial.configured_dt_s=std::numeric_limits<double>::quiet_NaN();
        if(kind==4)c.initial.original_configuration_receipt_sha256={};if(kind==5)++c.initial.leg;if(kind==6)c.initial.configuration_sha256[0]^=1;
        check("initial interval must retain finite canonical domain and declared source",!build(s,c,output)&&output.failure==ib::Failure::Interval&&empty_output(output));}
    {auto c=b;c.context.step_kind=ib::StepKind::SubsequentObservedSource;
        check("first zero actual-delta cannot be treated as measured subsequent dt",!build(s,c,output)&&output.failure==ib::Failure::Interval&&empty_output(output));}
    ok=ok&&capture(factory,rows[0],s,1009000,17);b=bindings(s);b.context.step_kind=ib::StepKind::SubsequentObservedSource;
    check("actual 9ms and generation jump6 retain original source values",ok&&build(s,b,output)&&output.source.sample_delta_us==9000&&
        output.source.generation_delta==6&&output.input36[34]==9000.0*1e-6&&output.tags2[0]==1009000000ULL&&output.tags2[1]==17&&
        output.interval_basis==ib::IntervalBasis::MeasuredSnapshotDelta);
    check("subsequent source may not override measured delta with initial interval",!ib::build(s,b.context,&b.rotor,&b.reference,&b.payload,&b.wind,&b.initial,output)&&output.failure==ib::Failure::Interval);
    {auto c=config();c.task_origin_ned_m={10,20,30};od::AtomicOdometryAdapter a(c);auto m=message(rows[0]);m.position[0]=12.5f;m.position[1]=-2.f;m.position[2]=34.f;
        od::Snapshot translated;ok=a.ingest(m,8,1000200,1000300,identity(),&topic,0,translated);auto bound=bindings(translated);
        check("exact explicit origin used once then unchanged canonical NED mapping",ok&&build(translated,bound,output)&&output.input36[0]==2.5&&output.input36[1]==-22&&output.input36[2]==-4&&translated.raw().position[0]==12.5f);}
    for(unsigned kind=0;kind<5;++kind){od::AtomicOdometryAdapter a(config());auto m=message(rows[0]);const void*t=&topic;std::uint8_t instance=0;
        if(kind==0)m.pose_frame=2;if(kind==1)m.q[0]=2;if(kind==2)m.timestamp_sample=m.timestamp+1;if(kind==3)t=&wrong_topic;if(kind==4)instance=1;
        od::Snapshot rejected;const bool accepted=a.ingest(m,9,1000200,1000300,identity(),t,instance,rejected);
        check("real factory rejects frame quaternion clock topic and instance without private bypass",!accepted&&!build(rejected,b,output)&&output.failure==ib::Failure::Snapshot&&empty_output(output));}
    {od::AtomicOdometryAdapter a(config());Subscription sub;sub.m=message(rows[0]);unsigned clock_calls=0;od::Snapshot polled;
        const auto clock=[&]()noexcept{++clock_calls;return 1000200ULL;};
        ok=od::poll(sub,clock,identity(),a,polled)==od::PollResult::Accepted;auto bound=bindings(polled);
        check("production poll update/get_last_generation/originalHRT path builds",ok&&build(polled,bound,output)&&sub.updates==1&&sub.gets==1&&clock_calls==1&&output.source.original_receipt_us==1000200);
        ok=od::poll(sub,clock,identity(),a,polled)==od::PollResult::NoUpdate;
        check("NoUpdate reads no new clock and produces no usable input",ok&&sub.gets==1&&clock_calls==1&&!build(polled,bound,output)&&empty_output(output));}
    {od::AtomicOdometryAdapter a(config());od::Snapshot large;const auto t=UINT64_MAX/1000+1;
        ok=capture(a,rows[0],large,t,2);auto bound=bindings(large);
        check("uint64 us-to-ns overflow rejected no float timestamp conversion",ok&&!build(large,bound,output)&&output.failure==ib::Failure::TimestampOverflow&&empty_output(output));}
    FILE*out=_wfopen(argv[3],L"w");if(!out)return 3;
    std::fprintf(out,"{\"pass\":%s,\"checks\":%u,\"passed\":%u,\"actual_generated_topic_rows\":76,\"accepted_builder_rows\":%u,\"x13_bit_exact_rows\":%u,\"nonquaternion_bit_exact_rows\":%u,\"maximum_quaternion_difference\":%.17g,\"first_interval_read_from_immutable_fixture\":%.17g,\"real_private_factory_used\":true,\"real_subscription_runtime_used\":false,\"external_rotor_reference_payload_wind_are_host_fixtures\":true,\"rotor_clock_provider_implemented\":false,\"COM_UDP_kernel_GP_solver_plant_board_actions\":0}\n",
        checks==passed?"true":"false",checks,passed,valid,exact,exact_other,q_error,prototype[34]);
    std::fclose(out);std::printf("LOCAL_INNER_INPUT_BUILDER %u/%u, private snapshots76; qdiff %.17g\n",passed,checks,q_error);
    return checks==passed?0:4;
}
