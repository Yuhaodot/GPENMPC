// Full-inner consumption fixtures using the private74 library and GP DLL.
// HRT, context envelopes, rotor association and publication use host fixtures.
#define wmain retained_abi_test_main
#include "full_inner_abi/test_full_inner_abi.cpp"
#undef wmain
#include "CanonicalFullInnerConsumption.hpp"
#include <limits>
namespace fc=gpenmpc_full_consumption;
namespace ib=gpenmpc_local_input;
namespace od=gpenmpc_odometry;
static vehicle_odometry_s originals_raw[60];
static fc::Hash fixture_sha{};
static int topic_fixture;
static fc::Identity session(){return {0x1122334455667788ULL,42,1,1};}
static od::Configuration source_configuration(){od::Configuration c{};c.identity=session();c.vehicle_odometry_topic=&topic_fixture;
    c.coordinates=od::Coordinates::ExplicitTranslatedNed;c.sample_max_age_us=5000;return c;}
static bool read_raw(const wchar_t*path){FILE*f=_wfopen(path,L"rb");if(!f)return false;char magic[4];uint32_t n{},bytes{};
    bool ok=read(f,magic,4)&&same(magic,"SJC1",4)&&read(f,&n,4)&&n==60&&read(f,&bytes,4)&&bytes==sizeof(vehicle_odometry_s);
    for(unsigned i=0;i<60&&ok;++i){Original saved{};ok=read(f,&originals_raw[i],bytes)&&read(f,&saved,sizeof saved)&&same(&saved,&original[i],sizeof saved);}
    ok=ok&&std::fgetc(f)==EOF;std::fclose(f);return ok;
}
static bool parse_sha(const wchar_t*s){if(std::wcslen(s)!=64)return false;
    for(unsigned i=0;i<64;++i){const auto c=s[i];unsigned v=16;if(c>=L'0'&&c<=L'9')v=c-L'0';if(c>=L'A'&&c<=L'F')v=c-L'A'+10;
        if(v>15)return false;fixture_sha[i/8]=(fixture_sha[i/8]<<4)|v;}return true;
}
static fc::Configuration consumption_configuration(){fc::Configuration c{};c.identity=session();c.numerical=config;gpenmpc_full_inner_identity(&c.build);
    c.limits={5000,100000,400000,5000,gpenmpc_consumption::PublicationPath::DirectCanonicalMotors};return c;}
static ib::Result assemble(unsigned index,const od::Snapshot&s,const gpenmpc_full_inner_reference_candidate&rc){
    ib::SnapshotKey key{};ib::snapshot_key(s,key);ib::Context c{};c.observed_session=session();c.explicit_leg=config.leg_index;
    c.task_sha256=fc::detail::words(config.task_sha256);c.configuration_sha256=fc::detail::words(config.configuration_sha256);
    c.reference_asset_sha256=fc::detail::words(config.reference_asset_sha256);
    c.step_kind=index?ib::StepKind::SubsequentObservedSource:ib::StepKind::FirstOfExplicitLeg;
    ib::BoundRotorLag rotor{};rotor.source=key;rotor.value_kind=ib::RotorValueKind::OriginalPlantLagState;
    rotor.verified_association_receipt_sha256=fixture_sha;auto&lag=rotor.original_observation;
    for(unsigned j=0;j<6;++j)lag.observed_thrust_n[j]=original[index].input[13+j];lag.dll_generation=index+1;lag.dll_session=11;
    lag.original_host_receive_ns=7000000000ULL+9000000ULL*index;lag.original_board_ingress_us=s.estimator().board_rx_us;
    lag.original_sim_time_s=1+.009*index;lag.original_observation_sha=fixture_sha;
    ib::BoundReference ref{};ref.source=key;ref.task_sha256=c.task_sha256;ref.configuration_sha256=c.configuration_sha256;
    ref.asset_sha256=c.reference_asset_sha256;ref.candidate_evidence_sha256=fixture_sha;
    auto&r=ref.candidate;r.leg=rc.leg_index;r.window_generation=rc.window_generation;r.candidate_token=rc.candidate_generation;
    r.original_source_generation=rc.source_generation;r.phase_before_s=rc.phase_before_s;r.phase_after_s=rc.phase_after_s;
    for(unsigned j=0;j<3;++j){r.position_m[j]=rc.actual_reference_pvaj[j];r.velocity_mps[j]=rc.actual_reference_pvaj[3+j];
        r.acceleration_mps2[j]=rc.actual_reference_pvaj[6+j];r.jerk_mps3[j]=rc.actual_reference_pvaj[9+j];}
    ib::BoundPayload payload{};payload.source=key;payload.task_sha256=c.task_sha256;payload.original_schedule_evidence_sha256=fixture_sha;
    payload.original_schedule_generation=index+1;payload.payload_kg=original[index].input[31];
    ib::BoundWind wind{};wind.source=key;wind.original_estimate_evidence_sha256=fixture_sha;wind.original_estimate_generation=index+1;
    wind.estimate_xy_mps={original[index].input[32],original[index].input[33]};
    ib::ExplicitInitialInterval initial{};initial.configuration_sha256=c.configuration_sha256;initial.original_configuration_receipt_sha256=fixture_sha;
    initial.leg=c.explicit_leg;initial.configured_dt_s=rows[index].args[9];ib::Result out{};
    check(ib::build(s,c,&rotor,&ref,&payload,&wind,index?nullptr:&initial,out),"real input builder uses original source and actual reference candidate");return out;
}
struct Fixture {od::Snapshot snapshot;ib::Result input;gpenmpc_full_inner_candidate candidate;fc::NumericalEvidence evidence;fc::Reference reference;fc::OuterCommand outer;
    uint64_t start,completed,publication,validation;};
static Fixture retained[60];
static bool begin(fc::CanonicalFullInnerConsumption&b,const Fixture&f,fc::StatefulToken&t){
    return b.begin(f.snapshot,f.input,f.candidate,f.evidence,f.reference,f.outer,f.start,f.completed,t);}
static bool publish(fc::CanonicalFullInnerConsumption&b,const Fixture&f,const fc::StatefulToken&t,fc::Receipt&r){return b.commit(t,f.candidate.control16,f.publication,f.validation,r);}
static bool replace_private_source(Fixture&f,vehicle_odometry_s raw,uint32_t generation){
    auto cfg=source_configuration();cfg.initial_reset_counter=raw.reset_counter;od::AtomicOdometryAdapter adapter(cfg);
    if(!adapter.ingest(raw,generation,raw.timestamp_sample+200,raw.timestamp_sample+300,session(),&topic_fixture,0,f.snapshot))return false;
    ib::snapshot_key(f.snapshot,f.input.source);f.input.tags2[0]=raw.timestamp_sample*1000;f.input.tags2[1]=generation;
    f.candidate.original_tags2[0]=f.input.tags2[0];f.candidate.original_tags2[1]=generation;
    f.evidence.original_reference_query.source_timestamp_ns=f.input.tags2[0];f.evidence.original_reference_query.source_generation=generation;
    return true;
}
static bool prepare_fixture(Owner&o,od::AtomicOdometryAdapter&a,unsigned i,Fixture&f){
    const auto&raw=originals_raw[i];const auto time=raw.timestamp_sample;
    if(!a.ingest(raw,static_cast<uint32_t>(original[i].tags[1]),time+200,time+300,session(),&topic_fixture,0,f.snapshot))return false;
    auto q=reference_input(i);
    // New actual fixture context events carry fresh generations because their
    // pvaj and target4 change. 
    q.reference_generation=i+1;q.outer_generation=i+1;
    gpenmpc_full_inner_state before{};gpenmpc_full_inner_reference_candidate ref{};
    if(gpenmpc_full_inner_copy_state(o.handle,&before)!=RFI_OK||gpenmpc_full_inner_prepare_reference(o.handle,&q,&ref)!=RFI_OK)return false;
    f.evidence.original_prior_committed_state_sha256=fc::detail::words(before.committed_state_sha256);
    f.evidence.original_loaded_window_sha256=fc::detail::words(before.loaded_window_sha256);f.evidence.original_reference_query=q;
    f.input=assemble(i,f.snapshot,ref);if(!f.input.assembled)return false;
    if(gpenmpc_full_inner_prepare_numeric(o.handle,f.input.input36,original[i].tags,&ref,&f.candidate)!=RFI_OK)return false;
    auto&r=f.reference;r.identity=session();r.generation=i+1;r.outer_generation=i+1;r.timestamp_us=time+210;r.board_rx_us=time+250;r.valid_until_us=time+100000;
    for(unsigned j=0;j<3;++j){const double sign=j==2?-1:1;r.p[j]=sign*f.input.input36[19+j];r.v[j]=sign*f.input.input36[22+j];r.a[j]=sign*f.input.input36[25+j];}
    auto&outer=f.outer;outer.identity=session();outer.generation=i+1;outer.based_on_sample_generation=original[i].tags[1];
    outer.based_on_timestamp_sample_us=time;outer.board_rx_us=time+220;outer.valid_until_us=time+400000;outer.payload_sha256=fc::detail::outer_payload_sha(q);
    f.start=time+300;f.completed=time+350;f.publication=time+400;f.validation=time+450;return true;
}
int wmain(int argc,wchar_t**argv){if(argc!=5||!load(argv[1],argv[2])||!read_raw(argv[2])||!parse_sha(argv[4]))return 2;
    HMODULE dll=LoadLibraryExW(argv[3],nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);if(!dll)return 3;
    using Predict=int(*)(const double*,double*);const auto predict=reinterpret_cast<Predict>(GetProcAddress(dll,"gpenmpc_gp256_predict"));if(!predict)return 3;
    Owner owner;check(owner.create(),"actual private74 owner initialized");od::AtomicOdometryAdapter adapter(source_configuration());
    const auto configuration=consumption_configuration();fc::CanonicalFullInnerConsumption binding(configuration);unsigned rows_ok=0,exact=0,gps=0,placeholder_rows=0;
    for(unsigned i=0;i<60;++i){auto&f=retained[i];check(prepare_fixture(owner,adapter,i,f),"actual private source / query / builder / C candidate");
        if(!f.input.assembled)break;
        const bool same_math=same(f.input.input36,original[i].input,288)&&same(f.candidate.kernel61,original[i].kernel,488)&&
            same(f.candidate.control16,original[i].controls,64)&&same(f.candidate.post_state64,original[i].state,512)&&same(f.candidate.request19,original[i].request,152);
        check(same_math,"actual input36 state64 kernel61 control16 request19 same retained math bits");exact+=same_math;
        for(double v:f.candidate.scaffold70)if(std::isnan(v)){++placeholder_rows;break;}
        fc::StatefulToken token{};const bool accepted=begin(binding,f,token);
        if(!accepted)std::fprintf(stderr,"row %u begin fault %u\n",i,static_cast<unsigned>(binding.fault()));
        check(accepted,"RFL2 independently verifies actual prior/full hash and exact context");if(!accepted)break;
        check(token.domain==fc::domain_rfl2&&!token.board_authority&&fc::detail::empty(token.lease_envelope.kernel_argument_sha256)&&
            token.lease_envelope.full_input_sha256!=token.full_inner_input_sha256&&token.original_valid_until_us==f.snapshot.estimator().timestamp_sample_us+5000,
            "new RFL2 lease domain not stateless101; exact conservative original expiry");
        fc::Receipt receipt{};check(publish(binding,f,token,receipt)&&receipt.valid&&!receipt.board_authority&&!receipt.full_numeric_reference_state_installed,
            "mock publication produces a numerical receipt before joint installation");
        auto actual=receipts(f.candidate);actual.e.output_generation=actual.n.output_generation=actual.r.output_generation=token.lease_envelope.output_generation;
        check(commit(owner,actual)==RFI_OK,"real joint state installation occurs separately after numerical receipt");
        gpenmpc_full_inner_state after{};gpenmpc_full_inner_copy_state(owner.handle,&after);
        if(after.prediction_required){double gp[18];++gps;check(predict(f.candidate.request19+1,gp)==0&&same(gp,original[i].gp,144),"actual original GP no recorded-prediction reinjection");
            check(gpenmpc_full_inner_fill_gp(owner.handle,f.candidate.original_tags2,gp)==RFI_OK,"actual GP fills only next pending sample");}
        gpenmpc_full_inner_copy_state(owner.handle,&after);check(same(after.pending70,original[i].pending,560),"next-source closed pending bits exact");++rows_ok;
    }
    check(rows_ok==60&&exact==60&&gps==59&&binding.committed_outputs()==60,"complete actual 60C/59GP continuous chain");
    check(placeholder_rows>0,"actual legal NaN scaffold placeholders were exercised");
    if(rows_ok!=60){FreeLibrary(dll);std::printf("{\"checks\":%u,\"failed\":%u,\"rows\":%u}\n",checks,failed,rows_ok);return 1;}
    // Each input mutation gets a fresh receipt object, never resetting a fault.
    for(unsigned k=0;k<39;++k){Fixture f=retained[0];auto cfg=configuration;fc::StatefulToken t{};
        if(k==0)f.input.source.source_generation++;
        if(k==1)f.input.source.state_and_origin_sha256[0]^=1;
        if(k==2)f.input.input36[0]+=.1;
        if(k==3)f.input.tags2[0]++;
        if(k==4)f.input.tags2[1]++;
        if(k==5)f.input.input36[19]+=.1;
        if(k==6)f.candidate.consumed_reference_pvaj[0]+=.1;
        if(k==7)f.evidence.original_reference_query.dt_s=.008;
        if(k==8)f.evidence.original_reference_query.source_generation++;
        if(k==9)f.evidence.original_reference_query.source_timestamp_ns++;
        if(k==10)f.evidence.original_reference_query.target_outer_f[0]+=.1;
        if(k==11)f.evidence.original_reference_query.target_phase_acceleration+=.1;
        if(k==12)f.evidence.original_prior_committed_state_sha256[0]^=1;
        if(k==13)f.evidence.original_loaded_window_sha256[0]^=1;
        if(k==14)f.candidate.full_inner_input_sha256[0]^=1;
        if(k==15)f.candidate.prior_committed_state_sha256[0]^=1;
        if(k==16)cfg.build.generated_source_set_sha256[0]^=1;
        if(k==17)cfg.numerical.configuration_sha256[0]^=1;
        if(k==18)cfg.numerical.task_sha256[0]^=1;
        if(k==19)f.reference.identity.boot_generation++;
        if(k==20)f.reference.p[2]=-f.reference.p[2];
        if(k==21)f.reference.valid_until_us=f.start-1;
        if(k==22)f.outer.valid_until_us=f.start-1;
        if(k==23)f.start=f.completed=f.snapshot.estimator().timestamp_sample_us+5001;
        if(k==24)f.completed=f.start-1;
        if(k==25)f.input.board_authority=true;
        if(k==26)f.candidate.control_authority=1;
        if(k==27)f.candidate.control16[0]=std::numeric_limits<float>::quiet_NaN();
        if(k==28)f.candidate.kernel61[20]=std::numeric_limits<double>::infinity();
        if(k==29)f.outer.payload_sha256[0]^=1;
        if(k==30)f.input.interval_basis=ib::IntervalBasis::MeasuredSnapshotDelta;
        if(k==31)f.input.initial_interval_configuration_receipt_sha256={};
        if(k==32)f.evidence.original_reference_query.reference_generation++;
        if(k==33)f.evidence.original_reference_query.outer_generation++;
        if(k==34)f.input.reference_candidate_token++;
        if(k==35)f.input.reference_window_generation++;
        if(k==36)f.input.input36[34]=.001;
        if(k==37)f.input.rotor_association_receipt_sha256={};
        if(k==38)f.snapshot=od::Snapshot{};
        fc::CanonicalFullInnerConsumption b(cfg);check(!begin(b,f,t)&&b.fault()!=fc::Fault::None&&b.committed_outputs()==0,"mutated original field rejects before publication receipt");
        const auto first=b.fault();check(!begin(b,retained[0],t)&&b.fault()==first,"original first-fault cannot be washed by good retry");
    }
    for(unsigned k=0;k<10;++k){const auto&f=retained[0];fc::CanonicalFullInnerConsumption b(configuration);fc::StatefulToken t{};fc::Receipt r{};
        check(begin(b,f,t),"negative publication begins genuine candidate");float actual[16];std::memcpy(actual,f.candidate.control16,64);auto pub=f.publication,val=f.validation;
        if(k==0)t.domain++;
        if(k==1)t.prior_committed_state_sha256[0]^=1;
        if(k==2)t.lease_envelope.full_input_sha256[0]^=1;
        if(k==3)t.lease_envelope.kernel_argument_sha256[0]=1;
        if(k==4)t.board_authority=true;
        if(k==5)actual[0]=std::nextafter(actual[0],1.0f);
        if(k==6)pub=f.completed-1;
        if(k==7)val=pub-1;
        if(k==8)val=t.original_valid_until_us+1;
        if(k==9)t.candidate_sha256[0]^=1;
        check(!b.commit(t,actual,pub,val,r)&&!r.valid&&b.committed_outputs()==0&&b.diagnostics().reported_publications==1,
            "bad actual publication stays counted but cannot numerically commit");
        const auto fault=b.fault();check(!b.commit(t,actual,pub,val,r)&&b.fault()==fault&&b.diagnostics().reported_publications==2,"retry side-effect report retained without recovery");
    }
    {const auto&f=retained[0];fc::CanonicalFullInnerConsumption b(configuration);fc::StatefulToken t{};fc::Receipt r{};begin(b,f,t);
        check(!b.note_publication_failure(t,f.publication)&&b.fault()==fc::Fault::PublicationFailed&&b.diagnostics().failed_publication_reports==1&&b.committed_outputs()==0,"failed publisher never commits");
        check(!publish(b,f,t,r)&&b.diagnostics().reported_publications==1&&b.fault()==fc::Fault::PublicationFailed,"late successful report preserved after original publisher failure");}
    {const auto&f=retained[0];fc::CanonicalFullInnerConsumption b(configuration);fc::StatefulToken t{};fc::Receipt r{};begin(b,f,t);
        check(publish(b,f,t,r)&&!publish(b,f,t,r)&&b.committed_outputs()==1&&b.diagnostics().reported_publications==2,"once-only commit counts duplicate physical report");}
    for(unsigned k=0;k<11;++k){fc::CanonicalFullInnerConsumption b(configuration);fc::StatefulToken t{};fc::Receipt r{};begin(b,retained[0],t);publish(b,retained[0],t,r);Fixture f=retained[1];
        if(k==0)f=retained[0];
        if(k==1)f.input.interval_basis=ib::IntervalBasis::ExplicitConfiguredLegInitial;
        if(k==2)f.reference.generation=retained[0].reference.generation;
        if(k==3)f.outer.generation=retained[0].outer.generation;
        if(k==4)f.start=retained[0].start;
        if(k==5)f.reference.board_rx_us=retained[0].reference.board_rx_us;
        if(k==6)f.outer.board_rx_us=retained[0].outer.board_rx_us;
        if(k==7)f.reference.timestamp_us=retained[0].reference.timestamp_us;
        if(k==8)f.evidence.original_reference_query.query_sequence=retained[0].evidence.original_reference_query.query_sequence;
        if(k==9)f.candidate.reference_candidate_generation=retained[0].candidate.reference_candidate_generation;
        if(k==10)f.input.source.sample_delta_us++;
        check(!begin(b,f,t)&&b.committed_outputs()==1,"held/replayed/regressed/reset interval cannot advance committed source");}
    for(unsigned k=0;k<3;++k){fc::CanonicalFullInnerConsumption b(configuration);fc::StatefulToken t{};fc::Receipt r{};
        begin(b,retained[0],t);publish(b,retained[0],t,r);Fixture f=retained[1];auto raw=originals_raw[1];
        if(k==0){raw.timestamp_sample=originals_raw[0].timestamp_sample-1;raw.timestamp=raw.timestamp_sample+100;}
        if(k==1)raw.reset_counter=1;
        if(k==2){raw.timestamp_sample=originals_raw[0].timestamp_sample+10001;raw.timestamp=raw.timestamp_sample+100;}
        check(replace_private_source(f,raw,2),"negative uses independently valid private Snapshot not forged public POD");
        const auto expected=k==0?fc::Fault::SourceRegression:k==1?fc::Fault::Reset:fc::Fault::Interval;
        check(!begin(b,f,t)&&b.fault()==expected&&b.committed_outputs()==1,"real private source regression/reset/too-large committed interval rejects");
    }
    {const auto&f=retained[0];fc::CanonicalFullInnerConsumption b(configuration);fc::StatefulToken t{};fc::Receipt r{};
        check(begin(b,f,t)&&b.commit(t,f.candidate.control16,t.original_valid_until_us,t.original_valid_until_us,r),"inclusive original deadline boundary accepted without renewal");}
    for(unsigned k=0;k<16;++k){auto cfg=configuration;
        if(k==0)cfg.identity.uid=0;if(k==1)cfg.identity.boot_generation=0;if(k==2)cfg.identity.system=0;if(k==3)cfg.identity.component=0;
        if(k==4)cfg.limits.sample_max_age_us=0;if(k==5)cfg.limits.reference_max_age_us=0;if(k==6)cfg.limits.outer_max_age_us=0;if(k==7)cfg.limits.transaction_max_wall_us=0;
        if(k==8)cfg.limits.publication_path=gpenmpc_consumption::PublicationPath::LegacyThrustTorque;
        if(k==9)cfg.numerical.abi_version=0;if(k==10)cfg.numerical.leg_index=0;
        if(k==11)std::memset(cfg.numerical.task_sha256,0,32);if(k==12)std::memset(cfg.numerical.configuration_sha256,0,32);
        if(k==13)std::memset(cfg.numerical.reference_asset_sha256,0,32);if(k==14)std::memset(cfg.build.generated_source_set_sha256,0,32);
        if(k==15)std::memset(cfg.build.facade_source_sha256,0,32);
        fc::CanonicalFullInnerConsumption b(cfg);fc::StatefulToken t{};
        check(b.fault()==fc::Fault::Configuration&&!begin(b,retained[0],t),"unbound identity/source/path/limits configuration never admits");
    }
    {fc::CanonicalFullInnerConsumption b(configuration);fc::StatefulToken t{};begin(b,retained[0],t);check(!begin(b,retained[1],t)&&b.fault()==fc::Fault::Pending,"pending candidate cannot be overwritten");}
    {fc::CanonicalFullInnerConsumption b(configuration);fc::StatefulToken t{};b.revoke();check(!begin(b,retained[0],t)&&b.fault()==fc::Fault::Revoked,"explicit revocation sticky");}
    {fc::CanonicalFullInnerConsumption b(configuration);fc::StatefulToken t{};b.note_exception();check(!begin(b,retained[0],t)&&b.fault()==fc::Fault::Exception,"exception sticky");}
    FreeLibrary(dll);std::printf("{\"checks\":%u,\"failed\":%u,\"actual_C_rows\":%u,\"exact_math_rows\":%u,\"actual_GP_calls\":%u,\"legal_NaN_scaffold_rows\":%u,\"domain\":\"RFL2\",\"legacy_kernel_argument_sha_zero\":true,\"source_context_rotor_association_publication_MOCK\":true,\"board_authority\":false,\"COM\":0}\n",checks,failed,rows_ok,exact,gps,placeholder_rows);
    return failed?1:0;
}
