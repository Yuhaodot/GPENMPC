// Compare implementations using RWI1 references and saved synthetic LCI1 inputs.
// Raw odometry passes through the Snapshot factory; clocks and receipts are mocks.
#include "CanonicalLocalInnerInputBuilder.hpp"
#include "CanonicalJointStateInstaller.hpp"
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_initialize.h"
#include "canonical_gp_standalone_api.h"
#include <windows.h>
#include <cstdio>
#include <new>
#include <limits>
#include <type_traits>
namespace ib=gpenmpc_local_input; namespace od=gpenmpc_odometry;
namespace num=gpenmpc_local_math; namespace ref=gpenmpc_reference_math;
namespace joint=gpenmpc_joint_math;
using Numeric=num::CanonicalLocalInnerStateStore;
using Reference=ref::CanonicalReferenceStateStore;
using Group=joint::CanonicalJointStateInstaller;
using Predict=int(*)(const double*,double*);
static Numeric::Workspace workspace,oracle_workspace;
static struct51_T windows[32];
struct Row {std::uint32_t ids[3];unsigned long long sequence;double phase,args[11],jet[12],expected[41];};
static Row rows[60];static double saved_input[60][36];
static ib::Hash config_sha{},task_sha{},fixture_sha{},reference_sha{};
static unsigned checks{},passed{},source_captures{},main_completed{},main_gp_calls{},oracle_gp_calls{};
static unsigned direct_calls{},all_factory_captures{};static double max_numeric{},max_reference{};
static int topic=0,wrong_topic=0;
alignas(Numeric) static unsigned char numeric_memory[sizeof(Numeric)];
alignas(Reference) static unsigned char reference_memory[sizeof(Reference)];
alignas(Group) static unsigned char group_memory[sizeof(Group)];
alignas(od::AtomicOdometryAdapter) static unsigned char factory_memory[sizeof(od::AtomicOdometryAdapter)];
static Numeric* numeric=nullptr;static Reference*reference=nullptr;static Group*group=nullptr;static od::AtomicOdometryAdapter*factory=nullptr;
static double oracle_state[64],oracle_pending[70];static unsigned long long oracle_tags[2];
static bool oracle_installed=false;
static void check(const char*n,bool ok){++checks;passed+=ok;if(!ok)std::fprintf(stderr,"FAIL %s\n",n);}
static bool bits(const void*a,const void*b,std::size_t n){return std::memcmp(a,b,n)==0;}
static bool same(const double*a,const double*b,unsigned n,double&maximum){bool ok=true;
    for(unsigned j=0;j<n;++j){if(std::isnan(a[j])&&std::isnan(b[j]))continue;
        if(!std::isfinite(a[j])||!std::isfinite(b[j])){ok=ok&&(a[j]==b[j]);continue;}
        const double e=std::fabs(a[j]-b[j]);if(e>maximum)maximum=e;
        ok=ok&&e<=1e-10*(1+std::fmax(std::fabs(a[j]),std::fabs(b[j])));}
    return ok;
}
static bool take(FILE*f,void*p,std::size_t size,std::size_t count){return std::fread(p,size,count,f)==count;}
static bool put(FILE*f,const void*p,std::size_t size,std::size_t count){return std::fwrite(p,size,count,f)==count;}
static bool hash(const wchar_t*s,ib::Hash&out){for(unsigned j=0;j<64;++j){unsigned v;
    if(s[j]>=L'0'&&s[j]<=L'9')v=unsigned(s[j]-L'0');else if(s[j]>=L'A'&&s[j]<=L'F')v=unsigned(s[j]-L'A'+10);
    else if(s[j]>=L'a'&&s[j]<=L'f')v=unsigned(s[j]-L'a'+10);else return false;
    out[j/8]=(out[j/8]<<4)|v;}return s[64]==0;
}
static ib::Hash bytes_hash(const unsigned char*b){ib::Hash h{};for(unsigned j=0;j<32;++j)h[j/4]=(h[j/4]<<8)|b[j];return h;}
static bool read_window(FILE*f,struct51_T&w){std::uint32_t m[7];double s[5];
    if(!take(f,m,4,7)||!take(f,w.reference_asset_sha256,1,32)||!take(f,&w.window_generation,8,1)||!take(f,s,8,5)||
       !take(f,w.time_s,8,256)||!take(f,w.nominal_jet,8,3072)||!take(f,w.prefix_coefficients,8,192)||
       !take(f,w.ground_jet,8,12)||!take(f,w.rest_jet,8,12)||!take(f,w.relaunch_offset_ned_m,8,3))return false;
    w.schema=m[0];w.capacity=static_cast<unsigned short>(m[1]);w.leg_index=m[2];w.source_first_row=m[3];w.source_total_rows=m[4];
    w.row_count=static_cast<unsigned short>(m[5]);w.binding_mode=static_cast<unsigned char>(m[6]);w.nominal_duration_s=s[0];
    w.total_duration_s=s[1];w.prefix_duration_s=s[2];w.relaunch_duration_s=s[3];w.vertical_frame_offset_ned_m=s[4];return true;
}
static bool load(const wchar_t*reference_path,const wchar_t*inner_path){
    FILE*f=_wfopen(reference_path,L"rb");if(!f)return false;char magic[4];std::uint32_t nw=0,nr=0;
    if(!take(f,magic,1,4)||std::memcmp(magic,"RWI1",4)||!take(f,&nw,4,1)||!take(f,&nr,4,1)||nw>32||nr!=3765)return false;
    for(unsigned j=0;j<nw;++j)if(!read_window(f,windows[j]))return false;
    for(auto&r:rows)if(!take(f,r.ids,4,3)||!take(f,&r.sequence,8,1)||!take(f,&r.phase,8,1)||!take(f,r.args,8,11)||
        !take(f,r.jet,8,12)||!take(f,r.expected,8,41))return false;
    std::fclose(f);f=_wfopen(inner_path,L"rb");if(!f)return false;
    if(!take(f,magic,1,4)||std::memcmp(magic,"LCI1",4)||!take(f,&nr,4,1)||nr!=60)return false;
    static double ignored[284];unsigned long long tags[2];
    for(auto&input:saved_input)if(!take(f,input,8,36)||!take(f,tags,8,2)||!take(f,ignored,8,284))return false;
    const bool eof=std::fgetc(f)==EOF;std::fclose(f);return eof;
}
static ib::Identity identity(){return {0x1122334455667788ULL,42,1,1};}
static od::Configuration source_configuration(){od::Configuration c{};c.identity=identity();c.vehicle_odometry_topic=&topic;
    c.coordinates=od::Coordinates::ExplicitTranslatedNed;c.sample_max_age_us=5000;return c;
}
static bool create(){
    if(group)group->~Group();if(numeric)numeric->~Numeric();if(reference)reference->~Reference();if(factory)factory->~AtomicOdometryAdapter();
    ref::Configuration c{};const auto&w=windows[rows[0].ids[0]-1];std::memcpy(c.reference_asset_sha256,w.reference_asset_sha256,32);
    c.leg_index=w.leg_index;c.initial_phase_acceleration=rows[0].args[1];std::memcpy(c.initial_outer_i,rows[0].args+3,24);c.jerk_limit_mps3=rows[0].args[10];
    reference=new(reference_memory)Reference(workspace,c);numeric=new(numeric_memory)Numeric(workspace);
    group=new(group_memory)Group(*numeric,*reference);factory=new(factory_memory)od::AtomicOdometryAdapter(source_configuration());
    std::memset(oracle_state,0,sizeof oracle_state);std::memset(oracle_pending,0,sizeof oracle_pending);oracle_tags[0]=oracle_tags[1]=0;oracle_installed=false;
    return reference->load_window(w);
}
static vehicle_odometry_s raw_message(unsigned index){
    // Explicit synthetic canonical-up -> raw PX4 NED fixture conversion.
    const auto*x=saved_input[index];vehicle_odometry_s m{};const double sign[3]={1,1,-1};
    m.timestamp_sample=1000000ULL+9000ULL*index;m.timestamp=m.timestamp_sample+100;
    m.pose_frame=m.POSE_FRAME_NED;m.velocity_frame=m.VELOCITY_FRAME_NED;
    for(unsigned j=0;j<3;++j){m.position[j]=static_cast<float>(x[j]*sign[j]);m.velocity[j]=static_cast<float>(x[j+3]*sign[j]);}
    const double qs[4]={1,-1,-1,1};for(unsigned j=0;j<4;++j)m.q[j]=static_cast<float>(x[6+j]*qs[j]);
    m.angular_velocity[0]=static_cast<float>(-x[10]);m.angular_velocity[1]=static_cast<float>(-x[11]);m.angular_velocity[2]=static_cast<float>(x[12]);
    m.position_variance[0]=std::numeric_limits<float>::quiet_NaN();return m;
}
static bool capture(unsigned index,od::Snapshot&s,bool wrong=false){const auto m=raw_message(index);
    const bool ok=factory->ingest(m,index+1,m.timestamp_sample+200,m.timestamp_sample+300,identity(),wrong?&wrong_topic:&topic,0,s);
    all_factory_captures+=ok;return ok;
}
static void flatten(double*p,const struct56_T&r){std::memcpy(p,r.position_m,24);std::memcpy(p+3,r.velocity_mps,24);
    std::memcpy(p+6,r.acceleration_mps2,24);std::memcpy(p+9,r.jerk_mps3,24);}
static void flatten_transition(double*p,const struct55_T&t){flatten(p,t.reference);p[12]=t.phase_acceleration_s_inv;p[13]=t.phase_jerk_s_inv2;
    std::memcpy(p+14,t.outer_correction_i_mps2,24);std::memcpy(p+17,t.outer_correction_jerk_i_mps3,24);p[20]=t.fraction;
    std::memcpy(p+21,t.frame_i_from_f,72);std::memcpy(p+30,t.reference_frame_i_from_f,72);p[39]=t.reference_curvature;p[40]=t.reference_signed_yaw_rate;}
static bool prepare_reference(unsigned index,const od::Snapshot&s){const auto&r=rows[index];const auto&w=windows[r.ids[0]-1];ref::Input input{};
    if(reference->installed_state().query.window_generation!=w.window_generation&&index&& !reference->load_window(w))return false;
    std::memcpy(input.query.reference_asset_sha256,w.reference_asset_sha256,32);input.query.leg_index=w.leg_index;input.query.window_generation=w.window_generation;
    input.query.query_sequence=r.sequence;input.query.progress_s=r.phase;input.source_timestamp_ns=s.estimator().timestamp_sample_us*1000;
    input.source_generation=s.estimator().generation;input.reference_generation=1;input.outer_generation=1;input.progress_rate=r.args[0];
    input.target_phase_acceleration=r.args[2];std::memcpy(input.target_outer_f,r.args+6,24);input.dt_s=r.args[9];
    return reference->prepare(input)==ref::PrepareResult::Candidate;
}
struct Bindings {ib::Context context;ib::BoundRotorLag rotor;ib::BoundReference reference;ib::BoundPayload payload;ib::BoundWind wind;ib::ExplicitInitialInterval initial;};
static Bindings bindings(unsigned index,const od::Snapshot&s){Bindings b{};ib::SnapshotKey key{};ib::snapshot_key(s,key);const auto&rc=*reference->candidate();
    const auto asset=bytes_hash(rc.input.query.reference_asset_sha256);const auto*x=saved_input[index];
    b.context.observed_session=identity();b.context.task_sha256=task_sha;b.context.reference_asset_sha256=asset;b.context.configuration_sha256=config_sha;
    b.context.explicit_leg=rc.input.query.leg_index;b.context.step_kind=index?ib::StepKind::SubsequentObservedSource:ib::StepKind::FirstOfExplicitLeg;
    b.rotor.source=b.reference.source=b.payload.source=b.wind.source=key;b.rotor.value_kind=ib::RotorValueKind::OriginalPlantLagState;
    b.rotor.verified_association_receipt_sha256=fixture_sha;auto&r=b.rotor.original_observation;
    for(unsigned j=0;j<6;++j)r.observed_thrust_n[j]=x[13+j];r.dll_generation=index+1;r.dll_session=11;
    r.original_host_receive_ns=7000000000ULL+9000000ULL*index;r.original_board_ingress_us=s.estimator().board_rx_us;
    r.original_sim_time_s=1+0.009*index;r.original_observation_sha=fixture_sha;
    auto&f=b.reference.candidate;f.leg=b.context.explicit_leg;f.window_generation=rc.input.query.window_generation;f.candidate_token=rc.candidate_generation;
    f.original_source_generation=s.estimator().generation;
    for(unsigned j=0;j<3;++j){f.position_m[j]=rc.transition.reference.position_m[j];f.velocity_mps[j]=rc.transition.reference.velocity_mps[j];
        f.acceleration_mps2[j]=rc.transition.reference.acceleration_mps2[j];f.jerk_mps3[j]=rc.transition.reference.jerk_mps3[j];}
    f.phase_before_s=reference->installed_state().query_progress_s;f.phase_after_s=rc.input.query.progress_s;
    b.reference.task_sha256=task_sha;b.reference.configuration_sha256=config_sha;b.reference.asset_sha256=asset;b.reference.candidate_evidence_sha256=reference_sha;
    b.payload.task_sha256=task_sha;b.payload.original_schedule_evidence_sha256=fixture_sha;b.payload.original_schedule_generation=index+1;b.payload.payload_kg=x[31];
    b.wind.original_estimate_evidence_sha256=fixture_sha;b.wind.original_estimate_generation=index+1;b.wind.estimate_xy_mps={x[32],x[33]};
    b.initial.configuration_sha256=config_sha;b.initial.original_configuration_receipt_sha256=reference_sha;b.initial.leg=b.context.explicit_leg;
    b.initial.configured_dt_s=rows[index].args[9];return b;
}
static bool build(const od::Snapshot&s,const Bindings&b,ib::Result&out,bool missing_rotor=false){return ib::build(s,b.context,missing_rotor?nullptr:&b.rotor,
    &b.reference,&b.payload,&b.wind,b.context.step_kind==ib::StepKind::FirstOfExplicitLeg?&b.initial:nullptr,out);}
struct Receipts {joint::BackendPublication event;num::PublicationReceipt n;ref::PublicationReceipt r;};
static Receipts receipts(){Receipts a{};const auto&n=*numeric->candidate();const auto&r=*reference->candidate();
    a.event.source_timestamp_ns=n.original_tags2[0];a.event.source_generation=n.original_tags2[1];a.event.output_generation=numeric->installed_count()+1;
    a.event.original_publication_us=n.original_tags2[0]/1000+400;std::memcpy(a.event.actual_control16,n.control16,sizeof n.control16);a.event.output_published=true;
    a.n.source_timestamp_ns=a.r.source_timestamp_ns=a.event.source_timestamp_ns;a.n.source_generation=a.r.source_generation=a.event.source_generation;
    a.n.output_generation=a.r.output_generation=a.event.output_generation;a.n.original_publication_us=a.r.original_publication_us=a.event.original_publication_us;
    std::memcpy(a.n.actual_control16,n.control16,sizeof n.control16);a.n.publication_succeeded=a.n.numerical_commit_succeeded=true;
    a.r.candidate_generation=r.candidate_generation;a.r.query_sequence=r.input.query.query_sequence;a.r.window_generation=r.input.query.window_generation;
    a.r.reference_generation=r.input.reference_generation;a.r.outer_generation=r.input.outer_generation;flatten(a.r.actual_reference_pvaj,r.transition.reference);
    a.r.publication_succeeded=a.r.numerical_commit_succeeded=true;return a;
}
static bool install(const Receipts&r){return group->install_after_publication(r.event,r.n,r.r);}
static void direct_control(const double*k,float*c){std::memset(c,0,16*sizeof(float));const unsigned map[6]={4,0,3,5,1,2};
    for(unsigned j=0;j<6;++j)c[map[j]]=static_cast<float>(k[4+j]/32.145727009134916);}
static bool complete_pending(double*p,const double*request,const double*result){
    if(p[44]!=0||p[45]!=0||p[1]!=0||request[0]!=(double)(p[2]!=0&&p[3]!=0))return false;
    if(request[0]==0)return result==nullptr;if(!result)return false;
    p[1]=1;std::memcpy(p+9,request+1,17*sizeof(double));std::memcpy(p+35,result,3*sizeof(double));std::memcpy(p+41,result+6,3*sizeof(double));
    p[5]=result[13];p[7]=result[12];p[8]=std::fmax(std::fmax(result[9],result[10]),result[11]);p[4]=(double)(result[14]!=0);
    for(unsigned j=0;j<3;++j)p[38+j]=(request[18]*p[5])*p[35+j];return true;
}
static bool prepare_row(unsigned index,od::Snapshot&s,ib::Result&assembled){
    if(!capture(index,s)||!prepare_reference(index,s))return false;const auto b=bindings(index,s);
    return build(s,b,assembled)&&numeric->prepare(assembled.input36,assembled.tags2);
}
static bool fill(Predict predict){if(!numeric->prediction_required())return true;double gp[18];const auto*q=numeric->gp_request19();
    const auto&st=reference->installed_state();const unsigned long long tags[2]={st.source_timestamp_ns,st.source_generation};
    return predict(q+1,gp)==GPENMPC_GP256_OK&&numeric->fill_open_prediction(tags,gp);
}
int wmain(int argc,wchar_t**argv){
    if(argc!=10||!hash(argv[6],config_sha)||!hash(argv[7],task_sha)||!hash(argv[8],fixture_sha)||!hash(argv[9],reference_sha)||!load(argv[1],argv[2]))return 2;
    static_assert(!std::is_constructible<od::Snapshot,vehicle_odometry_s>::value,"private Snapshot must not be bypassed");
    HMODULE dll=LoadLibraryExW(argv[3],nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);if(!dll)return 3;
    const auto predict=reinterpret_cast<Predict>(GetProcAddress(dll,"gpenmpc_gp256_predict"));if(!predict)return 3;
    gpenmpcNative_canonicalLocalInnerFixedFirst_initialize();FILE*raw=_wfopen(argv[5],L"wb");if(!raw)return 3;
    const std::uint32_t count=60,topic_bytes=sizeof(vehicle_odometry_s);bool written=put(raw,"SJC1",1,4)&&put(raw,&count,4,1)&&put(raw,&topic_bytes,4,1);
    check("fixed declared combined fixture loaded and owner initialized",create());
    unsigned state_exact=0,kernel_exact=0,controls_exact=0,pending_exact=0,reference_checked=0;
    for(unsigned index=0;index<60;++index){od::Snapshot snapshot;ib::Result input{};
        const bool captured=capture(index,snapshot);source_captures+=captured;check("real private source factory capture",captured);if(!captured)break;
        bool ok=prepare_reference(index,snapshot);check("original task query plus FromJet prepare",ok);if(!ok)break;
        const auto b=bindings(index,snapshot);ok=build(snapshot,b,input);check("bound lag reference payload wind to exact input36",ok);if(!ok)break;
        double transition[41];flatten_transition(transition,reference->candidate()->transition);
        const bool refok=same(transition,rows[index].expected,41,max_reference);reference_checked+=refok;check("original MATLAB reference oracle retained",refok);
        check("source uint64 generation and explicit first or measured subsequent dt retained",input.tags2[0]==snapshot.estimator().timestamp_sample_us*1000&&
            input.tags2[1]==snapshot.subscription_generation()&&input.source.sample_delta_us==(index?9000u:0u)&&input.input36[34]==rows[index].args[9]&&
            !input.board_authority&&!input.external_clock_association_proven_here);
        static ref::Candidate saved_reference;std::memcpy(&saved_reference,reference->candidate(),sizeof saved_reference);
        ok=numeric->prepare(input.input36,input.tags2);check("actual numeric store C preparation",ok);if(!ok)break;
        check("same SD serialized query candidate survives numerical scratch reuse",bits(&saved_reference,reference->candidate(),sizeof saved_reference));
        static double state[64],kernel[61],pending[70],request[19];float controls[16];
        if(!oracle_installed)gpenmpcNative_canonicalLocalInnerFixedFirst(&oracle_workspace,input.input36,input.tags2,state,kernel,pending,request);
        else gpenmpcNative_canonicalLocalInnerFixedStep(&oracle_workspace,oracle_state,oracle_tags,input.input36,input.tags2,oracle_pending,oracle_tags,state,kernel,pending,request);
        ++direct_calls;direct_control(kernel,controls);const auto n=*numeric->candidate();
        state_exact+=bits(state,n.post_state64,sizeof state);kernel_exact+=bits(kernel,n.kernel61,sizeof kernel);controls_exact+=bits(controls,n.control16,sizeof controls);
        check("independent direct actual C same-input 64 61 scaffold request 16 match",same(state,n.post_state64,64,max_numeric)&&same(kernel,n.kernel61,61,max_numeric)&&
            same(pending,n.scaffold70,70,max_numeric)&&same(request,n.request19,19,max_numeric)&&bits(controls,n.control16,sizeof controls));
        const auto r=receipts();ok=install(r);check("one mock output event jointly installs reference and numeric state",ok&&numeric->installed_count()==index+1&&
            reference->installed_count()==index+1&&group->diagnostics().unexpected_partial_installs==0);if(!ok)break;
        std::memcpy(oracle_state,state,sizeof state);std::memcpy(oracle_pending,pending,sizeof pending);std::memcpy(oracle_tags,input.tags2,sizeof oracle_tags);oracle_installed=true;
        double gp[18],ogp[18];for(unsigned j=0;j<18;++j)gp[j]=ogp[j]=std::numeric_limits<double>::quiet_NaN();
        if(numeric->prediction_required()){
            ++main_gp_calls;++oracle_gp_calls;const bool g1=predict(numeric->gp_request19()+1,gp)==GPENMPC_GP256_OK;
            const bool g2=predict(request+1,ogp)==GPENMPC_GP256_OK;
            ok=g1&&g2&&same(gp,ogp,18,max_numeric)&&numeric->fill_open_prediction(input.tags2,gp)&&complete_pending(oracle_pending,request,ogp);
        }else ok=request[0]==0&&complete_pending(oracle_pending,request,nullptr);
        check("original GP DLL serial same-input results only complete current pending",ok);if(!ok)break;
        pending_exact+=bits(numeric->open_pending70(),oracle_pending,sizeof oracle_pending);
        check("k pending and committed state remain equal for next actual source",same(numeric->installed_state64(),oracle_state,64,max_numeric)&&
            same(numeric->open_pending70(),oracle_pending,70,max_numeric)&&numeric->open_pending70()[44]==0&&numeric->open_pending70()[45]==0);
        written=written&&put(raw,&snapshot.raw(),1,topic_bytes)&&put(raw,input.tags2,8,2)&&put(raw,input.input36,8,36)&&put(raw,transition,8,41)&&
            put(raw,n.post_state64,8,64)&&put(raw,n.kernel61,8,61)&&put(raw,n.control16,4,16)&&put(raw,n.request19,8,19)&&put(raw,gp,8,18)&&
            put(raw,state,8,64)&&put(raw,kernel,8,61)&&put(raw,controls,4,16)&&put(raw,ogp,8,18)&&put(raw,numeric->open_pending70(),8,70)&&put(raw,oracle_pending,8,70);
        ++main_completed;
    }
    std::fclose(raw);check("all 60 complete no partial install and full raw rows saved",main_completed==60&&written&&source_captures==60&&
        numeric->installed_count()==60&&reference->installed_count()==60&&group->diagnostics().unique_publications_reported==60);
    check("independent direct C state kernel controls and pending bit exact on all rows",state_exact==60&&kernel_exact==60&&controls_exact==60&&pending_exact==60&&reference_checked==60);
    // Each negative gets its own explicitly constructed fixture owner. No
    // retries/reset of a faulted production transaction are implied.
    {od::Snapshot s;bool ok=create()&&!capture(0,s,true);
        check("wrong actual source topic fails before reference numerical publication",ok&&!s.valid()&&numeric->kernel_calls()==0&&reference->installed_count()==0&&numeric->installed_count()==0&&group->diagnostics().attempts==0);}
    for(unsigned kind=0;kind<2;++kind){od::Snapshot s;ib::Result input{};bool ok=create()&&capture(0,s)&&prepare_reference(0,s);auto b=bindings(0,s);
        if(kind==1)++b.reference.source.source_generation;ok=ok&&!build(s,b,input,kind==0);
        check("missing rotor or wrong reference source cannot create numeric candidate or install",ok&&numeric->kernel_calls()==0&&numeric->installed_count()==0&&reference->installed_count()==0&&group->diagnostics().attempts==0);}
    for(unsigned kind=0;kind<3;++kind){od::Snapshot s;ib::Result input{};bool ok=create()&&prepare_row(0,s,input);auto r=receipts();
        if(kind==0){r.event.output_published=false;r.n.publication_succeeded=r.r.publication_succeeded=false;}
        if(kind==1)r.r.actual_reference_pvaj[0]+=1;
        if(kind==2)r.n.actual_control16[0]+=1;
        ok=ok&&!install(r)&&numeric->installed_count()==0&&reference->installed_count()==0&&group->diagnostics().unexpected_partial_installs==0&&
            group->diagnostics().unique_publications_reported==(kind?1u:0u);
        check("failed output or wrong committed reference/control bits locks both with no half installation",ok&&!prepare_row(1,s,input));}
    {bool ok=create(),found=false;std::uint64_t before=0;
        for(unsigned index=0;index<10&&ok;++index){od::Snapshot s;ib::Result input{};ok=prepare_row(index,s,input)&&install(receipts());
            if(ok&&numeric->prediction_required()){
                found=true;before=numeric->installed_count();ok=capture(index+1,s)&&prepare_reference(index+1,s);auto b=bindings(index+1,s);
                ok=ok&&build(s,b,input)&&!numeric->prepare(input.input36,input.tags2)&&numeric->failure()==num::Failure::MissingPrediction&&
                    numeric->installed_count()==before&&reference->installed_count()==before&&group->diagnostics().joint_installs==before;
                // Stop the owner when a required GP result is missing or late.
                break;
            }
        }
        check("missing required previous GP stops next source before either next installation",ok&&found&&before>0);
    }
    {bool ok=create();od::Snapshot s;ib::Result input{};ok=ok&&prepare_row(0,s,input)&&install(receipts())&&fill(predict);
        const auto before=numeric->installed_count();const auto m=raw_message(0);od::Snapshot duplicate;
        ok=ok&&!factory->ingest(m,1,m.timestamp_sample+200,m.timestamp_sample+300,identity(),&topic,0,duplicate);
        check("duplicate original source rejected without reusing control generation",ok&&!duplicate.valid()&&numeric->installed_count()==before&&reference->installed_count()==before);
    }
    FILE*out=_wfopen(argv[4],L"w");if(!out)return 3;
    std::fprintf(out,"{\"pass\":%s,\"checks\":%u,\"passed\":%u,\"rows\":%u,\"private_factory_main_captures\":%u,\"private_factory_all_captures\":%u,\"direct_oracle_actual_C_calls\":%u,\"store_actual_C_main_calls\":%u,\"main_GP_DLL_calls\":%u,\"oracle_GP_DLL_calls\":%u,\"state64_bit_exact_rows\":%u,\"kernel61_bit_exact_rows\":%u,\"control16_bit_exact_rows\":%u,\"pending70_bit_exact_rows\":%u,\"reference_MATLAB_oracle_rows\":%u,\"maximum_inner_numeric_difference\":%.17g,\"maximum_reference_difference\":%.17g,\"topic_bytes\":%u,\"source_dt_us\":9000,\"first_dt_explicit_RWI_configuration\":true,\"actual_delta_first_us\":0,\"private_factory_used\":true,\"external_rotor_clock_association_and_publication_mock\":true,\"single_owner_serial_GP_calls\":true,\"COM_UDP_model_solver_board_actions\":0}\n",
        checks==passed?"true":"false",checks,passed,main_completed,source_captures,all_factory_captures,direct_calls,main_completed,main_gp_calls,oracle_gp_calls,
        state_exact,kernel_exact,controls_exact,pending_exact,reference_checked,max_numeric,max_reference,topic_bytes);
    std::fclose(out);FreeLibrary(dll);std::printf("SOURCE_TO_GP_JOINT_CHAIN %u/%u; rows %u; GP %u+%u; numeric diff %.17g; reference diff %.17g\n",
        passed,checks,main_completed,main_gp_calls,oracle_gp_calls,max_numeric,max_reference);return checks==passed?0:4;
}
