// Test reference querying and transition state with mock publication and source tags.
#include "CanonicalReferenceStateStore.hpp"
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_initialize.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <limits>
namespace math=gpenmpc_reference_math;
using Store=math::CanonicalReferenceStateStore;
static Store::Workspace workspace;
static struct51_T windows[32],bad_window;
alignas(Store) static unsigned char storage[sizeof(Store)];
static Store*owner=nullptr;
static unsigned checks=0,passed=0,bad=0;
static void check(const char*name,bool ok){++checks;passed+=ok;if(!ok)std::fprintf(stderr,"FAIL %s\n",name);}
static bool bits(const void*a,const void*b,std::size_t n){return std::memcmp(a,b,n)==0;}
static bool take(FILE*f,void*p,std::size_t size,std::size_t n){return std::fread(p,size,n,f)==n;}
static bool read_window(FILE*f,struct51_T&w){
    std::uint32_t m[7];double s[5];
    if(!take(f,m,4,7)||!take(f,w.reference_asset_sha256,1,32)||!take(f,&w.window_generation,8,1)||!take(f,s,8,5)||
       !take(f,w.time_s,8,256)||!take(f,w.nominal_jet,8,3072)||!take(f,w.prefix_coefficients,8,192)||
       !take(f,w.ground_jet,8,12)||!take(f,w.rest_jet,8,12)||!take(f,w.relaunch_offset_ned_m,8,3))return false;
    w.schema=m[0];w.capacity=static_cast<unsigned short>(m[1]);w.leg_index=m[2];w.source_first_row=m[3];
    w.source_total_rows=m[4];w.row_count=static_cast<unsigned short>(m[5]);w.binding_mode=static_cast<unsigned char>(m[6]);
    w.nominal_duration_s=s[0];w.total_duration_s=s[1];w.prefix_duration_s=s[2];w.relaunch_duration_s=s[3];w.vertical_frame_offset_ned_m=s[4];return true;
}
struct Row {std::uint32_t ids[3];unsigned long long sequence;double phase,args[11],jet[12],expected[41];};
static bool read_row(FILE*f,Row&r){return take(f,r.ids,4,3)&&take(f,&r.sequence,8,1)&&take(f,&r.phase,8,1)&&
    take(f,r.args,8,11)&&take(f,r.jet,8,12)&&take(f,r.expected,8,41);}
static Store&create(const Row&r){
    if(owner)owner->~Store();math::Configuration c{};const auto&w=windows[r.ids[0]-1];
    std::memcpy(c.reference_asset_sha256,w.reference_asset_sha256,32);c.leg_index=w.leg_index;
    c.initial_phase_acceleration=r.args[1];std::memcpy(c.initial_outer_i,r.args+3,24);c.jerk_limit_mps3=r.args[10];
    owner=new(storage)Store(workspace,c);return *owner;
}
static math::Input input(const Row&r,const struct51_T&w){
    math::Input i{};std::memcpy(i.query.reference_asset_sha256,w.reference_asset_sha256,32);
    i.query.leg_index=w.leg_index;i.query.window_generation=w.window_generation;i.query.query_sequence=r.sequence;i.query.progress_s=r.phase;
    i.source_timestamp_ns=1000000000ULL+r.sequence*9000000ULL;i.source_generation=r.sequence;
    i.reference_generation=w.leg_index;i.outer_generation=1;i.progress_rate=r.args[0];i.target_phase_acceleration=r.args[2];
    std::memcpy(i.target_outer_f,r.args+6,24);i.dt_s=r.args[9];return i;
}
static void reference(double*v,const struct56_T&r){std::memcpy(v,r.position_m,24);std::memcpy(v+3,r.velocity_mps,24);
    std::memcpy(v+6,r.acceleration_mps2,24);std::memcpy(v+9,r.jerk_mps3,24);}
static void flatten(double*v,const struct55_T&t){
    reference(v,t.reference);v[12]=t.phase_acceleration_s_inv;v[13]=t.phase_jerk_s_inv2;
    std::memcpy(v+14,t.outer_correction_i_mps2,24);std::memcpy(v+17,t.outer_correction_jerk_i_mps3,24);v[20]=t.fraction;
    std::memcpy(v+21,t.frame_i_from_f,72);std::memcpy(v+30,t.reference_frame_i_from_f,72);v[39]=t.reference_curvature;v[40]=t.reference_signed_yaw_rate;
}
static math::PublicationReceipt publication(const Store&s){
    const auto&c=*s.candidate();math::PublicationReceipt r{};
    r.source_timestamp_ns=c.input.source_timestamp_ns;r.source_generation=c.input.source_generation;
    r.candidate_generation=c.candidate_generation;r.query_sequence=c.input.query.query_sequence;r.window_generation=c.input.query.window_generation;
    r.reference_generation=c.input.reference_generation;r.outer_generation=c.input.outer_generation;
    r.output_generation=s.installed_state().output_generation+1;r.original_publication_us=r.source_timestamp_ns/1000+1;
    reference(r.actual_reference_pvaj,c.transition.reference);r.publication_succeeded=r.numerical_commit_succeeded=true;return r;
}
static bool start(const Row&r){auto&s=create(r);return s.load_window(windows[r.ids[0]-1])&&s.failure()==math::Failure::None;}
static bool prepare(const Row&r){return owner->prepare(input(r,windows[r.ids[0]-1]))==math::PrepareResult::Candidate;}
static bool publish(){return owner->install_after_publication(publication(*owner));}
int wmain(int argc,wchar_t**argv){
    if(argc!=3)return 2;FILE*f=_wfopen(argv[1],L"rb");if(!f)return 3;
    char magic[4];std::uint32_t nw=0,nr=0;
    if(!take(f,magic,1,4)||std::memcmp(magic,"RWI1",4)||!take(f,&nw,4,1)||!take(f,&nr,4,1)||nw>32||nr!=3765)return 4;
    for(unsigned j=0;j<nw;++j)if(!read_window(f,windows[j]))return 5;
    gpenmpcNative_canonicalLocalInnerFixedFirst_initialize();
    unsigned left_probe_rejections=0,midpoint_probe_acceptances=0;
    for(unsigned j=0;j<nw;++j){const auto&w=windows[j];struct52_T s{},next{};struct53_T q{};struct54_T receipt{};double jet[12];
        std::memcpy(s.reference_asset_sha256,w.reference_asset_sha256,32);s.leg_index=w.leg_index;s.window_generation=w.window_generation;
        std::memcpy(q.reference_asset_sha256,w.reference_asset_sha256,32);q.leg_index=w.leg_index;q.window_generation=w.window_generation;q.query_sequence=1;
        q.progress_s=w.time_s[0]+(w.binding_mode==1?25.0:0.0);
        gpenmpcNative_queryCanonicalReferenceWindow(&workspace,&w,&s,&q,&next,jet,&receipt);
        if(!receipt.accepted&&receipt.reason==5){++left_probe_rejections;
            std::printf("Original left-edge validation probe: window %u first %.17g effective %.17g reason 5\n",j+1,w.time_s[0],receipt.effective_nominal_progress_s);}
        q.progress_s=w.time_s[0]*0.5+w.time_s[1]*0.5+(w.binding_mode==1?25.0:0.0);
        gpenmpcNative_queryCanonicalReferenceWindow(&workspace,&w,&s,&q,&next,jet,&receipt);midpoint_probe_acceptances+=receipt.accepted;
    }
    check("actual prefix roundoff counterexample and internal-only validation probe",left_probe_rejections>0&&midpoint_probe_acceptances==nw);
    Row row{},seed{},second{};unsigned leg=0,last_window=0,commits=0,misses=0,refills=0,preserved=0,exact_jet=0,exact_transition=0;
    unsigned by_leg[5]{},by_leg_bad[5]{};double max_jet=0,max_transition=0;bool run_ok=true;
    for(unsigned n=0;n<nr;++n){
        if(!read_row(f,row)||row.ids[0]<1||row.ids[0]>nw||row.ids[1]<1||row.ids[1]>5)return 6;
        if(n==0)seed=row;if(n==1)second=row;
        const auto&w=windows[row.ids[0]-1];
        if(row.ids[1]!=leg){if(!start(row)){std::fprintf(stderr,"initial window rejected leg %u\n",row.ids[1]);run_ok=false;break;}leg=row.ids[1];last_window=row.ids[0];}
        const auto before=owner->installed_state();
        if(row.ids[0]!=last_window){
            auto missing=input(row,windows[last_window-1]);
            const bool miss=owner->prepare(missing)==math::PrepareResult::WindowMiss&&owner->failure()==math::Failure::None&&
                owner->candidate()==nullptr&&bits(&before,&owner->installed_state(),sizeof before);
            if(!miss){std::fprintf(stderr,"expected window miss row %u\n",n);run_ok=false;break;}++misses;
            if(!owner->load_window(w)||!bits(&before,&owner->installed_state(),sizeof before)){
                std::fprintf(stderr,"refill rejected row %u failure %u\n",n,unsigned(owner->failure()));run_ok=false;break;}
            ++refills;last_window=row.ids[0];
        }
        if(!prepare(row)||!owner->candidate()){std::fprintf(stderr,"prepare rejected row %u\n",n);run_ok=false;break;}
        preserved+=bits(&before,&owner->installed_state(),sizeof before);
        const auto&c=*owner->candidate();double out[41];flatten(out,c.transition);
        exact_jet+=bits(c.jet,row.jet,sizeof row.jet);exact_transition+=bits(out,row.expected,sizeof out);
        for(unsigned j=0;j<12;++j){const double e=std::fabs(c.jet[j]-row.jet[j]);max_jet=std::max(max_jet,e);
            if(!std::isfinite(c.jet[j])||e>2e-11*std::max(1.0,std::fabs(row.jet[j]))){++bad;++by_leg_bad[leg-1];}}
        for(unsigned j=0;j<41;++j){const double e=std::fabs(out[j]-row.expected[j]);max_transition=std::max(max_transition,e);
            if(!std::isfinite(out[j])||e>2e-11*std::max(1.0,std::fabs(row.expected[j]))){++bad;++by_leg_bad[leg-1];}}
        if(!publish()){run_ok=false;break;}++commits;++by_leg[leg-1];
        if(owner->installed_state().query.last_accepted_sequence!=row.sequence||owner->candidate()!=nullptr){run_ok=false;break;}
    }
    const bool exhausted=std::fgetc(f)==EOF;std::fclose(f);
    check("3765 same-SD actual query transition and successful installs",run_ok&&exhausted&&commits==nr&&bad==0);
    check("every prepare preserves installed reference state",preserved==nr);
    check("24 window misses/refills never advance reference state",misses==24&&refills==24);
    for(unsigned j=0;j<5;++j)check("actual leg complete and original oracle compatible",by_leg[j]>0&&by_leg_bad[j]==0);
    check("initial load is cache only and has no committed phase/query state",start(seed)&&!owner->committed()&&owner->installed_state().query.window_generation==0&&owner->installed_state().query.last_accepted_sequence==0);
    for(unsigned kind=0;kind<12;++kind){
        bool ok=start(seed)&&prepare(seed);const auto before=owner->installed_state();auto r=publication(*owner);
        if(kind==0)r.publication_succeeded=false;
        if(kind==1)r.numerical_commit_succeeded=false;
        if(kind==2)++r.source_timestamp_ns;
        if(kind==3)++r.source_generation;
        if(kind==4)++r.candidate_generation;
        if(kind==5)++r.query_sequence;
        if(kind==6)++r.window_generation;
        if(kind==7)++r.reference_generation;
        if(kind==8)++r.outer_generation;
        if(kind==9)r.output_generation=0;
        if(kind==10)r.original_publication_us=r.source_timestamp_ns/1000-1;
        if(kind==11)r.actual_reference_pvaj[0]+=1;
        ok=ok&&!owner->install_after_publication(r)&&owner->failure()==math::Failure::Publication&&
            bits(&before,&owner->installed_state(),sizeof before)&&owner->installed_count()==0&&owner->candidate()==nullptr;
        const auto calls=owner->transition_calls();ok=ok&&!prepare(seed)&&owner->transition_calls()==calls;
        check("bad/failed publication binding cannot install or retry",ok);
    }
    {bool ok=start(seed)&&prepare(seed);auto receipt=publication(*owner);ok=ok&&owner->install_after_publication(receipt);
        const auto before=owner->installed_state();ok=ok&&!owner->install_after_publication(receipt)&&bits(&before,&owner->installed_state(),sizeof before)&&owner->installed_count()==1;
        check("replayed publication cannot reinstall",ok);}
    {bool ok=start(seed)&&prepare(seed);const auto before=owner->installed_state();ok=ok&&!prepare(second)&&
        owner->failure()==math::Failure::PendingCandidate&&bits(&before,&owner->installed_state(),sizeof before);
        check("second prepare while candidate pending fails without state advance",ok);}
    {bool ok=start(seed)&&prepare(seed);const auto before=owner->installed_state();bad_window=windows[seed.ids[0]-1];++bad_window.window_generation;
        ok=ok&&!owner->load_window(bad_window)&&owner->failure()==math::Failure::PendingCandidate&&bits(&before,&owner->installed_state(),sizeof before);
        check("refill cannot replace pending candidate window",ok);}
    for(unsigned kind=0;kind<6;++kind){
        bool ok=start(seed)&&prepare(seed)&&publish();const auto before=owner->installed_state();auto in=input(second,windows[second.ids[0]-1]);
        if(kind==0)in.source_timestamp_ns=before.source_timestamp_ns;
        if(kind==1)in.source_generation=before.source_generation;
        if(kind==2)in.query.query_sequence=before.query.last_accepted_sequence;
        if(kind==3)++in.query.leg_index;
        if(kind==4)in.query.progress_s=std::numeric_limits<double>::quiet_NaN();
        if(kind==5)in.target_outer_f[0]=std::numeric_limits<double>::infinity();
        const auto calls=owner->transition_calls();ok=ok&&owner->prepare(in)==math::PrepareResult::Rejected&&
            bits(&before,&owner->installed_state(),sizeof before)&&owner->transition_calls()==calls;
        check("source/sequence/identity/nonfinite rejects before transition/install",ok);
    }
    for(unsigned kind=0;kind<6;++kind){
        bool ok=start(seed)&&prepare(seed)&&publish();const auto before=owner->installed_state();const auto generation=owner->loaded_window_generation();
        bad_window=windows[seed.ids[0]-1];++bad_window.window_generation;
        if(kind==0)bad_window.leg_index=2;
        if(kind==1)bad_window.row_count=1;
        if(kind==2)bad_window.time_s[1]=bad_window.time_s[0];
        if(kind==3)bad_window.nominal_jet[0]=std::numeric_limits<double>::quiet_NaN();
        if(kind==4)bad_window.nominal_jet[0]+=1;
        if(kind==5)bad_window.prefix_duration_s+=1;
        ok=ok&&!owner->load_window(bad_window)&&owner->loaded_window_generation()==generation&&
            bits(&before,&owner->installed_state(),sizeof before)&&owner->installed_count()==1;
        check("invalid or changed refill preserves prior window and installed state",ok);
    }
    {bool ok=start(seed)&&prepare(seed)&&publish();auto i=input(second,windows[second.ids[0]-1]);
        i.source_generation+=17;i.query.query_sequence+=13;
        ok=ok&&owner->prepare(i)==math::PrepareResult::Candidate&&publish();
        check("strictly increasing nonunit source/query generations are accepted",ok);}
    {bool ok=start(seed);auto i=input(seed,windows[seed.ids[0]-1]);i.dt_s=std::numeric_limits<double>::quiet_NaN();
        ok=ok&&owner->prepare(i)==math::PrepareResult::Candidate&&owner->candidate()->transition.fraction==0&&publish();
        check("original FromJet invalid-dt fraction-zero numerical branch retained",ok);}
    FILE*out=_wfopen(argv[2],L"w");if(!out)return 7;
    std::fprintf(out,"{\"pass\":%s,\"checks\":%u,\"passed\":%u,\"queries\":%u,\"commits\":%u,\"refills\":%u,\"window_misses\":%u,\"bit_exact_jet_queries\":%u,\"bit_exact_transition_queries\":%u,\"maximum_jet_error\":%.17g,\"maximum_transition_error\":%.17g,\"bad_values\":%u,\"workspace_bytes\":%zu,\"reference_store_bytes\":%zu,\"window_bytes\":%zu,\"old_validation_probe_roundoff_rejections\":%u,\"same_serial_SD\":true,\"publication_source_clock_are_mocks\":true,\"atomic_group_commit_with_inner_store_proven\":false,\"phase_integrator_invoked\":false,\"COM_UDP_plant_board_actions\":0}\n",
        checks==passed?"true":"false",checks,passed,nr,commits,refills,misses,exact_jet,exact_transition,max_jet,max_transition,bad,sizeof workspace,sizeof(Store),sizeof(struct51_T),left_probe_rejections);
    std::fclose(out);std::printf("REFERENCE_STATE_STORE %u/%u; %u original queries; max %.17g\n",passed,checks,commits,max_transition);
    return checks==passed?0:8;
}
