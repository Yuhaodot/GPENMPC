// Test task-reference wiring into the C kernel with saved synthetic-state inputs.
#include "CanonicalJointStateInstaller.hpp"
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_initialize.h"
#include <cstdio>
#include <new>
namespace joint=gpenmpc_joint_math;
namespace num=gpenmpc_local_math;
namespace ref=gpenmpc_reference_math;
using Numeric=num::CanonicalLocalInnerStateStore;using Reference=ref::CanonicalReferenceStateStore;
using Group=joint::CanonicalJointStateInstaller;
static Numeric::Workspace workspace;
static struct51_T windows[32];
struct Row{std::uint32_t ids[3];unsigned long long sequence;double phase,args[11],jet[12],expected[41];};
static Row rows[2];static double input_state[2][36];
alignas(Numeric) static unsigned char numeric_storage[sizeof(Numeric)];
alignas(Reference) static unsigned char reference_storage[sizeof(Reference)];
alignas(Group) static unsigned char group_storage[sizeof(Group)];
static Numeric*numeric=nullptr;static Reference*reference=nullptr;static Group*group=nullptr;
static unsigned checks{},passed{},actual_first_calls{},actual_step_calls{};
static void check(const char*n,bool ok){++checks;passed+=ok;if(!ok)std::fprintf(stderr,"FAIL %s\n",n);}
static bool bits(const void*a,const void*b,std::size_t n){return std::memcmp(a,b,n)==0;}
static bool take(FILE*f,void*p,std::size_t s,std::size_t n){return std::fread(p,s,n,f)==n;}
static bool read_window(FILE*f,struct51_T&w){std::uint32_t m[7];double s[5];
    if(!take(f,m,4,7)||!take(f,w.reference_asset_sha256,1,32)||!take(f,&w.window_generation,8,1)||!take(f,s,8,5)||
       !take(f,w.time_s,8,256)||!take(f,w.nominal_jet,8,3072)||!take(f,w.prefix_coefficients,8,192)||
       !take(f,w.ground_jet,8,12)||!take(f,w.rest_jet,8,12)||!take(f,w.relaunch_offset_ned_m,8,3))return false;
    w.schema=m[0];w.capacity=static_cast<unsigned short>(m[1]);w.leg_index=m[2];w.source_first_row=m[3];w.source_total_rows=m[4];
    w.row_count=static_cast<unsigned short>(m[5]);w.binding_mode=static_cast<unsigned char>(m[6]);w.nominal_duration_s=s[0];
    w.total_duration_s=s[1];w.prefix_duration_s=s[2];w.relaunch_duration_s=s[3];w.vertical_frame_offset_ned_m=s[4];return true;}
static bool read_inputs(const wchar_t*reference_path,const wchar_t*inner_path){
    FILE*f=_wfopen(reference_path,L"rb");if(!f)return false;char magic[4];std::uint32_t nw=0,nr=0;
    if(!take(f,magic,1,4)||std::memcmp(magic,"RWI1",4)||!take(f,&nw,4,1)||!take(f,&nr,4,1)||nw>32||nr!=3765)return false;
    for(unsigned j=0;j<nw;++j)if(!read_window(f,windows[j]))return false;
    for(auto&r:rows)if(!take(f,r.ids,4,3)||!take(f,&r.sequence,8,1)||!take(f,&r.phase,8,1)||!take(f,r.args,8,11)||
        !take(f,r.jet,8,12)||!take(f,r.expected,8,41))return false;
    std::fclose(f);f=_wfopen(inner_path,L"rb");if(!f)return false;
    if(!take(f,magic,1,4)||std::memcmp(magic,"LCI1",4)||!take(f,&nr,4,1)||nr!=60)return false;
    static double unused[284];unsigned long long tags[2];
    for(auto&input:input_state)if(!take(f,input,8,36)||!take(f,tags,8,2)||!take(f,unused,8,284))return false;
    std::fclose(f);return true;
}
static bool create(){
    if(group)group->~Group();if(numeric)numeric->~Numeric();if(reference)reference->~Reference();
    ref::Configuration c{};const auto&w=windows[rows[0].ids[0]-1];
    std::memcpy(c.reference_asset_sha256,w.reference_asset_sha256,32);c.leg_index=w.leg_index;
    c.initial_phase_acceleration=rows[0].args[1];std::memcpy(c.initial_outer_i,rows[0].args+3,24);c.jerk_limit_mps3=rows[0].args[10];
    reference=new(reference_storage)Reference(workspace,c);numeric=new(numeric_storage)Numeric(workspace);
    group=new(group_storage)Group(*numeric,*reference);return reference->load_window(w);
}
static void flatten(double*p,const struct56_T&r){std::memcpy(p,r.position_m,24);std::memcpy(p+3,r.velocity_mps,24);
    std::memcpy(p+6,r.acceleration_mps2,24);std::memcpy(p+9,r.jerk_mps3,24);}
static bool prepare(unsigned index,bool different_consumed_reference=false){
    const auto&row=rows[index];const auto&w=windows[row.ids[0]-1];ref::Input input{};
    std::memcpy(input.query.reference_asset_sha256,w.reference_asset_sha256,32);input.query.leg_index=w.leg_index;
    input.query.window_generation=w.window_generation;input.query.query_sequence=row.sequence;input.query.progress_s=row.phase;
    input.source_timestamp_ns=1000000000ULL+row.sequence*9000000ULL;input.source_generation=row.sequence;
    input.reference_generation=1;input.outer_generation=1;input.progress_rate=row.args[0];input.target_phase_acceleration=row.args[2];
    std::memcpy(input.target_outer_f,row.args+6,24);input.dt_s=row.args[9];
    if(reference->prepare(input)!=ref::PrepareResult::Candidate)return false;
    static ref::Candidate saved_reference;std::memcpy(&saved_reference,reference->candidate(),sizeof saved_reference);
    double x[36];std::memcpy(x,input_state[index],sizeof x);flatten(x+19,reference->candidate()->transition.reference);
    x[34]=input.dt_s;x[35]=static_cast<double>(w.leg_index);
    if(different_consumed_reference)x[19]+=0.001;
    const unsigned long long tags[2]={input.source_timestamp_ns,input.source_generation};
    const bool ok=numeric->prepare(x,tags);if(index==0)++actual_first_calls;else ++actual_step_calls;
    return ok&&bits(&saved_reference,reference->candidate(),sizeof saved_reference)&&
        bits(numeric->candidate()->consumed_reference_pvaj,x+19,96);
}
struct Receipts{joint::BackendPublication event;num::PublicationReceipt n;ref::PublicationReceipt r;};
static Receipts receipts(){
    Receipts all{};const auto&n=*numeric->candidate();const auto&r=*reference->candidate();
    all.event.source_timestamp_ns=n.original_tags2[0];all.event.source_generation=n.original_tags2[1];
    all.event.output_generation=numeric->installed_count()+1;all.event.original_publication_us=n.original_tags2[0]/1000+1;
    std::memcpy(all.event.actual_control16,n.control16,sizeof n.control16);all.event.output_published=true;
    all.n.source_timestamp_ns=all.event.source_timestamp_ns;all.n.source_generation=all.event.source_generation;
    all.n.output_generation=all.event.output_generation;all.n.original_publication_us=all.event.original_publication_us;
    std::memcpy(all.n.actual_control16,n.control16,sizeof n.control16);all.n.publication_succeeded=all.n.numerical_commit_succeeded=true;
    all.r.source_timestamp_ns=all.event.source_timestamp_ns;all.r.source_generation=all.event.source_generation;
    all.r.output_generation=all.event.output_generation;all.r.original_publication_us=all.event.original_publication_us;
    all.r.candidate_generation=r.candidate_generation;all.r.query_sequence=r.input.query.query_sequence;
    all.r.window_generation=r.input.query.window_generation;all.r.reference_generation=r.input.reference_generation;
    all.r.outer_generation=r.input.outer_generation;flatten(all.r.actual_reference_pvaj,r.transition.reference);
    all.r.publication_succeeded=all.r.numerical_commit_succeeded=true;return all;
}
static bool install(const Receipts&r){return group->install_after_publication(r.event,r.n,r.r);}
int wmain(int argc,wchar_t**argv){
    if(argc!=4||!read_inputs(argv[1],argv[2]))return 2;gpenmpcNative_canonicalLocalInnerFixedFirst_initialize();
    bool ok=create()&&prepare(0);check("real query then actual First share same SD without candidate corruption",ok);
    auto r=receipts();static num::Candidate nc;static ref::Candidate rc;
    std::memcpy(&nc,numeric->candidate(),sizeof nc);std::memcpy(&rc,reference->candidate(),sizeof rc);
    const auto prior=reference->installed_state();const auto installs=numeric->install_attempts();const auto pubs=reference->publication_attempts();
    auto badnum=r.n;badnum.actual_control16[0]+=1;auto badref=r.r;++badref.query_sequence;
    for(unsigned j=0;j<10;++j)ok=ok&&numeric->validate_publication(r.n)&&reference->validate_publication(r.r)&&
        group->validate_publication(r.event,r.n,r.r)&&!numeric->validate_publication(badnum)&&!reference->validate_publication(badref)&&
        !group->validate_publication(r.event,badnum,r.r)&&!group->validate_publication(r.event,r.n,badref);
    check("valid and invalid const validators have no state counter or fault effects",ok&&numeric->install_attempts()==installs&&
        reference->publication_attempts()==pubs&&bits(&nc,numeric->candidate(),sizeof nc)&&bits(&rc,reference->candidate(),sizeof rc)&&
        bits(&prior,&reference->installed_state(),sizeof prior)&&group->diagnostics().attempts==0);
    ok=install(r);check("same actual candidate references and one event install both once",ok&&numeric->installed_count()==1&&reference->installed_count()==1&&
        group->diagnostics().numeric_installs==1&&group->diagnostics().reference_installs==1&&group->diagnostics().unique_publications_reported==1);
    ok=prepare(1);r=receipts();ok=ok&&install(r);
    check("actual Step next candidate jointly installs without GP or clock fabrication",ok&&numeric->installed_count()==2&&reference->installed_count()==2&&
        group->diagnostics().joint_installs==2&&group->diagnostics().unexpected_partial_installs==0);
    for(unsigned kind=0;kind<16;++kind){
        ok=create()&&prepare(0,kind==15);r=receipts();const auto before=reference->installed_state();
        if(kind==0){r.n.actual_control16[0]+=1;r.event.actual_control16[0]=r.n.actual_control16[0];}
        if(kind==1)r.r.actual_reference_pvaj[0]+=1;
        if(kind==2)++r.n.source_generation;
        if(kind==3)++r.r.source_timestamp_ns;
        if(kind==4)++r.r.candidate_generation;
        if(kind==5)++r.r.query_sequence;
        if(kind==6)++r.r.window_generation;
        if(kind==7)++r.r.reference_generation;
        if(kind==8)++r.r.outer_generation;
        if(kind==9)r.n.publication_succeeded=false;
        if(kind==10)r.r.publication_succeeded=false;
        if(kind==11)r.n.numerical_commit_succeeded=false;
        if(kind==12)r.r.numerical_commit_succeeded=false;
        if(kind==13)r.event.output_published=false;
        if(kind==14)++r.r.original_publication_us;
        if(kind==15)check("each receipt valid separately but actual consumed reference differs",numeric->validate_publication(r.n)&&reference->validate_publication(r.r));
        ok=ok&&!install(r)&&numeric->installed_count()==0&&reference->installed_count()==0&&numeric->installed_state64()==nullptr&&
            bits(&before,&reference->installed_state(),sizeof before)&&group->diagnostics().unexpected_partial_installs==0&&
            group->diagnostics().unique_publications_reported==(kind==13?0u:1u)&&numeric->failure()==num::Failure::Publication&&reference->failure()==ref::Failure::Publication;
        const auto calls=numeric->kernel_calls();ok=ok&&!install(r)&&numeric->kernel_calls()==calls&&numeric->installed_count()==0&&reference->installed_count()==0&&
            group->diagnostics().unique_publications_reported==(kind==13?0u:1u)&&!prepare(1);
        check("any bad receipt or consumed bits causes zero installs and irreversible fault",ok);
    }
    {ok=create()&&prepare(0);const auto old=receipts();ok=ok&&install(old)&&prepare(1);
        const auto before=reference->installed_state();double state[64];std::memcpy(state,numeric->installed_state64(),sizeof state);
        ok=ok&&!install(old)&&numeric->installed_count()==1&&reference->installed_count()==1&&bits(state,numeric->installed_state64(),sizeof state)&&
            bits(&before,&reference->installed_state(),sizeof before)&&group->diagnostics().unique_publications_reported==1&&group->diagnostics().duplicate_event_reports==1;
        check("old sequence replay cannot install a fresh candidate or count another output",ok);}
    {ok=create()&&prepare(0);r=receipts();ok=ok&&numeric->install_after_publication(r.n);
        // Detect a pre-existing half-installation as an owner violation.
        ok=ok&&!install(r)&&group->diagnostics().failure==joint::Failure::StatePair&&numeric->installed_count()==1&&reference->installed_count()==0&&
            group->diagnostics().numeric_installs==0&&group->diagnostics().reference_installs==0&&group->diagnostics().unique_publications_reported==1;
        check("preexisting external half-state is detected without hiding its actual count",ok);}
    for(unsigned which=0;which<2;++which){
        ok=create()&&prepare(0);r=receipts();
        // Backend action was reported, then the caller erroneously tried
        // another prepare before delivering its receipt. Keep the first
        // actual numerical fault when the late group receipt is handled.
        if(which==0){const unsigned long long tags[2]={1,1};ok=ok&&!numeric->prepare(input_state[0],tags);}
        else{ref::Input i{};ok=ok&&reference->prepare(i)==ref::PrepareResult::Rejected;}
        const auto nf=numeric->failure();const auto rf=reference->failure();
        ok=ok&&!install(r)&&numeric->installed_count()==0&&reference->installed_count()==0&&group->diagnostics().unique_publications_reported==1;
        if(which==0)ok=ok&&nf==num::Failure::PendingCandidate&&numeric->failure()==nf;
        else ok=ok&&rf==ref::Failure::PendingCandidate&&reference->failure()==rf;
        check("preexisting first numerical/query failure survives late published receipt",ok);
    }
    FILE*out=_wfopen(argv[3],L"w");if(!out)return 3;
    std::fprintf(out,"{\"pass\":%s,\"checks\":%u,\"passed\":%u,\"actual_First_calls\":%u,\"actual_Step_calls\":%u,\"group_bytes_host\":%zu,\"numeric_store_bytes_host\":%zu,\"reference_store_bytes_host\":%zu,\"shared_SD_bytes_host\":%zu,\"publication_source_clock_are_mocks\":true,\"same_serial_owner_only\":true,\"hardware_atomicity_proven\":false,\"COM_UDP_plant_board_actions\":0}\n",
        checks==passed?"true":"false",checks,passed,actual_first_calls,actual_step_calls,sizeof(Group),sizeof(Numeric),sizeof(Reference),sizeof workspace);
    std::fclose(out);std::printf("JOINT_STATE_INSTALLER %u/%u, First %u Step %u\n",passed,checks,actual_first_calls,actual_step_calls);
    return checks==passed?0:4;
}
