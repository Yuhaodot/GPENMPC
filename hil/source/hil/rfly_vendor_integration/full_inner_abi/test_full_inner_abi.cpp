// Public POD API only: deliberately NO Coder or Store header in this TU.
// Joint source and GP fixture for the full-inner controller ABI.
// Runs the controller C library and GP DLL with mocked publication and source clocks.
#include "CanonicalFullInnerAbi.h"
#include <windows.h>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cwchar>
#include <vector>
#include <cstdint>
static unsigned checks=0,failed=0,rows_done=0,gp_calls=0,exact_rows=0;
static void check(bool ok,const char *s){++checks;if(!ok){++failed;std::fprintf(stderr,"FAIL %s\n",s);}}
static bool read(FILE*f,void*p,size_t n){return std::fread(p,1,n,f)==n;}
static bool same(const void*a,const void*b,size_t n){return std::memcmp(a,b,n)==0;}
struct Row {uint32_t ids[3];uint64_t seq;double phase,args[11],jet[12],expected[41];};
struct Original {uint64_t tags[2];double input[36],transition[41],state[64],kernel[61];float controls[16];
    double request[19],gp[18],oracle_state[64],oracle_kernel[61];float oracle_controls[16];double oracle_gp[18],pending[70],oracle_pending[70];};
static gpenmpc_full_inner_window windows[32];static Row rows[60];static Original original[60];
static uint8_t combined_hashes[60][64];
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
extern "C" void rfi_direct_closed_oracle(const gpenmpc_full_inner_state*,const double[36],const uint64_t[2],double[5],double[12]);
#endif
static unsigned split_rows=0,split_gp_calls=0;
static gpenmpc_full_inner_configuration config{};
static bool load(const wchar_t *rwipath,const wchar_t *sjcpath){FILE*f=_wfopen(rwipath,L"rb");if(!f)return false;
    char magic[4];uint32_t nw=0,nr=0;if(!read(f,magic,4)||std::memcmp(magic,"RWI1",4)||!read(f,&nw,4)||!read(f,&nr,4)||nw>32||nr!=3765)return false;
    for(unsigned i=0;i<nw;++i){auto&w=windows[i];uint32_t m[7];double s[5];
        if(!read(f,m,28)||!read(f,w.reference_asset_sha256,32)||!read(f,&w.window_generation,8)||!read(f,s,40)||
           !read(f,w.time_s,sizeof w.time_s)||!read(f,w.nominal_jet,sizeof w.nominal_jet)||!read(f,w.prefix_coefficients,sizeof w.prefix_coefficients)||
           !read(f,w.ground_jet,96)||!read(f,w.rest_jet,96)||!read(f,w.relaunch_offset_ned_m,24))return false;
        w.schema=m[0];w.capacity=m[1];w.leg_index=m[2];w.source_first_row=m[3];w.source_total_rows=m[4];w.row_count=m[5];w.binding_mode=m[6];
        w.nominal_duration_s=s[0];w.total_duration_s=s[1];w.prefix_duration_s=s[2];w.relaunch_duration_s=s[3];w.vertical_frame_offset_ned_m=s[4];}
    for(auto&r:rows)if(!read(f,r.ids,12)||!read(f,&r.seq,8)||!read(f,&r.phase,8)||!read(f,r.args,88)||!read(f,r.jet,96)||!read(f,r.expected,328))return false;
    std::fclose(f);f=_wfopen(sjcpath,L"rb");if(!f)return false;uint32_t topic=0;
    if(!read(f,magic,4)||std::memcmp(magic,"SJC1",4)||!read(f,&nr,4)||nr!=60||!read(f,&topic,4)||topic>4096)return false;
    std::vector<unsigned char>raw(topic);
    for(auto&r:original)if(!read(f,raw.data(),topic)||!read(f,r.tags,16)||!read(f,r.input,sizeof r.input)||!read(f,r.transition,sizeof r.transition)||
       !read(f,r.state,sizeof r.state)||!read(f,r.kernel,sizeof r.kernel)||!read(f,r.controls,sizeof r.controls)||!read(f,r.request,sizeof r.request)||
       !read(f,r.gp,sizeof r.gp)||!read(f,r.oracle_state,sizeof r.oracle_state)||!read(f,r.oracle_kernel,sizeof r.oracle_kernel)||
       !read(f,r.oracle_controls,sizeof r.oracle_controls)||!read(f,r.oracle_gp,sizeof r.oracle_gp)||!read(f,r.pending,sizeof r.pending)||!read(f,r.oracle_pending,sizeof r.oracle_pending))return false;
    const bool eof=std::fgetc(f)==EOF;std::fclose(f);
    const auto&w=windows[rows[0].ids[0]-1];config.abi_version=1;config.leg_index=w.leg_index;std::memcpy(config.reference_asset_sha256,w.reference_asset_sha256,32);
    config.initial_phase_acceleration=rows[0].args[1];std::memcpy(config.initial_outer_i,rows[0].args+3,24);config.jerk_limit_mps3=rows[0].args[10];
    for(unsigned j=0;j<32;++j){config.task_sha256[j]=static_cast<uint8_t>(j+1);config.configuration_sha256[j]=static_cast<uint8_t>(j+33);}return eof;
}
struct Owner {
    std::vector<uint64_t>storage,scratch;gpenmpc_full_inner_owner *handle{};
    Owner():storage((gpenmpc_full_inner_storage_bytes()+7)/8),scratch((gpenmpc_full_inner_window_scratch_bytes()+7)/8){}
    bool create(){return gpenmpc_full_inner_construct(storage.data(),storage.size()*8,&config,&handle)==RFI_OK&&
        gpenmpc_full_inner_load_window(handle,&windows[rows[0].ids[0]-1],scratch.data(),scratch.size()*8)==RFI_OK;}
};
static gpenmpc_full_inner_reference_input reference_input(unsigned index){const auto&r=rows[index];const auto&w=windows[r.ids[0]-1];gpenmpc_full_inner_reference_input p{};
    std::memcpy(p.reference_asset_sha256,w.reference_asset_sha256,32);p.leg_index=w.leg_index;p.window_generation=w.window_generation;p.query_sequence=r.seq;
    p.source_timestamp_ns=original[index].tags[0];p.source_generation=original[index].tags[1];p.reference_generation=1;p.outer_generation=1;
    p.progress_s=r.phase;p.progress_rate=r.args[0];p.target_phase_acceleration=r.args[2];std::memcpy(p.target_outer_f,r.args+6,24);p.dt_s=r.args[9];return p;
}
static int prepare(Owner&o,unsigned i,gpenmpc_full_inner_candidate&c){const auto p=reference_input(i);return gpenmpc_full_inner_prepare(o.handle,original[i].input,original[i].tags,&p,&c);}
struct Receipts {gpenmpc_full_inner_backend e{};gpenmpc_full_inner_numeric_receipt n{};gpenmpc_full_inner_reference_receipt r{};};
static Receipts receipts(const gpenmpc_full_inner_candidate&c){Receipts a;
    a.e.source_timestamp_ns=a.n.source_timestamp_ns=a.r.source_timestamp_ns=c.original_tags2[0];
    a.e.source_generation=a.n.source_generation=a.r.source_generation=c.original_tags2[1];
    a.e.output_generation=a.n.output_generation=a.r.output_generation=c.original_tags2[1];
    a.e.original_publication_us=a.n.original_publication_us=a.r.original_publication_us=c.original_tags2[0]/1000+400;
    std::memcpy(a.e.actual_control16,c.control16,64);std::memcpy(a.n.actual_control16,c.control16,64);std::memcpy(a.r.actual_reference_pvaj,c.consumed_reference_pvaj,96);
    a.e.output_published=a.n.publication_succeeded=a.n.numerical_commit_succeeded=a.r.publication_succeeded=a.r.numerical_commit_succeeded=1;
    a.r.candidate_generation=c.reference_candidate_generation;a.r.query_sequence=c.reference_query_sequence;a.r.window_generation=c.reference_window_generation;
    a.r.reference_generation=c.reference_generation;a.r.outer_generation=c.outer_generation;return a;
}
static int commit(Owner&o,const Receipts&a){return gpenmpc_full_inner_commit(o.handle,&a.e,&a.n,&a.r);}
int wmain(int argc,wchar_t**argv){
    if(argc==4&&std::wcscmp(argv[3],L"--operator-yaw-prior-state")==0){
        if(!load(argv[1],argv[2]))return 2;
        Owner owner;if(!owner.create())return 3;
        gpenmpc_full_inner_state before{},prepared{},installed{},next_prepared{};
        gpenmpc_full_inner_reference_candidate ref{};gpenmpc_full_inner_candidate candidate{};
        auto p=reference_input(0);p.progress_rate=1;p.target_phase_acceleration=.25;
        std::memset(p.target_outer_f,0,sizeof p.target_outer_f);
        check(gpenmpc_full_inner_copy_state(owner.handle,&before)==RFI_OK,"initial committed state available");
        check(gpenmpc_full_inner_prepare_operator_reference(owner.handle,&p,&ref)==RFI_OK,"first operator reference prepares");
        check(gpenmpc_full_inner_copy_state(owner.handle,&prepared)==RFI_OK&&same(before.committed_state_sha256,prepared.committed_state_sha256,32),
              "pending first operator/yaw selection cannot change prior committed hash");
        double input[36];std::memcpy(input,original[0].input,sizeof input);
        std::memcpy(input+19,ref.actual_reference_pvaj,96);
        input[12]=5.0; // Yaw saturation fixture.
        check(gpenmpc_full_inner_prepare_numeric(owner.handle,input,original[0].tags,&ref,&candidate)==RFI_OK,
              "actual selected numerical facade prepares operator command");
        check(same(before.committed_state_sha256,candidate.prior_committed_state_sha256,32),"numeric candidate binds the original prior state");
        double collective=0;for(unsigned j=0;j<6;++j)collective+=candidate.kernel61[4+j];
        check(std::fabs(collective-candidate.kernel61[0])<1e-8&&candidate.kernel61[60]==1,
              "actual linked manual allocator retains collective during saturated yaw");
        check(commit(owner,receipts(candidate))==RFI_OK,"mock output publication permits joint install");
        check(gpenmpc_full_inner_copy_state(owner.handle,&installed)==RFI_OK&&installed.reference_committed&&installed.numeric_installed,
              "only successful joint install changes committed state");
        p=reference_input(1);p.progress_rate=1;p.target_phase_acceleration=-.25;
        p.dt_s=.4;p.source_timestamp_ns=original[0].tags[0]+400000000;
        p.source_generation=original[0].tags[1]+1;
        std::memset(p.target_outer_f,0,sizeof p.target_outer_f);
        check(gpenmpc_full_inner_prepare_operator_reference(owner.handle,&p,&ref)==RFI_OK,"next opposite-yaw reference prepares");
        check(gpenmpc_full_inner_copy_state(owner.handle,&next_prepared)==RFI_OK&&same(installed.committed_state_sha256,next_prepared.committed_state_sha256,32),
              "next pending yaw also leaves installed state unchanged");
        std::memcpy(input+19,ref.actual_reference_pvaj,96);input[34]=.4;
        const uint64_t next_tags[2]={p.source_timestamp_ns,p.source_generation};
        check(gpenmpc_full_inner_prepare_numeric(owner.handle,input,next_tags,&ref,&candidate)==RFI_OK,
              "manual numeric path accepts real command-lifetime interval without timestamp replacement");
        std::printf("OPERATOR_YAW_PRIOR_STATE checks=%u failed=%u actual_numeric_calls=2 mock_publications=1\n",checks,failed);
        return failed?1:0;
    }
    if(argc!=5||!load(argv[1],argv[2]))return 2;
    HMODULE dll=LoadLibraryExW(argv[3],nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);if(!dll)return 3;
    using Predict=int(*)(const double*,double*);const auto predict=reinterpret_cast<Predict>(GetProcAddress(dll,"gpenmpc_gp256_predict"));if(!predict)return 3;
    FILE*raw=_wfopen(argv[4],L"wb");if(!raw)return 4;bool written=true;
    Owner owner;check(owner.create(),"opaque POD owner creates three real stores plus one SD");
    gpenmpc_full_inner_build_identity identity{};gpenmpc_full_inner_identity(&identity);uint8_t zero[32]{};
    check(!same(identity.generated_source_set_sha256,zero,32)&&!same(identity.private_archive_sha256,zero,32),"actual compiled generated/archive identity available");
    for(unsigned i=0;i<60;++i){gpenmpc_full_inner_state before{},after{},filled{};gpenmpc_full_inner_candidate c{};
        check(gpenmpc_full_inner_copy_state(owner.handle,&before)==RFI_OK,"actual internal prior state export");
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
        gpenmpc_full_inner_closed_evidence old_closed{},prepared_closed{},installed_closed{},filled_closed{};double expected_closed[5]{},expected_learning[12]{};
        check(gpenmpc_full_inner_copy_closed_evidence(owner.handle,&old_closed)==RFI_OK&&old_closed.exported==1&&old_closed.installed==(i!=0),"closed extension explicitly unavailable before first commit");
        rfi_direct_closed_oracle(&before,original[i].input,original[i].tags,expected_closed,expected_learning);
#endif
        const int ok=prepare(owner,i,c);check(ok==RFI_OK,"POD exact reference and numeric preparation");if(ok)break;
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
        check(gpenmpc_full_inner_copy_closed_evidence(owner.handle,&prepared_closed)==RFI_OK&&same(&old_closed,&prepared_closed,sizeof old_closed),"numeric preparation cannot expose uncommitted closed evidence");
#endif
        check(same(before.committed_state_sha256,c.prior_committed_state_sha256,32),"candidate binds actual old state plus pending plus reference hash");
        check(same(c.post_state64,original[i].state,512)&&same(c.kernel61,original[i].kernel,488)&&same(c.control16,original[i].controls,64)&&
            same(c.request19,original[i].request,152)&&same(c.consumed_reference_pvaj,original[i].input+19,96)&&!c.control_authority,"same raw full closure 64/61/16/request/reference bits");
        ++exact_rows;const auto a=receipts(c);check(commit(owner,a)==RFI_OK,"original receipt PODs jointly install with mock publication");
        gpenmpc_full_inner_copy_state(owner.handle,&after);check(after.numeric_installed&&after.reference_committed&&same(after.state64,c.post_state64,512),"joint installed actual state");
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
        check(gpenmpc_full_inner_copy_closed_evidence(owner.handle,&installed_closed)==RFI_OK&&installed_closed.exported&&installed_closed.installed&&
            same(installed_closed.values5,expected_closed,sizeof expected_closed)&&same(installed_closed.original_installed_tags2,c.original_tags2,16)&&
            installed_closed.original_publication_us==a.e.original_publication_us,"closed values match direct generated math and exact successful publication");
        check(i?installed_closed.values5[1]==1.0&&installed_closed.values5[2]==1.0:installed_closed.values5[0]==0.0,"first closed record unavailable; later innovation actually closed");
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
        check(installed_closed.learning_exported==1&&same(installed_closed.learning12,expected_learning,sizeof expected_learning),
            "actual committed innovation and per-axis physical learning bit-exact to direct generated oracle");
#endif
#endif
        if(after.prediction_required){double gp[18];++gp_calls;check(predict(c.request19+1,gp)==0&&same(gp,original[i].gp,144),"actual original GP18 same-input bits");
            check(gpenmpc_full_inner_fill_gp(owner.handle,c.original_tags2,gp)==RFI_OK,"POD exact generation fills only original open pending");}
        gpenmpc_full_inner_copy_state(owner.handle,&filled);check(same(filled.pending70,original[i].pending,560),"next-step actual pending70 exact");
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
        check(gpenmpc_full_inner_copy_closed_evidence(owner.handle,&filled_closed)==RFI_OK&&same(&installed_closed,&filled_closed,sizeof installed_closed),"next open GP fill cannot alter current closed evidence or publication provenance");
#endif
        if(after.prediction_required)check(!same(after.committed_state_sha256,filled.committed_state_sha256,32)&&same(after.state64,filled.state64,512),"GP fill changes pending provenance without changing installed observer state");
        written=written&&std::fwrite(c.prior_committed_state_sha256,1,32,raw)==32&&std::fwrite(c.full_inner_input_sha256,1,32,raw)==32;
        std::memcpy(combined_hashes[i],c.prior_committed_state_sha256,32);std::memcpy(combined_hashes[i]+32,c.full_inner_input_sha256,32);
        ++rows_done;
    }
    std::fclose(raw);gpenmpc_full_inner_diagnostics d{};gpenmpc_full_inner_diagnose(owner.handle,&d);
    check(rows_done==60&&exact_rows==60&&gp_calls==59&&d.kernel_calls==60&&d.joint_installs==60&&d.partial_installs==0&&written,"all 60 rows and 59 real GP fills complete");
    {Owner o;gpenmpc_full_inner_candidate c{};check(o.create()&&prepare(o,0,c)==RFI_OK,"negative setup");auto a=receipts(c);a.e.output_published=0;
#ifdef GPENMPC_CANONICAL_CLOSED_EVIDENCE
        gpenmpc_full_inner_closed_evidence first{};check(gpenmpc_full_inner_copy_closed_evidence(o.handle,&first)==RFI_OK&&!first.installed,"unpublished candidate has no committed closed evidence");
#endif
        check(commit(o,a)!=RFI_OK,"missing actual publication cannot install");gpenmpc_full_inner_diagnose(o.handle,&d);
        check(d.joint_installs==0&&d.numeric_installs==0&&d.reference_installs==0,"no half state install on failed event");}
    {Owner o;gpenmpc_full_inner_candidate c{};o.create();prepare(o,0,c);auto a=receipts(c);a.n.actual_control16[0]+=1;
        check(commit(o,a)!=RFI_OK,"wrong output bits reject");gpenmpc_full_inner_diagnose(o.handle,&d);
        check(d.unique_publications==1&&d.numeric_installs==0,"already reported publication remains in ledger");}
    {Owner o;gpenmpc_full_inner_candidate c{};o.create();prepare(o,0,c);auto a=receipts(c);commit(o,a);
        check(commit(o,a)!=RFI_OK,"duplicate commit reject");gpenmpc_full_inner_diagnose(o.handle,&d);
        check(d.numeric_installs==1&&d.duplicate_reports==1,"duplicate event not duplicate install");}
    {Owner o;gpenmpc_full_inner_candidate c{};o.create();prepare(o,0,c);commit(o,receipts(c));prepare(o,1,c);commit(o,receipts(c));
        check(prepare(o,2,c)!=RFI_OK,"missing per-inner GP stops next numeric candidate");gpenmpc_full_inner_diagnose(o.handle,&d);
        check(d.kernel_calls==2&&d.numeric_installs==2&&d.numeric_failure==4,"original MissingPrediction retained");}
    {Owner o;gpenmpc_full_inner_candidate c{};o.create();auto p=reference_input(0);double x[36];std::memcpy(x,original[0].input,sizeof x);x[19]+=1;
        check(gpenmpc_full_inner_prepare(o.handle,x,original[0].tags,&p,&c)==RFI_REFERENCE_BITS,"no reference re-interpolation or silent replacement");}
    {Owner a,b;gpenmpc_full_inner_candidate ca{},cb{};a.create();b.create();prepare(a,0,ca);
        auto p=reference_input(0);double x[36];std::memcpy(x,original[0].input,sizeof x);x[0]+=0.001;
        check(gpenmpc_full_inner_prepare(b.handle,x,original[0].tags,&p,&cb)==RFI_OK&&
            same(ca.prior_committed_state_sha256,cb.prior_committed_state_sha256,32)&&!same(ca.full_inner_input_sha256,cb.full_inner_input_sha256,32),"full input hash responds to actual source field without falsifying prior state");}
    {Owner o;o.create();gpenmpc_full_inner_owner *reused=nullptr;check(gpenmpc_full_inner_construct(o.storage.data(),o.storage.size()*8,&config,&reused)==RFI_ARGUMENT,"live storage cannot be reconstructed");
        check(gpenmpc_full_inner_retire(o.handle)==RFI_OK&&gpenmpc_full_inner_construct(o.storage.data(),o.storage.size()*8,&config,&reused)==RFI_ARGUMENT,"retired storage cannot silently reset");}
    const unsigned legacy_checks=checks,legacy_failed=failed;
    // The actual board-local caller need not know the new p/v/a/j in advance.
    // Only the single real reference candidate fills those twelve input slots.
    {Owner o;check(o.create(),"split owner setup");
        for(unsigned i=0;i<60;++i){const auto p=reference_input(i);gpenmpc_full_inner_reference_candidate rc{};
            gpenmpc_full_inner_state before{},after{};gpenmpc_full_inner_copy_state(o.handle,&before);
            const int ref_ok=gpenmpc_full_inner_prepare_reference(o.handle,&p,&rc);check(ref_ok==RFI_OK,"one reference query/transition before input builder");if(ref_ok)break;
            gpenmpc_full_inner_copy_state(o.handle,&after);gpenmpc_full_inner_diagnose(o.handle,&d);
            check(d.kernel_calls==i&&d.joint_installs==i&&same(before.committed_state_sha256,after.committed_state_sha256,32),"reference alone neither executes inner nor installs state");
            check(rc.candidate_generation==i+1&&rc.query_sequence==rows[i].seq&&rc.source_timestamp_ns==original[i].tags[0]&&rc.source_generation==original[i].tags[1]&&
                same(&rc.phase_after_s,&rows[i].phase,8)&&same(&rc.phase_before_s,&before.reference.query_progress_s,8)&&!rc.control_authority,"actual original reference candidate identity and phase endpoints");
            double x[36];std::memcpy(x,original[i].input,sizeof x);std::memset(x+19,0,96);std::memcpy(x+19,rc.actual_reference_pvaj,96);
            gpenmpc_full_inner_candidate c{};const int num_ok=gpenmpc_full_inner_prepare_numeric(o.handle,x,original[i].tags,&rc,&c);
            check(num_ok==RFI_OK,"numeric step uses matching pending reference without another query");if(num_ok)break;
            check(same(c.post_state64,original[i].state,512)&&same(c.kernel61,original[i].kernel,488)&&same(c.control16,original[i].controls,64)&&
                same(c.prior_committed_state_sha256,combined_hashes[i],32)&&same(c.full_inner_input_sha256,combined_hashes[i]+32,32),"two-step outputs and full provenance are bit exact with combined path");
            check(commit(o,receipts(c))==RFI_OK,"two-step candidates install only after mock joint publication");gpenmpc_full_inner_copy_state(o.handle,&after);
            if(after.prediction_required){double gp[18];++split_gp_calls;check(predict(c.request19+1,gp)==0&&same(gp,original[i].gp,144)&&
                gpenmpc_full_inner_fill_gp(o.handle,c.original_tags2,gp)==RFI_OK,"two-step actual original GP reply");}
            gpenmpc_full_inner_copy_state(o.handle,&after);check(same(after.pending70,original[i].pending,560),"two-step k GP closes only for next source");++split_rows;
        }
        gpenmpc_full_inner_diagnose(o.handle,&d);check(split_rows==60&&split_gp_calls==59&&d.kernel_calls==60&&d.joint_installs==60&&d.partial_installs==0,"complete new two-step 60/59 path");}
    {Owner o;o.create();gpenmpc_full_inner_candidate c{};gpenmpc_full_inner_reference_candidate rc{};
        check(gpenmpc_full_inner_prepare_numeric(o.handle,original[0].input,original[0].tags,&rc,&c)==RFI_REFERENCE_IDENTITY,"numeric cannot invent a missing reference candidate");
        gpenmpc_full_inner_diagnose(o.handle,&d);check(d.kernel_calls==0&&d.joint_installs==0,"missing reference has zero inner/install");}
    {Owner o;o.create();auto p=reference_input(0);gpenmpc_full_inner_reference_candidate rc{};
        check(gpenmpc_full_inner_prepare_reference(o.handle,&p,&rc)==RFI_OK&&gpenmpc_full_inner_prepare_reference(o.handle,&p,&rc)==RFI_STORE_FAILURE,"duplicate reference prepare fails closed");
        gpenmpc_full_inner_diagnose(o.handle,&d);check(d.kernel_calls==0&&d.joint_installs==0&&d.reference_failure!=0,"duplicate reference retains original store failure without install");}
    for(unsigned kind=0;kind<12;++kind){Owner o;o.create();auto p=reference_input(0);gpenmpc_full_inner_reference_candidate rc{};gpenmpc_full_inner_candidate c{};
        gpenmpc_full_inner_prepare_reference(o.handle,&p,&rc);
        if(kind==0)++rc.source_timestamp_ns;if(kind==1)++rc.source_generation;if(kind==2)++rc.reference_generation;
        if(kind==3)++rc.outer_generation;if(kind==4)++rc.window_generation;if(kind==5)++rc.query_sequence;
        if(kind==6)++rc.candidate_generation;if(kind==7)++rc.leg_index;if(kind==8)rc.reference_asset_sha256[0]^=1;
        if(kind==9)rc.phase_before_s+=1;if(kind==10)rc.phase_after_s+=1;if(kind==11)rc.control_authority=1;
        check(gpenmpc_full_inner_prepare_numeric(o.handle,original[0].input,original[0].tags,&rc,&c)==RFI_REFERENCE_IDENTITY,"each reference identity field must match original pending candidate");
        gpenmpc_full_inner_diagnose(o.handle,&d);check(d.kernel_calls==0&&d.joint_installs==0,"wrong reference identity rejected before numerical execution/install");}
    for(unsigned kind=0;kind<2;++kind){Owner o;o.create();auto p=reference_input(0);gpenmpc_full_inner_reference_candidate rc{};gpenmpc_full_inner_candidate c{};
        gpenmpc_full_inner_prepare_reference(o.handle,&p,&rc);double x[36];std::memcpy(x,original[0].input,sizeof x);
        if(kind)rc.actual_reference_pvaj[0]+=1;else x[19]+=1;
        check(gpenmpc_full_inner_prepare_numeric(o.handle,x,original[0].tags,&rc,&c)==RFI_REFERENCE_BITS,"echoed reference and built input each require actual pvaj bits");
        gpenmpc_full_inner_diagnose(o.handle,&d);check(d.kernel_calls==0&&d.joint_installs==0,"wrong reference bits never execute/install");}
    {Owner o;o.create();auto p=reference_input(0);gpenmpc_full_inner_reference_candidate rc{};gpenmpc_full_inner_candidate c{};
        gpenmpc_full_inner_prepare_reference(o.handle,&p,&rc);gpenmpc_full_inner_prepare_numeric(o.handle,original[0].input,original[0].tags,&rc,&c);
        check(gpenmpc_full_inner_prepare_numeric(o.handle,original[0].input,original[0].tags,&rc,&c)==RFI_STORE_FAILURE,"numeric candidate cannot execute twice");
        gpenmpc_full_inner_diagnose(o.handle,&d);check(d.kernel_calls==1&&d.joint_installs==0,"duplicate numeric preserves once-only kernel count");}
    {Owner o;check(gpenmpc_full_inner_construct(o.storage.data(),o.storage.size()*8,&config,&o.handle)==RFI_OK,"window-miss owner has no preloaded window");
        auto p=reference_input(0);gpenmpc_full_inner_reference_candidate rc{};gpenmpc_full_inner_state before{},after{};
        gpenmpc_full_inner_copy_state(o.handle,&before);
        check(gpenmpc_full_inner_prepare_reference(o.handle,&p,&rc)==RFI_REFERENCE_WINDOW_MISS&&!rc.candidate_generation,"window miss creates no fake pending state");
        gpenmpc_full_inner_copy_state(o.handle,&after);check(same(before.committed_state_sha256,after.committed_state_sha256,32),"window miss leaves committed numerical/reference states unchanged");
        check(gpenmpc_full_inner_load_window(o.handle,&windows[rows[0].ids[0]-1],o.scratch.data(),o.scratch.size()*8)==RFI_OK&&
            gpenmpc_full_inner_prepare_reference(o.handle,&p,&rc)==RFI_OK&&rc.candidate_generation==1,"original refill followed by first real query works without reset");}
    FreeLibrary(dll);
    std::printf("{\"checks\":%u,\"failed\":%u,\"legacy_checks\":%u,\"legacy_failed\":%u,\"rows\":%u,\"exact_rows\":%u,\"actual_GP_calls\":%u,\"two_step_rows\":%u,\"two_step_GP_calls\":%u,\"resident_bytes\":%zu,\"scratch_bytes\":%zu,\"source_and_publication_mock\":true,\"authority\":false,\"COM\":0}\n",checks,failed,legacy_checks,legacy_failed,rows_done,exact_rows,gp_calls,split_rows,split_gp_calls,gpenmpc_full_inner_storage_bytes(),gpenmpc_full_inner_window_scratch_bytes());return failed?1:0;
}
