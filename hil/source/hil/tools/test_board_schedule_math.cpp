// Test the generated C and GP256 library with mock sources, clocks,
// reference/rotor providers and publication.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>
#include "../rfly_vendor_integration/BoardLocalInnerSchedule.hpp"
#include "../rfly_vendor_integration/CanonicalLocalInnerStateStore.hpp"
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_initialize.h"
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_terminate.h"
#include "canonical_gp_standalone_api.h"
#if !defined(GPENMPC_CANONICAL_EXPLICIT_WORKSPACE) || GPENMPC_CANONICAL_EXPLICIT_WORKSPACE != 1
#error Actual combined C requires its explicit resident Workspace ABI
#endif
namespace sch=gpenmpc_local_schedule;
namespace math=gpenmpc_local_math;
using Schedule=sch::BoardLocalInnerSchedule<>;
using Predict=int(*)(const double*,double*);
struct Row { double input[36];unsigned long long tags[2];double prior[70],state[64],kernel[61],scaffold[70],request[19]; };
struct Comparison {
    unsigned values{},bad{},exact_rows{};double maximum_absolute{},maximum_scaled{};
    void add(const double*a,const double*b,unsigned n) {
        exact_rows+=std::memcmp(a,b,n*sizeof(double))==0;
        for(unsigned j=0;j<n;++j){++values;
            if(std::isnan(a[j])!=std::isnan(b[j])||std::isfinite(a[j])!=std::isfinite(b[j])||
               (std::isinf(b[j])&&std::signbit(a[j])!=std::signbit(b[j]))){++bad;continue;}
            if(std::isfinite(b[j])){const double d=std::fabs(a[j]-b[j]),s=d/std::fmax(1.0,std::fabs(b[j]));
                maximum_absolute=std::fmax(maximum_absolute,d);maximum_scaled=std::fmax(maximum_scaled,s);
                if(s>1e-10)++bad; // inherited generated-C comparison, frozen before this run
            }
        }
    }
    void write(FILE*f,const char*name,bool comma)const {
        std::fprintf(f,"\"%s\":{\"values\":%u,\"bad\":%u,\"exact_rows\":%u,\"max_abs\":%.17g,\"max_scaled\":%.17g}%s\n",
            name,values,bad,exact_rows,maximum_absolute,maximum_scaled,comma?",":"");
    }
};
static FILE*checks_file{};static unsigned checks{},bad{};
static void check(const char*n,bool pass){++checks;bad+=!pass;std::fprintf(checks_file,"%s,%u\n",n,unsigned(pass));}
static bool bits(const void*a,const void*b,std::size_t n){return std::memcmp(a,b,n)==0;}
static sch::Hash mock_hash(unsigned n){sch::Hash h{};h[0]=n;return h;}
static sch::Configuration configuration() {
    sch::Configuration c{};c.observed_session={17,19,1,1};c.task_sha=mock_hash(1);
    c.configuration_sha=mock_hash(2);c.gp_model_sha=mock_hash(3);
    // Test-event limits only, NOT a production profile or measured throughput.
    c.source_max_age_us=1000;c.gp_reply_max_age_us=20000;c.initial_global_leg=1;
    c.initial_dt_s=.01;c.maximum_reference_lookahead_s=1;c.original_reset_counter=0;return c;
}
static double mock_first_window_end() {
    // Same exact saved per-step phase progression as this MOCK provider. A
    // decimal 0.3 is not the bit-identical sum of thirty 0.01 increments.
    double phase=0;for(unsigned k=0;k<30;++k)phase+=.01;return phase;
}
static sch::ReferenceWindow window(const sch::Configuration&c,unsigned gen,double first,double last) {
    sch::ReferenceWindow w{};w.task_sha=c.task_sha;w.configuration_sha=c.configuration_sha;
    w.asset_sha=mock_hash(4);w.leg=1;w.generation=gen;w.original_asset_token=10+gen;
    w.first_phase_s=first;w.last_phase_s=last;return w;
}
static sch::Estimator source(const Row&r,const sch::Configuration&c) {
    sch::Estimator s{};s.identity=c.observed_session;s.generation=r.tags[1];
    // Exact conversion of archived numerical ns to a MOCK us domain. The
    // fixture is checked divisible by 1000. This is NOT a HOST-to-HRT map.
    s.timestamp_sample_us=r.tags[0]/1000;s.publication_us=s.timestamp_sample_us;
    s.board_rx_us=s.timestamp_sample_us+1;s.pose_frame=s.velocity_frame=1;
    const double c3[3]={1,1,-1},q4[4]={1,-1,-1,1},omega3[3]={-1,-1,1};
    for(unsigned j=0;j<3;++j){s.p[j]=c3[j]*r.input[j];s.v[j]=c3[j]*r.input[j+3];s.body_rates[j]=omega3[j]*r.input[j+10];}
    for(unsigned j=0;j<4;++j)s.q[j]=q4[j]*r.input[j+6];return s;
}
struct MockPorts final:sch::Ports {
    // One heap-resident generated workspace per numerical owner, no union
    // aliasing with another owner or GP DLL, and no per-tick allocation.
    std::unique_ptr<math::CanonicalLocalInnerStateStore::Workspace> workspace{
        new math::CanonicalLocalInnerStateStore::Workspace{}};
    math::CanonicalLocalInnerStateStore store{*workspace};
    const Row*row{};std::uint64_t now{};unsigned long long committed_tags[2]{};
    bool authorized{true},rotor_available{true},revoked{};
    unsigned publication_variant{},reconstruct_exact{},candidate_before_install{},prior_links{},prior_errors{};
    double last_inner_ms{},last_install_ms{};std::uint64_t last_mock_commit_us{};math::Candidate last_candidate{};
    LARGE_INTEGER frequency{};
    MockPorts(){QueryPerformanceFrequency(&frequency);}
    std::uint64_t actual_now_us()noexcept override{return now;}
    bool authority(const sch::Identity&,std::uint64_t)noexcept override{return authorized&&!revoked;}
    bool observed_disarmed(std::uint64_t)noexcept override{return authorized&&!revoked;}
    bool local_source_verified(const sch::Estimator&)noexcept override{return row!=nullptr;}
    bool rotor_for(const sch::Estimator&s,sch::RotorLag&r)noexcept override {
        if(!rotor_available||!row)return false;
        for(unsigned j=0;j<6;++j)r.observed_thrust_n[j]=row->input[13+j];
        r.dll_generation=s.generation;r.dll_session=123;r.original_host_receive_ns=row->tags[0];
        r.original_board_ingress_us=s.board_rx_us;r.original_sim_time_s=double(row->tags[0])*1e-9;
        r.original_observation_sha=mock_hash(5);return true; // Synthetic association.
    }
    bool reference_for(const sch::Estimator&s,const sch::ReferenceWindow&w,double phase,bool,double dt,
                       sch::ReferenceCandidate&r)noexcept override {
        if(!row)return false;
        for(unsigned j=0;j<3;++j){r.position_m[j]=row->input[19+j];r.velocity_mps[j]=row->input[22+j];
            r.acceleration_mps2[j]=row->input[25+j];r.jerk_mps3[j]=row->input[28+j];}
        r.leg=w.leg;r.window_generation=w.generation;r.original_source_generation=s.generation;
        r.candidate_token=s.generation;r.phase_before_s=phase;r.phase_after_s=phase+dt;
        return true; // Replay the fixture jet.
    }
    sch::CommitResult local_step(const sch::Estimator&s,const sch::RotorLag&rotor,const sch::ReferenceCandidate&ref,
                                  const sch::GpReply*prior,double dt,bool new_leg)noexcept override {
        sch::CommitResult result{};if(!row)return result;
        if(prior){++prior_links;
            if(prior->source_generation!=committed_tags[1]||prior->source_sample_us*1000!=committed_tags[0]||
               !store.prediction_ready()||prior->source_generation>=s.generation||prior->prediction_sample_closed||
               prior->observed_innovation_available)++prior_errors;
        }else if(store.prediction_required())++prior_errors;
        if(new_leg!=(store.installed_count()==0))++prior_errors;
        double input[36]{};const double c3[3]={1,1,-1},q4[4]={1,-1,-1,1},o3[3]={-1,-1,1};
        for(unsigned j=0;j<3;++j){input[j]=c3[j]*s.p[j];input[3+j]=c3[j]*s.v[j];input[10+j]=o3[j]*s.body_rates[j];
            input[19+j]=ref.position_m[j];input[22+j]=ref.velocity_mps[j];input[25+j]=ref.acceleration_mps2[j];input[28+j]=ref.jerk_mps3[j];}
        for(unsigned j=0;j<4;++j)input[6+j]=q4[j]*s.q[j];
        for(unsigned j=0;j<6;++j)input[13+j]=rotor.observed_thrust_n[j];
        input[31]=row->input[31];input[32]=row->input[32];input[33]=row->input[33];input[34]=dt;input[35]=double(ref.leg);
        const unsigned long long tags[2]={s.timestamp_sample_us*1000,s.generation};
        reconstruct_exact+=bits(input,row->input,sizeof input)&&bits(tags,row->tags,sizeof tags);
        double old[64]{};const bool installed=store.installed_state64()!=nullptr;
        if(installed)std::memcpy(old,store.installed_state64(),sizeof old);
        LARGE_INTEGER a,b,c;QueryPerformanceCounter(&a);
        const auto before_calls=store.kernel_calls();const bool prepared=store.prepare(input,tags);
        result.kernel_called=store.kernel_calls()!=before_calls;QueryPerformanceCounter(&b);
        last_inner_ms=double(b.QuadPart-a.QuadPart)*1000.0/double(frequency.QuadPart);
        if(!prepared||!store.candidate())return result;
        last_candidate=*store.candidate();
        candidate_before_install+=installed?bits(old,store.installed_state64(),sizeof old):store.installed_state64()==nullptr;
        math::PublicationReceipt receipt{};receipt.source_timestamp_ns=tags[0];receipt.source_generation=tags[1];
        receipt.output_generation=store.installed_count()+1;receipt.original_publication_us=now+1;
        receipt.publication_succeeded=publication_variant!=1;receipt.numerical_commit_succeeded=publication_variant!=2;
        std::memcpy(receipt.actual_control16,last_candidate.control16,sizeof receipt.actual_control16);
        if(publication_variant==3)receipt.actual_control16[15]=1; // Mismatched payload fixture.
        result.output_published=receipt.publication_succeeded;
        result.output_committed=store.install_after_publication(receipt);now=receipt.original_publication_us;
        result.original_commit_us=result.output_committed?now:0;
        last_mock_commit_us=result.original_commit_us;
        if(result.output_committed){std::memcpy(committed_tags,tags,sizeof tags);
            result.prediction_required=store.prediction_required();
            if(result.prediction_required)for(unsigned j=0;j<17;++j)result.features_f17[j]=store.gp_request19()[j+1];}
        if(publication_variant==4)now+=1001; // Late postcheck fixture.
        QueryPerformanceCounter(&c);last_install_ms=double(c.QuadPart-b.QuadPart)*1000.0/double(frequency.QuadPart);
        return result;
    }
    void revoke()noexcept override{revoked=true;}
};
struct Fixture {
    sch::Configuration cfg{configuration()};MockPorts ports;Schedule schedule{cfg,&ports};
    bool start(){ports.now=1;return schedule.begin_leg(1,1)&&schedule.add_reference_window(window(cfg,1,0,mock_first_window_end()),1)&&schedule.resume(1);}
    sch::Event step(const Row&r){ports.row=&r;auto s=source(r,cfg);ports.now=s.board_rx_us;return schedule.local_source(s,ports.now);}
};
static sch::GpReply make_reply(const sch::GpQuery&q,Predict predict,bool&success,double&elapsed) {
    sch::GpReply r{};r.request_sha=q.request_sha;r.model_sha=q.model_sha;r.session=q.session;
    r.source_generation=q.source_generation;r.source_sample_us=q.source_sample_us;r.original_commit_us=q.original_commit_us;
    r.original_valid_until_us=q.original_valid_until_us;
    double features[17],values[18];for(unsigned j=0;j<17;++j)features[j]=q.features_f17[j];
    LARGE_INTEGER a,b,f;QueryPerformanceFrequency(&f);QueryPerformanceCounter(&a);
    success=predict(features,values)==GPENMPC_GP256_OK;QueryPerformanceCounter(&b);
    elapsed=double(b.QuadPart-a.QuadPart)*1000.0/double(f.QuadPart);
    for(unsigned j=0;j<18;++j)r.original_output18[j]=values[j];return r;
}
static sch::Event accept_reply(Fixture&f,const sch::GpReply&r,std::uint64_t ingress) {
    const auto event=f.schedule.gp_reply(r,ingress,f.ports.now);
    if(event==sch::Event::Stored){double result[18];for(unsigned j=0;j<18;++j)result[j]=r.original_output18[j];
        // Fill installed slot k only from its scheduler-accepted observation.
        if(r.source_generation!=f.ports.committed_tags[1]||r.source_sample_us*1000!=f.ports.committed_tags[0]||
           !f.ports.store.fill_open_prediction(f.ports.committed_tags,result))return sch::Event::Rejected;
    }return event;
}
static bool first_two(Fixture&f,const std::vector<Row>&rows) {
    return f.start()&&f.step(rows[0])==sch::Event::Committed&&f.step(rows[1])==sch::Event::Committed;
}
int wmain(int argc,wchar_t**argv) {
    if(argc!=7)return 2;
    FILE*input=_wfopen(argv[1],L"rb");checks_file=_wfopen(argv[3],L"wb");
    FILE*timing=_wfopen(argv[4],L"wb"),*raw=_wfopen(argv[5],L"wb");if(!input||!checks_file||!timing||!raw)return 3;
    char magic[4];unsigned count{};if(std::fread(magic,1,4,input)!=4||std::memcmp(magic,"LCI1",4)||std::fread(&count,4,1,input)!=1||count!=60)return 4;
    std::vector<Row>rows(count);for(auto&r:rows){
        if(std::fread(r.input,8,36,input)!=36||std::fread(r.tags,8,2,input)!=2||std::fread(r.prior,8,70,input)!=70||
           std::fread(r.state,8,64,input)!=64||std::fread(r.kernel,8,61,input)!=61||std::fread(r.scaffold,8,70,input)!=70||std::fread(r.request,8,19,input)!=19)return 5;
        if(r.tags[0]%1000)return 6;
    }if(std::fgetc(input)!=EOF)return 7;std::fclose(input);
    HMODULE dll=LoadLibraryExW(argv[2],nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);if(!dll)return 8;
    const auto predict=reinterpret_cast<Predict>(GetProcAddress(dll,"gpenmpc_gp256_predict"));if(!predict)return 9;
    std::fprintf(checks_file,"check,pass\n");std::fprintf(timing,"row,original_fixture_ns,source_generation,mock_sample_us,mock_commit_us,inner_host_ms,install_host_ms,gp_host_ms,gp_required,prior_reply_passed,gp_active\n");
    std::fwrite("BSM1",1,4,raw);std::fwrite(&count,4,1,raw);
    gpenmpcNative_canonicalLocalInnerFixedFirst_initialize();
    Fixture main;check("mock_disarmed_begin_reference_resume_no_kernel",main.start()&&main.ports.store.kernel_calls()==0);
    Comparison state_c,kernel_c,scaffold_c,request_c,prior_c;unsigned commits{},gp_calls{},duplicates{},reply_duplicates{},exact16{},active{},nondriving{};
    bool main_ok=true;double last_phase=0;
    for(unsigned k=0;k<count;++k){
        auto&f=main;auto&p=f.ports;const auto&r=rows[k];
        if(k){prior_c.add(p.store.open_pending70(),r.prior,70);}
        if(f.step(r)!=sch::Event::Committed){main_ok=false;break;}++commits;
        state_c.add(p.store.installed_state64(),r.state,64);kernel_c.add(p.last_candidate.kernel61,r.kernel,61);
        scaffold_c.add(p.last_candidate.scaffold70,r.scaffold,70);request_c.add(p.last_candidate.request19,r.request,19);
        active+=p.store.installed_state64()[19]!=0;
        float expected[16]{};const unsigned map[6]={4,0,3,5,1,2};for(unsigned j=0;j<6;++j)expected[map[j]]=float(r.kernel[4+j]/32.145727009134916);
        exact16+=bits(expected,p.last_candidate.control16,sizeof expected);
        double installed_before[64];std::memcpy(installed_before,p.store.installed_state64(),sizeof installed_before);
        const auto count_before=p.store.kernel_calls();last_phase=f.schedule.phase_s();
        duplicates+=f.schedule.local_source(source(r,f.cfg),p.now)==sch::Event::Duplicate;
        if(k==20)main_ok&=f.schedule.add_reference_window(window(f.cfg,2,mock_first_window_end(),.8),p.now);
        main_ok&=f.schedule.tick(p.now);
        double gp_ms=0;sch::GpQuery query{};
        if(f.schedule.take_gp_request(query,p.now)){
            ++gp_calls;bool success=false;auto reply=make_reply(query,predict,success,gp_ms);main_ok&=success;
            ++p.now;const auto ingress=p.now;main_ok&=accept_reply(f,reply,ingress)==sch::Event::Stored;
            reply_duplicates+=accept_reply(f,reply,ingress)==sch::Event::Duplicate;
            main_ok&=p.store.open_pending70()[44]==0&&p.store.open_pending70()[45]==0;
        }
        nondriving+=p.store.kernel_calls()==count_before&&f.schedule.phase_s()==last_phase&&bits(installed_before,p.store.installed_state64(),sizeof installed_before);
        std::fprintf(timing,"%u,%llu,%llu,%llu,%llu,%.9f,%.9f,%.9f,%u,%u,%.0f\n",k+1,r.tags[0],r.tags[1],r.tags[0]/1000,
            static_cast<unsigned long long>(p.last_mock_commit_us),p.last_inner_ms,p.last_install_ms,gp_ms,
            unsigned(p.store.prediction_required()),unsigned(k>=2),p.store.installed_state64()[19]);
        // Per-row raw: state64, kernel61, pending70 AFTER original GP reply,
        // request19, float16. No publication/board evidence is encoded.
        std::fwrite(p.store.installed_state64(),8,64,raw);std::fwrite(p.last_candidate.kernel61,8,61,raw);
        std::fwrite(p.store.open_pending70(),8,70,raw);std::fwrite(p.last_candidate.request19,8,19,raw);std::fwrite(p.last_candidate.control16,4,16,raw);
    }
    check("60_local_sources_alone_invoke_actual_combined_C",main_ok&&commits==60&&main.ports.store.kernel_calls()==60&&main.schedule.diagnostics().kernel_calls==60);
    check("all_36_double_and_original_tags_reconstructed_bit_exact",main.ports.reconstruct_exact==60);
    check("candidate_never_installs_before_mock_publication",main.ports.candidate_before_install==60);
    check("state64_kernel61_scaffold70_request19_original_oracle",!state_c.bad&&!kernel_c.bad&&!scaffold_c.bad&&!request_c.bad&&commits==60);
    check("59_actual_prior_slots_match_original_oracle",!prior_c.bad&&prior_c.values==59*70);
    check("60_official_16_float_bits_original_oracle",exact16==60);
    check("reference_tick_59_gp_arrivals_and_duplicates_do_not_run_inner",nondriving==60&&gp_calls==59&&reply_duplicates==59&&duplicates==60);
    check("gp_k_remains_open_until_k_plus_1_and_no_early_causal_label",main.ports.prior_links==58&&main.ports.prior_errors==0&&main.schedule.diagnostics().prior_predictions_passed_to_next_source==58&&main.ports.store.prediction_fills()==59);
    check("original_gp_active_count_preserved",active==26);
    {Fixture f;f.ports.authorized=false;check("no_authority_no_source_execution",!f.start()&&f.schedule.diagnostics().first_fault==sch::Fault::Authority&&f.ports.store.kernel_calls()==0);}
    {Fixture f;bool ok=f.start();f.ports.rotor_available=false;ok&=f.step(rows[0])==sch::Event::Rejected;
        check("missing_actual_rotor_port_no_C_or_install",ok&&f.schedule.diagnostics().first_fault==sch::Fault::Rotor&&f.ports.store.kernel_calls()==0&&f.ports.store.installed_count()==0);}
    {Fixture f;bool ok=f.start()&&f.step(rows[0])==sch::Event::Committed;auto changed=source(rows[0],f.cfg);changed.p[0]+=1;
        ok&=f.schedule.local_source(changed,f.ports.now)==sch::Event::Rejected;
        check("nonidentical_same_generation_no_second_execution",ok&&f.schedule.diagnostics().first_fault==sch::Fault::Generation&&f.ports.store.kernel_calls()==1);}
    {Fixture f;bool ok=f.start()&&f.step(rows[0])==sch::Event::Committed;auto next=source(rows[1],f.cfg);next.reset_counter=1;f.ports.now=next.board_rx_us;
        ok&=f.schedule.local_source(next,f.ports.now)==sch::Event::Rejected;
        check("source_reset_no_second_execution",ok&&f.schedule.diagnostics().first_fault==sch::Fault::Reset&&f.ports.store.kernel_calls()==1);}
    {Fixture f;bool ok=first_two(f,rows);double prior[64];std::memcpy(prior,f.ports.store.installed_state64(),sizeof prior);
        ok&=f.step(rows[2])==sch::Event::Rejected;const auto calls=f.ports.store.kernel_calls();
        ok&=f.step(rows[3])==sch::Event::Rejected;
        check("missing_required_GP_terminal_before_C_and_preserves_last_state",ok&&f.schedule.diagnostics().first_fault==sch::Fault::MissingGp&&calls==2&&f.ports.store.kernel_calls()==2&&bits(prior,f.ports.store.installed_state64(),sizeof prior));}
    for(unsigned kind=0;kind<3;++kind){Fixture f;bool ok=first_two(f,rows);sch::GpQuery q{};ok&=f.schedule.take_gp_request(q,f.ports.now);
        bool predicted=false;double elapsed=0;auto reply=make_reply(q,predict,predicted,elapsed);ok&=predicted;
        if(kind==0)f.ports.now=q.original_valid_until_us+1;
        if(kind==1){reply.prediction_sample_closed=true;++f.ports.now;}
        if(kind==2){f.schedule.stop();++f.ports.now;}
        ok&=accept_reply(f,reply,f.ports.now)==sch::Event::Rejected;
        check(kind==0?"late_GP_never_fills_or_runs":kind==1?"premature_GP_closure_flag_never_fills_or_runs":"stop_clears_pending_and_rejects_GP_source_without_reset",
            ok&&f.ports.store.kernel_calls()==2&&f.ports.store.prediction_fills()==0&&f.ports.store.installed_count()==2&&!f.schedule.gp_pending()&&f.schedule.reference_windows()==0&&f.step(rows[2])==sch::Event::Rejected);
    }
    for(unsigned kind=1;kind<=3;++kind){Fixture f;bool ok=f.start()&&f.step(rows[0])==sch::Event::Committed;
        double old[64];std::memcpy(old,f.ports.store.installed_state64(),sizeof old);f.ports.publication_variant=kind;
        ok&=f.step(rows[1])==sch::Event::Rejected;const auto calls=f.ports.store.kernel_calls();
        math::PublicationReceipt forged{};forged.publication_succeeded=forged.numerical_commit_succeeded=true;
        ok&=!f.ports.store.install_after_publication(forged)&&f.step(rows[2])==sch::Event::Rejected;
        check(kind==1?"failed_publication_cannot_install_or_retry":kind==2?"published_but_failed_commit_preserves_action_and_old_state":"wrong_16_payload_cannot_install_or_retry",
            ok&&calls==2&&f.ports.store.kernel_calls()==2&&f.ports.store.installed_count()==1&&bits(old,f.ports.store.installed_state64(),sizeof old)&&
            f.ports.store.failure()==math::Failure::Publication&&f.schedule.diagnostics().commits==1&&f.schedule.diagnostics().output_publications==(kind==1?1u:2u));
    }
    {Fixture f;bool ok=f.start();f.ports.publication_variant=4;ok&=f.step(rows[0])==sch::Event::Rejected;
        ok&=f.step(rows[1])==sch::Event::Rejected;
        check("late_postcheck_preserves_actual_mock_pub_commit_and_faults_without_replay",ok&&
            f.schedule.diagnostics().first_fault==sch::Fault::SourceAge&&f.schedule.diagnostics().commits==1&&
            f.schedule.diagnostics().output_publications==1&&f.ports.store.installed_count()==1&&f.ports.store.kernel_calls()==1&&
            !f.schedule.gp_pending()&&f.ports.revoked);
    }
    main.schedule.stop();check("normal_stop_revokes_and_keeps_committed_audit",main.ports.revoked&&main.schedule.reference_windows()==0&&!main.schedule.gp_pending()&&main.ports.store.installed_count()==60);
    gpenmpcNative_canonicalLocalInnerFixedFirst_terminate();FreeLibrary(dll);std::fclose(timing);std::fclose(raw);std::fclose(checks_file);
    FILE*out=_wfopen(argv[6],L"wb");if(!out)return 10;
    std::fprintf(out,"{\"pass\":%s,\"checks\":%u,\"failed\":%u,\"rows\":%u,\"actual_C_calls_main\":%llu,\"actual_GP_calls_main\":%u,\"pending_fills\":%llu,\"prior_slots_compared\":%u,\"required_GP_replies_passed_k_plus_1\":%u,\"gp_active\":%u,\"float16_bit_exact_rows\":%u,\"reconstructed_input36_and_tags_bit_exact_rows\":%u,\"main_first_fault\":%u,\n",
        bad?"false":"true",checks,bad,commits,static_cast<unsigned long long>(main.ports.store.kernel_calls()),gp_calls,
        static_cast<unsigned long long>(main.ports.store.prediction_fills()),prior_c.values/70,main.ports.prior_links,active,exact16,
        main.ports.reconstruct_exact,unsigned(main.schedule.diagnostics().first_fault));
    state_c.write(out,"state64",true);kernel_c.write(out,"kernel61",true);scaffold_c.write(out,"scaffold70",true);request_c.write(out,"request19",true);prior_c.write(out,"prior70",true);
    std::fprintf(out,"\"workspace_bytes_host\":%zu,\"scheduler_bytes_host\":%zu,\"source_rotor_reference_authority_clock_are_mocks\":true,\"publication_is_mock\":true,\"actual_board_HRT_available\":false,\"real_time_or_WCET_proven\":false,\"COM_UDP_plant_board_actions\":0}\n",
        sizeof(math::CanonicalLocalInnerStateStore::Workspace),sizeof(Schedule));std::fclose(out);
    std::printf("Schedule -> actual combined C -> mock publication -> original GP -> next source: %u/%u; %u rows\n",checks-bad,checks,commits);
    return bad?11:0;
}
