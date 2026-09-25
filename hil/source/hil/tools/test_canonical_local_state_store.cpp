#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include "../rfly_vendor_integration/CanonicalLocalInnerStateStore.hpp"
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_initialize.h"
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_terminate.h"
#include "canonical_gp_standalone_api.h"
using namespace gpenmpc_local_math;
using Predict=int(*)(const double*,double*);
struct Row{double input[36];unsigned long long tags[2];double prior[70],state[64],kernel[61],scaffold[70],request[19];};
struct StoreFixture {
#ifdef GPENMPC_CANONICAL_EXPLICIT_WORKSPACE
    CanonicalLocalInnerStateStore::Workspace workspace{};
    CanonicalLocalInnerStateStore owner{workspace};
#else
    CanonicalLocalInnerStateStore owner{};
#endif
};
static unsigned checks=0,bad=0;static FILE *ledger{};
static void check(const char*name,bool ok){++checks;if(!ok)++bad;std::fprintf(ledger,"%s,%u\n",name,unsigned(ok));}
static bool close_values(const double*a,const double*b,unsigned n){
    for(unsigned j=0;j<n;++j){
        if(std::isnan(a[j])!=std::isnan(b[j])||std::isfinite(a[j])!=std::isfinite(b[j]))return false;
        if(std::isfinite(b[j])&&std::fabs(a[j]-b[j])/std::fmax(1.0,std::fabs(b[j]))>1e-10)return false;
    }return true;
}
static PublicationReceipt mock_receipt(const Candidate&c,std::uint64_t generation){
    PublicationReceipt r{};r.source_timestamp_ns=c.original_tags2[0];r.source_generation=c.original_tags2[1];
    r.output_generation=generation;r.original_publication_us=r.source_timestamp_ns/1000+1;
    r.publication_succeeded=r.numerical_commit_succeeded=true;
    std::memcpy(r.actual_control16,c.control16,sizeof r.actual_control16);return r;
}
static bool first_two(CanonicalLocalInnerStateStore&s,const std::vector<Row>&r){
    for(unsigned k=0;k<2;++k){
        if(!s.prepare(r[k].input,r[k].tags))return false;
        if(!s.install_after_publication(mock_receipt(*s.candidate(),k+1)))return false;
    }return s.prediction_required()&&!s.prediction_ready();
}
int wmain(int argc,wchar_t**argv){
    if(argc!=5)return 2;
    FILE*f=_wfopen(argv[1],L"rb");ledger=_wfopen(argv[3],L"wb");if(!f||!ledger)return 3;
    char magic[4];unsigned count{};if(std::fread(magic,1,4,f)!=4||std::memcmp(magic,"LCI1",4)||std::fread(&count,4,1,f)!=1||count!=60)return 4;
    std::vector<Row> rows(count);for(auto&r:rows){
        if(std::fread(r.input,8,36,f)!=36||std::fread(r.tags,8,2,f)!=2||std::fread(r.prior,8,70,f)!=70||
           std::fread(r.state,8,64,f)!=64||std::fread(r.kernel,8,61,f)!=61||std::fread(r.scaffold,8,70,f)!=70||std::fread(r.request,8,19,f)!=19)return 5;
    }if(std::fgetc(f)!=EOF)return 6;std::fclose(f);
    HMODULE dll=LoadLibraryExW(argv[2],nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
    if(!dll)return 7;auto predict=reinterpret_cast<Predict>(GetProcAddress(dll,"gpenmpc_gp256_predict"));if(!predict)return 8;
    std::fprintf(ledger,"check,pass\n");gpenmpcNative_canonicalLocalInnerFixedFirst_initialize();
    StoreFixture main_fixture;auto&owner=main_fixture.owner;unsigned gp_calls=0;
    for(unsigned k=0;k<count;++k){
        const auto&r=rows[k];double before[64]{};if(k)std::memcpy(before,owner.installed_state64(),sizeof before);
        check("actual_generated_candidate",owner.prepare(r.input,r.tags));
        if(!owner.candidate())return 9;
        check("candidate_math_matches_original",close_values(owner.candidate()->kernel61,r.kernel,61)&&
            close_values(owner.candidate()->post_state64,r.state,64)&&close_values(owner.candidate()->scaffold70,r.scaffold,70));
        check("prepare_does_not_install_state",k?std::memcmp(before,owner.installed_state64(),sizeof before)==0:owner.installed_state64()==nullptr);
        const auto receipt=mock_receipt(*owner.candidate(),k+1);
        check("exact_mock_publication_allows_install",owner.install_after_publication(receipt));
        check("installed_math_state_matches",close_values(owner.installed_state64(),r.state,64));
        if(owner.prediction_required()){
            double prediction[18];++gp_calls;
            check("actual_original_gp_success",predict(owner.gp_request19()+1,prediction)==GPENMPC_GP256_OK);
            double prior_state[64];std::memcpy(prior_state,owner.installed_state64(),sizeof prior_state);
            check("open_gp_fill",owner.fill_open_prediction(r.tags,prediction));
            check("reply_neither_closes_innovation_nor_runs_control",owner.kernel_calls()==k+1&&owner.open_pending70()[44]==0&&
                owner.open_pending70()[45]==0&&std::memcmp(prior_state,owner.installed_state64(),sizeof prior_state)==0);
        }
        if(k+1<count)check("actual_next_pending_matches_original",close_values(owner.open_pending70(),rows[k+1].prior,70));
    }
    check("full_continuous_counts",owner.kernel_calls()==60&&owner.installed_count()==60&&owner.prediction_fills()==59&&gp_calls==59);
    {StoreFixture fixture;auto&s=fixture.owner;auto r=rows[0];r.input[34]=.02;check("invalid_actual_dt_no_kernel",!s.prepare(r.input,r.tags)&&s.kernel_calls()==0);}
    {StoreFixture fixture;auto&s=fixture.owner;auto r=rows[0];r.input[0]=NAN;check("nonfinite_input_no_kernel",!s.prepare(r.input,r.tags)&&s.kernel_calls()==0);}
    {StoreFixture fixture;auto&s=fixture.owner;check("double_prepare_rejected_without_install",s.prepare(rows[0].input,rows[0].tags)&&!s.prepare(rows[1].input,rows[1].tags)&&s.installed_count()==0);}
    for(unsigned variant=0;variant<5;++variant){
        StoreFixture fixture;auto&s=fixture.owner;if(!s.prepare(rows[0].input,rows[0].tags))return 10;
        auto receipt=mock_receipt(*s.candidate(),1);
        if(variant==0)receipt.publication_succeeded=false;
        if(variant==1)receipt.numerical_commit_succeeded=false;
        if(variant==2)receipt.source_generation++;
        if(variant==3)receipt.actual_control16[10]=1;
        if(variant==4)receipt.original_publication_us=1;
        check("failed_or_mismatched_publication_never_installs",!s.install_after_publication(receipt)&&s.installed_state64()==nullptr&&s.installed_count()==0&&s.install_attempts()==1);
    }
    {StoreFixture fixture;auto&s=fixture.owner;if(!first_two(s,rows))return 11;double prior[64];std::memcpy(prior,s.installed_state64(),sizeof prior);
        check("missing_gp_cannot_advance_or_replay",!s.prepare(rows[2].input,rows[2].tags)&&s.failure()==Failure::MissingPrediction&&s.kernel_calls()==2&&std::memcmp(prior,s.installed_state64(),sizeof prior)==0);}
    {StoreFixture fixture;auto&s=fixture.owner;if(!first_two(s,rows))return 12;double result[18]{};unsigned long long wrong[2]={rows[1].tags[0],rows[1].tags[1]+1};
        check("wrong_source_gp_rejected",!s.fill_open_prediction(wrong,result)&&s.prediction_fills()==0&&s.kernel_calls()==2);}
    {StoreFixture fixture;auto&s=fixture.owner;if(!first_two(s,rows))return 13;double result[18]{};for(auto&v:result)v=NAN;result[14]=1;
        check("explicit_hard_invalid_original_gp_not_fabricated_transport_failure",s.fill_open_prediction(rows[1].tags,result)&&
            s.open_pending70()[1]==1&&s.open_pending70()[4]==1&&std::isnan(s.open_pending70()[35])&&s.open_pending70()[44]==0&&s.kernel_calls()==2);
        check("duplicate_gp_rejected_no_second_fill",!s.fill_open_prediction(rows[1].tags,result)&&s.prediction_fills()==1);
    }
    {StoreFixture fixture;auto&s=fixture.owner;if(!s.prepare(rows[0].input,rows[0].tags)||!s.install_after_publication(mock_receipt(*s.candidate(),1)))return 14;
        check("duplicate_source_cannot_reexecute",!s.prepare(rows[0].input,rows[0].tags)&&s.kernel_calls()==1&&s.installed_count()==1);}
    gpenmpcNative_canonicalLocalInnerFixedFirst_terminate();FreeLibrary(dll);std::fclose(ledger);
    FILE*out=_wfopen(argv[4],L"wb");if(!out)return 15;
    std::fprintf(out,"{\"pass\":%s,\"checks\":%u,\"bad\":%u,\"continuous_rows\":60,\"actual_gp_queries\":%u,"
        "\"state_store_bytes_host\":%zu,\"publication_receipts_are_mock\":true,\"board_consumption_proven\":false,"
        "\"COM_or_UDP\":0,\"plant_runs\":0}\n",bad?"false":"true",checks,bad,gp_calls,sizeof(CanonicalLocalInnerStateStore));
    std::fclose(out);std::printf("State store actual math: %u/%u; mock publication only\n",checks-bad,checks);return bad?16:0;
}
