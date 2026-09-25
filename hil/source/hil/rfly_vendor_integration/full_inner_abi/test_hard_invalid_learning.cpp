// Reuse the fixture reader and POD helpers for hard-invalid GP tests.
// Source and publication events are supplied by the host fixture.
#define wmain retained_fixture_main_not_executed
#include "test_full_inner_abi.cpp"
#undef wmain
#include <cmath>
int wmain(int argc,wchar_t**argv){
    if(argc!=4||!load(argv[1],argv[2]))return 2;
    FILE*f=_wfopen(argv[3],L"wb");if(!f)return 3;
    std::fprintf(f,"{\"scope\":\"ACTUAL_CANONICAL_C_HARD_INVALID_ONLY_WITH_MOCK_SOURCE_PUBLICATION\",\"cases\":[");
    for(unsigned kind=0;kind<2;++kind){
        Owner o;check(o.create(),"actual owner created");gpenmpc_full_inner_candidate c{};
        check(prepare(o,0,c)==RFI_OK&&commit(o,receipts(c))==RFI_OK,"first actual kernel commit");
        check(prepare(o,1,c)==RFI_OK&&commit(o,receipts(c))==RFI_OK,"next actual kernel opens GP request");
        double gp[18];std::memcpy(gp,original[1].gp,sizeof gp);
        // Exact actual GP18 schema: trust at 13, hard-invalid at 14. Means
        // remain nonzero to detect accidental use despite invalidity.
        gp[14]=1.0;gp[13]=kind?0.0:1.0;
        check(gpenmpc_full_inner_fill_gp(o.handle,c.original_tags2,gp)==RFI_OK,"hard-invalid reply retained without fabricating a kernel step");
        const int prepared=prepare(o,2,c);check(prepared==RFI_OK,"original next-step hard-invalid handling executes");
        const int installed=prepared==RFI_OK?commit(o,receipts(c)):-1;check(installed==RFI_OK,"hard-invalid fallback state jointly installed");
        gpenmpc_full_inner_closed_evidence evidence{};
        check(gpenmpc_full_inner_copy_closed_evidence(o.handle,&evidence)==RFI_OK&&evidence.learning_exported&&evidence.installed,
            "actual committed learning evidence exposed");
        check(evidence.values5[3]==1.0,"hard invalid remains visible");
        bool axes_zero=true;for(unsigned j=3;j<9;++j)axes_zero=axes_zero&&evidence.learning12[j]==0.0;
        check(axes_zero,"hard invalid prediction and physical axes zero");
        check(evidence.learning12[10]==0.0,"hard invalid responsibility blend zero");
        check(evidence.learning12[9]==1.0,"original Exact B1 fallback selected");
        bool finite=true;for(const auto v:c.control16)finite=finite&&std::isfinite(v);check(finite,"sixrotor published fixture controls remain finite");
        std::fprintf(f,"%s{\"input_hard_invalid\":true,\"input_trust\":%.17g,\"prepare_rc\":%d,\"commit_rc\":%d,\"closed5\":[",kind?",":"",gp[13],prepared,installed);
        for(unsigned j=0;j<5;++j)std::fprintf(f,"%s%.17g",j?",":"",evidence.values5[j]);
        std::fprintf(f,"],\"learning_authority_and_fallback9\":[");
        for(unsigned j=3;j<12;++j)std::fprintf(f,"%s%.17g",j>3?",":"",evidence.learning12[j]);
        std::fprintf(f,"]}");
    }
    std::fprintf(f,"],\"checks\":%u,\"failed\":%u}\n",checks,failed);std::fclose(f);
    std::printf("hard-invalid original C: %u/%u\n",checks-failed,checks);return failed?1:0;
}
