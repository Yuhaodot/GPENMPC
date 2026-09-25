/* HOST replay chains generated inner state and GP pending values.
 * Archived pending values provide an independent comparison.
 * A separate GP DLL isolates utility symbols and mutable scratch. */
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <wchar.h>
#include "gpenmpcNative_canonicalLocalInnerFixedFirst.h"
#include "gpenmpcNative_canonicalLocalInnerFixedStep.h"
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_initialize.h"
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_terminate.h"
#include "canonical_gp_standalone_api.h"

_Static_assert(sizeof(double)==8 && sizeof(unsigned long long)==8,"fixed ABI size");
typedef int (*Predict)(const double *,double *);
typedef int (*Stats)(GPENMPCGp256Stats *);
typedef struct {unsigned bad,exact_rows,values;double abs_error,scaled_error;} Comparison;
static void compare(Comparison *c,const double *a,const double *e,unsigned n)
{
    if(memcmp(a,e,n*sizeof(double))==0)++c->exact_rows;
    for(unsigned j=0;j<n;++j){
        ++c->values;
        if(isnan(a[j])!=isnan(e[j]) || isfinite(a[j])!=isfinite(e[j]) ||
           (isinf(e[j])&&signbit(a[j])!=signbit(e[j]))){++c->bad;continue;}
        if(isfinite(e[j])){
            double d=fabs(a[j]-e[j]),s=d/fmax(1.0,fabs(e[j]));
            if(d>c->abs_error)c->abs_error=d;
            if(s>c->scaled_error)c->scaled_error=s;
            /* Numerical implementation-comparison tolerance. */
            if(s>1e-10)++c->bad;
        }
    }
}
static void report(FILE *f,const char *name,Comparison c,int comma)
{
    fprintf(f,"\"%s\":{\"bad_values\":%u,\"values\":%u,\"bit_exact_rows\":%u,"
        "\"maximum_absolute_error\":%.17g,\"maximum_scaled_error\":%.17g}%s\n",
        name,c.bad,c.values,c.exact_rows,c.abs_error,c.scaled_error,comma?",":"");
}
static int read_values(FILE *f,double *x,unsigned n){return fread(x,8,n,f)==n;}
static double milliseconds(LARGE_INTEGER a,LARGE_INTEGER b,LARGE_INTEGER q)
{return ((double)(b.QuadPart-a.QuadPart)*1000.0)/(double)q.QuadPart;}

/* Field assignments follow completeCanonicalCurrentGpPrediction.m.
 * max uses MATLAB omit-NaN behavior; required GP outputs are checked
 * for finiteness throughout the 60-row replay. */
static int complete_pending(double pending[70],const double request[19],const double result[18])
{
    if(pending[44]!=0 || pending[45]!=0 || pending[1]!=0)return 0;
    if(request[0]!=(double)(pending[2]!=0 && pending[3]!=0))return 0;
    if(request[0]==0)return result==NULL;
    if(!result)return 0;
    pending[1]=1;
    memcpy(pending+9,request+1,17*sizeof(double));
    memcpy(pending+35,result,3*sizeof(double));
    memcpy(pending+41,result+6,3*sizeof(double));
    pending[5]=result[13];pending[7]=result[12];
    pending[8]=fmax(fmax(result[9],result[10]),result[11]);
    pending[4]=(double)(result[14]!=0);
    for(unsigned j=0;j<3;++j)pending[38+j]=(request[18]*pending[5])*pending[35+j];
    return 1;
}
int wmain(int argc,wchar_t **argv)
{
    if(argc!=6){fprintf(stderr,"usage: chain fixture gp.dll actual.bin rows.csv result.json\n");return 2;}
    const uint32_t endian=1;if(*(const unsigned char *)&endian!=1)return 3;
    HMODULE dll=LoadLibraryExW(argv[2],NULL,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
    if(!dll){fprintf(stderr,"LoadLibrary error %lu\n",GetLastError());return 4;}
    Predict predict=(Predict)GetProcAddress(dll,"gpenmpc_gp256_predict");
    Stats stats=(Stats)GetProcAddress(dll,"gpenmpc_gp256_stats");
    if(!predict||!stats)return 5;
    FILE *f=_wfopen(argv[1],L"rb"),*actual=_wfopen(argv[3],L"wb"),*rows=_wfopen(argv[4],L"wb");
    if(!f||!actual||!rows)return 6;
    char magic[4];uint32_t count=0;
    if(fread(magic,1,4,f)!=4||memcmp(magic,"LCI1",4)||fread(&count,4,1,f)!=1||count!=60)return 7;
    double state[64]={0},pending[70]={0},next[64],y[61],scaffold[70],request[19];
    unsigned long long tags[2],previous_tags[2]={0};
    Comparison state_c={0},kernel_c={0},scaffold_c={0},request_c={0},pending_c={0};
#ifdef GPENMPC_CANONICAL_EXPLICIT_WORKSPACE
    static f_gpenmpcNative_canonicalLocalIn workspace;
#endif
    unsigned bad_flags=0,gp_calls=0,gp_ok=0,causal_slots=0,mode_active=0,oracle_mode_active=0,norm16_exact=0;
    const unsigned flags[]={12,13,19,27,28,29,53};
    LARGE_INTEGER frequency;QueryPerformanceFrequency(&frequency);
    fprintf(rows,"row,source_timestamp_ns,source_generation,dt_s,gp_required,gp_result,gp_mode_active,inner_host_ms,gp_host_ms,prior_pending_compared\n");
    gpenmpcNative_canonicalLocalInnerFixedFirst_initialize();
    for(unsigned k=0;k<count;++k){
        double input[36],oracle_pending[70],es[64],ey[61],ep[70],eq[19],gp[18];
        if(!read_values(f,input,36)||fread(tags,8,2,f)!=2||!read_values(f,oracle_pending,70)||
           !read_values(f,es,64)||!read_values(f,ey,61)||!read_values(f,ep,70)||!read_values(f,eq,19))return 8;
        LARGE_INTEGER begin,after_inner,after_gp;int rc=-1;
        if(k){
            compare(&pending_c,pending,oracle_pending,70);
            if(pending[44]!=0||pending[45]!=0)++bad_flags;
            if(tags[0]<=previous_tags[0]||tags[1]<=previous_tags[1])++bad_flags;
            ++causal_slots;
        }
        QueryPerformanceCounter(&begin);
        if(k==0)gpenmpcNative_canonicalLocalInnerFixedFirst(
#ifdef GPENMPC_CANONICAL_EXPLICIT_WORKSPACE
            &workspace,
#endif
            input,tags,next,y,scaffold,request);
        else gpenmpcNative_canonicalLocalInnerFixedStep(
#ifdef GPENMPC_CANONICAL_EXPLICIT_WORKSPACE
            &workspace,
#endif
            state,previous_tags,input,tags,pending,previous_tags,next,y,scaffold,request);
        QueryPerformanceCounter(&after_inner);
        compare(&state_c,next,es,64);compare(&kernel_c,y,ey,61);
        compare(&scaffold_c,scaffold,ep,70);compare(&request_c,request,eq,19);
        for(unsigned j=0;j<sizeof(flags)/sizeof(flags[0]);++j)if(next[flags[j]]!=es[flags[j]])++bad_flags;
        if(request[0]!=eq[0]||y[60]!=ey[60])++bad_flags;
        for(unsigned j=0;j<5;++j)if(scaffold[j]!=ep[j])++bad_flags;
        if(scaffold[44]!=ep[44]||scaffold[45]!=ep[45]||scaffold[52]!=ep[52]||scaffold[53]!=ep[53])++bad_flags;
        if(next[19]!=0)++mode_active;
        if(es[19]!=0)++oracle_mode_active;
        memcpy(pending,scaffold,sizeof pending);
        if(request[0]!=0){
            ++gp_calls;rc=predict(request+1,gp);
            if(rc==GPENMPC_GP256_OK){
                ++gp_ok;
                for(unsigned j=0;j<18;++j)if(!isfinite(gp[j]))++bad_flags;
                if(!complete_pending(pending,request,gp))++bad_flags;
            }else{++bad_flags;}
        }else if(!complete_pending(pending,request,NULL)){++bad_flags;}
        QueryPerformanceCounter(&after_gp);
        float a16[16]={0},e16[16]={0};const unsigned map[6]={4,0,3,5,1,2};
        const double upper=32.145727009134916;
        for(unsigned j=0;j<6;++j){
            if(!isfinite(y[4+j])||y[4+j]<0||y[4+j]>upper){++bad_flags;continue;}
            a16[map[j]]=(float)(y[4+j]/upper);e16[map[j]]=(float)(ey[4+j]/upper);
        }
        if(memcmp(a16,e16,sizeof a16)==0)++norm16_exact;else ++bad_flags;
        if(fwrite(next,8,64,actual)!=64||fwrite(y,8,61,actual)!=61||fwrite(scaffold,8,70,actual)!=70||
           fwrite(request,8,19,actual)!=19||fwrite(pending,8,70,actual)!=70)return 9;
        fprintf(rows,"%u,%llu,%llu,%.17g,%.0f,%d,%.0f,%.9f,%.9f,%u\n",k+1,tags[0],tags[1],input[34],request[0],rc,next[19],
            milliseconds(begin,after_inner,frequency),milliseconds(after_inner,after_gp,frequency),k!=0);
        memcpy(state,next,sizeof state);memcpy(previous_tags,tags,sizeof tags);
    }
    if(fgetc(f)!=EOF)return 10;
    fclose(f);fclose(actual);fclose(rows);gpenmpcNative_canonicalLocalInnerFixedFirst_terminate();
    GPENMPCGp256Stats measured={0};if(stats(&measured)!=0)++bad_flags;
    if(gp_calls!=59||gp_ok!=59||causal_slots!=59||mode_active!=oracle_mode_active||
       measured.attempts!=59||measured.successes!=59||measured.busy_rejections||measured.invalid_arguments)++bad_flags;
    FreeLibrary(dll);
    unsigned bad=state_c.bad+kernel_c.bad+scaffold_c.bad+request_c.bad+pending_c.bad+bad_flags;
    FILE *out=_wfopen(argv[5],L"wb");if(!out)return 11;
    fprintf(out,"{\"pass\":%s,\"rows\":60,\"discrete_or_causal_mismatches\":%u,\"gp_calls\":%u,\"gp_successes\":%u,"
        "\"next_sample_closure_slots\":%u,\"gp_responsibility_active_rows\":%u,\"oracle_gp_responsibility_active_rows\":%u,"
        "\"official_numeric_single16_bit_exact_rows\":%u,\n",bad?"false":"true",bad_flags,gp_calls,gp_ok,causal_slots,mode_active,oracle_mode_active,norm16_exact);
    report(out,"state64",state_c,1);report(out,"kernel61",kernel_c,1);report(out,"pending_scaffold70",scaffold_c,1);
    report(out,"gp_request19",request_c,1);report(out,"actual_previous_gp_pending70",pending_c,0);
    fprintf(out,"}\n");fclose(out);
    printf("HOST C inner + actual original GP DLL: 60 rows, 59 queries; mismatches=%u\n",bad);
    return bad?12:0;
}
