/* Generated C replay with recorded GP predictions.
 * Generated C chains the state and produces closed5. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditStep.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_initialize.h"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_terminate.h"
#define FIRST gpenmpcNative_canonicalLocalInnerWithAuditFirst
#define STEP gpenmpcNative_canonicalLocalInnerWithAuditStep
#define INITIALIZE gpenmpcNative_canonicalLocalInnerWithAuditFirst_initialize
#define TERMINATE gpenmpcNative_canonicalLocalInnerWithAuditFirst_terminate
#define INPUT_MAGIC "LCA1"
#define COMPARISONS 6
#else
#include "gpenmpcNative_canonicalLocalInnerWithEvidenceFirst.h"
#include "gpenmpcNative_canonicalLocalInnerWithEvidenceStep.h"
#include "gpenmpcNative_canonicalLocalInnerWithEvidenceFirst_initialize.h"
#include "gpenmpcNative_canonicalLocalInnerWithEvidenceFirst_terminate.h"
#define FIRST gpenmpcNative_canonicalLocalInnerWithEvidenceFirst
#define STEP gpenmpcNative_canonicalLocalInnerWithEvidenceStep
#define INITIALIZE gpenmpcNative_canonicalLocalInnerWithEvidenceFirst_initialize
#define TERMINATE gpenmpcNative_canonicalLocalInnerWithEvidenceFirst_terminate
#define INPUT_MAGIC "LCE1"
#define COMPARISONS 5
#endif
static e_gpenmpcNative_canonicalLocalIn workspace;
typedef struct {unsigned values,bad,bit_exact_rows;double max_abs,max_scaled;} Compare;
static void compare(Compare*c,const double*a,const double*b,unsigned n){
    if(!memcmp(a,b,n*sizeof(double)))++c->bit_exact_rows;
    for(unsigned j=0;j<n;++j){++c->values;
        if(isnan(a[j])!=isnan(b[j])||isfinite(a[j])!=isfinite(b[j])||
            (isinf(b[j])&&signbit(a[j])!=signbit(b[j]))){++c->bad;continue;}
        if(isfinite(b[j])){double d=fabs(a[j]-b[j]),s=d/fmax(1.,fabs(b[j]));
            if(d>c->max_abs)c->max_abs=d;
            if(s>c->max_scaled)c->max_scaled=s;
            if(s>1e-10)++c->bad;}
    }
}
static int rd(FILE*f,double*v,size_t n){return fread(v,8,n,f)==n;}
int main(int argc,char**argv){
    if(argc!=4)return 2;
    FILE*f=fopen(argv[1],"rb"),*raw=fopen(argv[2],"wb");if(!f||!raw)return 3;
    char magic[4];uint32_t count;
    if(fread(magic,1,4,f)!=4||memcmp(magic,INPUT_MAGIC,4)||fread(&count,4,1,f)!=1||count!=60)return 4;
    double state[64]={0},s[64],y[61],p[70],q[19],closed[5];
    unsigned long long tags[2],previous[2]={0};Compare c[COMPARISONS]={{0}};
    unsigned exact_flags=0,closed_rows=0,open_rows=0;
    INITIALIZE();
    for(unsigned k=0;k<60;++k){double input[36],pending[70],es[64],ey[61],ep[70],eq[19],ec[5];
        if(!rd(f,input,36)||fread(tags,8,2,f)!=2||!rd(f,pending,70)||!rd(f,es,64)||!rd(f,ey,61)||
            !rd(f,ep,70)||!rd(f,eq,19)||!rd(f,ec,5))return 5;
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
        double learning[12],expected_learning[12];
        if(!rd(f,expected_learning,12))return 5;
        if(k==0)FIRST(&workspace,input,tags,s,y,p,q,closed,learning);
        else STEP(&workspace,state,previous,input,tags,pending,previous,s,y,p,q,closed,learning);
        compare(&c[5],learning,expected_learning,12);
#else
        if(k==0)FIRST(&workspace,input,tags,s,y,p,q,closed);
        else STEP(&workspace,state,previous,input,tags,pending,previous,s,y,p,q,closed);
#endif
        compare(&c[0],s,es,64);compare(&c[1],y,ey,61);compare(&c[2],p,ep,70);
        compare(&c[3],q,eq,19);compare(&c[4],closed,ec,5);
        if(!memcmp(closed,ec,4*sizeof(double)))++exact_flags;
        if(closed[1]==1&&closed[2]==1)++closed_rows;
        if(p[44]==0&&p[45]==0)++open_rows;
        if(fwrite(s,8,64,raw)!=64||fwrite(y,8,61,raw)!=61||fwrite(p,8,70,raw)!=70||
            fwrite(q,8,19,raw)!=19||fwrite(closed,8,5,raw)!=5)return 6;
#ifdef GPENMPC_CANONICAL_LEARNING_AUDIT
        if(fwrite(learning,8,12,raw)!=12)return 6;
#endif
        memcpy(state,s,sizeof state);memcpy(previous,tags,sizeof tags);
    }
    if(fgetc(f)!=EOF)return 7;fclose(f);fclose(raw);
    TERMINATE();
    unsigned bad=0;for(unsigned j=0;j<COMPARISONS;++j)bad+=c[j].bad;
    int pass=!bad&&exact_flags==60&&closed_rows==59&&open_rows==60;
    FILE*r=fopen(argv[3],"wb");if(!r)return 8;
    fprintf(r,"{\"pass\":%s,\"rows\":60,\"closed_flags_exact_rows\":%u,\"closed_rows\":%u,\"open_scaffold_rows\":%u,\"workspace_bytes\":%zu,\"comparisons\":[",pass?"true":"false",exact_flags,closed_rows,open_rows,sizeof workspace);
    for(unsigned j=0;j<COMPARISONS;++j)fprintf(r,"%s{\"index\":%u,\"values\":%u,\"bad\":%u,\"bit_exact_rows\":%u,\"max_abs\":%.17g,\"max_scaled\":%.17g}",j?",":"",j,c[j].values,c[j].bad,c[j].bit_exact_rows,c[j].max_abs,c[j].max_scaled);
    fprintf(r,"],\"board_actions\":0,\"live_gp_calls\":0,\"candidate_only\":true}\n");fclose(r);
    printf("actual C evidence rows=60 bad=%u discrete_exact=%u closed=%u open=%u\n",bad,exact_flags,closed_rows,open_rows);
    return pass?0:9;
}
