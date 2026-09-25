/* HOST replay of MATLAB Coder output with archived GP predictions.
 * Chained state exposes accumulated numerical differences. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "gpenmpcNative_canonicalLocalInnerFixedFirst.h"
#include "gpenmpcNative_canonicalLocalInnerFixedStep.h"
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_initialize.h"
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_terminate.h"

_Static_assert(sizeof(double)==8 && sizeof(unsigned long long)==8,"fixed ABI size");
typedef struct {unsigned bad, exact_rows, values; double abs_error, scaled_error;} Comparison;
static void compare(Comparison *c,const double *a,const double *e,unsigned n)
{
    if(memcmp(a,e,n*sizeof(double))==0) ++c->exact_rows;
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
static int read_doubles(FILE *f,double *x,unsigned n){return fread(x,sizeof(double),n,f)==n;}
static void print_comparison(FILE *f,const char *name,Comparison c,int comma)
{
    fprintf(f,"\"%s\":{\"bad_values\":%u,\"values\":%u,\"bit_exact_rows\":%u,"
        "\"maximum_absolute_error\":%.17g,\"maximum_scaled_error\":%.17g}%s\n",
        name,c.bad,c.values,c.exact_rows,c.abs_error,c.scaled_error,comma?",":"");
}
int main(int argc,char **argv)
{
    if(argc!=4){fprintf(stderr,"usage: local_inner fixture actual.bin result.json\n");return 2;}
    const uint32_t endian=1;if(*(const unsigned char*)&endian!=1)return 3;
    FILE *f=fopen(argv[1],"rb"),*actual=fopen(argv[2],"wb");
    if(!f||!actual)return 4;
    char magic[4];uint32_t count=0;
    if(fread(magic,1,4,f)!=4||memcmp(magic,"LCI1",4)||fread(&count,4,1,f)!=1||count!=60)return 5;
    double state[64]={0},next[64],y[61],scaffold[70],request[19];
    unsigned long long previous_tags[2]={0},tags[2];
    Comparison state_c={0},kernel_c={0},scaffold_c={0},request_c={0};
    unsigned flag_bad=0,norm16_exact_rows=0;const unsigned flags[]={12,13,19,27,28,29,53};
    gpenmpcNative_canonicalLocalInnerFixedFirst_initialize();
    for(unsigned k=0;k<count;++k){
        double input[36],pending[70],es[64],ey[61],ep[70],eq[19];
        if(!read_doubles(f,input,36)||fread(tags,8,2,f)!=2||!read_doubles(f,pending,70)||
           !read_doubles(f,es,64)||!read_doubles(f,ey,61)||!read_doubles(f,ep,70)||!read_doubles(f,eq,19))return 6;
        if(k==0)gpenmpcNative_canonicalLocalInnerFixedFirst(input,tags,next,y,scaffold,request);
        else gpenmpcNative_canonicalLocalInnerFixedStep(state,previous_tags,input,tags,pending,previous_tags,next,y,scaffold,request);
        compare(&state_c,next,es,64);compare(&kernel_c,y,ey,61);
        compare(&scaffold_c,scaffold,ep,70);compare(&request_c,request,eq,19);
        for(unsigned j=0;j<sizeof(flags)/sizeof(flags[0]);++j)if(next[flags[j]]!=es[flags[j]])++flag_bad;
        if(request[0]!=eq[0]||y[60]!=ey[60])++flag_bad;
        for(unsigned j=0;j<5;++j)if(scaffold[j]!=ep[j])++flag_bad;
        if(scaffold[44]!=ep[44]||scaffold[45]!=ep[45]||scaffold[52]!=ep[52]||scaffold[53]!=ep[53])++flag_bad;
        /* Map controls([5,1,4,6,2,3])=single(rotorN/upper); set the rest to zero.
         * Validate binary64 operands against the canonical range before casting. */
        float actual16[16]={0},expected16[16]={0};
        const unsigned map[6]={4,0,3,5,1,2};const double upper=32.145727009134916;
        for(unsigned j=0;j<6;++j){
            if(!isfinite(y[4+j])||!isfinite(ey[4+j])||y[4+j]<0||ey[4+j]<0||
               y[4+j]>upper||ey[4+j]>upper){++flag_bad;continue;}
            actual16[map[j]]=(float)(y[4+j]/upper);
            expected16[map[j]]=(float)(ey[4+j]/upper);
        }
        if(memcmp(actual16,expected16,sizeof actual16)==0)++norm16_exact_rows;
        else ++flag_bad;
        if(fwrite(next,8,64,actual)!=64||fwrite(y,8,61,actual)!=61||
            fwrite(scaffold,8,70,actual)!=70||fwrite(request,8,19,actual)!=19)return 7;
        memcpy(state,next,sizeof state);memcpy(previous_tags,tags,sizeof tags);
    }
    if(fgetc(f)!=EOF)return 8;
    fclose(f);fclose(actual);gpenmpcNative_canonicalLocalInnerFixedFirst_terminate();
    unsigned bad=state_c.bad+kernel_c.bad+scaffold_c.bad+request_c.bad+flag_bad;
    FILE *out=fopen(argv[3],"wb");if(!out)return 9;
    fprintf(out,"{\"pass\":%s,\"rows\":60,\"discrete_flag_mismatches\":%u,\"official_numeric_single16_bit_exact_rows\":%u,\n",bad?"false":"true",flag_bad,norm16_exact_rows);
    print_comparison(out,"state64",state_c,1);print_comparison(out,"kernel61",kernel_c,1);
    print_comparison(out,"pending_scaffold70",scaffold_c,1);print_comparison(out,"gp_request19",request_c,0);
    fprintf(out,"}\n");fclose(out);
    printf("HOST actual generated C: rows60, bad%u, kernel max %.17g, state max %.17g\n",bad,kernel_c.abs_error,state_c.abs_error);
    return bad?10:0;
}
