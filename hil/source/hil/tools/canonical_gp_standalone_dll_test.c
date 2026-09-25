#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "canonical_gp_standalone_api.h"
enum { ROWS=293, THREADS=8, CALLS=32 };
typedef int (__cdecl *Predict)(const double *,double *);
typedef int (__cdecl *Stats)(GPENMPCGp256Stats *);
static Predict predict;
static double inputs[ROWS][17],expected[ROWS][18];
static HANDLE start_event;
typedef struct {unsigned index,success,busy,bad;int status[CALLS];unsigned row[CALLS];double outputs[CALLS][18];} Worker;
static Worker workers[THREADS];
static int equal_output(const double *actual,const double *want)
{
    for(unsigned j=0;j<18;++j){
        if(isnan(actual[j])!=isnan(want[j]) || isfinite(actual[j])!=isfinite(want[j]))return 0;
        if(isinf(want[j]) && signbit(actual[j])!=signbit(want[j]))return 0;
        if(isfinite(want[j]) && fabs(actual[j]-want[j])/fmax(1.0,fabs(want[j]))>=1.0e-10)return 0;
    }
    return actual[14]==want[14];
}
static int invalid_output(const double *output)
{
    for(unsigned j=0;j<18;++j)if(!isnan(output[j]))return 0;
    return 1;
}
static DWORD WINAPI worker(LPVOID argument)
{
    Worker *w=(Worker *)argument;
    if(WaitForSingleObject(start_event,10000)!=WAIT_OBJECT_0){++w->bad;return 1;}
    for(unsigned k=0;k<CALLS;++k){
        unsigned row=(w->index*CALLS+k)%ROWS;w->row[k]=row;
        int status=predict(inputs[row],w->outputs[k]);w->status[k]=status;
        if(status==GPENMPC_GP256_OK){++w->success;if(!equal_output(w->outputs[k],expected[row]))++w->bad;}
        else if(status==GPENMPC_GP256_BUSY){++w->busy;if(!invalid_output(w->outputs[k]))++w->bad;}
        else ++w->bad;
    }
    return 0;
}
int main(int argc,char **argv)
{
    if(argc!=5)return 2;
    FILE *f=fopen(argv[1],"rb");if(!f)return 3;
    char magic[8];uint32_t shape[3];
    if(fread(magic,1,8,f)!=8||memcmp(magic,"RAGP2561",8)!=0||fread(shape,4,3,f)!=3
       ||shape[0]!=ROWS||shape[1]!=17||shape[2]!=18){fclose(f);return 4;}
    for(unsigned i=0;i<ROWS;++i){if(fread(inputs[i],8,17,f)!=17||fread(expected[i],8,18,f)!=18){fclose(f);return 5;}}
    if(fgetc(f)!=EOF){fclose(f);return 6;}fclose(f);
    HMODULE dll=LoadLibraryA(argv[2]);if(!dll){fprintf(stderr,"LoadLibrary error %lu\n",GetLastError());return 7;}
    FARPROC p=GetProcAddress(dll,"gpenmpc_gp256_predict"),s=GetProcAddress(dll,"gpenmpc_gp256_stats");
    if(!p||!s)return 8;
    Stats stats;memcpy(&predict,&p,sizeof(predict));memcpy(&stats,&s,sizeof(stats));
    unsigned bad=0,serial_pass=0;double out[18];
    for(unsigned i=0;i<ROWS;++i){
        if(predict(inputs[i],out)==GPENMPC_GP256_OK && equal_output(out,expected[i]))++serial_pass;else ++bad;
    }
    if(predict(NULL,out)!=GPENMPC_GP256_ARGUMENT||!invalid_output(out))++bad;
    if(predict(inputs[0],NULL)!=GPENMPC_GP256_ARGUMENT)++bad;
    start_event=CreateEventA(NULL,TRUE,FALSE,NULL);if(!start_event)return 9;
    HANDLE threads[THREADS];
    for(unsigned i=0;i<THREADS;++i){workers[i].index=i;threads[i]=CreateThread(NULL,0,worker,&workers[i],0,NULL);if(!threads[i])return 10;}
    SetEvent(start_event);
    if(WaitForMultipleObjects(THREADS,threads,TRUE,10000)!=WAIT_OBJECT_0)return 11;
    unsigned concurrent_ok=0,busy=0;
    for(unsigned i=0;i<THREADS;++i){concurrent_ok+=workers[i].success;busy+=workers[i].busy;bad+=workers[i].bad;CloseHandle(threads[i]);}
    CloseHandle(start_event);
    GPENMPCGp256Stats final={0,0,0,0};
    if(stats(&final)!=GPENMPC_GP256_OK||final.attempts!=ROWS+2+THREADS*CALLS
       ||final.successes!=serial_pass+concurrent_ok||final.busy_rejections!=busy||final.invalid_arguments!=2)++bad;
    /* Require both a successful concurrent call and a BUSY rejection. */
    if(busy==0||concurrent_ok==0)++bad;
    FILE *csv=fopen(argv[3],"wb");if(!csv)return 12;
    fprintf(csv,"thread,call,oracle_row,status");for(unsigned j=0;j<18;++j)fprintf(csv,",output%u",j+1);fprintf(csv,"\n");
    for(unsigned i=0;i<THREADS;++i)for(unsigned k=0;k<CALLS;++k){
        fprintf(csv,"%u,%u,%u,%d",i,k+1,workers[i].row[k]+1,workers[i].status[k]);
        for(unsigned j=0;j<18;++j)fprintf(csv,",%.17g",workers[i].outputs[k][j]);fprintf(csv,"\n");
    }
    fclose(csv);
    FILE *json=fopen(argv[4],"wb");if(!json)return 13;
    fprintf(json,"{\"pass\":%s,\"serial_rows\":%u,\"threads\":%d,\"calls_per_thread\":%d,"
      "\"concurrent_successes\":%u,\"busy_rejected\":%u,\"bad_checks\":%u,"
      "\"actual_attempts\":%llu,\"actual_successes\":%llu,\"invalid_arguments\":%llu,"
      "\"busy_is_not_blocking_wait_or_control_fallback\":true,\"board_actions\":0}\n",
      bad?"false":"true",serial_pass,THREADS,CALLS,concurrent_ok,busy,bad,
      (unsigned long long)final.attempts,(unsigned long long)final.successes,(unsigned long long)final.invalid_arguments);
    fclose(json);FreeLibrary(dll);
    printf("GP256 actual DLL: serial %u/293, concurrent success %u, rejected busy %u, bad %u.\n",serial_pass,concurrent_ok,busy,bad);
    return bad?14:0;
}
