/* Benchmark generated C within one native process. */
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gpenmpcNative_canonicalSparseGpFixedInput.h"
#include "gpenmpcNative_canonicalSparseGpFixedInput_initialize.h"
#include "gpenmpcNative_canonicalSparseGpFixedInput_terminate.h"

enum { kRows=293, kInputs=17, kOutputs=18, kWarmup=16 };
static double inputs[kRows][kInputs];
static double expected[kRows][kOutputs];
static double actual[kRows][kOutputs];
static double seconds[kRows];
static double errors[kRows];
static unsigned row_bad[kRows];

static int load_fixture(const char *path)
{
    FILE *f=fopen(path,"rb");
    char magic[8]; uint32_t sizes[3];
    if (!f) return 1;
    int ok=fread(magic,1,8,f)==8 && memcmp(magic,"RAGP2561",8)==0;
    ok=ok && fread(sizes,sizeof(uint32_t),3,f)==3;
    ok=ok && sizes[0]==kRows && sizes[1]==kInputs && sizes[2]==kOutputs;
    for (unsigned i=0;i<kRows && ok;++i) {
        ok=fread(inputs[i],sizeof(double),kInputs,f)==kInputs;
        ok=ok && fread(expected[i],sizeof(double),kOutputs,f)==kOutputs;
    }
    ok=ok && fgetc(f)==EOF;
    fclose(f); return !ok;
}

int main(int argc,char **argv)
{
    if (argc!=5) {fprintf(stderr,"usage: gp_test fixture timing.csv actual.bin result.json\n");return 2;}
    if (sizeof(double)!=8 || load_fixture(argv[1])) {fprintf(stderr,"Invalid fixture.\n");return 3;}
    LARGE_INTEGER frequency,start,end;
    if (!QueryPerformanceFrequency(&frequency) || frequency.QuadPart<=0) return 4;
    gpenmpcNative_canonicalSparseGpFixedInput_initialize();
    double warm[kOutputs];
    for (unsigned i=0;i<kWarmup;++i) gpenmpcNative_canonicalSparseGpFixedInput(inputs[i],warm);
    double max_abs=0,max_scaled=0,max_s=0,total_s=0;
    unsigned bad=0,nonfinite_bad=0,classification_bad=0;
    for (unsigned i=0;i<kRows;++i) {
        QueryPerformanceCounter(&start);
        gpenmpcNative_canonicalSparseGpFixedInput(inputs[i],actual[i]);
        QueryPerformanceCounter(&end);
        seconds[i]=(double)(end.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
        if (seconds[i]>max_s) max_s=seconds[i];
        total_s+=seconds[i];
        for (unsigned j=0;j<kOutputs;++j) {
            const double a=actual[i][j],e=expected[i][j];
            if (isnan(a)!=isnan(e) || isfinite(a)!=isfinite(e) ||
                    (isinf(e) && (signbit(a)!=signbit(e)))) {
                ++nonfinite_bad; ++row_bad[i]; continue;
            }
            if (isfinite(e)) {
                const double delta=fabs(a-e),scaled=delta/fmax(1.0,fabs(e));
                if (delta>max_abs) max_abs=delta;
                if (scaled>max_scaled) max_scaled=scaled;
                if (scaled>errors[i]) errors[i]=scaled;
                if (scaled>=1.0e-10) ++row_bad[i];
            }
        }
        if (actual[i][14]!=expected[i][14]) {++classification_bad;++row_bad[i];}
        if (row_bad[i]) ++bad;
    }
    gpenmpcNative_canonicalSparseGpFixedInput_terminate();
    FILE *csv=fopen(argv[2],"wb");
    if (!csv) return 5;
    fprintf(csv,"row,native_predictor_s,max_scaled_error,bad_values,hard_invalid,trust\n");
    for (unsigned i=0;i<kRows;++i)
        fprintf(csv,"%u,%.17g,%.17g,%u,%.17g,%.17g\n",i+1,seconds[i],errors[i],row_bad[i],actual[i][14],actual[i][13]);
    fclose(csv);
    FILE *raw=fopen(argv[3],"wb");if(!raw)return 6;
    if(fwrite(actual,sizeof(actual),1,raw)!=1){fclose(raw);return 7;}fclose(raw);
    FILE *json=fopen(argv[4],"wb");if(!json)return 8;
    fprintf(json,"{\n\"pass\":%s,\n\"row_count\":%d,\n\"output_count\":%d,\n"
        "\"bad_rows\":%u,\n\"nonfinite_mismatches\":%u,\n\"hard_invalid_mismatches\":%u,\n"
        "\"maximum_absolute_error\":%.17g,\n\"maximum_scaled_error\":%.17g,\n"
        "\"timed_calls\":%d,\n\"warmup_calls\":%d,\n\"mean_s\":%.17g,\n\"max_s\":%.17g,\n"
        "\"counter_frequency_hz\":%lld,\n\"single_native_process\":true,\n"
        "\"matlab_runtime_used\":false,\n\"board_actions\":0,\n\"solver_calls\":0,\n\"plant_runs\":0\n}\n",
        bad?"false":"true",kRows,kOutputs,bad,nonfinite_bad,classification_bad,max_abs,max_scaled,
        kRows,kWarmup,total_s/kRows,max_s,(long long)frequency.QuadPart);
    fclose(json);
    printf("GP256 standalone: %u/%u rows, max scaled %.9g, mean %.9g s, max %.9g s.\n",kRows-bad,(unsigned)kRows,max_scaled,total_s/kRows,max_s);
    return bad?9:0;
}
