/* Host SIL runner for generated controller C. Reads a fixed fixture and writes stdout. */
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "GPENMPC_Rfly_Canonical_Controller.h"
int main(int argc,char **argv) {
    if(argc!=2)return 2;
    FILE *f=fopen(argv[1],"rb");if(!f)return 3;
    uint32_t n=0,valid_rows=0,checks=0;
    double max_error=0;const unsigned map[6]={4,0,3,5,1,2};
    if(fread(&n,sizeof(n),1,f)!=1 || n!=2129){fclose(f);return 4;}
    GPENMPC_Rfly_Canonical_Controller_initialize();
    for(uint32_t k=0;k<n;++k) {
        double row[164];if(fread(row,sizeof(double),164,f)!=164){fclose(f);return 5;}
        for(unsigned i=0;i<34;++i)GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[i]=row[i];
        for(unsigned i=34;i<101;++i)GPENMPC_Rfly_Canonical_Control_U.KernelArguments101[i]=row[i+1];
        GPENMPC_Rfly_Canonical_Control_U.ContinuityEnabled=(row[34]!=0);
        GPENMPC_Rfly_Canonical_Control_U.InputGenerationAccepted=1;
        GPENMPC_Rfly_Canonical_Controller_step();
        const int valid=row[163]!=0;
        if((GPENMPC_Rfly_Canonical_Control_Y.OutputValid!=0)!=valid){fclose(f);return 6;}
        ++checks;valid_rows+=(uint32_t)valid;
        for(unsigned j=0;j<61;++j){
            const double e=fabs(GPENMPC_Rfly_Canonical_Control_Y.FullKernel61[j]-row[102+j]);
            if(!isfinite(e)||e>1e-10){fclose(f);return 7;}
            if(e>max_error)max_error=e;++checks;
        }
        float expected[16]={0};
        if(valid)for(unsigned j=0;j<6;++j)expected[map[j]]=(float)(row[106+j]/32.145727009134916);
        if(memcmp(expected,GPENMPC_Rfly_Canonical_Control_Y.Controls16,sizeof(expected))!=0){fclose(f);return 8;}
        ++checks;
        /* An invalid enable must clear every output on the next actual step. */
        GPENMPC_Rfly_Canonical_Control_U.InputGenerationAccepted=0;
        GPENMPC_Rfly_Canonical_Controller_step();
        if(GPENMPC_Rfly_Canonical_Control_Y.OutputValid){fclose(f);return 9;}
        for(unsigned j=0;j<16;++j)if(GPENMPC_Rfly_Canonical_Control_Y.Controls16[j]!=0){fclose(f);return 10;}
        for(unsigned j=0;j<61;++j)if(GPENMPC_Rfly_Canonical_Control_Y.FullKernel61[j]!=0){fclose(f);return 11;}
        ++checks;
    }
    if(fgetc(f)!=EOF){fclose(f);return 12;}fclose(f);
    GPENMPC_Rfly_Canonical_Controller_terminate();
    printf("{\"status\":\"PASS_GENERATED_SIMULINK_C_HOST_SIL\",\"rows\":%u,\"valid_rows\":%u,"
           "\"checks\":%u,\"maximum_kernel61_error\":%.17g,\"controls16_byte_equal\":true,"
           "\"board_actions\":0,\"COM_open\":0,\"firmware_build\":0}\n",n,valid_rows,checks,max_error);
    return 0;
}
