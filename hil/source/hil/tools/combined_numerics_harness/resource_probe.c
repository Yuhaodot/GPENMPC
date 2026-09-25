#include "gpenmpcNative_canonicalLocalInnerFixedFirst_types.h"
#include <stddef.h>
#include <stdio.h>
int main(int argc,char **argv) {
  if(argc!=2)return 2;FILE *f=fopen(argv[1],"w");if(!f)return 3;
  fprintf(f,"{\"sizeof_owner_spill_bytes\":%zu,\"spill_u1_bytes\":%zu,\"spill_u2_bytes\":%zu,\"spill_u3_bytes\":%zu,\"spill_u4_bytes\":%zu,\"sizeof_window_bytes\":%zu,\"sizeof_query_state_bytes\":%zu,\"sizeof_query_request_bytes\":%zu,\"sizeof_transition_bytes\":%zu,\"compiler_abi\":\"HOST_x86_64\",\"target_stack_proven\":false,\"board_actions\":0}\n",
    sizeof(f_gpenmpcNative_canonicalLocalIn),sizeof(((f_gpenmpcNative_canonicalLocalIn*)0)->u1),
    sizeof(((f_gpenmpcNative_canonicalLocalIn*)0)->u2),sizeof(((f_gpenmpcNative_canonicalLocalIn*)0)->u3),
    sizeof(((f_gpenmpcNative_canonicalLocalIn*)0)->u4),sizeof(struct51_T),sizeof(struct52_T),sizeof(struct53_T),sizeof(struct55_T));
  fclose(f);return 0;
}
