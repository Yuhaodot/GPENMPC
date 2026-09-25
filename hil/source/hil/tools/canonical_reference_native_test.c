#include "gpenmpcNative_queryCanonicalReferenceWindow.h"
#include "gpenmpcNative_canonicalReferenceTransitionFromJet.h"
#include "gpenmpcNative_queryCanonicalReferenceWindow_initialize.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static struct0_T window;
static struct1_T state, next_state;
static struct2_T request;
static struct3_T receipt;
static struct4_T transition;
static int take(FILE *f, void *p, size_t n, size_t count) { return fread(p,n,count,f)==count; }
static int rejected(unsigned char reason) {
  double jet[12];
  gpenmpcNative_queryCanonicalReferenceWindow(&window,&state,&request,&next_state,jet,&receipt);
  if(receipt.accepted||receipt.reason!=reason||next_state.last_accepted_sequence!=state.last_accepted_sequence) return 0;
  for(int k=0;k<12;k++) if(!isnan(jet[k])) return 0;
  return 1;
}
static void flatten(double y[41]) {
  memcpy(y,transition.reference.position_m,3*sizeof(double));
  memcpy(y+3,transition.reference.velocity_mps,3*sizeof(double));
  memcpy(y+6,transition.reference.acceleration_mps2,3*sizeof(double));
  memcpy(y+9,transition.reference.jerk_mps3,3*sizeof(double));
  y[12]=transition.phase_acceleration_s_inv;y[13]=transition.phase_jerk_s_inv2;
  memcpy(y+14,transition.outer_correction_i_mps2,3*sizeof(double));
  memcpy(y+17,transition.outer_correction_jerk_i_mps3,3*sizeof(double));y[20]=transition.fraction;
  memcpy(y+21,transition.frame_i_from_f,9*sizeof(double));
  memcpy(y+30,transition.reference_frame_i_from_f,9*sizeof(double));
  y[39]=transition.reference_curvature;y[40]=transition.reference_signed_yaw_rate;
}
int main(int argc,char **argv) {
  if(argc!=3) return 2;
  FILE *f=fopen(argv[1],"rb");if(!f) return 3;
  char magic[4];uint32_t count=0;
  if(!take(f,magic,1,4)||memcmp(magic,"RWJ1",4)||!take(f,&count,4,1)) return 4;
  gpenmpcNative_queryCanonicalReferenceWindow_initialize();
  double max_jet=0,max_transition=0;uint32_t checks=0,passed=0,bit_jet=0,bit_transition=0;
  for(uint32_t row=0;row<count;row++) {
    uint32_t meta[7];double scalar[5],args[11],expected_jet[12],expected[41],jet[12],got[41];
    if(!take(f,meta,4,7)||!take(f,window.reference_asset_sha256,1,32)||!take(f,&window.window_generation,8,1)
      ||!take(f,scalar,8,5)||!take(f,window.time_s,8,256)||!take(f,window.nominal_jet,8,3072)
      ||!take(f,window.prefix_coefficients,8,192)||!take(f,window.ground_jet,8,12)
      ||!take(f,window.rest_jet,8,12)||!take(f,window.relaunch_offset_ned_m,8,3)
      ||!take(f,&state.last_accepted_sequence,8,1)||!take(f,&request.query_sequence,8,1)
      ||!take(f,&request.progress_s,8,1)||!take(f,args,8,11)||!take(f,expected_jet,8,12)||!take(f,expected,8,41)) return 5;
    window.schema=meta[0];window.capacity=(unsigned short)meta[1];window.leg_index=meta[2];
    window.source_first_row=meta[3];window.source_total_rows=meta[4];window.row_count=(unsigned short)meta[5];
    window.binding_mode=(unsigned char)meta[6];window.nominal_duration_s=scalar[0];window.total_duration_s=scalar[1];
    window.prefix_duration_s=scalar[2];window.relaunch_duration_s=scalar[3];window.vertical_frame_offset_ned_m=scalar[4];
    memcpy(state.reference_asset_sha256,window.reference_asset_sha256,32);
    memcpy(request.reference_asset_sha256,window.reference_asset_sha256,32);
    state.leg_index=request.leg_index=window.leg_index;
    state.window_generation=request.window_generation=window.window_generation;
    gpenmpcNative_queryCanonicalReferenceWindow(&window,&state,&request,&next_state,jet,&receipt);
    checks++;passed+=receipt.accepted&&next_state.last_accepted_sequence==request.query_sequence;
    bit_jet+=(memcmp(jet,expected_jet,sizeof(jet))==0);
    int jet_ok=1;
    for(int k=0;k<12;k++){double e=fabs(jet[k]-expected_jet[k]);if(e>max_jet)max_jet=e;
      if(!isfinite(jet[k])||e>2e-11*fmax(1,fabs(expected_jet[k])))jet_ok=0;}
    checks++;passed+=(uint32_t)jet_ok;
    gpenmpcNative_canonicalReferenceTransitionFromJet(jet,args[0],args[1],args[2],args+3,args+6,args[9],args[10],&transition);
    flatten(got);bit_transition+=(memcmp(got,expected,sizeof(got))==0);int output_ok=1;
    for(int k=0;k<41;k++){double e=fabs(got[k]-expected[k]);if(e>max_transition)max_transition=e;
      if(!isfinite(got[k])||e>2e-11*fmax(1,fabs(expected[k])))output_ok=0;}
    checks++;passed+=(uint32_t)output_ok;
    unsigned long long original=request.query_sequence;
    request.query_sequence=state.last_accepted_sequence;checks++;passed+=rejected(3);request.query_sequence=original;
    request.window_generation++;checks++;passed+=rejected(2);request.window_generation--;
    request.reference_asset_sha256[0]^=1;checks++;passed+=rejected(2);request.reference_asset_sha256[0]^=1;
    double phase=request.progress_s;request.progress_s=NAN;checks++;passed+=rejected(4);request.progress_s=phase;
  }
  if(fgetc(f)!=EOF) return 6;fclose(f);
  FILE *out=fopen(argv[2],"w");if(!out)return 7;
  fprintf(out,"{\"pass\":%s,\"cases\":%u,\"checks\":%u,\"passed\":%u,\"bit_exact_jet_cases\":%u,\"bit_exact_transition_cases\":%u,\"maximum_jet_error\":%.17g,\"maximum_transition_error\":%.17g,\"numerical_tolerance_scaled\":2e-11,\"sizeof_window_bytes\":%zu,\"sizeof_state_bytes\":%zu,\"sizeof_transition_bytes\":%zu,\"hardware_actions\":0,\"matlab_runtime_dependency\":false}\n",
    checks==passed?"true":"false",count,checks,passed,bit_jet,bit_transition,max_jet,max_transition,sizeof(window),sizeof(state),sizeof(transition));
  fclose(out);printf("REFERENCE_NATIVE %u/%u cases=%u jet=%.17g transition=%.17g\n",passed,checks,count,max_jet,max_transition);
  return passed==checks?0:8;
}
