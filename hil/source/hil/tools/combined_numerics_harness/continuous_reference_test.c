/* Replay 3765 saved-task queries through the combined query/transition C.
 * Calls share one serially owned resident SD. */
#include "gpenmpcNative_queryCanonicalReferenceWindow.h"
#include "gpenmpcNative_canonicalReferenceTransitionFromJet.h"
#include "gpenmpcNative_canonicalLocalInnerFixedFirst_initialize.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static f_gpenmpcNative_canonicalLocalIn workspace;
static struct51_T windows[32];
static struct52_T state,next;
static struct53_T request;
static struct54_T receipt;
static struct55_T transition;
static unsigned checks,passed,bad;
static void check(int ok){checks++;passed+=(unsigned)(ok!=0);}
static int take(FILE*f,void*p,size_t size,size_t n){return fread(p,size,n,f)==n;}
static int read_window(FILE*f,struct51_T*w){
  uint32_t m[7];double s[5];
  if(!take(f,m,4,7)||!take(f,w->reference_asset_sha256,1,32)||!take(f,&w->window_generation,8,1)
    ||!take(f,s,8,5)||!take(f,w->time_s,8,256)||!take(f,w->nominal_jet,8,3072)
    ||!take(f,w->prefix_coefficients,8,192)||!take(f,w->ground_jet,8,12)||!take(f,w->rest_jet,8,12)
    ||!take(f,w->relaunch_offset_ned_m,8,3))return 0;
  w->schema=m[0];w->capacity=(unsigned short)m[1];w->leg_index=m[2];w->source_first_row=m[3];
  w->source_total_rows=m[4];w->row_count=(unsigned short)m[5];w->binding_mode=(unsigned char)m[6];
  w->nominal_duration_s=s[0];w->total_duration_s=s[1];w->prefix_duration_s=s[2];w->relaunch_duration_s=s[3];
  w->vertical_frame_offset_ned_m=s[4];return 1;
}
static int reject(const struct51_T*w,unsigned char reason){
  double jet[12];gpenmpcNative_queryCanonicalReferenceWindow(&workspace,w,&state,&request,&next,jet,&receipt);
  if(receipt.accepted||receipt.reason!=reason||next.last_accepted_sequence!=state.last_accepted_sequence)return 0;
  for(unsigned j=0;j<12;j++)if(!isnan(jet[j]))return 0;
  return 1;
}
static void flatten(double y[41]){
  memcpy(y,transition.reference.position_m,24);memcpy(y+3,transition.reference.velocity_mps,24);
  memcpy(y+6,transition.reference.acceleration_mps2,24);memcpy(y+9,transition.reference.jerk_mps3,24);
  y[12]=transition.phase_acceleration_s_inv;y[13]=transition.phase_jerk_s_inv2;
  memcpy(y+14,transition.outer_correction_i_mps2,24);memcpy(y+17,transition.outer_correction_jerk_i_mps3,24);
  y[20]=transition.fraction;memcpy(y+21,transition.frame_i_from_f,72);memcpy(y+30,transition.reference_frame_i_from_f,72);
  y[39]=transition.reference_curvature;y[40]=transition.reference_signed_yaw_rate;
}
int main(int argc,char**argv){
  if(argc!=3)return 2;FILE*f=fopen(argv[1],"rb");if(!f)return 3;
  char magic[4];uint32_t nw=0,nr=0;
  if(!take(f,magic,1,4)||memcmp(magic,"RWI1",4)||!take(f,&nw,4,1)||!take(f,&nr,4,1)||nw>32||nr!=3765)return 4;
  for(uint32_t j=0;j<nw;j++)if(!read_window(f,&windows[j]))return 5;
  gpenmpcNative_canonicalLocalInnerFixedFirst_initialize();
  unsigned last_leg=0,last_window=0,refills=0,rejections=0,calls=0,exact_jet=0,exact_transition=0;
  unsigned leg_bad[5]={0},leg_refills[5]={0};double max_jet=0,max_transition=0;
  for(uint32_t row=0;row<nr;row++){
    uint32_t ids[3];unsigned long long sequence;double phase,args[11],expected_jet[12],expected[41],jet[12],got[41];
    if(!take(f,ids,4,3)||!take(f,&sequence,8,1)||!take(f,&phase,8,1)||!take(f,args,8,11)
      ||!take(f,expected_jet,8,12)||!take(f,expected,8,41)||ids[0]<1||ids[0]>nw||ids[1]<1||ids[1]>5)return 6;
    struct51_T*w=&windows[ids[0]-1];
    request.query_sequence=sequence;request.progress_s=phase;
    if(ids[1]==last_leg&&ids[0]!=last_window){
      const struct51_T*previous=&windows[last_window-1];
      memcpy(request.reference_asset_sha256,previous->reference_asset_sha256,32);
      request.leg_index=previous->leg_index;request.window_generation=previous->window_generation;
      check(reject(previous,5));refills++;leg_refills[ids[1]-1]++;
    }
    memcpy(state.reference_asset_sha256,w->reference_asset_sha256,32);state.leg_index=w->leg_index;
    state.window_generation=w->window_generation; // explicit original-row installation, preserves query sequence
    memcpy(request.reference_asset_sha256,w->reference_asset_sha256,32);request.leg_index=w->leg_index;
    request.window_generation=w->window_generation;
    if(ids[2]==2){request.leg_index=w->leg_index%5+1;check(reject(w,2));request.leg_index=w->leg_index;rejections++;}
    if(ids[2]==3){request.query_sequence=state.last_accepted_sequence;check(reject(w,3));request.query_sequence=sequence;rejections++;}
    gpenmpcNative_queryCanonicalReferenceWindow(&workspace,w,&state,&request,&next,jet,&receipt);
    if(!receipt.accepted||next.last_accepted_sequence!=sequence){bad++;leg_bad[ids[1]-1]++;}
    state=next;
    exact_jet+=(unsigned)(memcmp(jet,expected_jet,sizeof jet)==0);
    for(unsigned j=0;j<12;j++){double e=fabs(jet[j]-expected_jet[j]);if(e>max_jet)max_jet=e;
      if(!isfinite(jet[j])||e>2e-11*fmax(1,fabs(expected_jet[j]))){bad++;leg_bad[ids[1]-1]++;}}
    // Carry ACTUAL C's preceding transition state within each numerical leg.
    // The caller-supplied q is not advanced or derived here.
    if(ids[1]==last_leg){args[1]=transition.phase_acceleration_s_inv;memcpy(args+3,transition.outer_correction_i_mps2,24);}
    gpenmpcNative_canonicalReferenceTransitionFromJet(jet,args[0],args[1],args[2],args+3,args+6,args[9],args[10],&transition);
    calls++;flatten(got);exact_transition+=(unsigned)(memcmp(got,expected,sizeof got)==0);
    for(unsigned j=0;j<41;j++){double e=fabs(got[j]-expected[j]);if(e>max_transition)max_transition=e;
      if(!isfinite(got[j])||e>2e-11*fmax(1,fabs(expected[j]))){bad++;leg_bad[ids[1]-1]++;}}
    last_leg=ids[1];last_window=ids[0];
  }
  if(fgetc(f)!=EOF)return 7;fclose(f);
  for(unsigned j=0;j<5;j++){check(leg_bad[j]==0);check(leg_refills[j]>=2);}
  check(bad==0);check(state.last_accepted_sequence==nr);check(calls==nr&&refills==24);
  FILE*out=fopen(argv[2],"w");if(!out)return 8;
  fprintf(out,"{\"pass\":%s,\"checks\":%u,\"passed\":%u,\"queries\":%u,\"transition_calls\":%u,\"refills\":%u,\"identity_sequence_rejections\":%u,\"bit_exact_jet_queries\":%u,\"bit_exact_transition_queries\":%u,\"maximum_jet_error\":%.17g,\"maximum_transition_error\":%.17g,\"bad_values\":%u,\"same_explicit_serial_workspace\":true,\"inner_fixture_joined\":false,\"hardware_actions\":0}\n",
    checks==passed&&bad==0?"true":"false",checks,passed,nr,calls,refills,rejections,exact_jet,exact_transition,max_jet,max_transition,bad);
  fclose(out);printf("CONTINUOUS_REFERENCE %u/%u + external source-binding check; query%u max %.17g\n",passed,checks,nr,max_transition);
  return checks==passed&&bad==0?0:9;
}
