// Encode validated MATLAB bindings as RLI1 packets.
#include "mex.h"
#include "../rfly_vendor_integration/px4_wire/CanonicalLocalTaskWire.hpp"
#include <cstring>
namespace tw=gpenmpc_local_task_wire;
static bool vector(const mxArray*a,mxClassID kind,mwSize n){
    return mxGetClassID(a)==kind&&!mxIsComplex(a)&&!mxIsSparse(a)&&mxGetNumberOfDimensions(a)==2&&
        (mxGetM(a)==1||mxGetN(a)==1)&&mxGetNumberOfElements(a)==n;
}
static tw::Hash hash(const uint8_t*p){
    tw::Hash h{};
    for(unsigned k=0;k<8;++k)h[k]=(uint32_t(p[4*k])<<24)|(uint32_t(p[4*k+1])<<16)|
        (uint32_t(p[4*k+2])<<8)|uint32_t(p[4*k+3]);
    return h;
}
void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
    if(nrhs!=6||nlhs<1||nlhs>3)
        mexErrMsgIdAndTxt("gpenmpcNative:LocalTaskWireMexArity","Six fixed typed inputs and one to three outputs required.");
    if(!vector(prhs[0],mxUINT64_CLASS,19)||!vector(prhs[1],mxUINT32_CLASS,1)||
       !vector(prhs[2],mxUINT8_CLASS,3)||!vector(prhs[3],mxDOUBLE_CLASS,14)||
       mxGetClassID(prhs[4])!=mxUINT8_CLASS||mxIsComplex(prhs[4])||mxIsSparse(prhs[4])||
       mxGetNumberOfDimensions(prhs[4])!=2||mxGetM(prhs[4])!=32||mxGetN(prhs[4])!=9||
       !vector(prhs[5],mxUINT8_CLASS,52))
        mexErrMsgIdAndTxt("gpenmpcNative:LocalTaskWireMexShape", "Expected uint64[19], uint32 scalar, uint8[3], double[14], uint8[32,9], uint8[52].");
    const auto*u=static_cast<const uint64_t*>(mxGetData(prhs[0]));
    const auto*c=static_cast<const uint8_t*>(mxGetData(prhs[2]));
    const auto*d=mxGetDoubles(prhs[3]);const auto*h=static_cast<const uint8_t*>(mxGetData(prhs[4]));
    tw::Message m{};auto&s=m.source;
    m.session_sha=hash(h);m.configuration_sha=hash(h+32);m.task_sha=hash(h+64);m.reference_sha=hash(h+96);
    m.leg=*static_cast<const uint32_t*>(mxGetData(prhs[1]));
    s.identity={u[0],u[1],c[0],c[1]};s.sample_us=u[2];s.publication_us=u[3];s.original_receipt_us=u[4];
    s.source_generation=u[5];s.generation_delta=u[6];s.sample_delta_us=u[7];s.reset_counter=c[2];
    s.state_and_origin_sha256=hash(h+128);std::memcpy(m.original_sensor52,mxGetData(prhs[5]),52);
    m.rotor_generation=u[8];m.rotor_session=u[9];m.rotor_host_receive_ns=u[10];m.rotor_sim_time_s=d[0];
    std::memcpy(m.rotor_n,d+1,sizeof(m.rotor_n));m.rotor_observation_sha=hash(h+160);m.rotor_association_sha=hash(h+192);
    m.payload_generation=u[11];m.payload_evidence_sha=hash(h+224);m.payload_kg=d[7];
    m.wind_generation=u[12];m.wind_evidence_sha=hash(h+256);std::memcpy(m.estimated_wind_xy,d+8,sizeof(m.estimated_wind_xy));
    m.outer_generation=u[13];m.outer_source_generation=u[14];m.outer_sample_us=u[15];
    m.outer_original_host_source_rx_ns=u[16];m.outer_creation_ns=u[17];m.outer_expiry_ns=u[18];
    std::memcpy(m.outer_target4,d+10,sizeof(m.outer_target4));
    if(m.configuration_sha!=gpenmpc_local_gp_wire::canonical_configuration())
        mexErrMsgIdAndTxt("gpenmpcNative:LocalTaskWireMexConfiguration","Original canonical configuration binding required.");
    tw::Bytes bytes{};tw::Message decoded{};
    if(!tw::encode(m,bytes)||!tw::decode(bytes,decoded))
        mexErrMsgIdAndTxt("gpenmpcNative:LocalTaskWireMexFields","Original C++ RLI1 field/digest validation rejected inputs.");
    plhs[0]=mxCreateNumericMatrix(bytes.size(),1,mxUINT8_CLASS,mxREAL);
    std::memcpy(mxGetData(plhs[0]),bytes.data(),bytes.size());
    if(nlhs>=2){
        plhs[1]=mxCreateNumericMatrix(128,tw::fragment_count,mxUINT8_CLASS,mxREAL);
        auto*payload=static_cast<uint8_t*>(mxGetData(plhs[1]));
        if(nlhs==3)plhs[2]=mxCreateNumericMatrix(tw::fragment_count,1,mxUINT8_CLASS,mxREAL);
        for(unsigned k=0;k<tw::fragment_count;++k){
            gpenmpc_argument_transport::Fragment f{};
            if(!tw::fragment(bytes,k,f))mexErrMsgIdAndTxt("gpenmpcNative:LocalTaskWireMexFragment","Original RLI1 fragmentation failed.");
            std::memcpy(payload+128*k,f.payload,128);
            if(nlhs==3)static_cast<uint8_t*>(mxGetData(plhs[2]))[k]=f.length;
        }
    }
}
