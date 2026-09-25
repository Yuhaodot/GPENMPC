// Manage process-local timer resolution on Windows 10 2004 or later.
// Retain active status until release succeeds.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>
#include "mex.h"
static bool active=false, registered=false;
static unsigned begins=0, ends=0;
static MMRESULT last=TIMERR_NOERROR;
static void release_at_exit() {
    if(active){last=timeEndPeriod(1);if(last==TIMERR_NOERROR){active=false;++ends;}}
}
void mexFunction(int nlhs,mxArray** out,int nrhs,const mxArray** in) {
    if(nrhs!=1||nlhs!=1||!mxIsDouble(in[0])||mxIsComplex(in[0])||mxGetNumberOfElements(in[0])!=1)
        mexErrMsgIdAndTxt("gpenmpcNative:TimerArguments","Use exactly one double command and one receipt.");
    const double command=mxGetScalar(in[0]);
    if(command!=0&&command!=1&&command!=2)
        mexErrMsgIdAndTxt("gpenmpcNative:TimerCommand","0=release, 1=begin, 2=status only.");
    TIMECAPS caps{};const MMRESULT available=timeGetDevCaps(&caps,sizeof(caps));
    if(!registered){mexAtExit(release_at_exit);registered=true;}
    if(command==1){
        if(active)mexErrMsgIdAndTxt("gpenmpcNative:TimerOwner","Only one calling-process timer request owner.");
        if(available!=TIMERR_NOERROR||caps.wPeriodMin>1||caps.wPeriodMax<1)
            mexErrMsgIdAndTxt("gpenmpcNative:TimerUnsupported","The fixed 1-ms request is unsupported.");
        last=timeBeginPeriod(1);
        if(last==TIMERR_NOERROR){active=true;++begins;mexLock();}
    }else if(command==0&&active){
        last=timeEndPeriod(1);
        if(last==TIMERR_NOERROR){active=false;++ends;mexUnlock();}
    }
    const char* names[]={"active","requested_period_ms","capability_result","minimum_period_ms",
        "maximum_period_ms","begin_successes","end_successes","last_result","process_id"};
    const double values[]={active?1.0:0.0,1.0,static_cast<double>(available),static_cast<double>(caps.wPeriodMin),
        static_cast<double>(caps.wPeriodMax),static_cast<double>(begins),static_cast<double>(ends),
        static_cast<double>(last),static_cast<double>(GetCurrentProcessId())};
    out[0]=mxCreateStructMatrix(1,1,9,names);
    for(unsigned k=0;k<9;++k)mxSetFieldByNumber(out[0],0,k,mxCreateDoubleScalar(values[k]));
}
