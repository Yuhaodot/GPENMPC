// Host gateway for the TakeoffHandling and Hysteresis translation units.
#include "mex.h"
#include <modules/mc_pos_control/Takeoff/Takeoff.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

static void require(bool v,const char *m) { if(!v) throw std::runtime_error(m); }
template<size_t N> static void exact(const mxArray *a,const char *const (&keys)[N]) {
    require(mxIsStruct(a)&&mxGetNumberOfElements(a)==1,"Expected scalar struct");
    require(mxGetNumberOfFields(a)==static_cast<int>(N),"Missing or unknown field");
    for(const char *k:keys) require(mxGetField(a,0,k)!=nullptr,"Missing required field");
}
static const mxArray *field(const mxArray *a,const char *n) {
    const mxArray *v=mxGetField(a,0,n);require(v!=nullptr,"Missing field");return v;
}
static double number(const mxArray *a,const char *n) {
    const mxArray *v=field(a,n);require(mxIsDouble(v)&&!mxIsComplex(v)&&!mxIsSparse(v)&&mxGetNumberOfElements(v)==1,"Expected real scalar double");
    double x=mxGetScalar(v);require(std::isfinite(x)&&std::isfinite(static_cast<float>(x)),"Nonfinite or float-overflow value");return x;
}
static bool flag(const mxArray *a,const char *n) {
    const mxArray *v=field(a,n);require(mxIsLogical(v)&&mxGetNumberOfElements(v)==1,"Expected scalar logical");return mxIsLogicalScalarTrue(v);
}
static std::string text(const mxArray *a) {
    require(mxIsChar(a)&&mxGetM(a)==1,"Expected char row");char *p=mxArrayToString(a);require(p!=nullptr,"String conversion failed");
    std::string s(p);mxFree(p);return s;
}
static void put(mxArray *out,const char *n,mxArray *v) { mxSetField(out,0,n,v); }
static void scalar(mxArray *out,const char *n,double v) { put(out,n,mxCreateDoubleScalar(v)); }
static mxArray *uint64Value(uint64_t v) { auto *a=mxCreateNumericMatrix(1,1,mxUINT64_CLASS,mxREAL);*static_cast<uint64_t*>(mxGetData(a))=v;return a; }
static const char *stateName(TakeoffState s) {
    switch(s) {
        case TakeoffState::disarmed:return "DISARMED";
        case TakeoffState::spoolup:return "SPOOLUP";
        case TakeoffState::ready_for_takeoff:return "READY_FOR_TAKEOFF";
        case TakeoffState::rampup:return "RAMPUP";
        case TakeoffState::flight:return "FLIGHT";
    }
    throw std::runtime_error("Unknown original TakeoffState");
}
struct Config {
    float spoolup{},ramp{},p{};std::string provenance;
    explicit Config(const mxArray *a) {
        const char *const keys[]={"spoolup_time_s","ramp_time_s","vertical_velocity_p","parameter_provenance"};exact(a,keys);
        spoolup=static_cast<float>(number(a,"spoolup_time_s"));ramp=static_cast<float>(number(a,"ramp_time_s"));p=static_cast<float>(number(a,"vertical_velocity_p"));
        require(spoolup>=0&&ramp>=0&&p>=0,"Negative parameter");
        require(static_cast<long double>(spoolup)*1000000.L<static_cast<long double>(std::numeric_limits<uint64_t>::max())/2.L,"Spoolup time overflows native timestamp arithmetic");
        provenance=text(field(a,"parameter_provenance"));
        require(provenance=="CURRENT_TYPED_WITH_EXPLICIT_MODULE_PHASE"||provenance=="EXPLICIT_HOST_FIXTURE","Unknown provenance");
    }
};
struct Kernel {
    Config cfg;TakeoffHandling takeoff;bool haveTime{false};uint64_t lastTime{};size_t count{};
    explicit Kernel(const Config &c):cfg(c) {
        takeoff.setSpoolupTime(cfg.spoolup);takeoff.setTakeoffRampTime(cfg.ramp);takeoff.generateInitialRampValue(cfg.p);
    }
    mxArray *step(const mxArray *s) {
        const char *const keys[]={"source_timestamp_us","initial_dt_s","armed","landed","want_takeoff","skip_takeoff","takeoff_desired_velocity_up_mps"};exact(s,keys);
        const mxArray *clock=field(s,"source_timestamp_us");
        require(mxIsUint64(clock)&&!mxIsComplex(clock)&&mxGetNumberOfElements(clock)==1,"Timestamp must be exact uint64 microseconds");
        const uint64_t time=*static_cast<const uint64_t*>(mxGetData(clock));
        require(time>0&&(!haveTime||time>lastTime),"Source timestamp repeated or reversed");
        const uint64_t nativeSpoolup=static_cast<uint64_t>(cfg.spoolup*1000000ULL);
        require(time<=std::numeric_limits<uint64_t>::max()-nativeSpoolup,"Timestamp/hysteresis addition overflow");
        const double first=number(s,"initial_dt_s");require(first>0&&static_cast<float>(first)>0,"Explicit initial dt must be positive");
        const bool armed=flag(s,"armed"),landed=flag(s,"landed"),want=flag(s,"want_takeoff"),skip=flag(s,"skip_takeoff");
        const float desired=static_cast<float>(number(s,"takeoff_desired_velocity_up_mps"));require(desired>=0,"Desired upward speed magnitude must be nonnegative");
        // This is the explicit caller-side mc_pos_control dt policy, not a
        // hidden property of TakeoffHandling. The source time itself is never
        // clamped/reset and drives the original spool-up hysteresis.
        const double sourceDelta=haveTime?static_cast<double>(time-lastTime)*1e-6:first;
        const float rawDt=haveTime?static_cast<float>(time-lastTime)*1e-6f:static_cast<float>(first);
        require(std::isfinite(rawDt)&&rawDt>0,"Invalid source dt");
        const float dt=rawDt<.002f?.002f:(rawDt>.04f?.04f:rawDt);
        const auto before=takeoff.getTakeoffState();
        takeoff.updateTakeoffState(armed,landed,want,desired,skip,time);
        const auto after=takeoff.getTakeoffState();
        const float limit=takeoff.updateRamp(dt,desired);
        require(std::isfinite(limit),"Original takeoff produced nonfinite velocity limit");
        const char *fields[]={"state_before","state_after","state_name","upward_velocity_limit_mps",
            "source_timestamp_us","source_delta_s","raw_dt_float_s","dt_s","dt_clamped",
            "initial_dt_used","not_taken_off","flying","armed_input","landed_input",
            "want_takeoff_input","skip_takeoff_input","desired_upward_velocity_mps","sample_count",
            "native_p_floor_applied","parameter_provenance","private_ramp_progress_observed",
            "authority_granted","claim"};
        auto *out=mxCreateStructMatrix(1,1,23,fields);
        scalar(out,"state_before",static_cast<int>(before));scalar(out,"state_after",static_cast<int>(after));put(out,"state_name",mxCreateString(stateName(after)));
        scalar(out,"upward_velocity_limit_mps",limit);put(out,"source_timestamp_us",uint64Value(time));
        scalar(out,"source_delta_s",sourceDelta);scalar(out,"raw_dt_float_s",rawDt);scalar(out,"dt_s",dt);
        put(out,"dt_clamped",mxCreateLogicalScalar(rawDt!=dt));put(out,"initial_dt_used",mxCreateLogicalScalar(!haveTime));
        put(out,"not_taken_off",mxCreateLogicalScalar(after<TakeoffState::rampup));put(out,"flying",mxCreateLogicalScalar(after>=TakeoffState::flight));
        put(out,"armed_input",mxCreateLogicalScalar(armed));put(out,"landed_input",mxCreateLogicalScalar(landed));
        put(out,"want_takeoff_input",mxCreateLogicalScalar(want));put(out,"skip_takeoff_input",mxCreateLogicalScalar(skip));
        scalar(out,"desired_upward_velocity_mps",desired);scalar(out,"sample_count",static_cast<double>(++count));
        put(out,"native_p_floor_applied",mxCreateLogicalScalar(cfg.p<.01f));put(out,"parameter_provenance",mxCreateString(cfg.provenance.c_str()));
        put(out,"private_ramp_progress_observed",mxCreateLogicalScalar(false));put(out,"authority_granted",mxCreateLogicalScalar(false));
        put(out,"claim",mxCreateString("TAKEOFF_AND_HYSTERESIS_HOST_NUMERICS_WITH_EXPLICIT_MODULE_DT"));
        haveTime=true;lastTime=time;return out;
    }
};
static std::unique_ptr<Kernel> instance;
static void clearInstance() { instance.reset(); }
void mexFunction(int nlhs,mxArray **plhs,int nrhs,const mxArray **prhs) {
    static bool registered=false;if(!registered){mexAtExit(clearInstance);registered=true;}
    try {
        require(nrhs>=1,"Command required");const std::string command=text(prhs[0]);
        if(command=="clear") {require(nrhs==1&&nlhs<=1,"clear arguments");clearInstance();if(nlhs)plhs[0]=mxCreateLogicalScalar(true);return;}
        if(command=="reset") {require(nrhs==1&&nlhs==1&&instance!=nullptr,"reset requires an existing instance and one output");Config c=instance->cfg;instance=std::make_unique<Kernel>(c);plhs[0]=mxCreateLogicalScalar(true);return;}
        require(nrhs==2&&nlhs==1,"init/step require one struct and one output");
        if(command=="init") {clearInstance();Config c(prhs[1]);instance=std::make_unique<Kernel>(c);plhs[0]=mxCreateLogicalScalar(true);return;}
        require(command=="step"&&instance!=nullptr,"Unknown command or uninitialized kernel");plhs[0]=instance->step(prhs[1]);
    } catch(const std::exception &e) {clearInstance();mexErrMsgIdAndTxt("gpenmpc:NativeTakeoffKernel","%s",e.what());}
}
