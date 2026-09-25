// Host gateway for PX4 attitude, rate and sequential allocation numerics.
#include "mex.h"
#include <cmath>
#include <cfloat>
#include <memory>
#include <stdexcept>
#include <string>
#include <modules/mc_att_control/AttitudeControl/AttitudeControl.hpp>
#include <rate_control/rate_control.hpp>
#include <control_allocation/control_allocation/ControlAllocationSequentialDesaturation.hpp>
#include <mathlib/math/filter/AlphaFilter.hpp>

using matrix::Vector3f;
using matrix::Quatf;
using Vector6f = matrix::Vector<float,6>;
using Vector16f = matrix::Vector<float,16>;
static void require(bool ok, const char *why) { if (!ok) throw std::runtime_error(why); }
static const mxArray *field(const mxArray *s,const char *key) {
    require(mxIsStruct(s) && mxGetNumberOfElements(s)==1,"Expected scalar struct");
    const mxArray *v=mxGetField(s,0,key); require(v!=nullptr,key); return v;
}
static const double *numbers(const mxArray *a,size_t n) {
    require(mxIsDouble(a)&&!mxIsComplex(a)&&!mxIsSparse(a)&&mxGetNumberOfElements(a)==n,"Expected real double field with exact size");
    const double *p=mxGetPr(a);
    for(size_t i=0;i<n;++i) require(std::isfinite(p[i]) && std::isfinite(static_cast<float>(p[i])),"Nonfinite/float-overflow input");
    return p;
}
static double scalar(const mxArray *s,const char *k) { return numbers(field(s,k),1)[0]; }
static bool boolean(const mxArray *s,const char *k) {
    const mxArray *a=field(s,k); require(mxIsLogicalScalar(a),"Boolean must be scalar logical"); return mxIsLogicalScalarTrue(a);
}
static std::string text(const mxArray *a) {
    require(mxIsChar(a),"Text must be MATLAB char"); char *p=mxArrayToString(a); require(p!=nullptr,"Text conversion");
    std::string s(p); mxFree(p); return s;
}
static Vector3f vector3(const mxArray *s,const char *k) {
    const double *p=numbers(field(s,k),3); return Vector3f(static_cast<float>(p[0]),static_cast<float>(p[1]),static_cast<float>(p[2]));
}
static Quatf quaternion(const mxArray *s,const char *k) {
    const double *p=numbers(field(s,k),4); double n=0; for(int i=0;i<4;++i)n+=p[i]*p[i];
    require(std::abs(n-1.0)<=1e-5,"Quaternion must be unit wxyz");
    return Quatf(static_cast<float>(p[0]),static_cast<float>(p[1]),static_cast<float>(p[2]),static_cast<float>(p[3]));
}
template<size_t N> static mxArray *outputVector(const matrix::Vector<float,N> &v) {
    mxArray *a=mxCreateDoubleMatrix(N,1,mxREAL); double *p=mxGetPr(a);
    for(size_t i=0;i<N;++i) { require(std::isfinite(v(i)),"Nonfinite kernel output"); p[i]=v(i); } return a;
}
static void put(mxArray *s,const char *k,mxArray *v) { mxSetField(s,0,k,v); }

struct Kernel {
    AttitudeControl attitude;
    RateControl rate;
    ControlAllocationSequentialDesaturation allocator;
    AlphaFilter<float> yawFilter;
    Vector3f heldRate;
    Vector3f rateLimit;
    Vector3f integralLimit;
    matrix::Vector<bool,3> satPositive;
    matrix::Vector<bool,3> satNegative;
    bool hasSlew{false}, batteryEnabled{false}, haveAttitude{false}, haveTime{false};
    double lastTime{0}, statusTime{-1};
    std::string provenance;
    explicit Kernel(const mxArray *c) {
        require(scalar(c,"mc_airmode")==0,"Only exact MC_AIRMODE=0 is supported");
        provenance=text(field(c,"parameter_provenance"));
        require(provenance=="CURRENT_TYPED_AND_EXPLICIT_FIXTURE" || provenance=="CURRENT_TYPED_ALL_REQUIRED","Parameter provenance required; no implicit defaults");
        const Vector3f ag=vector3(c,"attitude_p"), k=vector3(c,"rate_k");
        const Vector3f p=vector3(c,"rate_p"), i=vector3(c,"rate_i"), d=vector3(c,"rate_d"), ff=vector3(c,"rate_ff");
        rateLimit=vector3(c,"rate_limits_rad_s"); integralLimit=vector3(c,"integral_limits");
        for(int a=0;a<3;++a) require(ag(a)>0 && k(a)>0 && p(a)>=0 && i(a)>=0 && d(a)>=0 && rateLimit(a)>0 && integralLimit(a)>0,"Invalid gains/limits");
        double yw=scalar(c,"yaw_weight"); require(yw>=0&&yw<=1,"Yaw weight range");
        attitude.setProportionalGain(ag,static_cast<float>(yw)); attitude.setRateLimit(rateLimit);
        rate.setPidGains(k.emult(p),k.emult(i),k.emult(d)); rate.setFeedForwardGain(ff); rate.setIntegratorLimit(integralLimit); rate.resetIntegral();
        double cutoff=scalar(c,"yaw_torque_cutoff_hz"); require(cutoff>=0,"Yaw cutoff range"); yawFilter.setCutoffFreq(static_cast<float>(cutoff));
        batteryEnabled=boolean(c,"battery_scale_enabled");
        const mxArray *m=field(c,"effectiveness"); require(mxGetM(m)==6&&mxGetN(m)==6,"Effectiveness must be 6x6 in board channel order");
        const double *b=numbers(m,36); matrix::Matrix<float,6,16> B; B.setZero();
        for(int col=0;col<6;++col) for(int row=0;row<6;++row) B(row,col)=static_cast<float>(b[row+6*col]);
        Vector16f lo,hi,zero,slew,start; lo.zero(); zero.zero(); hi.setAll(1.f); slew.zero(); start.zero();
        const double *sl=numbers(field(c,"slew_limits_s"),6), *sp=numbers(field(c,"initial_motor_commands"),6);
        for(int a=0;a<6;++a){require(sl[a]>=0&&sp[a]>=0&&sp[a]<=1,"Slew/initial command range");slew(a)=static_cast<float>(sl[a]);start(a)=static_cast<float>(sp[a]);hasSlew=hasSlew||sl[a]>0;}
        allocator.setNormalizeRPY(true); allocator.setMetricAllocation(false);
        allocator.setActuatorMin(lo);allocator.setActuatorMax(hi);allocator.setSlewRateLimit(slew);
        allocator.setEffectivenessMatrix(B,zero,zero,6,true);allocator.setActuatorSetpoint(start);allocator.updateParameters();
        heldRate.zero(); for(int a=0;a<3;++a){satPositive(a)=false;satNegative(a)=false;}
    }
    mxArray *step(const mxArray *s) {
        const double time=scalar(s,"timestamp_sample_s"), firstdt=scalar(s,"initial_dt_s");
        require(!haveTime||time>lastTime,"Source timestamp did not progress");require(firstdt>0,"initial_dt_s must be positive");
        const double delta=haveTime?time-lastTime:firstdt;
        const float dt=math::constrain(static_cast<float>(delta),.000125f,.02f);
        const float allocdt=math::constrain(static_cast<float>(delta),.0002f,.02f);
        const bool armed=boolean(s,"armed"), rotary=boolean(s,"rotary_wing"), landed=boolean(s,"landed"), maybe=boolean(s,"maybe_landed");
        require(boolean(s,"rates_enabled"),"Kernel step requires rates_enabled; disable path is outside this diagnostic");
        const Quatf q=quaternion(s,"q_est"), qr=quaternion(s,"q_reference");
        const Vector3f rates=vector3(s,"body_rate"), accel=vector3(s,"angular_acceleration"), thrust=vector3(s,"thrust_body_normalized");
        const float yawff=static_cast<float>(scalar(s,"yaw_rate_ff"));
        const bool attitudeUpdated=boolean(s,"attitude_updated");
        const double batteryScale=scalar(s,"battery_scale");require(batteryScale>0,"battery_scale must be positive");
        require(haveAttitude||attitudeUpdated,"First step requires attitude update");
        // All input validation precedes state updates.
        if(attitudeUpdated){attitude.setAttitudeSetpoint(qr,yawff);heldRate=attitude.update(q);haveAttitude=true;}
        if(!armed||!rotary)rate.resetIntegral();
        rate.setSaturationStatus(satPositive,satNegative);
        Vector3f torque=rate.update(rates,heldRate,accel,dt,landed||maybe), applied=torque, appliedThrust=thrust;
        applied(2)=yawFilter.update(applied(2),dt);
        if(batteryEnabled) for(int a=0;a<3;++a){applied(a)=math::constrain(applied(a)*static_cast<float>(batteryScale),-1.f,1.f);appliedThrust(a)=math::constrain(thrust(a)*static_cast<float>(batteryScale),-1.f,1.f);}
        Vector6f request;for(int a=0;a<3;++a){request(a)=applied(a);request(a+3)=appliedThrust(a);}
        allocator.setControlSetpoint(request);allocator.allocate();if(hasSlew)allocator.applySlewRateLimit(allocdt);allocator.clipActuatorSetpoint();
        const Vector6f allocated=allocator.getAllocatedControl(), residual=request-allocated;
        Vector3f unallocated(residual(0),residual(1),residual(2));
        const bool achieved=unallocated.norm_squared()<1e-6f;
        // Hold allocator feedback for the next rate step; status publishes at most every 5 ms.
        if(statusTime<0 || time-statusTime>=.005) {
            for(int a=0;a<3;++a){satPositive(a)=!achieved&&unallocated(a)>FLT_EPSILON;satNegative(a)=!achieved&&unallocated(a)<-FLT_EPSILON;}statusTime=time;
        }
        rate_ctrl_status_s rs{};rate.getRateControlStatus(rs);
        Vector3f ints(rs.rollspeed_integ,rs.pitchspeed_integ,rs.yawspeed_integ);
        bool rateBound=false,intBound=false,saturated=false;
        const auto motors=allocator.getActuatorSetpoint();
        for(int a=0;a<3;++a){rateBound=rateBound||std::abs(heldRate(a))>=rateLimit(a)-1e-6f;intBound=intBound||std::abs(ints(a))>=integralLimit(a)-1e-6f;}
        for(int a=0;a<6;++a)saturated=saturated||motors(a)<=FLT_EPSILON||motors(a)>=1.f-FLT_EPSILON;
        const char *fields[]={"rate_setpoint","torque_raw","torque_applied","motor_commands","allocated_control","unallocated_control","integral","torque_achieved","motor_saturated","rate_limit_reached","integral_limit_reached","dt_s","dt_clamped","integral_updates_enabled","armed_input","parameter_provenance","claim"};
        mxArray *out=mxCreateStructMatrix(1,1,17,fields);
        put(out,"rate_setpoint",outputVector(heldRate));put(out,"torque_raw",outputVector(torque));put(out,"torque_applied",outputVector(applied));
        mxArray *u=mxCreateDoubleMatrix(6,1,mxREAL);for(int a=0;a<6;++a)mxGetPr(u)[a]=motors(a);put(out,"motor_commands",u);
        put(out,"allocated_control",outputVector(allocated));put(out,"unallocated_control",outputVector(residual));put(out,"integral",outputVector(ints));
        put(out,"torque_achieved",mxCreateLogicalScalar(achieved));put(out,"motor_saturated",mxCreateLogicalScalar(saturated));put(out,"rate_limit_reached",mxCreateLogicalScalar(rateBound));put(out,"integral_limit_reached",mxCreateLogicalScalar(intBound));
        put(out,"dt_s",mxCreateDoubleScalar(dt));put(out,"dt_clamped",mxCreateLogicalScalar(delta<.000125||delta>.02));put(out,"integral_updates_enabled",mxCreateLogicalScalar(!landed&&!maybe));
        put(out,"armed_input",mxCreateLogicalScalar(armed));put(out,"parameter_provenance",mxCreateString(provenance.c_str()));
        put(out,"claim",mxCreateString("PX4_ATTITUDE_RATE_SEQUENTIAL_ALLOCATION_KERNEL__CALLER_OWNED_ARM_STATE"));
        haveTime=true;lastTime=time;return out;
    }
};
static std::unique_ptr<Kernel> instance;
static bool exitRegistered=false;
static void cleanup(){instance.reset();}
void mexFunction(int nlhs,mxArray **plhs,int nrhs,const mxArray **prhs) {
    try {
        require(nrhs>=1,"Command required");const auto cmd=text(prhs[0]);
        if(cmd=="clear"){require(nrhs==1&&nlhs==0,"clear has no inputs/outputs");cleanup();return;}
        require(nrhs==2&&nlhs==1,"init/step require one struct and one output");
        if(cmd=="init") {
            std::unique_ptr<Kernel> candidate(new Kernel(prhs[1])); instance=std::move(candidate);
            if(!exitRegistered){mexAtExit(cleanup);exitRegistered=true;}
            plhs[0]=mxCreateLogicalScalar(true);return;
        }
        require(cmd=="step"&&instance!=nullptr,"Unknown command or kernel not initialized");plhs[0]=instance->step(prhs[1]);
    }catch(const std::exception &e){cleanup();mexErrMsgIdAndTxt("gpenmpc:NativeKernel", "%s",e.what());}
}
