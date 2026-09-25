// Host numerical gateway to the bound PX4 PositionControl translation units.
#include "mex.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>
#include <modules/mc_pos_control/PositionControl/PositionControl.hpp>

using matrix::Vector3f;
static void require(bool ok,const char *why) { if(!ok) throw std::runtime_error(why); }
static const mxArray *field(const mxArray *s,const char *key) {
    require(mxIsStruct(s)&&mxGetNumberOfElements(s)==1,"Expected scalar struct");
    const mxArray *v=mxGetField(s,0,key);require(v!=nullptr,key);return v;
}
template<size_t N> static void exactFields(const mxArray *s,const char *const (&keys)[N]) {
    require(mxIsStruct(s)&&mxGetNumberOfElements(s)==1,"Expected scalar struct");
    require(mxGetNumberOfFields(s)==static_cast<int>(N),"Missing or unknown struct field");
    for(size_t n=0;n<N;++n)field(s,keys[n]);
}
static const double *numeric(const mxArray *a,size_t n,bool allowNaN=false) {
    require(mxIsDouble(a)&&!mxIsComplex(a)&&!mxIsSparse(a)&&mxGetM(a)==n&&mxGetN(a)==1,"Expected real double scalar or exact column vector");
    const double *v=mxGetPr(a);
    for(size_t j=0;j<n;++j) {
        if(allowNaN&&std::isnan(v[j]))continue;
        require(std::isfinite(v[j])&&std::isfinite(static_cast<float>(v[j])),"Inf, invalid NaN or float-overflow input");
    }
    return v;
}
static double scalar(const mxArray *s,const char *key,bool allowNaN=false) { return numeric(field(s,key),1,allowNaN)[0]; }
static Vector3f vector3(const mxArray *s,const char *key,bool allowNaN=false) {
    const double *v=numeric(field(s,key),3,allowNaN);
    return Vector3f(static_cast<float>(v[0]),static_cast<float>(v[1]),static_cast<float>(v[2]));
}
static bool boolean(const mxArray *s,const char *key) {
    const mxArray *v=field(s,key);require(mxIsLogicalScalar(v),"Expected scalar logical");return mxIsLogicalScalarTrue(v);
}
static std::string text(const mxArray *a) {
    require(mxIsChar(a)&&mxGetM(a)==1,"Expected char row");
    char *v=mxArrayToString(a);require(v!=nullptr,"Text conversion failed");std::string out(v);mxFree(v);return out;
}
static void put(mxArray *s,const char *key,mxArray *a) { mxSetField(s,0,key,a); }
static mxArray *vectorOut(const float *v,size_t n,bool allowNaN) {
    mxArray *a=mxCreateDoubleMatrix(n,1,mxREAL);
    for(size_t j=0;j<n;++j) {
        require(std::isfinite(v[j])||(allowNaN&&std::isnan(v[j])),"Nonfinite numerical kernel output");
        mxGetPr(a)[j]=v[j];
    }
    return a;
}
static mxArray *nanVector(size_t n) {
    mxArray *a=mxCreateDoubleMatrix(n,1,mxREAL);std::fill(mxGetPr(a),mxGetPr(a)+n,mxGetNaN());return a;
}
static void scalarOut(mxArray *s,const char *key,float value,bool allowNaN=true) {
    require(std::isfinite(value)||(allowNaN&&std::isnan(value)),"Invalid diagnostic output");
    put(s,key,mxCreateDoubleScalar(value));
}

struct Configuration {
    Vector3f positionP,velocityP,velocityI,velocityD,velocityLimits;
    float thrustMin,thrustMax,thrustMargin,tilt,hover;
    bool decouple;
    std::string provenance;
    explicit Configuration(const mxArray *c) {
        const char *const keys[]={"parameter_provenance","position_p","velocity_p","velocity_i","velocity_d",
            "velocity_limits_mps","thrust_limits","horizontal_thrust_margin","tilt_limit_rad","hover_thrust","decouple_horizontal_vertical"};
        exactFields(c,keys);
        provenance=text(field(c,"parameter_provenance"));
        require(provenance=="CURRENT_TYPED_WITH_EXPLICIT_MODULE_PHASE"||provenance=="CURRENT_TYPED_ALL_REQUIRED"||provenance=="EXPLICIT_HOST_FIXTURE","Unknown parameter provenance.");
        positionP=vector3(c,"position_p");velocityP=vector3(c,"velocity_p");velocityI=vector3(c,"velocity_i");velocityD=vector3(c,"velocity_d");
        velocityLimits=vector3(c,"velocity_limits_mps");
        const double *tl=numeric(field(c,"thrust_limits"),2);
        thrustMin=static_cast<float>(tl[0]);thrustMax=static_cast<float>(tl[1]);
        thrustMargin=static_cast<float>(scalar(c,"horizontal_thrust_margin"));
        tilt=static_cast<float>(scalar(c,"tilt_limit_rad"));hover=static_cast<float>(scalar(c,"hover_thrust"));
        decouple=boolean(c,"decouple_horizontal_vertical");
        for(int j=0;j<3;++j)require(positionP(j)>=0&&velocityP(j)>=0&&velocityI(j)>=0&&velocityD(j)>=0&&velocityLimits(j)>=0,"Negative gain or velocity limit");
        // Native horizontal anti-reset-windup divides by the X P gain.
        require(velocityP(0)>0,"Velocity X P must be positive for native ARW division");
        require(thrustMin>=0&&thrustMin<=thrustMax&&thrustMax>=.001f&&thrustMax<=1,"Normalized thrust limits outside native domain");
        require(thrustMargin>=0&&thrustMargin<=thrustMax,"Horizontal thrust margin outside collective domain");
        require(tilt>=0&&tilt<static_cast<float>(1.57079632679489661923),"Tilt must be below pi/2 for thrust projection");
        require(hover>=.05f&&hover<=.9f,"Hover thrust is outside the native declared range.");
    }
};

static mxArray *verticalDiagnostics(const PositionControl::VerticalControlDiagnostics &d,bool valid) {
    const char *keys[]={"position_setpoint","position","position_term","velocity_feedforward","velocity_setpoint","velocity",
        "velocity_error","acceleration_feedforward","acceleration_p","acceleration_i","acceleration_d",
        "acceleration_setpoint","thrust_setpoint","hover_thrust","integral_gain","anti_windup",
        "integral_updated","terminal_negative_integral_update_blocked","current_update_valid"};
    mxArray *out=mxCreateStructMatrix(1,1,19,keys);
    const float values[]={d.position_setpoint,d.position,d.position_term,d.velocity_feedforward,d.velocity_setpoint,d.velocity,
        d.velocity_error,d.acceleration_feedforward,d.acceleration_p,d.acceleration_i,d.acceleration_d,
        d.acceleration_setpoint,d.thrust_setpoint,d.hover_thrust,d.integral_gain};
    for(int j=0;j<15;++j)scalarOut(out,keys[j],valid?values[j]:static_cast<float>(mxGetNaN()));
    put(out,"anti_windup",mxCreateLogicalScalar(valid&&d.anti_windup));
    put(out,"integral_updated",mxCreateLogicalScalar(valid&&d.integral_updated));
    put(out,"terminal_negative_integral_update_blocked",mxCreateLogicalScalar(valid&&d.terminal_negative_integral_update_blocked));
    put(out,"current_update_valid",mxCreateLogicalScalar(valid));return out;
}

struct Kernel {
    Configuration cfg;
    PositionControl control;
    bool haveTime{false};
    double lastTime{0};
    explicit Kernel(const Configuration &c):cfg(c) {
        control.setPositionGains(cfg.positionP);control.setVelocityGains(cfg.velocityP,cfg.velocityI,cfg.velocityD);
        control.setVelocityLimits(cfg.velocityLimits(0),cfg.velocityLimits(1),cfg.velocityLimits(2));
        control.setThrustLimits(cfg.thrustMin,cfg.thrustMax);control.setHorizontalThrustMargin(cfg.thrustMargin);
        control.setTiltLimit(cfg.tilt);control.setHoverThrust(cfg.hover);
        control.decoupleHorizontalAndVecticalAcceleration(cfg.decouple);
        control.setTerminalNegativeIntegralUpdateInhibit(false);control.resetIntegral();
    }
    mxArray *step(const mxArray *s,bool &valid) {
        const char *const keys[]={"timestamp_sample_s","dt_s","state_position_ned","state_velocity_ned",
            "state_acceleration_ned","state_yaw","trajectory_position_ned","trajectory_velocity_ned",
            "trajectory_acceleration_ned","trajectory_yaw","trajectory_yawspeed","vertical_i_gain",
            "terminal_negative_integral_update_inhibit"};
        exactFields(s,keys);
        const double timestamp=scalar(s,"timestamp_sample_s"),dtInput=scalar(s,"dt_s");
        const float dt=static_cast<float>(dtInput);
        require(dt>0,"dt_s must remain positive after float conversion");
        require(!haveTime||timestamp>lastTime,"Source time did not strictly progress");
        const double elapsed=haveTime?timestamp-lastTime:dtInput;
        const double tolerance=std::max(1e-9,1e-6*std::max(std::abs(elapsed),std::abs(dtInput)));
        require(std::abs(elapsed-dtInput)<=tolerance,"Explicit dt and source timestamp delta disagree");
        PositionControlStates state{};
        state.position=vector3(s,"state_position_ned");state.velocity=vector3(s,"state_velocity_ned");
        state.acceleration=vector3(s,"state_acceleration_ned");state.yaw=static_cast<float>(scalar(s,"state_yaw"));
        const Vector3f pos=vector3(s,"trajectory_position_ned",true),vel=vector3(s,"trajectory_velocity_ned",true),acc=vector3(s,"trajectory_acceleration_ned",true);
        const float yaw=static_cast<float>(scalar(s,"trajectory_yaw",true)),yawSpeed=static_cast<float>(scalar(s,"trajectory_yawspeed",true));
        const float effectiveI=static_cast<float>(scalar(s,"vertical_i_gain"));
        require(effectiveI>=0,"Effective vertical I gain must be nonnegative");
        const bool inhibit=boolean(s,"terminal_negative_integral_update_inhibit");
        trajectory_setpoint_s trajectory=PositionControl::empty_trajectory_setpoint;
        pos.copyTo(trajectory.position);vel.copyTo(trajectory.velocity);acc.copyTo(trajectory.acceleration);
        trajectory.yaw=yaw;trajectory.yawspeed=yawSpeed;
        // Validation ends here. NaN means native missing-setpoint semantics;
        // source/state/config Inf or overflow was rejected before mutation.
        const float beforeI=control.getVelocityIntegralZ();
        control.setVelocityIntegralGainZ(effectiveI);control.setTerminalNegativeIntegralUpdateInhibit(inhibit);
        control.setState(state);control.setInputSetpoint(trajectory);valid=control.update(dt);
        const float afterI=control.getVelocityIntegralZ();require(std::isfinite(afterI),"Nonfinite integral after update");
        const char *fields[]={"update_valid","outputs_usable_for_host_numerical_chain","authority_granted","instance_cleared",
            "q_d","thrust_body","position_setpoint_ned","velocity_setpoint_ned","acceleration_setpoint_ned",
            "thrust_setpoint_ned","yaw_setpoint","yawspeed_setpoint","vertical_diagnostics",
            "vertical_integral_z_before","vertical_integral_z_after","effective_vertical_i_gain",
            "terminal_negative_integral_update_inhibit","timestamp_sample_s","source_delta_s","dt_s",
            "dt_clamped","parameter_provenance","claim","failure","effective_minimum_thrust"};
        mxArray *out=mxCreateStructMatrix(1,1,25,fields);
        put(out,"update_valid",mxCreateLogicalScalar(valid));put(out,"outputs_usable_for_host_numerical_chain",mxCreateLogicalScalar(valid));
        put(out,"authority_granted",mxCreateLogicalScalar(false));put(out,"instance_cleared",mxCreateLogicalScalar(!valid));
        if(valid) {
            vehicle_local_position_setpoint_s local{};vehicle_attitude_setpoint_s attitude{};
            control.getLocalPositionSetpoint(local);control.getAttitudeSetpoint(attitude);
            const float outPos[]={local.x,local.y,local.z},outVel[]={local.vx,local.vy,local.vz};
            put(out,"q_d",vectorOut(attitude.q_d,4,false));put(out,"thrust_body",vectorOut(attitude.thrust_body,3,false));
            put(out,"position_setpoint_ned",vectorOut(outPos,3,true));put(out,"velocity_setpoint_ned",vectorOut(outVel,3,true));
            put(out,"acceleration_setpoint_ned",vectorOut(local.acceleration,3,false));put(out,"thrust_setpoint_ned",vectorOut(local.thrust,3,false));
            scalarOut(out,"yaw_setpoint",local.yaw,false);scalarOut(out,"yawspeed_setpoint",local.yawspeed,false);
        } else {
            // Never expose a prior successful attitude/thrust as a command
            // after a failed input-validity check. The gateway clears state.
            put(out,"q_d",nanVector(4));put(out,"thrust_body",nanVector(3));
            for(const char *key:{"position_setpoint_ned","velocity_setpoint_ned","acceleration_setpoint_ned","thrust_setpoint_ned"})put(out,key,nanVector(3));
            put(out,"yaw_setpoint",mxCreateDoubleScalar(mxGetNaN()));put(out,"yawspeed_setpoint",mxCreateDoubleScalar(mxGetNaN()));
        }
        put(out,"vertical_diagnostics",verticalDiagnostics(control.getVerticalControlDiagnostics(),valid));
        scalarOut(out,"vertical_integral_z_before",beforeI,false);scalarOut(out,"vertical_integral_z_after",afterI,false);
        scalarOut(out,"effective_vertical_i_gain",effectiveI,false);
        put(out,"terminal_negative_integral_update_inhibit",mxCreateLogicalScalar(inhibit));
        put(out,"timestamp_sample_s",mxCreateDoubleScalar(timestamp));put(out,"source_delta_s",mxCreateDoubleScalar(elapsed));
        scalarOut(out,"dt_s",dt,false);put(out,"dt_clamped",mxCreateLogicalScalar(false));
        put(out,"parameter_provenance",mxCreateString(cfg.provenance.c_str()));
        put(out,"claim",mxCreateString("GPENMPC_PX4_POSITIONCONTROL_CONTROLMATH_NUMERICAL_KERNEL"));
        put(out,"failure",mxCreateString(valid?"":"NATIVE_POSITIONCONTROL_UPDATE_INVALID__INSTANCE_CLEARED__NO_OUTPUT_AUTHORITY"));
        scalarOut(out,"effective_minimum_thrust",std::max(cfg.thrustMin,.001f),false);
        if(valid){haveTime=true;lastTime=timestamp;}
        return out;
    }
};
static std::unique_ptr<Kernel> instance;
static bool exitRegistered=false;
static void cleanup() { instance.reset(); }
void mexFunction(int nlhs,mxArray **plhs,int nrhs,const mxArray **prhs) {
    try {
        require(nrhs>=1,"Command required");const std::string command=text(prhs[0]);
        if(command=="clear") { require(nrhs==1&&nlhs==0,"clear accepts no struct and no output");cleanup();return; }
        if(command=="reset") {
            require(nrhs==1&&nlhs==1&&instance!=nullptr,"reset requires initialized kernel and one output");
            Configuration retained=instance->cfg;instance.reset(new Kernel(retained));plhs[0]=mxCreateLogicalScalar(true);return;
        }
        require(nrhs==2&&nlhs==1,"init/step require one struct and one output");
        if(command=="init") {
            Configuration c(prhs[1]);std::unique_ptr<Kernel> next(new Kernel(c));instance=std::move(next);
            if(!exitRegistered){mexAtExit(cleanup);exitRegistered=true;}
            plhs[0]=mxCreateLogicalScalar(true);return;
        }
        require(command=="step"&&instance!=nullptr,"Unknown command or kernel not initialized");
        bool valid=false;plhs[0]=instance->step(prhs[1],valid);if(!valid)cleanup();
    } catch(const std::exception &e) { cleanup();mexErrMsgIdAndTxt("gpenmpc:NativePositionKernel","%s",e.what()); }
}
