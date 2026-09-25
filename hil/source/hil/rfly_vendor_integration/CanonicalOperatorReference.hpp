#pragma once
#include <cmath>
namespace gpenmpc_operator_reference {
// World-frame velocity and yaw-rate reference shaping for manual SE(3) control.
constexpr double horizontal_speed_mps=5.0;
constexpr double vertical_speed_mps=1.5;
constexpr double yaw_rate_rad_s=1.0;
// Yaw acceleration design margin: at Jzz=3, 0.3 rad/s2 uses 0.9 Nm
// of the 1.951 Nm hover yaw headroom, reserving torque for tracking.
constexpr double yaw_acceleration_rad_s2=.3;
constexpr double velocity_pole_rad_s=2.4;
// Reference integration uses the measured interval; autonomous updates also
// enforce the configured upper bound.
inline bool valid_control_interval_s(double dt,bool operator_reference=true) noexcept {
    return std::isfinite(dt)&&dt>=.002&&(operator_reference||dt<=.0500001);
}
// Manual allocation preserves collective, then roll/pitch and feasible yaw torque.
inline bool allocate_manual(const double wrench[4],double rotor[6],bool &limited) noexcept {
    constexpr double upper=32.145727009134916,arm=.5665,yaw_arm=.025;
    constexpr double sine[6]={0,.86602540378443864676,.86602540378443864676,0,-.86602540378443864676,-.86602540378443864676};
    constexpr double cosine[6]={1,.5,-.5,-1,-.5,.5};
    if(!wrench||!rotor)return false;
    for(unsigned j=0;j<4;++j)if(!std::isfinite(wrench[j]))return false;
    const double collective=::fmax(0.0,::fmin(6*upper,wrench[0]));
    const double base=collective/6;
    double tilt[6]{},tilt_scale=1,yaw_scale=1;
    for(unsigned j=0;j<6;++j){
        tilt[j]=(sine[j]*wrench[1]-cosine[j]*wrench[2])/(3*arm);
        if(!std::isfinite(tilt[j]))return false;
        if(tilt[j]>0)tilt_scale=::fmin(tilt_scale,(upper-base)/tilt[j]);
        if(tilt[j]<0)tilt_scale=::fmin(tilt_scale,-base/tilt[j]);
    }
    for(unsigned j=0;j<6;++j){
        rotor[j]=base+tilt_scale*tilt[j];
        const double yaw=(j%2?-1.0:1.0)*wrench[3]/(6*yaw_arm);
        if(!std::isfinite(yaw))return false;
        if(yaw>0)yaw_scale=::fmin(yaw_scale,(upper-rotor[j])/yaw);
        if(yaw<0)yaw_scale=::fmin(yaw_scale,-rotor[j]/yaw);
    }
    yaw_scale=::fmax(0.0,yaw_scale);
    for(unsigned j=0;j<6;++j){
        rotor[j]+=yaw_scale*(j%2?-1.0:1.0)*wrench[3]/(6*yaw_arm);
        rotor[j]=::fmax(0.0,::fmin(upper,rotor[j]));
    }
    limited=collective<wrench[0]||collective>wrench[0]||tilt_scale<1-1e-12||yaw_scale<1-1e-12;
    return true;
}
// Fourth channel: at most 1 rad/s desired yaw rate (57.30 deg/s).
// Centred stick holds the integrated heading; the SO(3)
// continuity layer supplies rate/acceleration filtering and feedforward.
inline bool advance_yaw(double previous,double rate,double dt,double &next) noexcept {
    if(!std::isfinite(previous)||!std::isfinite(rate)||std::fabs(rate)>yaw_rate_rad_s||
       !valid_control_interval_s(dt))return false;
    constexpr double pi=3.14159265358979323846;
    if(std::fabs(previous)>pi)return false;
    next=previous+rate*dt;
    if(!std::isfinite(next))return false;
    // Wrap intervals spanning multiple turns before the single-turn adjustment.
    if(next>3*pi||next< -3*pi)next=::fmod(next,2*pi);
    if(next>pi)next-=2*pi;
    if(next< -pi)next+=2*pi;
    return std::isfinite(next);
}
inline bool advance_yaw_reference(double previous,double previous_rate,
        double requested_rate,double dt,double &next,double &next_rate) noexcept {
    if(!std::isfinite(previous_rate)||std::fabs(previous_rate)>yaw_rate_rad_s||
       !std::isfinite(requested_rate)||std::fabs(requested_rate)>yaw_rate_rad_s||
       !valid_control_interval_s(dt))return false;
    const double delta=requested_rate-previous_rate;
    const double bound=yaw_acceleration_rad_s2*dt;
    next_rate=previous_rate+(delta>bound?bound:(delta< -bound?-bound:delta));
    // Integrate the bounded-rate transition using source dt; install after commit.
    return advance_yaw(previous,.5*(previous_rate+next_rate),dt,next);
}
// Reconstruct the heading axis while retaining desired thrust direction/norm.
// Column-major SO(3), with the cross-product construction from
// gpenmpcDesiredSe3Command with b1=[cos(yaw),sin(yaw),0] instead of [1,0,0].
inline bool apply_yaw(double yaw,double rotation[9]) noexcept {
    if(!std::isfinite(yaw)||!rotation)return false;
    const double c=std::cos(yaw),s=std::sin(yaw);
    double b2[3]={-rotation[8]*s,rotation[8]*c,rotation[6]*s-rotation[7]*c};
    const double norm=std::sqrt(b2[0]*b2[0]+b2[1]*b2[1]+b2[2]*b2[2]);
    if(!std::isfinite(norm)||norm<1e-9)return false;
    for(unsigned j=0;j<3;++j)rotation[3+j]=b2[j]/norm;
    rotation[0]=rotation[4]*rotation[8]-rotation[7]*rotation[5];
    rotation[1]=rotation[6]*rotation[5]-rotation[3]*rotation[8];
    rotation[2]=rotation[3]*rotation[7]-rotation[6]*rotation[4];
    return true;
}
inline bool advance(const double previous[12],const double velocity[3],
                    double dt,double next[12]) noexcept {
    if(!previous||!velocity||!next||!valid_control_interval_s(dt))return false;
    for(unsigned k=0;k<12;++k)if(!std::isfinite(previous[k]))return false;
    for(unsigned k=0;k<3;++k)if(!std::isfinite(velocity[k])||std::fabs(velocity[k])>(k==2?vertical_speed_mps:horizontal_speed_mps))return false;
    const double w=velocity_pole_rad_s,decay=std::exp(-w*dt);
    // dt>=2 ms bounds cancellation; use the existing NuttX exp export.
    const double i0=(1.0-decay)/w;
    const double i1=(i0-dt*decay)/w,i2=(2*i1-dt*dt*decay)/w;
    for(unsigned k=0;k<3;++k){
        const double alpha=previous[k+3]-velocity[k];
        const double beta=previous[k+6]+w*alpha;
        const double gamma=.5*(previous[k+9]+2*w*previous[k+6]+w*w*alpha);
        const double polynomial=alpha+beta*dt+gamma*dt*dt;
        const double first=beta+2*gamma*dt;
        next[k]=previous[k]+velocity[k]*dt+alpha*i0+beta*i1+gamma*i2;
        next[k+3]=velocity[k]+polynomial*decay;
        next[k+6]=(first-w*polynomial)*decay;
        next[k+9]=(2*gamma-2*w*first+w*w*polynomial)*decay;
    }
    for(unsigned k=0;k<12;++k)if(!std::isfinite(next[k]))return false;
    return true;
}
}
