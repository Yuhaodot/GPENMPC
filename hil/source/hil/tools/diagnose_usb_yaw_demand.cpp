// Offline allocation diagnostic for command continuity and the SE3 controller.
// Perfect attitude and rate tracking isolate feedforward allocation demand.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "../rfly_vendor_integration/CanonicalOperatorReference.hpp"
#include "gpenmpcNative_canonicalLocalInnerWithAuditFirst_initialize.h"
#include "gpenmpcUpdateDesiredAttitudeContinuity.h"
#include "se3WrenchKernel.h"
static e_gpenmpcNative_canonicalLocalIn scratch;
// Evaluate the yaw-reference update alongside the RC allocator.
static bool candidate_yaw(double previous,double previous_rate,double request,
                          double dt,double &next,double &next_rate){
    return gpenmpc_operator_reference::advance_yaw_reference(previous,previous_rate,request,dt,next,next_rate);
}
static void run(double accel_limit,double jitter_s=.01,bool priority_allocation=false) {
    o_struct_T state{}; k_struct_T cmd{};
    double yaw=0,rate=0,peak_acc=0,peak_torque=0,min_thrust=1e9;
    int saturation=0;
    for(int k=0;k<4000;++k){
        const double dt=k%100==50?jitter_s:.01;
        const double request=k<100?0:k<1200?1:k<2400?-1:0;
        double next=0,next_rate=0;
        if(accel_limit>0){
            assert(candidate_yaw(yaw,rate,request,dt,next,next_rate));
            assert(std::fabs(next_rate-rate)<=accel_limit*dt+1e-12);
            rate=next_rate;
        }else{rate=request;assert(gpenmpc_operator_reference::advance_yaw(yaw,rate,dt,next));}
        yaw=next;
        double rawR[9]={1,0,0,0,1,0,0,0,1};
        assert(gpenmpc_operator_reference::apply_yaw(yaw,rawR));
        o_struct_T candidate{};
        c_gpenmpcUpdateDesiredAttitudeCo(&scratch,state.initialized,state.angular_velocity_valid,
            state.filtered_rotation,state.c_desired_angular_velocity_body,
            state.c_desired_angular_acceleration_,state.update_count,state.reset_count,
            rawR,dt,false,&cmd,&candidate);
        state=candidate;
        // Set actual attitude and body rate equal to current desired values:
        // any excess allocation here exists even without tracking error.
        const double actual_yaw=std::atan2(cmd.desired_rotation[1],cmd.desired_rotation[0]);
        double x[19]{};x[6]=std::cos(actual_yaw*.5);x[9]=std::sin(actual_yaw*.5);
        std::memcpy(x+10,cmd.c_desired_angular_velocity_body,3*sizeof(double));
        double zero[3]{},wind[2]{},wrench[4]{},rotor[6]{},diag[51]{};
        assert(se3WrenchKernel(x,zero,zero,zero,2.21,wind,zero,cmd.desired_rotation,
            cmd.c_desired_angular_velocity_body,cmd.c_desired_angular_acceleration_,wrench,rotor,diag));
        peak_acc=std::max(peak_acc,std::fabs(cmd.c_desired_angular_acceleration_[2]));
        peak_torque=std::max(peak_torque,std::fabs(wrench[3]));
        if(priority_allocation){
            bool limited=false;
            assert(gpenmpc_operator_reference::allocate_manual(wrench,rotor,limited));
            diag[50]=limited?1:0;
            for(double r:rotor)assert(r>=0&&r<=32.145727009134916);
        }
        double sum=0;for(double r:rotor)sum+=r;
        if(priority_allocation)assert(std::fabs(sum-wrench[0])<1e-9);
        min_thrust=std::min(min_thrust,sum);
        if(diag[50]>.5)++saturation;
    }
    const double weight=11.71*9.80665,upper=32.145727009134916;
    std::printf("manual_accel_limit=%.3f jitter_s=%.3f priority_allocation=%d command_peak_accel=%.6f peak_yaw_torque=%.6f hover_yaw_headroom=%.6f saturated_ticks=%d min_total_thrust=%.6f weight=%.6f\n",
        accel_limit,jitter_s,priority_allocation,peak_acc,peak_torque,(upper-weight/6)/6.666666666666667,saturation,min_thrust,weight);
}
int main(int argc,char**){
    gpenmpcNative_canonicalLocalInnerWithAuditFirst_initialize();
    if(argc>1){
        run(.3,.084045,true); // Observed scheduling interval.
        run(.3,.4,true); // Command-lifetime boundary.
        return 0;
    }
    run(0); // Direct stick-rate integration.
    run(.3); // Nominal reference input.
    run(.3,.06); // Scheduling-interval variation.
    run(.3,.08); // RC jitter-envelope diagnostic.
    run(.3,.01,true);
    run(.3,.06,true);
    run(.3,.08,true);
}
