// Test operator reference shaping.
#include "../rfly_vendor_integration/CanonicalOperatorReference.hpp"
#include <cassert>
#include <cstring>
#include <cstdio>
#include <initializer_list>
#include <limits>
#include "../evidence/tracking_controller/generated/gpenmpcDesiredSe3Command.h"
int main(){
    // Retained real gaps plus explicit HOST large-dt numerical fixtures.
    // These are reference advances, not manufactured sensor/control commits.
    double probe_prior[12]{},probe_next[12]{},probe_velocity[3]{1,-1,.5};
    assert(gpenmpc_operator_reference::advance(probe_prior,probe_velocity,.084045,probe_next));
    for(const double dt: {.4,.401,.408236,1.,20.}){
        assert(gpenmpc_operator_reference::valid_control_interval_s(dt,true));
        assert(gpenmpc_operator_reference::advance(probe_prior,probe_velocity,dt,probe_next));
        for(double value:probe_next)assert(std::isfinite(value));
        double resumed[12]{};
        assert(gpenmpc_operator_reference::advance(probe_next,probe_velocity,.01,resumed));
        assert(resumed[0]>probe_next[0]);
        double split[12]{},step[12]{};const unsigned count=100;
        for(unsigned j=0;j<count;++j){
            assert(gpenmpc_operator_reference::advance(split,probe_velocity,dt/count,step));
            std::memcpy(split,step,sizeof split);
        }
        for(unsigned j=0;j<12;++j)assert(std::fabs(split[j]-probe_next[j])<1e-9);
    }
    assert(gpenmpc_operator_reference::valid_control_interval_s(.002,true));
    assert(gpenmpc_operator_reference::valid_control_interval_s(.05,false));
    assert(!gpenmpc_operator_reference::valid_control_interval_s(.050001,false));
    for(const double dt: {0.,-1.,.001999,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){
        assert(!gpenmpc_operator_reference::valid_control_interval_s(dt,true));
        assert(!gpenmpc_operator_reference::advance(probe_prior,probe_velocity,dt,probe_next));
    }
    const double allocation_cases[][4]={{114.835871,0,0,.9},
        {114.835871,0,0,5.5},{114.835871,0,0,-5.5},
        {114.835871,4,-3,5.5},{114.835871,-4,3,-5.5},{114.835871,100,100,10}};
    for(const auto &w: allocation_cases){
        double rotor[6]{};bool limited=false;
        assert(gpenmpc_operator_reference::allocate_manual(w,rotor,limited));
        double sum=0,yaw_moment=0;
        for(unsigned j=0;j<6;++j){
            assert(rotor[j]>=0&&rotor[j]<=32.145727009134916);
            sum+=rotor[j];yaw_moment+=(j%2?-1:1)*.025*rotor[j];
        }
        assert(std::fabs(sum-w[0])<1e-9);
        if(w[3]==.9){assert(!limited);assert(std::fabs(yaw_moment-.9)<1e-12);}
        else assert(limited);
    }
    double previous[12]{},next[12]{},velocity[3]{5,0,1.5};
    double t90=-1,max_acc=0,max_jerk=0;
    for(unsigned tick=0;tick<2000;++tick){
        assert(gpenmpc_operator_reference::advance(previous,velocity,.01,next));
        for(unsigned k=0;k<3;++k){
            assert(std::fabs(next[k+3])<=std::fabs(velocity[k])+1e-12);
            assert(std::fabs(next[k+6])<3.249);
            assert(std::fabs(next[k+9])<6.641);
        }
        if(t90<0&&next[3]>=.9*velocity[0])t90=(tick+1)*.01;
        max_acc=std::fmax(max_acc,std::fabs(next[6]));max_jerk=std::fmax(max_jerk,std::fabs(next[9]));
        std::memcpy(previous,next,sizeof next);
    }
    assert(previous[0]>93&&previous[1]==0&&previous[2]>28);
    assert(t90>=2.21&&t90<=2.22);
    std::printf("Reference step: t90=%.3f s, max_acc=%.6f m/s2, max_jerk=%.6f m/s3\n",t90,max_acc,max_jerk);
    double stop[3]{};
    for(unsigned tick=0;tick<2000;++tick){
        assert(gpenmpc_operator_reference::advance(previous,stop,.01,next));
        std::memcpy(previous,next,sizeof next);
    }
    assert(std::fabs(previous[3])+std::fabs(previous[4])+std::fabs(previous[5])<1e-10);
    double unsafe[3]{5.01,0,0};assert(!gpenmpc_operator_reference::advance(previous,unsafe,.01,next));
    assert(gpenmpc_operator_reference::advance(previous,stop,.401,next));
    // Check that one 20 ms zero-order-hold step equals two 10 ms steps.
    double a[12]{},b[12]{},c[12]{};
    assert(gpenmpc_operator_reference::advance(a,velocity,.02,b));
    assert(gpenmpc_operator_reference::advance(a,velocity,.01,c));
    assert(gpenmpc_operator_reference::advance(c,velocity,.01,a));
    for(unsigned k=0;k<12;++k)assert(std::fabs(a[k]-b[k])<1e-12);
    // Test left/right input, centered hold and +/- pi crossing.
    double yaw=0,next_yaw=0;
    for(unsigned j=0;j<1000;++j){assert(gpenmpc_operator_reference::advance_yaw(yaw,1,.01,next_yaw));yaw=next_yaw;}
    assert(std::fabs(yaw-(10-4*3.14159265358979323846))<1e-12);
    assert(gpenmpc_operator_reference::advance_yaw(yaw,0,.01,next_yaw)&&yaw==next_yaw);
    for(unsigned j=0;j<1000;++j){assert(gpenmpc_operator_reference::advance_yaw(yaw,-1,.01,next_yaw));yaw=next_yaw;}
    assert(std::fabs(yaw)<1e-12);
    assert(!gpenmpc_operator_reference::advance_yaw(yaw,1.00001,.01,next_yaw));
    for(const double dt: {.401,.408236,1.,20.})for(const double rate: {-1.,0.,1.}){
        constexpr double pi=3.14159265358979323846;
        assert(gpenmpc_operator_reference::advance_yaw(yaw,rate,dt,next_yaw));
        assert(std::isfinite(next_yaw)&&next_yaw>=-pi&&next_yaw<=pi);
        assert(std::fabs(std::sin(next_yaw)-std::sin(yaw+rate*dt))<1e-12);
        assert(std::fabs(std::cos(next_yaw)-std::cos(yaw+rate*dt))<1e-12);
        double resumed=0;assert(gpenmpc_operator_reference::advance_yaw(next_yaw,rate,.01,resumed));
    }
    // Bound yaw acceleration during reference shaping.
    // Candidate preparation preserves the caller's prior heading and rate.
    double shaped_yaw=0,shaped_rate=0,next_rate=0;
    for(unsigned j=0;j<1000;++j){
        const double requested=j<400?1:(j<800?-1:0);
        const double before_yaw=shaped_yaw,before_rate=shaped_rate;
        assert(gpenmpc_operator_reference::advance_yaw_reference(shaped_yaw,shaped_rate,requested,.01,next_yaw,next_rate));
        assert(shaped_yaw==before_yaw&&shaped_rate==before_rate);
        assert(std::fabs(next_rate-shaped_rate)<=.003+1e-12);
        assert(std::fabs(next_rate)<=1.0);
        if(j==399)assert(next_rate==1.0);
        shaped_yaw=next_yaw;shaped_rate=next_rate;
    }
    assert(!gpenmpc_operator_reference::advance_yaw_reference(shaped_yaw,shaped_rate,1.01,.01,next_yaw,next_rate));
    for(const double dt: {.401,.408236,1.,20.}){
        assert(gpenmpc_operator_reference::advance_yaw_reference(shaped_yaw,shaped_rate,1,dt,next_yaw,next_rate));
        assert(std::fabs(next_yaw)<=3.14159265358979323846&&std::fabs(next_rate)<=1.0);
        double resumed_yaw=0,resumed_rate=0;
        assert(gpenmpc_operator_reference::advance_yaw_reference(next_yaw,next_rate,-1,.01,resumed_yaw,resumed_rate));
        assert(std::fabs(resumed_rate-next_rate)<=.003+1e-12);
    }
    // Actual selected generated desired-force leaf; the yaw input changes
    // heading, not total force or thrust direction. Remaining command continuity,
    // robust attitude feedback and allocation are not replaced.
    double x[19]{},p[3]{},v[3]{},acc[3]{},wind[2]{},aug[3]{},raw[3],force[3],R[9],original[9];
    x[6]=1;
    for(double tilt: {0.0,.2}){
        acc[0]=tilt;acc[1]=-.5*tilt;
        gpenmpcDesiredSe3Command(x,p,v,acc,0,wind,aug,raw,force,R);
        std::memcpy(original,R,sizeof R);
        assert(gpenmpc_operator_reference::apply_yaw(0,R));
        for(unsigned j=0;j<9;++j)assert(std::fabs(R[j]-original[j])<1e-14);
        for(double heading: {-.5,.5,3.14,-3.14}){
            std::memcpy(R,original,sizeof R);
            assert(gpenmpc_operator_reference::apply_yaw(heading,R));
            for(unsigned j=6;j<9;++j)assert(R[j]==original[j]);
            for(unsigned j=0;j<3;++j)for(unsigned k=0;k<3;++k){
                double dot=0;for(unsigned t=0;t<3;++t)dot+=R[3*j+t]*R[3*k+t];
                assert(std::fabs(dot-(j==k?1.:0.))<1e-12);
            }
            if(tilt==0)assert(std::fabs(std::atan2(R[1],R[0])-heading)<1e-12);
        }
    }
    // Test diagonal, reversed and vertical demands through the generated force projection
    // with ideal position and velocity tracking.
    double combined[12]{},combined_next[12]{};
    for(unsigned tick=0;tick<1600;++tick){
        const double sign=(tick/400)%2?-1.0:1.0;
        double demand[3]{sign*5/std::sqrt(2.0),sign*5/std::sqrt(2.0),sign*1.5};
        const double dt=tick%100==50?.084045:.01;
        assert(gpenmpc_operator_reference::advance(combined,demand,dt,combined_next));
        for(unsigned j=0;j<3;++j){
            x[j]=p[j]=combined_next[j];x[j+3]=v[j]=combined_next[j+3];acc[j]=combined_next[j+6];
        }
        gpenmpcDesiredSe3Command(x,p,v,acc,2.21,wind,aug,raw,force,R);
        for(double f:force)assert(std::isfinite(f));
        assert(gpenmpc_operator_reference::apply_yaw(sign*.7,R));
        const double thrust=std::sqrt(force[0]*force[0]+force[1]*force[1]+force[2]*force[2]);
        double wrench[4]{thrust,sign*4,-sign*3,sign*5.5},rotor[6]{};bool limited=false;
        assert(gpenmpc_operator_reference::allocate_manual(wrench,rotor,limited));
        double sum=0;for(double r:rotor){assert(r>=0&&r<=32.145727009134916);sum+=r;}
        assert(thrust<6*32.145727009134916&&std::fabs(sum-thrust)<1e-9);
        std::memcpy(combined,combined_next,sizeof combined);
    }
    std::puts("Reference tests passed.");
}
