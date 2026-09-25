#include "NativeLandOutputAuthority.hpp"
#include "CanonicalOutputAuthority.hpp"
#include "../px4_stream/SharedOutputRegistry.hpp"
#include <cstdio>
#include <limits>

using namespace gpenmpc_rfly_px4;
using gpenmpc_rfly_stream::Source;
static unsigned checks=0,failed=0;
static void check(bool ok,const char *name){++checks;if(!ok){++failed;std::fprintf(stderr,"FAIL %s\n",name);}}
struct TestClock{std::uint64_t t{1000};std::uint64_t now()noexcept{return t;}};
struct TestGuard{
    TestClock &clock;BoardSafetyEvidence value{};bool available{true};
    bool observe(BoardSafetyEvidence &out)noexcept{out=value;out.observation_us=clock.now();return available;}
};
using Land=NativeLandOutputAuthority<TestGuard,TestClock>;
struct Fixture{
    gpenmpc_consumption::Identity identity{0x1122334455667788ULL,42,1,1};
    TestClock clock{};TestGuard guard{clock};Land land{guard,clock,identity,500,600};
    gpenmpc_rfly_stream::Observation sample{};
    Fixture(){
        auto &e=guard.value;e.identity=identity;e.identity_valid_until_us=1400;
        e.original_valid_until_us=850; // expired old OFFBOARD, deliberately NOT renewed
        e.status_timestamp_us=e.mode_timestamp_us=e.power_timestamp_us=900;e.offboard_timestamp_us=350;
        e.uid=e.mavlink_identity=e.boot_session=e.usb_transport=e.hil_configuration=
        e.pwm_functions_zero=e.pwm_out_stopped=e.io_driver_stopped=e.dshot_stopped=
        e.usb_power_observed=e.external_physical_isolation=GuardFact::Pass;
        e.telemetry_fresh=e.active_direct_mode=e.native_controllers_disabled=GuardFact::Fail;
        e.native_land_mode=GuardFact::Pass;e.disarmed_control=GuardFact::Fail;
        auto &s=sample.status;s.timestamp=900;s.system_id=1;s.component_id=1;
        s.vehicle_type=vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
        s.hil_state=vehicle_status_s::HIL_STATE_ON;s.arming_state=vehicle_status_s::ARMING_STATE_ARMED;
        s.nav_state=vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
        auto &m=sample.control_mode;m.timestamp=900;m.flag_armed=true;
        m.flag_multicopter_position_control_enabled=m.flag_control_auto_enabled=
        m.flag_control_position_enabled=m.flag_control_velocity_enabled=m.flag_control_altitude_enabled=
        m.flag_control_climb_rate_enabled=m.flag_control_attitude_enabled=m.flag_control_rates_enabled=
        m.flag_control_allocation_enabled=true;
        sample.source=Source::NativeOutputsSim;sample.output.timestamp=1000;sample.output.noutputs=6;
        for(unsigned i=0;i<6;++i)sample.output.output[i]=0.1f*float(i+1);
        sample.output_uorb_generation=10;
    }
    bool begin(){return land.begin_native_land(850,1500,sample.status,sample.control_mode);}
    bool consume(){gpenmpc_rfly_stream::OriginalValidity end{};
        return land.accept(sample,clock.now(),end)&&end.valid_until_us==1400;}
    NativeLandDiagnostics diagnostics(){NativeLandDiagnostics d{};check(land.diagnostic_snapshot(d),"diagnostic lock");return d;}
};

int main(){
    {Fixture f;
        check(f.land.begin_observed_safety_state(1500,f.sample.status,f.sample.control_mode),"actual LAND observation without inventing request ACK");
        auto d=f.diagnostics();check(d.activation==NativeLandActivation::ObservedLand&&d.request_us==900,"observed event provenance is original status/mode");
        check(f.consume(),"observed native ownership accepts actual sample");
        check(!f.land.begin_observed_safety_state(1800,f.sample.status,f.sample.control_mode),"observation activation cannot renew deadline");}
    {Fixture f;f.sample.status.arming_state=vehicle_status_s::ARMING_STATE_DISARMED;
        f.sample.control_mode.flag_armed=false;f.guard.value.native_land_mode=GuardFact::Fail;
        f.guard.value.disarmed_control=GuardFact::Pass;for(auto &x:f.sample.output.output)x=-0.f;
        check(f.land.begin_observed_safety_state(1400,f.sample.status,f.sample.control_mode),"explicit initially disarmed zero-only source");
        const auto d=f.diagnostics();check(d.activation==NativeLandActivation::ObservedDisarmedZero&&d.observed_disarmed,"zero-only activation latches actual disarm");
        check(f.consume(),"negative zero remains exact safe zero");
        f.sample.status.arming_state=vehicle_status_s::ARMING_STATE_ARMED;f.sample.control_mode.flag_armed=true;
        f.guard.value.native_land_mode=GuardFact::Pass;f.guard.value.disarmed_control=GuardFact::Fail;
        check(f.land.choose(1000,f.sample.status,f.sample.control_mode)==Source::Unavailable,"zero-only source cannot become LAND actuator authority");}
    {Fixture f;f.sample.status.nav_state=vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
        check(!f.land.begin_observed_safety_state(1500,f.sample.status,f.sample.control_mode),"observed non-native mode cannot activate safety output");}
    {Fixture f;
        check(f.land.choose(1000,f.sample.status,f.sample.control_mode)==Source::Unavailable,"unselected never falls back");
        check(f.begin(),"explicit original LAND plus actual native ownership");
        check(f.land.choose(1000,f.sample.status,f.sample.control_mode)==Source::NativeOutputsSim,"native selected only explicitly");
        check(f.consume(),"actual output original expiry accepted with stale unused OFFBOARD visible");
        check(f.guard.value.original_valid_until_us==850,"no rewriting direct original expiry");
        check(!f.consume(),"same generation cannot replay");
        check(f.land.choose(1000,f.sample.status,f.sample.control_mode)==Source::Unavailable,"fault is sticky");}
    {Fixture f;check(f.begin()&&f.consume(),"disarm sequence initial");
        f.clock.t=1010;f.sample.output.timestamp=1010;f.sample.output_uorb_generation=12;
        f.sample.status.arming_state=vehicle_status_s::ARMING_STATE_DISARMED;
        f.sample.control_mode.flag_armed=false;
        f.guard.value.native_land_mode=GuardFact::Fail;f.guard.value.disarmed_control=GuardFact::Pass;
        for(auto &x:f.sample.output.output)x=0.f;
        check(f.consume(),"fresh observed disarmed permits only actual zero output");
        check(f.diagnostics().disarmed_zero_accepted==1,"zero send acceptance separately counted, not plant ACK");
        f.sample.status.arming_state=vehicle_status_s::ARMING_STATE_ARMED;f.sample.control_mode.flag_armed=true;
        f.guard.value.native_land_mode=GuardFact::Pass;f.guard.value.disarmed_control=GuardFact::Fail;
        check(f.land.choose(1010,f.sample.status,f.sample.control_mode)==Source::Unavailable,"same tail cannot rearm after observed disarmed");}
    {Fixture f;check(f.begin()&&f.consume(),"generation skip initial");
        f.clock.t=1010;f.sample.output.timestamp=1010;f.sample.output_uorb_generation=17;
        check(f.consume(),"actual uORB positive skipped generation retained");}
    {Fixture f;f.sample.output_uorb_generation=~0u;check(f.begin()&&f.consume(),"generation wrap initial");
        f.clock.t=1010;f.sample.output.timestamp=1010;f.sample.output_uorb_generation=0;
        check(f.consume(),"uint32 generation wrap accepted");}
    for(unsigned n=0;n<20;++n){Fixture f;check(f.begin(),"negative initial");
        switch(n){
        case 0:f.guard.available=false;break;
        case 1:++f.guard.value.identity.boot_generation;break;
        case 2:f.guard.value.external_physical_isolation=GuardFact::Unknown;break;
        case 3:f.guard.value.pwm_functions_zero=GuardFact::Fail;break;
        case 4:f.guard.value.pwm_out_stopped=GuardFact::Fail;break;
        case 5:f.guard.value.identity_valid_until_us=999;break;
        case 6:f.guard.value.power_timestamp_us=499;break;
        case 7:f.sample.status.nav_state=vehicle_status_s::NAVIGATION_STATE_OFFBOARD;break;
        case 8:f.sample.control_mode.flag_control_offboard_enabled=true;break;
        case 9:f.sample.control_mode.flag_multicopter_position_control_enabled=false;break;
        case 10:f.sample.source=Source::RflyOutputs;break;
        case 11:f.sample.output.timestamp=0;break;
        case 12:f.sample.output.timestamp=1001;break;
        case 13:f.sample.output.noutputs=0;break;
        case 14:f.sample.output.output[0]=std::numeric_limits<float>::quiet_NaN();break;
        case 15:f.sample.output.output[0]=1.001f;break;
        case 16:f.sample.output.output[6]=0.1f;break;
        case 17:f.sample.status.arming_state=vehicle_status_s::ARMING_STATE_DISARMED;
            f.sample.control_mode.flag_armed=false;f.guard.value.disarmed_control=GuardFact::Pass;break;
        case 18:f.guard.value.native_land_mode=GuardFact::Fail;break;
        case 19:f.sample.status.arming_state=vehicle_status_s::ARMING_STATE_DISARMED;
            f.sample.control_mode.flag_armed=false;for(auto &x:f.sample.output.output)x=0.f;break;
        }
        gpenmpc_rfly_stream::OriginalValidity end{};
        check(!f.land.accept(f.sample,1000,end)&&end.valid_until_us==0,"mode/identity/time/output invalid rejects");
        check(f.diagnostics().first_fault!=NativeLandFault::None,"failure preserved");
        check(!f.begin(),"bad tail cannot self restart");
    }
    for(unsigned n=0;n<4;++n){Fixture f;const auto request=n==0?0:(n==1?1001:850);
        const auto deadline=n==2?849:(n==3?999:1500);
        check(!f.land.begin_native_land(request,deadline,f.sample.status,f.sample.control_mode),"bad original request/deadline rejected");}
    {Fixture f;check(f.begin(),"cached pre-selection output initial");
        f.sample.output.timestamp=849;
        gpenmpc_rfly_stream::OriginalValidity end{};
        check(!f.land.accept(f.sample,1000,end)&&end.valid_until_us==0,"cached pre-LAND output not transmitted");
        auto d=f.diagnostics();
        check(d.first_fault==NativeLandFault::None&&d.selected&&d.accepted==0&&
            d.request_us==850&&d.original_deadline_us==1500,"discard does not poison or renew original native handoff");
        f.sample.output.timestamp=1000;f.sample.output_uorb_generation=11;
        check(f.consume(),"fresh native output can follow discarded cached output");
        f.sample.output.timestamp=849;f.sample.output_uorb_generation=12;
        check(!f.consume()&&f.diagnostics().first_fault==NativeLandFault::Clock,
            "old sample after accepted native output remains fatal");}
    {Fixture f;check(f.begin(),"cached output deadline initial");
        f.sample.output.timestamp=849;f.clock.t=1501;
        gpenmpc_rfly_stream::OriginalValidity end{};
        check(!f.land.accept(f.sample,1501,end)&&f.diagnostics().first_fault!=NativeLandFault::None,
            "cached output cannot extend original handoff deadline");}
    {Fixture f;f.sample.status.timestamp=849;
        check(!f.begin(),"old same-command mode before LAND event not matched");}
    {Fixture f;f.sample.status.timestamp=f.sample.control_mode.timestamp=450;
        f.guard.value.status_timestamp_us=f.guard.value.mode_timestamp_us=450;
        check(f.land.begin_observed_safety_state(1500,f.sample.status,f.sample.control_mode),
            "550us Commander LAND state accepted under explicit 600us source cadence");
        gpenmpc_rfly_stream::OriginalValidity end{};
        check(f.land.accept(f.sample,1000,end)&&end.valid_until_us==1050,
            "native output expiry remains minimum of fast output and slow Commander ages");}
    {Fixture f;f.sample.status.timestamp=399;f.guard.value.status_timestamp_us=399;
        check(!f.land.begin_observed_safety_state(1500,f.sample.status,f.sample.control_mode),
            "Commander LAND state beyond 600us fails closed");}
    {Fixture f;check(f.begin(),"revoke initial");f.land.revoke();
        check(f.land.choose(1000,f.sample.status,f.sample.control_mode)==Source::Unavailable&&!f.begin(),"revoke permanent, no reset");}
    {Fixture f;check(f.begin(),"deadline initial");f.clock.t=1501;
        check(f.land.choose(1501,f.sample.status,f.sample.control_mode)==Source::Unavailable,"no tail deadline renewal");}
    // Real same-owner registry cannot replace direct owner implicitly. This
    // uses explicit fixture link identity, no real Mavlink object or board.
    {Fixture f;CanonicalOutputAuthority<TestGuard,TestClock> direct(f.guard,f.clock,f.identity,500);
        gpenmpc_rfly_stream::SharedOutputRegistry registry;
        gpenmpc_rfly_stream::Registration old{},next{},selection{};
        int link_storage=0,view=0;gpenmpc_rfly_stream::Link link{&link_storage,7};
        check(registry.bind(link,direct,old)==gpenmpc_rfly_stream::Bind::Registered,"direct first owner registered");
        check(f.begin(),"separate native owner ready");
        check(registry.bind(link,f.land,next)==gpenmpc_rfly_stream::Bind::Busy,"two output owners forbidden");
        direct.revoke();check(registry.unbind(old)==gpenmpc_rfly_stream::Detach::Detached,"direct revoke and real PI quiescent detach");
        check(registry.bind(link,f.land,next)==gpenmpc_rfly_stream::Bind::Registered,"only then bind native safety owner");
        gpenmpc_rfly_stream::OriginalValidity valid{};
        check(!registry.accept(link,&view,old,f.sample,1000,valid),"old selection not reused after handoff");
        check(registry.choose(link,&view,1000,f.sample.status,f.sample.control_mode,selection)==Source::NativeOutputsSim,"real new registry chooses native");
        check(registry.accept(link,&view,selection,f.sample,1000,valid),"real new registry accepts actual original native sample");
        check(registry.unbind(next)==gpenmpc_rfly_stream::Detach::Detached,"native view drained before storage release");}
    std::printf("{\"checks\":%u,\"failed\":%u,\"guard_and_HRT\":\"EXPLICIT_HOST_FIXTURE\"}\n",checks,failed);
    return failed?1:0;
}
