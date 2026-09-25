#include "CanonicalOutputAuthority.hpp"
#include <atomic>
#include <cstdio>
#include <limits>
#include <thread>
#include <initializer_list>

using namespace gpenmpc_rfly_px4;
static unsigned checks=0,failures=0;
static void check(bool ok,const char *name){++checks;if(!ok){++failures;std::fprintf(stderr,"FAIL %s\n",name);}}
struct MockClock {std::atomic<std::uint64_t> t{1000};std::uint64_t now()noexcept{return t.load();}};
struct MockGuard {
    MockClock &clock;
    BoardSafetyEvidence value{};
    std::atomic<unsigned> inside{0},overlap{0};
    bool available{true};
    bool observe(BoardSafetyEvidence &out)noexcept
    {
        if(inside.fetch_add(1)!=0)++overlap;
        out=value;out.observation_us=clock.now();
        std::this_thread::yield();
        --inside;
        return available;
    }
};
using Owner=CanonicalOutputAuthority<MockGuard,MockClock>;
struct Fixture {
    Identity id{0x1122334455667788ULL,9007199254740999ULL,1,1};
    MockClock clock{};
    MockGuard guard{clock};
    Owner owner{guard,clock,id,500,600};
    vehicle_status_s status{};
    vehicle_control_mode_s mode{};
    offboard_control_mode_s offboard{};
    Token token{};
    actuator_outputs_s output{};
    Fixture()
    {
        auto &e=guard.value;e.identity=id;e.original_valid_until_us=1400;e.identity_valid_until_us=1400;
        e.status_timestamp_us=900;e.mode_timestamp_us=900;e.offboard_timestamp_us=900;
        e.uid=e.mavlink_identity=e.boot_session=e.usb_transport=e.hil_configuration=e.telemetry_fresh=
        e.active_direct_mode=e.native_controllers_disabled=e.pwm_functions_zero=e.pwm_out_stopped=
        e.io_driver_stopped=e.dshot_stopped=e.usb_power_observed=e.external_physical_isolation=GuardFact::Pass;
        status.timestamp=900;status.hil_state=vehicle_status_s::HIL_STATE_ON;
        status.arming_state=vehicle_status_s::ARMING_STATE_ARMED;status.nav_state=vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
        mode.timestamp=900;mode.flag_armed=true;mode.flag_control_offboard_enabled=true;
        offboard.timestamp=900;offboard.direct_actuator=true;
        token.identity=id;token.publication_path=gpenmpc_consumption::PublicationPath::DirectCanonicalMotors;
        token.transaction=1;token.output_generation=1;token.sample_generation=1;
        token.timestamp_sample_us=800;token.control_tick_us=950;
        token.reference_valid_until_us=1300;token.outer_valid_until_us=1350;
        output.timestamp=1000;output.noutputs=0;
        for(unsigned i=0;i<6;++i)output.output[i]=float(i)/10.f;
    }
    bool validate(){std::uint64_t expiry=0;const bool ok=owner.validate(token,status,mode,offboard,clock.now(),expiry);return ok && expiry==1300;}
    bool install(){return owner.install_lease(token,output,1250);}
    bool confirm(){return owner.confirm_publication(token,clock.now(),clock.now());}
    bool ready(){return owner.begin_direct()&&validate()&&install()&&confirm();}
    gpenmpc_rfly_stream::Observation sample(unsigned generation=1)
    {
        gpenmpc_rfly_stream::Observation s{};
        s.source=gpenmpc_rfly_stream::Source::RflyOutputs;s.output=output;s.status=status;s.control_mode=mode;
        s.output_uorb_generation=generation;return s;
    }
    bool consume(unsigned generation=1){gpenmpc_rfly_stream::OriginalValidity v{};return owner.accept(sample(generation),clock.now(),v)&&v.valid_until_us==1250;}
    bool faulted(LeaseFault why){LeaseDiagnostics d{};return owner.diagnostic_snapshot(d)&&d.first_fault==why;}
};

int main(int argc,char**argv)
{
    if(argc==2&&std::strcmp(argv[1],"TRANSPORT_ONLY")==0){
        for(bool split:{false,true}){Fixture f;f.clock.t=71479817;
            const auto start=f.clock.now();auto&e=f.guard.value;
            e.identity_valid_until_us=e.original_valid_until_us=start+100000;
            f.status.timestamp=f.mode.timestamp=f.offboard.timestamp=start;
            f.token.timestamp_sample_us=71479026;f.token.control_tick_us=start;
            f.token.reference_valid_until_us=f.token.outer_valid_until_us=start+50000;
            check(f.owner.begin_direct(),"actual authority selected under mock fresh physical guard");
            std::uint64_t bound=0;check(f.owner.validate(f.token,f.status,f.mode,f.offboard,start,bound),"token validated");
            f.output.timestamp=71481684;f.clock.t=f.output.timestamp;
            const auto until=start+(split?20000:4000);
            // The fixture uses a 500us fast telemetry bound: refresh real test
            // publications, never rewrite the retained output/sample time.
            f.status.timestamp=f.mode.timestamp=f.offboard.timestamp=f.clock.now();
            check(f.owner.install_lease(f.token,f.output,until),"original output install");
            f.clock.t=71481832;check(f.owner.confirm_publication(f.token,71481684,71481832),"actual recorded timely commit");
            f.clock.t=71485000;f.status.timestamp=f.mode.timestamp=f.offboard.timestamp=f.clock.now();
            auto s=f.sample();gpenmpc_rfly_stream::OriginalValidity v{};Identity observed{};
            const auto chosen=f.owner.choose(f.clock.now(),f.status,f.mode);
            if(!split){check(chosen==gpenmpc_rfly_stream::Source::Unavailable&&f.faulted(LeaseFault::Time),"5ms stream misses compute-derived expiry");
                check(!f.owner.observed_identity(observed),"expired stream lease surfaces as identity unavailable on next capture");}
            else{check(chosen==gpenmpc_rfly_stream::Source::RflyOutputs&&f.owner.accept(s,f.clock.now(),v)&&v.valid_until_us==until,"same delayed stream consumes exact valid output");
                check(f.owner.observed_identity(observed)&&observed==f.id,"next capture retains original identity");
                check(f.owner.choose(f.clock.now(),f.status,f.mode)==gpenmpc_rfly_stream::Source::Unavailable,"accepted output is never replayed");}
        }
        std::printf("TRANSPORT_ONLY checks=%u failed=%u actual_authority=1 mock_guard_clock=1 COM=0\n",checks,failures);return failures?1:0;
    }
    {InheritingMutex mutex;std::atomic<bool> locked{false};
        std::thread abandoned([&](){locked=mutex.lock();});abandoned.join();
        check(locked,"real POSIX robust owner-death setup");
        check(!mutex.lock(),"owner death never recovers untrusted output state");
        check(!mutex.lock(),"poisoned mutex remains unusable without waiting");}
    {Fixture f;Identity observed{};
        f.status.arming_state=vehicle_status_s::ARMING_STATE_DISARMED;
        f.guard.value.active_direct_mode=GuardFact::Fail;f.guard.value.original_valid_until_us=850;
        for(unsigned i=0;i<3;++i)check(f.owner.choose(f.clock.now(),f.status,f.mode)==gpenmpc_rfly_stream::Source::Unavailable,"normal unselected polling");
        check(f.owner.observed_identity(observed)&&observed==f.id,"disarmed capture identity independent of active mode");
        check(f.faulted(LeaseFault::None),"unselected polling not poisoned");}
    {Fixture f;check(f.ready(),"real PI mutex and exact lease installed");
        check(f.owner.choose(f.clock.now(),f.status,f.mode)==gpenmpc_rfly_stream::Source::RflyOutputs,"selected canonical only");
        check(f.consume(),"one exact sample original expiry consumed");
        check(f.owner.choose(f.clock.now(),f.status,f.mode)==gpenmpc_rfly_stream::Source::Unavailable,"no repeated cached output");
        check(!f.consume()&&f.faulted(LeaseFault::NoLease),"duplicate accept rejected");}
    {Fixture f;check(f.ready(),"asynchronous status setup");
        f.guard.value.status_timestamp_us=990;f.guard.value.mode_timestamp_us=991;
        check(f.consume(),"different fresh subscription timestamps are not a spurious fault");}
    {Fixture f;check(f.ready(),"revoke setup");f.owner.revoke();
        check(f.owner.choose(f.clock.now(),f.status,f.mode)==gpenmpc_rfly_stream::Source::Unavailable,"revoke without subsequent update");
        check(!f.consume()&&f.faulted(LeaseFault::Revoked),"revoked cannot restart or fallback");
        check(!f.owner.begin_direct(),"revocation permanent in same owner lifetime");}
    {Fixture f;check(f.ready(),"expiry setup");f.clock.t=1251;
        check(f.owner.choose(f.clock.now(),f.status,f.mode)==gpenmpc_rfly_stream::Source::Unavailable&&f.faulted(LeaseFault::Time),"original expiry not renewed");}
    {Fixture f;check(f.ready(),"at expiry setup");f.clock.t=1250;check(f.consume(),"expiry equality accepted unchanged");}
    {Fixture f;check(f.ready(),"pending setup");f.token.transaction=2;f.token.output_generation=2;
        check(!f.validate()&&f.faulted(LeaseFault::Unconsumed),"no overwrite of unconsumed lease");}
    {Fixture f;check(f.owner.begin_direct()&&f.validate(),"mutation setup");++f.token.reference_generation;
        check(!f.install()&&f.faulted(LeaseFault::TokenMismatch),"all token fields bound");}
    {Fixture f;check(f.owner.begin_direct()&&f.validate(),"install expiry setup");
        check(!f.owner.install_lease(f.token,f.output,1301)&&f.faulted(LeaseFault::Time),"cannot extend validation expiry");}
    {Fixture f;check(f.owner.begin_direct()&&f.validate(),"future output setup");f.output.timestamp=1001;
        check(!f.install()&&f.faulted(LeaseFault::Time),"future output rejects");}
    {Fixture f;check(f.ready(),"foreign source setup");auto s=f.sample();s.source=gpenmpc_rfly_stream::Source::NativeOutputsSim;
        gpenmpc_rfly_stream::OriginalValidity v{};check(!f.owner.accept(s,1000,v)&&v.valid_until_us==0,"no native source masquerade");}
    for(unsigned index=0;index<16;++index){Fixture f;check(f.ready(),"bit mutation setup");auto s=f.sample();
        if(index==0)s.output.output[index]=-0.0f;else s.output.output[index]+=0.001f;
        gpenmpc_rfly_stream::OriginalValidity v{};
        check(!f.owner.accept(s,1000,v)&&f.faulted(LeaseFault::Payload),"exact output bits including signed zero");}
    for(unsigned kind=0;kind<5;++kind){Fixture f;check(f.owner.begin_direct()&&f.validate(),"invalid output setup");
        if(kind==0)f.output.noutputs=16;
        if(kind==1)f.output.output[0]=std::numeric_limits<float>::quiet_NaN();
        if(kind==2)f.output.output[0]=-0.01f;
        if(kind==3)f.output.output[0]=1.01f;
        if(kind==4)f.output.output[6]=0.01f;
        check(!f.install()&&f.faulted(LeaseFault::Payload),"raw noutputs/range/nonfinite/spare reject");}
    {Fixture f;check(f.ready()&&f.consume(100),"generation setup");
        f.token.transaction=2;f.token.output_generation=2;f.clock.t=1010;f.output.timestamp=1010;
        check(f.validate()&&f.install()&&f.confirm()&&f.consume(103),"real uORB generation gap accepted");}
    {Fixture f;check(f.ready()&&f.consume(~0u),"wrap setup");
        f.token.transaction=2;f.token.output_generation=2;f.clock.t=1010;f.output.timestamp=1010;
        check(f.validate()&&f.install()&&f.confirm()&&f.consume(0),"real unsigned generation wrap accepted");}
    {Fixture f;check(f.ready()&&f.consume(20),"generation replay setup");
        f.token.transaction=2;f.token.output_generation=2;f.clock.t=1010;f.output.timestamp=1010;
        check(f.validate()&&f.install()&&f.confirm(),"second lease independent token");
        check(!f.consume(20)&&f.faulted(LeaseFault::Generation),"duplicate uORB generation rejected");}
    {Fixture f;check(f.ready(),"late changed identity setup");f.guard.value.identity.uid++;
        check(!f.consume()&&f.faulted(LeaseFault::IdentityMismatch),"fresh guard rechecked at consumption");}
    {Fixture f;check(f.ready(),"stale subscription setup");f.status.timestamp=1;
        check(!f.consume()&&f.faulted(LeaseFault::Control),"old stream status cannot borrow guard freshness");}
    {Fixture f;f.status.timestamp=f.mode.timestamp=450;
        check(f.ready()&&f.consume(),"550us Commander sample accepted by explicit 600us source cadence");}
    {Fixture f;f.status.timestamp=399;
        check(!f.owner.begin_direct()||!f.validate(),"Commander sample beyond 600us fails closed");
        check(f.faulted(LeaseFault::Control),"Commander stale fault retained");}
    {Fixture f;f.offboard.timestamp=499;
        check(f.owner.begin_direct()&&!f.validate()&&f.faulted(LeaseFault::Control),
            "offboard sample beyond unchanged 500us fast age fails closed");}
    using Member=GuardFact BoardSafetyEvidence::*;
    const Member facts[]={&BoardSafetyEvidence::uid,&BoardSafetyEvidence::mavlink_identity,&BoardSafetyEvidence::boot_session,
        &BoardSafetyEvidence::usb_transport,&BoardSafetyEvidence::hil_configuration,&BoardSafetyEvidence::telemetry_fresh,
        &BoardSafetyEvidence::active_direct_mode,&BoardSafetyEvidence::native_controllers_disabled,&BoardSafetyEvidence::pwm_functions_zero,
        &BoardSafetyEvidence::pwm_out_stopped,&BoardSafetyEvidence::io_driver_stopped,&BoardSafetyEvidence::dshot_stopped,&BoardSafetyEvidence::usb_power_observed,
        &BoardSafetyEvidence::external_physical_isolation};
    for(const auto fact:facts){for(const auto state:{GuardFact::Unknown,GuardFact::Fail}){Fixture f;f.guard.value.*fact=state;
        check(!f.owner.begin_direct(),"every unknown/failing independent guard rejects");}}
    {Fixture f;std::atomic<unsigned> accepted{0};auto read=[&](){for(unsigned i=0;i<2000;++i){Identity id{};
        if(f.owner.observed_identity(id)&&id==f.id)++accepted;}};
        std::thread a(read),b(read);a.join();b.join();
        check(accepted==4000&&f.guard.overlap==0,"actual two-thread PI mutex serializes guard subscriptions");}
    {Fixture f;check(f.ready(),"concurrent consume setup");std::atomic<unsigned> accepted{0};
        auto consume=[&](){gpenmpc_rfly_stream::OriginalValidity v{};if(f.owner.accept(f.sample(),1000,v))++accepted;};
        std::thread a(consume),b(consume);a.join();b.join();
        check(accepted==1&&f.guard.overlap==0,"concurrent exact output consumed once");}
    {Fixture f;check(f.owner.begin_direct()&&f.validate()&&f.install(),"prepublication setup");
        check(f.owner.choose(1000,f.status,f.mode)==gpenmpc_rfly_stream::Source::Unavailable,"install before orb publish does not expose output");
        check(f.confirm()&&f.consume(),"only real publication plus commit confirmation exposes output");}
    {Fixture f;check(f.owner.begin_direct()&&f.validate()&&f.install(),"failed publish setup");f.owner.revoke();
        check(!f.confirm()&&!f.consume(),"failed publish cannot confirm or send");}
    {Fixture f;check(f.owner.begin_direct()&&f.validate()&&f.install(),"late commit setup");f.clock.t=1251;
        check(!f.owner.confirm_publication(f.token,1000,1251)&&f.faulted(LeaseFault::Time),"late commit cannot become transmissible");}
    using Flag=bool vehicle_control_mode_s::*;
    const Flag forbidden[]={&vehicle_control_mode_s::flag_multicopter_position_control_enabled,
        &vehicle_control_mode_s::flag_control_manual_enabled,&vehicle_control_mode_s::flag_control_auto_enabled,
        &vehicle_control_mode_s::flag_control_position_enabled,&vehicle_control_mode_s::flag_control_velocity_enabled,
        &vehicle_control_mode_s::flag_control_altitude_enabled,&vehicle_control_mode_s::flag_control_climb_rate_enabled,
        &vehicle_control_mode_s::flag_control_acceleration_enabled,&vehicle_control_mode_s::flag_control_attitude_enabled,
        &vehicle_control_mode_s::flag_control_rates_enabled,&vehicle_control_mode_s::flag_control_allocation_enabled,
        &vehicle_control_mode_s::flag_control_termination_enabled};
    for(const auto flag:forbidden){Fixture f;check(f.owner.begin_direct(),"conflicting publisher setup");f.mode.*flag=true;
        check(!f.validate()&&f.faulted(LeaseFault::Control),"every conflicting native-control flag rejects");}
    std::printf("{\"scope\":\"HOST_REAL_PTHREAD_PI_MUTEX_MOCK_GUARD\",\"checks\":%u,\"failed\":%u,\"real_message_shapes\":true,\"uorb_broker\":false,\"serial\":false,\"commander_or_plant_failsafe_proven\":false}\n",checks,failures);
    return failures?1:0;
}
