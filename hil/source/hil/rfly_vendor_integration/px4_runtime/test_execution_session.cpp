#include "SessionBoundGuard.hpp"
#include <atomic>
#include <cstdio>
#include <thread>
#include <vector>

using namespace gpenmpc_rfly_px4;
using namespace gpenmpc_rfly_stream;
static unsigned checks=0,failures=0;
static void check(bool value,const char *label)
{++checks;if(!value){++failures;std::fprintf(stderr,"FAIL: %s\n",label);}}
static SessionDigest digest(unsigned value){SessionDigest d{};d[0]=value;return d;}
struct MockClock {std::uint64_t value=1000;std::uint64_t now()const noexcept{return value;}};
struct MockRawGuard {
    MockClock &clock;BoardSafetyEvidence raw{};bool available=true;
    explicit MockRawGuard(MockClock &c):clock(c)
    {
        raw.identity.uid=0x1122334455667788ULL;raw.identity.system=1;raw.identity.component=1;
        raw.uid=raw.mavlink_identity=raw.usb_transport=raw.hil_configuration=GuardFact::Pass;
        raw.telemetry_fresh=raw.active_direct_mode=raw.native_controllers_disabled=GuardFact::Pass;
        raw.pwm_functions_zero=raw.pwm_out_stopped=raw.io_driver_stopped=raw.dshot_stopped=GuardFact::Pass;
        raw.usb_power_observed=GuardFact::Pass;raw.checked_pwm_functions=16;
        raw.raw_usb_connected=1;raw.raw_usb_valid=1;raw.raw_servo_valid=1;
        raw.status_timestamp_us=raw.mode_timestamp_us=raw.offboard_timestamp_us=raw.power_timestamp_us=900;
        raw.original_valid_until_us=2000;raw.identity_valid_until_us=2000;raw.reason=GuardReason::BootSessionUnknown;
    }
    bool observe(BoardSafetyEvidence &out)noexcept{out=raw;out.observation_us=clock.now();return available;}
};
using Guard=SessionBoundGuard<MockRawGuard,MockClock>;
struct Fixture {
    MockClock clock{};MockRawGuard raw{clock};ExecutionSessionLedger ledger{};LinkLifetimeRegistry links{};
    int link_object=0;LinkToken token{};SessionBoardIdentity expected{0x1122334455667788ULL,1,1};
    Fixture(){check(links.register_constructed(&link_object,token)==LinkRegistration::Registered,"real registry register");
        check(links.activate(&link_object)==LinkAccess::Read,"real registry activate");}
};
static SessionPhysicalDeclaration declaration(const SessionEcho &echo)
{
    SessionPhysicalDeclaration d{};
    d.source=PhysicalDeclarationSource::OperatorUsbIsolationDeclaration;
    d.physical_setup_record_sha256=digest(17);d.exact_session_sha256=execution_session_digest(echo);
    d.usb_only=d.props_removed=d.no_actuator_propulsion_power=HumanIsolationClaim::Declared;
    return d;
}
static SessionTicketBinding binding(const SessionEcho &echo)
{return {execution_session_digest(echo),echo.process_session_generation};}
static bool enroll(Guard &guard,SessionEcho &echo,HostSessionChallenge challenge={1,2})
{return guard.register_session(challenge,echo) && guard.confirm_echo(echo);}
int main()
{
    {
        Fixture f;Guard g(f.raw,f.clock,f.ledger,f.links,f.token,f.expected,digest(7));SessionEcho echo{};
        BoardSafetyEvidence e{};check(!g.observe(e),"unregistered unavailable");
        check(g.register_session({1,2},echo),"actual fixture observed registration");
        check(echo.observed_identity==f.expected && echo.board_registration_hrt_us==1000,"observed identity and separate board HRT");
        check(echo.process_session_generation==1 && echo.link==f.token,"board assigned generation and lifecycle");
        check(echo.identity_semantics==SessionIdentitySemantics::BoardRegisteredExecutionSessionGenerationV1,"explicit legacy semantics");
        check(!g.observe(e),"unconfirmed echo unavailable");check(g.confirm_echo(echo),"exact echo confirmation");
        check(g.observe(e) && e.require_identity_only() && e.boot_session==GuardFact::Pass,"identity-only before physical attestation");
        check(e.identity.boot_generation==echo.process_session_generation,"execution-session ABI field mapping");
        check(!e.require_physical_facts() && e.external_physical_isolation==GuardFact::Unknown,"missing human declaration cannot grant");
        check(g.bind_physical_declaration(declaration(echo)),"bind physical setup declaration");
        check(g.observe(e) && e.require_active_direct() && e.require_physical_facts(),"composed mock facts plus declaration");
        check(e.status_timestamp_us==900 && e.power_timestamp_us==900 && e.original_valid_until_us==2000,"source stamps and expiry unchanged");
        SessionGuardEvidence side{};check(g.evidence_snapshot(side),"sidecar readable");
        check(side.raw_board.identity.boot_generation==0 && side.raw_board.external_physical_isolation==GuardFact::Unknown,"raw observation never rewritten");
        check(side.human_declaration_bound && side.human_declaration.physical_setup_record_sha256==digest(17),"human provenance retained");
        check(g.validate_ticket_binding(binding(echo)),"new exact per-ticket binding accepted");
        check(g.revoke() && !g.observe(e) && !g.validate_ticket_binding(binding(echo)),"revoke denies both observation and tickets");
        Guard next(f.raw,f.clock,f.ledger,f.links,f.token,f.expected,digest(7));SessionEcho later{};
        check(!next.register_session({1,2},later),"old challenge rejected after revoked object");
        Guard fresh(f.raw,f.clock,f.ledger,f.links,f.token,f.expected,digest(7));
        check(enroll(fresh,later,{1,3}) && later.process_session_generation==2,"fresh challenge increments process generation");
        check(!fresh.validate_ticket_binding(binding(echo)),"previous-session ticket rejected");
    }
    {
        Fixture f;Guard g(f.raw,f.clock,f.ledger,f.links,f.token,f.expected,digest(7));SessionEcho echo{};
        check(enroll(g,echo),"link reuse setup");check(f.links.retire(f.token)==LinkRetirement::Quiesced,"real link retirement");
        LinkToken replacement{};check(f.links.register_constructed(&f.link_object,replacement)==LinkRegistration::Registered,"same address new construction");
        check(f.links.activate(&f.link_object)==LinkAccess::Read && replacement.generation!=f.token.generation,"new lifecycle activation");
        BoardSafetyEvidence e{};check(!g.observe(e),"same address cannot revive old session");
        SessionGuardEvidence side{};check(g.evidence_snapshot(side) && side.first_fault==SessionFault::LinkUnavailable,"link fault typed");
    }
    for(unsigned kind=0;kind<3;++kind){
        Fixture f;Guard g(f.raw,f.clock,f.ledger,f.links,f.token,f.expected,digest(7));SessionEcho echo{};check(enroll(g,echo),"identity drift setup");
        if(kind==0)++f.raw.raw.identity.uid;
        if(kind==1)++f.raw.raw.identity.system;
        if(kind==2)++f.raw.raw.identity.component;
        BoardSafetyEvidence e{};check(!g.observe(e),"each observed identity drift rejected");
    }
    for(unsigned kind=0;kind<6;++kind){
        Fixture f;Guard g(f.raw,f.clock,f.ledger,f.links,f.token,f.expected,digest(7));SessionEcho echo{};check(enroll(g,echo),"time setup");
        if(kind==0)f.clock.value=999;
        if(kind==1)f.raw.raw.status_timestamp_us=899;
        if(kind==2)f.raw.raw.mode_timestamp_us=899;
        if(kind==3)f.raw.raw.offboard_timestamp_us=899;
        if(kind==4)f.raw.raw.power_timestamp_us=899;
        if(kind==5)f.raw.raw.identity_valid_until_us=999;
        BoardSafetyEvidence e{};check(!g.observe(e),"original HRT regression or actual expiry rejected");
        const auto failed_time=f.clock.value;f.clock.value=1001;
        SessionGuardEvidence side{};check(g.evidence_snapshot(side) && side.first_fault_hrt_us==failed_time,"first fault original HRT retained");
        check(!g.observe(e),"fault permanently closes session");
    }
    {
        Fixture f;Guard g(f.raw,f.clock,f.ledger,f.links,f.token,f.expected,digest(7));SessionEcho echo{};check(enroll(g,echo),"duplicate setup");
        check(!g.register_session({8,9},echo),"same object duplicate registration rejected");
        SessionGuardEvidence side{};check(g.evidence_snapshot(side) && side.first_fault==SessionFault::DuplicateRegistration,"duplicate cannot silently replace context");
    }
    for(unsigned kind=0;kind<9;++kind){
        Fixture f;Guard g(f.raw,f.clock,f.ledger,f.links,f.token,f.expected,digest(7));SessionEcho echo{};
        check(g.register_session({1,2},echo),"echo mismatch setup");
        if(kind==0)++echo.host_challenge.high;
        if(kind==1)++echo.host_challenge.low;
        if(kind==2)++echo.observed_identity.uid;
        if(kind==3)++echo.board_registration_hrt_us;
        if(kind==4)++echo.process_session_generation;
        if(kind==5)++echo.link.generation;
        if(kind==6)echo.configuration_sha256=digest(8);
        if(kind==7)echo.link.pointer=nullptr;
        if(kind==8)echo.identity_semantics=static_cast<SessionIdentitySemantics>(99);
        check(!g.confirm_echo(echo),"echo fields cannot be relabeled");
    }
    for(unsigned kind=0;kind<6;++kind){
        Fixture f;Guard g(f.raw,f.clock,f.ledger,f.links,f.token,f.expected,digest(7));SessionEcho echo{};check(enroll(g,echo),"declaration negative setup");
        auto d=declaration(echo);
        if(kind==0)d.source=PhysicalDeclarationSource::Missing;
        if(kind==1)d.physical_setup_record_sha256={};
        if(kind==2)d.exact_session_sha256=digest(99);
        if(kind==3)d.usb_only=HumanIsolationClaim::Unknown;
        if(kind==4)d.props_removed=HumanIsolationClaim::Contradicted;
        if(kind==5)d.no_actuator_propulsion_power=HumanIsolationClaim::Unknown;
        check(!g.bind_physical_declaration(d),"missing or incorrect attributable scope rejected");
    }
    for(unsigned kind=0;kind<5;++kind){
        Fixture f;Guard g(f.raw,f.clock,f.ledger,f.links,f.token,f.expected,digest(7));SessionEcho echo{};check(enroll(g,echo),"independent board facts setup");
        check(g.bind_physical_declaration(declaration(echo)),"human assertion setup");
        if(kind==0)f.raw.raw.pwm_functions_zero=GuardFact::Fail;
        if(kind==1)f.raw.raw.pwm_out_stopped=GuardFact::Fail;
        if(kind==2)f.raw.raw.io_driver_stopped=GuardFact::Fail;
        if(kind==3)f.raw.raw.dshot_stopped=GuardFact::Unknown;
        if(kind==4)f.raw.raw.usb_power_observed=GuardFact::Unknown;
        BoardSafetyEvidence e{};check(g.observe(e) && !e.require_physical_facts(),"human declaration cannot override real physical guard");
    }
    {
        Fixture f;f.raw.raw.external_physical_isolation=GuardFact::Pass;
        Guard g(f.raw,f.clock,f.ledger,f.links,f.token,f.expected,digest(7));SessionEcho echo{};check(enroll(g,echo),"raw boolean setup");
        BoardSafetyEvidence e{};check(g.observe(e) && e.external_physical_isolation==GuardFact::Unknown,"raw plan boolean cannot bypass provenance");
    }
    {
        Fixture f;f.raw.raw.identity.boot_generation=1;
        Guard g(f.raw,f.clock,f.ledger,f.links,f.token,f.expected,digest(7));SessionEcho echo{};
        check(!g.register_session({1,2},echo),"HOST legacy boot1 rejected as raw observation");
    }
    {
        ExecutionSessionLedger ledger;std::uint64_t generation=999;
        check(ledger.reserve({},generation)==SessionRegistration::Invalid && generation==0,"zero challenge rejected");
        for(unsigned i=0;i<ExecutionSessionLedger::capacity;++i)
            check(ledger.reserve({3,i+1},generation)==SessionRegistration::Registered && generation==i+1,"bounded ledger unique allocation");
        check(ledger.reserve({4,1},generation)==SessionRegistration::Full && generation==0,"full ledger rejects without eviction");
        check(ledger.reserve({3,1},generation)==SessionRegistration::Replay,"oldest nonce still rejected when full");
    }
    {
        ExecutionSessionLedger ledger;std::atomic<unsigned> registered{0},replayed{0};std::vector<std::thread> threads;
        for(unsigned i=0;i<24;++i)threads.emplace_back([&]{std::uint64_t gen=0;auto r=ledger.reserve({77,88},gen);
            if(r==SessionRegistration::Registered)++registered;
            if(r==SessionRegistration::Replay)++replayed;});
        for(auto &t:threads)t.join();
        check(registered==1 && replayed==23,"real pthread concurrent duplicate only one registration");
    }
    {
        Fixture f;f.raw.raw.original_valid_until_us=850;f.raw.raw.telemetry_fresh=GuardFact::Fail;
        Guard g(f.raw,f.clock,f.ledger,f.links,f.token,f.expected,digest(7));SessionEcho echo{};
        check(enroll(g,echo),"identity registration does not renew or require unused old OFFBOARD");
        BoardSafetyEvidence e{};
        check(g.observe(e)&&e.identity_valid_until_us==2000&&e.original_valid_until_us==850,
            "original identity and direct bounds remain separate");
        check(e.telemetry_fresh==GuardFact::Fail&&!e.require_active_direct(),"identity cannot erase active-direct stale fault");
        f.raw.raw.identity_valid_until_us=999;
        check(!g.observe(e),"stale identity still fail closed");
    }
    std::printf("{\"checks\":%u,\"failures\":%u,\"passed\":%s,\"raw_guard_clock_and_declaration\":\"HOST_FIXTURE\",\"real_pthread_and_link_registry\":true}\n",checks,failures,failures?"false":"true");
    return failures?1:0;
}
