#include "RegisteredHostHeartbeat.hpp"
#include "../px4_runtime/DirectOffboardPrestream.hpp"
#include <cstdio>
#include <initializer_list>

using namespace gpenmpc_rfly_stream;
using namespace gpenmpc_rfly_px4;
namespace {
unsigned checks{},failed{};
void check(bool ok,const char *name){++checks;if(!ok){++failed;std::fprintf(stderr,"FAIL %s\n",name);}}
HostHeartbeatBinding binding(){return {77,1000,42,191};}
PrestreamPrerequisites healthy(){return {PrestreamFact::Pass,PrestreamFact::Pass,PrestreamFact::Pass,false,false,false};}
bool no_output(const PrestreamDecision &d){return d.action!=PrestreamAction::EmitDirect &&
    !d.offboard.timestamp && !d.offboard.direct_actuator;}
bool only_direct(const DirectOffboardSuggestion &v){return v.direct_actuator &&
    !v.position && !v.velocity && !v.acceleration && !v.attitude && !v.body_rate && !v.thrust_and_torque;}
}
int main()
{
    constexpr std::uint64_t age=100000;
    RegisteredHostHeartbeat h;
    check(h.snapshot(1001,age).freshness==HeartbeatFreshness::Unbound,"default unbound");
    check(h.record(42,191,6,8,1001)==HeartbeatRecord::Unbound,"unbound ignores actual arrivals");
    check(!h.bind({0,1000,42,191})&&!h.bind({77,0,42,191})&&!h.bind({77,1000,0,191})&&
          !h.bind({77,1000,42,0}),"invalid original binding");
    check(h.bind(binding()),"explicit binding");
    check(!h.bind(binding()),"duplicate binding cannot reset");
    check(h.snapshot(1001,age).freshness==HeartbeatFreshness::Missing,"bound no heartbeat");
    for(const auto time:{std::uint64_t(0),std::uint64_t(999),std::uint64_t(1000)})
        check(h.record(42,191,6,8,time)==HeartbeatRecord::RejectedTime,"receipt must follow registration");
    check(h.record(41,191,6,8,UINT64_MAX)==HeartbeatRecord::IgnoredTuple,"other system ignored");
    check(h.record(42,190,6,8,UINT64_MAX)==HeartbeatRecord::IgnoredTuple,"other component ignored");
    check(h.record(42,191,18,8,UINT64_MAX)==HeartbeatRecord::IgnoredTuple,"not GCS ignored");
    check(h.record(42,191,6,12,UINT64_MAX)==HeartbeatRecord::IgnoredTuple,"not INVALID autopilot ignored");
    check(h.record(42,191,6,8,1001)==HeartbeatRecord::Recorded,"wrong tuple did not move time watermark");
    auto original=h.snapshot(1001,age);
    check(original.freshness==HeartbeatFreshness::Fresh && original.receipt_generation==1 &&
          original.original_receiver_hrt_us==1001 && original.original_valid_until_us==101001 &&
          original.binding==binding() && original.source_system==42 && original.source_component==191,
          "exact original receipt tuple and deadline");
    check(h.record(42,191,6,8,1001)==HeartbeatRecord::Duplicate,"same original receipt no new generation");
    check(h.record(42,191,6,8,1000)==HeartbeatRecord::RejectedTime,"HRT rollback not accepted");
    check(h.snapshot(1001,age).receipt_generation==1,"duplicates and rollback retain generation");
    check(h.snapshot(1000,age).freshness==HeartbeatFreshness::InvalidClock,"snapshot before original receipt");
    check(h.snapshot(101001,age).freshness==HeartbeatFreshness::Fresh,"100 ms inclusive boundary");
    check(h.snapshot(101002,age).freshness==HeartbeatFreshness::Expired,"one us after 100 ms expires");
    check(h.snapshot(1001,0).freshness==HeartbeatFreshness::InvalidAge,"zero explicit age rejected");
    for(unsigned k=0;k<2000;++k){
        const auto s=h.snapshot(1001+100*k,age);
        check(s.original_receiver_hrt_us==1001 && s.receipt_generation==1 && s.original_valid_until_us==101001,
              "2000 snapshots cannot renew source time/deadline/generation");
    }
    DirectOffboardPrestream dormant;
    check(no_output(dormant.poll(1001,original,healthy())) && dormant.state()==PrestreamState::Dormant,"no implicit start");
    RegisteredHostHeartbeat empty;check(empty.bind(binding()),"waiting binding");
    DirectOffboardPrestream wait;check(wait.start(binding(),age),"explicit prestream start");
    for(unsigned k=0;k<2000;++k)
        check(wait.poll(1001+k,empty.snapshot(1001+k,age),healthy()).action==PrestreamAction::Waiting &&
              wait.state()==PrestreamState::Waiting,"2000 no-input cycles remain waiting without OCM");
    check(empty.record(42,191,6,8,4000)==HeartbeatRecord::Recorded,"new heartbeat after wait");
    auto emitted=wait.poll(4000,empty.snapshot(4000,age),healthy());
    check(emitted.action==PrestreamAction::EmitDirect && only_direct(emitted.offboard) && emitted.offboard.timestamp==4000 &&
          emitted.original_valid_until_us==104000,"only new original heartbeat emits direct-only mode suggestion");
    check(!wait.start(binding(),age),"repeated start cannot reset enabled policy");
    const auto same=wait.poll(4001,empty.snapshot(4001,age),healthy());
    check(same.action==PrestreamAction::NoChange && no_output(same) && same.original_valid_until_us==104000,
          "same heartbeat emits no duplicate publication or renewed credit");
    check(empty.record(42,191,6,8,5000)==HeartbeatRecord::Recorded,"fresh second heartbeat");
    emitted=wait.poll(5000,empty.snapshot(5000,age),healthy());
    check(emitted.action==PrestreamAction::EmitDirect && emitted.offboard.timestamp==5000 && emitted.receipt_generation==2,
          "second receipt creates one new suggestion");
    check(wait.poll(105000,empty.snapshot(105000,age),healthy()).action==PrestreamAction::NoChange,"policy 100 ms inclusive boundary");
    check(wait.poll(105001,empty.snapshot(105001,age),healthy()).action==PrestreamAction::Revoked,"expired enabled policy revoked");
    check(empty.record(42,191,6,8,105002)==HeartbeatRecord::Recorded,"recorder remains usable independently of direct policy");
    check(no_output(wait.poll(105002,empty.snapshot(105002,age),healthy())) && !wait.start(binding(),age),
          "new heartbeat cannot automatically retake revoked lifetime");
    check(empty.snapshot(105002,age).binding==binding() && empty.snapshot(105002,age).freshness==HeartbeatFreshness::Fresh,
          "HOST liveness loss does not poison session binding or independent native recovery");
    RegisteredHostHeartbeat held;check(held.bind(binding()) && held.record(42,191,6,8,1001)==HeartbeatRecord::Recorded,"held heartbeat fixture");
    DirectOffboardPrestream lost;check(lost.start(binding(),age) &&
        lost.poll(1001,held.snapshot(1001,age),healthy()).action==PrestreamAction::EmitDirect,"enabled before 2000 no-new-input cycles");
    for(unsigned k=0;k<2000;++k){
        const std::uint64_t now=1001+100*k;
        const auto d=lost.poll(now,held.snapshot(now,age),healthy());
        const auto wanted=now<=101001?PrestreamAction::NoChange:PrestreamAction::Revoked;
        check(d.action==wanted && no_output(d) && d.original_valid_until_us==101001 && d.receipt_generation==1,
              "2000 enabled polls never create output credit and permanently revoke at original expiry");
    }
    check(lost.state()==PrestreamState::Revoked,"2000-cycle original deadline not revived");
    for(unsigned k=0;k<3;++k){
        DirectOffboardPrestream policy;check(policy.start(binding(),age),"prerequisite policy start");
        auto p=healthy();
        if(k==0)p.session=PrestreamFact::Unknown;
        if(k==1)p.physical=PrestreamFact::Unknown;
        if(k==2)p.atomic_source=PrestreamFact::Unknown;
        check(policy.poll(105002,empty.snapshot(105002,age),p).action==PrestreamAction::Waiting,"unknown before enable waits");
        check(policy.poll(105002,empty.snapshot(105002,age),healthy()).action==PrestreamAction::EmitDirect,"real prerequisite results supplied");
        check(policy.poll(105003,empty.snapshot(105003,age),p).action==PrestreamAction::Revoked,"lost enabled prerequisite revokes");
        check(no_output(policy.poll(105004,empty.snapshot(105004,age),healthy())),"prerequisite restoration cannot restart");
    }
    for(unsigned k=0;k<3;++k){
        DirectOffboardPrestream policy;check(policy.start(binding(),age),"stop LAND fault policy start");
        check(policy.poll(105002,empty.snapshot(105002,age),healthy()).action==PrestreamAction::EmitDirect,"enabled before terminal event");
        auto p=healthy();if(k==0)p.stop=true;if(k==1)p.land=true;if(k==2)p.fault=true;
        const auto stopped=policy.poll(105003,empty.snapshot(105003,age),p);
        check(stopped.action==PrestreamAction::Revoked && no_output(stopped),"stop LAND fault no OCM suggestion");
        const auto reason=policy.first_reason();policy.stop();
        check(policy.first_reason()==reason && !policy.start(binding(),age) &&
              no_output(policy.poll(105004,empty.snapshot(105004,age),healthy())),"first terminal reason preserved forever");
    }
    DirectOffboardPrestream stale;
    check(stale.start(binding(),age),"old binding policy start");
    auto bad=empty.snapshot(105002,age);bad.binding.session_generation++;
    check(no_output(stale.poll(105002,bad,healthy())),"other original session cannot emit");
    check(stale.poll(105002,empty.snapshot(105002,age),healthy()).action==PrestreamAction::EmitDirect,"original session enables");
    bad=empty.snapshot(105003,age);++bad.original_receiver_hrt_us;++bad.original_valid_until_us;
    check(stale.poll(105003,bad,healthy()).action==PrestreamAction::Revoked,"same generation changed original timestamp rejected");
    DirectOffboardPrestream clock;check(clock.start(binding(),age),"clock policy start");
    check(clock.poll(105002,empty.snapshot(105002,age),healthy()).action==PrestreamAction::EmitDirect,"clock policy enabled");
    check(clock.poll(105001,empty.snapshot(105001,age),healthy()).reason==PrestreamReason::Clock,"policy original clock reversal revokes");
    DirectOffboardPrestream expanded;check(expanded.start(binding(),age),"frozen age policy start");
    check(expanded.poll(105002,empty.snapshot(105002,age),healthy()).action==PrestreamAction::EmitDirect,"frozen age policy enabled");
    check(expanded.poll(105003,empty.snapshot(105003,age+1),healthy()).action==PrestreamAction::Revoked,"snapshot cannot silently widen frozen 100 ms age");
    RegisteredHostHeartbeat edge;check(edge.bind({1,UINT64_MAX-2,42,191}),"uint64 edge original binding");
    check(edge.record(42,191,6,8,UINT64_MAX-1)==HeartbeatRecord::Recorded,"uint64 edge actual receipt");
    check(edge.snapshot(UINT64_MAX,2).freshness==HeartbeatFreshness::InvalidAge &&
          edge.snapshot(UINT64_MAX,2).original_valid_until_us==0,"deadline overflow rejected not saturated");
    empty.retire();const auto retired=empty.snapshot(105004,age);
    check(retired.freshness==HeartbeatFreshness::Unbound && !retired.binding.session_generation &&
          !retired.original_receiver_hrt_us && !retired.receipt_generation && !retired.original_valid_until_us,"retire clears all bound evidence");
    check(empty.record(42,191,6,8,105005)==HeartbeatRecord::Unbound,"retired recorder cannot resurrect receipt");
    check(empty.bind({78,200000,42,191}),"external owner explicitly binds a later session");
    check(empty.record(42,191,6,8,105005)==HeartbeatRecord::RejectedTime,"old original receipt cannot enter new registration");
    DirectOffboardPrestream invalid;check(!invalid.start(binding(),0) && !invalid.start(binding(),age),"invalid configuration does not get automatic restart");
    std::printf("{\"checks\":%u,\"failed\":%u,\"no_input_snapshot_cycles\":2000,\"no_input_policy_cycles\":2000,\"enabled_no_new_input_cycles\":2000,"
        "\"frozen_heartbeat_max_age_us\":100000,\"actual_uorb_publications\":0,"
        "\"scope\":\"HOST_PURE_COMPONENTS_EXTERNAL_PI_LOCK_REQUIRED_NOT_AUTHENTICATED_LIVENESS\"}\n",checks,failed);
    return failed?1:0;
}
