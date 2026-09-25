#include "NativeOffboardBorrow.hpp"
#include "../px4_runtime/ExecutionSessionRegistration.hpp"
#include <cstdio>
#include <future>
#include <thread>
namespace rs=gpenmpc_rfly_stream;
namespace px=gpenmpc_rfly_px4;
static unsigned checks=0,failed=0;
static void check(bool value,const char *why){++checks;if(!value){++failed;std::fprintf(stderr,"FAIL %s\n",why);}}
static px::SessionEcho echo(rs::LinkToken link)
{
    px::SessionEcho out{};out.link=link;out.observed_identity={123,1,1};
    out.process_session_generation=7;out.board_registration_hrt_us=1000;
    out.host_challenge={2,3};out.configuration_sha256[0]=1;return out;
}
static void local_registry()
{
    rs::LinkLifetimeRegistry registry;int link_a{},link_b{},owner{};rs::LinkToken a{},b{};
    check(registry.register_constructed(&link_a,a)==rs::LinkRegistration::Registered&&registry.activate(&link_a)==rs::LinkAccess::Read,"first actual lifecycle entry");
    check(registry.register_constructed(&link_b,b)==rs::LinkRegistration::Registered&&registry.activate(&link_b)==rs::LinkAccess::Read,"second link entry");
    rs::NativeBorrowToken borrowed{};rs::CanonicalReservation reservation{};
    check(registry.enter_native_offboard(&link_b,borrowed)==rs::LinkAccess::Read,"native handler enters before reservation");
    check(registry.bind_canonical(echo(a),42,191,&owner,reservation)==rs::CanonicalBind::Busy&&!reservation.generation,"in-flight handler on any link blocks canonical reservation");
    auto wrong=borrowed;++wrong.generation;
    check(registry.leave_native_offboard(wrong)==rs::LinkAccess::Rejected,"mismatched borrow generation preserves the in-flight scope");
    check(registry.bind_canonical(echo(a),42,191,&owner,reservation)==rs::CanonicalBind::Busy,"failed fake leave does not open reservation");
    check(registry.leave_native_offboard(borrowed)==rs::LinkAccess::Read,"actual scope exits");
    check(registry.leave_native_offboard(borrowed)==rs::LinkAccess::Rejected,"double leave is not a second decrement");
    check(registry.bind_canonical(echo(a),42,191,&owner,reservation)==rs::CanonicalBind::Bound,"reservation begins only after scope exit");
    for(unsigned handler=0;handler<3;++handler){
        check(registry.enter_native_offboard(&link_a,borrowed)==rs::LinkAccess::Rejected,"each native handler blocked on selected link");
        check(registry.enter_native_offboard(&link_b,borrowed)==rs::LinkAccess::Rejected,"each native handler blocked globally on other link");
    }
    check(registry.native_offboard_allowed()==rs::LinkAccess::Rejected,"legacy spawn query sees reserved OCM");
    rs::HostHeartbeatSnapshot hb{};
    check(registry.read_heartbeat(reservation,1001,100000,hb)==rs::LinkAccess::Read&&hb.freshness==rs::HeartbeatFreshness::Missing,"binding alone creates no heartbeat");
    check(registry.record_heartbeat(&link_b,42,191,6,8,1010)==rs::HeartbeatRecord::Unbound,"other actual link cannot feed selected binding");
    check(registry.record_heartbeat(&link_a,43,191,6,8,1010)==rs::HeartbeatRecord::IgnoredTuple,"other host tuple cannot refresh selected heartbeat");
    check(registry.record_heartbeat(&link_a,42,191,6,8,1010)==rs::HeartbeatRecord::Recorded,"actual registered tuple original HRT recorded");
    check(registry.record_heartbeat(&link_a,42,191,6,8,1010)==rs::HeartbeatRecord::Duplicate,"duplicate original receipt creates no credit");
    check(registry.record_heartbeat(&link_a,42,191,6,8,1009)==rs::HeartbeatRecord::RejectedTime,"backward original receipt rejected");
    check(registry.read_heartbeat(reservation,1011,100000,hb)==rs::LinkAccess::Read&&hb.original_receiver_hrt_us==1010&&hb.receipt_generation==1&&hb.original_valid_until_us==101010,"original HRT and exact original expiry preserved");
    auto wrong_reservation=reservation;++wrong_reservation.generation;
    check(registry.release_canonical(wrong_reservation)==rs::LinkAccess::Rejected,"wrong owner receipt cannot release global reservation");
    check(registry.retire(a)==rs::LinkRetirement::Quiesced,"link pointer read access retired");
    check(registry.native_offboard_allowed()==rs::LinkAccess::Rejected,"Closing tombstone retains global exclusion after link retirement");
    check(registry.read_heartbeat(reservation,1012,100000,hb)==rs::LinkAccess::Unavailable,"retired link cannot provide heartbeat evidence");
    check(registry.release_canonical(reservation)==rs::LinkAccess::Read,"exact detached owner releases tombstone");
    check(registry.native_offboard_allowed()==rs::LinkAccess::Read,"native path restored only after explicit owner release");
    check(registry.release_canonical(reservation)==rs::LinkAccess::Rejected,"stale release cannot affect next owner");
    rs::LinkToken next{};
    check(registry.register_constructed(&link_a,next)==rs::LinkRegistration::Registered&&next.generation>a.generation,"same address later lifecycle cannot inherit heartbeat");
    check(registry.activate(&link_a)==rs::LinkAccess::Read,"new actual lifecycle activated");
    check(registry.bind_canonical(echo(a),42,191,&owner,reservation)==rs::CanonicalBind::Unavailable,"old lifecycle cannot reserve replacement");
    check(registry.retire(next)==rs::LinkRetirement::Quiesced&&registry.retire(b)==rs::LinkRetirement::Quiesced,"cleanup real entries");
}
static void scoped_concurrency()
{
    auto *registry=rs::link_lifetime_registry();int link{},owner{};rs::LinkToken token{};
    check(registry&&registry->register_constructed(&link,token)==rs::LinkRegistration::Registered&&registry->activate(&link)==rs::LinkAccess::Read,"singleton scope test setup");
    std::promise<void> entered,release;auto released=release.get_future();
    std::thread native([&]{rs::NativeOffboardBorrow scope(&link);if(!scope)std::terminate();entered.set_value();released.wait();});
    entered.get_future().wait();rs::CanonicalReservation reservation{};
    check(registry->bind_canonical(echo(token),42,191,&owner,reservation)==rs::CanonicalBind::Busy,"real concurrent entered RAII scope blocks reservation without holding PI lock");
    release.set_value();native.join();
    check(registry->bind_canonical(echo(token),42,191,&owner,reservation)==rs::CanonicalBind::Bound,"RAII destructor releases exact native borrow");
    for(unsigned handler=0;handler<3;++handler){rs::NativeOffboardBorrow scope(&link);check(!scope,"reserved actual scoped handler entry rejected");}
    check(registry->release_canonical(reservation)==rs::LinkAccess::Read&&registry->retire(token)==rs::LinkRetirement::Quiesced,"reservation and lifecycle cleanly released");
}
static void retiring_entered_native_scope()
{
    rs::LinkLifetimeRegistry registry;int a{},b{},owner{};rs::LinkToken first{},second{};
    check(registry.register_constructed(&a,first)==rs::LinkRegistration::Registered&&registry.activate(&a)==rs::LinkAccess::Read,"native-retirement first lifecycle");
    check(registry.register_constructed(&b,second)==rs::LinkRegistration::Registered&&registry.activate(&b)==rs::LinkAccess::Read,"native-retirement second lifecycle");
    rs::NativeBorrowToken borrow{};rs::CanonicalReservation reservation{};
    check(registry.enter_native_offboard(&a,borrow)==rs::LinkAccess::Read,"actual native scope entered before retirement");
    check(registry.retire(first)==rs::LinkRetirement::Unproven,"destruction not granted while native scope entered");
    check(registry.bind_canonical(echo(second),42,191,&owner,reservation)==rs::CanonicalBind::Busy,"Closing in-flight scope still blocks canonical bind");
    check(registry.leave_native_offboard(borrow)==rs::LinkAccess::Read&&registry.retire(first)==rs::LinkRetirement::NeverRegistered,"actual leave drains Closing without a fake pointer read");
    check(registry.bind_canonical(echo(second),42,191,&owner,reservation)==rs::CanonicalBind::Bound,"bind succeeds only after actual scope drain");
    check(registry.release_canonical(reservation)==rs::LinkAccess::Read&&registry.retire(second)==rs::LinkRetirement::Quiesced,"native-retirement test cleanup");
}
int main()
{
    local_registry();scoped_concurrency();retiring_entered_native_scope();
    std::printf("{\"checks\":%u,\"failed\":%u,\"scope\":\"REAL_LINK_REGISTRY_PI_MUTEX_RAII_HOST_THREADS_MOCK_LINK_DATA\"}\n",checks,failed);
    return failed?1:0;
}
