"""Exercise the acquire body with HOST dependencies."""
from pathlib import Path
import hashlib,json,subprocess,sys
root=Path(__file__).resolve().parents[1]
runtime=root/'rfly_vendor_integration/px4_runtime'
out=root/'rfly_vendor_integration/application_integration/local_runtime_integration/acquisition_diagnostics'
source=runtime/'RegisteredCanonicalLocalModuleContext.cpp'
text=source.read_text()
body=text[text.index('ModuleAcquire RegisteredCanonicalLocalModuleContext::acquire('):text.index('bool RegisteredCanonicalLocalModuleContext::release_reservation(')]
header=(runtime/'RegisteredCanonicalLocalModuleContext.hpp').read_text()
evidence=header[header.index('enum class RegisteredLocalContextFault:'):header.index('struct RegisteredLocalContextDiagnostics {')]
harness='#include "'+str(runtime/'BoardSafetyEvidence.hpp')+'"\n'
harness+=r'''
#include <cstdio>
#include <cstdlib>
using namespace gpenmpc_rfly_px4;
void require(bool value){if(!value)std::abort();}
std::uint64_t hrt_absolute_time(){return 123456;}
int scenario{},safety_calls{},snapshots{},legacy_calls{},bind_calls{},releases{};
enum class ModuleAcquire {Ready,Unavailable,Rejected};
struct ModuleConfiguration {void *authority{};std::uint64_t telemetry_max_age_us=100000;};
struct SessionGuardEvidence {unsigned first_fault{};BoardSafetyEvidence raw_board{};};
'''
harness+=evidence
harness+=r'''
namespace gpenmpc_rfly_stream {
enum class CanonicalBind {Bound,Busy,Unavailable,Invalid,Unproven};
struct Registry {bool output{};bool ready(){return !(scenario==4&&!output);}} outputs{true},wire{false};
struct Reservation {int heartbeat{};};
struct Links {CanonicalBind bind_canonical(int,unsigned,unsigned,void *,Reservation &){
 ++bind_calls;return scenario==5?CanonicalBind::Busy:CanonicalBind::Bound;}} links;
Registry *shared_output_registry(){return scenario==3?nullptr:&outputs;}
Registry *canonical_local_wire_route_registry(){return &wire;}
Links *link_lifetime_registry(){return &links;}
}
bool legacy_trajectory_stopped_with_module_lock(){++legacy_calls;return true;}
struct Guard {bool evidence_snapshot(SessionGuardEvidence &out){
 ++snapshots;out.raw_board.identity.uid=22;return scenario!=8;}};
struct Prestream {bool start(int,std::uint64_t){return scenario!=6;}};
struct RegisteredCanonicalLocalModuleContext {
 struct Diagnostics {LocalAcquireEvidence acquire{};
 RegisteredLocalContextFault first_fault{RegisteredLocalContextFault::None};
 std::uint64_t first_fault_us{};bool acquire_aborted{},acquired{},reservation_bound{};} diagnostics_;
 struct Config {ModuleConfiguration module{};
 struct {unsigned source_system=255,source_component=190;} transport;} configuration_;
 Guard guard_;Prestream prestream_;int direct_{},echo_{};
 gpenmpc_rfly_stream::Registry *outputs_{},*wire_{};
 gpenmpc_rfly_stream::Links *links_{};gpenmpc_rfly_stream::Reservation reservation_{};
 bool fail(RegisteredLocalContextFault fault,std::uint64_t now){
  if(diagnostics_.first_fault==RegisteredLocalContextFault::None){
   diagnostics_.first_fault=fault;diagnostics_.first_fault_us=now;}return false;}
 bool session_and_physical(BoardSafetyEvidence &out){
  ++safety_calls;out.identity.uid=11;out.native_recovery_configuration=GuardFact::Pass;
  if(scenario==1)return fail(RegisteredLocalContextFault::Session,123456);
  if(scenario==2)return fail(RegisteredLocalContextFault::Physical,123456);
  return true;
 }
 bool release_reservation(){++releases;return true;}
 ModuleAcquire acquire(ModuleConfiguration &) noexcept;
};
'''
harness+=body
harness+=r'''
void reset(int s){scenario=s;safety_calls=snapshots=legacy_calls=bind_calls=releases=0;}
int main(){
 ModuleConfiguration result;
 {reset(0);RegisteredCanonicalLocalModuleContext c;
  require(c.acquire(result)==ModuleAcquire::Ready);
  require(c.diagnostics_.acquire.result==0&&c.diagnostics_.acquire.reason==0&&
   c.diagnostics_.acquire.observed.identity.uid==11&&c.diagnostics_.acquire.guard.raw_board.identity.uid==22&&
   safety_calls==1&&snapshots==1&&bind_calls==1&&releases==0);
  const auto saved=c.diagnostics_.acquire;
  require(c.acquire(result)==ModuleAcquire::Rejected);
  require(c.diagnostics_.acquire.result==saved.result&&c.diagnostics_.acquire.reason==saved.reason&&snapshots==1);
 }std::puts("ready_first_evidence_immutable_on_duplicate PASS");
 for(int s=1;s<=2;++s){reset(s);RegisteredCanonicalLocalModuleContext c;
  require(c.acquire(result)==ModuleAcquire::Unavailable);
  require(c.diagnostics_.acquire.reason==4&&c.diagnostics_.acquire.result==1&&
   unsigned(c.diagnostics_.acquire.first_context_fault)==unsigned(s+1)&&
   safety_calls==1&&snapshots==1&&bind_calls==0&&legacy_calls==0);
 }std::puts("session_vs_physical_original_output_and_snapshot_retained PASS");
 {reset(3);RegisteredCanonicalLocalModuleContext c;require(c.acquire(result)==ModuleAcquire::Unavailable);
  require(c.diagnostics_.acquire.reason==6&&c.diagnostics_.acquire.outputs_present==0&&
   c.diagnostics_.acquire.outputs_ready==255&&bind_calls==0);
 }std::puts("missing_registry_no_readiness_fabrication PASS");
 {reset(4);RegisteredCanonicalLocalModuleContext c;require(c.acquire(result)==ModuleAcquire::Unavailable);
  require(c.diagnostics_.acquire.reason==6&&c.diagnostics_.acquire.outputs_ready==1&&
   c.diagnostics_.acquire.wire_ready==0&&bind_calls==0);
 }std::puts("wire_not_ready_exact_branch PASS");
 {reset(5);RegisteredCanonicalLocalModuleContext c;require(c.acquire(result)==ModuleAcquire::Unavailable);
  require(c.diagnostics_.acquire.reason==7&&c.diagnostics_.acquire.bind_result==1&&bind_calls==1&&releases==1);
 }std::puts("bind_busy_one_attempt_cleanup_unchanged PASS");
 {reset(6);RegisteredCanonicalLocalModuleContext c;require(c.acquire(result)==ModuleAcquire::Rejected);
  require(c.diagnostics_.acquire.reason==8&&c.diagnostics_.acquire.result==2&&releases==1&&
   c.diagnostics_.acquire.first_context_fault==RegisteredLocalContextFault::Configuration);
 }std::puts("prestream_reject_original_cleanup_and_first_fault PASS");
 {reset(7);RegisteredCanonicalLocalModuleContext c;c.diagnostics_.first_fault=RegisteredLocalContextFault::Configuration;
  require(c.acquire(result)==ModuleAcquire::Rejected);
  require(c.diagnostics_.acquire.reason==1&&safety_calls==0&&snapshots==0);
 }std::puts("existing_fault_immediate_no_observation PASS");
 {reset(8);RegisteredCanonicalLocalModuleContext c;require(c.acquire(result)==ModuleAcquire::Ready);
  require(!c.diagnostics_.acquire.guard_snapshot_valid&&snapshots==1&&safety_calls==1);
 }std::puts("diagnostic_snapshot_failure_not_new_admission_gate PASS");
}
'''
cpp=out/'HOST_ACQUIRE_DIAGNOSTIC.cpp';exe=out/'HOST_ACQUIRE_DIAGNOSTIC'
with cpp.open('x') as stream:stream.write(harness)
compiled=subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',str(cpp),'-o',str(exe)],capture_output=True,text=True)
run=subprocess.run([str(exe)],capture_output=True,text=True) if compiled.returncode==0 else None
groups=run.stdout.splitlines() if run else []
receipt=dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest().upper(),
 compile_returncode=compiled.returncode,compile_stderr=compiled.stderr,run_returncode=run.returncode if run else None,
 case_groups=groups,case_group_count=len(groups),passed=bool(run and run.returncode==0 and len(groups)==8),
 scope='VERBATIM_ACQUIRE_BODY_HOST_STUB_DEPENDENCIES_DIAGNOSTIC_ONLY',
 limitation='HOST acquisition regression using mocked registry and physical-state inputs.',COM=0,hardware_actions=0)
with (out/'HOST_ACQUIRE_DIAGNOSTIC.json').open('x') as stream:json.dump(receipt,stream,indent=2);stream.write('\n')
print(json.dumps(receipt));sys.exit(0 if receipt['passed'] else 1)
