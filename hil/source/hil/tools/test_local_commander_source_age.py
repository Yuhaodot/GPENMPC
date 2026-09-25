"""HOST regression of RawGuard source-time logic."""
from pathlib import Path
import hashlib
import json
import re
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
runtime = root / 'rfly_vendor_integration/px4_runtime'
out = root / 'rfly_vendor_integration/application_integration/local_runtime_integration/commander_source_age'
source = runtime / 'Px4ReadOnlyGuard.cpp'
text = source.read_text()
tail = text[text.index('    vehicle_status_s status{};'):text.rindex('\n}\n\n} // namespace')]
helpers = text[text.index('void reason('):text.index('bool read_int32(')]
helpers += text[text.index('bool fresh('):text.index('bool read_usb_link(')]
ctor = text[text.index('Px4ReadOnlyGuard::Px4ReadOnlyGuard('):text.index('bool Px4ReadOnlyGuard::observe(')]
initializers = ctor[ctor.index('noexcept :') + len('noexcept :'):ctor.index('\n{')]
entry = (runtime / 'CanonicalLocalSessionEntry.cpp').read_text()
assert 'now - original.timestamp_sample > 5000' in entry
assert 'now - sensor.timestamp > 5000' in entry
owner = (runtime / 'CanonicalLocalApplicationOwner.cpp').read_text()
entry = (runtime / 'CanonicalLocalSessionEntry.cpp').read_text()
assert 'profile.module.commander_telemetry_max_age_us' in owner
assert 'm.commander_telemetry_max_age_us = 600000;' in entry
for name in ('Px4ReadOnlyGuard.hpp', 'Px4ExecutionSessionGuard.hpp'):
    assert 'frozen_commander_max_age_us = 0' in (runtime / name).read_text()

def fake_record(struct, variable):
    fields = set(re.findall(r'\b' + variable + r'\.(\w+)', tail))
    fields.discard('timestamp')
    constants = set(re.findall(struct + r'::(\w+)', tail))
    result = 'struct ' + struct + ' { std::uint64_t timestamp{};\n'
    result += ''.join('int ' + f + '{};\n' for f in sorted(fields))
    result += ''.join('static constexpr int ' + c + '=1;\n' for c in sorted(constants))
    return result + '};\n'

harness = '#include "' + str(runtime / 'BoardSafetyEvidence.hpp') + '"\n'
harness += r'''
#include <cstdio>
#include <cstdlib>
using namespace gpenmpc_rfly_px4;
std::uint64_t now_us=2000000;
std::uint64_t hrt_absolute_time() { return now_us; }
void require(bool b) { if(!b) std::abort(); }
'''
for struct, variable in [('vehicle_status_s','status'),('vehicle_control_mode_s','mode'),
                         ('offboard_control_mode_s','offboard'),('system_power_s','power')]:
    harness += fake_record(struct, variable)
harness += r'''
template<class T> struct Sub {
  T value{}; bool present=true;
  bool copy(T *out) { if(present) *out=value; return present; }
};
bool native_land_mode_shape(const vehicle_status_s &,const vehicle_control_mode_s &) { return true; }
bool disarmed_control_shape(const vehicle_status_s &,const vehicle_control_mode_s &) { return true; }
'''
harness += helpers
harness += '''
struct Guard {
  const std::uint64_t max_age_us_, commander_max_age_us_;
  Sub<vehicle_status_s> status_;
  Sub<vehicle_control_mode_s> mode_;
  Sub<offboard_control_mode_s> offboard_;
  Sub<system_power_s> power_;
  Guard(std::uint64_t frozen_telemetry_max_age_us,
        std::uint64_t frozen_commander_max_age_us=0) :
''' + initializers + r''' {}
  bool observe(BoardSafetyEvidence &e) {
    e={}; e.identity.uid=0x1122334455667788ULL;
    e.identity.system=e.identity.component=1;
    e.uid=e.mavlink_identity=e.usb_transport=e.hil_configuration=GuardFact::Pass;
'''
harness += tail + '\n  }\n};\n'
harness += r'''
void setup(Guard &g,std::uint64_t status_age=500000,std::uint64_t mode_age=500000,
           std::uint64_t offboard_age=1000,std::uint64_t power_age=1000) {
  g.status_.value.timestamp=now_us-status_age;
  g.status_.value.system_id=g.status_.value.component_id=1;
  g.mode_.value.timestamp=now_us-mode_age;
  g.offboard_.value.timestamp=now_us-offboard_age;
  g.power_.value.timestamp=now_us-power_age;
  g.power_.value.usb_connected=g.power_.value.usb_valid=1;
}
bool identity_live(const BoardSafetyEvidence &e) {
  return e.identity_valid_until_us>=now_us;
}
int main() {
  BoardSafetyEvidence e;
  { Guard old(100000); setup(old); require(!old.observe(e));
    require(!identity_live(e)); }
  { Guard selected(100000,600000); setup(selected);
    require(selected.observe(e) && identity_live(e));
    require(e.telemetry_fresh==GuardFact::Pass && e.original_valid_until_us>=now_us); }
  std::puts("500ms_commander_old_reject_local_accept PASS");
  { Guard g(100000,600000); setup(g,600000,600000); require(g.observe(e));
    require(e.identity_valid_until_us==now_us && e.original_valid_until_us==now_us &&
      e.telemetry_fresh==GuardFact::Pass); }
  std::puts("600ms_exact_fresh_and_expiry_boundary PASS");
  { Guard g(100000,600000); setup(g,601000,1000); require(!g.observe(e));
    require(!identity_live(e) && e.telemetry_fresh==GuardFact::Fail); }
  { Guard g(100000,600000); setup(g,1000,601000); require(g.observe(e));
    require(!identity_live(e) && e.telemetry_fresh==GuardFact::Fail); }
  std::puts("601ms_status_or_mode_rejected_by_session_expiry PASS");
  { Guard g(100000,600000); setup(g,500000,500000,100000); require(g.observe(e));
    require(e.telemetry_fresh==GuardFact::Pass && e.original_valid_until_us==now_us); }
  { Guard g(100000,600000); setup(g,500000,500000,100001); require(g.observe(e));
    require(e.telemetry_fresh==GuardFact::Fail && e.original_valid_until_us<now_us); }
  std::puts("offboard_100ms_boundary_unchanged PASS");
  { Guard g(100000,600000); setup(g,500000,500000,1000,100000); require(g.observe(e));
    require(e.usb_power_observed==GuardFact::Pass && identity_live(e)); }
  { Guard g(100000,600000); setup(g,500000,500000,1000,100001); require(g.observe(e));
    require(e.usb_power_observed!=GuardFact::Pass && !identity_live(e)); }
  std::puts("power_100ms_boundary_unchanged PASS");
  { Guard g(100000,600000); setup(g); g.status_.present=false;
    require(!g.observe(e) && e.identity_valid_until_us==0); }
  { Guard g(100000,600000); setup(g); g.status_.value.timestamp=now_us+1;
    require(!g.observe(e)); }
  std::puts("missing_or_future_status_no_admission PASS");
  { Guard g(100000,600000); setup(g); g.mode_.value.timestamp=now_us+1;
    require(g.observe(e) && e.mode_timestamp_us>e.observation_us &&
      e.telemetry_fresh==GuardFact::Fail); }
  std::puts("future_mode_retained_for_existing_session_timestamp_guard PASS");
  { Guard g(100000,600000); setup(g); g.offboard_.present=false;
    require(g.observe(e) && identity_live(e) && e.telemetry_fresh!=GuardFact::Pass); }
  std::puts("identity_only_missing_offboard_not_active_authority PASS");
  require(fresh(now_us-5000,now_us,5000));
  require(!fresh(now_us-5001,now_us,5000));
  require(!fresh(0,now_us,5000) && !fresh(now_us+1,now_us,5000));
  std::puts("unchanged_5ms_zero_future_boundary PASS");
}
'''
cpp=out/'HOST_COMMANDER_SOURCE_AGE.cpp'
exe=out/'HOST_COMMANDER_SOURCE_AGE'
with cpp.open('x') as stream:
    stream.write(harness)
compile_result=subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',str(cpp),'-o',str(exe)],text=True,capture_output=True)
run=subprocess.run([str(exe)],text=True,capture_output=True) if compile_result.returncode==0 else None
checks=run.stdout.splitlines() if run else []
receipt=dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest().upper(),
    compile_returncode=compile_result.returncode,compile_stderr=compile_result.stderr,
    run_returncode=run.returncode if run else None,case_groups=checks,
    case_group_count=len(checks),passed=bool(run and run.returncode==0 and len(checks)==9),
    scope='VERBATIM_RAWGUARD_TIME_BRANCH_HOST_STUB_TOPIC_VALUES',
    limitation='Mocked topic and control-shape inputs exercise the source-age checks.',
    hardware_actions=0,COM=0)
with (out/'HOST_COMMANDER_SOURCE_AGE.json').open('x') as stream:
    json.dump(receipt,stream,indent=2);stream.write('\n')
print(json.dumps(receipt))
sys.exit(0 if receipt['passed'] else 1)
