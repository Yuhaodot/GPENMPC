"""Offline acquisition regression; extract production code, never access board IO.

Use a distinct output directory to preserve prior acquisition evidence.
Clock, uORB copies, locked visitor and owner side effects are simulated.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--source', type=Path, default=root /
                    'rfly_vendor_integration/px4_runtime/CanonicalLocalSessionEntry.cpp')
parser.add_argument('--compiler', default=None)
args = parser.parse_args()
source, out = args.source.resolve(), args.output.resolve()
compiler = args.compiler or os.environ.get('GPENMPC_HOST_CXX') or shutil.which('clang++') or shutil.which('g++')
if not compiler or not Path(compiler).is_file():
    raise SystemExit('An existing host C++ compiler is required; no toolchain is installed.')

text = source.read_text(encoding='utf-8')
declarations = text[text.index('struct PrepareDiagnostics {'):text.index('void print_prepare_diagnostic(')]
acquire = text[text.index('bool acquire_prepare('):text.index('int print_post_release()')]
actual = text[text.index('bool prepare_locked('):text.index('bool acquire_prepare(')]
# Actual short-circuit ordering and the deadline check preceding registration.
predicates = actual[actual.index('  // Same predicates and first-false order;'):
                    actual.index('  profile = {};')]
assert actual.index('prepare_acquisition_in_time(now)') < actual.index('owner->prepare(')
assert actual.count('owner->prepare(') == 1
assert actual.count('owner->bind_inputs(') == 1
assert 'now - original.timestamp_sample > 5000' in predicates
assert 'now - sensor.timestamp > 5000' in predicates
assert 'prepare_acquisition_max_us = 20000;' in declarations
assert 'prepare_acquisition_max_attempts = 40;' in declarations
assert 'prepare_acquisition_sleep_us = 500;' in declarations

harness = r'''
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
int scenario=0, invalid=0;
unsigned checks=0, cases=0, visits=0, prepare_calls=0, bind_calls=0, phase_us=0;
std::uint64_t clock_us=100000;
bool in_visitor=false;
void require_at(bool pass, unsigned line) {
  ++checks;
  if (!pass) {
    std::fprintf(stderr,"FAIL scenario=%d invalid=%d phase=%u line=%u\n",
                 scenario,invalid,phase_us,line);
    std::exit(1);
  }
}
#define require(value) require_at((value),__LINE__)
enum class LocalApplicationState { Empty, Registered };
struct Observation { LocalApplicationState state{LocalApplicationState::Empty}; } observation;
struct Owner {
  LocalApplicationState state{LocalApplicationState::Empty};
  bool observe(Observation &o) {
    require(!in_visitor);
    o.state=state;
    return scenario!=22;
  }
} owned, *owner=&owned;
struct Mavlink {};
std::uint64_t hrt_absolute_time() { return clock_us; }
int px4_usleep(unsigned us) {
  require(!in_visitor); require(us==500);
  if(scenario==11) return -1;
  if(scenario==8) clock_us+=20000;
  else if(scenario==9) clock_us-=1;
  else clock_us+=us;
  return 0;
}
'''
harness += declarations
harness += r'''
bool prepare_locked(Mavlink &, void *) {
  auto &d=prepare_diagnostics;
  d.visitor_entered=d.ingress_valid=d.odom_copy=d.hil_copy=true;
  d.same_link=d.same_instance=d.same_channel=d.gyro_called=d.accel_called=true;
  if(scenario==13) clock_us+=20000;
  if(scenario==23) clock_us+=19750;
  d.stage=4; d.now_us=clock_us;
  d.odom_sample_us=clock_us-1000;
  d.hil_timestamp_us=clock_us-1000;
  if(scenario==1) d.hil_timestamp_us=clock_us-2951;
  if(scenario==1 || scenario==2 || scenario==3 || scenario==4 ||
     scenario==8 || scenario==9 || scenario==10 || scenario==11 ||
     scenario==16 || scenario==21 || scenario==23 || scenario==30)
    d.odom_sample_us=clock_us-10702;
  if(scenario==2 || scenario==14 || scenario==15 || scenario==16)
    d.hil_timestamp_us=clock_us-5001;
  if((scenario==1 || scenario==2 || scenario==14) && visits>1)
    d.odom_sample_us=d.hil_timestamp_us=clock_us-1000;
  if(scenario==3) d.same_link=false;
  if(scenario==4) d.ingress_valid=false;
  if(scenario==17) d.odom_sample_us=d.hil_timestamp_us=clock_us-5000;
  if((scenario==18 || scenario==20) && visits==1) d.odom_sample_us=clock_us-5001;
  if((scenario==19 || scenario==20) && visits==1) d.hil_timestamp_us=clock_us-5001;
  if(scenario==25) {
    const auto sample=clock_us-((clock_us-100000+phase_us)%10000);
    d.odom_sample_us=d.hil_timestamp_us=sample;
  }
  switch(invalid) {
    case 1: d.odom_sample_us=0; break;
    case 2: d.odom_sample_us=clock_us+1; break;
    case 3: d.hil_timestamp_us=0; break;
    case 4: d.hil_timestamp_us=clock_us+1; break;
    case 5: d.same_link=false; break;
    case 6: d.same_instance=false; break;
    case 7: d.same_channel=false; break;
    case 8: d.gyro_called=false; break;
    case 9: d.accel_called=false; break;
    case 10: d.odom_copy=false; break;
    case 11: d.hil_copy=false; break;
    case 12: d.ingress_valid=false; break;
    default: break;
  }
  if(!d.ingress_valid) return prepare_failed(3);
  if(!d.odom_copy) return prepare_failed(4);
  if(!d.hil_copy) return prepare_failed(5);
  const struct { std::uint64_t timestamp_sample; } original{d.odom_sample_us};
  const struct { std::uint64_t timestamp; } sensor{d.hil_timestamp_us};
  const auto now=d.now_us;
'''
harness += predicates
harness += r'''
  ++prepare_calls;
  d.owner_result=scenario==5 ? 1 : 0;
  if(scenario==5) return prepare_failed(17);
  ++bind_calls;
  d.bind_result=scenario==6 ? 1 : 0;
  if(scenario==6) return prepare_failed(20);
  owned.state=LocalApplicationState::Registered;
  return true;
}
bool gpenmpc_visit_locked_mavlink_device(const char *device,
    bool (*callback)(Mavlink &,void *),void *arg) {
  require(std::strcmp(device,"/dev/ttyACM0")==0);
  ++visits;
  if(scenario==7) return false;
  Mavlink m; in_visitor=true; const bool result=callback(m,arg);
  in_visitor=false;
  if(scenario==21) owned.state=LocalApplicationState::Registered;
  return result;
}
'''
harness += acquire
harness += r'''
void reset(int which) {
  ++cases;
  scenario=which; invalid=0; phase_us=0; clock_us=100000;
  visits=prepare_calls=bind_calls=0;
  owned.state=LocalApplicationState::Empty; observation={};
}
void fresh_registered(unsigned expected_visits) {
  require(acquire_prepare("/dev/ttyACM0"));
  require(visits==expected_visits && prepare_calls==1 && bind_calls==1);
  require(prepare_acquisition.sleeps+1==visits);
  require(prepare_acquisition.stop_reason==1 && prepare_diagnostics.first_reason==0);
  require(prepare_diagnostics.now_us>=prepare_diagnostics.odom_sample_us);
  require(prepare_diagnostics.now_us>=prepare_diagnostics.hil_timestamp_us);
  require(prepare_diagnostics.now_us-prepare_diagnostics.odom_sample_us<=5000);
  require(prepare_diagnostics.now_us-prepare_diagnostics.hil_timestamp_us<=5000);
  require(prepare_acquisition.elapsed_us<20000);
}
void immediate_rejection() {
  require(!acquire_prepare("/dev/ttyACM0"));
  require(visits==1 && prepare_calls==0 && bind_calls==0 && prepare_acquisition.sleeps==0);
  require(prepare_acquisition.stop_reason==3);
}
int main() {
  reset(0); fresh_registered(1);
  std::puts("fresh_single_prepare_and_bind PASS");
  for(int s: {1,2,14}) {
    reset(s); fresh_registered(2);
    require(prepare_acquisition.first_stale_present);
    const auto &first=prepare_acquisition.first_stale;
    require(first.first_reason==(s==14 ? 11u : 8u));
    require(first.owner_result==255 && first.bind_result==255);
    require(first.now_us==100000 && prepare_acquisition.elapsed_us==500);
    require(first.now_us-first.odom_sample_us==(s==14 ? 1000u : 10702u));
    require(first.now_us-first.hil_timestamp_us==(s==1 ? 2951u : 5001u));
  }
  std::puts("odom_hil_and_both_stale_reacquire_original_first_atom PASS");
  for(int s: {3,4}) { reset(s); immediate_rejection(); }
  std::puts("source_fault_behind_stale_short_circuit_no_resample PASS");
  for(int s: {5,6}) {
    reset(s); require(!acquire_prepare("/dev/ttyACM0"));
    require(visits==1 && prepare_calls==1 && prepare_acquisition.sleeps==0);
    require(bind_calls==(s==5 ? 0u : 1u));
    require(prepare_diagnostics.first_reason==(s==5 ? 17u : 20u));
  }
  std::puts("owner_prepare_or_bind_failure_never_retried PASS");
  reset(7); require(!acquire_prepare("/dev/ttyACM0"));
  require(visits==1 && prepare_calls==0 && prepare_acquisition.stop_reason==2);
  std::puts("missing_visitor_immediate PASS");
  reset(8); require(!acquire_prepare("/dev/ttyACM0"));
  require(visits==1 && prepare_calls==0 && prepare_acquisition.stop_reason==5);
  std::puts("sleep_overshoot_no_late_registration PASS");
  reset(9); require(!acquire_prepare("/dev/ttyACM0"));
  require(visits==1 && prepare_calls==0 && prepare_acquisition.stop_reason==7);
  std::puts("clock_regression_no_rebase PASS");
  for(int s: {10,15,16}) {
    reset(s); require(!acquire_prepare("/dev/ttyACM0"));
    require(visits==40 && prepare_calls==0 && bind_calls==0 && prepare_acquisition.sleeps==39);
    require(prepare_acquisition.stop_reason==6 && prepare_acquisition.elapsed_us==19500);
  }
  std::puts("persistent_odom_hil_and_both_stale_bounded_without_registration PASS");
  reset(11); require(!acquire_prepare("/dev/ttyACM0"));
  require(visits==1 && prepare_calls==0 && prepare_acquisition.stop_reason==8);
  std::puts("sleep_error_immediate PASS");
  reset(12); owned.state=LocalApplicationState::Registered;
  require(!acquire_prepare("/dev/ttyACM0"));
  require(visits==0 && prepare_calls==0 && prepare_acquisition.stop_reason==4);
  std::puts("nonempty_owner_no_visit PASS");
  reset(13); require(!acquire_prepare("/dev/ttyACM0"));
  require(visits==1 && prepare_calls==0 && prepare_acquisition.stop_reason==5);
  require(prepare_diagnostics.first_reason==21 && prepare_acquisition.elapsed_us==20000);
  std::puts("exact_twenty_ms_deadline_before_prepare PASS");
  reset(17); fresh_registered(1);
  for(int s: {18,19,20}) { reset(s); fresh_registered(2); }
  std::puts("five_ms_inclusive_boundary_and_5001us_reacquire_not_accept PASS");
  reset(21); require(!acquire_prepare("/dev/ttyACM0"));
  require(visits==1 && prepare_calls==0 && prepare_acquisition.sleeps==0);
  require(prepare_acquisition.stop_reason==4);
  reset(22); require(!acquire_prepare("/dev/ttyACM0"));
  require(visits==0 && prepare_calls==0 && prepare_acquisition.stop_reason==4);
  std::puts("owner_changes_or_observation_failure_no_reacquire PASS");
  reset(23); require(!acquire_prepare("/dev/ttyACM0"));
  require(visits==1 && prepare_calls==0 && prepare_acquisition.sleeps==0);
  require(prepare_acquisition.stop_reason==5 && prepare_acquisition.elapsed_us==19750);
  std::puts("insufficient_sleep_budget_no_extra_attempt PASS");
  for(int s: {0,14,30}) {
    for(int bad=1;bad<=12;++bad) {
      reset(s); invalid=bad; immediate_rejection();
    }
  }
  std::puts("zero_future_copy_ingress_identity_gyro_accel_faults_never_retried PASS");
  for(unsigned p=0;p<10000;++p) {
    reset(25); phase_us=p;
    const unsigned expected=p<=5000 ? 1u : 1u+(10000-p+499)/500;
    fresh_registered(expected);
  }
  std::puts("synthetic_ten_ms_synchronous_inputs_all_integer_phases PASS");
  reset(0); fresh_registered(1);
  PrepareDiagnostics d=prepare_diagnostics;
  d.first_reason=8; d.odom_sample_us=d.now_us-5001;
  require(!prepare_stale_only(d));
  d.owner_result=d.bind_result=255; require(prepare_stale_only(d));
  d.bind_result=0; require(!prepare_stale_only(d));
  d.bind_result=255; d.owner_result=1; require(!prepare_stale_only(d));
  d.owner_result=255; d.odom_sample_us=d.now_us-5000;
  require(!prepare_stale_only(d));
  d.first_reason=11; require(!prepare_stale_only(d));
  d.hil_timestamp_us=d.now_us-5001; require(prepare_stale_only(d));
  d.first_reason=17; require(!prepare_stale_only(d));
  std::puts("retry_predicate_requires_staleness_and_unattempted_owner PASS");
  std::printf("SUMMARY cases=%u checks=%u\n",cases,checks);
}
'''
out.mkdir(parents=True, exist_ok=False)
cpp = out / 'HOST_PREPARE_ACQUISITION.cpp'
exe = out / ('HOST_PREPARE_ACQUISITION.exe' if os.name == 'nt' else 'HOST_PREPARE_ACQUISITION')
cpp.write_text(harness, encoding='utf-8')
command = [str(compiler), '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror']
if os.name == 'nt':
    command.append('-static')
command += [str(cpp), '-o', str(exe)]
compiled = subprocess.run(command, text=True, capture_output=True)
run = subprocess.run([str(exe)], text=True, capture_output=True) if compiled.returncode == 0 else None
lines = run.stdout.splitlines() if run else []
groups = [line for line in lines if line.endswith(' PASS')]
sha = lambda data: hashlib.sha256(data).hexdigest().upper()
receipt = dict(source=str(source), source_sha256=sha(source.read_bytes()),
    extracted_declarations_sha256=sha(declarations.encode()),
    extracted_acquire_sha256=sha(acquire.encode()),
    extracted_predicates_sha256=sha(predicates.encode()), fixture_sha256=sha(cpp.read_bytes()),
    command=command, compile_returncode=compiled.returncode, compile_stderr=compiled.stderr,
    run_returncode=run.returncode if run else None, run_stderr=run.stderr if run else None,
    stdout=lines, case_groups=groups, case_group_count=len(groups),
    passed=bool(run and run.returncode == 0 and len(groups)==17),
    scope='VERBATIM_ACQUISITION_AND_FRESHNESS_PREDICATES_HOST_STUB_CLOCK_VISITOR_OWNER',
    limitation='HOST acquisition regression with mocked topic timing and registration.',
    unchanged_thresholds=dict(source_age_us=5000, acquisition_us=20000, attempts=40, sleep_us=500),
    hardware_actions=0, firmware_compiles=0, firmware_uploads=0, COM=0)
(out / 'HOST_PREPARE_ACQUISITION.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8')
print(json.dumps(receipt))
sys.exit(0 if receipt['passed'] else 1)
