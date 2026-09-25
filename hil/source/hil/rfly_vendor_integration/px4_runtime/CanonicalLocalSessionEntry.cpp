#include "mavlink_main.h" // Must establish real send macros before dialect headers.
#include "../local_application_profile/CanonicalLocalDeploymentProfile.hpp"
#include "CanonicalLocalApplicationOwner.hpp"
#include "ConfiguredBoardIdentity.hpp"
#include "CanonicalLocalTaskInputOwner.hpp"
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <uORB/topics/gpenmpc_original_hil_receipt.h>

using GPENMPCLockedMavlinkVisitor = bool (*)(Mavlink &, void *);
extern bool gpenmpc_visit_locked_mavlink_device(const char *,
                                               GPENMPCLockedMavlinkVisitor,
                                               void *) noexcept;
namespace {
using namespace gpenmpc_rfly_px4;
using namespace gpenmpc_rfly_local_px4;
namespace frozen = gpenmpc_local_profile;
CanonicalLocalApplicationOwner *owner{};
CanonicalLocalTaskInputOwner *inputs{};
InheritingMutex entry_mutex{};
// Persistent, serialized observation/profile storage, never a multi-KiB NSH
// stack.
LocalApplicationObservation observation{};
LocalTaskInputDiagnostics retained_inputs{};
RegisteredLocalContextConfiguration profile{};
LocalTaskInputConfiguration task_profile{};
HostSessionChallenge challenge{};
SessionEcho echo{};
gpenmpc_portable::Array<double, 3> requested_origin{};
std::uint8_t host_system{}, host_component{}, requested_leg{};
bool requested_operator_reference{};
// SHA256 of CanonicalOperatorReference.hpp.
constexpr unsigned char operator_reference_sha[32]={
  0x78,0xBA,0x9A,0x5E,0x29,0xF8,0x25,0xD4,0xA5,0x94,0xA1,0xD9,0x1E,0xDC,0x90,0x12,0xE0,0xBA,0x88,0xA1,0x6E,0x52,0xD0,0x6C,0x8E,0x86,0xB6,0xA7,0x97,0x24,0x6E,0xA7};
LocalApplicationResult preparation{LocalApplicationResult::Unavailable};
// Preparation diagnostics: reason zero means no rejection was recorded;
// owner/bind result 255 means the call was not reached.
struct PrepareDiagnostics {
  unsigned stage{}, first_reason{};
  std::uint64_t entry_hrt_us{}, now_us{}, odom_timestamp_us{},
      odom_sample_us{}, hil_timestamp_us{};
  int actual_instance{-1}, sensor_instance{-1}, actual_channel{}, sensor_channel{};
  unsigned owner_result{255}, bind_result{255};
  bool visitor_entered{}, ingress_valid{}, odom_copy{}, hil_copy{},
      same_link{}, same_instance{}, same_channel{}, gyro_called{}, accel_called{};
} prepare_diagnostics{};
// Startup sample-acquisition bounds.
constexpr std::uint64_t prepare_acquisition_max_us = 20000;
constexpr unsigned prepare_acquisition_max_attempts = 40;
constexpr unsigned prepare_acquisition_sleep_us = 500;
struct PrepareAcquisition {
  std::uint64_t started_us{}, last_clock_us{}, elapsed_us{};
  unsigned attempts{}, sleeps{}, stop_reason{};
  bool first_stale_present{};
  PrepareDiagnostics first_stale{};
} prepare_acquisition{};

bool prepare_acquisition_in_time(std::uint64_t now) noexcept {
  auto &a = prepare_acquisition;
  if (now < a.last_clock_us || now < a.started_us) {
    a.stop_reason = 7; // Clock regression; never rebase the acquisition window.
    return false;
  }
  a.last_clock_us = now;
  a.elapsed_us = now - a.started_us;
  if (a.elapsed_us >= prepare_acquisition_max_us) {
    a.stop_reason = 5;
    return false;
  }
  return true;
}

bool prepare_stale_only(const PrepareDiagnostics &d) noexcept {
  // Either input may age while prepare is dispatched. Reacquire only before
  // registration; prepare_locked still requires both original ages <= 5000.
  // Reason 8 short-circuits later checks, so recheck every copied source fact.
  return (d.first_reason == 8 || d.first_reason == 11) &&
         d.visitor_entered && d.ingress_valid &&
         d.odom_copy && d.hil_copy && d.owner_result == 255 &&
         d.bind_result == 255 && d.odom_sample_us != 0 &&
         d.odom_sample_us <= d.now_us &&
         d.hil_timestamp_us != 0 && d.hil_timestamp_us <= d.now_us &&
         (d.now_us - d.odom_sample_us > 5000 ||
          d.now_us - d.hil_timestamp_us > 5000) && d.same_link &&
         d.same_instance && d.same_channel && d.gyro_called && d.accel_called;
}

bool prepare_failed(unsigned reason) noexcept {
  if (!prepare_diagnostics.first_reason)
    prepare_diagnostics.first_reason = reason;
  return false;
}
void print_prepare_diagnostic(const char *label,
                              const PrepareDiagnostics &d) noexcept {
  std::printf(
      "%s stage=%u reason=%u visitor=%u "
      "entry_hrt_us=%llu now_us=%llu odom_pub_us=%llu odom_sample_us=%llu "
      "hil_us=%llu node=%u odom_copy=%u hil_copy=%u same_link=%u "
      "same_instance=%u same_channel=%u gyro=%u accel=%u "
      "actual_instance=%d receipt_instance=%d actual_channel=%d "
      "receipt_channel=%d owner_result=%u bind_result=%u diagnostic_only=1\n",
      label, d.stage, d.first_reason, unsigned(d.visitor_entered),
      static_cast<unsigned long long>(d.entry_hrt_us),
      static_cast<unsigned long long>(d.now_us),
      static_cast<unsigned long long>(d.odom_timestamp_us),
      static_cast<unsigned long long>(d.odom_sample_us),
      static_cast<unsigned long long>(d.hil_timestamp_us),
      unsigned(d.ingress_valid), unsigned(d.odom_copy), unsigned(d.hil_copy),
      unsigned(d.same_link), unsigned(d.same_instance), unsigned(d.same_channel),
      unsigned(d.gyro_called), unsigned(d.accel_called), d.actual_instance,
      d.sensor_instance, d.actual_channel, d.sensor_channel, d.owner_result,
      d.bind_result);
}

bool digit(char c, unsigned &v) noexcept {
  if (c >= '0' && c <= '9') {
    v = unsigned(c - '0');
    return true;
  }
  if (c >= 'a' && c <= 'f') {
    v = unsigned(c - 'a' + 10);
    return true;
  }
  if (c >= 'A' && c <= 'F') {
    v = unsigned(c - 'A' + 10);
    return true;
  }
  return false;
}
bool hex64(const char *t, std::uint64_t &out) noexcept {
  out = 0;
  for (unsigned k = 0; k < 16; ++k) {
    unsigned v{};
    if (!digit(t[k], v))
      return false;
    out = (out << 4) | v;
  }
  return true;
}
bool digest(const char *t, SessionDigest &out) noexcept {
  if (!t || std::strlen(t) != 64)
    return false;
  out = {};
  for (unsigned k = 0; k < 64; ++k) {
    unsigned v{};
    if (!digit(t[k], v))
      return false;
    out[k / 8] = (out[k / 8] << 4) | v;
  }
  return nonzero_session_digest(out);
}
bool byte(const char *t, std::uint8_t &out) noexcept {
  if (!t || !*t)
    return false;
  char *end{};
  errno = 0;
  const long v = std::strtol(t, &end, 10);
  if (errno || !end || *end || v < 1 || v > 255)
    return false;
  out = static_cast<std::uint8_t>(v);
  return true;
}
bool real(const char *t, double &out) noexcept {
  if (!t || !*t)
    return false;
  char *end{};
  errno = 0;
  out = std::strtod(t, &end);
  return !errno && end && !*end && std::isfinite(out);
}
void print_digest(const SessionDigest &d) noexcept {
  for (auto v : d)
    std::printf("%08lX", static_cast<unsigned long>(v));
}
void print_echo() noexcept {
  const auto &e = observation.echo;
  std::printf(
      "RFLY_LOCAL_SESSION state=%u challenge=%016llX%016llX uid=%llu system=%u "
      "component=%u registration_hrt_us=%llu session_generation=%llu "
      "link_generation=%llu semantics=%u config_sha=",
      unsigned(observation.state),
      static_cast<unsigned long long>(e.host_challenge.high),
      static_cast<unsigned long long>(e.host_challenge.low),
      static_cast<unsigned long long>(e.observed_identity.uid),
      unsigned(e.observed_identity.system),
      unsigned(e.observed_identity.component),
      static_cast<unsigned long long>(e.board_registration_hrt_us),
      static_cast<unsigned long long>(e.process_session_generation),
      static_cast<unsigned long long>(e.link.generation),
      unsigned(e.identity_semantics));
  print_digest(e.configuration_sha256);
  std::printf(" session_sha=");
  print_digest(execution_session_digest(e));
  std::printf(" task_sha=");
  print_digest(gpenmpc_full_consumption::detail::words(frozen::task_sha256));
  std::printf(" leg=%u registered=%u echo_confirmed=%u declared_isolation=%u "
              "declaration_is_sensor_proof=0 session_fault=%u "
              "start_requests=%llu stop_requests=%llu\n",
              unsigned(requested_leg), unsigned(observation.session.registered),
              unsigned(observation.session.echo_confirmed),
              unsigned(observation.session.human_declaration_bound),
              unsigned(observation.session.first_fault),
              static_cast<unsigned long long>(observation.start_requests),
              static_cast<unsigned long long>(observation.stop_requests));
}
void print_raw_guard(const char *label,const BoardSafetyEvidence &r) noexcept {
  // This is the exact saved register/observe snapshot, including failures.
  // Do not resample and accidentally replace the first failed observation.
  std::printf(
      "%s reason=%u first_bad_param=%s uid=%llu system=%u "
      "component=%u uid_fact=%u mavlink_fact=%u usb=%u hil=%u "
      "observation_us=%llu status_us=%llu mode_us=%llu offboard_us=%llu "
      "power_us=%llu identity_expiry_us=%llu original_expiry_us=%llu "
      "telemetry_fresh=%u native_land=%u disarmed_control=%u external_isolation=%u "
      "active_direct=%u native_controllers_disabled=%u boot_fact=%u "
      "native_recovery=%u ra_mode=%ld obl_action=%ld "
      "pwm_zero=%u checked_pwm=%u pwm_stopped=%u io_stopped=%u dshot_stopped=%u "
      "usb_power=%u usb_connected=%u usb_valid=%u brick_valid=%u servo_valid=%u "
      "commander_age_us=600000 offboard_power_age_us=100000 "
      "retained_snapshot=1 diagnostic_only=1\n",
      label, unsigned(r.reason), r.first_bad_parameter,
      static_cast<unsigned long long>(r.identity.uid), unsigned(r.identity.system),
      unsigned(r.identity.component), unsigned(r.uid), unsigned(r.mavlink_identity),
      unsigned(r.usb_transport), unsigned(r.hil_configuration),
      static_cast<unsigned long long>(r.observation_us),
      static_cast<unsigned long long>(r.status_timestamp_us),
      static_cast<unsigned long long>(r.mode_timestamp_us),
      static_cast<unsigned long long>(r.offboard_timestamp_us),
      static_cast<unsigned long long>(r.power_timestamp_us),
      static_cast<unsigned long long>(r.identity_valid_until_us),
      static_cast<unsigned long long>(r.original_valid_until_us),
      unsigned(r.telemetry_fresh), unsigned(r.native_land_mode),
      unsigned(r.disarmed_control), unsigned(r.external_physical_isolation),
      unsigned(r.active_direct_mode), unsigned(r.native_controllers_disabled),
      unsigned(r.boot_session), unsigned(r.native_recovery_configuration),
      static_cast<long>(r.raw_ra_ctrl_mode), static_cast<long>(r.raw_com_obl_rc_act),
      unsigned(r.pwm_functions_zero), unsigned(r.checked_pwm_functions),
      unsigned(r.pwm_out_stopped), unsigned(r.io_driver_stopped),
      unsigned(r.dshot_stopped), unsigned(r.usb_power_observed),
      unsigned(r.raw_usb_connected), unsigned(r.raw_usb_valid),
      unsigned(r.raw_brick_valid), unsigned(r.raw_servo_valid));
}
struct LocalStartPrint {
  LocalAcquireEvidence acquire{};
  int returned{};
  bool task_created{};
};
// Only start commands own this fixed snapshot. Keep its ownership through
// post-entry-unlock printing so another invocation cannot overwrite it. It is
// never an invocation-local allocation on the 2048-byte NSH command stack.
InheritingMutex start_print_mutex{};
LocalStartPrint retained_start_print{};
void print_start_evidence(const LocalStartPrint &s) noexcept {
  const auto &a=s.acquire;
  std::printf("RFLY_LOCAL_START acquire_attempted=%u acquire_result=%u reason=%u "
              "context_fault=%u context_fault_us=%llu outputs=%u outputs_ready=%u "
              "wire=%u wire_ready=%u links=%u bind_result=%u legacy_stopped=%u "
              "guard_snapshot_valid=%u session_fault=%u session_fault_us=%llu "
              "registered=%u echo_confirmed=%u declaration=%u start_return=%d "
              "task_created=%u retained_first_acquire=1 diagnostic_only=1\n",
              unsigned(a.attempted),a.result,a.reason,unsigned(a.first_context_fault),
              static_cast<unsigned long long>(a.first_context_fault_us),
              a.outputs_present,a.outputs_ready,a.wire_present,a.wire_ready,
              a.links_present,a.bind_result,a.legacy_stopped,
              unsigned(a.guard_snapshot_valid),unsigned(a.guard.first_fault),
              static_cast<unsigned long long>(a.guard.first_fault_hrt_us),
              unsigned(a.guard.registered),unsigned(a.guard.echo_confirmed),
              unsigned(a.guard.human_declaration_bound),s.returned,unsigned(s.task_created));
  print_raw_guard("RFLY_LOCAL_ACQUIRE_OBSERVED",a.observed);
  print_raw_guard("RFLY_LOCAL_ACQUIRE_GUARD_SNAPSHOT",a.guard.raw_board);
}
bool prepare_locked(Mavlink &actual, void *) noexcept {
  auto &d = prepare_diagnostics;
  d.visitor_entered = true;
  d.stage = 1;
  const int instance = actual.get_instance_id();
  d.actual_instance = instance;
  if (instance < 0 || instance >= ORB_MULTI_MAX_INSTANCES)
    return prepare_failed(2); // Actual MAVLink instance out of range.
  // The actual receiver must have advertised the empty node first. A node
  // existence check consumes/publishes no ingress sample and grants no data.
  uORB::Subscription ingress_node{ORB_ID(gpenmpc_full_inner_ingress)};
  d.stage = 2;
  d.ingress_valid = ingress_node.valid();
  if (!d.ingress_valid)
    return prepare_failed(3);
  uORB::Subscription odometry{ORB_ID(vehicle_odometry)},
      hil{ORB_ID(gpenmpc_original_hil_receipt)};
  vehicle_odometry_s original{};
  gpenmpc_original_hil_receipt_s sensor{};
  d.stage = 3;
  d.odom_copy = odometry.copy(&original);
  d.odom_timestamp_us = original.timestamp;
  d.odom_sample_us = original.timestamp_sample;
  if (!d.odom_copy)
    return prepare_failed(4); // Preserve the original short-circuit order.
  d.hil_copy = hil.copy(&sensor);
  d.hil_timestamp_us = sensor.timestamp;
  if (!d.hil_copy)
    return prepare_failed(5);
  const auto now = hrt_absolute_time();
  d.stage = 4;
  d.now_us = now;
  d.actual_channel = actual.get_channel();
  d.sensor_instance = sensor.receiver_instance;
  d.sensor_channel = sensor.channel;
  d.same_link = sensor.link_address == reinterpret_cast<std::uintptr_t>(&actual);
  d.same_instance = sensor.receiver_instance == instance;
  d.same_channel = sensor.channel == d.actual_channel;
  d.gyro_called = sensor.gyro_update_called;
  d.accel_called = sensor.accel_update_called;
  // Same predicates and first-false order; no freshness relaxation.
  if (!original.timestamp_sample) return prepare_failed(6);
  if (original.timestamp_sample > now) return prepare_failed(7);
  if (now - original.timestamp_sample > 5000) return prepare_failed(8);
  if (!sensor.timestamp) return prepare_failed(9);
  if (sensor.timestamp > now) return prepare_failed(10);
  if (now - sensor.timestamp > 5000) return prepare_failed(11);
  if (!d.same_link) return prepare_failed(12);
  if (!d.same_instance) return prepare_failed(13);
  if (!d.same_channel) return prepare_failed(14);
  if (!d.gyro_called) return prepare_failed(15);
  if (!d.accel_called) return prepare_failed(16);
  // Register once after accepting a sample within the acquisition bound.
  if (!prepare_acquisition_in_time(now)) return prepare_failed(21);
  profile = {};
  auto &m = profile.module;
  auto &e = m.execution;
  auto &n = e.numerical;
  e.operator_reference = requested_operator_reference;
  m.source.vehicle_odometry_topic = ORB_ID(vehicle_odometry);
  m.source.instance = 0;
  m.source.initial_reset_counter = original.reset_counter;
  m.source.coordinates = gpenmpc_odometry::Coordinates::ExplicitTranslatedNed;
  m.source.task_origin_ned_m = requested_origin;
  m.source.sample_max_age_us = 50000;
  // Command age budget includes the 300-ms outer cadence, 280-ms solve budget
  // and transport/control margin. The original source timestamp is retained.
  e.limits = {50000, 400000, 700000, 4000,
              gpenmpc_consumption::PublicationPath::DirectCanonicalMotors};
  gpenmpc_full_inner_identity(&e.build);
  n.abi_version = GPENMPC_FULL_INNER_ABI_VERSION;
  n.leg_index = requested_leg;
  std::memcpy(n.task_sha256, frozen::task_sha256, 32);
  const auto*reference_sha=requested_operator_reference?operator_reference_sha:frozen::task_sha256;
  std::memcpy(n.reference_asset_sha256, reference_sha, 32);
  std::memcpy(n.configuration_sha256, frozen::configuration_sha256, 32);
  n.jerk_limit_mps3 = frozen::reference_jerk_limit;
  // Exact original new-leg reference/outer initializer: phase acceleration=0
  // and outer_i=0.
  m.telemetry_max_age_us = 100000;
  m.commander_telemetry_max_age_us = 600000;
  m.poll_period_us = 1000;
  m.task_priority = SCHED_PRIORITY_ATTITUDE_CONTROL;
  m.task_stack_bytes = 8192;
  profile.transport = {
      host_system, host_component, 1, 1, static_cast<std::uint8_t>(instance),
      200000};
  // Transport drain allowance; numerical freshness is checked separately.
  // Pending GP is withdrawn at the next control opportunity if unavailable.
  profile.ingress_topic_instance = 0;
  profile.native_land_tail_max_us = 30000000;
  profile.gp_request_transport_max_age_us =
      profile.snapshot_transport_max_age_us = 50000;
  profile.original_source.expected_channel = actual.get_channel();
  profile.original_source.expected_noui_system = sensor.system_id;
  profile.original_source.expected_noui_component = sensor.component_id;
  auto &p = profile.phase;
  p.configuration_sha256 =
      gpenmpc_full_consumption::detail::words(frozen::configuration_sha256);
  std::memcpy(p.reference_asset_sha256, reference_sha, 32);
  p.leg_index = requested_leg;
  p.duration_s = frozen::leg_duration_s[requested_leg - 1];
  p.rate_min = frozen::phase_rate_min;
  p.rate_max = frozen::phase_rate_max;
  auto &c = profile.cycle;
  c.context.task_sha256 = gpenmpc_full_consumption::detail::words(frozen::task_sha256);
  c.context.reference_asset_sha256 = gpenmpc_full_consumption::detail::words(reference_sha);
  c.context.configuration_sha256 = p.configuration_sha256;
  c.context.explicit_leg = requested_leg;
  c.context.step_kind = gpenmpc_local_input::StepKind::FirstOfExplicitLeg;
  c.initial_interval.configuration_sha256 = p.configuration_sha256;
  c.initial_interval.original_configuration_receipt_sha256 =
      gpenmpc_full_consumption::detail::words(
          frozen::configuration_binding_sha256);
  c.initial_interval.leg = requested_leg;
  c.initial_interval.configured_dt_s = frozen::initial_dt_s;
  c.reference_max_age_us = 400000;
  c.runtime_state_only = true;
  // Select the integrated outer/GP numerical path for autonomous operation.
  c.component_initialization = requested_operator_reference;
  c.operator_reference = requested_operator_reference;
  c.context.allow_valid_held_inputs = true;
  c.context.operator_reference = requested_operator_reference;
  d.stage = 5;
  preparation = owner->prepare(actual, {gpenmpc_board_configuration::expected_uid, 1, 1},
                               challenge, profile, echo);
  d.owner_result = unsigned(preparation);
  if (preparation != LocalApplicationResult::Ready)
    return prepare_failed(17);
  task_profile = {};
  auto &w = task_profile.window;
  w.expected.identity = {
      echo.observed_identity.uid, echo.process_session_generation,
      echo.observed_identity.system, echo.observed_identity.component};
  w.expected.execution_session_sha256 = execution_session_digest(echo);
  w.expected.configuration_sha256 = p.configuration_sha256;
  w.expected.task_sha256 = c.context.task_sha256;
  w.expected.reference_asset_sha256 = c.context.reference_asset_sha256;
  w.expected.leg_index = requested_leg;
  w.max_assembly_us = 1000000;
  task_profile.input_assembly_max_us = 50000;
  task_profile.runtime_state_only = true;
  task_profile.component_initialization = requested_operator_reference;
  task_profile.operator_reference = requested_operator_reference;
  task_profile.outer_max_age_us = 700000;
  d.stage = 6;
  inputs = new CanonicalLocalTaskInputOwner(task_profile);
  if (!inputs)
    return prepare_failed(18);
  if (inputs->diagnostics().first_fault != LocalTaskInputFault::None)
    return prepare_failed(19);
  d.stage = 7;
  const auto bound = owner->bind_inputs(inputs->port());
  d.bind_result = unsigned(bound);
  return bound == LocalApplicationResult::Ready ? true : prepare_failed(20);
}
bool acquire_prepare(const char *device) noexcept {
  auto &a = prepare_acquisition;
  a = {};
  a.started_us = a.last_clock_us = hrt_absolute_time();
  prepare_diagnostics = {};
  prepare_diagnostics.entry_hrt_us = a.started_us;
  for (;;) {
    if (!owner->observe(observation) ||
        observation.state != LocalApplicationState::Empty) {
      a.stop_reason = 4;
      return false;
    }
    if (!prepare_acquisition_in_time(hrt_absolute_time())) return false;
    if (a.attempts >= prepare_acquisition_max_attempts) {
      a.stop_reason = 6;
      return false;
    }
    prepare_diagnostics = {};
    prepare_diagnostics.entry_hrt_us = a.started_us;
    ++a.attempts;
    const bool ok =
        gpenmpc_visit_locked_mavlink_device(device, prepare_locked, nullptr);
    // The visitor's lifecycle LockGuard has returned: no sleep or observe
    // below holds the global MAVLink mutex, and no Mavlink pointer is retained.
    if (ok) {
      a.stop_reason = 1;
      return true;
    }
    if (!prepare_diagnostics.visitor_entered) {
      (void)prepare_failed(1);
      a.stop_reason = 2;
      return false;
    }
    if ((prepare_diagnostics.first_reason == 8 ||
         prepare_diagnostics.first_reason == 11) && !a.first_stale_present) {
      a.first_stale = prepare_diagnostics;
      a.first_stale_present = true;
    }
    if (!prepare_stale_only(prepare_diagnostics)) {
      if (!a.stop_reason) a.stop_reason = 3;
      return false;
    }
    if (!owner->observe(observation) ||
        observation.state != LocalApplicationState::Empty) {
      a.stop_reason = 4;
      return false;
    }
    if (!prepare_acquisition_in_time(hrt_absolute_time())) return false;
    if (a.attempts >= prepare_acquisition_max_attempts) {
      a.stop_reason = 6;
      return false;
    }
    if (prepare_acquisition_max_us - a.elapsed_us <=
        prepare_acquisition_sleep_us) {
      a.stop_reason = 5;
      return false;
    }
    ++a.sleeps;
    if (px4_usleep(prepare_acquisition_sleep_us) != 0) {
      a.stop_reason = 8;
      return false;
    }
  }
}
int print_post_release() noexcept {
  if (owner || inputs || observation.state != LocalApplicationState::Retired)
    return -EBUSY;
  const auto &f = observation.failed_feedback;
  const auto &l = observation.late_feedback;
  std::printf(
      "RFLY_LOCAL_RETAINED context_fault=%u context_first_us=%llu "
      "inputs_fault=%u inputs_first_us=%llu failed_present=%u "
      "failed_phase_installed=%u failed_published=%u late_published=%u "
      "late_installed=%u interrupted_fault=%u acquire_reason=%u "
      "acquire_result=%u acquire_first_context_fault=%u audit_only=1\n",
      unsigned(observation.context.first_fault),
      static_cast<unsigned long long>(observation.context.first_fault_us),
      unsigned(retained_inputs.first_fault),
      static_cast<unsigned long long>(retained_inputs.first_fault_us),
      unsigned(f.present), unsigned(f.phase_installed),
      unsigned(f.actual.publication_succeeded),
      unsigned(l.publication_succeeded),
      unsigned(l.numerical_reference_installed),
      unsigned(observation.interrupted_wire.first_fault),
      observation.context.acquire.reason,observation.context.acquire.result,
      unsigned(observation.context.acquire.first_context_fault));
  std::printf("RFLY_LOCAL_INPUT max_assembly_us=%llu fault_generation=%llu "
              "first_us=%llu last_us=%llu next=%u ready=%u audit_only=1\n",
              static_cast<unsigned long long>(retained_inputs.maximum_assembly_span_us),
              static_cast<unsigned long long>(retained_inputs.fault_sequence),
              static_cast<unsigned long long>(retained_inputs.fault_first_arrival_us),
              static_cast<unsigned long long>(retained_inputs.fault_last_arrival_us),
              retained_inputs.fault_next_fragment,unsigned(retained_inputs.fault_ready));
  const auto &c = observation.context;
  std::printf(
      "RFLY_LOCAL_NESTED module_reason=%u observed=%u source_observed=%u exchange_fault=%u "
      "exchange_first_us=%llu exchange_polls=%llu exchange_snapshots=%llu "
      "exchange_missing_selected=%llu cycle_fault=%u cycle_first_us=%llu "
      "cycle_observation_captures=%llu cycle_io_releases=%llu "
      "cycle_endpoint_retirements=%llu io_fault=%u io_first_us=%llu "
      "io_source_updates=%llu io_captured=%llu io_adapter_fault=%u "
      "io_failed_sample_us=%llu io_previous_sample_us=%llu io_failed_received_us=%llu source_fault=%u "
      "source_lookup_fault=%u source_observed_records=%llu "
      "source_accepted=%llu source_lookups=%llu source_retained=%llu "
      "selected_result=%u selected_reason=%u selected_drained=%llu "
      "selected_matched=%llu selected_missing=%llu selected_retired=%llu "
      "selected_gaps=%llu audit_only=1\n",
      unsigned(c.module_stop_reason),
      unsigned(c.nested_diagnostics_observed),
      unsigned(c.original_source_diagnostics_observed),
      unsigned(c.exchange_before_stop.first_fault),
      static_cast<unsigned long long>(c.exchange_before_stop.first_fault_us),
      static_cast<unsigned long long>(c.exchange_before_stop.polls),
      static_cast<unsigned long long>(c.exchange_before_stop.snapshots_enqueued),
      static_cast<unsigned long long>(c.exchange_before_stop.missing_selected_endpoints),
      unsigned(c.cycle_before_stop.first_fault),
      static_cast<unsigned long long>(c.cycle_before_stop.first_fault_us),
      static_cast<unsigned long long>(c.cycle_before_stop.observation_captures),
      static_cast<unsigned long long>(c.cycle_before_stop.observation_io_releases),
      static_cast<unsigned long long>(c.cycle_before_stop.observation_endpoint_retirements),
      unsigned(c.local_io_before_stop.first_fault),
      static_cast<unsigned long long>(c.local_io_before_stop.first_fault_us),
      static_cast<unsigned long long>(c.local_io_before_stop.source_updates),
      static_cast<unsigned long long>(c.local_io_before_stop.captured),
      unsigned(c.local_io_before_stop.source_adapter_fault),
      static_cast<unsigned long long>(c.local_io_before_stop.source_failure_sample_us),
      static_cast<unsigned long long>(c.local_io_before_stop.source_failure_previous_sample_us),
      static_cast<unsigned long long>(c.local_io_before_stop.source_failure_received_us),
      unsigned(c.original_source_before_stop.first_fault),
      unsigned(c.original_source_before_stop.lookup_fault),
      static_cast<unsigned long long>(c.original_source_before_stop.observed_topic_records),
      static_cast<unsigned long long>(c.original_source_before_stop.accepted_records),
      static_cast<unsigned long long>(c.original_source_before_stop.successful_lookups),
      static_cast<unsigned long long>(c.original_source_before_stop.retained),
      unsigned(c.selected_source_before_stop.latched_result),
      unsigned(c.selected_source_before_stop.first_reason),
      static_cast<unsigned long long>(c.selected_source_before_stop.drained),
      static_cast<unsigned long long>(c.selected_source_before_stop.matched),
      static_cast<unsigned long long>(c.selected_source_before_stop.missing),
      static_cast<unsigned long long>(c.selected_source_before_stop.retired),
      static_cast<unsigned long long>(c.selected_source_before_stop.generation_gaps));
  return 0;
}
int entry(int argc, char *argv[]) noexcept {
  if (argc < 2 || !argv || !argv[1])
    return -EINVAL;
  const auto *cmd = argv[1];
  if (std::strcmp(cmd, "prepare") == 0 || std::strcmp(cmd, "prepare_rc") == 0) {
    if (argc != 10 || owner || inputs || !argv[2] ||
        std::strcmp(argv[2], "/dev/ttyACM0") || !argv[3] ||
        std::strlen(argv[3]) != 32 || !hex64(argv[3], challenge.high) ||
        !hex64(argv[3] + 16, challenge.low) ||
        (!challenge.high && !challenge.low) ||
        !real(argv[4], requested_origin[0]) ||
        !real(argv[5], requested_origin[1]) ||
        !real(argv[6], requested_origin[2]) || !byte(argv[7], host_system) ||
        !byte(argv[8], host_component) || !byte(argv[9], requested_leg) ||
        requested_leg > 5)
      return -EINVAL;
    requested_operator_reference=std::strcmp(cmd,"prepare_rc")==0;
    if(requested_operator_reference&&requested_leg!=1)return -EINVAL;
    observation = {};
    retained_inputs = {};
    owner = new CanonicalLocalApplicationOwner;
    if (!owner)
      return -ENOMEM;
    const bool ok = acquire_prepare(argv[2]);
    (void)owner->observe(observation);
    // The visitor's MAVLink lock is already released. No printf in that lock.
    if (prepare_acquisition.first_stale_present)
      print_prepare_diagnostic("RFLY_LOCAL_PREPARE_FIRST_STALE",
                               prepare_acquisition.first_stale);
    print_prepare_diagnostic("RFLY_LOCAL_PREPARE_DIAG", prepare_diagnostics);
    std::printf("RFLY_LOCAL_PREPARE_ACQUISITION attempts=%u sleeps=%u "
                "elapsed_us=%llu stop_reason=%u first_stale=%u "
                "bound_us=20000 max_attempts=40 sleep_us=500 "
                "source_age_us=5000 diagnostic_only=1 measured_wcet=0\n",
                prepare_acquisition.attempts, prepare_acquisition.sleeps,
                static_cast<unsigned long long>(prepare_acquisition.elapsed_us),
                prepare_acquisition.stop_reason,
                unsigned(prepare_acquisition.first_stale_present));
    print_echo();
    std::printf("RFLY_REFERENCE_SELECTION operator_velocity=%u component_only=%u "
                "operator_yaw_rate_rad_s=%.9g horizontal_mps=%.9g vertical_mps=%.9g shaping_rad_s=%.9g\n",
                unsigned(requested_operator_reference),unsigned(requested_operator_reference),
                gpenmpc_operator_reference::yaw_rate_rad_s,
                gpenmpc_operator_reference::horizontal_speed_mps,
                gpenmpc_operator_reference::vertical_speed_mps,
                gpenmpc_operator_reference::velocity_pole_rad_s);
    print_raw_guard("RFLY_LOCAL_RAW_GUARD",observation.session.raw_board);
    std::printf(
        "RFLY_LOCAL_PROFILE local=1 full_inner=1 host_inner_rpc=0 "
        "source_age_us=%llu window_assembly_us=%llu input_assembly_us=%llu "
        "outer_age_us=700000 kernel_to_publish_us=4000 telemetry_age_us=100000 "
        "commander_telemetry_age_us=600000 "
        "poll_us=1000 stack_bytes=8192 native_tail_us=30000000 measured_wcet=0 "
        "original_reset=%u origin=%.17g,%.17g,%.17g\n",
        static_cast<unsigned long long>(profile.module.source.sample_max_age_us),
        static_cast<unsigned long long>(task_profile.window.max_assembly_us),
        static_cast<unsigned long long>(task_profile.input_assembly_max_us),
        unsigned(profile.module.source.initial_reset_counter),
        requested_origin[0], requested_origin[1], requested_origin[2]);
    return ok ? 0 : -EACCES;
  }
  if (std::strcmp(cmd, "evidence") == 0)
    return argc == 2 ? print_post_release() : -EINVAL;
  if (!owner)
    return -ENODEV;
  if (std::strcmp(cmd, "confirm") == 0) {
    SessionDigest returned{}, record{};
    if (argc != 4 || !digest(argv[2], returned) || !digest(argv[3], record) ||
        !owner->observe(observation) ||
        returned != execution_session_digest(observation.echo))
      return -EINVAL;
    SessionPhysicalDeclaration d{};
    d.source = PhysicalDeclarationSource::
        OperatorUsbIsolationDeclaration;
    d.physical_setup_record_sha256 = record;
    d.exact_session_sha256 = returned;
    d.usb_only = d.props_removed = d.no_actuator_propulsion_power =
        HumanIsolationClaim::Declared;
    const auto r = owner->confirm(observation.echo, d);
    (void)owner->observe(observation);
    print_echo();
    return r == LocalApplicationResult::Ready ? 0 : -EACCES;
  }
  if (argc != 2)
    return -EINVAL;
  if (std::strcmp(cmd, "start") == 0) {
    const auto r=owner->start();
    (void)owner->observe(observation); // evidence_snapshot only, no resampling
    return r == LocalApplicationResult::Ready ? 0 : -EACCES;
  }
  if (std::strcmp(cmd, "stop") == 0)
    return owner->stop() == LocalApplicationResult::Pending ? 0 : -EIO;
  if (std::strcmp(cmd, "status") == 0) {
    const bool ok = owner->observe(observation);
    print_echo();
    return ok ? 0 : -EIO;
  }
  if (std::strcmp(cmd, "release") == 0) {
    const auto r = owner->release(observation);
    print_echo();
    std::printf("RFLY_LOCAL_RELEASE result=%u quiescent=%u disarmed=%u "
                "virtual_zero_stream_accepted=%u plant_cache_zero_proven=0 "
                "publication_attempts=%llu publication_successes=%llu\n",
                unsigned(r), unsigned(observation.context_observed_after_stop),
                unsigned(observation.context.board_disarmed_observed),
                unsigned(observation.context.virtual_zero_stream_accepted),
                static_cast<unsigned long long>(
                    observation.context.actual_publication_attempts),
                static_cast<unsigned long long>(
                    observation.context.actual_publication_successes));
    if (r != LocalApplicationResult::Detached)
      return -EBUSY;
    // Only after actual task and callbacks detach may the receiver disappear.
    if (inputs) {
      retained_inputs = inputs->diagnostics();
      delete inputs;
      inputs = nullptr;
    }
    delete owner;
    owner = nullptr;
    return 0;
  }
  return -EINVAL;
}
} // namespace
extern "C" __EXPORT int gpenmpc_rfly_session_main(int argc, char *argv[]) {
  const bool report_start=argc==2&&argv&&argv[1]&&std::strcmp(argv[1],"start")==0;
  // The order is start-print ownership -> entry. Non-start commands never
  // access retained_start_print and need only the existing entry mutex.
  if (report_start && !start_print_mutex.lock())
    return -EIO;
  if (!entry_mutex.lock()) {
    if (report_start) (void)start_print_mutex.unlock();
    return -EIO;
  }
  const int r = entry(argc, argv);
  if(report_start){retained_start_print.acquire=observation.context.acquire;
    retained_start_print.returned=observation.last_start_return;
    retained_start_print.task_created=observation.last_task_created;}
  const bool unlocked=entry_mutex.unlock();
  if(report_start&&unlocked)print_start_evidence(retained_start_print);
  const bool print_unlocked=!report_start||start_print_mutex.unlock();
  return unlocked&&print_unlocked ? r : -EIO;
}
