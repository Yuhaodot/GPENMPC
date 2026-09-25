#include "GPENMPCTrajectoryExec.hpp"
#include "LinkLifetimeRegistry.hpp"

#include <cmath>
#include <cstring>
#if defined(__PX4_POSIX)
#include <time.h>
#endif

namespace
{
constexpr double kGravity = 9.80665;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kPerRotorUpperN = 32.145727009134916;
constexpr double kArmRadiusM = 0.5665;
constexpr double kYawMomentArmM = 0.025;
// Frozen GPENMPC M600 software-profile bus value used by the parent generated-C
// boundary. PX4's generic SITL battery topic models a different pack voltage;
// battery warning/remaining are still checked independently in state_valid().
constexpr double kM600SoftwareBusVoltageV = 50.0;
constexpr double kBaseInertia[3] {1.6, 1.6, 3.0};
constexpr double kPayloadAttachmentZM = -0.25;
constexpr double kPositionGain[3] {0.4225, 0.4225, 0.5625};
constexpr double kVelocityGain[3] {1.17, 1.17, 1.35};
constexpr double kDrag[3] {0.0634905529323215, 0.0634905529323215, 0.0};
constexpr double kAttitudeGain[3] {12.544, 12.544, 12.0};
constexpr double kRateGain[3] {7.616, 7.616, 10.2};
constexpr double kRotorAngles[6] {
	1.5707963267948966, -1.5707963267948966, -0.5235987755982988,
	2.6179938779914944, 0.5235987755982988, -2.6179938779914944
};
constexpr double kRotorSpin[6] {-1.0, 1.0, -1.0, 1.0, 1.0, -1.0};

#if defined(__PX4_POSIX)
uint64_t monotonic_wall_time_ns()
{
	timespec ts{};
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}
#endif

template<typename T>
constexpr T clamp_value(T value, T lower, T upper)
{
	return value < lower ? lower : (value > upper ? upper : value);
}

template<typename T>
constexpr T maximum_value(T lhs, T rhs)
{
	return lhs < rhs ? rhs : lhs;
}

template<typename T>
bool execution_contract_scalar_equal(T lhs, T rhs)
{
	// Transport-decoded immutable contract values may differ by the final
	// representable bit across serializers.  Keep the equivalence tolerance
	// far below every frozen task/geometry resolution while avoiding unsafe
	// floating-point equality in the production PX4 build.
	if (!std::isfinite(lhs) || !std::isfinite(rhs)) {
		return false;
	}

	const T scale = maximum_value(static_cast<T>(1), maximum_value(std::fabs(lhs), std::fabs(rhs)));
	return std::fabs(lhs - rhs) <= static_cast<T>(1e-6) * scale;
}
}

GPENMPCTrajectoryExec::GPENMPCTrajectoryExec() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	rtrpdc_cg_initialize();
}

GPENMPCTrajectoryExec::~GPENMPCTrajectoryExec()
{
	rtrpdc_cg_terminate();
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

bool GPENMPCTrajectoryExec::init()
{
	_gpenmpc_control_mode_handle = param_find("RA_CTRL_MODE");

	if (_gpenmpc_control_mode_handle == PARAM_INVALID) {
		PX4_ERR("RA_CTRL_MODE unavailable");
		return false;
	}

	_last_run = hrt_absolute_time();
	ScheduleOnInterval(20_ms);
	return true;
}

bool GPENMPCTrajectoryExec::update_ground_observer(hrt_abstime now, float dt, uint32_t &failure_reason)
{
	if (_vehicle_air_data_sub.update(&_vehicle_air_data)) {
		const uint64_t sample_timestamp = _vehicle_air_data.timestamp_sample > 0
			? _vehicle_air_data.timestamp_sample : _vehicle_air_data.timestamp;
		_ground_observer.updateBaro(sample_timestamp, _vehicle_air_data.baro_alt_meter);
	}

	if (_vehicle_gps_position_sub.update(&_vehicle_gps_position)) {
		const uint64_t sample_timestamp = _vehicle_gps_position.timestamp_sample > 0
			? _vehicle_gps_position.timestamp_sample : _vehicle_gps_position.timestamp;
		_ground_observer.updateGps(sample_timestamp, static_cast<float>(_vehicle_gps_position.altitude_msl_m),
					   _vehicle_gps_position.epv, _vehicle_gps_position.vel_d_m_s,
					   _vehicle_gps_position.vel_ned_valid);
	}

	_local_position_sub.copy(&_local_position);
	_vehicle_status_sub.copy(&_vehicle_status);
	_vehicle_land_detected_sub.copy(&_vehicle_land_detected);
	const bool disarmed = _vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED;
	const bool safe_baseline_state = disarmed && _vehicle_land_detected.landed && _vehicle_land_detected.at_rest;
	_ground_observer.observeVerticalResetCounter(_local_position.z_reset_counter, safe_baseline_state);
	_ground_observer.collectBaseline(now, disarmed, _vehicle_land_detected.landed, _vehicle_land_detected.at_rest);

#if defined(__PX4_POSIX)
	// The genuine host fixture uses the exact same HIL_GPS packet generator and
	// exercises the production code path, but Commander reports HIL_STATE_OFF in
	// POSIX SITL.  The board build below instead requires native HIL_STATE_ON.
	const bool precise_hil_sensor_contract = true;
#else
	const bool precise_hil_sensor_contract = _vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON;
#endif

	_ground_estimate = _ground_observer.estimate(now, _local_position.vz, precise_hil_sensor_contract);
	GPENMPCTimeAlignedGroundFrameEstimator::Input frame_input{};
	frame_input.now_us = now;
	frame_input.local_timestamp_sample_us = _local_position.timestamp_sample > 0
		? _local_position.timestamp_sample : _local_position.timestamp;
	frame_input.ground_height_timestamp_us = maximum_value(
		_ground_observer.baroTimestamp(), _ground_observer.gpsTimestamp());
	frame_input.ground_velocity_timestamp_us = _ground_observer.gpsTimestamp();
	frame_input.precise_hil_sensor_contract = precise_hil_sensor_contract;
	frame_input.safe_ground_reset = safe_baseline_state;
	frame_input.ground_height_valid = _ground_estimate.usable;
	frame_input.ground_height_already_aligned_to_now = true;
	frame_input.ground_velocity_valid = _ground_estimate.velocity_usable;
	frame_input.local_position_valid = _local_position.z_valid;
	frame_input.local_velocity_valid = _local_position.v_z_valid;
	frame_input.ground_height_center_m = _ground_estimate.height_center_m;
	frame_input.ground_height_lower_m = _ground_estimate.height_lower_m;
	frame_input.ground_height_upper_m = _ground_estimate.height_upper_m;
	frame_input.ground_velocity_d_mps = _ground_estimate.vertical_velocity_d_mps;
	frame_input.ground_velocity_bound_mps = GPENMPCGroundRelativeObserver::kGpsHilVerticalVelocityBoundMps;
	frame_input.local_position_d_m = _local_position.z;
	frame_input.local_velocity_d_mps = _local_position.vz;
	frame_input.local_acceleration_d_mps2 = _local_position.az;
	frame_input.position_reset_counter = _local_position.z_reset_counter;
	frame_input.velocity_reset_counter = _local_position.vz_reset_counter;
	frame_input.delta_position_reset_d_m = _local_position.delta_z;
	frame_input.delta_velocity_reset_d_mps = _local_position.delta_vz;
	_ground_frame_estimate = _ground_frame_estimator.update(frame_input);
	_ground_reference_alignment_output = _ground_reference_alignment.update(
		dt, precise_hil_sensor_contract, _ground_frame_estimate.valid,
		safe_baseline_state, _ground_frame_estimate.position_bias_d_m,
		_ground_frame_estimate.velocity_bias_d_mps,
		_ground_frame_estimate.position_bias_three_sigma_m,
		_ground_frame_estimate.velocity_bias_three_sigma_mps);
	GPENMPCPrearmVerticalAdmission::Input prearm_input{};
	prearm_input.now_us = now;
	prearm_input.disarmed = disarmed;
	prearm_input.safe_baseline_state = safe_baseline_state;
	prearm_input.precise_hil_sensor_contract = precise_hil_sensor_contract;
	prearm_input.ground_usable = _ground_estimate.usable;
	prearm_input.baseline_locked = _ground_estimate.baseline_locked;
	prearm_input.baro_fresh = _ground_estimate.baro_fresh;
	prearm_input.gps_fresh = _ground_estimate.gps_fresh;
	prearm_input.interval_consistent = _ground_estimate.interval_consistent;
	prearm_input.velocity_usable = _ground_estimate.velocity_usable;
	prearm_input.ground_plant_truth_used = false;
	prearm_input.frame_initialized = _ground_frame_estimate.initialized;
	prearm_input.frame_valid = _ground_frame_estimate.valid;
	prearm_input.frame_fresh = _ground_frame_estimate.fresh;
	prearm_input.frame_hard_fault = _ground_frame_estimate.hard_fault;
	prearm_input.frame_plant_truth_used = _ground_frame_estimate.plant_truth_used;
	prearm_input.alignment_eligible = _ground_reference_alignment_output.eligible;
	prearm_input.alignment_uncertainty_ok = _ground_reference_alignment_output.uncertainty_acceptable;
	prearm_input.alignment_hard_fault = _ground_reference_alignment_output.hard_fault;
	prearm_input.baro_baseline_m = _ground_observer.baroBaselineM();
	prearm_input.gps_baseline_m = _ground_observer.gpsBaselineM();
	prearm_input.local_z_m = _local_position.z;
	prearm_input.local_vz_mps = _local_position.vz;
	prearm_input.h_ground_m = _ground_frame_estimate.height_ground_m;
	prearm_input.v_ground_d_mps = _ground_frame_estimate.velocity_ground_d_mps;
	prearm_input.h_three_sigma_m = _ground_frame_estimate.height_three_sigma_m;
	prearm_input.v_three_sigma_mps = _ground_frame_estimate.velocity_three_sigma_mps;
	prearm_input.b_z_three_sigma_m = _ground_frame_estimate.position_bias_three_sigma_m;
	prearm_input.b_v_three_sigma_mps = _ground_frame_estimate.velocity_bias_three_sigma_mps;
	prearm_input.b_z_target_m = _ground_reference_alignment_output.position_offset_target_m;
	prearm_input.b_z_applied_m = _ground_reference_alignment_output.position_offset_applied_m;
	prearm_input.b_v_target_mps = _ground_reference_alignment_output.velocity_offset_target_mps;
	prearm_input.b_v_applied_mps = _ground_reference_alignment_output.velocity_offset_applied_mps;
	prearm_input.frame_reset_events = _ground_frame_estimate.reset_event_count;
	prearm_input.vertical_reset_counter = _ground_observer.verticalResetCounter();
	_prearm_vertical_admission_output = _prearm_vertical_admission.update(prearm_input);
	gpenmpc_ground_observability_status_s status{};
	status.timestamp = now;
	status.baro_timestamp_sample = _ground_observer.baroTimestamp();
	status.gps_timestamp_sample = _ground_observer.gpsTimestamp();
	status.state = static_cast<uint8_t>(_ground_estimate.state);
	status.reason = static_cast<uint8_t>(_ground_estimate.reason);
	status.usable = _ground_estimate.usable;
	status.baseline_locked = _ground_estimate.baseline_locked;
	status.safe_baseline_state = safe_baseline_state;
	status.baro_fresh = _ground_estimate.baro_fresh;
	status.gps_fresh = _ground_estimate.gps_fresh;
	status.interval_consistent = _ground_estimate.interval_consistent;
	status.velocity_usable = _ground_estimate.velocity_usable;
	status.precise_hil_sensor_contract = precise_hil_sensor_contract;
	status.plant_truth_used = false;
	status.reference_alignment_active = _ground_reference_alignment_output.active;
	status.reference_alignment_hard_fault = _ground_reference_alignment_output.hard_fault;
	status.baro_baseline_samples = static_cast<uint32_t>(_ground_observer.baroBaselineSamples());
	status.gps_baseline_samples = static_cast<uint32_t>(_ground_observer.gpsBaselineSamples());
	status.vertical_reset_counter = _ground_observer.verticalResetCounter();
	status.baro_altitude_m = _ground_observer.baroAltitudeM();
	status.gps_altitude_m = _ground_observer.gpsAltitudeM();
	status.baro_baseline_m = _ground_observer.baroBaselineM();
	status.gps_baseline_m = _ground_observer.gpsBaselineM();
	status.baro_relative_height_m = _ground_estimate.baro_relative_height_m;
	status.gps_relative_height_m = _ground_estimate.gps_relative_height_m;
	status.gps_vertical_velocity_d_mps = _ground_observer.gpsVerticalVelocityDMps();
	status.vertical_velocity_d_mps = _ground_estimate.vertical_velocity_d_mps;
	status.vertical_velocity_lower_d_mps = _ground_estimate.vertical_velocity_lower_d_mps;
	status.vertical_velocity_upper_d_mps = _ground_estimate.vertical_velocity_upper_d_mps;
	status.height_center_m = _ground_estimate.height_center_m;
	status.height_lower_m = _ground_estimate.height_lower_m;
	status.height_upper_m = _ground_estimate.height_upper_m;
	status.baro_age_s = _ground_estimate.baro_age_s;
	status.gps_age_s = _ground_estimate.gps_age_s;
	status.baro_height_bound_m = GPENMPCGroundRelativeObserver::kBaroHeightBoundM;
	status.gps_height_bound_m = precise_hil_sensor_contract
		? GPENMPCGroundRelativeObserver::kGpsHilHeightBoundM
		: maximum_value(GPENMPCGroundRelativeObserver::kGpsMinimumHeightBoundM,
			   std::isfinite(_ground_observer.gpsAdvertisedEpvM())
			   ? _ground_observer.gpsAdvertisedEpvM() + 0.10f
			   : GPENMPCGroundRelativeObserver::kGpsMinimumHeightBoundM);
	status.gps_advertised_epv_m = _ground_observer.gpsAdvertisedEpvM();
	status.gps_vertical_velocity_bound_mps = GPENMPCGroundRelativeObserver::kGpsHilVerticalVelocityBoundMps;
	status.local_position_d_m = _local_position.z;
	status.local_velocity_d_mps = _local_position.vz;
	status.reference_position_offset_target_m = _ground_reference_alignment_output.position_offset_target_m;
	status.reference_position_offset_applied_m = _ground_reference_alignment_output.position_offset_applied_m;
	status.reference_position_offset_rate_mps = _ground_reference_alignment_output.position_offset_rate_mps;
	status.reference_velocity_offset_target_mps = _ground_reference_alignment_output.velocity_offset_target_mps;
	status.reference_velocity_offset_applied_mps = _ground_reference_alignment_output.velocity_offset_applied_mps;
	status.reference_velocity_offset_rate_mps2 = _ground_reference_alignment_output.velocity_offset_rate_mps2;
	_ground_observability_status_pub.publish(status);

	gpenmpc_ground_frame_status_s frame_status{};
	frame_status.timestamp = now;
	frame_status.local_ts = frame_input.local_timestamp_sample_us;
	frame_status.ground_h_ts = frame_input.ground_height_timestamp_us;
	frame_status.ground_v_ts = frame_input.ground_velocity_timestamp_us;
	frame_status.state = static_cast<uint8_t>(_ground_frame_estimate.state);
	frame_status.reason = static_cast<uint8_t>(_ground_frame_estimate.reason);
	frame_status.initialized = _ground_frame_estimate.initialized;
	frame_status.valid = _ground_frame_estimate.valid;
	frame_status.fresh = _ground_frame_estimate.fresh;
	frame_status.hard_fault = _ground_frame_estimate.hard_fault;
	frame_status.hybrid_transition = _ground_frame_estimate.hybrid_transition_detected;
	frame_status.innovation_rejected = _ground_frame_estimate.innovation_rejected;
	frame_status.bias_hold_near_ground = _ground_frame_estimate.bias_update_held_near_ground;
	frame_status.local_z_accepted = _ground_frame_estimate.local_position_update_accepted;
	frame_status.local_vz_accepted = _ground_frame_estimate.local_velocity_update_accepted;
	frame_status.ground_h_accepted = _ground_frame_estimate.ground_height_update_accepted;
	frame_status.ground_v_accepted = _ground_frame_estimate.ground_velocity_update_accepted;
	frame_status.alignment_active = _ground_reference_alignment_output.active;
	frame_status.alignment_uncertainty_ok = _ground_reference_alignment_output.uncertainty_acceptable;
	frame_status.alignment_hard_fault = _ground_reference_alignment_output.hard_fault;
	frame_status.plant_truth_used = false;
	frame_status.prearm_reason = static_cast<uint8_t>(_prearm_vertical_admission_output.reason);
	frame_status.prearm_predicate = _prearm_vertical_admission_output.predicate;
	frame_status.prearm_ready = _prearm_vertical_admission_output.ready;
	frame_status.prearm_baseline_ok = _prearm_vertical_admission_output.baseline_ok;
	frame_status.prearm_source_identity_ok = _prearm_vertical_admission_output.source_identity_ok;
	frame_status.prearm_covariance_ok = _prearm_vertical_admission_output.covariance_ok;
	frame_status.prearm_position_ok = _prearm_vertical_admission_output.position_ok;
	frame_status.prearm_velocity_ok = _prearm_vertical_admission_output.velocity_ok;
	frame_status.prearm_alignment_ok = _prearm_vertical_admission_output.alignment_ok;
	frame_status.prearm_reset_ok = _prearm_vertical_admission_output.reset_ok;
	frame_status.reset_events = _ground_frame_estimate.reset_event_count;
	frame_status.accepted_updates = _ground_frame_estimate.accepted_measurement_count;
	frame_status.rejected_updates = _ground_frame_estimate.rejected_measurement_count;
	frame_status.prearm_dwell_reset_count = _prearm_vertical_admission_output.dwell_reset_count;
	frame_status.prearm_frame_reset_events = _prearm_vertical_admission_output.frame_reset_events;
	frame_status.prearm_vertical_reset_counter = _prearm_vertical_admission_output.vertical_reset_counter;
	frame_status.prearm_dwell_started = _prearm_vertical_admission_output.dwell_started_us;
	frame_status.h_ground_m = _ground_frame_estimate.height_ground_m;
	frame_status.v_ground_d_mps = _ground_frame_estimate.velocity_ground_d_mps;
	frame_status.b_z_m = _ground_frame_estimate.position_bias_d_m;
	frame_status.b_v_mps = _ground_frame_estimate.velocity_bias_d_mps;
	frame_status.h_3sigma_m = _ground_frame_estimate.height_three_sigma_m;
	frame_status.v_3sigma_mps = _ground_frame_estimate.velocity_three_sigma_mps;
	frame_status.b_z_3sigma_m = _ground_frame_estimate.position_bias_three_sigma_m;
	frame_status.b_v_3sigma_mps = _ground_frame_estimate.velocity_bias_three_sigma_mps;
	frame_status.z_aligned_m = _ground_frame_estimate.aligned_local_position_d_m;
	frame_status.vz_aligned_mps = _ground_frame_estimate.aligned_local_velocity_d_mps;
	frame_status.innovation_nsigma = _ground_frame_estimate.maximum_normalized_innovation;
	frame_status.local_age_s = _ground_frame_estimate.local_age_s;
	frame_status.ground_h_age_s = _ground_frame_estimate.ground_height_age_s;
	frame_status.ground_v_age_s = _ground_frame_estimate.ground_velocity_age_s;
	frame_status.b_z_target_m = _ground_reference_alignment_output.position_offset_target_m;
	frame_status.b_z_applied_m = _ground_reference_alignment_output.position_offset_applied_m;
	frame_status.b_z_rate_mps = _ground_reference_alignment_output.position_offset_rate_mps;
	frame_status.b_v_target_mps = _ground_reference_alignment_output.velocity_offset_target_mps;
	frame_status.b_v_applied_mps = _ground_reference_alignment_output.velocity_offset_applied_mps;
	frame_status.b_v_rate_mps2 = _ground_reference_alignment_output.velocity_offset_rate_mps2;
	frame_status.prearm_dwell_s = _prearm_vertical_admission_output.dwell_s;
	frame_status.prearm_z_residual_m = _prearm_vertical_admission_output.z_residual_m;
	frame_status.prearm_vz_residual_mps = _prearm_vertical_admission_output.vz_residual_mps;
	frame_status.prearm_z_envelope_m = _prearm_vertical_admission_output.z_envelope_m;
	frame_status.prearm_vz_envelope_mps = _prearm_vertical_admission_output.vz_envelope_mps;
	frame_status.prearm_z_ratio = _prearm_vertical_admission_output.z_ratio;
	frame_status.prearm_vz_ratio = _prearm_vertical_admission_output.vz_ratio;
	frame_status.prearm_alignment_z_residual_m = _prearm_vertical_admission_output.alignment_z_residual_m;
	frame_status.prearm_alignment_vz_residual_mps = _prearm_vertical_admission_output.alignment_vz_residual_mps;
	_ground_frame_status_pub.publish(frame_status);

	gpenmpc_vertical_state_coupling_status_s coupling_status{};
	coupling_status.timestamp = now;
	coupling_status.valid = _route_vertical_state_coupling_output.valid;
	coupling_status.active = _route_vertical_state_coupling_output.active;
	coupling_status.hard_fault = _route_vertical_state_coupling_output.hard_fault;
	coupling_status.source_available = _route_vertical_state_coupling_output.source_available;
	coupling_status.plant_truth_used = false;
	coupling_status.availability_hold_s =
		_route_vertical_state_coupling_output.availability_hold_s;
	coupling_status.position_target_m =
		_route_vertical_state_coupling_output.position_target_d_m;
	coupling_status.position_applied_m =
		_route_vertical_state_coupling_output.position_applied_d_m;
	coupling_status.position_rate_mps =
		_route_vertical_state_coupling_output.position_rate_d_mps;
	coupling_status.velocity_target_mps =
		_route_vertical_state_coupling_output.velocity_target_d_mps;
	coupling_status.velocity_applied_mps =
		_route_vertical_state_coupling_output.velocity_applied_d_mps;
	coupling_status.acceleration_applied_mps2 =
		_route_vertical_state_coupling_output.acceleration_applied_d_mps2;
	coupling_status.jerk_applied_mps3 =
		_route_vertical_state_coupling_output.jerk_applied_d_mps3;
	_vertical_state_coupling_status_pub.publish(coupling_status);

	if (_ground_estimate.state == GPENMPCGroundRelativeObserver::State::HardFault
	    || _ground_frame_estimate.hard_fault
	    || _ground_reference_alignment_output.hard_fault
	    || (!disarmed && (!_ground_observer.baselineLocked() || !_ground_frame_estimate.valid
			      || !_ground_reference_alignment_output.eligible))) {
		failure_reason = gpenmpc_exec_status_s::FAIL_GROUND_OBSERVER_IDENTITY;
		return false;
	}

	return true;
}

bool GPENMPCTrajectoryExec::hash_equal(const uint32_t lhs[8], const uint32_t rhs[8])
{
	for (int i = 0; i < 8; ++i) {
		if (lhs[i] != rhs[i]) { return false; }
	}

	return true;
}

bool GPENMPCTrajectoryExec::finite_segment(const gpenmpc_trajectory_segment_s &segment)
{
	for (double value : segment.control_points_ned_m) {
		if (!std::isfinite(value)) { return false; }
	}

	for (double value : segment.estimated_wind_ned_mps) {
		if (!std::isfinite(value)) { return false; }
	}

	return std::isfinite(segment.duration_s)
	       && std::isfinite(segment.initial_progress)
	       && std::isfinite(segment.progress_rate)
	       && std::isfinite(segment.yaw_start_rad)
	       && std::isfinite(segment.yaw_end_rad)
	       && std::isfinite(segment.yaw_rate_limit_rad_s)
	       && std::isfinite(segment.base_mass_kg)
	       && std::isfinite(segment.payload_remaining_kg)
	       && std::isfinite(segment.delivered_payload_kg);
}

bool GPENMPCTrajectoryExec::same_execution_contract(const gpenmpc_trajectory_segment_s &lhs,
		const gpenmpc_trajectory_segment_s &rhs)
{
	// The host republishes the active immutable curve at 10 Hz with a new
	// sequence and an updated initial-progress sample.  Those transport fields
	// must not move the onboard clock backwards after the causal takeoff latch.
	// Geometry, yaw, mass, service, hash, duration, wind, or feature changes are
	// intentionally not normalized and therefore remain distinct contracts.
	if (lhs.schema_version != rhs.schema_version
	    || lhs.control_point_count != rhs.control_point_count
	    || !execution_contract_scalar_equal(lhs.duration_s, rhs.duration_s)
	    || !execution_contract_scalar_equal(lhs.progress_rate, rhs.progress_rate)
	    || !execution_contract_scalar_equal(lhs.yaw_start_rad, rhs.yaw_start_rad)
	    || !execution_contract_scalar_equal(lhs.yaw_end_rad, rhs.yaw_end_rad)
	    || !execution_contract_scalar_equal(lhs.yaw_rate_limit_rad_s, rhs.yaw_rate_limit_rad_s)
	    || !execution_contract_scalar_equal(lhs.base_mass_kg, rhs.base_mass_kg)
	    || !execution_contract_scalar_equal(lhs.payload_remaining_kg, rhs.payload_remaining_kg)
	    || !execution_contract_scalar_equal(lhs.delivered_payload_kg, rhs.delivered_payload_kg)
	    || lhs.service_edge != rhs.service_edge
	    || lhs.payload_transition_enabled != rhs.payload_transition_enabled
	    || lhs.host_feedforward_disabled != rhs.host_feedforward_disabled
	    || !hash_equal(lhs.source_hash, rhs.source_hash)
	    || !hash_equal(lhs.firmware_contract_hash, rhs.firmware_contract_hash)
	    || !hash_equal(lhs.profile_hash, rhs.profile_hash)
	    || !hash_equal(lhs.component_hash, rhs.component_hash)) {
		return false;
	}

	for (int i = 0; i < 18; ++i) {
		if (!execution_contract_scalar_equal(lhs.control_points_ned_m[i], rhs.control_points_ned_m[i])) {
			return false;
		}
	}

	for (int i = 0; i < 3; ++i) {
		if (!execution_contract_scalar_equal(lhs.estimated_wind_ned_mps[i], rhs.estimated_wind_ned_mps[i])) {
			return false;
		}
	}

	return true;
}

bool GPENMPCTrajectoryExec::update_takeoff_phase(hrt_abstime now, uint32_t &failure_reason)
{
	_takeoff_status_sub.copy(&_takeoff_status);

	GPENMPCTakeoffPhaseAlignment::Input input{};
	input.now_us = now;
	input.takeoff_status_timestamp_us = _takeoff_status.timestamp;
	input.takeoff_state = _takeoff_status.takeoff_state;
	input.armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	input.offboard = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
	input.landed = _vehicle_land_detected.landed;
	input.ground_contact = _vehicle_land_detected.ground_contact;
	input.maybe_landed = _vehicle_land_detected.maybe_landed;
	input.ground_frame_valid = _ground_frame_estimate.valid;
	input.ground_frame_fresh = _ground_frame_estimate.fresh;
	input.ground_frame_hard_fault = _ground_frame_estimate.hard_fault;
	input.height_ground_m = _ground_frame_estimate.height_ground_m;
	input.velocity_ground_d_mps = _ground_frame_estimate.velocity_ground_d_mps;
	input.height_three_sigma_m = _ground_frame_estimate.height_three_sigma_m;
	input.velocity_three_sigma_mps = _ground_frame_estimate.velocity_three_sigma_mps;
	input.local_position_d_m = _local_position.z;
	input.local_velocity_d_mps = _local_position.vz;
	input.local_acceleration_d_mps2 = _local_position.az;
	input.reset_event_count = _ground_frame_estimate.reset_event_count;
	_takeoff_phase_output = _takeoff_phase_alignment.update(input);

	if (_takeoff_phase_output.announce_phase_start) {
		mavlink_log_info(&_mavlink_log_pub, "RA_TKO_START:%llu",
				 static_cast<unsigned long long>(_takeoff_phase_output.phase_start_timestamp_us));
	}

	if (!_takeoff_phase_output.valid) {
		failure_reason = gpenmpc_exec_status_s::FAIL_TAKEOFF_PHASE_ALIGNMENT;
		return false;
	}

	failure_reason = gpenmpc_exec_status_s::FAIL_NONE;
	return true;
}

uint32_t GPENMPCTrajectoryExec::transition_failure_reason(PayloadTransitionCompensator::Fault fault)
{
	using Fault = PayloadTransitionCompensator::Fault;

	switch (fault) {
	case Fault::HteTimestampDuplicate:
	case Fault::HteTimestampRegression:
	case Fault::TimeRegression:
		return gpenmpc_exec_status_s::FAIL_HTE_TIMESTAMP;

	case Fault::HteInvalid:
		return gpenmpc_exec_status_s::FAIL_HTE_INVALID;

	case Fault::HteStale:
		return gpenmpc_exec_status_s::FAIL_HTE_STALE;

	case Fault::ServiceIdentity:
	case Fault::ServiceSequence:
		return gpenmpc_exec_status_s::FAIL_SERVICE_IDENTITY;

	case Fault::CompensationNonfinite:
	case Fault::CompensationSaturation:
	case Fault::ConvergenceTimeout:
	case Fault::ApplicationAckTimestamp:
	case Fault::ApplicationAckIdentity:
	case Fault::ApplicationAckFault:
	case Fault::HteMismatchGrowth:
		return gpenmpc_exec_status_s::FAIL_HTE_COMPENSATION;

	case Fault::None:
	default:
		return gpenmpc_exec_status_s::FAIL_NONE;
	}
}

uint32_t GPENMPCTrajectoryExec::hte_arbitration_failure_reason(HteSameTimestampArbitrator::Fault fault)
{
	using Fault = HteSameTimestampArbitrator::Fault;

	switch (fault) {
	case Fault::GenerationDuplicate:
	case Fault::GenerationRegression:
	case Fault::GenerationGap:
	case Fault::PublicationTimestampRegression:
	case Fault::PublicationTimestampFuture:
	case Fault::SampleTimestampRegression:
	case Fault::ResetEpochInconsistent:
	case Fault::GroupTimeout:
	case Fault::SourceFailureIdentity:
		return gpenmpc_exec_status_s::FAIL_HTE_TIMESTAMP;

	case Fault::ValidSampleNonfinite:
	case Fault::CovarianceNotPsd:
	case Fault::SameSampleValueConflict:
	case Fault::InvalidOnlyDtTooSmall:
	case Fault::MalformedInvalid:
		return gpenmpc_exec_status_s::FAIL_HTE_INVALID;

	case Fault::None:
	default:
		return gpenmpc_exec_status_s::FAIL_NONE;
	}
}

bool GPENMPCTrajectoryExec::update_payload_transition_ack(uint32_t &failure_reason)
{
	gpenmpc_payload_transition_ack_s ack{};

	if (_payload_transition_ack_sub.update(&ack)) {
		// The acknowledgement is published by mc_pos_control on another work
		// queue.  It can therefore be published after this Run() captured its
		// cycle-start timestamp but before the subscription copy above.  Validate
		// against the causal receive time, not that stale cycle-start snapshot;
		// the compensator still rejects a genuinely future timestamp and all
		// duplicate/regressed acknowledgement publications.
		const hrt_abstime acknowledgement_receive_time = hrt_absolute_time();
		if (!_payload_transition.acknowledge_application(ack.timestamp, ack.source_status_timestamp,
				ack.last_application_timestamp, ack.service_timestamp, ack.application_count,
				ack.service_count, ack.service_sequence, ack.fault, ack.application_valid,
				ack.applied_hover_thrust, acknowledgement_receive_time)) {
			failure_reason = transition_failure_reason(_payload_transition.diagnostics().fault);
			return false;
		}
	}

	return true;
}

bool GPENMPCTrajectoryExec::validate_segment(const gpenmpc_trajectory_segment_s &candidate, hrt_abstime now,
		uint32_t &failure_reason) const
{
	if (candidate.schema_version != gpenmpc_identity::schema_version) {
		failure_reason = gpenmpc_exec_status_s::FAIL_SCHEMA;
		return false;
	}

	if (!hash_equal(candidate.source_hash, gpenmpc_identity::source_hash)
	    || !hash_equal(candidate.firmware_contract_hash, gpenmpc_identity::firmware_contract_hash)
	    || !hash_equal(candidate.profile_hash, gpenmpc_identity::profile_hash)
	    || !hash_equal(candidate.component_hash, gpenmpc_identity::component_hash)) {
		failure_reason = gpenmpc_exec_status_s::FAIL_HASH;
		return false;
	}

	if (!finite_segment(candidate)) {
		failure_reason = gpenmpc_exec_status_s::FAIL_NONFINITE;
		return false;
	}

	if (candidate.timestamp == 0 || candidate.timestamp > now + 200000ULL
	    || now > candidate.timestamp + gpenmpc_identity::input_lease_us
	    || candidate.segment_start_timestamp > now + 200000ULL) {
		failure_reason = gpenmpc_exec_status_s::FAIL_TIMESTAMP;
		return false;
	}

	if (_have_segment && candidate.sequence < _last_sequence) {
		failure_reason = gpenmpc_exec_status_s::FAIL_SEQUENCE;
		return false;
	}

	if (candidate.control_point_count != 6 || candidate.duration_s <= 0.05 || candidate.progress_rate <= 0.0
	    || candidate.progress_rate > 1.0 || candidate.initial_progress < 0.0
	    || candidate.initial_progress > 1.0 || candidate.yaw_rate_limit_rad_s <= 0.0
	    || candidate.base_mass_kg <= 0.0 || candidate.payload_remaining_kg < 0.0
	    || candidate.delivered_payload_kg < 0.0
	    || candidate.delivered_payload_kg > candidate.payload_remaining_kg + 1e-9
	    || std::fabs(candidate.base_mass_kg - 9.5) > 0.002
	    || (candidate.service_edge && candidate.delivered_payload_kg <= 0.0)
	    || (!candidate.service_edge && candidate.delivered_payload_kg > 1e-6)
	    || (candidate.payload_transition_enabled && !candidate.host_feedforward_disabled)) {
		failure_reason = gpenmpc_exec_status_s::FAIL_SCHEMA;
		return false;
	}

	failure_reason = gpenmpc_exec_status_s::FAIL_NONE;
	return true;
}

bool GPENMPCTrajectoryExec::update_hover_thrust_estimate(hrt_abstime now, uint32_t &failure_reason)
{
	// Independent executor-side flight envelope.  The mirrored HTE fields are
	// necessary source identity, but they are not sufficient to authorize a
	// held control target.  A bounded dropout is task-flight-only and cannot
	// cross ground contact, terminal LAND, or an arming/nav-state transition.
	const bool bounded_availability_flight_envelope =
		_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
		&& _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
		&& !_vehicle_land_detected.landed
		&& !_vehicle_land_detected.ground_contact
		&& !_vehicle_land_detected.maybe_landed
		&& _have_segment
		&& now != 0ULL;
	using Arbitrator = HteSameTimestampArbitrator;
	auto consume_result = [&](const Arbitrator::Result &arbitration_result) -> bool {
		if (arbitration_result.action == Arbitrator::Action::HardFault) {
			failure_reason = hte_arbitration_failure_reason(arbitration_result.fault);
			return false;
		}

		if (arbitration_result.action == Arbitrator::Action::Forward) {
			const auto &publication = arbitration_result.publication;
			const hrt_abstime consumption_time = hrt_absolute_time();

			if (!_payload_transition.update_hte(publication.timestamp, publication.timestamp_sample,
					publication.hover_thrust, publication.variance, publication.valid, consumption_time,
					static_cast<uint8_t>(publication.eligibility_reason), publication.eligibility_available,
					publication.armed, publication.in_air, publication.landed,
					publication.local_position_dt, publication.local_position_sample_advanced,
					publication.failure_count, publication.last_failure_timestamp,
					static_cast<uint8_t>(publication.last_failure_reason),
					publication.last_failure_armed, publication.last_failure_in_air,
					publication.last_failure_landed,
					publication.last_failure_local_position_dt,
					publication.last_failure_local_position_sample_advanced,
					arbitration_result.same_time_dt_too_small_coalesced,
					arbitration_result.bounded_availability_dropout,
					arbitration_result.repeated_availability_dropout,
					bounded_availability_flight_envelope)) {
				failure_reason = transition_failure_reason(_payload_transition.diagnostics().fault);
				return false;
			}
		}

		return true;
	};

	// Drain the complete bounded uORB queue.  The generated message constant is
	// authoritative; do not duplicate a stale queue-depth number in this code.
	// available generation in order so a valid publication and a subsequent
	// same-HRT DtTooSmall sentinel are presented to one atomic group.  The loop
	// is bounded by the queue depth; a real overflow is detected by the
	// arbitrator's generation-gap hard gate on the next copied generation.
	for (uint8_t index = 0; index < gpenmpc_hte_publication_s::ORB_QUEUE_LENGTH; ++index) {
		gpenmpc_hte_publication_s estimate{};

		if (!_gpenmpc_hte_publication_sub.update(&estimate)) {
			break;
		}

		Arbitrator::Publication publication{};
		publication.generation = _gpenmpc_hte_publication_sub.get_last_generation();
		publication.timestamp = estimate.timestamp;
		publication.timestamp_sample = estimate.timestamp_sample;
		publication.hover_thrust = estimate.hover_thrust;
		publication.variance = estimate.hover_thrust_var;
		publication.valid = estimate.valid;
		publication.eligibility_reason = static_cast<Arbitrator::EligibilityReason>(estimate.eligibility_reason);
		publication.eligibility_available = estimate.eligibility_available;
		publication.armed = estimate.eligibility_armed;
		publication.in_air = estimate.eligibility_in_air;
		publication.landed = estimate.eligibility_landed;
		publication.local_position_dt = estimate.local_position_dt;
		publication.local_position_timestamp = estimate.local_position_timestamp;
		publication.local_position_timestamp_sample = estimate.local_position_timestamp_sample;
		publication.local_position_sample_advanced = estimate.local_position_sample_advanced;
		publication.z_reset_counter = estimate.local_position_z_reset_counter;
		publication.vz_reset_counter = estimate.local_position_vz_reset_counter;
		publication.dist_bottom_reset_counter = estimate.local_position_dist_bottom_reset_counter;
		publication.failure_count = estimate.eligibility_failure_count;
		publication.last_failure_timestamp = estimate.last_eligibility_failure_timestamp;
		publication.last_failure_reason =
			static_cast<Arbitrator::EligibilityReason>(estimate.last_eligibility_failure_reason);
		publication.last_failure_armed = estimate.last_failure_armed;
		publication.last_failure_in_air = estimate.last_failure_in_air;
		publication.last_failure_landed = estimate.last_failure_landed;
		publication.last_failure_local_position_dt = estimate.last_failure_local_position_dt;
		publication.last_failure_local_position_timestamp = estimate.last_failure_local_position_timestamp;
		publication.last_failure_local_position_timestamp_sample =
			estimate.last_failure_local_position_timestamp_sample;
		publication.last_failure_local_position_sample_advanced =
			estimate.last_failure_local_position_sample_advanced;
		publication.last_failure_z_reset_counter = estimate.last_failure_z_reset_counter;
		publication.last_failure_vz_reset_counter = estimate.last_failure_vz_reset_counter;
		publication.last_failure_dist_bottom_reset_counter =
			estimate.last_failure_dist_bottom_reset_counter;

		// The subscription copy can occur after the Run() cycle-start snapshot.
		// Use the actual receive time for the future/publication check.
		const auto push_result = _hte_same_timestamp_arbitrator.push(publication, hrt_absolute_time());

		if (!consume_result(push_result)) {
			return false;
		}
	}

	if (!consume_result(_hte_same_timestamp_arbitrator.poll(hrt_absolute_time()))) {
		return false;
	}

	failure_reason = gpenmpc_exec_status_s::FAIL_NONE;
	return true;
}

bool GPENMPCTrajectoryExec::update_segment(hrt_abstime now, uint32_t &failure_reason)
{
	gpenmpc_trajectory_segment_s candidate{};

	if (_segment_sub.update(&candidate)) {
		if (!validate_segment(candidate, now, failure_reason)) {
			++_rejected_input_count;
			return false;
		}

		if (!_have_segment || candidate.sequence > _last_sequence) {
			const bool same_contract = _have_segment && same_execution_contract(candidate, _segment);
			constexpr double kMassToleranceKg = 0.002;
			// The route identity binds the initial payload mass used for admission.
			const double expected_initial_payload_kg = 2.27;

			if ((!_have_segment
			     && std::fabs(candidate.payload_remaining_kg - expected_initial_payload_kg) > kMassToleranceKg)
			    || (_have_segment
				&& std::fabs(candidate.payload_remaining_kg - _payload_remaining_kg) > kMassToleranceKg)) {
				failure_reason = gpenmpc_exec_status_s::FAIL_SERVICE_IDENTITY;
				++_rejected_input_count;
				return false;
			}

			if (candidate.service_edge && candidate.payload_transition_enabled
			    && !_payload_transition.start_service(candidate.sequence, now,
					static_cast<float>(candidate.base_mass_kg),
					static_cast<float>(candidate.payload_remaining_kg),
					static_cast<float>(candidate.delivered_payload_kg))) {
				failure_reason = transition_failure_reason(_payload_transition.diagnostics().fault);
				++_rejected_input_count;
				return false;
			}

			_segment = candidate;
			_payload_transition_enabled = candidate.payload_transition_enabled;
			_production_host_feedforward_disabled = candidate.host_feedforward_disabled;
			_last_sequence = candidate.sequence;
			if (!_takeoff_phase_output.task_progress_enabled) {
				_progress = 0.0;

			} else if (same_contract) {
				_progress = maximum_value(_progress, candidate.initial_progress);

			} else {
				_progress = candidate.initial_progress;
			}
			_payload_remaining_kg = candidate.payload_remaining_kg;
			_component_reset_pending = _component_reset_pending || candidate.reset;
			_have_segment = true;

		} else {
			// A repeated sequence is a byte-exact lease refresh only. Any service,
			// payload, geometry, timing, or reset change requires a new sequence.
			gpenmpc_trajectory_segment_s previous = _segment;
			previous.timestamp = candidate.timestamp;
			const bool same_contract = std::memcmp(&candidate, &previous, sizeof(candidate)) == 0;

			if (!same_contract) {
				failure_reason = gpenmpc_exec_status_s::FAIL_SEQUENCE;
				++_rejected_input_count;
				return false;
			}

			_segment.timestamp = candidate.timestamp;
		}
	}

	if (!_have_segment || now > _segment.timestamp + gpenmpc_identity::input_lease_us) {
		failure_reason = gpenmpc_exec_status_s::FAIL_STALE;
		return false;
	}

	return true;
}

bool GPENMPCTrajectoryExec::state_valid(hrt_abstime now, uint32_t &failure_reason)
{
	_local_position_sub.copy(&_local_position);
	_attitude_sub.copy(&_attitude);
	_angular_velocity_sub.copy(&_angular_velocity);
	_battery_sub.copy(&_battery);
	_estimator_sub.copy(&_estimator);
	_vehicle_status_sub.copy(&_vehicle_status);
	_allocator_status_sub.copy(&_allocator_status);
	_actuator_motors_sub.copy(&_actuator_motors);

	if (_local_position.timestamp == 0 || _attitude.timestamp == 0 || _angular_velocity.timestamp == 0
	    || now > _local_position.timestamp + 500000ULL || now > _attitude.timestamp + 500000ULL
	    || now > _angular_velocity.timestamp + 500000ULL || !_local_position.xy_valid
	    || !_local_position.z_valid || !_local_position.v_xy_valid || !_local_position.v_z_valid
	    || _estimator.filter_fault_flags != 0) {
		failure_reason = gpenmpc_exec_status_s::FAIL_STATE;
		return false;
	}

	if (_battery.timestamp != 0 && (_battery.warning >= battery_status_s::WARNING_CRITICAL
	    || (_battery.connected && std::isfinite(_battery.remaining) && _battery.remaining < 0.05f))) {
		failure_reason = gpenmpc_exec_status_s::FAIL_BATTERY;
		return false;
	}

	for (float value : _attitude.q) {
		if (!std::isfinite(value)) {
			failure_reason = gpenmpc_exec_status_s::FAIL_STATE;
			return false;
		}
	}

	failure_reason = gpenmpc_exec_status_s::FAIL_NONE;
	return true;
}

double GPENMPCTrajectoryExec::wrap_pi(double value)
{
	while (value > kPi) { value -= 2.0 * kPi; }
	while (value < -kPi) { value += 2.0 * kPi; }
	return value;
}

void GPENMPCTrajectoryExec::smooth_yaw(double u, double duration, double yaw_start, double yaw_end,
		double rate_limit, float &yaw, float &yaw_rate)
{
	u = clamp_value(u, 0.0, 1.0);
	const double delta = wrap_pi(yaw_end - yaw_start);
	const double s = 10.0 * u * u * u - 15.0 * u * u * u * u + 6.0 * u * u * u * u * u;
	const double ds = (30.0 * u * u - 60.0 * u * u * u + 30.0 * u * u * u * u) / duration;
	yaw = static_cast<float>(wrap_pi(yaw_start + s * delta));
	yaw_rate = static_cast<float>(clamp_value(ds * delta, -rate_limit, rate_limit));
}

bool GPENMPCTrajectoryExec::run_generated_c(float dt, hrt_abstime now, trajectory_setpoint_s &setpoint,
		uint32_t &failure_reason)
{
	double *u = rtrpdc_cg_U.u;
	for (int i = 0; i < 96; ++i) {
		u[i] = 0.0;
	}
	u[0] = dt;
	u[1] = _component_reset_pending ? 1.0 : 0.0;
	u[8] = _segment.progress_rate;
	u[9] = _segment.progress_rate;
	u[10] = 0.0; // G_full is permanently shadow-only.
	u[11] = 0.0;
	u[12] = 0.5;
	u[13] = kM600SoftwareBusVoltageV;
	u[14] = 43.0;
	u[15] = 1.0;
	u[16] = _segment.service_edge ? 1.0 : 0.0;
	u[17] = _payload_remaining_kg;
	u[18] = _segment.delivered_payload_kg;
	u[19] = _segment.base_mass_kg;
	u[20] = clamp_value(_progress, 0.0, 1.0);
	u[21] = _segment.duration_s;

	for (int i = 0; i < 18; ++i) { u[22 + i] = _segment.control_points_ned_m[i]; }

	u[40] = _local_position.x;
	u[41] = _local_position.y;
	u[42] = _local_position.z;
	u[43] = _local_position.vx;
	u[44] = _local_position.vy;
	u[45] = _local_position.vz;

	for (int i = 0; i < 3; ++i) {
		u[46 + i] = _segment.estimated_wind_ned_mps[i];
		u[49 + i] = _segment.estimated_wind_ned_mps[i];
		u[52 + i] = kPositionGain[i];
		u[55 + i] = kVelocityGain[i];
		u[58 + i] = kDrag[i];
		u[66 + i] = kAttitudeGain[i];
		u[69 + i] = kRateGain[i];
		u[76 + i] = _angular_velocity.xyz[i];
	}

	u[61] = 2.0;
	u[62] = kGravity;
	u[63] = kPerRotorUpperN;
	u[64] = kArmRadiusM;
	u[65] = kYawMomentArmM;

	for (int i = 0; i < 4; ++i) { u[72 + i] = _attitude.q[i]; }

	const double payload_inertia = _payload_remaining_kg * kPayloadAttachmentZM * kPayloadAttachmentZM;
	u[79] = kBaseInertia[0] + payload_inertia;
	u[80] = kBaseInertia[1] + payload_inertia;
	u[81] = kBaseInertia[2];

	float yaw = 0.f;
	float yaw_rate = 0.f;
	smooth_yaw(_progress, _segment.duration_s / _segment.progress_rate, _segment.yaw_start_rad,
		   _segment.yaw_end_rad, _segment.yaw_rate_limit_rad_s, yaw, yaw_rate);
	u[82] = yaw;
	u[83] = yaw_rate;

	for (int i = 0; i < 6; ++i) {
		u[84 + i] = kRotorAngles[i];
		u[90 + i] = kRotorSpin[i];
	}

	rtrpdc_cg_step();
	++_generated_c_calls;
	_component_reset_pending = false;

	for (double value : rtrpdc_cg_Y.y) {
		if (!std::isfinite(value)) {
			failure_reason = gpenmpc_exec_status_s::FAIL_CODEGEN;
			return false;
		}
	}

	if (std::fabs(rtrpdc_cg_Y.y[2]) > 0.5
	    || std::fabs(rtrpdc_cg_Y.y[3] - _segment.progress_rate) > 1e-12) {
		failure_reason = gpenmpc_exec_status_s::FAIL_C_AUTHORITY;
		return false;
	}

	if (std::fabs(rtrpdc_cg_Y.y[4]) > 0.5) {
		failure_reason = gpenmpc_exec_status_s::FAIL_CODEGEN;
		return false;
	}

	setpoint.timestamp = now;

	for (int i = 0; i < 3; ++i) {
		setpoint.position[i] = static_cast<float>(rtrpdc_cg_Y.y[8 + i]);
		setpoint.velocity[i] = static_cast<float>(rtrpdc_cg_Y.y[11 + i]);
		setpoint.acceleration[i] = static_cast<float>(rtrpdc_cg_Y.y[14 + i]);
		setpoint.jerk[i] = static_cast<float>(rtrpdc_cg_Y.y[17 + i]);
	}

	// The generated trajectory is immutable in the physical launch-plane frame.
	// Admission and the pre-latch takeoff command use the complete
	// independently estimated b_z/b_v coordinate transform.  At TaskActive the
	// position transform remains absolute because local-z and physical launch-
	// plane altitude have different origins.  The velocity transform becomes
	// origin-relative at TaskActive because a fixed pre-arm b_v must not become a
	// permanent route command.  The five-second p/v/a state-match polynomial
	// keeps the transition continuous and later reset deltas remain compensated.
	const auto task_reference_offset = _ground_reference_alignment.taskReferenceOffset(
		_takeoff_phase_output.task_progress_enabled,
		_ground_reference_alignment_output.eligible);

	if (!task_reference_offset.valid) {
		failure_reason = gpenmpc_exec_status_s::FAIL_GROUND_OBSERVER_IDENTITY;
		return false;
	}

	setpoint.position[2] += task_reference_offset.position_d_m;
	setpoint.velocity[2] += task_reference_offset.velocity_d_mps;

	// Keep the immutable task clock at zero and the vertical position channel in
	// velocity-only form before the causal phase latch.  At TaskActive, capture
	// the current onboard z/vz/az and remove its exact residual to the generated
	// raw task reference with a five-second HRT-timed quintic. The correction
	// aligns the onboard state with the fixed task reference.
	if (!_takeoff_phase_output.task_progress_enabled) {
		setpoint.position[2] = NAN;
		setpoint.velocity[2] += _takeoff_phase_output.intent_velocity_d_mps;
		setpoint.acceleration[2] += _takeoff_phase_output.intent_acceleration_d_mps2;
		setpoint.jerk[2] += _takeoff_phase_output.intent_jerk_d_mps3;
		_takeoff_state_match_correction = {};

	} else {
		_takeoff_state_match_correction = _takeoff_phase_alignment.stateMatchCorrection(
						 now, setpoint.position[2], setpoint.velocity[2],
						 setpoint.acceleration[2]);

		if (!_takeoff_state_match_correction.valid) {
			failure_reason = gpenmpc_exec_status_s::FAIL_TAKEOFF_PHASE_ALIGNMENT;
			return false;
		}

		setpoint.position[2] += _takeoff_state_match_correction.position_d_m;
		setpoint.velocity[2] += _takeoff_state_match_correction.velocity_d_mps;
		setpoint.acceleration[2] += _takeoff_state_match_correction.acceleration_d_mps2;
		setpoint.jerk[2] += _takeoff_state_match_correction.jerk_d_mps3;
		// Preserve the existing telemetry fields as the bounded vertical
		// correction channels.  The underlying task reference remains available
		// independently in gpenmpc_exec_status.
		_takeoff_phase_output.intent_gain = _takeoff_state_match_correction.remaining_gain;
		_takeoff_phase_output.intent_velocity_d_mps = _takeoff_state_match_correction.velocity_d_mps;
		_takeoff_phase_output.intent_acceleration_d_mps2 =
			_takeoff_state_match_correction.acceleration_d_mps2;
		_takeoff_phase_output.intent_jerk_d_mps3 = _takeoff_state_match_correction.jerk_d_mps3;
		_takeoff_phase_output.intent_active = _takeoff_state_match_correction.active;
	}

	// Close the independently observed PX4-local versus causal
	// GPS/barometer ground-frame residual after the launch state-match polynomial
	// has completed.  This is a bounded coordinate/reference transform, not a
	// second controller and not additional actuator authority.  It is disabled
	// before route motion and immediately reset on LAND, stale/nonfinite data,
	// or any identity/hard fault.  A fresh identity-valid valid=false interval
	// may only retain the last verified targets for the frozen 50 ms bound.
	GPENMPCVerticalStateCoupling::Input coupling_input{};
	coupling_input.dt_s = dt;
	coupling_input.route_motion_active = _takeoff_phase_output.task_progress_enabled
		&& !_takeoff_state_match_correction.active
		&& _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
		&& _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
		&& !_vehicle_land_detected.landed
		&& !_vehicle_land_detected.ground_contact
		&& !_vehicle_land_detected.maybe_landed;
	coupling_input.source_valid = _ground_frame_estimate.valid;
	coupling_input.source_fresh = _ground_frame_estimate.fresh;
	coupling_input.source_identity_valid = !_ground_frame_estimate.hard_fault
		&& !_ground_frame_estimate.plant_truth_used
		&& _ground_reference_alignment_output.eligible
		&& !_ground_reference_alignment_output.hard_fault;
	coupling_input.local_position_d_m = _local_position.z;
	coupling_input.local_velocity_d_mps = _local_position.vz;
	coupling_input.ground_height_m = _ground_frame_estimate.height_ground_m;
	coupling_input.ground_velocity_d_mps = _ground_frame_estimate.velocity_ground_d_mps;
	coupling_input.base_position_offset_d_m = task_reference_offset.position_d_m;
	coupling_input.base_velocity_offset_d_mps = task_reference_offset.velocity_d_mps;
	_route_vertical_state_coupling_output = _route_vertical_state_coupling.update(coupling_input);

	if (coupling_input.route_motion_active
	    && (!_route_vertical_state_coupling_output.valid
		|| _route_vertical_state_coupling_output.hard_fault)) {
		failure_reason = gpenmpc_exec_status_s::FAIL_GROUND_OBSERVER_IDENTITY;
		return false;
	}

	if (_route_vertical_state_coupling_output.active) {
		setpoint.position[2] += _route_vertical_state_coupling_output.position_applied_d_m;
		setpoint.velocity[2] += _route_vertical_state_coupling_output.velocity_applied_d_mps;
		setpoint.acceleration[2] += _route_vertical_state_coupling_output.acceleration_applied_d_mps2;
		setpoint.jerk[2] += _route_vertical_state_coupling_output.jerk_applied_d_mps3;
	}

	setpoint.yaw = yaw;
	setpoint.yawspeed = yaw_rate;
	_payload_remaining_kg = rtrpdc_cg_Y.y[5];

	float transition_acceleration_down_mps2 = 0.f;
	float transition_acceleration_rate_mps3 = 0.f;

	const bool bounded_availability_flight_envelope =
		_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
		&& _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
		&& !_vehicle_land_detected.landed
		&& !_vehicle_land_detected.ground_contact
		&& !_vehicle_land_detected.maybe_landed
		&& _have_segment;

	if (!_payload_transition.step(now, dt, transition_acceleration_down_mps2,
			transition_acceleration_rate_mps3,
			bounded_availability_flight_envelope)) {
		failure_reason = transition_failure_reason(_payload_transition.diagnostics().fault);
		return false;
	}

	if (_payload_transition.diagnostics().recovery_requested) {
		// The status topic keeps the last verified hover-thrust target active in
		// mc_pos_control while the execution path fail-closes into native landing.
		failure_reason = gpenmpc_exec_status_s::FAIL_HTE_AVAILABILITY_TIMEOUT;
		return false;
	}

	// The production path publishes the causal h_target for an exactly-once
	// mc_pos_control application. A simultaneous trajectory-acceleration term
	// would double-compensate stock HTE's bumpless integrator update.
	if (std::fabs(transition_acceleration_down_mps2) > 1e-6f
	    || std::fabs(transition_acceleration_rate_mps3) > 1e-6f) {
		failure_reason = gpenmpc_exec_status_s::FAIL_SCHEMA;
		return false;
	}
	failure_reason = gpenmpc_exec_status_s::FAIL_NONE;
	return true;
}

void GPENMPCTrajectoryExec::update_shadow(hrt_abstime now)
{
	gpenmpc_shadow_suggestion_s shadow{};

	if (_shadow_sub.update(&shadow)) {
		++_shadow_sample_count;
		const bool valid = shadow.timestamp != 0 && shadow.timestamp <= now + 200000ULL
				   && now <= shadow.timestamp + gpenmpc_identity::input_lease_us
				   && hash_equal(shadow.source_hash, gpenmpc_identity::source_hash)
				   && hash_equal(shadow.profile_hash, gpenmpc_identity::profile_hash)
				   && std::isfinite(shadow.predicted_position_risk_m)
				   && std::isfinite(shadow.recoverability_margin)
				   && std::isfinite(shadow.proposed_progress_rate);

		if (valid) {
			_shadow = shadow;

		} else {
			++_shadow_invalid_count;
			std::memset(&_shadow, 0, sizeof(_shadow));
		}
	}
}

void GPENMPCTrajectoryExec::publish_status(hrt_abstime now, uint32_t failure_reason,
		bool active_published, float execution_time_us)
{
	_status.timestamp = now;
	_status.generated_c_call_count = _generated_c_calls;
	_status.active_publish_count = _active_publish_count;
	_status.rejected_input_count = _rejected_input_count;
	_status.shadow_sample_count = _shadow_sample_count;
	_status.shadow_invalid_count = _shadow_invalid_count;
	_status.same_timestamp_schedule_reuse_count = _same_timestamp_schedule_reuse_count;
	const auto &transition = _payload_transition.diagnostics();
	const auto &arbitration = _hte_same_timestamp_arbitrator.diagnostics();
	_status.sequence = _last_sequence;
	_status.failure_reason = failure_reason;
	std::memcpy(_status.source_hash, gpenmpc_identity::source_hash, sizeof(_status.source_hash));
	std::memcpy(_status.firmware_contract_hash, gpenmpc_identity::firmware_contract_hash,
		    sizeof(_status.firmware_contract_hash));
	std::memcpy(_status.profile_hash, gpenmpc_identity::profile_hash, sizeof(_status.profile_hash));
	std::memcpy(_status.component_hash, gpenmpc_identity::component_hash, sizeof(_status.component_hash));
	// Hold the sole-writer lease while the module runs, including fail-closed
	// cycles. Actual reference publication is reported separately below.
	_status.writer_active = true;
	_status.active_reference_published = active_published;
	_status.input_valid = failure_reason == gpenmpc_exec_status_s::FAIL_NONE;
	_status.fail_closed = failure_reason != gpenmpc_exec_status_s::FAIL_NONE;
	_status.shadow_valid = _shadow.timestamp != 0 && now <= _shadow.timestamp + gpenmpc_identity::input_lease_us;
	_status.shadow_proposed = _status.shadow_valid && _shadow.proposed;
	_status.shadow_authorized = _status.shadow_valid && _shadow.shadow_authorized;
	_status.shadow_rejected = _status.shadow_valid && _shadow.shadow_rejected;
	_status.shadow_locked = _status.shadow_valid && _shadow.shadow_locked;
	_status.shadow_released = _status.shadow_valid && _shadow.shadow_released;
	_status.progress = static_cast<float>(_progress);
	_status.progress_rate = _have_segment ? static_cast<float>(_segment.progress_rate) : 0.f;
	_status.execution_time_us = execution_time_us;
	_status.maximum_execution_time_us = _maximum_execution_time_us;
	_status.shadow_predicted_position_risk_m = _status.shadow_valid ? _shadow.predicted_position_risk_m : NAN;
	_status.shadow_recoverability_margin = _status.shadow_valid ? _shadow.recoverability_margin : NAN;
	_status.shadow_proposed_progress_rate = _status.shadow_valid ? _shadow.proposed_progress_rate : NAN;
	_status_pub.publish(_status);

	_gpenmpc_takeoff_phase_status.timestamp = now;
	_gpenmpc_takeoff_phase_status.takeoff_status_timestamp = _takeoff_status.timestamp;
	_gpenmpc_takeoff_phase_status.evidence_timestamp = _takeoff_phase_output.evidence_timestamp_us;
	_gpenmpc_takeoff_phase_status.phase_start_timestamp = _takeoff_phase_output.phase_start_timestamp_us;
	_gpenmpc_takeoff_phase_status.ground_frame_reset_event_count = _takeoff_phase_output.reset_event_count;
	_gpenmpc_takeoff_phase_status.state = static_cast<uint8_t>(_takeoff_phase_output.state);
	_gpenmpc_takeoff_phase_status.reason = static_cast<uint8_t>(_takeoff_phase_output.reason);
	_gpenmpc_takeoff_phase_status.stock_takeoff_state = _takeoff_status.takeoff_state;
	_gpenmpc_takeoff_phase_status.liftoff_evidence = _takeoff_phase_output.liftoff_evidence;
	_gpenmpc_takeoff_phase_status.phase_scheduled = _takeoff_phase_output.phase_start_scheduled;
	_gpenmpc_takeoff_phase_status.task_progress_enabled = _takeoff_phase_output.task_progress_enabled;
	_gpenmpc_takeoff_phase_status.intent_active = _takeoff_phase_output.intent_active;
	_gpenmpc_takeoff_phase_status.conservative_height_lower_m = _takeoff_phase_output.conservative_height_lower_m;
	_gpenmpc_takeoff_phase_status.conservative_velocity_upper_d_mps =
		_takeoff_phase_output.conservative_velocity_upper_d_mps;
	_gpenmpc_takeoff_phase_status.intent_gain = _takeoff_phase_output.intent_gain;
	_gpenmpc_takeoff_phase_status.intent_velocity_d_mps = _takeoff_phase_output.intent_velocity_d_mps;
	_gpenmpc_takeoff_phase_status.intent_acceleration_d_mps2 = _takeoff_phase_output.intent_acceleration_d_mps2;
	_gpenmpc_takeoff_phase_status.intent_jerk_d_mps3 = _takeoff_phase_output.intent_jerk_d_mps3;
	_takeoff_phase_status_pub.publish(_gpenmpc_takeoff_phase_status);

	_payload_transition_status.timestamp = now;
	_payload_transition_status.ht = transition.hte_timestamp;
	_payload_transition_status.hs = transition.hte_timestamp_sample;
	_payload_transition_status.ha = transition.hte_timestamp != 0 && now >= transition.hte_timestamp
					 ? now - transition.hte_timestamp : 0;
	_payload_transition_status.svt = transition.service_timestamp;
	_payload_transition_status.hn = transition.hte_sample_count;
	_payload_transition_status.hi = transition.hte_invalid_count;
	_payload_transition_status.tu = transition.transition_update_count;
	_payload_transition_status.tc = transition.transition_completion_count;
	_payload_transition_status.tx = transition.transition_superseded_count;
	_payload_transition_status.ad = transition.availability_dropout_count;
	_payload_transition_status.ar = transition.availability_reacquisition_count;
	_payload_transition_status.ato = transition.availability_timeout_count;
	_payload_transition_status.owr = transition.overwritten_sentinel_recovery_count;
	_payload_transition_status.irr =
		transition.interleaved_valid_repeat_recovery_count;
	_payload_transition_status.ezr =
		transition.exact_publication_zoh_reuse_count;
	_payload_transition_status.sfg =
		transition.same_tick_bounded_failure_generation_count;
	_payload_transition_status.rpt =
		transition.repeated_availability_dropout_count;
	_payload_transition_status.rvs = transition.reacquisition_valid_sample_count;
	_payload_transition_status.hsf = transition.eligibility_failure_count;
	_payload_transition_status.hcf =
		transition.eligibility_last_consumed_failure_count;
	_payload_transition_status.sc = transition.service_count;
	_payload_transition_status.sq = transition.service_sequence;
	_payload_transition_status.fault = static_cast<uint8_t>(transition.fault);
	_payload_transition_status.av = static_cast<uint8_t>(transition.availability_state);
	_payload_transition_status.er = static_cast<uint8_t>(transition.eligibility_reason);
	_payload_transition_status.hlr =
		static_cast<uint8_t>(transition.eligibility_last_failure_reason);
	_payload_transition_status.hv = transition.hte_valid;
	_payload_transition_status.ea = transition.eligibility_available;
	_payload_transition_status.earmed = transition.eligibility_armed;
	_payload_transition_status.eia = transition.eligibility_in_air;
	_payload_transition_status.eld = transition.eligibility_landed;
	_payload_transition_status.edt = transition.eligibility_local_position_dt;
	_payload_transition_status.eso =
		transition.eligibility_local_position_sample_advanced;
	_payload_transition_status.ta = transition.transition_active;
	_payload_transition_status.cd = transition.convergence_dwell_active;
	_payload_transition_status.rr = transition.recovery_requested;
	_payload_transition_status.te = _payload_transition_enabled;
	_payload_transition_status.hfd = _production_host_feedforward_disabled;
	_payload_transition_status.afa = false;
	_payload_transition_status.daa = transition.direct_application_acked;
	_payload_transition_status.dac = transition.direct_application_ack_count;
	_payload_transition_status.aat = transition.application_ack_timestamp;
	_payload_transition_status.ads = transition.availability_dropout_started;
	_payload_transition_status.rs = transition.reacquisition_started;
	_payload_transition_status.lrv = transition.last_reacquisition_valid_timestamp;
	_payload_transition_status.hlf = transition.eligibility_last_failure_timestamp;
	_payload_transition_status.hcl =
		transition.eligibility_last_consumed_failure_timestamp;
	_payload_transition_status.hte = transition.hte;
	_payload_transition_status.hvv = transition.hte_variance;
	_payload_transition_status.hp = transition.hte_pre_service;
	_payload_transition_status.htg = transition.hte_target;
	_payload_transition_status.mr = transition.mass_ratio;
	_payload_transition_status.dad = transition.desired_feedforward_down_mps2;
	_payload_transition_status.aad = transition.applied_feedforward_down_mps2;
	_payload_transition_status.acr = transition.feedforward_rate_mps3;
	_payload_transition_status.hma = transition.hte_mismatch_at_ack;
	_payload_transition_status_pub.publish(_payload_transition_status);

	_hte_arbitration_status.timestamp = now;
	_hte_arbitration_status.pending_group_timestamp = arbitration.pending_group_timestamp;
	_hte_arbitration_status.last_resolved_timestamp = arbitration.last_resolved_publication_timestamp;
	_hte_arbitration_status.last_valid_sample_timestamp = arbitration.last_valid_sample_timestamp;
	_hte_arbitration_status.last_verified_publication_timestamp =
		arbitration.last_verified_publication_timestamp;
	_hte_arbitration_status.last_verified_sample_timestamp = arbitration.last_verified_sample_timestamp;
	_hte_arbitration_status.last_bounded_dropout_timestamp = arbitration.last_bounded_dropout_timestamp;
	_hte_arbitration_status.last_bounded_dropout_hold_age_us =
		arbitration.last_bounded_dropout_hold_age_us;
	_hte_arbitration_status.last_forwarded_local_position_timestamp =
		arbitration.last_forwarded_local_position_timestamp;
	_hte_arbitration_status.last_forwarded_local_position_timestamp_sample =
		arbitration.last_forwarded_local_position_timestamp_sample;
	_hte_arbitration_status.hold_ts = transition.availability_hold_timestamp;
	_hte_arbitration_status.hold_sample_ts = transition.availability_hold_timestamp_sample;
	_hte_arbitration_status.hold_age_us = transition.availability_hold_age_us;
	_hte_arbitration_status.last_generation = arbitration.last_observed_generation;
	_hte_arbitration_status.same_timestamp_valid_winners = transition.same_timestamp_valid_winner_count;
	_hte_arbitration_status.publications = arbitration.publications_seen;
	_hte_arbitration_status.generation_gaps = arbitration.generation_gap_count;
	_hte_arbitration_status.groups = arbitration.groups_resolved;
	_hte_arbitration_status.valid_winner_groups = arbitration.groups_with_valid_winner;
	_hte_arbitration_status.valid_then_dt_small = arbitration.valid_then_dt_too_small_groups;
	_hte_arbitration_status.dt_small_then_valid = arbitration.dt_too_small_then_valid_groups;
	_hte_arbitration_status.multiple_valid_groups = arbitration.multiple_valid_groups;
	_hte_arbitration_status.equivalent_valid_duplicates = arbitration.equivalent_valid_duplicates;
	_hte_arbitration_status.same_time_valid_advances = arbitration.same_time_valid_advances;
	_hte_arbitration_status.coalesced_soft_transients = arbitration.coalesced_soft_transients;
	_hte_arbitration_status.invalid_only_groups = arbitration.invalid_only_groups;
	_hte_arbitration_status.bounded_invalid_only_forwarded =
		arbitration.bounded_availability_dropouts_forwarded;
	_hte_arbitration_status.rep_snap =
		arbitration.repeated_availability_dropouts_forwarded;
	_hte_arbitration_status.accepted_dropouts =
		transition.arbitrated_invalid_only_dropout_count;
	_hte_arbitration_status.reset_epoch_changes = arbitration.coherent_reset_epoch_changes;
	_hte_arbitration_status.hard_fault_count = arbitration.hard_fault_count;
	_hte_arbitration_status.fault = static_cast<uint8_t>(arbitration.fault);
	_hte_arbitration_status.group_pending = arbitration.pending_group_timestamp != 0ULL;
	_hte_arbitration_status.hard_fault = arbitration.fault != HteSameTimestampArbitrator::Fault::None;
	_hte_arbitration_status.hold_hte = transition.availability_hold_hte;
	_hte_arbitration_status_pub.publish(_hte_arbitration_status);
}

void GPENMPCTrajectoryExec::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);
#if defined(__PX4_POSIX)
	const uint64_t host_cycle_start_ns = monotonic_wall_time_ns();
#endif
	const hrt_abstime cycle_start = hrt_absolute_time();

	// In HIL/SITL the scheduled work item can be dispatched twice while the
	// lockstep HRT clock still has the same microsecond value.  That second
	// dispatch is a legitimate zero-order hold: it has no elapsed plant/control
	// time and must not advance generated-C, reference progress, or the payload
	// transition state machine.  Count it and let the next positive-time cycle
	// consume any pending uORB update.  A true clock regression remains a hard,
	// latched TimeRegression fault.
	if (cycle_start == _last_run) {
		++_same_timestamp_schedule_reuse_count;
		perf_end(_cycle_perf);
		return;
	}

	if (cycle_start < _last_run) {
		float ignored_feedforward = 0.f;
		float ignored_rate = 0.f;
		(void)_payload_transition.step(cycle_start, 0.f, ignored_feedforward, ignored_rate);
		publish_status(_last_run, gpenmpc_exec_status_s::FAIL_HTE_TIMESTAMP, false, 0.f);
		perf_end(_cycle_perf);
		return;
	}

	const float dt = clamp_value(static_cast<float>((cycle_start - _last_run) * 1e-6), 0.001f, 0.1f);
	_last_run = cycle_start;
	uint32_t failure_reason = gpenmpc_exec_status_s::FAIL_NONE;
	bool active_published = false;
	update_shadow(cycle_start);

	trajectory_setpoint_s setpoint{};
	// Ingestion must not be short-circuited by an unrelated upstream gate.
	// Evaluate every subscription-backed gate exactly once per
	// cycle, then preserve the original first-failure precedence for the
	// externally visible status.  Generated-C remains gated on all inputs.
	uint32_t ground_failure = gpenmpc_exec_status_s::FAIL_NONE;
	uint32_t hte_failure = gpenmpc_exec_status_s::FAIL_NONE;
	uint32_t ack_failure = gpenmpc_exec_status_s::FAIL_NONE;
	uint32_t segment_failure = gpenmpc_exec_status_s::FAIL_NONE;
	uint32_t state_failure = gpenmpc_exec_status_s::FAIL_NONE;
	uint32_t takeoff_phase_failure = gpenmpc_exec_status_s::FAIL_NONE;
	uint32_t codegen_failure = gpenmpc_exec_status_s::FAIL_NONE;
	const bool ground_valid = update_ground_observer(cycle_start, dt, ground_failure);
	const bool hte_valid = update_hover_thrust_estimate(cycle_start, hte_failure);
	const bool ack_valid = update_payload_transition_ack(ack_failure);
	const bool segment_valid = update_segment(cycle_start, segment_failure);
	const bool vehicle_state_valid = state_valid(cycle_start, state_failure);
	const bool takeoff_phase_valid = update_takeoff_phase(cycle_start, takeoff_phase_failure);
	const bool inputs_valid = ground_valid && hte_valid && ack_valid
				  && segment_valid && vehicle_state_valid && takeoff_phase_valid;
	const bool codegen_valid = inputs_valid
			   ? run_generated_c(dt, cycle_start, setpoint, codegen_failure)
			   : false;
	bool valid = inputs_valid && codegen_valid;

	if (!ground_valid) {
		failure_reason = ground_failure;
	} else if (!hte_valid) {
		failure_reason = hte_failure;
	} else if (!ack_valid) {
		failure_reason = ack_failure;
	} else if (!segment_valid) {
		failure_reason = segment_failure;
	} else if (!vehicle_state_valid) {
		failure_reason = state_failure;
	} else if (!takeoff_phase_valid) {
		failure_reason = takeoff_phase_failure;
	} else if (!codegen_valid) {
		failure_reason = codegen_failure;
	}

	const float prepublish_execution_us = static_cast<float>(hrt_absolute_time() - cycle_start);
	_maximum_execution_time_us = prepublish_execution_us > _maximum_execution_time_us
				     ? prepublish_execution_us : _maximum_execution_time_us;

	if (valid && prepublish_execution_us > gpenmpc_identity::execution_budget_us) {
		valid = false;
		failure_reason = gpenmpc_exec_status_s::FAIL_OVERRUN;
	}

	if (valid) {
		int32_t control_mode = 0;

		if (param_get(_gpenmpc_control_mode_handle, &control_mode) != PX4_OK
		    || control_mode < 0 || control_mode > 2) {
			valid = false;
			failure_reason = gpenmpc_exec_status_s::FAIL_STATE;
		}
	}

	if (valid) {
		int32_t control_mode = 0;
		(void)param_get(_gpenmpc_control_mode_handle, &control_mode);
		offboard_control_mode_s offboard{};
		offboard.timestamp = cycle_start;

		if (control_mode == 0) {
			offboard.position = true;
			offboard.velocity = true;
			offboard.acceleration = true;

		} else {
			// The GPENMPC nominal/robust controller owns the sole position-control publication.
			// PX4 commander therefore disables mc_pos_control while retaining the
			// native attitude, rate and control-allocation chain.
			offboard.attitude = true;
		}
		_offboard_control_mode_pub.publish(offboard);

		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			_trajectory_setpoint_pub.publish(setpoint);
			++_active_publish_count;
			active_published = true;
		}

		for (int i = 0; i < 3; ++i) {
			_status.active_position_ned_m[i] = setpoint.position[i];
			_status.active_velocity_ned_mps[i] = setpoint.velocity[i];
			_status.active_acceleration_ned_mps2[i] = setpoint.acceleration[i];
			_status.active_jerk_ned_mps3[i] = setpoint.jerk[i];
		}

		_status.active_yaw_rad = setpoint.yaw;
		_status.active_yaw_rate_rad_s = setpoint.yawspeed;
		_status.rotor_reserve = static_cast<float>(rtrpdc_cg_Y.y[39]);

		for (int i = 0; i < 6; ++i) {
			_status.predicted_rotor_thrust_n[i] = static_cast<float>(rtrpdc_cg_Y.y[33 + i]);
		}

		if (_takeoff_phase_output.task_progress_enabled) {
			_progress = clamp_value(_progress + static_cast<double>(dt)
						* _segment.progress_rate / _segment.duration_s, 0.0, 1.0);
		}
	}

	const float execution_us = static_cast<float>(hrt_absolute_time() - cycle_start);
	float status_execution_us = execution_us;
	_maximum_execution_time_us = execution_us > _maximum_execution_time_us
				     ? execution_us : _maximum_execution_time_us;
#if defined(__PX4_POSIX)
	const float host_execution_us = static_cast<float>((monotonic_wall_time_ns() - host_cycle_start_ns) * 1e-3);
	++_host_timing_sample_count;
	_host_timing_sum_us += static_cast<double>(host_execution_us);
	_host_timing_sum_square_us2 += static_cast<double>(host_execution_us) * static_cast<double>(host_execution_us);
	_host_timing_min_us = host_execution_us < _host_timing_min_us ? host_execution_us : _host_timing_min_us;
	_host_timing_max_us = host_execution_us > _host_timing_max_us ? host_execution_us : _host_timing_max_us;
	status_execution_us = host_execution_us;
#endif
	publish_status(cycle_start, failure_reason, active_published, status_execution_us);
	perf_end(_cycle_perf);
}

int GPENMPCTrajectoryExec::task_spawn(int argc, char *argv[])
{
	// ModuleBase invokes task_spawn with px4_modules_mutex held. The canonical
	// reservation binds under that same module lock; never start a second writer.
	auto *canonical_registry = gpenmpc_rfly_stream::link_lifetime_registry();
	if (!canonical_registry || canonical_registry->native_offboard_allowed()
	    != gpenmpc_rfly_stream::LinkAccess::Read) { return PX4_ERROR; }

	GPENMPCTrajectoryExec *instance = new GPENMPCTrajectoryExec();

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) { return PX4_OK; }
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int GPENMPCTrajectoryExec::print_status()
{
	PX4_INFO("writer=%d fail=%u seq=%u codegen=%llu active=%llu shadow=%llu same_time_reuse=%llu max_us=%.1f",
		 (int)_status.writer_active, (unsigned)_status.failure_reason, (unsigned)_last_sequence,
		 (unsigned long long)_generated_c_calls, (unsigned long long)_active_publish_count,
		 (unsigned long long)_shadow_sample_count,
		 (unsigned long long)_same_timestamp_schedule_reuse_count,
		 (double)_maximum_execution_time_us);
	const auto &transition = _payload_transition.diagnostics();
	PX4_INFO("hte valid=%d h=%.5f target=%.5f age_us=%llu service=%u bounded_service=%u transition=%d ff_d=%.5f ff_rate=%.5f fault=%u",
		 (int)transition.hte_valid, (double)transition.hte, (double)transition.hte_target,
		 (unsigned long long)(_payload_transition_status.ha), (unsigned)transition.service_count,
		 (unsigned)transition.service_started_during_bounded_dropout_count,
		 (int)transition.transition_active, (double)transition.applied_feedforward_down_mps2,
		 (double)transition.feedforward_rate_mps3, (unsigned)transition.fault);
#if defined(__PX4_POSIX)
	const double host_mean_us = _host_timing_sample_count > 0
		? _host_timing_sum_us / static_cast<double>(_host_timing_sample_count) : -1.0;
	const double host_rms_us = _host_timing_sample_count > 0
		? sqrt(_host_timing_sum_square_us2 / static_cast<double>(_host_timing_sample_count)) : -1.0;
	const double host_min_us = _host_timing_sample_count > 0 ? static_cast<double>(_host_timing_min_us) : -1.0;
	const double host_max_us = _host_timing_sample_count > 0 ? static_cast<double>(_host_timing_max_us) : -1.0;
	PX4_INFO("host_monotonic_wall_us count=%llu mean=%.3f rms=%.3f min=%.3f max=%.3f (POSIX/SITL only; not board WCET)",
		 (unsigned long long)_host_timing_sample_count, host_mean_us, host_rms_us,
		 host_min_us, host_max_us);
#endif
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	return 0;
}

int GPENMPCTrajectoryExec::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GPENMPCTrajectoryExec::print_usage(const char *reason)
{
	if (reason != nullptr) { PX4_WARN("%s", reason); }
	PRINT_MODULE_DESCRIPTION("GPENMPC C-active, G_full-shadow trajectory executive.");
	PRINT_MODULE_USAGE_NAME("gpenmpc_trajectory_exec", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int gpenmpc_trajectory_exec_main(int argc, char *argv[])
{
	return GPENMPCTrajectoryExec::main(argc, argv);
}
