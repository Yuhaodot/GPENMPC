//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// gpenmpcM600CodegenDerivative_types.h
//
// Code generation for function 'gpenmpcM600CodegenDerivative'
//

#pragma once

// Include files
#include "rtwtypes.h"
#include "emlrt.h"

// Type Definitions
struct struct3_T {
  real_T inertia_nominal_kg_m2[3];
};

struct struct4_T {
  real_T frame_drag_nominal_n_per_mps2[3];
};

struct struct5_T {
  real_T tilt_deg;
};

struct struct7_T {
  real_T base_mass_kg;
};

struct struct6_T {
  struct7_T mass_properties;
};

struct struct12_T {
  real_T actual_acceleration_mps2[3];
  real_T actual_airspeed_mps;
  real_T actual_air_velocity_mps[3];
  real_T true_drag_n[3];
  real_T true_wrench[4];
  real_T structured_acceleration_mps2[3];
  real_T fast_acceleration_mps2[3];
  real_T true_mass_kg;
};

struct struct2_T {
  real_T angles_deg[6];
  real_T spin_sign[6];
  real_T arm_radius_m;
  real_T yaw_moment_arm_nominal_m;
  real_T per_rotor_thrust_upper_n;
  real_T c_actuator_time_constant_nomina;
};

struct struct1_T {
  struct2_T rotor_allocation;
  struct3_T mass_inertia;
  struct4_T aerodynamics;
  struct5_T development_limits;
};

struct struct9_T {
  real_T mass_bias_kg;
  real_T drag_scale_xyz[3];
  real_T cross_drag_matrix_n_per_mps2[9];
  real_T thrust_effectiveness_by_rotor[6];
  real_T c_external_acceleration_frequen[3];
  real_T c_external_acceleration_phases_[3];
  real_T c_external_acceleration_amplitu[3];
  real_T acceleration_bias_inertial_mps2[3];
};

struct struct10_T {
  boolean_T enabled;
  real_T c_drag_payload_coupling_n_per_m[3];
  real_T drag_tilt_gain;
  real_T turn_sideforce_n_per_mps2;
  real_T turn_centripetal_gain;
  real_T thrust_efficiency_base_loss;
  real_T thrust_efficiency_payload_gain;
  real_T thrust_efficiency_command_gain;
  real_T c_thrust_efficiency_vertical_de;
};

struct struct8_T {
  struct9_T plant_mismatch;
  struct10_T structured_residual;
};

struct struct11_T {
  real_T static_deflection_m;
  real_T damping_ratio;
  real_T maximum_deflection_m;
  real_T smooth_force_fraction_of_weight;
  real_T bump_stop_stiffness_multiplier;
  real_T c_damping_engagement_depth_frac;
};

struct struct0_T {
  struct1_T calibration;
  struct6_T profile;
  struct8_T mission;
  struct11_T contact;
};

struct struct13_T {
  boolean_T contact_active;
  real_T contact_force_n;
  real_T support_force_n;
  real_T elastic_force_n;
  real_T damping_force_n;
  real_T damping_engagement_fraction;
  real_T damping_engagement_depth_m;
  real_T bump_stop_force_n;
  real_T contact_deflection_m;
  real_T contact_compression_rate_mps;
  real_T contact_overtravel_m;
  real_T stiffness_n_per_m;
  real_T damping_n_s_per_m;
  real_T static_deflection_m;
  real_T maximum_deflection_m;
  real_T vertical_acceleration_up_mps2;
  boolean_T c_force_continuous_at_first_con;
  real_T tensile_force_n;
  boolean_T plant_truth_used_for_command;
};

// End of code generation (gpenmpcM600CodegenDerivative_types.h)
