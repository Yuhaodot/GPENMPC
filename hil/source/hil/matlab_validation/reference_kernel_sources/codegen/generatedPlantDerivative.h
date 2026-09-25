//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// generatedPlantDerivative.h
//
// Code generation for function 'generatedPlantDerivative'
//

#pragma once

// Include files
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Type Declarations
struct struct10_T;

struct struct2_T;

struct struct3_T;

struct struct4_T;

struct struct7_T;

struct struct12_T;

// Function Declarations
namespace m600check {
void generatedPlantDerivative(const emlrtStack &sp, const real_T plantState[19],
                              const real_T rotorCommandN[6],
                              const real_T reference_velocity_mps[3],
                              const real_T reference_acceleration_mps2[3],
                              real_T payloadKg, const real_T actualWindXyMps[2],
                              real_T globalTimeS,
                              real_T c_mission_plant_mismatch_mass_b,
                              const real_T c_mission_plant_mismatch_drag_s[3],
                              const real_T c_mission_plant_mismatch_cross_[9],
                              const real_T c_mission_plant_mismatch_thrust[6],
                              const real_T c_mission_plant_mismatch_extern[3],
                              const real_T d_mission_plant_mismatch_extern[3],
                              const real_T e_mission_plant_mismatch_extern[3],
                              const real_T c_mission_plant_mismatch_accele[3],
                              const struct10_T &mission_structured_residual,
                              const struct2_T &calibration_rotor_allocation,
                              const struct3_T calibration_mass_inertia,
                              const struct4_T calibration_aerodynamics,
                              const struct7_T profile_mass_properties,
                              real_T derivative[19], struct12_T *diagnostic);

}

// End of code generation (generatedPlantDerivative.h)
