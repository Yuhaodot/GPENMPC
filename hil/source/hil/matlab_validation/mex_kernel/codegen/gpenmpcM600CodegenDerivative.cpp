//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// gpenmpcM600CodegenDerivative.cpp
//
// Code generation for function 'gpenmpcM600CodegenDerivative'
//

// Include files
#include "gpenmpcM600CodegenDerivative.h"
#include "all.h"
#include "generatedPlantDerivative.h"
#include "norm.h"
#include "gpenmpcM600CodegenDerivative_data.h"
#include "gpenmpcM600CodegenDerivative_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <algorithm>

// Variable Definitions
static emlrtRSInfo emlrtRSI{
    4,                             // lineNo
    "gpenmpcM600CodegenDerivative", // fcnName
    "source/hil/matlab_validation/gpenmpcM600CodegenDerivative.m" // pathName
};

static emlrtRSInfo b_emlrtRSI{
    22,              // lineNo
    "derivativeNed", // fcnName
    "source/hil/matlab_validation/+m600check/derivativeNed.m" // pathName
};

static emlrtRSInfo c_emlrtRSI{
    13,              // lineNo
    "derivativePx4", // fcnName
    "source/hil/matlab_validation/+m600check/derivativePx4.m" // pathName
};

static emlrtRSInfo d_emlrtRSI{
    31,                   // lineNo
    "derivativeSoftware", // fcnName
    "source/hil/matlab_validation/+m600check/derivativeSoftware.m" // pathName
};

static emlrtRSInfo e_emlrtRSI{
    34,                   // lineNo
    "derivativeSoftware", // fcnName
    "source/hil/matlab_validation/+m600check/derivativeSoftware.m" // pathName
};

static emlrtRTEInfo emlrtRTEI{
    10,              // lineNo
    1,               // colNo
    "derivativePx4", // fName
    "source/hil/matlab_validation/+m600check/derivativePx4.m" // pName
};

static emlrtRTEInfo b_emlrtRTEI{
    9,                    // lineNo
    1,                    // colNo
    "derivativeSoftware", // fName
    "source/hil/matlab_validation/+m600check/derivativeSoftware.m" // pName
};

static emlrtRTEInfo c_emlrtRTEI{
    10,                   // lineNo
    1,                    // colNo
    "derivativeSoftware", // fName
    "source/hil/matlab_validation/+m600check/derivativeSoftware.m" // pName
};

static emlrtRTEInfo d_emlrtRTEI{
    11,                   // lineNo
    1,                    // colNo
    "derivativeSoftware", // fName
    "source/hil/matlab_validation/+m600check/derivativeSoftware.m" // pName
};

static emlrtRTEInfo e_emlrtRTEI{
    12,                   // lineNo
    1,                    // colNo
    "derivativeSoftware", // fName
    "source/hil/matlab_validation/+m600check/derivativeSoftware.m" // pName
};

static emlrtRTEInfo f_emlrtRTEI{
    13,                   // lineNo
    1,                    // colNo
    "derivativeSoftware", // fName
    "source/hil/matlab_validation/+m600check/derivativeSoftware.m" // pName
};

static emlrtRTEInfo g_emlrtRTEI{
    14,                   // lineNo
    1,                    // colNo
    "derivativeSoftware", // fName
    "source/hil/matlab_validation/+m600check/derivativeSoftware.m" // pName
};

static emlrtRTEInfo h_emlrtRTEI{
    16,                   // lineNo
    1,                    // colNo
    "derivativeSoftware", // fName
    "source/hil/matlab_validation/+m600check/derivativeSoftware.m" // pName
};

static emlrtRTEInfo i_emlrtRTEI{
    19,                   // lineNo
    1,                    // colNo
    "derivativeSoftware", // fName
    "source/hil/matlab_validation/+m600check/derivativeSoftware.m" // pName
};

static emlrtRTEInfo j_emlrtRTEI{
    19,              // lineNo
    1,               // colNo
    "contactKernel", // fName
    "source/hil/matlab_validation/+m600check/contactKernel.m" // pName
};

static emlrtRTEInfo k_emlrtRTEI{
    18,              // lineNo
    1,               // colNo
    "contactKernel", // fName
    "source/hil/matlab_validation/+m600check/contactKernel.m" // pName
};

static emlrtRTEInfo l_emlrtRTEI{
    17,              // lineNo
    1,               // colNo
    "contactKernel", // fName
    "source/hil/matlab_validation/+m600check/contactKernel.m" // pName
};

static emlrtRTEInfo m_emlrtRTEI{
    16,              // lineNo
    1,               // colNo
    "contactKernel", // fName
    "source/hil/matlab_validation/+m600check/contactKernel.m" // pName
};

static emlrtRTEInfo n_emlrtRTEI{
    15,              // lineNo
    1,               // colNo
    "contactKernel", // fName
    "source/hil/matlab_validation/+m600check/contactKernel.m" // pName
};

static emlrtRTEInfo o_emlrtRTEI{
    7,               // lineNo
    1,               // colNo
    "contactKernel", // fName
    "source/hil/matlab_validation/+m600check/contactKernel.m" // pName
};

// Function Definitions
void gpenmpcM600CodegenDerivative(const emlrtStack *sp, const real_T x[19],
                                 const real_T u[16], const real_T jet[12],
                                 real_T payload, const real_T wind[2],
                                 real_T b_time, const struct0_T *p,
                                 real_T dx[19], struct12_T *diagnostic,
                                 struct13_T *contact, real_T rotorCommandN[6])
{
  static const int8_T iv[6]{4, 0, 3, 5, 1, 2};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  real_T xUp[19];
  real_T jetUp[12];
  real_T reference_acceleration_mps2[3];
  real_T reference_velocity_mps[3];
  real_T damping;
  real_T dampingEngagement;
  real_T dampingForce;
  real_T deflection;
  real_T elasticForce;
  real_T engagementDepth;
  real_T mass;
  real_T normalized;
  real_T rawForce;
  real_T stiffness;
  real_T totalForce;
  real_T transition;
  int32_T k;
  boolean_T b[19];
  boolean_T b_b[12];
  boolean_T c_b[12];
  boolean_T b_u[6];
  boolean_T d_b[2];
  boolean_T e_b[2];
  boolean_T exitg1;
  boolean_T y;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  //  Fixed-size Coder entry to the existing MATLAB M600 equations and adapters.
  st.site = &emlrtRSI;
  // DERIVATIVENED Exact current LiveHilPlantService frame/sign adaptation.
  //  xNed=[world NED p;world NED v;q_wxyz body-to-NED;FRD omega;thrust N].
  //  This is NOT the generic RflySim body-velocity/RPM state layout.
  //  Contact and diagnostics retain explicitly named z-up/world quantities.
  std::copy(&x[0], &x[19], &xUp[0]);
  xUp[2] = -x[2];
  xUp[5] = -x[5];
  xUp[7] = -x[7];
  xUp[8] = -x[8];
  xUp[10] = -x[10];
  xUp[11] = -x[11];
  for (int32_T i{0}; i < 4; i++) {
    int8_T b_i;
    b_i = static_cast<int8_T>(3 * i + 1);
    jetUp[b_i - 1] = jet[b_i - 1];
    jetUp[b_i] = jet[b_i];
    jetUp[b_i + 1] = -jet[b_i + 1];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&st);
    }
  }
  b_st.site = &b_emlrtRSI;
  // DERIVATIVEPX4 Exact accepted normalized-thrust feedback adaptation.
  //  controls: 16-by-1 HIL actuator values; channels 1:6 must be in [0,1].
  //  Unused 7:16 may be NaN, matching adaptV7NominalControlFeedback. The first
  //  six represent linear normalized THRUST, not PWM-to-speed, RPM or voltage.
  for (int32_T i{0}; i < 6; i++) {
    normalized = u[i];
    b_u[i] =
        (!muDoubleScalarIsInf(normalized) && !muDoubleScalarIsNaN(normalized));
  }
  if (coder::all(b_u)) {
    for (int32_T i{0}; i < 6; i++) {
      b_u[i] = (u[i] >= 0.0);
    }
    if (coder::all(b_u)) {
      for (int32_T i{0}; i < 6; i++) {
        b_u[i] = (u[i] <= 1.0);
      }
      if (!coder::all(b_u)) {
        emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI,
                                      "Coder:builtins:AssertionFailed",
                                      "Coder:builtins:AssertionFailed", 0);
      }
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
  } else {
    emlrtErrorWithMessageIdR2018a(&b_st, &emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  transition = p->calibration.rotor_allocation.per_rotor_thrust_upper_n;
  for (int32_T i{0}; i < 6; i++) {
    rotorCommandN[i] = u[iv[i]] * transition;
  }
  c_st.site = &c_emlrtRSI;
  // DERIVATIVESOFTWARE Reuse current 19-state MATLAB plant without physics
  // fork.
  //  x: 19-by-1 z-up state [world p;world v;q_wxyz;body rates;6 thrust states
  //  N]. rotorCommandN: 6-by-1 in software rotor-angle order, not PX4
  //  order/RPM. referenceJet: 12-by-1 z-up [p;v;a;jerk]. Only v/a drive
  //  residual model. p is the fixed numeric nested struct from
  //  m600check.packParameters.
  for (int32_T i{0}; i < 19; i++) {
    normalized = xUp[i];
    b[i] =
        (!muDoubleScalarIsInf(normalized) && !muDoubleScalarIsNaN(normalized));
  }
  y = true;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k <= 18)) {
    if (!b[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (!y) {
    emlrtErrorWithMessageIdR2018a(&c_st, &b_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  for (int32_T i{0}; i < 6; i++) {
    normalized = rotorCommandN[i];
    b_u[i] =
        (!muDoubleScalarIsInf(normalized) && !muDoubleScalarIsNaN(normalized));
  }
  if (!coder::all(b_u)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  for (int32_T i{0}; i < 12; i++) {
    normalized = jetUp[i];
    b_b[i] = muDoubleScalarIsInf(normalized);
    c_b[i] = muDoubleScalarIsNaN(normalized);
  }
  y = true;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k <= 11)) {
    if (b_b[k] || c_b[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (!y) {
    emlrtErrorWithMessageIdR2018a(&c_st, &d_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  d_b[0] = muDoubleScalarIsInf(wind[0]);
  e_b[0] = muDoubleScalarIsNaN(wind[0]);
  d_b[1] = muDoubleScalarIsInf(wind[1]);
  e_b[1] = muDoubleScalarIsNaN(wind[1]);
  y = true;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k <= 1)) {
    if (d_b[k] || e_b[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (!y) {
    emlrtErrorWithMessageIdR2018a(&c_st, &e_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (muDoubleScalarIsInf(payload) || muDoubleScalarIsNaN(payload) ||
      !(payload >= 0.0) ||
      (muDoubleScalarIsInf(b_time) || muDoubleScalarIsNaN(b_time))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &f_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (!(coder::b_norm(&xUp[6]) > 1.0E-15)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &g_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  for (int32_T i{0}; i < 6; i++) {
    b_u[i] = (rotorCommandN[i] >= 0.0);
  }
  if (coder::all(b_u)) {
    for (int32_T i{0}; i < 6; i++) {
      b_u[i] = (rotorCommandN[i] <= transition);
    }
    if (!coder::all(b_u)) {
      emlrtErrorWithMessageIdR2018a(&c_st, &h_emlrtRTEI,
                                    "Coder:builtins:AssertionFailed",
                                    "Coder:builtins:AssertionFailed", 0);
    }
  } else {
    emlrtErrorWithMessageIdR2018a(&c_st, &h_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  mass = (p->profile.mass_properties.base_mass_kg + payload) +
         p->mission.plant_mismatch.mass_bias_kg;
  if (muDoubleScalarIsInf(mass) || muDoubleScalarIsNaN(mass) || !(mass > 0.0)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &i_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  reference_velocity_mps[0] = jetUp[3];
  reference_acceleration_mps2[0] = jetUp[6];
  reference_velocity_mps[1] = jetUp[4];
  reference_acceleration_mps2[1] = jetUp[7];
  reference_velocity_mps[2] = jetUp[5];
  reference_acceleration_mps2[2] = jetUp[8];
  //  Exact authoritative function and its allocation/residual/quaternion
  //  dependencies remain on the source path; not copied or altered here.
  //  Coder rejects the original allocation's incremental struct construction.
  //  This explicit counterpart changes structure layout only, not equations.
  d_st.site = &d_emlrtRSI;
  m600check::generatedPlantDerivative(
      d_st, xUp, rotorCommandN, reference_velocity_mps,
      reference_acceleration_mps2, payload, wind, b_time,
      p->mission.plant_mismatch.mass_bias_kg,
      p->mission.plant_mismatch.drag_scale_xyz,
      p->mission.plant_mismatch.cross_drag_matrix_n_per_mps2,
      p->mission.plant_mismatch.thrust_effectiveness_by_rotor,
      p->mission.plant_mismatch.c_external_acceleration_frequen,
      p->mission.plant_mismatch.c_external_acceleration_phases_,
      p->mission.plant_mismatch.c_external_acceleration_amplitu,
      p->mission.plant_mismatch.acceleration_bias_inertial_mps2,
      p->mission.structured_residual, p->calibration.rotor_allocation,
      p->calibration.mass_inertia, p->calibration.aerodynamics,
      p->profile.mass_properties, dx, diagnostic);
  d_st.site = &e_emlrtRSI;
  // CONTACTKERNEL Numeric-only exact port of compliantContactState arithmetic.
  //  Oracle SHA
  //  D90339C9D83373C121AC3A4C5826E72420B6F29C1127344685626097AEC4C095. This
  //  removes only arguments/default/dynamic-field/schema-string handling. It
  //  preserves every numeric diagnostic and the original proxy intervals.
  y = true;
  k = 0;
  exitg1 = false;
  while (!exitg1 && (k <= 18)) {
    if (!b[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (!y) {
    emlrtErrorWithMessageIdR2018a(&d_st, &o_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (!(p->contact.static_deflection_m >= 0.02) ||
      !(p->contact.static_deflection_m <= 0.05)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &n_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (!(p->contact.damping_ratio >= 0.8) ||
      !(p->contact.damping_ratio <= 1.2)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &m_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (!(p->contact.maximum_deflection_m > p->contact.static_deflection_m)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &l_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (!(p->contact.smooth_force_fraction_of_weight >= 0.005) ||
      !(p->contact.smooth_force_fraction_of_weight <= 0.05)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &k_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (!(p->contact.bump_stop_stiffness_multiplier >= 1.0) ||
      !(muDoubleScalarAbs(p->contact.c_damping_engagement_depth_frac - 1.0) <=
        1.0E-15)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &j_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  stiffness = mass * 9.80665 / p->contact.static_deflection_m;
  damping =
      2.0 * p->contact.damping_ratio * muDoubleScalarSqrt(stiffness * mass);
  deflection = muDoubleScalarMax(0.0, p->contact.static_deflection_m - (-x[2]));
  engagementDepth = p->contact.static_deflection_m *
                    p->contact.c_damping_engagement_depth_frac;
  normalized = muDoubleScalarMin(
      muDoubleScalarMax(deflection / engagementDepth, 0.0), 1.0);
  dampingEngagement = normalized * normalized * (3.0 - 2.0 * normalized);
  elasticForce = stiffness * deflection;
  dampingForce = damping * dampingEngagement * x[5];
  if (deflection > 0.0) {
    rawForce = elasticForce + dampingForce;
  } else {
    rawForce = 0.0;
  }
  transition = p->contact.smooth_force_fraction_of_weight * mass * 9.80665;
  if (rawForce <= 0.0) {
    rawForce = 0.0;
  } else if (!(rawForce >= transition)) {
    normalized = rawForce / transition;
    rawForce = transition * (normalized * normalized) * (2.0 - normalized);
  }
  normalized =
      muDoubleScalarMax(0.0, deflection - p->contact.maximum_deflection_m);
  transition =
      p->contact.bump_stop_stiffness_multiplier * stiffness * normalized;
  totalForce = rawForce + transition;
  contact->contact_active = (totalForce > 1.0E-9);
  contact->contact_force_n = totalForce;
  contact->support_force_n = rawForce;
  contact->elastic_force_n = elasticForce;
  contact->damping_force_n = dampingForce;
  contact->damping_engagement_fraction = dampingEngagement;
  contact->damping_engagement_depth_m = engagementDepth;
  contact->bump_stop_force_n = transition;
  contact->contact_deflection_m = deflection;
  contact->contact_compression_rate_mps = x[5];
  contact->contact_overtravel_m = normalized;
  contact->stiffness_n_per_m = stiffness;
  contact->damping_n_s_per_m = damping;
  contact->static_deflection_m = p->contact.static_deflection_m;
  contact->maximum_deflection_m = p->contact.maximum_deflection_m;
  contact->vertical_acceleration_up_mps2 = totalForce / mass;
  contact->c_force_continuous_at_first_con = true;
  contact->tensile_force_n = 0.0;
  contact->plant_truth_used_for_command = false;
  dx[5] += contact->vertical_acceleration_up_mps2;
  diagnostic->actual_acceleration_mps2[2] +=
      contact->vertical_acceleration_up_mps2;
  dx[2] = -dx[2];
  dx[5] = -dx[5];
  dx[7] = -dx[7];
  dx[8] = -dx[8];
  dx[10] = -dx[10];
  dx[11] = -dx[11];
}

// End of code generation (gpenmpcM600CodegenDerivative.cpp)
