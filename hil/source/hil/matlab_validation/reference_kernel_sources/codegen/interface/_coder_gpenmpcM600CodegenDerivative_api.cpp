//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_gpenmpcM600CodegenDerivative_api.cpp
//
// Code generation for function '_coder_gpenmpcM600CodegenDerivative_api'
//

// Include files
#include "_coder_gpenmpcM600CodegenDerivative_api.h"
#include "gpenmpcM600CodegenDerivative.h"
#include "gpenmpcM600CodegenDerivative_data.h"
#include "gpenmpcM600CodegenDerivative_types.h"
#include "rt_nonfinite.h"

// Function Declarations
static void b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3]);

static real_T (*b_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[16];

static real_T (*b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[16];

static const mxArray *b_emlrt_marshallOut(real_T u[6]);

static void c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[9]);

static real_T (*c_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[12];

static real_T (*c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[12];

static void d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[6]);

static real_T d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier);

static real_T (*e_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[2];

static real_T (*e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[2];

static void e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3]);

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[19];

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[6]);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct10_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct0_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct0_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct1_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct2_T &y);

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier))[19];

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct8_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct9_T &y);

static const mxArray *emlrt_marshallOut(real_T u[19]);

static const mxArray *emlrt_marshallOut(const struct12_T &u);

static const mxArray *emlrt_marshallOut(const struct13_T &u);

static void f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[9]);

static struct3_T f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static struct4_T g_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static struct5_T h_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static struct6_T i_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static struct7_T j_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static boolean_T k_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static struct11_T l_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId);

static real_T (*m_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[19];

static real_T (*n_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[16];

static real_T (*o_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[12];

static real_T p_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T (*q_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[2];

static boolean_T r_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

// Function Definitions
static real_T (*b_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[16]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[16];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

static real_T (*b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[16]
{
  real_T(*y)[16];
  y = n_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3])
{
  e_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *b_emlrt_marshallOut(real_T u[6])
{
  static const int32_T i{0};
  static const int32_T i1{6};
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  existingData = emlrtMxGetData((mxArray *)m);
  if (existingData != (void *)&u[0]) {
    emlrtFreeMex(existingData);
  }
  emlrtMxSetData((mxArray *)m, &u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static void c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[9])
{
  f_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T (*c_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[12]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[12];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = c_emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[12]
{
  real_T(*y)[12];
  y = o_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

static void d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[6])
{
  static const int32_T dims{6};
  real_T(*r)[6];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[6])emlrtMxGetData(src);
  for (int32_T i{0}; i < 6; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static real_T d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = p_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*e_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[2]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[2];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = e_emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

static real_T (*e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[2]
{
  real_T(*y)[2];
  y = q_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims{3};
  real_T(*r)[3];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier))[19]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[19];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[19]
{
  real_T(*y)[19];
  y = m_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct9_T &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[8]{"mass_bias_kg",
                                     "drag_scale_xyz",
                                     "cross_drag_matrix_n_per_mps2",
                                     "thrust_effectiveness_by_rotor",
                                     "external_acceleration_frequencies_hz",
                                     "external_acceleration_phases_rad",
                                     "external_acceleration_amplitude_mps2",
                                     "acceleration_bias_inertial_mps2"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 8,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "mass_bias_kg";
  y.mass_bias_kg =
      d_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 0, "mass_bias_kg")),
                         &thisId);
  thisId.fIdentifier = "drag_scale_xyz";
  b_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 1,
                                                    "drag_scale_xyz")),
                     &thisId, y.drag_scale_xyz);
  thisId.fIdentifier = "cross_drag_matrix_n_per_mps2";
  c_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 2,
                                     "cross_drag_matrix_n_per_mps2")),
      &thisId, y.cross_drag_matrix_n_per_mps2);
  thisId.fIdentifier = "thrust_effectiveness_by_rotor";
  emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3,
                                     "thrust_effectiveness_by_rotor")),
      &thisId, y.thrust_effectiveness_by_rotor);
  thisId.fIdentifier = "external_acceleration_frequencies_hz";
  b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 4,
                                     "external_acceleration_frequencies_hz")),
      &thisId, y.c_external_acceleration_frequen);
  thisId.fIdentifier = "external_acceleration_phases_rad";
  b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 5,
                                     "external_acceleration_phases_rad")),
      &thisId, y.c_external_acceleration_phases_);
  thisId.fIdentifier = "external_acceleration_amplitude_mps2";
  b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 6,
                                     "external_acceleration_amplitude_mps2")),
      &thisId, y.c_external_acceleration_amplitu);
  thisId.fIdentifier = "acceleration_bias_inertial_mps2";
  b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 7,
                                     "acceleration_bias_inertial_mps2")),
      &thisId, y.acceleration_bias_inertial_mps2);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct8_T &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[2]{"plant_mismatch", "structured_residual"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 2,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "plant_mismatch";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0,
                                                  "plant_mismatch")),
                   &thisId, y.plant_mismatch);
  thisId.fIdentifier = "structured_residual";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 1,
                                                  "structured_residual")),
                   &thisId, y.structured_residual);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct0_T &y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId, y);
  emlrtDestroyArray(&b_nullptr);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct0_T &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[4]{"calibration", "profile", "mission",
                                     "contact"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 4,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "calibration";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0,
                                                  "calibration")),
                   &thisId, y.calibration);
  thisId.fIdentifier = "profile";
  y.profile = i_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 1, "profile")),
      &thisId);
  thisId.fIdentifier = "mission";
  emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 2, "mission")),
      &thisId, y.mission);
  thisId.fIdentifier = "contact";
  y.contact = l_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3, "contact")),
      &thisId);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct1_T &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[4]{"rotor_allocation", "mass_inertia",
                                     "aerodynamics", "development_limits"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 4,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "rotor_allocation";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0,
                                                  "rotor_allocation")),
                   &thisId, y.rotor_allocation);
  thisId.fIdentifier = "mass_inertia";
  y.mass_inertia =
      f_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 1, "mass_inertia")),
                         &thisId);
  thisId.fIdentifier = "aerodynamics";
  y.aerodynamics =
      g_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 2, "aerodynamics")),
                         &thisId);
  thisId.fIdentifier = "development_limits";
  y.development_limits = h_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3,
                                     "development_limits")),
      &thisId);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct2_T &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[6]{"angles_deg",
                                     "spin_sign",
                                     "arm_radius_m",
                                     "yaw_moment_arm_nominal_m",
                                     "per_rotor_thrust_upper_n",
                                     "actuator_time_constant_nominal_s"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 6,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "angles_deg";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0,
                                                  "angles_deg")),
                   &thisId, y.angles_deg);
  thisId.fIdentifier = "spin_sign";
  emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 1, "spin_sign")),
      &thisId, y.spin_sign);
  thisId.fIdentifier = "arm_radius_m";
  y.arm_radius_m =
      d_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 2, "arm_radius_m")),
                         &thisId);
  thisId.fIdentifier = "yaw_moment_arm_nominal_m";
  y.yaw_moment_arm_nominal_m = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3,
                                     "yaw_moment_arm_nominal_m")),
      &thisId);
  thisId.fIdentifier = "per_rotor_thrust_upper_n";
  y.per_rotor_thrust_upper_n = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 4,
                                     "per_rotor_thrust_upper_n")),
      &thisId);
  thisId.fIdentifier = "actuator_time_constant_nominal_s";
  y.c_actuator_time_constant_nomina = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 5,
                                     "actuator_time_constant_nominal_s")),
      &thisId);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[6])
{
  d_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct10_T &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[9]{"enabled",
                                     "drag_payload_coupling_n_per_mps2_xyz",
                                     "drag_tilt_gain",
                                     "turn_sideforce_n_per_mps2",
                                     "turn_centripetal_gain",
                                     "thrust_efficiency_base_loss",
                                     "thrust_efficiency_payload_gain",
                                     "thrust_efficiency_command_gain",
                                     "thrust_efficiency_vertical_demand_gain"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 9,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "enabled";
  y.enabled = k_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0, "enabled")),
      &thisId);
  thisId.fIdentifier = "drag_payload_coupling_n_per_mps2_xyz";
  b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 1,
                                     "drag_payload_coupling_n_per_mps2_xyz")),
      &thisId, y.c_drag_payload_coupling_n_per_m);
  thisId.fIdentifier = "drag_tilt_gain";
  y.drag_tilt_gain =
      d_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, 0, 2, "drag_tilt_gain")),
                         &thisId);
  thisId.fIdentifier = "turn_sideforce_n_per_mps2";
  y.turn_sideforce_n_per_mps2 = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3,
                                     "turn_sideforce_n_per_mps2")),
      &thisId);
  thisId.fIdentifier = "turn_centripetal_gain";
  y.turn_centripetal_gain = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 4,
                                     "turn_centripetal_gain")),
      &thisId);
  thisId.fIdentifier = "thrust_efficiency_base_loss";
  y.thrust_efficiency_base_loss = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 5,
                                     "thrust_efficiency_base_loss")),
      &thisId);
  thisId.fIdentifier = "thrust_efficiency_payload_gain";
  y.thrust_efficiency_payload_gain = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 6,
                                     "thrust_efficiency_payload_gain")),
      &thisId);
  thisId.fIdentifier = "thrust_efficiency_command_gain";
  y.thrust_efficiency_command_gain = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 7,
                                     "thrust_efficiency_command_gain")),
      &thisId);
  thisId.fIdentifier = "thrust_efficiency_vertical_demand_gain";
  y.c_thrust_efficiency_vertical_de = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 8,
                                     "thrust_efficiency_vertical_demand_gain")),
      &thisId);
  emlrtDestroyArray(&u);
}

static const mxArray *emlrt_marshallOut(real_T u[19])
{
  static const int32_T i{0};
  static const int32_T i1{19};
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  existingData = emlrtMxGetData((mxArray *)m);
  if (existingData != (void *)&u[0]) {
    emlrtFreeMex(existingData);
  }
  emlrtMxSetData((mxArray *)m, &u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static const mxArray *emlrt_marshallOut(const struct12_T &u)
{
  static const int32_T i{3};
  static const int32_T i1{3};
  static const int32_T i2{3};
  static const int32_T i3{4};
  static const int32_T i4{3};
  static const int32_T i5{3};
  static const char_T *sv[8]{"actual_acceleration_mps2",
                             "actual_airspeed_mps",
                             "actual_air_velocity_mps",
                             "true_drag_n",
                             "true_wrench",
                             "structured_acceleration_mps2",
                             "fast_acceleration_mps2",
                             "true_mass_kg"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *i_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  y = nullptr;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&sv[0]));
  b_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u.actual_acceleration_mps2[0];
  pData[1] = u.actual_acceleration_mps2[1];
  pData[2] = u.actual_acceleration_mps2[2];
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "actual_acceleration_mps2", b_y, 0);
  c_y = nullptr;
  m = emlrtCreateDoubleScalar(u.actual_airspeed_mps);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, "actual_airspeed_mps", c_y, 1);
  d_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i1, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u.actual_air_velocity_mps[0];
  pData[1] = u.actual_air_velocity_mps[1];
  pData[2] = u.actual_air_velocity_mps[2];
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, "actual_air_velocity_mps", d_y, 2);
  e_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i2, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u.true_drag_n[0];
  pData[1] = u.true_drag_n[1];
  pData[2] = u.true_drag_n[2];
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, "true_drag_n", e_y, 3);
  f_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i3, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u.true_wrench[0];
  pData[1] = u.true_wrench[1];
  pData[2] = u.true_wrench[2];
  pData[3] = u.true_wrench[3];
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, "true_wrench", f_y, 4);
  g_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i4, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u.structured_acceleration_mps2[0];
  pData[1] = u.structured_acceleration_mps2[1];
  pData[2] = u.structured_acceleration_mps2[2];
  emlrtAssign(&g_y, m);
  emlrtSetFieldR2017b(y, 0, "structured_acceleration_mps2", g_y, 5);
  h_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i5, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u.fast_acceleration_mps2[0];
  pData[1] = u.fast_acceleration_mps2[1];
  pData[2] = u.fast_acceleration_mps2[2];
  emlrtAssign(&h_y, m);
  emlrtSetFieldR2017b(y, 0, "fast_acceleration_mps2", h_y, 6);
  i_y = nullptr;
  m = emlrtCreateDoubleScalar(u.true_mass_kg);
  emlrtAssign(&i_y, m);
  emlrtSetFieldR2017b(y, 0, "true_mass_kg", i_y, 7);
  return y;
}

static const mxArray *emlrt_marshallOut(const struct13_T &u)
{
  static const char_T *sv[19]{"contact_active",
                              "contact_force_n",
                              "support_force_n",
                              "elastic_force_n",
                              "damping_force_n",
                              "damping_engagement_fraction",
                              "damping_engagement_depth_m",
                              "bump_stop_force_n",
                              "contact_deflection_m",
                              "contact_compression_rate_mps",
                              "contact_overtravel_m",
                              "stiffness_n_per_m",
                              "damping_n_s_per_m",
                              "static_deflection_m",
                              "maximum_deflection_m",
                              "vertical_acceleration_up_mps2",
                              "force_continuous_at_first_contact",
                              "tensile_force_n",
                              "plant_truth_used_for_command"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *i_y;
  const mxArray *j_y;
  const mxArray *k_y;
  const mxArray *l_y;
  const mxArray *m;
  const mxArray *m_y;
  const mxArray *n_y;
  const mxArray *o_y;
  const mxArray *p_y;
  const mxArray *q_y;
  const mxArray *r_y;
  const mxArray *s_y;
  const mxArray *t_y;
  const mxArray *y;
  y = nullptr;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 19, (const char_T **)&sv[0]));
  b_y = nullptr;
  m = emlrtCreateLogicalScalar(u.contact_active);
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "contact_active", b_y, 0);
  c_y = nullptr;
  m = emlrtCreateDoubleScalar(u.contact_force_n);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, "contact_force_n", c_y, 1);
  d_y = nullptr;
  m = emlrtCreateDoubleScalar(u.support_force_n);
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, "support_force_n", d_y, 2);
  e_y = nullptr;
  m = emlrtCreateDoubleScalar(u.elastic_force_n);
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, "elastic_force_n", e_y, 3);
  f_y = nullptr;
  m = emlrtCreateDoubleScalar(u.damping_force_n);
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, "damping_force_n", f_y, 4);
  g_y = nullptr;
  m = emlrtCreateDoubleScalar(u.damping_engagement_fraction);
  emlrtAssign(&g_y, m);
  emlrtSetFieldR2017b(y, 0, "damping_engagement_fraction", g_y, 5);
  h_y = nullptr;
  m = emlrtCreateDoubleScalar(u.damping_engagement_depth_m);
  emlrtAssign(&h_y, m);
  emlrtSetFieldR2017b(y, 0, "damping_engagement_depth_m", h_y, 6);
  i_y = nullptr;
  m = emlrtCreateDoubleScalar(u.bump_stop_force_n);
  emlrtAssign(&i_y, m);
  emlrtSetFieldR2017b(y, 0, "bump_stop_force_n", i_y, 7);
  j_y = nullptr;
  m = emlrtCreateDoubleScalar(u.contact_deflection_m);
  emlrtAssign(&j_y, m);
  emlrtSetFieldR2017b(y, 0, "contact_deflection_m", j_y, 8);
  k_y = nullptr;
  m = emlrtCreateDoubleScalar(u.contact_compression_rate_mps);
  emlrtAssign(&k_y, m);
  emlrtSetFieldR2017b(y, 0, "contact_compression_rate_mps", k_y, 9);
  l_y = nullptr;
  m = emlrtCreateDoubleScalar(u.contact_overtravel_m);
  emlrtAssign(&l_y, m);
  emlrtSetFieldR2017b(y, 0, "contact_overtravel_m", l_y, 10);
  m_y = nullptr;
  m = emlrtCreateDoubleScalar(u.stiffness_n_per_m);
  emlrtAssign(&m_y, m);
  emlrtSetFieldR2017b(y, 0, "stiffness_n_per_m", m_y, 11);
  n_y = nullptr;
  m = emlrtCreateDoubleScalar(u.damping_n_s_per_m);
  emlrtAssign(&n_y, m);
  emlrtSetFieldR2017b(y, 0, "damping_n_s_per_m", n_y, 12);
  o_y = nullptr;
  m = emlrtCreateDoubleScalar(u.static_deflection_m);
  emlrtAssign(&o_y, m);
  emlrtSetFieldR2017b(y, 0, "static_deflection_m", o_y, 13);
  p_y = nullptr;
  m = emlrtCreateDoubleScalar(u.maximum_deflection_m);
  emlrtAssign(&p_y, m);
  emlrtSetFieldR2017b(y, 0, "maximum_deflection_m", p_y, 14);
  q_y = nullptr;
  m = emlrtCreateDoubleScalar(u.vertical_acceleration_up_mps2);
  emlrtAssign(&q_y, m);
  emlrtSetFieldR2017b(y, 0, "vertical_acceleration_up_mps2", q_y, 15);
  r_y = nullptr;
  m = emlrtCreateLogicalScalar(true);
  emlrtAssign(&r_y, m);
  emlrtSetFieldR2017b(y, 0, "force_continuous_at_first_contact", r_y, 16);
  s_y = nullptr;
  m = emlrtCreateDoubleScalar(0.0);
  emlrtAssign(&s_y, m);
  emlrtSetFieldR2017b(y, 0, "tensile_force_n", s_y, 17);
  t_y = nullptr;
  m = emlrtCreateLogicalScalar(false);
  emlrtAssign(&t_y, m);
  emlrtSetFieldR2017b(y, 0, "plant_truth_used_for_command", t_y, 18);
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[9])
{
  static const int32_T dims[2]{3, 3};
  real_T(*r)[9];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[9])emlrtMxGetData(src);
  for (int32_T i{0}; i < 9; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static struct3_T f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  static const char_T *fieldNames{"inertia_nominal_kg_m2"};
  emlrtMsgIdentifier thisId;
  struct3_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 1,
                         (const char_T **)&fieldNames, 0U, (const void *)&dims);
  thisId.fIdentifier = "inertia_nominal_kg_m2";
  b_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0,
                                                    "inertia_nominal_kg_m2")),
                     &thisId, y.inertia_nominal_kg_m2);
  emlrtDestroyArray(&u);
  return y;
}

static struct4_T g_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  static const char_T *fieldNames{"frame_drag_nominal_n_per_mps2"};
  emlrtMsgIdentifier thisId;
  struct4_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 1,
                         (const char_T **)&fieldNames, 0U, (const void *)&dims);
  thisId.fIdentifier = "frame_drag_nominal_n_per_mps2";
  b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0,
                                     "frame_drag_nominal_n_per_mps2")),
      &thisId, y.frame_drag_nominal_n_per_mps2);
  emlrtDestroyArray(&u);
  return y;
}

static struct5_T h_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  static const char_T *fieldNames{"tilt_deg"};
  emlrtMsgIdentifier thisId;
  struct5_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 1,
                         (const char_T **)&fieldNames, 0U, (const void *)&dims);
  thisId.fIdentifier = "tilt_deg";
  y.tilt_deg = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0, "tilt_deg")),
      &thisId);
  emlrtDestroyArray(&u);
  return y;
}

static struct6_T i_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  static const char_T *fieldNames{"mass_properties"};
  emlrtMsgIdentifier thisId;
  struct6_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 1,
                         (const char_T **)&fieldNames, 0U, (const void *)&dims);
  thisId.fIdentifier = "mass_properties";
  y.mass_properties =
      j_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, 0, 0, "mass_properties")),
                         &thisId);
  emlrtDestroyArray(&u);
  return y;
}

static struct7_T j_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  static const char_T *fieldNames{"base_mass_kg"};
  emlrtMsgIdentifier thisId;
  struct7_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 1,
                         (const char_T **)&fieldNames, 0U, (const void *)&dims);
  thisId.fIdentifier = "base_mass_kg";
  y.base_mass_kg =
      d_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 0, "base_mass_kg")),
                         &thisId);
  emlrtDestroyArray(&u);
  return y;
}

static boolean_T k_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = r_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static struct11_T l_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[6]{
      "static_deflection_m",
      "damping_ratio",
      "maximum_deflection_m",
      "smooth_force_fraction_of_weight",
      "bump_stop_stiffness_multiplier",
      "damping_engagement_depth_fraction_of_static_deflection"};
  emlrtMsgIdentifier thisId;
  struct11_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 6,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "static_deflection_m";
  y.static_deflection_m = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0,
                                     "static_deflection_m")),
      &thisId);
  thisId.fIdentifier = "damping_ratio";
  y.damping_ratio =
      d_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        0, 1, "damping_ratio")),
                         &thisId);
  thisId.fIdentifier = "maximum_deflection_m";
  y.maximum_deflection_m = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 2,
                                     "maximum_deflection_m")),
      &thisId);
  thisId.fIdentifier = "smooth_force_fraction_of_weight";
  y.smooth_force_fraction_of_weight = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3,
                                     "smooth_force_fraction_of_weight")),
      &thisId);
  thisId.fIdentifier = "bump_stop_stiffness_multiplier";
  y.bump_stop_stiffness_multiplier = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 4,
                                     "bump_stop_stiffness_multiplier")),
      &thisId);
  thisId.fIdentifier = "damping_engagement_depth_fraction_of_static_deflection";
  y.c_damping_engagement_depth_frac = d_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b(
          (emlrtConstCTX)&sp, u, 0, 5,
          "damping_engagement_depth_fraction_of_static_deflection")),
      &thisId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*m_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[19]
{
  static const int32_T dims{19};
  real_T(*ret)[19];
  int32_T i;
  boolean_T b{false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[19])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*n_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[16]
{
  static const int32_T dims{16};
  real_T(*ret)[16];
  int32_T i;
  boolean_T b{false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[16])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*o_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[12]
{
  static const int32_T dims{12};
  real_T(*ret)[12];
  int32_T i;
  boolean_T b{false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[12])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T p_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *static_cast<real_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*q_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[2]
{
  static const int32_T dims{2};
  real_T(*ret)[2];
  int32_T i;
  boolean_T b{false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[2])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static boolean_T r_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  boolean_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "logical", false, 0U,
                          (const void *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

void gpenmpcM600CodegenDerivative_api(const mxArray *const prhs[7], int32_T nlhs,
                                     const mxArray *plhs[4])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  struct0_T p;
  struct12_T diagnostic;
  struct13_T contact;
  real_T(*dx)[19];
  real_T(*x)[19];
  real_T(*u)[16];
  real_T(*jet)[12];
  real_T(*rotorCommandN)[6];
  real_T(*wind)[2];
  real_T b_time;
  real_T payload;
  st.tls = emlrtRootTLSGlobal;
  dx = (real_T(*)[19])mxMalloc(sizeof(real_T[19]));
  rotorCommandN = (real_T(*)[6])mxMalloc(sizeof(real_T[6]));
  // Marshall function inputs
  x = emlrt_marshallIn(st, emlrtAlias(prhs[0]), "x");
  u = b_emlrt_marshallIn(st, emlrtAlias(prhs[1]), "u");
  jet = c_emlrt_marshallIn(st, emlrtAlias(prhs[2]), "jet");
  payload = d_emlrt_marshallIn(st, emlrtAliasP(prhs[3]), "payload");
  wind = e_emlrt_marshallIn(st, emlrtAlias(prhs[4]), "wind");
  b_time = d_emlrt_marshallIn(st, emlrtAliasP(prhs[5]), "time");
  emlrt_marshallIn(st, emlrtAliasP(prhs[6]), "p", p);
  // Invoke the target function
  gpenmpcM600CodegenDerivative(&st, *x, *u, *jet, payload, *wind, b_time, &p,
                              *dx, &diagnostic, &contact, *rotorCommandN);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(*dx);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(diagnostic);
  }
  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(contact);
  }
  if (nlhs > 3) {
    plhs[3] = b_emlrt_marshallOut(*rotorCommandN);
  }
}

// End of code generation (_coder_gpenmpcM600CodegenDerivative_api.cpp)
