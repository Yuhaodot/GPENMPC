//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// generatedPlantDerivative.cpp
//
// Code generation for function 'generatedPlantDerivative'
//

// Include files
#include "generatedPlantDerivative.h"
#include "mrdivide_helper.h"
#include "norm.h"
#include "gpenmpcM600CodegenDerivative_data.h"
#include "gpenmpcM600CodegenDerivative_types.h"
#include "rt_nonfinite.h"
#include "sumMatrixIncludeNaN.h"
#include "svd.h"
#include "warning.h"
#include "blas.h"
#include "mwmathutil.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <emmintrin.h>

// Variable Definitions
static emlrtRSInfo f_emlrtRSI{
    10,                         // lineNo
    "generatedPlantDerivative", // fcnName
    "source/hil/matlab_validation/+m600check/generatedPlantDerivative.m" // pathName
};

static emlrtRSInfo g_emlrtRSI{
    30,                         // lineNo
    "generatedPlantDerivative", // fcnName
    "source/hil/matlab_validation/+m600check/generatedPlantDerivative.m" // pathName
};

static emlrtRSInfo h_emlrtRSI{
    39,                         // lineNo
    "generatedPlantDerivative", // fcnName
    "source/hil/matlab_validation/+m600check/generatedPlantDerivative.m" // pathName
};

static emlrtRSInfo i_emlrtRSI{
    10,                    // lineNo
    "generatedAllocation", // fcnName
    "source/hil/matlab_validation/+m600check/generatedAllocation.m" // pathName
};

static emlrtRSInfo j_emlrtRSI{
    11,                    // lineNo
    "generatedAllocation", // fcnName
    "source/hil/matlab_validation/+m600check/generatedAllocation.m" // pathName
};

static emlrtRSInfo k_emlrtRSI{
    14,                    // lineNo
    "generatedAllocation", // fcnName
    "source/hil/matlab_validation/+m600check/generatedAllocation.m" // pathName
};

static emlrtRSInfo l_emlrtRSI{
    11,     // lineNo
    "rank", // fcnName
    "matlab\\toolbox\\eml\\lib\\matlab\\matfun\\rank.m" // pathName
};

static emlrtRSInfo m_emlrtRSI{
    20,           // lineNo
    "local_rank", // fcnName
    "matlab\\toolbox\\eml\\lib\\matlab\\matfun\\rank.m" // pathName
};

static emlrtRSInfo n_emlrtRSI{
    18,    // lineNo
    "svd", // fcnName
    "matlab\\toolbox\\eml\\lib\\matlab\\matfun\\svd.m" // pathName
};

static emlrtRSInfo s_emlrtRSI{
    20,                // lineNo
    "mrdivide_helper", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\mrdivide_helper.m" // pathName
};

static emlrtRSInfo hb_emlrtRSI{
    30,                             // lineNo
    "gpenmpcM600StructuredResidual", // fcnName
    "assets/canonical/method_source/matlab/dynamics/"
    "gpenmpcM600StructuredResidual.m" // pathName
};

static emlrtRSInfo ib_emlrtRSI{
    35,                             // lineNo
    "gpenmpcM600StructuredResidual", // fcnName
    "assets/canonical/method_source/matlab/dynamics/"
    "gpenmpcM600StructuredResidual.m" // pathName
};

static emlrtRSInfo jb_emlrtRSI{
    58,                             // lineNo
    "gpenmpcM600StructuredResidual", // fcnName
    "assets/canonical/method_source/matlab/dynamics/"
    "gpenmpcM600StructuredResidual.m" // pathName
};

static emlrtRSInfo kb_emlrtRSI{
    71,      // lineNo
    "power", // fcnName
    "matlab\\toolbox\\eml\\lib\\matlab\\ops\\power.m" // pathName
};

static emlrtRSInfo lb_emlrtRSI{
    20,         // lineNo
    "mldivide", // fcnName
    "matlab\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" // pathName
};

static emlrtRSInfo mb_emlrtRSI{
    42,      // lineNo
    "mldiv", // fcnName
    "matlab\\toolbox\\eml\\lib\\matlab\\ops\\mldivide.m" // pathName
};

static emlrtRSInfo nb_emlrtRSI{
    61,        // lineNo
    "lusolve", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\lusolve."
    "m" // pathName
};

static emlrtRSInfo ob_emlrtRSI{
    293,          // lineNo
    "lusolve3x3", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\lusolve."
    "m" // pathName
};

static emlrtMCInfo emlrtMCI{
    27,      // lineNo
    5,       // colNo
    "error", // fName
    "matlab\\toolbox\\eml\\lib\\matlab\\lang\\error.m" // pName
};

static emlrtRSInfo pb_emlrtRSI{
    27,      // lineNo
    "error", // fcnName
    "matlab\\toolbox\\eml\\lib\\matlab\\lang\\error.m" // pathName
};

// Function Declarations
static void b_error(const emlrtStack &sp, const mxArray *m, const mxArray *m1,
                    emlrtMCInfo &location);

// Function Definitions
static void b_error(const emlrtStack &sp, const mxArray *m, const mxArray *m1,
                    emlrtMCInfo &location)
{
  const mxArray *pArrays[2];
  pArrays[0] = m;
  pArrays[1] = m1;
  emlrtCallMATLABR2012b((emlrtConstCTX)&sp, 0, nullptr, 2, &pArrays[0], "error",
                        true, &location);
}

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
                              real_T derivative[19], struct12_T *diagnostic)
{
  static const real_T dv[3]{0.0, 0.0, 9.80665};
  static const int32_T iv[2]{1, 26};
  static const int32_T iv1[2]{1, 59};
  static const char_T varargin_2[59]{
      'T', 'h', 'e', ' ', 's', 'i', 'x', '-', 'r', 'o', 't', 'o', 'r', ' ', 'a',
      'l', 'l', 'o', 'c', 'a', 't', 'i', 'o', 'n', ' ', 'm', 'a', 't', 'r', 'i',
      'x', ' ', 'm', 'u', 's', 't', ' ', 'h', 'a', 'v', 'e', ' ', 'f', 'u', 'l',
      'l', ' ', 'w', 'r', 'e', 'n', 'c', 'h', ' ', 'r', 'a', 'n', 'k', '.'};
  static const char_T varargin_1[26]{
      'g', 'p', 'e', 'n', 'm', 'p', 'c', 'M', '6', '0', '0', 'A', 'l', 'l',
      'o', 'c', 'a', 't', 'i', 'o', 'n', ':', 'R', 'a', 'n', 'k'};
  __m128d b_r2;
  __m128d r;
  __m128d r1;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *m;
  const mxArray *y;
  real_T gram_tmp[24];
  real_T matrix[24];
  real_T gram[16];
  real_T allocation_inertia_kg_m2[9];
  real_T rotation[9];
  real_T angles[6];
  real_T q[4];
  real_T s[4];
  real_T B[3];
  real_T structured_drag[3];
  real_T structured_thrust[3];
  real_T structured_turn[3];
  real_T trueDragBasis[3];
  real_T tangentXy[2];
  real_T absx;
  real_T absxk;
  real_T b_rotation_tmp;
  real_T c_rotor_actuator_time_constant_;
  real_T payloadFactor;
  real_T payloadFraction;
  real_T rotation_tmp;
  real_T t;
  real_T trueMassKg;
  real_T trueMassKg_tmp;
  int32_T exponent;
  int32_T irank;
  int32_T r2;
  boolean_T exitg1;
  boolean_T p;
  st.prev = &sp;
  st.tls = sp.tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  //  Arithmetic body of immutable gpenmpcM600SixDofPlantDerivative, SHA
  //  9B493F... Only functional substitution is the struct-layout-compatible
  //  allocation.
  st.site = &f_emlrtRSI;
  //  Same arithmetic as immutable gpenmpcM600Allocation, SHA 8888EFB8...2D00F.
  //  Only change: construct every field before reading the structure (Coder).
  c_rotor_actuator_time_constant_ =
      calibration_rotor_allocation.c_actuator_time_constant_nomina;
  for (int32_T k{0}; k < 6; k++) {
    absx = 0.017453292519943295 * calibration_rotor_allocation.angles_deg[k];
    absxk = muDoubleScalarSin(absx);
    absx = muDoubleScalarCos(absx);
    r2 = k << 2;
    matrix[r2] = 1.0;
    matrix[r2 + 1] = calibration_rotor_allocation.arm_radius_m * absxk;
    matrix[r2 + 2] = -calibration_rotor_allocation.arm_radius_m * absx;
    matrix[r2 + 3] = calibration_rotor_allocation.yaw_moment_arm_nominal_m *
                     calibration_rotor_allocation.spin_sign[k];
  }
  std::memset(&gram[0], 0, sizeof(real_T) << 4);
  for (int32_T k{0}; k < 4; k++) {
    r2 = k << 2;
    for (int32_T i{0}; i < 6; i++) {
      irank = i << 2;
      absx = matrix[k + irank];
      gram_tmp[i + 6 * k] = absx;
      r = _mm_loadu_pd(&matrix[irank]);
      r1 = _mm_loadu_pd(&gram[r2]);
      b_r2 = _mm_set1_pd(absx);
      _mm_storeu_pd(&gram[r2], _mm_add_pd(r1, _mm_mul_pd(r, b_r2)));
      r = _mm_loadu_pd(&matrix[irank + 2]);
      r1 = _mm_loadu_pd(&gram[r2 + 2]);
      _mm_storeu_pd(&gram[r2 + 2], _mm_add_pd(r1, _mm_mul_pd(r, b_r2)));
    }
  }
  b_st.site = &i_emlrtRSI;
  c_st.site = &l_emlrtRSI;
  irank = 0;
  d_st.site = &m_emlrtRSI;
  p = true;
  for (int32_T k{0}; k < 16; k++) {
    if (p) {
      absx = gram[k];
      if (muDoubleScalarIsInf(absx) || muDoubleScalarIsNaN(absx)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (p) {
    e_st.site = &n_emlrtRSI;
    coder::internal::svd(e_st, gram, s);
  } else {
    s[0] = rtNaN;
    s[1] = rtNaN;
    s[2] = rtNaN;
    s[3] = rtNaN;
  }
  absx = muDoubleScalarAbs(s[0]);
  if (muDoubleScalarIsInf(absx) || muDoubleScalarIsNaN(absx)) {
    absx = rtNaN;
  } else if (absx < 4.450147717014403E-308) {
    absx = 5.0E-324;
  } else {
    std::frexp(absx, &exponent);
    absx = std::ldexp(1.0, exponent - 53);
  }
  absx *= 4.0;
  r2 = 0;
  exitg1 = false;
  while (!exitg1 && (r2 < 4)) {
    if (muDoubleScalarIsInf(s[r2]) || muDoubleScalarIsNaN(s[r2])) {
      absx = 1.7976931348623157E+308;
      exitg1 = true;
    } else {
      r2++;
    }
  }
  r2 = 0;
  while ((r2 < 4) && (s[r2] > absx)) {
    irank++;
    r2++;
  }
  if (irank != 4) {
    b_st.site = &j_emlrtRSI;
    y = nullptr;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(&b_st, 26, m, &varargin_1[0]);
    emlrtAssign(&y, m);
    b_y = nullptr;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a(&b_st, 59, m, &varargin_2[0]);
    emlrtAssign(&b_y, m);
    c_st.site = &pb_emlrtRSI;
    b_error(c_st, y, b_y, emlrtMCI);
  }
  b_st.site = &k_emlrtRSI;
  c_st.site = &s_emlrtRSI;
  coder::internal::mrdiv(c_st, gram_tmp, gram);
  std::memset(&allocation_inertia_kg_m2[0], 0, 9U * sizeof(real_T));
  allocation_inertia_kg_m2[0] =
      calibration_mass_inertia.inertia_nominal_kg_m2[0];
  allocation_inertia_kg_m2[4] =
      calibration_mass_inertia.inertia_nominal_kg_m2[1];
  allocation_inertia_kg_m2[8] =
      calibration_mass_inertia.inertia_nominal_kg_m2[2];
  r = _mm_set1_pd(muDoubleScalarMax(coder::b_norm(&plantState[6]), 1.0E-15));
  _mm_storeu_pd(&s[0], _mm_div_pd(_mm_loadu_pd(&plantState[6]), r));
  _mm_storeu_pd(&s[2], _mm_div_pd(_mm_loadu_pd(&plantState[8]), r));
  // GPENMPCQUATERNIONROTATION Convert a scalar-first unit quaternion to SO(3).
  r = _mm_loadu_pd(&s[0]);
  r1 = _mm_set1_pd(muDoubleScalarMax(coder::b_norm(s), 1.0E-15));
  _mm_storeu_pd(&q[0], _mm_div_pd(r, r1));
  r = _mm_loadu_pd(&s[2]);
  _mm_storeu_pd(&q[2], _mm_div_pd(r, r1));
  trueMassKg_tmp = profile_mass_properties.base_mass_kg + payloadKg;
  trueMassKg = trueMassKg_tmp + c_mission_plant_mismatch_mass_b;
  diagnostic->actual_air_velocity_mps[0] = plantState[3] - actualWindXyMps[0];
  diagnostic->actual_air_velocity_mps[1] = plantState[4] - actualWindXyMps[1];
  diagnostic->actual_air_velocity_mps[2] = plantState[5];
  tangentXy[0] = muDoubleScalarAbs(diagnostic->actual_air_velocity_mps[0]);
  tangentXy[1] = muDoubleScalarAbs(diagnostic->actual_air_velocity_mps[1]);
  r = _mm_loadu_pd(&diagnostic->actual_air_velocity_mps[0]);
  r1 = _mm_loadu_pd(&tangentXy[0]);
  _mm_storeu_pd(&trueDragBasis[0], _mm_mul_pd(r, r1));
  trueDragBasis[2] = plantState[5] * muDoubleScalarAbs(plantState[5]);
  r = _mm_loadu_pd(&calibration_aerodynamics.frame_drag_nominal_n_per_mps2[0]);
  r1 = _mm_loadu_pd(&c_mission_plant_mismatch_drag_s[0]);
  r = _mm_mul_pd(r, r1);
  r1 = _mm_loadu_pd(&trueDragBasis[0]);
  b_r2 = _mm_mul_pd(r, r1);
  r = _mm_loadu_pd(&c_mission_plant_mismatch_cross_[0]);
  r1 = _mm_mul_pd(r, _mm_set1_pd(trueDragBasis[0]));
  r = _mm_loadu_pd(&c_mission_plant_mismatch_cross_[3]);
  r = _mm_mul_pd(r, _mm_set1_pd(trueDragBasis[1]));
  r1 = _mm_add_pd(r1, r);
  r = _mm_loadu_pd(&c_mission_plant_mismatch_cross_[6]);
  r = _mm_mul_pd(r, _mm_set1_pd(trueDragBasis[2]));
  r = _mm_add_pd(r1, r);
  r = _mm_add_pd(b_r2, r);
  _mm_storeu_pd(&diagnostic->true_drag_n[0], r);
  diagnostic->true_drag_n[2] =
      calibration_aerodynamics.frame_drag_nominal_n_per_mps2[2] *
          c_mission_plant_mismatch_drag_s[2] * trueDragBasis[2] +
      ((trueDragBasis[0] * c_mission_plant_mismatch_cross_[2] +
        trueDragBasis[1] * c_mission_plant_mismatch_cross_[5]) +
       trueDragBasis[2] * c_mission_plant_mismatch_cross_[8]);
  _mm_storeu_pd(&angles[0],
                _mm_mul_pd(_mm_loadu_pd(&plantState[13]),
                           _mm_loadu_pd(&c_mission_plant_mismatch_thrust[0])));
  _mm_storeu_pd(&angles[2],
                _mm_mul_pd(_mm_loadu_pd(&plantState[15]),
                           _mm_loadu_pd(&c_mission_plant_mismatch_thrust[2])));
  _mm_storeu_pd(&angles[4],
                _mm_mul_pd(_mm_loadu_pd(&plantState[17]),
                           _mm_loadu_pd(&c_mission_plant_mismatch_thrust[4])));
  diagnostic->true_wrench[0] = 0.0;
  diagnostic->true_wrench[1] = 0.0;
  diagnostic->true_wrench[2] = 0.0;
  diagnostic->true_wrench[3] = 0.0;
  for (int32_T k{0}; k < 6; k++) {
    r2 = k << 2;
    r = _mm_loadu_pd(&matrix[r2]);
    r1 = _mm_loadu_pd(&diagnostic->true_wrench[0]);
    b_r2 = _mm_set1_pd(angles[k]);
    _mm_storeu_pd(&diagnostic->true_wrench[0],
                  _mm_add_pd(r1, _mm_mul_pd(r, b_r2)));
    r = _mm_loadu_pd(&matrix[r2 + 2]);
    r1 = _mm_loadu_pd(&diagnostic->true_wrench[2]);
    _mm_storeu_pd(&diagnostic->true_wrench[2],
                  _mm_add_pd(r1, _mm_mul_pd(r, b_r2)));
  }
  st.site = &g_emlrtRSI;
  // GPENMPCM600STRUCTUREDRESIDUAL Software-only structured plant mismatch.
  //  This is part of the common plant used to compare B1 and ordinary B2.  It
  //  is never exposed to either controller as plant-private information.
  if (!mission_structured_residual.enabled) {
    structured_drag[0] = 0.0;
    structured_turn[0] = 0.0;
    structured_thrust[0] = 0.0;
    structured_drag[1] = 0.0;
    structured_turn[1] = 0.0;
    structured_thrust[1] = 0.0;
    structured_drag[2] = 0.0;
    structured_turn[2] = 0.0;
    structured_thrust[2] = 0.0;
  } else {
    real_T speed;
    speed = coder::c_norm(&reference_velocity_mps[0]);
    if (speed >= 0.75) {
      _mm_storeu_pd(&tangentXy[0],
                    _mm_div_pd(_mm_loadu_pd(&reference_velocity_mps[0]),
                               _mm_set1_pd(speed)));
    } else {
      absx = muDoubleScalarMax(
          coder::c_norm(&diagnostic->actual_air_velocity_mps[0]), 1.0E-12);
      r = _mm_loadu_pd(&diagnostic->actual_air_velocity_mps[0]);
      _mm_storeu_pd(&tangentXy[0], _mm_div_pd(r, _mm_set1_pd(absx)));
    }
    trueDragBasis[0] = tangentXy[0];
    trueDragBasis[1] = tangentXy[1];
    trueDragBasis[2] = 0.0;
    diagnostic->fast_acceleration_mps2[0] = -tangentXy[1];
    diagnostic->fast_acceleration_mps2[1] = tangentXy[0];
    diagnostic->fast_acceleration_mps2[2] = 0.0;
    // GPENMPCQUATERNIONROTATION Convert a scalar-first unit quaternion to SO(3).
    t = s[3] * s[3];
    rotation_tmp = s[2] * s[2];
    rotation[0] = 1.0 - 2.0 * (rotation_tmp + t);
    absx = s[1] * s[2];
    absxk = s[0] * s[3];
    rotation[3] = 2.0 * (absx - absxk);
    payloadFraction = s[1] * s[3];
    b_rotation_tmp = s[0] * s[2];
    rotation[6] = 2.0 * (payloadFraction + b_rotation_tmp);
    rotation[1] = 2.0 * (absx + absxk);
    payloadFactor = s[1] * s[1];
    rotation[4] = 1.0 - 2.0 * (payloadFactor + t);
    absx = s[2] * s[3];
    absxk = s[0] * s[1];
    rotation[7] = 2.0 * (absx - absxk);
    rotation[2] = 2.0 * (payloadFraction - b_rotation_tmp);
    rotation[5] = 2.0 * (absx + absxk);
    rotation[8] = 1.0 - 2.0 * (payloadFactor + rotation_tmp);
    b_st.site = &hb_emlrtRSI;
    payloadFraction =
        muDoubleScalarMin(muDoubleScalarMax(payloadKg / 4.54, 0.0), 1.0);
    payloadFactor = 0.65 * payloadFraction + 0.35;
    rotation_tmp = muDoubleScalarSin(
        muDoubleScalarAcos(muDoubleScalarMax(rotation[8], -1.0)));
    b_st.site = &ib_emlrtRSI;
    c_st.site = &kb_emlrtRSI;
    absx = mission_structured_residual.drag_tilt_gain *
               (rotation_tmp * rotation_tmp) +
           1.0;
    b_rotation_tmp = muDoubleScalarMax(trueMassKg, 1.0E-12);
    tangentXy[0] = muDoubleScalarAbs(diagnostic->actual_air_velocity_mps[0]);
    tangentXy[1] = muDoubleScalarAbs(diagnostic->actual_air_velocity_mps[1]);
    r = _mm_loadu_pd(&diagnostic->actual_air_velocity_mps[0]);
    r1 = _mm_loadu_pd(&tangentXy[0]);
    _mm_storeu_pd(
        &structured_drag[0],
        _mm_div_pd(
            _mm_mul_pd(
                _mm_mul_pd(
                    _mm_mul_pd(
                        _mm_mul_pd(
                            _mm_mul_pd(
                                _mm_loadu_pd(
                                    &mission_structured_residual
                                         .c_drag_payload_coupling_n_per_m[0]),
                                _mm_set1_pd(-1.0)),
                            _mm_set1_pd(payloadFactor)),
                        _mm_set1_pd(absx)),
                    r),
                r1),
            _mm_set1_pd(b_rotation_tmp)));
    structured_drag[2] =
        -mission_structured_residual.c_drag_payload_coupling_n_per_m[2] *
        payloadFactor * absx * diagnostic->actual_air_velocity_mps[2] *
        muDoubleScalarAbs(diagnostic->actual_air_velocity_mps[2]) /
        b_rotation_tmp;
    n_t = (ptrdiff_t)3;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    absx = ddot(&n_t, &diagnostic->actual_air_velocity_mps[0], &incx_t,
                &trueDragBasis[0], &incy_t);
    n_t = (ptrdiff_t)3;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    absxk = ddot(&n_t, &diagnostic->actual_air_velocity_mps[0], &incx_t,
                 &diagnostic->fast_acceleration_mps2[0], &incy_t);
    t = 0.0;
    if (speed >= 0.75) {
      t = (reference_velocity_mps[0] * reference_acceleration_mps2[1] -
           reference_acceleration_mps2[0] * reference_velocity_mps[1]) /
          speed;
    }
    absx = -mission_structured_residual.turn_sideforce_n_per_mps2 *
               payloadFactor * absxk * muDoubleScalarAbs(absx) /
               b_rotation_tmp -
           mission_structured_residual.turn_centripetal_gain * payloadFactor *
               t * (0.75 * muDoubleScalarAbs(rotation_tmp) + 0.25);
    r = _mm_loadu_pd(&diagnostic->fast_acceleration_mps2[0]);
    _mm_storeu_pd(&structured_turn[0], _mm_mul_pd(_mm_set1_pd(absx), r));
    structured_turn[2] = absx * 0.0;
    absxk = coder::sumColumnB(&plantState[13]);
    absx = absxk / muDoubleScalarMax(trueMassKg_tmp * 9.80665, 1.0E-12);
    b_st.site = &jb_emlrtRSI;
    c_st.site = &kb_emlrtRSI;
    b_st.site = &jb_emlrtRSI;
    c_st.site = &kb_emlrtRSI;
    b_st.site = &jb_emlrtRSI;
    absx =
        -(((mission_structured_residual.thrust_efficiency_base_loss +
            mission_structured_residual.thrust_efficiency_payload_gain *
                payloadFraction) +
           mission_structured_residual.thrust_efficiency_command_gain *
               (0.5 *
                ((absx - 0.9) +
                 muDoubleScalarSqrt((absx - 0.9) * (absx - 0.9) + 0.0001)))) +
          mission_structured_residual.c_thrust_efficiency_vertical_de *
              (muDoubleScalarAbs(reference_acceleration_mps2[2]) / 9.80665)) *
        absxk / b_rotation_tmp;
    r = _mm_loadu_pd(&rotation[6]);
    _mm_storeu_pd(&structured_thrust[0], _mm_mul_pd(_mm_set1_pd(absx), r));
    structured_thrust[2] = absx * rotation[8];
  }
  t = q[3] * q[3];
  rotation_tmp = q[2] * q[2];
  rotation[0] = 1.0 - 2.0 * (rotation_tmp + t);
  absx = q[1] * q[2];
  absxk = q[0] * q[3];
  rotation[3] = 2.0 * (absx - absxk);
  payloadFraction = q[1] * q[3];
  b_rotation_tmp = q[0] * q[2];
  rotation[6] = 2.0 * (payloadFraction + b_rotation_tmp);
  rotation[1] = 2.0 * (absx + absxk);
  payloadFactor = q[1] * q[1];
  rotation[4] = 1.0 - 2.0 * (payloadFactor + t);
  absx = q[2] * q[3];
  absxk = q[0] * q[1];
  rotation[7] = 2.0 * (absx - absxk);
  rotation[2] = 2.0 * (payloadFraction - b_rotation_tmp);
  rotation[5] = 2.0 * (absx + absxk);
  rotation[8] = 1.0 - 2.0 * (payloadFactor + rotation_tmp);
  std::memset(&trueDragBasis[0], 0, 3U * sizeof(real_T));
  absx = trueDragBasis[0];
  absxk = trueDragBasis[1];
  rotation_tmp = trueDragBasis[2];
  t = diagnostic->true_wrench[0];
  for (int32_T k{0}; k < 3; k++) {
    payloadFraction =
        e_mission_plant_mismatch_extern[k] *
        muDoubleScalarSin(6.283185307179586 *
                              c_mission_plant_mismatch_extern[k] * globalTimeS +
                          d_mission_plant_mismatch_extern[k]);
    diagnostic->fast_acceleration_mps2[k] = payloadFraction;
    payloadFactor = plantState[k + 10];
    absx += allocation_inertia_kg_m2[3 * k] * payloadFactor;
    absxk += allocation_inertia_kg_m2[3 * k + 1] * payloadFactor;
    rotation_tmp += allocation_inertia_kg_m2[3 * k + 2] * payloadFactor;
    diagnostic->actual_acceleration_mps2[k] =
        ((((((((rotation[k] * 0.0 + rotation[k + 3] * 0.0) +
               rotation[k + 6] * t) -
              diagnostic->true_drag_n[k]) /
                 trueMassKg -
             dv[k]) +
            c_mission_plant_mismatch_accele[k]) +
           payloadFraction) +
          structured_drag[k]) +
         structured_turn[k]) +
        structured_thrust[k];
  }
  st.site = &h_emlrtRSI;
  std::copy(&allocation_inertia_kg_m2[0], &allocation_inertia_kg_m2[9],
            &rotation[0]);
  B[0] = diagnostic->true_wrench[1] -
         (rotation_tmp * plantState[11] - absxk * plantState[12]);
  B[1] = diagnostic->true_wrench[2] -
         (absx * plantState[12] - rotation_tmp * plantState[10]);
  B[2] = diagnostic->true_wrench[3] -
         (absxk * plantState[10] - absx * plantState[11]);
  b_st.site = &lb_emlrtRSI;
  c_st.site = &mb_emlrtRSI;
  d_st.site = &nb_emlrtRSI;
  r2 = 1;
  irank = 2;
  rotation[1] = 0.0 / calibration_mass_inertia.inertia_nominal_kg_m2[0];
  rotation[2] /= rotation[0];
  rotation[4] -= rotation[1] * rotation[3];
  rotation[5] -= rotation[2] * rotation[3];
  rotation[7] -= rotation[1] * rotation[6];
  rotation[8] -= rotation[2] * rotation[6];
  if (muDoubleScalarAbs(rotation[5]) > muDoubleScalarAbs(rotation[4])) {
    r2 = 2;
    irank = 1;
  }
  rotation[irank + 3] /= rotation[r2 + 3];
  rotation[irank + 6] -= rotation[irank + 3] * rotation[r2 + 6];
  if ((rotation[0] == 0.0) || (rotation[r2 + 3] == 0.0) ||
      (rotation[irank + 6] == 0.0)) {
    e_st.site = &ob_emlrtRSI;
    f_st.site = &gb_emlrtRSI;
    coder::internal::warning(f_st);
  }
  trueDragBasis[1] = B[r2] - B[0] * rotation[r2];
  trueDragBasis[2] = (B[irank] - B[0] * rotation[irank]) -
                     trueDragBasis[1] * rotation[irank + 3];
  trueDragBasis[2] /= rotation[irank + 6];
  trueDragBasis[0] = B[0] - trueDragBasis[2] * rotation[6];
  trueDragBasis[1] -= trueDragBasis[2] * rotation[r2 + 6];
  trueDragBasis[1] /= rotation[r2 + 3];
  trueDragBasis[0] -= trueDragBasis[1] * rotation[3];
  trueDragBasis[0] /= rotation[0];
  // GPENMPCQUATERNIONDERIVATIVEMATRIX Scalar-first quaternion rate matrix.
  gram[0] = 0.0;
  absxk = 0.5 * -plantState[10];
  gram[4] = absxk;
  absx = 0.5 * -plantState[11];
  gram[8] = absx;
  rotation_tmp = 0.5 * -plantState[12];
  gram[12] = rotation_tmp;
  t = 0.5 * plantState[10];
  gram[1] = t;
  gram[5] = 0.0;
  payloadFraction = 0.5 * plantState[12];
  gram[9] = payloadFraction;
  gram[13] = absx;
  absx = 0.5 * plantState[11];
  gram[2] = absx;
  gram[6] = rotation_tmp;
  gram[10] = 0.0;
  gram[14] = t;
  gram[3] = payloadFraction;
  gram[7] = absx;
  gram[11] = absxk;
  gram[15] = 0.0;
  std::memset(&q[0], 0, sizeof(real_T) << 2);
  for (int32_T k{0}; k < 4; k++) {
    r2 = k << 2;
    r = _mm_loadu_pd(&gram[r2]);
    r1 = _mm_loadu_pd(&q[0]);
    b_r2 = _mm_set1_pd(s[k]);
    _mm_storeu_pd(&q[0], _mm_add_pd(r1, _mm_mul_pd(r, b_r2)));
    r = _mm_loadu_pd(&gram[r2 + 2]);
    r1 = _mm_loadu_pd(&q[2]);
    _mm_storeu_pd(&q[2], _mm_add_pd(r1, _mm_mul_pd(r, b_r2)));
  }
  derivative[0] = plantState[3];
  derivative[3] = diagnostic->actual_acceleration_mps2[0];
  derivative[1] = plantState[4];
  derivative[4] = diagnostic->actual_acceleration_mps2[1];
  derivative[2] = plantState[5];
  derivative[5] = diagnostic->actual_acceleration_mps2[2];
  derivative[6] = q[0];
  derivative[7] = q[1];
  derivative[8] = q[2];
  derivative[9] = q[3];
  derivative[10] = trueDragBasis[0];
  derivative[11] = trueDragBasis[1];
  derivative[12] = trueDragBasis[2];
  r = _mm_set1_pd(c_rotor_actuator_time_constant_);
  _mm_storeu_pd(&derivative[13],
                _mm_div_pd(_mm_sub_pd(_mm_loadu_pd(&rotorCommandN[0]),
                                      _mm_loadu_pd(&plantState[13])),
                           r));
  _mm_storeu_pd(&derivative[15],
                _mm_div_pd(_mm_sub_pd(_mm_loadu_pd(&rotorCommandN[2]),
                                      _mm_loadu_pd(&plantState[15])),
                           r));
  _mm_storeu_pd(&derivative[17],
                _mm_div_pd(_mm_sub_pd(_mm_loadu_pd(&rotorCommandN[4]),
                                      _mm_loadu_pd(&plantState[17])),
                           r));
  absx = 3.312168642111238E-170;
  absxk = muDoubleScalarAbs(diagnostic->actual_air_velocity_mps[0]);
  if (absxk > 3.312168642111238E-170) {
    rotation_tmp = 1.0;
    absx = absxk;
  } else {
    t = absxk / 3.312168642111238E-170;
    rotation_tmp = t * t;
  }
  absxk = muDoubleScalarAbs(diagnostic->actual_air_velocity_mps[1]);
  if (absxk > absx) {
    t = absx / absxk;
    rotation_tmp = rotation_tmp * t * t + 1.0;
    absx = absxk;
  } else {
    t = absxk / absx;
    rotation_tmp += t * t;
  }
  absxk = muDoubleScalarAbs(diagnostic->actual_air_velocity_mps[2]);
  if (absxk > absx) {
    t = absx / absxk;
    rotation_tmp = rotation_tmp * t * t + 1.0;
    absx = absxk;
  } else {
    t = absxk / absx;
    rotation_tmp += t * t;
  }
  rotation_tmp = absx * muDoubleScalarSqrt(rotation_tmp);
  p = muDoubleScalarIsNaN(rotation_tmp);
  if (p) {
    r2 = 0;
    int32_T exitg2;
    do {
      exitg2 = 0;
      if (r2 < 3) {
        if (muDoubleScalarIsNaN(diagnostic->actual_air_velocity_mps[r2])) {
          exitg2 = 1;
        } else {
          r2++;
        }
      } else {
        rotation_tmp = rtInf;
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }
  diagnostic->actual_airspeed_mps = rotation_tmp;
  r = _mm_loadu_pd(&structured_drag[0]);
  r1 = _mm_loadu_pd(&structured_turn[0]);
  b_r2 = _mm_loadu_pd(&structured_thrust[0]);
  _mm_storeu_pd(&diagnostic->structured_acceleration_mps2[0],
                _mm_add_pd(_mm_add_pd(r, r1), b_r2));
  diagnostic->structured_acceleration_mps2[2] =
      (structured_drag[2] + structured_turn[2]) + structured_thrust[2];
  diagnostic->true_mass_kg = trueMassKg;
}

} // namespace m600check

// End of code generation (generatedPlantDerivative.cpp)
