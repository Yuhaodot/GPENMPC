//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mrdivide_helper.cpp
//
// Code generation for function 'mrdivide_helper'
//

// Include files
#include "mrdivide_helper.h"
#include "eml_int_forloop_overflow_error.h"
#include "gpenmpcM600CodegenDerivative_data.h"
#include "rt_nonfinite.h"
#include "warning.h"
#include "mwmathutil.h"
#include <algorithm>
#include <emmintrin.h>

// Variable Definitions
static emlrtRSInfo r_emlrtRSI{
    21,                               // lineNo
    "eml_int_forloop_overflow_check", // fcnName
    "matlab\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_"
    "forloop_overflow_check.m" // pathName
};

static emlrtRSInfo t_emlrtRSI{
    42,      // lineNo
    "mrdiv", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\mrdivide_helper.m" // pathName
};

static emlrtRSInfo u_emlrtRSI{
    67,        // lineNo
    "lusolve", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\lusolve."
    "m" // pathName
};

static emlrtRSInfo v_emlrtRSI{
    107,          // lineNo
    "lusolveNxN", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\lusolve."
    "m" // pathName
};

static emlrtRSInfo w_emlrtRSI{
    112,          // lineNo
    "lusolveNxN", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\lusolve."
    "m" // pathName
};

static emlrtRSInfo x_emlrtRSI{
    135,          // lineNo
    "XtimesInvA", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\lusolve."
    "m" // pathName
};

static emlrtRSInfo y_emlrtRSI{
    36,       // lineNo
    "xgetrf", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgetrf.m" // pathName
};

static emlrtRSInfo ab_emlrtRSI{
    55,        // lineNo
    "xzgetrf", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgetrf.m" // pathName
};

static emlrtRSInfo bb_emlrtRSI{
    63,        // lineNo
    "xzgetrf", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzgetrf.m" // pathName
};

static emlrtRSInfo cb_emlrtRSI{
    45,      // lineNo
    "xgeru", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "blas\\xgeru.m" // pathName
};

static emlrtRSInfo db_emlrtRSI{
    45,     // lineNo
    "xger", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "blas\\xger.m" // pathName
};

static emlrtRSInfo eb_emlrtRSI{
    15,     // lineNo
    "xger", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xger.m" // pathName
};

static emlrtRSInfo fb_emlrtRSI{
    54,      // lineNo
    "xgerx", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "refblas\\xgerx.m" // pathName
};

// Function Definitions
namespace coder {
namespace internal {
void mrdiv(const emlrtStack &sp, real_T A[24], const real_T B[16])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack j_st;
  emlrtStack k_st;
  emlrtStack st;
  real_T b_A[16];
  real_T smax;
  int32_T info;
  int32_T jA;
  int32_T jBcol;
  int32_T kBcol;
  int32_T mmj;
  int8_T ipiv[4];
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
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  j_st.prev = &i_st;
  j_st.tls = i_st.tls;
  k_st.prev = &j_st;
  k_st.tls = j_st.tls;
  st.site = &t_emlrtRSI;
  b_st.site = &u_emlrtRSI;
  c_st.site = &v_emlrtRSI;
  d_st.site = &x_emlrtRSI;
  std::copy(&B[0], &B[16], &b_A[0]);
  e_st.site = &y_emlrtRSI;
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  info = 0;
  for (int32_T j{0}; j < 3; j++) {
    int32_T jj;
    mmj = 2 - j;
    jBcol = j * 5;
    jj = j * 5;
    jA = 5 - j;
    kBcol = 0;
    smax = muDoubleScalarAbs(b_A[jj]);
    for (int32_T k{2}; k < jA; k++) {
      real_T s;
      s = muDoubleScalarAbs(b_A[(jBcol + k) - 1]);
      if (s > smax) {
        kBcol = k - 1;
        smax = s;
      }
    }
    if (b_A[jj + kBcol] != 0.0) {
      if (kBcol != 0) {
        jA = j + kBcol;
        ipiv[j] = static_cast<int8_T>(jA + 1);
        smax = b_A[j];
        b_A[j] = b_A[jA];
        b_A[jA] = smax;
        smax = b_A[j + 4];
        b_A[j + 4] = b_A[jA + 4];
        b_A[jA + 4] = smax;
        smax = b_A[j + 8];
        b_A[j + 8] = b_A[jA + 8];
        b_A[jA + 8] = smax;
        smax = b_A[j + 12];
        b_A[j + 12] = b_A[jA + 12];
        b_A[jA + 12] = smax;
      }
      kBcol = (jj - j) + 4;
      f_st.site = &ab_emlrtRSI;
      for (int32_T i{jBcol + 2}; i <= kBcol; i++) {
        b_A[i - 1] /= b_A[jj];
      }
    } else {
      info = j + 1;
    }
    f_st.site = &bb_emlrtRSI;
    g_st.site = &cb_emlrtRSI;
    h_st.site = &db_emlrtRSI;
    i_st.site = &eb_emlrtRSI;
    jA = jj + 6;
    for (int32_T i{0}; i <= mmj; i++) {
      smax = b_A[(jBcol + (i << 2)) + 4];
      if (smax != 0.0) {
        kBcol = (jA - j) + 2;
        j_st.site = &fb_emlrtRSI;
        if ((jA <= kBcol) && (kBcol > 2147483646)) {
          k_st.site = &r_emlrtRSI;
          eml_int_forloop_overflow_error(k_st);
        }
        for (int32_T k{jA}; k <= kBcol; k++) {
          b_A[k - 1] += b_A[((jj + k) - jA) + 1] * -smax;
        }
      }
      jA += 4;
    }
  }
  if ((info == 0) && !(b_A[15] != 0.0)) {
    info = 4;
  }
  for (int32_T j{0}; j < 4; j++) {
    jBcol = 6 * j - 1;
    jA = j << 2;
    for (int32_T i{0}; i < j; i++) {
      kBcol = 6 * i;
      smax = b_A[i + jA];
      if (smax != 0.0) {
        for (int32_T k{0}; k < 6; k++) {
          mmj = (k + jBcol) + 1;
          A[mmj] -= smax * A[k + kBcol];
        }
      }
    }
    smax = 1.0 / b_A[j + jA];
    for (int32_T i{0}; i <= 4; i += 2) {
      __m128d r;
      jA = (i + jBcol) + 1;
      r = _mm_loadu_pd(&A[jA]);
      r = _mm_mul_pd(_mm_set1_pd(smax), r);
      _mm_storeu_pd(&A[jA], r);
    }
  }
  for (int32_T i{3}; i >= 0; i--) {
    jA = 6 * i - 1;
    kBcol = (i << 2) - 1;
    for (int32_T k{i + 2}; k < 5; k++) {
      mmj = 6 * (k - 1);
      smax = b_A[k + kBcol];
      if (smax != 0.0) {
        for (int32_T j{0}; j < 6; j++) {
          jBcol = (j + jA) + 1;
          A[jBcol] -= smax * A[j + mmj];
        }
      }
    }
  }
  for (int32_T i{2}; i >= 0; i--) {
    int8_T b_i;
    b_i = ipiv[i];
    if (b_i != i + 1) {
      for (int32_T k{0}; k < 6; k++) {
        jA = k + 6 * i;
        smax = A[jA];
        kBcol = k + 6 * (b_i - 1);
        A[jA] = A[kBcol];
        A[kBcol] = smax;
      }
    }
  }
  if (info > 0) {
    c_st.site = &w_emlrtRSI;
    d_st.site = &gb_emlrtRSI;
    warning(d_st);
  }
}

} // namespace internal
} // namespace coder

// End of code generation (mrdivide_helper.cpp)
