//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// svd.cpp
//
// Code generation for function 'svd'
//

// Include files
#include "svd.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xscal.h"
#include "xzlangeM.h"
#include "xzlascl.h"
#include "blas.h"
#include "mwmathutil.h"
#include <algorithm>
#include <emmintrin.h>

// Variable Definitions
static emlrtRSInfo
    o_emlrtRSI{
        28,    // lineNo
        "svd", // fcnName
        "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\svd."
        "m" // pathName
    };

static emlrtRSInfo
    p_emlrtRSI{
        107,          // lineNo
        "callLAPACK", // fcnName
        "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\svd."
        "m" // pathName
    };

static emlrtRSInfo q_emlrtRSI{
    34,       // lineNo
    "xgesvd", // fcnName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "lapack\\xgesvd.m" // pathName
};

static emlrtRTEInfo p_emlrtRTEI{
    293,      // lineNo
    13,       // colNo
    "xzsvdc", // fName
    "matlab\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "reflapack\\xzsvdc.m" // pName
};

// Function Definitions
namespace coder {
namespace internal {
void svd(const emlrtStack &sp, const real_T A[16], real_T U[4])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T b_A[16];
  real_T e[4];
  real_T work[4];
  real_T anrm;
  real_T cscale;
  real_T f;
  real_T nrm;
  real_T r;
  real_T rt;
  real_T scale;
  real_T sm;
  real_T snorm;
  real_T sqds;
  int32_T iter;
  int32_T m;
  int32_T qp1;
  int32_T qq;
  int32_T qq_tmp;
  int32_T qs;
  boolean_T doscale;
  boolean_T exitg1;
  st.prev = &sp;
  st.tls = sp.tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  st.site = &o_emlrtRSI;
  b_st.site = &p_emlrtRSI;
  c_st.site = &q_emlrtRSI;
  std::copy(&A[0], &A[16], &b_A[0]);
  U[0] = 0.0;
  e[0] = 0.0;
  work[0] = 0.0;
  U[1] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  U[2] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  U[3] = 0.0;
  e[3] = 0.0;
  work[3] = 0.0;
  doscale = false;
  anrm = reflapack::xzlangeM(A);
  cscale = anrm;
  if ((anrm > 0.0) && (anrm < 6.717876107567089E-139)) {
    doscale = true;
    cscale = 6.717876107567089E-139;
    reflapack::b_xzlascl(anrm, cscale, b_A);
  } else if (anrm > 1.488565707357403E+138) {
    doscale = true;
    cscale = 1.488565707357403E+138;
    reflapack::b_xzlascl(anrm, cscale, b_A);
  }
  for (int32_T q{0}; q < 3; q++) {
    __m128d b_r;
    boolean_T apply_transform;
    qp1 = q + 2;
    qq_tmp = q + (q << 2);
    qq = qq_tmp + 1;
    apply_transform = false;
    nrm = blas::xnrm2(4 - q, b_A, qq_tmp + 1);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq_tmp] < 0.0) {
        nrm = -nrm;
      }
      U[q] = nrm;
      if (muDoubleScalarAbs(nrm) >= 1.0020841800044864E-292) {
        blas::xscal(4 - q, 1.0 / nrm, b_A, qq_tmp + 1);
      } else {
        qs = (qq_tmp - q) + 4;
        iter = ((((qs - qq_tmp) / 2) << 1) + qq_tmp) + 1;
        m = iter - 2;
        for (int32_T jj{qq}; jj <= m; jj += 2) {
          b_r = _mm_loadu_pd(&b_A[jj - 1]);
          _mm_storeu_pd(&b_A[jj - 1], _mm_div_pd(b_r, _mm_set1_pd(U[q])));
        }
        for (int32_T jj{iter}; jj <= qs; jj++) {
          b_A[jj - 1] /= U[q];
        }
      }
      b_A[qq_tmp]++;
      U[q] = -U[q];
    } else {
      U[q] = 0.0;
    }
    for (int32_T jj{qp1}; jj < 5; jj++) {
      qq = q + ((jj - 1) << 2);
      if (apply_transform) {
        blas::xaxpy(
            4 - q,
            -(blas::xdotc(4 - q, b_A, qq_tmp + 1, b_A, qq + 1) / b_A[qq_tmp]),
            qq_tmp + 1, b_A, qq + 1);
      }
      e[jj - 1] = b_A[qq];
    }
    if (q + 1 <= 2) {
      nrm = blas::b_xnrm2(3 - q, e, q + 2);
      if (nrm == 0.0) {
        e[q] = 0.0;
      } else {
        if (e[q + 1] < 0.0) {
          e[q] = -nrm;
        } else {
          e[q] = nrm;
        }
        nrm = e[q];
        if (muDoubleScalarAbs(e[q]) >= 1.0020841800044864E-292) {
          blas::b_xscal(3 - q, 1.0 / e[q], e, q + 2);
        } else {
          qq = ((((3 - q) / 2) << 1) + q) + 2;
          qs = qq - 2;
          for (int32_T jj{qp1}; jj <= qs; jj += 2) {
            b_r = _mm_loadu_pd(&e[jj - 1]);
            _mm_storeu_pd(&e[jj - 1], _mm_div_pd(b_r, _mm_set1_pd(nrm)));
          }
          for (int32_T jj{qq}; jj < 5; jj++) {
            e[jj - 1] /= nrm;
          }
        }
        e[q + 1]++;
        e[q] = -e[q];
        for (int32_T jj{qp1}; jj < 5; jj++) {
          work[jj - 1] = 0.0;
        }
        for (int32_T jj{qp1}; jj < 5; jj++) {
          blas::xaxpy(3 - q, e[jj - 1], b_A, (q + ((jj - 1) << 2)) + 2, work,
                      q + 2);
        }
        for (int32_T jj{qp1}; jj < 5; jj++) {
          blas::b_xaxpy(3 - q, -e[jj - 1] / e[q + 1], work, q + 2, b_A,
                        (q + ((jj - 1) << 2)) + 2);
        }
      }
    }
  }
  m = 2;
  U[3] = b_A[15];
  e[2] = b_A[14];
  e[3] = 0.0;
  iter = 0;
  nrm = U[0];
  if (U[0] != 0.0) {
    rt = muDoubleScalarAbs(U[0]);
    r = U[0] / rt;
    nrm = rt;
    U[0] = rt;
    e[0] /= r;
  }
  if (e[0] != 0.0) {
    rt = muDoubleScalarAbs(e[0]);
    r = rt / e[0];
    e[0] = rt;
    U[1] *= r;
  }
  snorm = muDoubleScalarMax(muDoubleScalarAbs(nrm), e[0]);
  nrm = U[1];
  if (U[1] != 0.0) {
    rt = muDoubleScalarAbs(U[1]);
    r = U[1] / rt;
    nrm = rt;
    U[1] = rt;
    e[1] /= r;
  }
  if (e[1] != 0.0) {
    rt = muDoubleScalarAbs(e[1]);
    r = rt / e[1];
    e[1] = rt;
    U[2] *= r;
  }
  snorm =
      muDoubleScalarMax(snorm, muDoubleScalarMax(muDoubleScalarAbs(nrm), e[1]));
  nrm = U[2];
  if (U[2] != 0.0) {
    rt = muDoubleScalarAbs(U[2]);
    r = U[2] / rt;
    nrm = rt;
    U[2] = rt;
    e[2] = b_A[14] / r;
  }
  if (e[2] != 0.0) {
    rt = muDoubleScalarAbs(e[2]);
    r = rt / e[2];
    e[2] = rt;
    U[3] = b_A[15] * r;
  }
  snorm =
      muDoubleScalarMax(snorm, muDoubleScalarMax(muDoubleScalarAbs(nrm), e[2]));
  nrm = U[3];
  if (U[3] != 0.0) {
    rt = muDoubleScalarAbs(U[3]);
    nrm = rt;
    U[3] = rt;
  }
  snorm =
      muDoubleScalarMax(snorm, muDoubleScalarMax(muDoubleScalarAbs(nrm), 0.0));
  exitg1 = false;
  while (!exitg1 && (m + 2 > 0)) {
    if (iter >= 75) {
      emlrtErrorWithMessageIdR2018a(&c_st, &p_emlrtRTEI,
                                    "Coder:MATLAB:svd_NoConvergence",
                                    "Coder:MATLAB:svd_NoConvergence", 0);
    } else {
      boolean_T exitg2;
      qq_tmp = m + 1;
      exitg2 = false;
      while (!(exitg2 || (qq_tmp == 0))) {
        nrm = muDoubleScalarAbs(e[qq_tmp - 1]);
        if ((nrm <= 2.220446049250313E-16 * (muDoubleScalarAbs(U[qq_tmp - 1]) +
                                             muDoubleScalarAbs(U[qq_tmp]))) ||
            (nrm <= 1.0020841800044864E-292) ||
            ((iter > 20) && (nrm <= 2.220446049250313E-16 * snorm))) {
          e[qq_tmp - 1] = 0.0;
          exitg2 = true;
        } else {
          qq_tmp--;
        }
      }
      if (qq_tmp == m + 1) {
        qq = 4;
      } else {
        qs = m + 2;
        qq = m + 2;
        exitg2 = false;
        while (!exitg2 && (qq >= qq_tmp)) {
          qs = qq;
          if (qq == qq_tmp) {
            exitg2 = true;
          } else {
            nrm = 0.0;
            if (qq < m + 2) {
              nrm = muDoubleScalarAbs(e[qq - 1]);
            }
            if (qq > qq_tmp + 1) {
              nrm += muDoubleScalarAbs(e[qq - 2]);
            }
            rt = muDoubleScalarAbs(U[qq - 1]);
            if ((rt <= 2.220446049250313E-16 * nrm) ||
                (rt <= 1.0020841800044864E-292)) {
              U[qq - 1] = 0.0;
              exitg2 = true;
            } else {
              qq--;
            }
          }
        }
        if (qs == qq_tmp) {
          qq = 3;
        } else if (qs == m + 2) {
          qq = 1;
        } else {
          qq = 2;
          qq_tmp = qs;
        }
      }
      switch (qq) {
      case 1:
        f = e[m];
        e[m] = 0.0;
        for (int32_T jj{m + 1}; jj >= qq_tmp + 1; jj--) {
          nrm = 0.0;
          rt = 0.0;
          drotg(&U[jj - 1], &f, &nrm, &rt);
          if (jj > qq_tmp + 1) {
            r = e[jj - 2];
            f = -rt * r;
            e[jj - 2] = r * nrm;
          }
        }
        break;
      case 2:
        f = e[qq_tmp - 1];
        e[qq_tmp - 1] = 0.0;
        for (int32_T jj{qq_tmp + 1}; jj <= m + 2; jj++) {
          nrm = 0.0;
          rt = 0.0;
          drotg(&U[jj - 1], &f, &nrm, &rt);
          r = e[jj - 1];
          f = -rt * r;
          e[jj - 1] = r * nrm;
        }
        break;
      case 3:
        nrm = U[m + 1];
        scale = muDoubleScalarMax(
            muDoubleScalarMax(
                muDoubleScalarMax(muDoubleScalarMax(muDoubleScalarAbs(nrm),
                                                    muDoubleScalarAbs(U[m])),
                                  muDoubleScalarAbs(e[m])),
                muDoubleScalarAbs(U[qq_tmp])),
            muDoubleScalarAbs(e[qq_tmp]));
        sm = nrm / scale;
        nrm = U[m] / scale;
        rt = e[m] / scale;
        sqds = U[qq_tmp] / scale;
        r = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
        nrm = sm * rt;
        nrm *= nrm;
        if ((r != 0.0) || (nrm != 0.0)) {
          rt = muDoubleScalarSqrt(r * r + nrm);
          if (r < 0.0) {
            rt = -rt;
          }
          rt = nrm / (r + rt);
        } else {
          rt = 0.0;
        }
        f = (sqds + sm) * (sqds - sm) + rt;
        nrm = sqds * (e[qq_tmp] / scale);
        for (int32_T jj{qq_tmp + 1}; jj <= m + 1; jj++) {
          sm = 0.0;
          r = 0.0;
          drotg(&f, &nrm, &sm, &r);
          if (jj > qq_tmp + 1) {
            e[jj - 2] = f;
          }
          nrm = e[jj - 1];
          rt = U[jj - 1];
          scale = sm * rt + r * nrm;
          e[jj - 1] = sm * nrm - r * rt;
          nrm = U[jj];
          rt = r * nrm;
          r = nrm * sm;
          sqds = 0.0;
          sm = 0.0;
          drotg(&scale, &rt, &sqds, &sm);
          U[jj - 1] = scale;
          nrm = e[jj - 1];
          f = sqds * nrm + sm * r;
          r = -sm * nrm + sqds * r;
          U[jj] = r;
          nrm = sm * e[jj];
          e[jj] *= sqds;
        }
        e[m] = f;
        iter++;
        break;
      default:
        if (U[qq_tmp] < 0.0) {
          U[qq_tmp] = -U[qq_tmp];
        }
        qp1 = qq_tmp + 1;
        while ((qq_tmp + 1 < 4) && (U[qq_tmp] < U[qp1])) {
          rt = U[qq_tmp];
          U[qq_tmp] = U[qp1];
          U[qp1] = rt;
          qq_tmp = qp1;
          qp1++;
        }
        iter = 0;
        m--;
        break;
      }
    }
  }
  if (doscale) {
    reflapack::xzlascl(cscale, anrm, U);
  }
}

} // namespace internal
} // namespace coder

// End of code generation (svd.cpp)
