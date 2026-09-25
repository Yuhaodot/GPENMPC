//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xaxpy.cpp
//
// Code generation for function 'xaxpy'
//

// Include files
#include "xaxpy.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void b_xaxpy(int32_T n, real_T a, const real_T x[4], int32_T ix0, real_T y[16],
             int32_T iy0)
{
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T vectorUB;
    scalarLB = n / 2 * 2;
    vectorUB = scalarLB - 2;
    for (int32_T k{0}; k <= vectorUB; k += 2) {
      __m128d r;
      __m128d r1;
      int32_T i;
      i = (iy0 + k) - 1;
      r = _mm_loadu_pd(&x[(ix0 + k) - 1]);
      r = _mm_mul_pd(_mm_set1_pd(a), r);
      r1 = _mm_loadu_pd(&y[i]);
      r = _mm_add_pd(r1, r);
      _mm_storeu_pd(&y[i], r);
    }
    for (int32_T k{scalarLB}; k < n; k++) {
      vectorUB = (iy0 + k) - 1;
      y[vectorUB] += a * x[(ix0 + k) - 1];
    }
  }
}

void xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[16], int32_T iy0)
{
  if (!(a == 0.0)) {
    for (int32_T k{0}; k < n; k++) {
      int32_T i;
      i = (iy0 + k) - 1;
      y[i] += a * y[(ix0 + k) - 1];
    }
  }
}

void xaxpy(int32_T n, real_T a, const real_T x[16], int32_T ix0, real_T y[4],
           int32_T iy0)
{
  if (!(a == 0.0)) {
    int32_T scalarLB;
    int32_T vectorUB;
    scalarLB = (n / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int32_T k{0}; k <= vectorUB; k += 2) {
      _mm_storeu_pd(
          &y[iy0 - 1],
          _mm_add_pd(_mm_loadu_pd(&y[iy0 - 1]),
                     _mm_mul_pd(_mm_set1_pd(a), _mm_loadu_pd(&x[ix0 - 1]))));
    }
    for (int32_T k{scalarLB}; k < n; k++) {
      vectorUB = (iy0 + k) - 1;
      y[vectorUB] += a * x[(ix0 + k) - 1];
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xaxpy.cpp)
