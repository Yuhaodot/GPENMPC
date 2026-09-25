//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xscal.cpp
//
// Code generation for function 'xscal'
//

// Include files
#include "xscal.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void b_xscal(int32_T n, real_T a, real_T x[4], int32_T ix0)
{
  int32_T i;
  int32_T scalarLB;
  int32_T vectorUB;
  i = ix0 + n;
  scalarLB = (((i - ix0) / 2) << 1) + ix0;
  vectorUB = scalarLB - 2;
  for (int32_T k{ix0}; k <= vectorUB; k += 2) {
    _mm_storeu_pd(&x[k - 1],
                  _mm_mul_pd(_mm_set1_pd(a), _mm_loadu_pd(&x[k - 1])));
  }
  for (int32_T k{scalarLB}; k < i; k++) {
    x[k - 1] *= a;
  }
}

void xscal(int32_T n, real_T a, real_T x[16], int32_T ix0)
{
  int32_T i;
  int32_T scalarLB;
  int32_T vectorUB;
  i = ix0 + n;
  scalarLB = (i - ix0) / 2 * 2 + ix0;
  vectorUB = scalarLB - 2;
  for (int32_T k{ix0}; k <= vectorUB; k += 2) {
    __m128d r;
    r = _mm_loadu_pd(&x[k - 1]);
    r = _mm_mul_pd(_mm_set1_pd(a), r);
    _mm_storeu_pd(&x[k - 1], r);
  }
  for (int32_T k{scalarLB}; k < i; k++) {
    x[k - 1] *= a;
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xscal.cpp)
