/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_gpenmpcNative_canonicalLocalInnerWithAuditFirst_api.c
 *
 * MATLAB Coder version            : 26.1
 * C/C++ source code generated on  : 2026-09-07 02:15:53
 */

/* Include Files */
#include "_coder_gpenmpcNative_canonicalLocalInnerWithAuditFirst_api.h"
#include "_coder_gpenmpcNative_canonicalLocalInnerWithAuditFirst_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131690U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "gpenmpcNative_canonicalLocalInnerWithAuditFirst",     /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

static const uint32_T uv[4] = {2156070934U, 2949246757U, 3236573511U,
                               2689710042U};

/* Function Declarations */
static real_T (*ab_emlrt_marshallIn(const emlrtStack *sp,
                                    const mxArray *nullptr,
                                    const char_T *identifier))[12];

static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[36];

static const mxArray *b_emlrt_marshallOut(real_T u[61]);

static real_T (*bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId))[12];

static uint64_T (*c_emlrt_marshallIn(const emlrtStack *sp,
                                     const mxArray *nullptr,
                                     const char_T *identifier))[2];

static const mxArray *c_emlrt_marshallOut(real_T u[70]);

static real_T cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                  const char_T *identifier);

static uint64_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId))[2];

static const mxArray *d_emlrt_marshallOut(real_T u[19]);

static real_T (*db_emlrt_marshallIn(const emlrtStack *sp,
                                    const mxArray *nullptr,
                                    const char_T *identifier))[3];

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[64];

static const mxArray *e_emlrt_marshallOut(real_T u[5]);

static real_T (*eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId))[3];

static void emlrtExitTimeCleanupDtorFcn(const void *r);

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[36];

static const mxArray *emlrt_marshallOut(real_T u[64]);

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[64];

static const mxArray *f_emlrt_marshallOut(real_T u[12]);

static real_T (*fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[36];

static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[70];

static const mxArray *g_emlrt_marshallOut(const struct52_T u);

static uint64_T (*gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                      const emlrtMsgIdentifier *msgId))[2];

static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[70];

static const mxArray *h_emlrt_marshallOut(real_T u[12]);

static real_T (*hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[64];

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, struct51_T *y);

static const mxArray *i_emlrt_marshallOut(const struct54_T u);

static real_T (*ib_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[70];

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct51_T *y);

static const mxArray *j_emlrt_marshallOut(const struct55_T *u);

static uint32_T jb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static uint32_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static const mxArray *k_emlrt_marshallOut(const real_T u[9]);

static uint16_T kb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static uint16_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static void lb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                uint8_T ret[32]);

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               uint8_T y[32]);

static uint64_T mb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static uint64_T n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static void nb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                real_T ret[256]);

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[256]);

static void ob_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                real_T ret[3072]);

static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[3072]);

static real_T pb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId);

static real_T q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static uint8_T qb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId);

static uint8_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId);

static void rb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                real_T ret[192]);

static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[192]);

static void sb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                real_T ret[12]);

static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[12]);

static void tb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, real_T ret[3]);

static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3]);

static real_T (*ub_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[12];

static struct52_T v_emlrt_marshallIn(const emlrtStack *sp,
                                     const mxArray *nullptr,
                                     const char_T *identifier);

static real_T (*vb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[3];

static struct52_T w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId);

static struct53_T x_emlrt_marshallIn(const emlrtStack *sp,
                                     const mxArray *nullptr,
                                     const char_T *identifier);

static struct53_T y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId);

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : real_T (*)[12]
 */
static real_T (*ab_emlrt_marshallIn(const emlrtStack *sp,
                                    const mxArray *nullptr,
                                    const char_T *identifier))[12]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[12];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = bb_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[36]
 */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[36]
{
  real_T(*y)[36];
  y = fb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : real_T u[61]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(real_T u[61])
{
  static const int32_T i = 0;
  static const int32_T i1 = 61;
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = NULL;
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

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[12]
 */
static real_T (*bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId))[12]
{
  real_T(*y)[12];
  y = ub_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : uint64_T (*)[2]
 */
static uint64_T (*c_emlrt_marshallIn(const emlrtStack *sp,
                                     const mxArray *nullptr,
                                     const char_T *identifier))[2]
{
  emlrtMsgIdentifier thisId;
  uint64_T(*y)[2];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : real_T u[70]
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrt_marshallOut(real_T u[70])
{
  static const int32_T i = 0;
  static const int32_T i1 = 70;
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = NULL;
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

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                  const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = q_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : uint64_T (*)[2]
 */
static uint64_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId))[2]
{
  uint64_T(*y)[2];
  y = gb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : real_T u[19]
 * Return Type  : const mxArray *
 */
static const mxArray *d_emlrt_marshallOut(real_T u[19])
{
  static const int32_T i = 0;
  static const int32_T i1 = 19;
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = NULL;
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

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : real_T (*)[3]
 */
static real_T (*db_emlrt_marshallIn(const emlrtStack *sp,
                                    const mxArray *nullptr,
                                    const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[3];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = eb_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : real_T (*)[64]
 */
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[64]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[64];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : real_T u[5]
 * Return Type  : const mxArray *
 */
static const mxArray *e_emlrt_marshallOut(real_T u[5])
{
  static const int32_T i = 0;
  static const int32_T i1 = 5;
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = NULL;
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

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[3]
 */
static real_T (*eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId))[3]
{
  real_T(*y)[3];
  y = vb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const void *r
 * Return Type  : void
 */
static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : real_T (*)[36]
 */
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[36]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[36];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : real_T u[64]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(real_T u[64])
{
  static const int32_T i = 0;
  static const int32_T i1 = 64;
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = NULL;
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

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[64]
 */
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[64]
{
  real_T(*y)[64];
  y = hb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : real_T u[12]
 * Return Type  : const mxArray *
 */
static const mxArray *f_emlrt_marshallOut(real_T u[12])
{
  static const int32_T i = 0;
  static const int32_T i1 = 12;
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = NULL;
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

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[36]
 */
static real_T (*fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[36]
{
  static const int32_T dims = 36;
  real_T(*ret)[36];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[36])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : real_T (*)[70]
 */
static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[70]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[70];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : const struct52_T u
 * Return Type  : const mxArray *
 */
static const mxArray *g_emlrt_marshallOut(const struct52_T u)
{
  static const int32_T i = 32;
  static const char_T *sv[4] = {"reference_asset_sha256", "leg_index",
                                "window_generation", "last_accepted_sequence"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *m;
  const mxArray *y;
  int32_T b_i;
  uint8_T *pData;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 4, (const char_T **)&sv[0]));
  b_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxUINT8_CLASS, mxREAL);
  pData = (uint8_T *)emlrtMxGetData(m);
  for (b_i = 0; b_i < 32; b_i++) {
    pData[b_i] = u.reference_asset_sha256[b_i];
  }
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "reference_asset_sha256", b_y, 0);
  c_y = NULL;
  m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
  *(uint32_T *)emlrtMxGetData(m) = u.leg_index;
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, "leg_index", c_y, 1);
  d_y = NULL;
  m = emlrtCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
  *(uint64_T *)emlrtMxGetData(m) = u.window_generation;
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, "window_generation", d_y, 2);
  e_y = NULL;
  m = emlrtCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
  *(uint64_T *)emlrtMxGetData(m) = u.last_accepted_sequence;
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, "last_accepted_sequence", e_y, 3);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : uint64_T (*)[2]
 */
static uint64_T (*gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                      const emlrtMsgIdentifier *msgId))[2]
{
  static const int32_T dims = 2;
  uint64_T(*ret)[2];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "uint64", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (uint64_T(*)[2])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[70]
 */
static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[70]
{
  real_T(*y)[70];
  y = ib_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : real_T u[12]
 * Return Type  : const mxArray *
 */
static const mxArray *h_emlrt_marshallOut(real_T u[12])
{
  static const int32_T iv[2] = {0, 0};
  static const int32_T iv1[2] = {3, 4};
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  existingData = emlrtMxGetData((mxArray *)m);
  if (existingData != (void *)&u[0]) {
    emlrtFreeMex(existingData);
  }
  emlrtMxSetData((mxArray *)m, &u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[64]
 */
static real_T (*hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[64]
{
  static const int32_T dims = 64;
  real_T(*ret)[64];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[64])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 *                struct51_T *y
 * Return Type  : void
 */
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, struct51_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  j_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

/*
 * Arguments    : const struct54_T u
 * Return Type  : const mxArray *
 */
static const mxArray *i_emlrt_marshallOut(const struct54_T u)
{
  static const char_T *sv[10] = {"accepted",
                                 "reason",
                                 "query_progress_s",
                                 "effective_nominal_progress_s",
                                 "query_sequence",
                                 "source_first_row",
                                 "source_last_row",
                                 "analytic_prefix_used",
                                 "reference_resampled",
                                 "hardware_actions"};
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
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 10, (const char_T **)&sv[0]));
  b_y = NULL;
  m = emlrtCreateLogicalScalar(u.accepted);
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "accepted", b_y, 0);
  c_y = NULL;
  m = emlrtCreateNumericMatrix(1, 1, mxUINT8_CLASS, mxREAL);
  *(uint8_T *)emlrtMxGetData(m) = u.reason;
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, "reason", c_y, 1);
  d_y = NULL;
  m = emlrtCreateDoubleScalar(u.query_progress_s);
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, "query_progress_s", d_y, 2);
  e_y = NULL;
  m = emlrtCreateDoubleScalar(u.effective_nominal_progress_s);
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, "effective_nominal_progress_s", e_y, 3);
  f_y = NULL;
  m = emlrtCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
  *(uint64_T *)emlrtMxGetData(m) = u.query_sequence;
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, "query_sequence", f_y, 4);
  g_y = NULL;
  m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
  *(uint32_T *)emlrtMxGetData(m) = u.source_first_row;
  emlrtAssign(&g_y, m);
  emlrtSetFieldR2017b(y, 0, "source_first_row", g_y, 5);
  h_y = NULL;
  m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
  *(uint32_T *)emlrtMxGetData(m) = u.source_last_row;
  emlrtAssign(&h_y, m);
  emlrtSetFieldR2017b(y, 0, "source_last_row", h_y, 6);
  i_y = NULL;
  m = emlrtCreateLogicalScalar(u.analytic_prefix_used);
  emlrtAssign(&i_y, m);
  emlrtSetFieldR2017b(y, 0, "analytic_prefix_used", i_y, 7);
  j_y = NULL;
  m = emlrtCreateLogicalScalar(u.reference_resampled);
  emlrtAssign(&j_y, m);
  emlrtSetFieldR2017b(y, 0, "reference_resampled", j_y, 8);
  k_y = NULL;
  m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
  *(uint32_T *)emlrtMxGetData(m) = u.hardware_actions;
  emlrtAssign(&k_y, m);
  emlrtSetFieldR2017b(y, 0, "hardware_actions", k_y, 9);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[70]
 */
static real_T (*ib_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[70]
{
  static const int32_T dims = 70;
  real_T(*ret)[70];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[70])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                struct51_T *y
 * Return Type  : void
 */
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct51_T *y)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[20] = {"schema",
                                         "capacity",
                                         "reference_asset_sha256",
                                         "leg_index",
                                         "window_generation",
                                         "source_first_row",
                                         "source_total_rows",
                                         "row_count",
                                         "time_s",
                                         "nominal_jet",
                                         "nominal_duration_s",
                                         "total_duration_s",
                                         "binding_mode",
                                         "prefix_duration_s",
                                         "prefix_coefficients",
                                         "ground_jet",
                                         "rest_jet",
                                         "relaunch_duration_s",
                                         "relaunch_offset_ned_m",
                                         "vertical_frame_offset_ned_m"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 20,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "schema";
  y->schema = k_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 0, "schema")),
      &thisId);
  thisId.fIdentifier = "capacity";
  y->capacity = l_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 1, "capacity")),
      &thisId);
  thisId.fIdentifier = "reference_asset_sha256";
  m_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 2,
                                                    "reference_asset_sha256")),
                     &thisId, y->reference_asset_sha256);
  thisId.fIdentifier = "leg_index";
  y->leg_index = k_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 3, "leg_index")),
      &thisId);
  thisId.fIdentifier = "window_generation";
  y->window_generation =
      n_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0, 4, "window_generation")),
                         &thisId);
  thisId.fIdentifier = "source_first_row";
  y->source_first_row =
      k_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                        5, "source_first_row")),
                         &thisId);
  thisId.fIdentifier = "source_total_rows";
  y->source_total_rows =
      k_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0, 6, "source_total_rows")),
                         &thisId);
  thisId.fIdentifier = "row_count";
  y->row_count = l_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 7, "row_count")),
      &thisId);
  thisId.fIdentifier = "time_s";
  o_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 8, "time_s")),
      &thisId, y->time_s);
  thisId.fIdentifier = "nominal_jet";
  p_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 9,
                                                    "nominal_jet")),
                     &thisId, y->nominal_jet);
  thisId.fIdentifier = "nominal_duration_s";
  y->nominal_duration_s = q_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 10,
                                     "nominal_duration_s")),
      &thisId);
  thisId.fIdentifier = "total_duration_s";
  y->total_duration_s =
      q_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0, 11, "total_duration_s")),
                         &thisId);
  thisId.fIdentifier = "binding_mode";
  y->binding_mode =
      r_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                        12, "binding_mode")),
                         &thisId);
  thisId.fIdentifier = "prefix_duration_s";
  y->prefix_duration_s =
      q_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0, 13, "prefix_duration_s")),
                         &thisId);
  thisId.fIdentifier = "prefix_coefficients";
  s_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 14,
                                                    "prefix_coefficients")),
                     &thisId, y->prefix_coefficients);
  thisId.fIdentifier = "ground_jet";
  t_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 15,
                                                    "ground_jet")),
                     &thisId, y->ground_jet);
  thisId.fIdentifier = "rest_jet";
  t_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 16, "rest_jet")),
      &thisId, y->rest_jet);
  thisId.fIdentifier = "relaunch_duration_s";
  y->relaunch_duration_s = q_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 17,
                                     "relaunch_duration_s")),
      &thisId);
  thisId.fIdentifier = "relaunch_offset_ned_m";
  u_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 18,
                                                    "relaunch_offset_ned_m")),
                     &thisId, y->relaunch_offset_ned_m);
  thisId.fIdentifier = "vertical_frame_offset_ned_m";
  y->vertical_frame_offset_ned_m = q_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 19,
                                     "vertical_frame_offset_ned_m")),
      &thisId);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const struct55_T *u
 * Return Type  : const mxArray *
 */
static const mxArray *j_emlrt_marshallOut(const struct55_T *u)
{
  static const int32_T i = 3;
  static const int32_T i1 = 3;
  static const int32_T i2 = 3;
  static const int32_T i3 = 3;
  static const int32_T i4 = 3;
  static const int32_T i5 = 3;
  static const char_T *sv[10] = {"reference",
                                 "phase_acceleration_s_inv",
                                 "phase_jerk_s_inv2",
                                 "outer_correction_i_mps2",
                                 "outer_correction_jerk_i_mps3",
                                 "fraction",
                                 "frame_i_from_f",
                                 "reference_frame_i_from_f",
                                 "reference_curvature",
                                 "reference_signed_yaw_rate"};
  static const char_T *sv1[4] = {"position_m", "velocity_mps",
                                 "acceleration_mps2", "jerk_mps3"};
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
  const mxArray *y;
  real_T *pData;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 10, (const char_T **)&sv[0]));
  b_y = NULL;
  emlrtAssign(&b_y, emlrtCreateStructMatrix(1, 1, 4, (const char_T **)&sv1[0]));
  c_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->reference.position_m[0];
  pData[1] = u->reference.position_m[1];
  pData[2] = u->reference.position_m[2];
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(b_y, 0, "position_m", c_y, 0);
  d_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i1, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->reference.velocity_mps[0];
  pData[1] = u->reference.velocity_mps[1];
  pData[2] = u->reference.velocity_mps[2];
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(b_y, 0, "velocity_mps", d_y, 1);
  e_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i2, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->reference.acceleration_mps2[0];
  pData[1] = u->reference.acceleration_mps2[1];
  pData[2] = u->reference.acceleration_mps2[2];
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(b_y, 0, "acceleration_mps2", e_y, 2);
  f_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i3, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->reference.jerk_mps3[0];
  pData[1] = u->reference.jerk_mps3[1];
  pData[2] = u->reference.jerk_mps3[2];
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(b_y, 0, "jerk_mps3", f_y, 3);
  emlrtSetFieldR2017b(y, 0, "reference", b_y, 0);
  g_y = NULL;
  m = emlrtCreateDoubleScalar(u->phase_acceleration_s_inv);
  emlrtAssign(&g_y, m);
  emlrtSetFieldR2017b(y, 0, "phase_acceleration_s_inv", g_y, 1);
  h_y = NULL;
  m = emlrtCreateDoubleScalar(u->phase_jerk_s_inv2);
  emlrtAssign(&h_y, m);
  emlrtSetFieldR2017b(y, 0, "phase_jerk_s_inv2", h_y, 2);
  i_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i4, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->outer_correction_i_mps2[0];
  pData[1] = u->outer_correction_i_mps2[1];
  pData[2] = u->outer_correction_i_mps2[2];
  emlrtAssign(&i_y, m);
  emlrtSetFieldR2017b(y, 0, "outer_correction_i_mps2", i_y, 3);
  j_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i5, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->outer_correction_jerk_i_mps3[0];
  pData[1] = u->outer_correction_jerk_i_mps3[1];
  pData[2] = u->outer_correction_jerk_i_mps3[2];
  emlrtAssign(&j_y, m);
  emlrtSetFieldR2017b(y, 0, "outer_correction_jerk_i_mps3", j_y, 4);
  k_y = NULL;
  m = emlrtCreateDoubleScalar(u->fraction);
  emlrtAssign(&k_y, m);
  emlrtSetFieldR2017b(y, 0, "fraction", k_y, 5);
  emlrtSetFieldR2017b(y, 0, "frame_i_from_f",
                      k_emlrt_marshallOut(u->frame_i_from_f), 6);
  emlrtSetFieldR2017b(y, 0, "reference_frame_i_from_f",
                      k_emlrt_marshallOut(u->reference_frame_i_from_f), 7);
  l_y = NULL;
  m = emlrtCreateDoubleScalar(u->reference_curvature);
  emlrtAssign(&l_y, m);
  emlrtSetFieldR2017b(y, 0, "reference_curvature", l_y, 8);
  m_y = NULL;
  m = emlrtCreateDoubleScalar(u->reference_signed_yaw_rate);
  emlrtAssign(&m_y, m);
  emlrtSetFieldR2017b(y, 0, "reference_signed_yaw_rate", m_y, 9);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : uint32_T
 */
static uint32_T jb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  uint32_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "uint32", false, 0U,
                          (const void *)&dims);
  ret = *(uint32_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : uint32_T
 */
static uint32_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId)
{
  uint32_T y;
  y = jb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const real_T u[9]
 * Return Type  : const mxArray *
 */
static const mxArray *k_emlrt_marshallOut(const real_T u[9])
{
  static const int32_T iv[2] = {3, 3};
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T i;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (i = 0; i < 3; i++) {
    pData[i * 3] = u[3 * i];
    pData[i * 3 + 1] = u[3 * i + 1];
    pData[i * 3 + 2] = u[3 * i + 2];
  }
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : uint16_T
 */
static uint16_T kb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  uint16_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "uint16", false, 0U,
                          (const void *)&dims);
  ret = *(uint16_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : uint16_T
 */
static uint16_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId)
{
  uint16_T y;
  y = kb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                uint8_T ret[32]
 * Return Type  : void
 */
static void lb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                uint8_T ret[32])
{
  static const int32_T dims = 32;
  int32_T i;
  uint8_T(*r)[32];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "uint8", false, 1U,
                          (const void *)&dims);
  r = (uint8_T(*)[32])emlrtMxGetData(src);
  for (i = 0; i < 32; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                uint8_T y[32]
 * Return Type  : void
 */
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               uint8_T y[32])
{
  lb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : uint64_T
 */
static uint64_T mb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  uint64_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "uint64", false, 0U,
                          (const void *)&dims);
  ret = *(uint64_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : uint64_T
 */
static uint64_T n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId)
{
  uint64_T y;
  y = mb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[256]
 * Return Type  : void
 */
static void nb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                real_T ret[256])
{
  static const int32_T dims = 256;
  real_T(*r)[256];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[256])emlrtMxGetData(src);
  for (i = 0; i < 256; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[256]
 * Return Type  : void
 */
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[256])
{
  nb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[3072]
 * Return Type  : void
 */
static void ob_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                real_T ret[3072])
{
  static const int32_T dims[3] = {256, 3, 4};
  real_T(*r)[3072];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 3U,
                          (const void *)&dims[0]);
  r = (real_T(*)[3072])emlrtMxGetData(src);
  for (i = 0; i < 3072; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[3072]
 * Return Type  : void
 */
static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[3072])
{
  ob_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T pb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                  const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = pb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : uint8_T
 */
static uint8_T qb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  uint8_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "uint8", false, 0U,
                          (const void *)&dims);
  ret = *(uint8_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : uint8_T
 */
static uint8_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId)
{
  uint8_T y;
  y = qb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[192]
 * Return Type  : void
 */
static void rb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId,
                                real_T ret[192])
{
  static const int32_T dims[4] = {3, 8, 4, 2};
  real_T(*r)[192];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 4U,
                          (const void *)&dims[0]);
  r = (real_T(*)[192])emlrtMxGetData(src);
  for (i = 0; i < 192; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[192]
 * Return Type  : void
 */
static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[192])
{
  rb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[12]
 * Return Type  : void
 */
static void sb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, real_T ret[12])
{
  static const int32_T dims[2] = {3, 4};
  real_T(*r)[12];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[12])emlrtMxGetData(src);
  for (i = 0; i < 12; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[12]
 * Return Type  : void
 */
static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[12])
{
  sb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T ret[3]
 * Return Type  : void
 */
static void tb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                const emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims = 3;
  real_T(*r)[3];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T y[3]
 * Return Type  : void
 */
static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3])
{
  tb_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[12]
 */
static real_T (*ub_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[12]
{
  static const int32_T dims[2] = {3, 4};
  real_T(*ret)[12];
  int32_T iv[2];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real_T(*)[12])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : struct52_T
 */
static struct52_T v_emlrt_marshallIn(const emlrtStack *sp,
                                     const mxArray *nullptr,
                                     const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  struct52_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = w_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[3]
 */
static real_T (*vb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims = 3;
  real_T(*ret)[3];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : struct52_T
 */
static struct52_T w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[4] = {"reference_asset_sha256", "leg_index",
                                        "window_generation",
                                        "last_accepted_sequence"};
  emlrtMsgIdentifier thisId;
  struct52_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 4,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "reference_asset_sha256";
  m_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 0,
                                                    "reference_asset_sha256")),
                     &thisId, y.reference_asset_sha256);
  thisId.fIdentifier = "leg_index";
  y.leg_index = k_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 1, "leg_index")),
      &thisId);
  thisId.fIdentifier = "window_generation";
  y.window_generation =
      n_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0, 2, "window_generation")),
                         &thisId);
  thisId.fIdentifier = "last_accepted_sequence";
  y.last_accepted_sequence = n_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 3,
                                     "last_accepted_sequence")),
      &thisId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *nullptr
 *                const char_T *identifier
 * Return Type  : struct53_T
 */
static struct53_T x_emlrt_marshallIn(const emlrtStack *sp,
                                     const mxArray *nullptr,
                                     const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  struct53_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = y_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : struct53_T
 */
static struct53_T y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims = 0;
  static const char_T *fieldNames[5] = {"reference_asset_sha256", "leg_index",
                                        "window_generation", "query_sequence",
                                        "progress_s"};
  emlrtMsgIdentifier thisId;
  struct53_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 5,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "reference_asset_sha256";
  m_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 0,
                                                    "reference_asset_sha256")),
                     &thisId, y.reference_asset_sha256);
  thisId.fIdentifier = "leg_index";
  y.leg_index = k_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 1, "leg_index")),
      &thisId);
  thisId.fIdentifier = "window_generation";
  y.window_generation =
      n_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0, 2, "window_generation")),
                         &thisId);
  thisId.fIdentifier = "query_sequence";
  y.query_sequence =
      n_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0,
                                                        3, "query_sequence")),
                         &thisId);
  thisId.fIdentifier = "progress_s";
  y.progress_s = q_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0, 4, "progress_s")),
      &thisId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[3]
 *                int32_T nlhs
 *                const mxArray *plhs[6]
 * Return Type  : void
 */
void c_gpenmpcNative_canonicalLocalIn(const mxArray *const prhs[3], int32_T nlhs,
                                     const mxArray *plhs[6])
{
  static const char_T *s = "numerics";
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*scaffold70)[70];
  real_T(*next64)[64];
  real_T(*kernel61)[61];
  real_T(*input36)[36];
  real_T(*request19)[19];
  real_T(*learning12)[12];
  real_T(*closed5)[5];
  uint64_T(*inputTags2)[2];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  next64 = (real_T(*)[64])mxMalloc(sizeof(real_T[64]));
  kernel61 = (real_T(*)[61])mxMalloc(sizeof(real_T[61]));
  scaffold70 = (real_T(*)[70])mxMalloc(sizeof(real_T[70]));
  request19 = (real_T(*)[19])mxMalloc(sizeof(real_T[19]));
  closed5 = (real_T(*)[5])mxMalloc(sizeof(real_T[5]));
  learning12 = (real_T(*)[12])mxMalloc(sizeof(real_T[12]));
  /* Check constant function inputs */
  i = 4;
  emlrtCheckArrayChecksumR2018b(&st, prhs[0], false, &i, (const char_T **)&s,
                                &uv[0]);
  /* Marshall function inputs */
  input36 = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "input36");
  inputTags2 = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "inputTags2");
  /* Invoke the target function */
  gpenmpcNative_canonicalLocalInnerWithAuditFirst(
      *input36, *inputTags2, *next64, *kernel61, *scaffold70, *request19,
      *closed5, *learning12);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*next64);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*kernel61);
  }
  if (nlhs > 2) {
    plhs[2] = c_emlrt_marshallOut(*scaffold70);
  }
  if (nlhs > 3) {
    plhs[3] = d_emlrt_marshallOut(*request19);
  }
  if (nlhs > 4) {
    plhs[4] = e_emlrt_marshallOut(*closed5);
  }
  if (nlhs > 5) {
    plhs[5] = f_emlrt_marshallOut(*learning12);
  }
}

/*
 * Arguments    : const mxArray * const prhs[8]
 *                const mxArray **plhs
 * Return Type  : void
 */
void c_gpenmpcNative_canonicalReferen(const mxArray *const prhs[8],
                                     const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *prhs_copy_idx_5;
  struct55_T transition;
  real_T(*trajectoryJet)[12];
  real_T(*previousOuterCorrectionI)[3];
  real_T(*targetOuterCorrectionF)[3];
  real_T dtS;
  real_T jerkLimitMps3;
  real_T previousPhaseAcceleration;
  real_T progressRate;
  real_T targetPhaseAcceleration;
  st.tls = emlrtRootTLSGlobal;
  prhs_copy_idx_5 = emlrtProtectR2012b(prhs[5], 5, false, -1);
  /* Marshall function inputs */
  trajectoryJet =
      ab_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "trajectoryJet");
  progressRate = cb_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "progressRate");
  previousPhaseAcceleration = cb_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]),
                                                  "previousPhaseAcceleration");
  targetPhaseAcceleration =
      cb_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "targetPhaseAcceleration");
  previousOuterCorrectionI =
      db_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "previousOuterCorrectionI");
  targetOuterCorrectionF = db_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_5),
                                               "targetOuterCorrectionF");
  dtS = cb_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "dtS");
  jerkLimitMps3 =
      cb_emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "jerkLimitMps3");
  /* Invoke the target function */
  gpenmpcNative_canonicalReferenceTransitionFromJet(
      *trajectoryJet, progressRate, previousPhaseAcceleration,
      targetPhaseAcceleration, *previousOuterCorrectionI,
      *targetOuterCorrectionF, dtS, jerkLimitMps3, &transition);
  /* Marshall function outputs */
  *plhs = j_emlrt_marshallOut(&transition);
}

/*
 * Arguments    : e_gpenmpcNative_canonicalLocalIn *SD
 *                const mxArray * const prhs[3]
 *                int32_T nlhs
 *                const mxArray *plhs[3]
 * Return Type  : void
 */
void c_gpenmpcNative_queryCanonicalRe(e_gpenmpcNative_canonicalLocalIn *SD,
                                     const mxArray *const prhs[3], int32_T nlhs,
                                     const mxArray *plhs[3])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  struct52_T next;
  struct52_T state;
  struct53_T request;
  struct54_T receipt;
  real_T(*jet)[12];
  st.tls = emlrtRootTLSGlobal;
  jet = (real_T(*)[12])mxMalloc(sizeof(real_T[12]));
  /* Marshall function inputs */
  i_emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "window", &SD->f0.window);
  state = v_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "state");
  request = x_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "request");
  /* Invoke the target function */
  gpenmpcNative_queryCanonicalReferenceWindow(&SD->f0.window, &state, &request,
                                             &next, *jet, &receipt);
  /* Marshall function outputs */
  plhs[0] = g_emlrt_marshallOut(next);
  if (nlhs > 1) {
    plhs[1] = h_emlrt_marshallOut(*jet);
  }
  if (nlhs > 2) {
    plhs[2] = i_emlrt_marshallOut(receipt);
  }
}

/*
 * Arguments    : const mxArray * const prhs[7]
 *                int32_T nlhs
 *                const mxArray *plhs[6]
 * Return Type  : void
 */
void d_gpenmpcNative_canonicalLocalIn(const mxArray *const prhs[7], int32_T nlhs,
                                     const mxArray *plhs[6])
{
  static const char_T *s = "numerics";
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*pending70)[70];
  real_T(*scaffold70)[70];
  real_T(*next64)[64];
  real_T(*state64)[64];
  real_T(*kernel61)[61];
  real_T(*input36)[36];
  real_T(*request19)[19];
  real_T(*learning12)[12];
  real_T(*closed5)[5];
  uint64_T(*inputTags2)[2];
  uint64_T(*pendingTags2)[2];
  uint64_T(*stateTags2)[2];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  next64 = (real_T(*)[64])mxMalloc(sizeof(real_T[64]));
  kernel61 = (real_T(*)[61])mxMalloc(sizeof(real_T[61]));
  scaffold70 = (real_T(*)[70])mxMalloc(sizeof(real_T[70]));
  request19 = (real_T(*)[19])mxMalloc(sizeof(real_T[19]));
  closed5 = (real_T(*)[5])mxMalloc(sizeof(real_T[5]));
  learning12 = (real_T(*)[12])mxMalloc(sizeof(real_T[12]));
  /* Check constant function inputs */
  i = 4;
  emlrtCheckArrayChecksumR2018b(&st, prhs[0], false, &i, (const char_T **)&s,
                                &uv[0]);
  /* Marshall function inputs */
  state64 = e_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "state64");
  stateTags2 = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "stateTags2");
  input36 = emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "input36");
  inputTags2 = c_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "inputTags2");
  pending70 = g_emlrt_marshallIn(&st, emlrtAlias(prhs[5]), "pending70");
  pendingTags2 = c_emlrt_marshallIn(&st, emlrtAlias(prhs[6]), "pendingTags2");
  /* Invoke the target function */
  gpenmpcNative_canonicalLocalInnerWithAuditStep(
      *state64, *stateTags2, *input36, *inputTags2, *pending70, *pendingTags2,
      *next64, *kernel61, *scaffold70, *request19, *closed5, *learning12);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*next64);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*kernel61);
  }
  if (nlhs > 2) {
    plhs[2] = c_emlrt_marshallOut(*scaffold70);
  }
  if (nlhs > 3) {
    plhs[3] = d_emlrt_marshallOut(*request19);
  }
  if (nlhs > 4) {
    plhs[4] = e_emlrt_marshallOut(*closed5);
  }
  if (nlhs > 5) {
    plhs[5] = f_emlrt_marshallOut(*learning12);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void gpenmpcNative_canonicalLocalInnerWithAuditFirst_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtPushHeapReferenceStackR2021a(
      &st, false, NULL, (void *)&emlrtExitTimeCleanupDtorFcn, NULL, NULL, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  gpenmpcNative_canonicalLocalInnerWithAuditFirst_xil_terminate();
  gpenmpcNative_canonicalLocalInnerWithAuditFirst_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void gpenmpcNative_canonicalLocalInnerWithAuditFirst_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void gpenmpcNative_canonicalLocalInnerWithAuditFirst_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_gpenmpcNative_canonicalLocalInnerWithAuditFirst_api.c
 *
 * [EOF]
 */
