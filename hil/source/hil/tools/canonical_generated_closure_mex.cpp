// Host numerical probe for three generated C sources.
#include "mex.h"
#include <cmath>
#include <cstdint>
#include <cstring>
extern "C" int gpenmpc_generated_closure_call(const double *, double *, float *);

static std::uint64_t read_be(const std::uint8_t *p)
{
    std::uint64_t value = 0;
    for (unsigned k = 0; k < 8; ++k) { value = (value << 8) | p[k]; }
    return value;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs != 1 || nlhs != 3 || !mxIsUint8(prhs[0]) || mxIsComplex(prhs[0])
        || mxGetNumberOfElements(prhs[0]) != 829) {
        mexErrMsgIdAndTxt("gpenmpc:GeneratedClosureShape", "One uint8 RAK1[829] input; exactly three outputs.");
    }
    const auto *bytes = static_cast<const std::uint8_t *>(mxGetData(prhs[0]));
    if (std::memcmp(bytes, "RAK1", 4) != 0 || bytes[276] != 1) {
        mexErrMsgIdAndTxt("gpenmpc:GeneratedClosureAbi", "Original RAK1 and enabled continuity required.");
    }
    const auto generation = read_be(bytes + 813);
    if (generation == 0 || generation != read_be(bytes + 821)) {
        mexErrMsgIdAndTxt("gpenmpc:GeneratedClosureGeneration", "Equal nonzero numerical generations are required.");
    }
    double arguments[101];
    for (unsigned k = 0; k < 101; ++k) {
        const unsigned offset = 4 + k * 8 + (k >= 34 ? 1 : 0);
        const auto bits = read_be(bytes + offset);
        double value;
        static_assert(sizeof(value) == sizeof(bits), "IEEE binary64 required");
        std::memcpy(&value, &bits, sizeof(value));
        if (!std::isfinite(value)) {
            mexErrMsgIdAndTxt("gpenmpc:GeneratedClosureFinite", "Finite binary64 numerical input required.");
        }
        arguments[k] = value;
    }
    plhs[0] = mxCreateDoubleMatrix(61, 1, mxREAL);
    plhs[1] = mxCreateNumericMatrix(16, 1, mxSINGLE_CLASS, mxREAL);
    const int valid = gpenmpc_generated_closure_call(arguments, mxGetDoubles(plhs[0]), mxGetSingles(plhs[1]));
    plhs[2] = mxCreateLogicalScalar(valid != 0);
}
