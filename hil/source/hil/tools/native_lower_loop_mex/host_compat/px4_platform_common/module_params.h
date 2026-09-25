#pragma once
// Host parameter adapter for allocator tests; require MC_AIRMODE=0.
#include <cstdint>
namespace px4 { enum class params { MC_AIRMODE }; }
template<px4::params P> class ParamInt {
    static_assert(P == px4::params::MC_AIRMODE, "Unknown host parameter");
public:
    int32_t get() const { return 0; }
    void update() {}
};
class ModuleParams {
public:
    explicit ModuleParams(ModuleParams *) {}
    virtual ~ModuleParams() = default;
protected:
    void updateParams() {}
};
// This source class declares exactly one (type) variable parameter.
#define PX4_HOST_PARAMETER_TYPE(...) __VA_ARGS__
#define DEFINE_PARAMETERS(x) PX4_HOST_PARAMETER_TYPE x
