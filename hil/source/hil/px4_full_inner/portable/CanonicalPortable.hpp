#pragma once
// Fixed storage and scalar utilities shared identically by HOST and NuttX.
// No std namespace additions, heap, alternate host alias, or control policy.
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <math.h>

namespace gpenmpc_portable {
// Exact field/ABI identity is intentional here: no tolerance, bytewise NaN
// equivalence, or signed-zero distinction is introduced. Keep the target's
// -Wfloat-equal active everywhere else rather than weakening compiler flags.
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#endif
template<class T> constexpr bool exact_equal(const T &a,const T &b) noexcept { return a == b; }
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
template<class T, std::size_t N> struct Array {
    static_assert(N > 0, "This fixed-capacity ABI does not use zero-size arrays");
    T elements[N];
    constexpr T *data() noexcept { return elements; }
    constexpr const T *data() const noexcept { return elements; }
    constexpr T *begin() noexcept { return elements; }
    constexpr const T *begin() const noexcept { return elements; }
    constexpr T *end() noexcept { return elements + N; }
    constexpr const T *end() const noexcept { return elements + N; }
    constexpr std::size_t size() const noexcept { return N; }
    constexpr bool empty() const noexcept { return false; }
    constexpr T &operator[](std::size_t i) noexcept { return elements[i]; }
    constexpr const T &operator[](std::size_t i) const noexcept { return elements[i]; }
    constexpr T &front() noexcept { return elements[0]; }
    constexpr const T &front() const noexcept { return elements[0]; }
    constexpr T &back() noexcept { return elements[N - 1]; }
    constexpr const T &back() const noexcept { return elements[N - 1]; }
    constexpr void fill(const T &value) noexcept { for (auto &element : elements) element = value; }
};
template<class T, std::size_t N>
constexpr bool operator==(const Array<T,N> &a,const Array<T,N> &b) noexcept {
    for (std::size_t i=0;i<N;++i) if (!exact_equal(a[i],b[i])) return false;
    return true;
}
template<class T, std::size_t N>
constexpr bool operator!=(const Array<T,N> &a,const Array<T,N> &b) noexcept { return !(a == b); }
template<class Input,class Output>
inline Output copy(Input first,Input last,Output output) noexcept {
    while (first != last) { *output = *first; ++output; ++first; }
    return output;
}
template<class Input,class Count,class Output>
inline Output copy_n(Input first,Count count,Output output) noexcept {
    for (Count i=0;i<count;++i) { *output = *first; ++output; ++first; }
    return output;
}
template<class T> constexpr const T &min(const T &a,const T &b) noexcept { return b < a ? b : a; }
template<class T> constexpr const T &max(const T &a,const T &b) noexcept { return a < b ? b : a; }
// NuttX exposes the actual C rounding routines but omits std::lround overloads.
// Delegate to matching precision; do not recalculate or change tie handling.
inline long lround(float value) noexcept { return ::lroundf(value); }
inline long lround(double value) noexcept { return ::lround(value); }

template<class T> struct Ieee754;
template<> struct Ieee754<float> {
    static constexpr bool is_iec559 = sizeof(float) == 4 && __FLT_RADIX__ == 2 &&
        __FLT_MANT_DIG__ == 24 && __FLT_MAX_EXP__ == 128 && __FLT_MIN_EXP__ == -125;
    static float quiet_NaN() noexcept {
        static_assert(is_iec559,"Requires IEEE binary32 storage");
        const std::uint32_t bits=UINT32_C(0x7fc00000);float value;
        std::memcpy(&value,&bits,sizeof(value));return value;
    }
};
template<> struct Ieee754<double> {
    static constexpr bool is_iec559 = sizeof(double) == 8 && __FLT_RADIX__ == 2 &&
        __DBL_MANT_DIG__ == 53 && __DBL_MAX_EXP__ == 1024 && __DBL_MIN_EXP__ == -1021;
    static double quiet_NaN() noexcept {
        static_assert(is_iec559,"Requires IEEE binary64 storage");
        const std::uint64_t bits=UINT64_C(0x7ff8000000000000);double value;
        std::memcpy(&value,&bits,sizeof(value));return value;
    }
};
} // namespace gpenmpc_portable
