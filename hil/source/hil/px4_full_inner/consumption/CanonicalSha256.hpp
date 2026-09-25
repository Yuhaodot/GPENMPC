#pragma once
#include "../portable/CanonicalPortable.hpp"
#include <cstdint>
#include <cstring>

namespace gpenmpc_consumption {

// Fixed-storage SHA-256 with explicit big-endian canonical scalar encoding.
// No hashing of C++ padding, host endianness, or compiler-specific layout.
class CanonicalSha256 final {
public:
    void byte(std::uint8_t x) noexcept {
        block_[used_++] = x; ++bytes_;
        if (used_ == 64) { transform(); used_ = 0; }
    }
    void u32(std::uint32_t x) noexcept { for (int n = 24; n >= 0; n -= 8) byte(static_cast<std::uint8_t>(x >> n)); }
    void u64(std::uint64_t x) noexcept { for (int n = 56; n >= 0; n -= 8) byte(static_cast<std::uint8_t>(x >> n)); }
    void real(double x) noexcept {
        static_assert(sizeof(double) == 8 && gpenmpc_portable::Ieee754<double>::is_iec559, "requires IEEE binary64");
        std::uint64_t bits = 0; std::memcpy(&bits, &x, sizeof(bits)); u64(bits);
    }
    template<std::size_t N> void reals(const gpenmpc_portable::Array<double, N> &a) noexcept { for (double x : a) real(x); }
    template<std::size_t N> void words(const gpenmpc_portable::Array<std::uint32_t, N> &a) noexcept { for (auto x : a) u32(x); }
    gpenmpc_portable::Array<std::uint32_t, 8> finish() noexcept {
        const std::uint64_t bit_count = bytes_ * 8;
        byte(0x80);
        while (used_ != 56) byte(0);
        u64(bit_count);
        return state_;
    }
private:
    static std::uint32_t rr(std::uint32_t v, unsigned n) noexcept { return (v >> n) | (v << (32 - n)); }
    void transform() noexcept {
        static constexpr std::uint32_t k[64] = {
            0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
            0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
            0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
            0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
            0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
            0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
            0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
            0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
        std::uint32_t w[64]{};
        for (unsigned n = 0; n < 16; ++n) {
            for (unsigned j = 0; j < 4; ++j) w[n] = (w[n] << 8) | block_[4 * n + j];
        }
        for (unsigned n = 16; n < 64; ++n) {
            const auto x = w[n - 15], y = w[n - 2];
            w[n] = w[n - 16] + (rr(x,7) ^ rr(x,18) ^ (x >> 3)) + w[n - 7] + (rr(y,17) ^ rr(y,19) ^ (y >> 10));
        }
        auto a=state_[0], b=state_[1], c=state_[2], d=state_[3], e=state_[4], f=state_[5], g=state_[6], h=state_[7];
        for (unsigned n=0; n<64; ++n) {
            const auto t1=h+(rr(e,6)^rr(e,11)^rr(e,25))+((e&f)^(~e&g))+k[n]+w[n];
            const auto t2=(rr(a,2)^rr(a,13)^rr(a,22))+((a&b)^(a&c)^(b&c));
            h=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
        }
        state_[0]+=a; state_[1]+=b; state_[2]+=c; state_[3]+=d;
        state_[4]+=e; state_[5]+=f; state_[6]+=g; state_[7]+=h;
    }
    gpenmpc_portable::Array<std::uint32_t,8> state_{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
    gpenmpc_portable::Array<std::uint8_t,64> block_{}; unsigned used_{0}; std::uint64_t bytes_{0};
};
} // namespace gpenmpc_consumption
