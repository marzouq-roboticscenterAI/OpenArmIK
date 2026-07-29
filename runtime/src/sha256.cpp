/* SPDX-License-Identifier: Apache-2.0 */
#include "runtime_internal.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace openarm::runtime {
namespace {

constexpr std::array<std::uint32_t, 64> k{
    0x428a2f98U,0x71374491U,0xb5c0fbcfU,0xe9b5dba5U,0x3956c25bU,0x59f111f1U,0x923f82a4U,0xab1c5ed5U,
    0xd807aa98U,0x12835b01U,0x243185beU,0x550c7dc3U,0x72be5d74U,0x80deb1feU,0x9bdc06a7U,0xc19bf174U,
    0xe49b69c1U,0xefbe4786U,0x0fc19dc6U,0x240ca1ccU,0x2de92c6fU,0x4a7484aaU,0x5cb0a9dcU,0x76f988daU,
    0x983e5152U,0xa831c66dU,0xb00327c8U,0xbf597fc7U,0xc6e00bf3U,0xd5a79147U,0x06ca6351U,0x14292967U,
    0x27b70a85U,0x2e1b2138U,0x4d2c6dfcU,0x53380d13U,0x650a7354U,0x766a0abbU,0x81c2c92eU,0x92722c85U,
    0xa2bfe8a1U,0xa81a664bU,0xc24b8b70U,0xc76c51a3U,0xd192e819U,0xd6990624U,0xf40e3585U,0x106aa070U,
    0x19a4c116U,0x1e376c08U,0x2748774cU,0x34b0bcb5U,0x391c0cb3U,0x4ed8aa4aU,0x5b9cca4fU,0x682e6ff3U,
    0x748f82eeU,0x78a5636fU,0x84c87814U,0x8cc70208U,0x90befffaU,0xa4506cebU,0xbef9a3f7U,0xc67178f2U};

std::uint32_t rotate(std::uint32_t value, unsigned amount) {
    return (value >> amount) | (value << (32U - amount));
}

void block(std::array<std::uint32_t, 8> &state, const std::uint8_t *input) {
    std::array<std::uint32_t, 64> w{};
    for (std::size_t i = 0U; i < 16U; ++i) {
        w[i] = (static_cast<std::uint32_t>(input[i * 4U]) << 24U) |
               (static_cast<std::uint32_t>(input[i * 4U + 1U]) << 16U) |
               (static_cast<std::uint32_t>(input[i * 4U + 2U]) << 8U) |
               static_cast<std::uint32_t>(input[i * 4U + 3U]);
    }
    for (std::size_t i = 16U; i < 64U; ++i) {
        const std::uint32_t s0 = rotate(w[i - 15U], 7U) ^ rotate(w[i - 15U], 18U) ^
                                 (w[i - 15U] >> 3U);
        const std::uint32_t s1 = rotate(w[i - 2U], 17U) ^ rotate(w[i - 2U], 19U) ^
                                 (w[i - 2U] >> 10U);
        w[i] = w[i - 16U] + s0 + w[i - 7U] + s1;
    }
    std::uint32_t a=state[0], b=state[1], c=state[2], d=state[3];
    std::uint32_t e=state[4], f=state[5], g=state[6], h=state[7];
    for (std::size_t i = 0U; i < 64U; ++i) {
        const std::uint32_t s1 = rotate(e,6U) ^ rotate(e,11U) ^ rotate(e,25U);
        const std::uint32_t choice = (e & f) ^ ((~e) & g);
        const std::uint32_t temp1 = h + s1 + choice + k[i] + w[i];
        const std::uint32_t s0 = rotate(a,2U) ^ rotate(a,13U) ^ rotate(a,22U);
        const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
        const std::uint32_t temp2 = s0 + majority;
        h=g; g=f; f=e; e=d+temp1; d=c; c=b; b=a; a=temp1+temp2;
    }
    state[0]+=a; state[1]+=b; state[2]+=c; state[3]+=d;
    state[4]+=e; state[5]+=f; state[6]+=g; state[7]+=h;
}

}

std::string sha256_hex(const std::uint8_t *data, std::size_t size) {
    std::array<std::uint32_t, 8> state{
        0x6a09e667U,0xbb67ae85U,0x3c6ef372U,0xa54ff53aU,
        0x510e527fU,0x9b05688cU,0x1f83d9abU,0x5be0cd19U};
    std::size_t offset = 0U;
    while (size - offset >= 64U) {
        block(state, data + offset);
        offset += 64U;
    }
    std::array<std::uint8_t, 128> tail{};
    const std::size_t remaining = size - offset;
    for (std::size_t i = 0U; i < remaining; ++i) tail[i] = data[offset + i];
    tail[remaining] = 0x80U;
    const std::size_t padded = remaining < 56U ? 64U : 128U;
    const std::uint64_t bits = static_cast<std::uint64_t>(size) * 8U;
    for (std::size_t i = 0U; i < 8U; ++i) {
        tail[padded - 1U - i] = static_cast<std::uint8_t>(bits >> (i * 8U));
    }
    block(state, tail.data());
    if (padded == 128U) block(state, tail.data() + 64U);
    constexpr char hex[] = "0123456789abcdef";
    std::string result(64U, '0');
    for (std::size_t i = 0U; i < 32U; ++i) {
        const std::uint8_t byte = static_cast<std::uint8_t>(state[i / 4U] >>
                                      (24U - static_cast<unsigned>(i % 4U) * 8U));
        result[i * 2U] = hex[byte >> 4U];
        result[i * 2U + 1U] = hex[byte & 0xfU];
    }
    return result;
}

std::string hmac_sha256_hex(
    const std::array<std::uint8_t, OA_RUNTIME_PERSISTENCE_KEY_BYTES> &key,
    const std::string &data) {
    std::array<std::uint8_t, 64U> inner_pad{};
    std::array<std::uint8_t, 64U> outer_pad{};
    for (std::size_t index = 0U; index < inner_pad.size(); ++index) {
        const std::uint8_t key_byte = index < key.size() ? key[index] : 0U;
        inner_pad[index] = static_cast<std::uint8_t>(key_byte ^ UINT8_C(0x36));
        outer_pad[index] = static_cast<std::uint8_t>(key_byte ^ UINT8_C(0x5c));
    }
    std::string inner(reinterpret_cast<const char *>(inner_pad.data()), inner_pad.size());
    inner += data;
    const std::string inner_hex = sha256_hex(inner);
    std::array<std::uint8_t, 32U> inner_digest{};
    const auto nibble = [](const char value) -> std::uint8_t {
        return value >= 'a' ? static_cast<std::uint8_t>(value - 'a' + 10)
                            : static_cast<std::uint8_t>(value - '0');
    };
    for (std::size_t index = 0U; index < inner_digest.size(); ++index) {
        inner_digest[index] = static_cast<std::uint8_t>(
            static_cast<std::uint8_t>(nibble(inner_hex[index * 2U]) << 4U) |
            nibble(inner_hex[index * 2U + 1U]));
    }
    std::string outer(reinterpret_cast<const char *>(outer_pad.data()), outer_pad.size());
    outer.append(reinterpret_cast<const char *>(inner_digest.data()), inner_digest.size());
    return sha256_hex(outer);
}

}

#ifdef OA_RUNTIME_ENABLE_TEST_HOOKS
extern "C" int oa_runtime_test_hmac_sha256_known_vector(void) {
    std::array<std::uint8_t, OA_RUNTIME_PERSISTENCE_KEY_BYTES> key{};
    std::fill_n(key.begin(), 20U, UINT8_C(0x0b));
    return openarm::runtime::hmac_sha256_hex(key, "Hi There") ==
                   "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7"
               ? 1
               : 0;
}
#endif
