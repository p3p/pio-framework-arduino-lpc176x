#pragma once

#include <cstdint>

namespace util {

template <typename Bit>
[[nodiscard]] constexpr auto bit_value(const Bit bit) noexcept {
  return (static_cast<uint32_t>(1) << bit);
}

template <typename Value, typename Bit>
[[nodiscard]] constexpr bool bit_test(const Value& val, const Bit bit) noexcept {
  return val & bit_value(bit);
}

template<typename... Args>
[[nodiscard]] constexpr auto bitset_value(Args... args) noexcept {
  return (... | bit_value(args));
}

template <typename Value, typename BitSet>
[[nodiscard]] constexpr auto bitset_mask(const Value val, const BitSet bitset) noexcept {
  return val & bitset;
}

template <typename Value, typename Bit>
constexpr void bit_set(Value& val, const Bit bit) noexcept {
  val |= bit_value(bit);
}

template <typename Value, typename Bit>
constexpr void bit_clear(Value& val, const Bit bit) noexcept {
  val &= ~bit_value(bit);
}

template <typename Value, typename BitSet>
constexpr void bitset_set(Value& val, const BitSet bitset) noexcept {
  val |= bitset;
}

template <typename Value, typename BitSet>
constexpr void bitset_clear(Value& val, const BitSet bitset) noexcept {
  val &= ~bitset;
}

}
