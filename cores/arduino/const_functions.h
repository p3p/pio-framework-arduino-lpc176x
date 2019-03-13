#pragma once

#include <cstdint>
#include <cmath>
#include <type_traits>

namespace util {

template <class L, class R>
constexpr auto min(const L lhs, const R rhs) noexcept -> decltype(lhs + rhs) {
  return lhs < rhs ? lhs : rhs;
}

template <class L, class R>
constexpr auto max(const L lhs, const R rhs) noexcept -> decltype(lhs + rhs) {
  return lhs > rhs ? lhs : rhs;
}

template <class T>
constexpr const T abs(const T v) noexcept {
  return v >= 0 ? v : -v;
}

template <typename T, std::size_t N>
constexpr std::size_t count(T const (&)[N]) noexcept {
  return N;
}

template <class L, class R>
constexpr auto difference(const L lhs, const R rhs) noexcept {
  return static_cast<std::make_signed_t<typeof(lhs + rhs)>>(lhs - rhs);
}

template <class L, class R>
constexpr bool pending(const L now, const R soon) noexcept {
  return difference(now, soon) < 0;
}

template <class L, class R>
constexpr bool elapsed(const L now, const R soon) noexcept {
  return !pending(now, soon);
}

template <class V, class N1, class N2>
constexpr bool within(const V v, const N1 n1, const N2 n2) noexcept {
  return (v) >= (n1) && (v) <= (n2);
}

template <class V, class N>
constexpr void noless(V& v, const N n) noexcept {
  if (v < n) v = n;
}

template <class V, class N>
constexpr void nomore(V& v, const N n) noexcept {
  if (v > n) v = n;
}

template <class V, class N1, class N2>
constexpr void limit(V& v, const N1 n1, const N2 n2) noexcept {
  if (v < n1) v = n1;
  else if (v > n2) v = n2;
}

template <typename Bit>
[[nodiscard]] constexpr auto bit_value(const Bit bit) noexcept {
  return (static_cast<uint32_t>(1) << bit);
}

template <typename Value, typename Bit>
[[nodiscard]] constexpr bool bit_test(const Value& val, const Bit bit) noexcept {
  return val & bit_value(bit);
}

template <typename Value, typename Bit>
constexpr void bit_set(Value& val, const Bit bit) noexcept {
  val |= bit_value(bit);
}

template <typename Value, typename Bit>
constexpr void bit_clear(Value& val, const Bit bit) noexcept {
  val &= ~bit_value(bit);
}

template<typename... Args>
[[nodiscard]] constexpr auto bitset_value(Args... args) noexcept {
  return (... | bit_value(args));
}

template <typename Value, typename BitSet>
constexpr void bitset_set(Value& val, const BitSet bitset) noexcept {
  val |= bitset;
}

template <typename Value, typename BitSet>
constexpr void bitset_clear(Value& val, const BitSet bitset) noexcept {
  val &= ~bitset;
}

template <typename Value, typename BitSet>
[[nodiscard]] constexpr auto bitset_mask(const Value val, const BitSet bitset) noexcept {
  return val & bitset;
}

template<typename T>
[[nodiscard]] constexpr auto memory_ptr(const std::size_t loc) {
  return reinterpret_cast<volatile T*>(loc);
}

// TODO: debug why this gets optimised away under some circumstances.
// template<typename T>
// [[nodiscard]] constexpr volatile T& memory_ref(const std::size_t loc) {
//   return *reinterpret_cast<volatile T*>(loc);
// }

#define _BV(n) (1<<(n))
#define TEST(n,b) !!((n)&_BV(b))
#define SBI(n,b) (n |= _BV(b))
#define CBI(n,b) (n &= ~_BV(b))

} // util