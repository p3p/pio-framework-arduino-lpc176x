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
constexpr bool pending(const L now, const R soon) {
  return ( static_cast<typename std::make_signed<typeof(now + soon)>::type>(now) - soon) < 0;
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
constexpr auto bit_value(const Bit bit) noexcept {
  return (static_cast<uint32_t>(1) << bit);
}

template <typename Value, typename Bit>
constexpr bool bit_test(const Value& val, const Bit bit) noexcept {
  return val & bit_value(bit);
}

template <typename Value, typename Bit>
constexpr auto bit_set(Value& val, const Bit bit) noexcept {
  return val |= bit_value(bit);
}

template <typename Value, typename Bit>
constexpr auto bit_clear(Value& val, const Bit bit) noexcept {
  return val &= ~bit_value(bit);
}

template<typename T>
constexpr auto memory_ptr(const std::size_t loc) {
  return reinterpret_cast<volatile T(*)>(loc);
}

#define _BV(n) (1<<(n))
#define TEST(n,b) !!((n)&_BV(b))
#define SBI(n,b) (n |= _BV(b))
#define CBI(n,b) (n &= ~_BV(b))

} // util