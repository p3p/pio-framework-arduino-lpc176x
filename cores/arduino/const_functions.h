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

[[nodiscard]] constexpr uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) noexcept{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

} // util
