#pragma once

// #define COUNT(a) (sizeof(a)/sizeof(*a))
template <typename T, std::size_t N>
constexpr std::size_t countof(T const (&)[N]) noexcept {
  return N;
}

//#define PENDING(NOW,SOON) ((long)(NOW-(SOON))<0)
template <class L, class R> static inline constexpr bool pending(const L now, const R soon) {
  return (__typeof__(now + soon))(now - soon) < 0;
}


//#define ELAPSED(NOW,SOON) (!PENDING(NOW,SOON))
template <class L, class R> static inline constexpr bool elapsed(const L now, const R soon) {
  return !pending(now, soon);
}