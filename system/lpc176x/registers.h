#pragma once

#include <cstdint>

namespace util {

template<typename T>
[[nodiscard]] constexpr auto memory_ptr(const std::size_t loc) noexcept {
  return reinterpret_cast<volatile T*>(loc);
}

template<typename T>
[[nodiscard]] constexpr auto& memory_ref(const std::size_t loc) noexcept {
  return *reinterpret_cast<volatile T*>(loc);
}

}
