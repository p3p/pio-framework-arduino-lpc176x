#pragma once

#include <cstdint>
#include <pin_control.h>

/**
 *   GPIO LPC176x::pin_type wrapper
 */

[[gnu::always_inline]] inline void gpio_set_input(const pin_t pin) {
  LPC176x::pin_type{pin}.input();
}

[[gnu::always_inline]] inline void gpio_set_output(const pin_t pin) {
  LPC176x::pin_type{pin}.output();
}

[[gnu::always_inline]] inline void gpio_direction(const pin_t pin, bool value) {
  value ? gpio_set_output(pin) : gpio_set_input(pin);
}

[[gnu::always_inline, nodiscard]] inline bool gpio_direction(const pin_t pin) {
  return LPC176x::pin_type{pin}.direction();
}

[[gnu::always_inline]] inline void gpio_set(const pin_t pin) {
  LPC176x::pin_type{pin}.set();
}

[[gnu::always_inline]] inline void gpio_clear(const pin_t pin) {
  LPC176x::pin_type{pin}.clear();
}

[[gnu::always_inline]] inline void gpio_set_port(const uint8_t port, const uint32_t pinbitset) {
  LPC176x::pin_type{port, 0}.port_set(pinbitset);
}

[[gnu::always_inline]] inline void gpio_clear_port(const uint8_t port, const uint32_t pinbitset) {
  LPC176x::pin_type{port, 0}.port_clear(pinbitset);
}

[[gnu::always_inline]] inline void gpio_set_port_mask(const uint8_t port, const uint32_t pinbitset) {
  LPC176x::pin_type{port, 0}.port_mask(pinbitset);
}

[[gnu::always_inline]] inline void gpio_set(const pin_t pin, const bool value) {
  LPC176x::pin_type{pin}.set(value);
}

[[gnu::always_inline, nodiscard]] inline bool gpio_get(const pin_t pin) {
  return LPC176x::pin_type{pin}.get();
}

[[gnu::always_inline]] inline void gpio_toggle(const pin_t pin) {
  LPC176x::pin_type{pin}.toggle();
}

[[gnu::always_inline, nodiscard]] constexpr bool gpio_interrupt_capable(const pin_t pin) {
  return LPC176x::pin_type{pin}.is_interrupt_capable();
}