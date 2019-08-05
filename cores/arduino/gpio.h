#pragma once

#include <cstdint>
#include <pinmapping.h>
#include <pin_control.h>

/**
 *   PIN FUNCTION LPC176x::pin_type wrapper
 */

[[gnu::always_inline, nodiscard]] constexpr uint8_t pin_port(const pin_t pin) {
  return LPC176x::pin_type{pin}.port();
}

[[gnu::always_inline, nodiscard]] constexpr  uint8_t pin_bit(const pin_t pin) {
  return LPC176x::pin_type{pin}.bit();
}

[[gnu::always_inline, nodiscard]] constexpr pin_t pin_index(const pin_t pin) {
  return LPC176x::pin_type{pin}.index();
}

[[gnu::always_inline, nodiscard]] constexpr bool pin_is_valid(const pin_t pin) {
  return LPC176x::pin_type{pin}.is_valid();
}

[[gnu::always_inline, nodiscard]] constexpr uint8_t pin_has_adc(const pin_t pin) {
  return LPC176x::pin_type{pin}.has_adc();
}

[[gnu::always_inline, nodiscard]] constexpr uint8_t pin_has_pwm(const pin_t pin) {
  return LPC176x::pin_type{pin}.has_pwm();
}

[[gnu::always_inline, nodiscard]] inline bool pin_adc_enabled(const pin_t pin) {
  return LPC176x::pin_type{pin}.adc_enabled();
}

[[gnu::always_inline, nodiscard]] inline bool pin_pwm_enabled(const pin_t pin) {
  return LPC176x::pin_type{pin}.pwm_enabled();
}

[[gnu::always_inline]] inline void pin_set_mode(const pin_t pin, const PinMode mode) {
  LPC176x::pin_type{pin}.mode(mode);
}

[[gnu::always_inline]] inline bool pin_get_mode(const pin_t pin) {
  return LPC176x::pin_type{pin}.mode();
}

[[gnu::always_inline]] inline void pin_enable_adc(const pin_t pin) {
  LPC176x::pin_type{pin}.enable_adc();
}

[[gnu::always_inline]] inline void pin_enable_pwm(const pin_t pin) {
  LPC176x::pin_type{pin}.enable_pwm();
}

[[gnu::always_inline]] inline void pin_enable_feature(const pin_t pin, uint8_t feature) {
  LPC176x::pin_type{pin}.function(feature);
}

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

[[gnu::always_inline]] inline void gpio_set_port(const uint8_t port, const uint32_t pinmap) {
  LPC176x::gpio::port_set(port, pinmap);
}

[[gnu::always_inline]] inline void gpio_clear_port(const uint8_t port, const uint32_t pinmap) {
  LPC176x::gpio::port_clear(port, pinmap);
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